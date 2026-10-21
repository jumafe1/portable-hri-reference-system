"""Optional pitch-only camera capability inside a supervised displacement trial.

Missing catalog capability permits fixed-camera fallback BEFORE acquisition.
Any failure after requesting the provider fails closed. Camera motion is staged
behind acknowledged zero base commands, followed by measured settling and a
fresh post-settle image. No command is ever sent by dry-run sessions.
"""

import math
import time

import rclpy
from capabilities2_msgs.srv import FreeCapability, GetInterfaces, GetProviders, GetRunningCapabilities, UseCapability
from hri_capability_interfaces.msg import CameraOrientation
from hri_capability_interfaces.srv import OrientCamera
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from yasmin_ros.basic_outcomes import ABORT, SUCCEED

from camera_geometry import calibration_from_info, elevation_step
from follow_controller import Decision


CAPABILITY = "hri_capability_interfaces/OrientCamera"
STATUS_TOPIC = "/hri/camera/orientation"


class FollowCamera:
    def __init__(self, session):
        self.session, self.node = session, session.node
        self.provider = session.args.camera_provider
        self.requested = self.active = self.released = False
        self.status = "camera_not_configured"
        self.error = ""
        self.feedback = self.calibration = None
        self.feedback_stamp = self.calibration_stamp = 0
        self.last_feedback_received = 0.0
        self.target = None
        self.pending = None
        self.future = None
        self.hold_started = self.command_started = self.stable_since = 0.0
        self.command_stamp = self.stable_stamp = 0
        self.settled_stamp = self.image_stamp = 0
        self.last_command = -float("inf")
        self.person = None
        self.commands_sent = 0
        self.hold_acknowledged = False
        self.interfaces = self.node.create_client(GetInterfaces, "/capabilities/get_interfaces")
        self.providers = self.node.create_client(GetProviders, "/capabilities/get_providers")
        self.acquire = self.node.create_client(UseCapability, "/capabilities/use_capability")
        self.command = self.node.create_client(OrientCamera, "/hri/camera/orient")
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.status_sub = self.node.create_subscription(CameraOrientation, STATUS_TOPIC, self.on_status, latest)
        self.info_sub = self.node.create_subscription(CameraInfo, "/hri/camera_info", self.on_info, latest)

    def _stamp(self, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp <= 0 or not 0 <= message.header.stamp.nanosec < 1_000_000_000 or not -0.1 <= age <= 0.75:
            raise ValueError("stale_or_invalid_camera_timestamp")
        return stamp

    def on_status(self, message):
        with self.session.lock:
            try:
                if message.provider != self.provider or message.header.frame_id != "camera_mount":
                    raise ValueError("unexpected_camera_provider_or_mount_frame")
                if (not self.active and not message.ready and message.status == "awaiting_camera_backend"
                        and message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0):
                    return  # Provider startup advertises no measured pose yet; readiness still times out.
                stamp = self._stamp(message)
                if not all(math.isfinite(v) for v in (message.elevation, message.azimuth)) or (
                    abs(message.elevation) > 0.265 or abs(message.azimuth) > 0.03
                ):
                    raise ValueError("camera_orientation_out_of_bounds")
                if stamp < self.feedback_stamp:
                    raise ValueError("non_increasing_camera_feedback")
                if self.active and not message.ready:
                    raise ValueError("camera_provider_not_ready")
                self.feedback, self.feedback_stamp = message, stamp
                self.last_feedback_received = time.monotonic()
            except ValueError as error:
                self.error = str(error)

    def on_info(self, message):
        with self.session.lock:
            try:
                stamp = self._stamp(message)
                args = self.session.args
                calibration = calibration_from_info(message, args.image_width, args.image_height, args.expected_frame)
                if stamp < self.calibration_stamp:
                    raise ValueError("non_increasing_camera_calibration")
                if self.active and self.calibration is not None and calibration != self.calibration:
                    raise ValueError("camera_calibration_changed")
                self.calibration, self.calibration_stamp = calibration, stamp
            except ValueError as error:
                self.error = str(error)

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("camera_service_unavailable")
        future = client.call_async(request)
        if not self.session.await_future(future, 2.0):
            raise RuntimeError("camera_service_failed")
        return future.result()

    def configure(self, blackboard):
        try:
            if CAPABILITY not in self._call(self.interfaces, GetInterfaces.Request()).interfaces:
                self.status = "fixed_camera_fallback_no_interface"
                return SUCCEED
            providers = self._call(self.providers, GetProviders.Request(interface=CAPABILITY)).providers
            if not providers:
                self.status = "fixed_camera_fallback_no_provider"
                return SUCCEED
            if self.provider not in providers:
                raise RuntimeError("requested_camera_provider_not_in_catalog")
            if self.session.cancelled.is_set():
                raise RuntimeError("cancelled")
            self.requested = True  # An unanswered request may still start a provider.
            self._call(self.acquire, UseCapability.Request(
                capability=CAPABILITY, preferred_provider=self.provider, bond_id=blackboard["bond_id"]))
            running = self._call(self.session.running_client, GetRunningCapabilities.Request())
            if not any(item.capability.capability == CAPABILITY and item.capability.provider == self.provider
                       for item in running.running_capabilities):
                raise RuntimeError("camera_provider_not_running")
            deadline = time.monotonic() + 5.0
            while rclpy.ok() and not self.session.cancelled.is_set() and time.monotonic() < deadline:
                with self.session.lock:
                    if self.error:
                        raise RuntimeError(self.error)
                    if not self.health_error() and self.feedback.ready and not self.feedback.moving:
                        self.active = True
                        self.target = self.feedback.elevation
                        self.settled_stamp = self.node.get_clock().now().nanoseconds
                        self.status = "camera_waiting_fresh_image"
                        return SUCCEED
                time.sleep(0.02)
            raise RuntimeError("cancelled_or_camera_not_ready")
        except Exception as error:
            self.error = self.status = str(error)
            blackboard["message"] = self.error
            return ABORT

    def health_error(self):
        if self.error:
            return self.error
        if (self.node.count_publishers(self.status_sub.topic_name) != 1
                or self.node.count_publishers(self.info_sub.topic_name) != 1):
            return "camera_publishers_missing_or_ambiguous"
        if self.feedback is None or self.calibration is None:
            return "camera_feedback_or_calibration_missing"
        now = self.node.get_clock().now().nanoseconds
        if any(not -0.1 <= (now - stamp) / 1e9 <= 0.75
               for stamp in (self.feedback_stamp, self.calibration_stamp)):
            return "camera_feedback_or_calibration_stale"
        if not self.feedback.ready:
            return "camera_provider_not_ready"
        if not self.command.service_is_ready():
            return "camera_command_service_lost"
        return ""

    def process_frame(self, people, stamp):
        """Called under session.lock; raw detector boxes are never rewritten."""
        if not self.active:
            return None
        self.person = None
        config = self.session.controller.config
        if any(not self.session.controller._valid_person(person) for person in people):
            return None  # The controller rejects the original invalid message.
        eligible = [person for person in people if person.confidence >= config.minimum_confidence]
        if len(eligible) != 1:
            return None
        self.person, self.image_stamp = eligible[0], stamp
        error = self.health_error()
        if error:
            self.error = error
            return None
        try:
            return self.calibration.equivalent_height_ratio(self.person)
        except ValueError as error:
            if str(error) != "clipped_detection":
                self.error = str(error)
            return None  # Clipping remains a zero-motion decision in the controller.

    def gate(self, decision, *, allow_commands=True):
        """Called under session.lock; no blocking ROS waits in the control loop."""
        if not self.active or self.session.controller.halted:
            return decision
        error = self.health_error()
        if error:
            return self.session.controller.stop(error)
        if not allow_commands or self.session.cancelled.is_set():
            return Decision(0.0, 0.0, "camera_trial_finishing", decision.locked)
        now = time.monotonic()
        if self.future is not None:
            if not self.future.done():
                if now - self.command_started > 2.0:
                    return self.session.controller.stop("camera_command_timeout")
            elif self.future.exception() is not None or not self.future.result().accepted:
                return self.session.controller.stop("camera_command_rejected")
            else:
                self.future = None
        if self.pending is not None:
            if not self.command_started:
                motion_status, motion_stamp = self.session.motion.status
                if now - self.hold_started > 2.0:
                    return self.session.controller.stop("base_stop_before_camera_not_acknowledged")
                if now - self.hold_started >= 0.4 and motion_status == "stopped" and motion_stamp >= self.hold_started:
                    self.future = self.command.call_async(OrientCamera.Request(elevation=self.pending, hold=False))
                    self.command_started = self.last_command = now
                    self.command_stamp = self.node.get_clock().now().nanoseconds
                    self.commands_sent += 1
                    self.target = self.pending
            elif now - self.command_started > 3.0:
                return self.session.controller.stop("camera_settle_timeout")
            elif (self.future is None and not self.feedback.moving
                  and self.feedback_stamp > self.command_stamp
                  and abs(self.feedback.elevation - self.target) <= 0.015):
                if not self.stable_since:
                    self.stable_since = now
                    self.stable_stamp = self.feedback_stamp
                if now - self.stable_since >= 0.25 and self.feedback_stamp > self.stable_stamp:
                    self.pending = None
                    self.command_started = self.stable_since = 0.0
                    self.settled_stamp = self.node.get_clock().now().nanoseconds
            else:
                self.stable_since = 0.0
            self.status = "camera_adjusting_base_held"
            return Decision(0.0, 0.0, self.status, decision.locked)
        if self.feedback.moving or abs(self.feedback.elevation - self.target) > 0.02:
            return self.session.controller.stop("unexpected_camera_motion")
        if self.image_stamp <= self.settled_stamp:
            self.status = "camera_waiting_fresh_image"
            return Decision(0.0, 0.0, self.status, decision.locked)
        if decision.locked and self.person is not None and now - self.last_command >= 1.0:
            target = elevation_step(self.person, self.session.args.image_height, self.feedback.elevation)
            if abs(target - self.feedback.elevation) > 0.005:
                self.pending, self.hold_started = target, now
                self.status = "camera_waiting_base_stop"
                return Decision(0.0, 0.0, self.status, decision.locked)
        self.status = "camera_ready"
        return decision

    def stop(self, blackboard):
        if not self.requested or self.released:
            return SUCCEED
        try:
            response = self._call(self.command, OrientCamera.Request(elevation=0.0, hold=True))
            self.hold_acknowledged = bool(response.accepted)
            if self.hold_acknowledged:
                return SUCCEED
        except Exception:
            pass
        blackboard["message"] += "; camera_hold_not_acknowledged"
        return ABORT

    def release(self, blackboard):
        if not self.requested or self.released:
            return SUCCEED
        try:
            self._call(self.session.release_client, FreeCapability.Request(
                capability=CAPABILITY, bond_id=blackboard["bond_id"]))
            self.released, self.active = True, False
            return SUCCEED
        except Exception:
            blackboard["message"] += "; camera_release_failed"
            return ABORT

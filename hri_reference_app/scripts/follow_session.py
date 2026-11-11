"""ROS visual-follow session. Motion endpoints exist only after explicit opt-in."""

import json
import math
import threading
import time

import rclpy
from bondpy import bondpy
from capabilities2_msgs.srv import FreeCapability, GetRunningCapabilities
from hri_capability_interfaces.msg import PersonDetection2DArray
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from yasmin_ros.basic_outcomes import ABORT, SUCCEED

from follow_controller import Config, Controller, Decision, Person
from follow_camera import FollowCamera
from follow_motion import FollowMotion
from follow_speech import FollowSpeech
from startup_speak import StartupSpeak


CAPABILITY = "hri_capability_interfaces/DetectPeople"
PROVIDER = "hri_yolo_providers/YoloDetectPeople"
PREVIEW_TOPIC = "/hri/follow_person/preview"


class FollowSession:
    def __init__(self, args, cancelled):
        self.args = args
        self.cancelled = cancelled
        self.lock = threading.Lock()
        self.controller = Controller(Config(
            image_width=args.image_width, image_height=args.image_height,
            minimum_confidence=args.minimum_confidence,
            target_height_ratio=args.target_height_ratio,
            max_forward_speed=getattr(args, "max_forward_speed", 0.10),
        ))
        self.frames = 0
        self.finished = False
        self.tracked_frames = 0
        self.samples = 0
        self.latest_stamp = 0
        self.latest_received = 0.0
        self.last_seen_stamp = 0
        self.image_recovery_started = None
        self.image_recovery_stamp = 0
        self.image_recovery_frames = 0
        self.image_recovery_zero_sent = None
        self.image_recoveries = 0
        self.person_wait_reason = ""
        self.person_wait_stamp = 0
        self.person_wait_frames = 0
        self.person_wait_zero_sent = None
        self.person_recoveries = 0
        self.perception_diagnostic = None
        self.last_print = 0.0
        self.last_reason = ""
        self.bond = None
        self.motion = None
        self.camera = None
        self.speech = None
        self.startup_speak = None
        # Diagnostics must not stall observation waiting for reliable readers.
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        # One explicit node/executor serves perception, services and heartbeat.
        # YASMIN receives this node through its public ServiceState API instead
        # of creating an additional multithreaded singleton executor.
        self.ros_closed = False
        self.node = Node("portable_hri_follow_app")
        self.executor = None
        try:
            self.executor = SingleThreadedExecutor(context=self.node.context)
            self.publisher = self.node.create_publisher(String, PREVIEW_TOPIC, latest)
            self.subscription = self.node.create_subscription(
                PersonDetection2DArray, "/hri/people", self.on_frame, latest,
            )
            self.running_client = self.node.create_client(
                GetRunningCapabilities, "/capabilities/get_running_capabilities"
            )
            self.release_client = self.node.create_client(
                FreeCapability, "/capabilities/free_capability"
            )
            if getattr(args, "startup_text", ""):
                self.startup_speak = StartupSpeak(self)
            if getattr(args, "transcribe_speech", False):
                self.speech = FollowSpeech(self)
            if getattr(args, "enable_motion", False):
                self.motion = FollowMotion(self)
                if getattr(args, "orient_camera", False):
                    self.camera = FollowCamera(self)
            self.executor.add_node(self.node)
            self.spin_thread = threading.Thread(
                target=self.executor.spin, name="follow-ros", daemon=True,
            )
            # Start only after all fields accessed by on_frame have been created.
            self.spin_thread.start()
        except Exception:
            if self.executor is not None:
                self.executor.shutdown(timeout_sec=2.0)
            self.node.destroy_node()
            raise

    def close_perception(self):
        """Close all owned ROS endpoints; caller must shut down the bond first."""
        if self.ros_closed:
            return
        shutdown_completed = self.executor.shutdown(timeout_sec=2.0)
        self.spin_thread.join(timeout=2.0)
        if not shutdown_completed or self.spin_thread.is_alive():
            raise RuntimeError("Follow executor did not stop; node retained to avoid a destruction race")
        self.node.destroy_node()
        self.ros_closed = True

    def start_bond(self, blackboard):
        if self.cancelled.is_set():
            blackboard["message"] = "cancelled"
            return ABORT
        self.bond = bondpy.Bond(self.node, "/capabilities/bonds", blackboard["bond_id"])
        self.bond.start()
        if not self.bond.wait_until_formed(Duration(seconds=5)) or self.bond.is_broken():
            blackboard["message"] = "capability_bond_not_formed"
            return ABORT
        return SUCCEED

    def on_frame(self, frame):
        stamp = frame.header.stamp.sec * 1_000_000_000 + frame.header.stamp.nanosec
        with self.lock:
            if self.finished or self.controller.halted:
                return
            # Serialize controller time with observe/tick. A callback waiting
            # for this lock must not submit a timestamp older than a timer tick.
            now = time.monotonic()
            age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
            self.frames += 1
            # Retain the actual incoming frame, including rejected headers. The
            # accepted source stamp alone cannot explain a failed new frame.
            people = [Person(
                confidence=p.confidence, tracking_id=p.tracking_id,
                center_x=p.center_x, center_y=p.center_y,
                width=p.width, height=p.height,
            ) for p in frame.people]
            valid = [self.controller._valid_person(p) for p in people]
            def number(value):
                return value if math.isfinite(value) else str(value)
            self.perception_diagnostic = {
                "frame_id": frame.header.frame_id, "provider": frame.provider,
                "stamp_ns": stamp, "age_at_receipt_s": age,
                "receipt_gap_s": now - self.latest_received if self.latest_received else None,
                "people_count": len(people),
                "eligible_count": sum(ok and p.confidence >= self.controller.config.minimum_confidence
                                      for p, ok in zip(people, valid)),
                "invalid_count": valid.count(False),
                "image_width": self.controller.config.image_width,
                "image_height": self.controller.config.image_height,
                "minimum_confidence": self.controller.config.minimum_confidence,
                "people": [dict(
                    confidence=number(p.confidence),
                    center_x=number(p.center_x), center_y=number(p.center_y),
                    width=number(p.width), height=number(p.height), valid=ok,
                ) for p, ok in list(zip(people, valid))[:10]],
                "people_truncated": len(people) > 10,
            }
            failure = ""
            if frame.provider != getattr(self.args, "people_provider", PROVIDER):
                failure = "unexpected_people_provider"
            elif frame.header.frame_id != self.args.expected_frame:
                failure = "unexpected_camera_frame"
            elif stamp <= 0 or not 0 <= frame.header.stamp.nanosec < 1_000_000_000:
                failure = "invalid_image_timestamp"
            elif age < -0.1:
                failure = "stale_or_future_image"
            elif stamp <= max(self.latest_stamp, self.last_seen_stamp):
                failure = "non_increasing_image_timestamp"
            if failure:
                self.controller.stop(failure)
                return
            self.last_seen_stamp = stamp
            # Check the previous observation BEFORE overwriting it: a callback
            # arriving between control ticks must not hide a freshness gap.
            self._check_image_freshness(now)
            if self.controller.halted:
                return
            if age > self.controller.config.max_age:
                self._pause_stale_image(now)
                return
            if self.image_recovery_started is not None and stamp <= self.image_recovery_stamp:
                # Queued images predating the pause cannot re-arm movement.
                return
            self.latest_stamp = stamp
            self.latest_received = now
            camera = getattr(self, "camera", None)
            ratio = camera.process_frame(people, stamp) if camera else None
            decision = self.controller.update(people, now, height_ratio_override=ratio)
            if decision.reason in {"person_missing", "ambiguous_people"}:
                if not self.person_wait_reason:
                    self.person_wait_stamp = self.node.get_clock().now().nanoseconds
                    self.person_wait_zero_sent = None
                self.person_wait_reason = decision.reason
                self.person_wait_frames = 0
            elif self.person_wait_reason:
                eligible = sum(ok and p.confidence >= self.controller.config.minimum_confidence
                               for p, ok in zip(people, valid))
                if (all(valid) and eligible == 1 and not self.controller.halted
                        and stamp > self.person_wait_stamp):
                    self.person_wait_frames = min(3, self.person_wait_frames + 1)
                else:
                    self.person_wait_frames = 0
            if self.image_recovery_started is not None:
                # A fresh empty/ambiguous frame proves that the image stream is
                # back. The independent person gate still holds both axes zero.
                if all(valid) and not self.controller.halted:
                    self.image_recovery_frames = min(3, self.image_recovery_frames + 1)
                else:
                    self.image_recovery_frames = 0
            if (decision.locked and not self.controller.halted
                    and decision.reason not in {"person_missing", "ambiguous_people", "acquiring_target"}):
                self.tracked_frames += 1

    def await_future(self, future, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done() and future.exception() is None

    def wait_ready(self, blackboard):
        if not self.running_client.wait_for_service(timeout_sec=1.0):
            blackboard["message"] = "provider_identity_unavailable"
            return ABORT
        future = self.running_client.call_async(GetRunningCapabilities.Request())
        if not self.await_future(future, 2.0) or not any(
            item.capability.capability == CAPABILITY
            and item.capability.provider == getattr(self.args, "people_provider", PROVIDER)
            for item in future.result().running_capabilities
        ):
            blackboard["message"] = "requested_provider_not_running"
            return ABORT
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not self.cancelled.is_set() and time.monotonic() < deadline:
            count = self.node.count_publishers("/hri/people")
            if count > 1:
                blackboard["message"] = "ambiguous_people_publishers"
                return ABORT
            if count == 1:
                return SUCCEED
            time.sleep(0.05)
        blackboard["message"] = "cancelled_or_provider_unavailable"
        return ABORT

    def publish_preview(self, decision):
        now = time.monotonic()
        payload = {
            "mode": "follow_person", "robot": self.args.robot,
            "dry_run": self.motion is None, "motion_enabled": self.motion is not None,
            "vx_m_s": decision.vx, "vy_m_s": 0.0, "wz_rad_s": decision.wz,
            "reason": decision.reason, "locked": decision.locked,
            "source_frame": self.args.expected_frame,
            "source_stamp_ns": self.latest_stamp,
            "receipt_age_s": now - self.latest_received if self.latest_received else None,
            "source_age_s": ((self.node.get_clock().now().nanoseconds - self.latest_stamp) / 1e9
                             if self.latest_stamp else None),
            "perception": getattr(self, "perception_diagnostic", None),
            "image_recovery_active": self.image_recovery_started is not None,
            "image_recovery_frames": self.image_recovery_frames,
            "image_recovery_elapsed_s": (now - self.image_recovery_started
                                         if self.image_recovery_started is not None else None),
            "image_recoveries": self.image_recoveries,
            "person_wait_active": bool(self.person_wait_reason),
            "person_wait_reason": self.person_wait_reason,
            "person_wait_frames": self.person_wait_frames,
            "person_recoveries": self.person_recoveries,
            "distance_m": None,
            "speech_status": self.speech.status if getattr(self, "speech", None) else "disabled",
            "transcripts_received": self.speech.transcripts_received if getattr(self, "speech", None) else 0,
            "camera_orientation_requested": getattr(self.args, "orient_camera", False),
            "camera_orientation_active": bool(self.camera and self.camera.active),
            "camera_status": self.camera.status if self.camera else (
                "disabled_in_preview" if getattr(self.args, "orient_camera", False) else "disabled"),
            "camera_elevation_rad": self.camera.feedback.elevation if self.camera and self.camera.feedback else None,
        }
        data = json.dumps(payload, allow_nan=False)
        self.publisher.publish(String(data=data))
        self.samples += 1
        if decision.reason != self.last_reason or now - self.last_print >= 1.0:
            self.node.get_logger().info(data)
            self.last_print, self.last_reason = now, decision.reason

    def _pause_stale_image(self, now):
        if self.controller.halted:
            return
        if self.image_recovery_started is None:
            self.image_recovery_started = now
            self.image_recovery_stamp = self.node.get_clock().now().nanoseconds
            self.image_recovery_zero_sent = None
        # Additional gaps reset acquisition; waiting has no trial deadline.
        self.image_recovery_frames = 0
        self.person_wait_frames = 0
        self.controller.pause_stale()

    def _base_stop_acknowledged(self, zero_sent, now):
        if not self.motion:
            return True
        status, stamp = self.motion.status
        return (zero_sent is not None and now - zero_sent >= 0.4
                and status == "stopped" and stamp > zero_sent
                and 0 <= now - stamp < 0.5)

    def _check_image_freshness(self, now):
        if self.latest_stamp:
            age = (self.node.get_clock().now().nanoseconds - self.latest_stamp) / 1e9
            if age < -0.1:
                self.controller.stop("stale_or_future_image")
            elif (age > self.controller.config.max_age
                  or now - self.latest_received > self.controller.config.max_age):
                self._pause_stale_image(now)

    def _checked_decision(self, *, allow_camera_commands=True):
        """Called with the session lock held; safety checks have no trial expiry."""
        if self.bond.is_broken():
            self.controller.stop("capability_bond_broken")
        elif self.node.count_publishers("/hri/people") != 1:
            self.controller.stop("people_publisher_lost_or_ambiguous")
        now = time.monotonic()
        self._check_image_freshness(now)
        decision = self.controller.tick(now)
        camera = getattr(self, "camera", None)
        if ((self.image_recovery_started is not None or self.person_wait_reason)
                and not self.controller.halted):
            # Keep optional-camera health checks, but issue no new head commands
            # from stale perception while the base is held.
            if camera and camera.active:
                error = camera.health_error()
                if error:
                    return self.controller.stop(error)
        if self.image_recovery_started is not None and not self.controller.halted:
            ready = self.image_recovery_frames >= 3
            reason = "waiting_fresh_images"
            stopped = self._base_stop_acknowledged(self.image_recovery_zero_sent, now)
            if ready and not stopped:
                reason = "waiting_image_recovery_stop"
            ready = ready and stopped
            if not ready:
                return Decision(0.0, 0.0, reason, decision.locked)
            self.image_recovery_started = None
            self.image_recoveries += 1
        if self.person_wait_reason and not self.controller.halted:
            if self.person_wait_frames < 3:
                reason = "reacquiring_person" if self.person_wait_frames else self.person_wait_reason
                return Decision(0.0, 0.0, reason, decision.locked)
            if not self._base_stop_acknowledged(self.person_wait_zero_sent, now):
                return Decision(0.0, 0.0, "waiting_person_stop", decision.locked)
            self.person_wait_reason = ""
            self.person_recoveries += 1
        return camera.gate(decision, allow_commands=allow_camera_commands) if camera else decision

    def observe(self, blackboard):
        # No trial or image-arrival deadline: zero commands keep the base held
        # until fresh perception returns or the operator cancels. Provider and
        # timestamp/geometry faults remain fatal and use the existing cleanup.
        speech = getattr(self, "speech", None)
        while rclpy.ok() and not self.cancelled.is_set():
            with self.lock:
                decision = self._checked_decision()
                if self.motion:
                    error = self.motion.send(decision)
                    if error:
                        decision = self.controller.stop(error)
                    else:
                        if (self.image_recovery_started is not None
                                and self.image_recovery_zero_sent is None):
                            self.image_recovery_zero_sent = time.monotonic()
                        if self.person_wait_reason and self.person_wait_zero_sent is None:
                            self.person_wait_zero_sent = time.monotonic()
                self.publish_preview(decision)
                if self.controller.halted:
                    self.finished = True
                    blackboard["message"] = decision.reason
                    if speech:
                        speech.freeze("stopped_visual_fault")
                    return ABORT
            if speech:
                speech.tick()
            self.cancelled.wait(0.05)
        if speech:
            speech.freeze("cancelled")
        with self.lock:
            # Cancellation never issues another camera/base movement command.
            blackboard["message"] = "cancelled_or_ros_shutdown"
            self.finished = True
            self.publish_preview(self.controller.stop(blackboard["message"]))
            return ABORT

    def finish_preview(self, reason):
        with self.lock:
            self.finished = True
            self.publish_preview(self.controller.stop(reason))

    def release_fallback(self, blackboard):
        if blackboard["released"] or "bond_id" not in blackboard or not rclpy.ok():
            return
        if self.release_client.wait_for_service(timeout_sec=1.0):
            request = FreeCapability.Request(capability=CAPABILITY, bond_id=blackboard["bond_id"])
            future = self.release_client.call_async(request)
            if self.await_future(future, 3.0):
                blackboard["released"] = True

"""ROS visual-follow session. Motion endpoints exist only after explicit opt-in."""

import json
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

from follow_controller import Config, Controller, Person
from follow_motion import FollowMotion


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
            max_forward_speed=getattr(args, "max_forward_speed", 0.03),
        ))
        self.frames = 0
        self.finished = False
        self.tracked_frames = 0
        self.samples = 0
        self.latest_stamp = 0
        self.latest_received = 0.0
        self.last_print = 0.0
        self.last_reason = ""
        self.bond = None
        self.motion = None
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
            if getattr(args, "enable_motion", False):
                self.motion = FollowMotion(self)
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
            if self.finished:
                return
            # Serialize controller time with observe/tick. A callback waiting
            # for this lock must not submit a timestamp older than a timer tick.
            now = time.monotonic()
            age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
            self.frames += 1
            failure = ""
            if frame.provider != PROVIDER:
                failure = "unexpected_people_provider"
            elif frame.header.frame_id != self.args.expected_frame:
                failure = "unexpected_camera_frame"
            elif stamp <= 0 or not 0 <= frame.header.stamp.nanosec < 1_000_000_000:
                failure = "invalid_image_timestamp"
            elif age > 0.75 or age < -0.1:
                failure = "stale_or_future_image"
            elif stamp <= self.latest_stamp:
                failure = "non_increasing_image_timestamp"
            if failure:
                self.controller.stop(failure)
                return
            self.latest_stamp = stamp
            self.latest_received = now
            people = [Person(
                confidence=p.confidence, tracking_id=p.tracking_id,
                center_x=p.center_x, center_y=p.center_y,
                width=p.width, height=p.height,
            ) for p in frame.people]
            decision = self.controller.update(people, now)
            if decision.locked and not self.controller.halted:
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
            and item.capability.provider == PROVIDER
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
            "distance_m": None,
        }
        data = json.dumps(payload, allow_nan=False)
        self.publisher.publish(String(data=data))
        self.samples += 1
        if decision.reason != self.last_reason or now - self.last_print >= 1.0:
            self.node.get_logger().info(data)
            self.last_print, self.last_reason = now, decision.reason

    def _checked_decision(self):
        """Called with the session lock held, including at the final deadline."""
        if self.bond.is_broken():
            self.controller.stop("capability_bond_broken")
        elif self.node.count_publishers("/hri/people") != 1:
            self.controller.stop("people_publisher_lost_or_ambiguous")
        if self.latest_stamp:
            age = (self.node.get_clock().now().nanoseconds - self.latest_stamp) / 1e9
            if age > 0.75 or age < -0.1:
                self.controller.stop("stale_or_future_image")
        return self.controller.tick(time.monotonic())

    def observe(self, blackboard):
        duration = min(self.args.timeout, self.args.duration) if self.motion else self.args.timeout
        deadline = time.monotonic() + duration
        while rclpy.ok() and not self.cancelled.is_set() and time.monotonic() < deadline:
            with self.lock:
                decision = self._checked_decision()
                if self.motion:
                    error = self.motion.send(decision)
                    if error:
                        decision = self.controller.stop(error)
                self.publish_preview(decision)
                if self.controller.halted:
                    self.finished = True
                    blackboard["message"] = decision.reason
                    return ABORT
            self.cancelled.wait(0.05)
        with self.lock:
            # A final callback must not turn a newly invalid target into success.
            decision = self._checked_decision()
            result = ABORT
            if self.cancelled.is_set() or not rclpy.ok():
                blackboard["message"] = "cancelled_or_ros_shutdown"
            elif self.controller.halted:
                blackboard["message"] = decision.reason
            elif not self.tracked_frames:
                blackboard["message"] = "no_target_acquired"
            else:
                blackboard["message"] = "follow_trial_completed" if self.motion else "preview_completed"
                result = SUCCEED
            self.finished = True
            self.publish_preview(self.controller.stop(blackboard["message"]))
            return result

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

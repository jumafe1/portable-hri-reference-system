"""Explicitly opted-in, bounded motion transport for the visual follow state.

Provider acknowledgement confirms a software command, never physical stopping.
Graph checks are diagnostic guards, not an instantaneous distributed lock.
"""

import math
import time

import rclpy
from capabilities2_msgs.srv import FreeCapability, GetRunningCapabilities, UseCapability
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from yasmin_ros.basic_outcomes import ABORT, SUCCEED


CAPABILITY = "hri_capability_interfaces/MoveVelocity"
PROVIDER = "hri_naoqi_providers/NaoqiMoveVelocity"
COMMAND_TOPIC = "/hri/cmd_vel"
STATUS_TOPIC = "/hri/move_velocity/status"


class FollowMotion:
    def __init__(self, session):
        self.session = session
        self.node = session.node
        self.requested = False
        self.acquired = False
        self.released = False
        self.stop_acknowledged = False
        self.commands_sent = 0
        self.status = ("", 0.0)
        self.publisher = self.node.create_publisher(Twist, COMMAND_TOPIC, 1)
        self.subscription = self.node.create_subscription(String, STATUS_TOPIC, self.on_status, 1)

    def on_status(self, message):
        self.status = (message.data, time.monotonic())

    def request(self, blackboard):
        # Even an unanswered request may have started the provider: free it on exit.
        self.requested = True
        return UseCapability.Request(capability=CAPABILITY, preferred_provider=PROVIDER,
                                     bond_id=blackboard["bond_id"])

    def graph_error(self):
        if self.node.count_publishers(COMMAND_TOPIC) != 1:
            return "multiple_or_missing_velocity_publishers"
        if self.publisher.get_subscription_count() != 1:
            return "multiple_or_missing_velocity_consumers"
        if self.node.count_publishers(STATUS_TOPIC) != 1:
            return "multiple_or_missing_velocity_status_publishers"
        return ""

    def wait_ready(self, blackboard):
        future = self.session.running_client.call_async(GetRunningCapabilities.Request())
        if not self.session.await_future(future, 2.0) or not any(
            item.capability.capability == CAPABILITY and item.capability.provider == PROVIDER
            for item in future.result().running_capabilities
        ):
            blackboard["message"] = "motion_provider_not_running"
            return ABORT
        deadline, stable_since = time.monotonic() + 5.0, None
        while rclpy.ok() and not self.session.cancelled.is_set() and time.monotonic() < deadline:
            now = time.monotonic()
            status, stamp = self.status
            error = self.graph_error()
            if status in {"multiple_velocity_publishers", "velocity_source_changed", "invalid_command"}:
                blackboard["message"] = status
                return ABORT
            if error:
                stable_since = None
            else:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= 0.5 and now - stamp < 0.5 and status in {"ready", "stopped"}:
                    self.acquired = True
                    return SUCCEED
            time.sleep(0.02)
        blackboard["message"] = "cancelled_or_motion_provider_not_ready"
        return ABORT

    def send(self, decision):
        if not self.acquired or self.released or self.session.cancelled.is_set():
            return "motion_not_armed_or_cancelled"
        error = self.graph_error()
        status, stamp = self.status
        if error:
            return error
        if time.monotonic() - stamp >= 0.5 or status not in {"ready", "moving", "stopped"}:
            return "motion_status_lost_or_faulted"
        if not all(math.isfinite(v) for v in (decision.vx, decision.wz)) or not (
            0.0 <= decision.vx <= self.session.controller.config.max_forward_speed <= 0.10
            and abs(decision.wz) <= 0.10
        ):
            return "invalid_follow_velocity"
        command = Twist()
        command.linear.x, command.angular.z = decision.vx, decision.wz
        self.publisher.publish(command)
        self.commands_sent += 1
        return ""

    def stop(self, blackboard):
        if not self.acquired or self.released:
            return SUCCEED
        started = time.monotonic()
        self.stop_acknowledged = False
        # Cleanup must run even after cancellation; never send a nonzero command here.
        while rclpy.ok() and time.monotonic() - started < 0.4:
            self.publisher.publish(Twist())
            time.sleep(0.05)
        status, stamp = self.status
        self.stop_acknowledged = (rclpy.ok() and status == "stopped" and stamp >= started
                                  and time.monotonic() - stamp < 0.5 and not self.graph_error())
        if not self.stop_acknowledged:
            blackboard["message"] += "; motion_stop_not_acknowledged"
            return ABORT
        return SUCCEED

    def release(self, blackboard):
        if self.released or not self.requested:
            return SUCCEED
        if "bond_id" not in blackboard or not rclpy.ok():
            return ABORT
        client = self.session.release_client
        if client.wait_for_service(timeout_sec=1.0):
            future = client.call_async(FreeCapability.Request(capability=CAPABILITY,
                                                              bond_id=blackboard["bond_id"]))
            if self.session.await_future(future, 3.0):
                self.released = True
                self.acquired = False
                return SUCCEED
        blackboard["message"] += "; motion_release_failed"
        return ABORT

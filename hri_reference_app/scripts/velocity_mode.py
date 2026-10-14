"""Bounded velocity-stream mode; success is delivery, not physical displacement."""

import argparse
import json
import math
import signal
import sys
import threading
import time

import rclpy
from bondpy import bondpy
from capabilities2_msgs.srv import EstablishBond, FreeCapability, GetRunningCapabilities, UseCapability
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from yasmin import Blackboard, CbState, StateMachine
from yasmin_ros import ServiceState, set_ros_loggers
from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED, TIMEOUT
from yasmin_ros.yasmin_node import YasminNode

from capability_lifecycle import EstablishBondState, FreeCapabilityState, UseCapabilityState


CAPABILITY = "hri_capability_interfaces/MoveVelocity"
DEFAULT_PROVIDER = "hri_naoqi_providers/NaoqiMoveVelocity"
APP_SUCCEEDED = "application_succeeded"
APP_FAILED = "application_failed"
PERIOD_SECONDS = 0.05
STATUS_MAX_AGE_SECONDS = 0.5


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Stream bounded portable planar velocities.")
    parser.add_argument("--robot", choices=("nao", "pepper"), required=True)
    parser.add_argument("--vx", type=float, default=0.0, help="Forward m/s.")
    parser.add_argument("--vy", type=float, default=0.0, help="Left m/s.")
    parser.add_argument("--wz", type=float, default=0.0, help="Yaw rad/s.")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--confirm", required=True)
    parser.add_argument("--provider", default=DEFAULT_PROVIDER)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:] if argv is None else argv)
    values = (args.vx, args.vy, args.wz)
    if args.confirm != "MOVER":
        parser.error("Movement requires --confirm MOVER.")
    if not all(math.isfinite(value) for value in (*values, args.duration)):
        parser.error("Velocities and duration must be finite.")
    if not 0.1 <= args.duration <= 5.0:
        parser.error("Duration must be in [0.1, 5.0] seconds.")
    if any(abs(value) > limit for value, limit in zip(values, (0.10, 0.05, 0.20))):
        parser.error("Velocity exceeds vx=0.10 m/s, vy=0.05 m/s or wz=0.20 rad/s.")
    if all(abs(value) <= 1e-6 for value in values):
        parser.error("At least one velocity component must be non-zero.")
    return args


class VelocitySession:
    """Keep ROS alive during cooperative cancellation and best-effort cleanup."""

    def __init__(self, args, cancelled):
        self.args = args
        self.cancelled = cancelled
        self.node = YasminNode.get_instance()
        self.publisher = self.node.create_publisher(Twist, "/hri/cmd_vel", 1)
        self.status = ("", 0.0)
        self.subscription = self.node.create_subscription(
            String, "/hri/move_velocity/status", self.on_status, 1
        )
        self.release_client = self.node.create_client(FreeCapability, "/capabilities/free_capability")
        self.running_client = self.node.create_client(GetRunningCapabilities, "/capabilities/get_running_capabilities")
        self.acquired = False
        self.bond = None

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

    def on_status(self, message):
        self.status = (message.data, time.monotonic())

    def status_is(self, allowed):
        status, received = self.status
        return status in allowed and time.monotonic() - received < STATUS_MAX_AGE_SECONDS

    def wait_ready(self, blackboard):
        if not self.running_client.wait_for_service(timeout_sec=1.0):
            blackboard["message"] = "provider_identity_unavailable"
            return ABORT
        future = self.running_client.call_async(GetRunningCapabilities.Request())
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done() or future.exception() is not None or not any(
            item.capability.capability == CAPABILITY
            and item.capability.provider == self.args.provider
            for item in future.result().running_capabilities
        ):
            blackboard["message"] = "requested_provider_not_running"
            return ABORT
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not self.cancelled.is_set() and time.monotonic() < deadline:
            if self.node.count_publishers("/hri/cmd_vel") != 1:
                blackboard["message"] = "multiple_velocity_publishers"
                return ABORT
            # This mode assumes one app/server. Do not command an ambiguous
            # shared endpoint, even with zero, before validating ownership.
            if self.publisher.get_subscription_count() > 1 or self.node.count_publishers("/hri/move_velocity/status") > 1:
                blackboard["message"] = "ambiguous_velocity_provider_endpoints"
                return ABORT
            if self.publisher.get_subscription_count() == 1 and self.node.count_publishers("/hri/move_velocity/status") == 1:
                if self.status_is({"ready", "stopped"}):
                    self.acquired = True
                    self.publisher.publish(Twist())
                    return SUCCEED
            time.sleep(PERIOD_SECONDS)
        blackboard["message"] = "cancelled" if self.cancelled.is_set() else "provider_not_ready"
        return ABORT

    def request_stop(self, blackboard):
        """Send zero for multiple cycles; acknowledgement is NOT physical telemetry."""
        if not self.acquired:
            return
        start = time.monotonic()
        blackboard["stop_acknowledged"] = False
        while rclpy.ok() and time.monotonic() - start < 0.4:
            self.publisher.publish(Twist())
            time.sleep(PERIOD_SECONDS)
        status, received = self.status
        blackboard["stop_acknowledged"] = (
            status == "stopped" and received >= start
            and time.monotonic() - received < STATUS_MAX_AGE_SECONDS
        )

    def stream(self, blackboard):
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = self.args.vx, self.args.vy, self.args.wz
        deadline = time.monotonic() + self.args.duration
        result = ABORT
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                if self.cancelled.is_set():
                    blackboard["message"] = "cancelled"
                    return ABORT
                if self.node.count_publishers("/hri/cmd_vel") != 1:
                    blackboard["message"] = "multiple_velocity_publishers"
                    return ABORT
                if (
                    self.publisher.get_subscription_count() != 1
                    or self.node.count_publishers("/hri/move_velocity/status") != 1
                    or not self.status_is({"ready", "moving", "stopped"})
                ):
                    blackboard["message"] = "provider_status_lost_or_faulted"
                    return ABORT
                self.publisher.publish(command)
                blackboard["commands_sent"] += 1
                self.cancelled.wait(PERIOD_SECONDS)
            if rclpy.ok() and not self.cancelled.is_set():
                blackboard["message"] = "velocity_stream_completed"
                result = SUCCEED
            else:
                blackboard["message"] = "cancelled_or_ros_shutdown"
        finally:
            self.request_stop(blackboard)
        if not blackboard["stop_acknowledged"]:
            blackboard["message"] = "stop_not_acknowledged"
            return ABORT
        return result

    def release_fallback(self, blackboard):
        if blackboard["released"] or "bond_id" not in blackboard or not rclpy.ok():
            return
        if not self.release_client.wait_for_service(timeout_sec=1.0):
            return
        request = FreeCapability.Request(capability=CAPABILITY, bond_id=blackboard["bond_id"])
        future = self.release_client.call_async(request)
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if future.done() and future.exception() is None:
            blackboard["released"] = True


def build_state_machine(session):
    machine = StateMachine(outcomes=[APP_SUCCEEDED, APP_FAILED], handle_sigint=False)
    bond = ServiceState(
        EstablishBond, "/capabilities/establish_bond", lambda _: EstablishBond.Request(),
        response_handler=EstablishBondState.handle_response,
        wait_timeout=5.0, response_timeout=5.0, maximum_retry=0,
    )
    acquire = ServiceState(
        UseCapability, "/capabilities/use_capability", UseCapabilityState.create_request,
        wait_timeout=5.0, response_timeout=5.0, maximum_retry=0,
    )
    machine.add_state("ESTABLISH_BOND", bond, transitions={
        SUCCEED: "START_BOND", ABORT: APP_FAILED, CANCEL: APP_FAILED, TIMEOUT: APP_FAILED,
    })
    machine.add_state("START_BOND", CbState({SUCCEED, ABORT}, session.start_bond), transitions={
        SUCCEED: "USE_MOVE_VELOCITY", ABORT: APP_FAILED,
    })
    machine.add_state("USE_MOVE_VELOCITY", acquire, transitions={
        SUCCEED: "WAIT_READY", ABORT: "FREE_AFTER_FAILURE",
        CANCEL: "FREE_AFTER_FAILURE", TIMEOUT: "FREE_AFTER_FAILURE",
    })
    machine.add_state("WAIT_READY", CbState({SUCCEED, ABORT}, session.wait_ready), transitions={
        SUCCEED: "STREAM_VELOCITY", ABORT: "FREE_AFTER_FAILURE",
    })
    machine.add_state("STREAM_VELOCITY", CbState({SUCCEED, ABORT}, session.stream), transitions={
        SUCCEED: "FREE_AFTER_SUCCESS", ABORT: "FREE_AFTER_FAILURE",
    })
    for name, success in (("FREE_AFTER_SUCCESS", APP_SUCCEEDED), ("FREE_AFTER_FAILURE", APP_FAILED)):
        release = ServiceState(
            FreeCapability, "/capabilities/free_capability", FreeCapabilityState.create_request,
            response_handler=FreeCapabilityState.handle_response,
            wait_timeout=1.0, response_timeout=2.0, maximum_retry=0,
        )
        machine.add_state(name, release, transitions={
            SUCCEED: success, ABORT: APP_FAILED, CANCEL: APP_FAILED, TIMEOUT: APP_FAILED,
        })
    return machine


def run(args):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    set_ros_loggers()
    cancelled = threading.Event()
    previous_handlers = {
        sig: signal.signal(sig, lambda _sig, _frame: cancelled.set())
        for sig in (signal.SIGINT, signal.SIGTERM)
    }
    blackboard = Blackboard()
    for key, value in {
        "capability": CAPABILITY, "preferred_provider": args.provider,
        "released": False, "message": "velocity_stream_not_started",
        "commands_sent": 0, "stop_acknowledged": False,
    }.items():
        blackboard[key] = value
    session = None
    outcome = APP_FAILED
    try:
        session = VelocitySession(args, cancelled)
        outcome = build_state_machine(session)(blackboard)
    except Exception as error:
        blackboard["message"] = f"application_exception: {error}"
    finally:
        if session is not None:
            try:
                if not blackboard["released"]:
                    session.request_stop(blackboard)
                    session.release_fallback(blackboard)
            except Exception as error:
                outcome = APP_FAILED
                blackboard["message"] += f"; cleanup_exception: {error}"
            finally:
                if session.bond is not None:
                    session.bond.shutdown()
        YasminNode.destroy_instance()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
    if cancelled.is_set() or not blackboard["released"]:
        outcome = APP_FAILED
    print(json.dumps({
        "outcome": outcome, "robot": args.robot, "capability": CAPABILITY,
        "requested_provider": args.provider,
        "request": {"vx_m_s": args.vx, "vy_m_s": args.vy, "wz_rad_s": args.wz, "duration_s": args.duration},
        "commands_sent": blackboard["commands_sent"], "message": blackboard["message"],
        "released": blackboard["released"], "stop_acknowledged": blackboard["stop_acknowledged"],
        "physical_success_verified": False,
    }, ensure_ascii=False))
    return 0 if outcome == APP_SUCCEEDED else 1

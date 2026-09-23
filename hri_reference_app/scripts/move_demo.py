#!/usr/bin/env python3
"""Run one portable relative movement as a YASMIN capability state."""

import argparse
import json
import math
import sys
import time

import rclpy
from hri_capability_interfaces.srv import MoveRelative
from rclpy.utilities import remove_ros_args

from capability_lifecycle import (
    EstablishBondState,
    FreeCapabilityState,
    UseCapabilityState,
)

try:
    import yasmin
    from yasmin import Blackboard, CbState, StateMachine
    from yasmin_ros import ServiceState, set_ros_loggers
    from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED, TIMEOUT
    from yasmin_ros.yasmin_node import YasminNode
except ModuleNotFoundError as error:
    if error.name and error.name.split(".")[0] in {"yasmin", "yasmin_ros"}:
        raise SystemExit(
            "YASMIN is not available. Install ros-jazzy-yasmin and "
            "ros-jazzy-yasmin-ros, then source /opt/ros/jazzy/setup.bash "
            "before the workspace overlay."
        ) from None
    raise


CAPABILITY = "hri_capability_interfaces/MoveRelative"
DEFAULT_PROVIDER = "hri_naoqi_providers/NaoqiMoveRelative"
APP_SUCCEEDED = "application_succeeded"
APP_FAILED = "application_failed"
MAXIMUM_X_METRES = 0.5
MAXIMUM_Y_METRES = 0.3
MAXIMUM_THETA_RADIANS = math.pi / 2.0
MOTION_RESPONSE_TIMEOUT_SECONDS = 35.0
ODOMETRY_STARTUP_SECONDS = 3.0
ODOMETRY_RETRY_DELAY_SECONDS = 0.2
ODOMETRY_PENDING = "odometry_pending"


class MoveRelativeState(ServiceState):
    """Request one bounded move and retain the provider's measured result."""

    def __init__(self):
        # The provider waits up to 30 s; one 35 s wait avoids repeated commands.
        super().__init__(
            MoveRelative,
            "/hri/move_relative",
            self.create_request,
            outcomes={ODOMETRY_PENDING},
            response_handler=self.handle_response,
            wait_timeout=10.0,
            response_timeout=MOTION_RESPONSE_TIMEOUT_SECONDS,
            maximum_retry=0,
        )

    @staticmethod
    def create_request(blackboard):
        return MoveRelative.Request(
            x_m=blackboard["x_m"],
            y_m=blackboard["y_m"],
            theta_rad=blackboard["theta_rad"],
        )

    @staticmethod
    def handle_response(blackboard, response):
        blackboard["provider"] = response.provider
        blackboard["message"] = response.message
        blackboard["achieved"] = {
            "x_m": response.achieved_x_m,
            "y_m": response.achieved_y_m,
            "theta_rad": response.achieved_theta_rad,
        }
        if response.success:
            return SUCCEED
        # This provider rejects before calling move_to when it has no odometry.
        if response.message == "odometry_unavailable_before_motion":
            blackboard["odometry_retries"] += 1
            if "odometry_deadline" not in blackboard:
                blackboard["odometry_deadline"] = (
                    time.monotonic() + ODOMETRY_STARTUP_SECONDS
                )
            return ODOMETRY_PENDING
        return ABORT


def wait_for_odometry(blackboard):
    if time.monotonic() >= blackboard["odometry_deadline"]:
        return ABORT
    time.sleep(ODOMETRY_RETRY_DELAY_SECONDS)
    return SUCCEED


def build_state_machine():
    state_machine = StateMachine(
        outcomes=[APP_SUCCEEDED, APP_FAILED], handle_sigint=True
    )
    state_machine.add_state(
        "ESTABLISH_BOND",
        EstablishBondState(),
        transitions={
            SUCCEED: "USE_MOVE_RELATIVE",
            ABORT: APP_FAILED,
            TIMEOUT: APP_FAILED,
            CANCEL: APP_FAILED,
        },
    )
    state_machine.add_state(
        "USE_MOVE_RELATIVE",
        UseCapabilityState(),
        transitions={
            SUCCEED: "MOVE_RELATIVE",
            ABORT: "FREE_AFTER_FAILURE",
            TIMEOUT: "FREE_AFTER_FAILURE",
            CANCEL: "FREE_AFTER_FAILURE",
        },
    )
    state_machine.add_state(
        "MOVE_RELATIVE",
        MoveRelativeState(),
        transitions={
            SUCCEED: "FREE_AFTER_SUCCESS",
            ODOMETRY_PENDING: "WAIT_FOR_ODOMETRY",
            ABORT: "FREE_AFTER_FAILURE",
            TIMEOUT: "FREE_AFTER_FAILURE",
            CANCEL: "FREE_AFTER_FAILURE",
        },
    )
    state_machine.add_state(
        "WAIT_FOR_ODOMETRY",
        CbState({SUCCEED, ABORT}, wait_for_odometry),
        transitions={SUCCEED: "MOVE_RELATIVE", ABORT: "FREE_AFTER_FAILURE"},
    )
    state_machine.add_state(
        "FREE_AFTER_SUCCESS",
        FreeCapabilityState(),
        transitions={
            SUCCEED: APP_SUCCEEDED,
            ABORT: APP_FAILED,
            TIMEOUT: APP_FAILED,
            CANCEL: APP_FAILED,
        },
    )
    state_machine.add_state(
        "FREE_AFTER_FAILURE",
        FreeCapabilityState(),
        transitions={
            SUCCEED: APP_FAILED,
            ABORT: APP_FAILED,
            TIMEOUT: APP_FAILED,
            CANCEL: APP_FAILED,
        },
    )
    return state_machine


def parse_args():
    parser = argparse.ArgumentParser(
        description="Request a portable relative movement through YASMIN."
    )
    parser.add_argument("--robot", choices=("nao", "pepper"), required=True)
    parser.add_argument("--x", type=float, required=True, help="Forward metres.")
    parser.add_argument("--y", type=float, required=True, help="Left metres.")
    parser.add_argument("--theta", type=float, required=True, help="Yaw radians.")
    parser.add_argument("--confirm", required=True, help='Must be "MOVER".')
    parser.add_argument("--provider", default=DEFAULT_PROVIDER)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    if args.confirm != "MOVER":
        parser.error('Movement requires --confirm MOVER.')
    values = (args.x, args.y, args.theta)
    if not all(math.isfinite(value) for value in values):
        parser.error("Movement components must be finite.")
    if all(abs(value) <= 1e-6 for value in values):
        parser.error("At least one movement component must be non-zero.")
    if (
        abs(args.x) > MAXIMUM_X_METRES
        or abs(args.y) > MAXIMUM_Y_METRES
        or abs(args.theta) > MAXIMUM_THETA_RADIANS
    ):
        parser.error("Movement exceeds limits: x=0.5 m, y=0.3 m, theta=pi/2 rad.")
    return args


def main():
    args = parse_args()
    rclpy.init()
    set_ros_loggers()
    blackboard = Blackboard()
    blackboard["capability"] = CAPABILITY
    blackboard["preferred_provider"] = args.provider
    blackboard["x_m"] = args.x
    blackboard["y_m"] = args.y
    blackboard["theta_rad"] = args.theta
    blackboard["released"] = False
    blackboard["message"] = "move_response_unavailable"
    blackboard["odometry_retries"] = 0

    outcome = APP_FAILED
    try:
        yasmin.YASMIN_LOG_INFO(
            f"Requesting MoveRelative for {args.robot}: "
            f"x={args.x:.3f} m, y={args.y:.3f} m, theta={args.theta:.3f} rad"
        )
        outcome = build_state_machine()(blackboard)
    except Exception as error:
        blackboard["message"] = f"application_exception: {error}"
        yasmin.YASMIN_LOG_ERROR(blackboard["message"])
    finally:
        YasminNode.destroy_instance()
        if rclpy.ok():
            rclpy.shutdown()

    result = {
        "outcome": outcome,
        "robot": args.robot,
        "capability": CAPABILITY,
        "requested_provider": args.provider,
        "provider": blackboard["provider"] if "provider" in blackboard else "",
        "request": {"x_m": args.x, "y_m": args.y, "theta_rad": args.theta},
        "achieved": blackboard["achieved"] if "achieved" in blackboard else None,
        "message": blackboard["message"],
        "released": blackboard["released"],
        "odometry_retries": blackboard["odometry_retries"],
    }
    print(json.dumps(result, ensure_ascii=False))
    return 0 if outcome == APP_SUCCEEDED else 1


if __name__ == "__main__":
    raise SystemExit(main())

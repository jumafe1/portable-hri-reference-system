#!/usr/bin/env python3
"""Run the portable Speak capability as a small YASMIN application."""

import argparse
import json
import sys

import rclpy
from hri_capability_interfaces.srv import Speak
from rclpy.utilities import remove_ros_args

from capability_lifecycle import (
    EstablishBondState,
    FreeCapabilityState,
    UseCapabilityState,
    service_options,
)

try:
    import yasmin
    from yasmin import Blackboard, StateMachine
    from yasmin_ros import ServiceState, set_ros_loggers
    from yasmin_ros.basic_outcomes import ABORT, SUCCEED, TIMEOUT
    from yasmin_ros.yasmin_node import YasminNode
except ModuleNotFoundError as error:
    if error.name and error.name.split(".")[0] in {"yasmin", "yasmin_ros"}:
        raise SystemExit(
            "YASMIN is not available in this Python environment. Install "
            "ros-jazzy-yasmin and ros-jazzy-yasmin-ros, then source "
            "/opt/ros/jazzy/setup.bash before the workspace overlay."
        ) from None
    raise


CAPABILITY = "hri_capability_interfaces/Speak"
DEFAULT_PROVIDER = "hri_naoqi_providers/NaoSpeak"
APP_SUCCEEDED = "application_succeeded"
APP_FAILED = "application_failed"


class SpeakState(ServiceState):
    def __init__(self):
        super().__init__(
            Speak,
            "/hri/speak",
            self.create_request,
            response_handler=self.handle_response,
            **service_options(),
        )

    @staticmethod
    def create_request(blackboard):
        return Speak.Request(
            text=blackboard["text"],
            language=blackboard["language"],
        )

    @staticmethod
    def handle_response(blackboard, response):
        blackboard["provider"] = response.provider
        blackboard["message"] = response.message
        return SUCCEED if response.success else ABORT


def build_state_machine():
    """Build a finite state machine that always releases an acquired capability."""
    state_machine = StateMachine(
        outcomes=[APP_SUCCEEDED, APP_FAILED],
        handle_sigint=True,
    )
    state_machine.add_state(
        "ESTABLISH_BOND",
        EstablishBondState(),
        transitions={
            SUCCEED: "USE_SPEAK",
            ABORT: APP_FAILED,
            TIMEOUT: APP_FAILED,
        },
    )
    state_machine.add_state(
        "USE_SPEAK",
        UseCapabilityState(),
        transitions={
            SUCCEED: "SPEAK",
            ABORT: "FREE_AFTER_FAILURE",
            TIMEOUT: "FREE_AFTER_FAILURE",
        },
    )
    state_machine.add_state(
        "SPEAK",
        SpeakState(),
        transitions={
            SUCCEED: "FREE_AFTER_SUCCESS",
            ABORT: "FREE_AFTER_FAILURE",
            TIMEOUT: "FREE_AFTER_FAILURE",
        },
    )
    state_machine.add_state(
        "FREE_AFTER_SUCCESS",
        FreeCapabilityState(),
        transitions={
            SUCCEED: APP_SUCCEEDED,
            ABORT: APP_FAILED,
            TIMEOUT: APP_FAILED,
        },
    )
    state_machine.add_state(
        "FREE_AFTER_FAILURE",
        FreeCapabilityState(),
        transitions={
            SUCCEED: APP_FAILED,
            ABORT: APP_FAILED,
            TIMEOUT: APP_FAILED,
        },
    )
    return state_machine


def parse_args():
    parser = argparse.ArgumentParser(
        description="Request portable speech through YASMIN and Capabilities2."
    )
    parser.add_argument("--robot", choices=("nao", "pepper"), required=True)
    parser.add_argument(
        "--text",
        default="Hola, esta es una aplicación portable con YASMIN.",
    )
    parser.add_argument("--language", default="Spanish")
    parser.add_argument("--provider", default=DEFAULT_PROVIDER)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main():
    args = parse_args()
    if not args.text.strip():
        raise SystemExit("The speech text cannot be empty.")

    rclpy.init()
    set_ros_loggers()
    blackboard = Blackboard()
    blackboard["text"] = args.text
    blackboard["language"] = args.language
    blackboard["capability"] = CAPABILITY
    blackboard["preferred_provider"] = args.provider
    blackboard["released"] = False

    outcome = APP_FAILED
    state_machine = None
    try:
        yasmin.YASMIN_LOG_INFO(
            f"Starting portable Speak application for {args.robot}"
        )
        state_machine = build_state_machine()
        outcome = state_machine(blackboard)
    except Exception as error:  # YASMIN validates transitions at runtime.
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
        "message": blackboard["message"] if "message" in blackboard else "",
        "released": blackboard["released"],
    }
    if state_machine is not None:
        result["final_state"] = state_machine.get_current_state()
    print(json.dumps(result, ensure_ascii=False))
    return 0 if outcome == APP_SUCCEEDED else 1


if __name__ == "__main__":
    raise SystemExit(main())

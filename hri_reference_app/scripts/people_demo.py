#!/usr/bin/env python3
"""Run portable continuous person detection as a reusable YASMIN state."""

import argparse
import json
import sys
import time

import rclpy
from hri_capability_interfaces.msg import PersonDetection2DArray
from rclpy.utilities import remove_ros_args

from capability_lifecycle import (
    EstablishBondState,
    FreeCapabilityState,
    UseCapabilityState,
)

try:
    import yasmin
    from yasmin import Blackboard, CbState, StateMachine
    from yasmin_ros import MonitorState, set_ros_loggers
    from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED, TIMEOUT
    from yasmin_ros.yasmin_node import YasminNode
except ModuleNotFoundError as error:
    if error.name and error.name.split(".")[0] in {"yasmin", "yasmin_ros"}:
        raise SystemExit(
            "YASMIN is not available in this Python environment. Install "
            "ros-jazzy-yasmin and ros-jazzy-yasmin-ros, then source "
            "/opt/ros/jazzy/setup.bash before the workspace overlay."
        ) from None
    raise


CAPABILITY = "hri_capability_interfaces/DetectPeople"
DEFAULT_PROVIDER = "hri_yolo_providers/YoloDetectPeople"
PEOPLE_TOPIC = "/hri/people"
APP_SUCCEEDED = "application_succeeded"
APP_NO_PERSON = "application_no_person"
APP_FAILED = "application_failed"
PERSON_FOUND = "person_found"
NO_PERSON = "no_person"
CONTINUE_MONITORING = "continue_monitoring"
OBSERVATION_TIMEOUT = "observation_timeout"
MONITOR_POLL_SECONDS = 1.0


def person_dict(person):
    return {
        "confidence": person.confidence,
        "tracking_id": person.tracking_id,
        "center_x": person.center_x,
        "center_y": person.center_y,
        "width": person.width,
        "height": person.height,
    }


class MonitorPeopleState(MonitorState):
    """Consume portable frames until one contains a person above the threshold."""

    def __init__(self):
        super().__init__(
            PersonDetection2DArray,
            PEOPLE_TOPIC,
            {PERSON_FOUND, NO_PERSON},
            self.handle_frame,
            timeout=MONITOR_POLL_SECONDS,
            maximum_retry=0,
        )

    @staticmethod
    def handle_frame(blackboard, frame):
        blackboard["observed_provider"] = frame.provider
        blackboard["last_frame_stamp"] = {
            "sec": frame.header.stamp.sec,
            "nanosec": frame.header.stamp.nanosec,
        }
        people = [
            person_dict(person)
            for person in frame.people
            if person.confidence >= blackboard["minimum_confidence"]
        ]
        blackboard["people"] = people
        if people:
            blackboard["message"] = "person_detected"
            return PERSON_FOUND
        blackboard["message"] = "no_people_in_frame"
        return NO_PERSON


def check_deadline(blackboard):
    if time.monotonic() >= blackboard["deadline_monotonic"]:
        blackboard["message"] = "observation_timeout"
        return OBSERVATION_TIMEOUT
    return CONTINUE_MONITORING


def build_state_machine():
    """Build a finite machine that releases DetectPeople after acquisition."""
    state_machine = StateMachine(
        outcomes=[APP_SUCCEEDED, APP_NO_PERSON, APP_FAILED],
        handle_sigint=True,
    )
    state_machine.add_state(
        "ESTABLISH_BOND",
        EstablishBondState(),
        transitions={SUCCEED: "USE_DETECT_PEOPLE", ABORT: APP_FAILED, TIMEOUT: APP_FAILED},
    )
    state_machine.add_state(
        "USE_DETECT_PEOPLE",
        UseCapabilityState(),
        transitions={
            SUCCEED: "MONITOR_PEOPLE",
            ABORT: APP_FAILED,
            TIMEOUT: APP_FAILED,
            CANCEL: APP_FAILED,
        },
    )
    state_machine.add_state(
        "MONITOR_PEOPLE",
        MonitorPeopleState(),
        transitions={
            PERSON_FOUND: "FREE_AFTER_SUCCESS",
            NO_PERSON: "CHECK_DEADLINE",
            TIMEOUT: "CHECK_DEADLINE",
            CANCEL: "FREE_AFTER_FAILURE",
        },
    )
    state_machine.add_state(
        "CHECK_DEADLINE",
        CbState({CONTINUE_MONITORING, OBSERVATION_TIMEOUT}, check_deadline),
        transitions={
            CONTINUE_MONITORING: "MONITOR_PEOPLE",
            OBSERVATION_TIMEOUT: "FREE_AFTER_NO_PERSON",
        },
    )
    state_machine.add_state(
        "FREE_AFTER_SUCCESS",
        FreeCapabilityState(),
        transitions={SUCCEED: APP_SUCCEEDED, ABORT: APP_FAILED, TIMEOUT: APP_FAILED},
    )
    state_machine.add_state(
        "FREE_AFTER_NO_PERSON",
        FreeCapabilityState(),
        transitions={SUCCEED: APP_NO_PERSON, ABORT: APP_FAILED, TIMEOUT: APP_FAILED},
    )
    state_machine.add_state(
        "FREE_AFTER_FAILURE",
        FreeCapabilityState(),
        transitions={SUCCEED: APP_FAILED, ABORT: APP_FAILED, TIMEOUT: APP_FAILED},
    )
    return state_machine


def parse_args():
    parser = argparse.ArgumentParser(
        description="Wait for a portable person detection through YASMIN and Capabilities2."
    )
    parser.add_argument("--robot", choices=("nao", "pepper"), required=True)
    parser.add_argument("--provider", default=DEFAULT_PROVIDER)
    parser.add_argument("--minimum-confidence", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=20.0)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main():
    args = parse_args()
    if not 0.5 <= args.minimum_confidence <= 1.0:
        raise SystemExit("--minimum-confidence must be in the [0.5, 1.0] range.")
    if args.timeout <= 0.0:
        raise SystemExit("--timeout must be greater than zero.")

    rclpy.init()
    set_ros_loggers()
    blackboard = Blackboard()
    blackboard["capability"] = CAPABILITY
    blackboard["preferred_provider"] = args.provider
    blackboard["minimum_confidence"] = args.minimum_confidence
    blackboard["deadline_monotonic"] = time.monotonic() + args.timeout
    blackboard["people"] = []
    blackboard["released"] = False
    blackboard["message"] = "waiting_for_people"

    outcome = APP_FAILED
    state_machine = None
    try:
        yasmin.YASMIN_LOG_INFO(
            f"Starting portable DetectPeople application for {args.robot}"
        )
        state_machine = build_state_machine()
        outcome = state_machine(blackboard)
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
        "provider": blackboard["observed_provider"] if "observed_provider" in blackboard else "",
        "message": blackboard["message"],
        "people": blackboard["people"],
        "released": blackboard["released"],
    }
    if "last_frame_stamp" in blackboard:
        result["last_frame_stamp"] = blackboard["last_frame_stamp"]
    if state_machine is not None:
        result["final_state"] = state_machine.get_current_state()
    print(json.dumps(result, ensure_ascii=False))
    return 0 if outcome in {APP_SUCCEEDED, APP_NO_PERSON} else 1


if __name__ == "__main__":
    raise SystemExit(main())

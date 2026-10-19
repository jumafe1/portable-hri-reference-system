"""Preview-only person-following mode within the common YASMIN application."""

import argparse
import json
import math
import signal
import sys
import threading

import rclpy
from capabilities2_msgs.srv import EstablishBond, FreeCapability, UseCapability
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from yasmin import Blackboard, CbState, StateMachine
from yasmin_ros import ServiceState, set_ros_loggers
from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED, TIMEOUT

from capability_lifecycle import EstablishBondState, FreeCapabilityState, UseCapabilityState
from follow_session import CAPABILITY, PROVIDER, FollowSession


APP_SUCCEEDED = "application_succeeded"
APP_FAILED = "application_failed"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="NON-ACTUATING follow preview; does not move the robot.")
    parser.add_argument("--robot", choices=("nao", "pepper"), required=True)
    parser.add_argument("--image-width", type=int, required=True)
    parser.add_argument("--image-height", type=int, required=True)
    parser.add_argument("--expected-frame", required=True)
    parser.add_argument("--minimum-confidence", type=float, default=0.65)
    parser.add_argument("--target-height-ratio", type=float, default=0.6)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:] if argv is None else argv)
    if not 1 <= args.image_width <= 8192 or not 1 <= args.image_height <= 8192:
        parser.error("Use actual image dimensions in [1, 8192].")
    if not args.expected_frame.strip():
        parser.error("Use the exact non-empty image header.frame_id.")
    if not all(math.isfinite(v) for v in (args.minimum_confidence, args.target_height_ratio, args.timeout)):
        parser.error("Follow preview arguments must be finite.")
    if not 0.5 <= args.minimum_confidence <= 1 or not 0.3 <= args.target_height_ratio <= 0.8:
        parser.error("Use confidence in [0.5, 1] and target height ratio in [0.3, 0.8].")
    if not 1 <= args.timeout <= 60:
        parser.error("Preview timeout must be in [1, 60] seconds.")
    return args


def build_state_machine(session):
    machine = StateMachine(outcomes=[APP_SUCCEEDED, APP_FAILED], handle_sigint=False)
    options = dict(wait_timeout=2.0, response_timeout=3.0, maximum_retry=0, node=session.node)
    bond = ServiceState(
        EstablishBond, "/capabilities/establish_bond", lambda _: EstablishBond.Request(),
        response_handler=EstablishBondState.handle_response,
        wait_timeout=5.0, response_timeout=5.0, maximum_retry=0, node=session.node,
    )
    machine.add_state("ESTABLISH_BOND", bond, transitions={
        SUCCEED: "START_HEARTBEAT", ABORT: APP_FAILED, CANCEL: APP_FAILED, TIMEOUT: APP_FAILED,
    })
    machine.add_state("START_HEARTBEAT", CbState({SUCCEED, ABORT}, session.start_bond), transitions={
        SUCCEED: "USE_DETECT_PEOPLE", ABORT: APP_FAILED,
    })
    acquire = ServiceState(UseCapability, "/capabilities/use_capability", UseCapabilityState.create_request, **options)
    machine.add_state("USE_DETECT_PEOPLE", acquire, transitions={
        SUCCEED: "VERIFY_PROVIDER", ABORT: "FREE_AFTER_FAILURE",
        CANCEL: "FREE_AFTER_FAILURE", TIMEOUT: "FREE_AFTER_FAILURE",
    })
    machine.add_state("VERIFY_PROVIDER", CbState({SUCCEED, ABORT}, session.wait_ready), transitions={
        SUCCEED: "PREVIEW_FOLLOW", ABORT: "FREE_AFTER_FAILURE",
    })
    machine.add_state("PREVIEW_FOLLOW", CbState({SUCCEED, ABORT}, session.observe), transitions={
        SUCCEED: "FREE_AFTER_SUCCESS", ABORT: "FREE_AFTER_FAILURE",
    })
    for name, result in (("FREE_AFTER_SUCCESS", APP_SUCCEEDED), ("FREE_AFTER_FAILURE", APP_FAILED)):
        release = ServiceState(
            FreeCapability, "/capabilities/free_capability", FreeCapabilityState.create_request,
            response_handler=FreeCapabilityState.handle_response, **options,
        )
        machine.add_state(name, release, transitions={
            SUCCEED: result, ABORT: APP_FAILED, CANCEL: APP_FAILED, TIMEOUT: APP_FAILED,
        })
    return machine


def run(args):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    cancelled = threading.Event()
    previous = {sig: signal.signal(sig, lambda _sig, _frame: cancelled.set())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    blackboard = Blackboard()
    for key, value in {"capability": CAPABILITY, "preferred_provider": PROVIDER,
                       "released": False, "message": "preview_not_started"}.items():
        blackboard[key] = value
    session = None
    outcome = APP_FAILED
    context_can_close = True
    try:
        session = FollowSession(args, cancelled)
        set_ros_loggers(session.node)
        outcome = build_state_machine(session)(blackboard)
    except Exception as error:
        blackboard["message"] = f"application_exception: {error}"
    finally:
        if session is not None:
            try:
                session.finish_preview("preview_finished")
                session.release_fallback(blackboard)
            except Exception as error:
                outcome = APP_FAILED
                blackboard["message"] += f"; cleanup_exception: {error}"
            finally:
                if session.bond is not None:
                    session.bond.shutdown()
                try:
                    session.close_perception()
                except Exception as error:
                    outcome = APP_FAILED
                    context_can_close = False
                    blackboard["message"] += f"; perception_cleanup_exception: {error}"
        # A live receiver must not race destruction of its shared ROS context.
        # Report failure and let process exit reclaim it if bounded cleanup failed.
        if context_can_close:
            if rclpy.ok():
                rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    if cancelled.is_set() or not blackboard["released"]:
        outcome = APP_FAILED
    print(json.dumps({
        "outcome": outcome, "mode": "follow_person", "robot": args.robot,
        "capability": CAPABILITY, "provider": PROVIDER,
        "dry_run": True, "motion_enabled": False, "physical_success_verified": False,
        "message": blackboard["message"], "released": blackboard["released"],
        "frames_received": session.frames if session else 0,
        "tracked_frames": session.tracked_frames if session else 0,
        "preview_samples": session.samples if session else 0,
    }, ensure_ascii=False))
    return 0 if outcome == APP_SUCCEEDED else 1

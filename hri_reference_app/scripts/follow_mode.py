"""Visual following until cancellation or safety stop; preview by default."""

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
from follow_speech import PROVIDER as SPEECH_PROVIDER


APP_SUCCEEDED = "application_succeeded"
APP_FAILED = "application_failed"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Follow preview by default; motion requires explicit opt-in.")
    parser.add_argument("--robot", choices=("nao", "pepper"), required=True)
    parser.add_argument("--image-width", type=int, required=True)
    parser.add_argument("--image-height", type=int, required=True)
    parser.add_argument("--expected-frame", required=True)
    parser.add_argument("--minimum-confidence", type=float, default=0.65)
    parser.add_argument("--target-height-ratio", type=float, default=0.6)
    parser.add_argument("--max-forward-speed", type=float, default=0.03,
                        help="Forward speed ceiling in m/s (0, 0.10]; also scales proportional response.")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--orient-camera", action="store_true",
                        help="Optional calibrated pitch framing during confirmed motion; preview never actuates it.")
    parser.add_argument("--camera-provider", default="hri_naoqi_providers/NaoqiOrientCamera")
    parser.add_argument("--transcribe-speech", action="store_true",
                        help="Optional serial microphone windows; external speech worker must be running.")
    parser.add_argument("--confirm-listen", default="")
    parser.add_argument("--speech-language", choices=("es", "en", ""), default="es")
    parser.add_argument("--speech-provider", default=SPEECH_PROVIDER)
    parser.add_argument("--print-transcript", action="store_true",
                        help="Print recognized text; terminal/ROS launch logs may persist it.")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:] if argv is None else argv)
    if not 1 <= args.image_width <= 8192 or not 1 <= args.image_height <= 8192:
        parser.error("Use actual image dimensions in [1, 8192].")
    if not args.expected_frame.strip():
        parser.error("Use the exact non-empty image header.frame_id.")
    if not args.camera_provider.strip():
        parser.error("camera-provider must not be empty.")
    if not all(math.isfinite(v) for v in (args.minimum_confidence, args.target_height_ratio)):
        parser.error("Follow preview arguments must be finite.")
    if not 0.5 <= args.minimum_confidence <= 1 or not 0.3 <= args.target_height_ratio <= 0.8:
        parser.error("Use confidence in [0.5, 1] and target height ratio in [0.3, 0.8].")
    if not math.isfinite(args.max_forward_speed) or not 0 < args.max_forward_speed <= 0.10:
        parser.error("Follow max-forward-speed must be finite and within (0, 0.10] m/s.")
    if args.enable_motion and args.confirm != "MOVER":
        parser.error("Motion requires --confirm MOVER.")
    if args.transcribe_speech and args.confirm_listen != "ESCUCHAR":
        parser.error("Microphone capture requires --confirm-listen ESCUCHAR.")
    if not args.speech_provider.strip():
        parser.error("speech-provider must not be empty.")
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
    motion = session.motion
    speech = getattr(session, "speech", None)
    following = "USE_MOVE_VELOCITY" if motion else "PREVIEW_FOLLOW"
    machine.add_state("VERIFY_PROVIDER", CbState({SUCCEED, ABORT}, session.wait_ready), transitions={
        SUCCEED: "CONFIGURE_SPEECH" if speech else following, ABORT: "FREE_AFTER_FAILURE",
    })
    if speech:
        machine.add_state("CONFIGURE_SPEECH", CbState({SUCCEED, ABORT}, speech.configure), transitions={
            SUCCEED: following, ABORT: "FREE_AFTER_FAILURE",
        })
    if motion:
        acquire_motion = ServiceState(UseCapability, "/capabilities/use_capability", motion.request, **options)
        machine.add_state("USE_MOVE_VELOCITY", acquire_motion, transitions={
            SUCCEED: "VERIFY_MOVE_PROVIDER", ABORT: "STOP_MOTION_FAILURE",
            CANCEL: "STOP_MOTION_FAILURE", TIMEOUT: "STOP_MOTION_FAILURE",
        })
        machine.add_state("VERIFY_MOVE_PROVIDER", CbState({SUCCEED, ABORT}, motion.wait_ready), transitions={
            SUCCEED: "CONFIGURE_CAMERA" if session.camera else "FOLLOW_PERSON", ABORT: "STOP_MOTION_FAILURE",
        })
        if session.camera:
            machine.add_state("CONFIGURE_CAMERA", CbState({SUCCEED, ABORT}, session.camera.configure), transitions={
                SUCCEED: "FOLLOW_PERSON", ABORT: "STOP_MOTION_FAILURE",
            })
        machine.add_state("FOLLOW_PERSON", CbState({SUCCEED, ABORT}, session.observe), transitions={
            SUCCEED: "STOP_MOTION_SUCCESS", ABORT: "STOP_MOTION_FAILURE",
        })
        for suffix in ("SUCCESS", "FAILURE"):
            machine.add_state("STOP_MOTION_" + suffix, CbState({SUCCEED, ABORT}, motion.stop), transitions={
                SUCCEED: ("STOP_CAMERA_" if session.camera else "FREE_MOTION_") + suffix,
                ABORT: "STOP_CAMERA_FAILURE" if session.camera else "FREE_MOTION_FAILURE",
            })
            if session.camera:
                machine.add_state("STOP_CAMERA_" + suffix, CbState({SUCCEED, ABORT}, session.camera.stop), transitions={
                    SUCCEED: "FREE_CAMERA_" + suffix, ABORT: "FREE_CAMERA_FAILURE",
                })
                machine.add_state("FREE_CAMERA_" + suffix, CbState({SUCCEED, ABORT}, session.camera.release), transitions={
                    SUCCEED: "FREE_MOTION_" + suffix, ABORT: "FREE_MOTION_FAILURE",
                })
            machine.add_state("FREE_MOTION_" + suffix, CbState({SUCCEED, ABORT}, motion.release), transitions={
                SUCCEED: "FREE_AFTER_" + suffix, ABORT: "FREE_AFTER_FAILURE",
            })
    else:
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
    motion_enabled = getattr(args, "enable_motion", False)
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
            speech = getattr(session, "speech", None) if getattr(args, "transcribe_speech", False) else None
            if speech:
                try:
                    speech.freeze("cancelled" if cancelled.is_set() else "stopped")
                except Exception:
                    outcome = APP_FAILED
                    blackboard["message"] += "; speech_freeze_exception"
            if motion_enabled and session.motion is not None:
                # Stop and each release are independent so one failure cannot skip the others.
                camera = session.camera if getattr(args, "orient_camera", False) else None
                cleanups = [session.motion.stop]
                if camera:
                    cleanups.extend((camera.stop, camera.release))
                cleanups.append(session.motion.release)
                for cleanup in cleanups:
                    try:
                        if cleanup(blackboard) != SUCCEED:
                            outcome = APP_FAILED
                    except Exception as error:
                        outcome = APP_FAILED
                        blackboard["message"] += f"; motion_cleanup_exception: {error}"
            if speech:
                try:
                    if speech.release() != SUCCEED:
                        outcome = APP_FAILED
                except Exception:
                    outcome = APP_FAILED
                    blackboard["message"] += "; speech_cleanup_exception"
            try:
                for cleanup in (lambda: session.finish_preview("preview_finished"),
                                lambda: session.release_fallback(blackboard)):
                    try:
                        cleanup()
                    except Exception as error:
                        outcome = APP_FAILED
                        blackboard["message"] += f"; cleanup_exception: {error}"
            finally:
                try:
                    if session.bond is not None:
                        session.bond.shutdown()
                except Exception as error:
                    outcome = APP_FAILED
                    blackboard["message"] += f"; bond_cleanup_exception: {error}"
                finally:
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
    motion = session.motion if session and motion_enabled else None
    motion_released = (not motion.requested or motion.released) if motion else not motion_enabled
    camera = session.camera if session and getattr(args, "orient_camera", False) and motion_enabled else None
    camera_released = not camera or not camera.requested or camera.released
    speech = getattr(session, "speech", None) if session and getattr(args, "transcribe_speech", False) else None
    speech_released = not speech or not speech.requested or speech.released
    if not motion_released or not camera_released or not speech_released:
        outcome = APP_FAILED
    print(json.dumps({
        "outcome": outcome, "mode": "follow_person", "robot": args.robot,
        "capability": CAPABILITY, "provider": PROVIDER,
        "dry_run": not motion_enabled, "motion_enabled": motion_enabled, "physical_success_verified": False,
        "message": blackboard["message"],
        "released": blackboard["released"] and motion_released and camera_released and speech_released,
        "speech_status": speech.status if speech else "disabled",
        "speech_released": speech_released,
        "transcripts_received": speech.transcripts_received if speech else 0,
        "camera_requested": bool(camera and camera.requested), "camera_released": camera_released,
        "camera_status": camera.status if camera else (
            "disabled_in_preview" if getattr(args, "orient_camera", False) else "disabled"),
        "camera_commands_sent": camera.commands_sent if camera else 0,
        "camera_hold_acknowledged": camera.hold_acknowledged if camera else False,
        "people_released": blackboard["released"], "motion_released": motion_released,
        "stop_acknowledged": motion.stop_acknowledged if motion else False,
        "commands_sent": motion.commands_sent if motion else 0,
        "frames_received": session.frames if session else 0,
        "tracked_frames": session.tracked_frames if session else 0,
        "preview_samples": session.samples if session else 0,
    }, ensure_ascii=False))
    return 0 if outcome == APP_SUCCEEDED else 1

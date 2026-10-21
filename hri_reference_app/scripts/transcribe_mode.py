"""Bounded, explicitly requested transcription in the common YASMIN application."""

import argparse
import json
import math
import signal
import sys
import threading
import time
import uuid

import rclpy
from bondpy import bondpy
from capabilities2_msgs.srv import EstablishBond, FreeCapability, GetRunningCapabilities, UseCapability
from hri_capability_interfaces.msg import SpeechTranscript
from hri_capability_interfaces.srv import TranscribeSpeech
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from yasmin import Blackboard, CbState, StateMachine


CAPABILITY = "hri_capability_interfaces/TranscribeSpeech"
PROVIDER = "hri_whisper_providers/WhisperTranscribeSpeech"
OK, FAIL = "succeeded", "aborted"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=("nao", "pepper"), required=True)
    parser.add_argument("--confirm", required=True)
    parser.add_argument("--record-duration", type=float, default=3.0)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--speech-language", choices=("es", "en", ""), default="es")
    parser.add_argument("--speech-provider", default=PROVIDER)
    parser.add_argument("--print-transcript", action="store_true",
                        help="Print recognized text; launch/terminal logging may persist it.")
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:] if argv is None else argv)
    if args.confirm != "ESCUCHAR":
        parser.error("Microphone capture requires --confirm ESCUCHAR.")
    if not math.isfinite(args.record_duration) or not 0.5 <= args.record_duration <= 10:
        parser.error("record-duration must be finite in [0.5,10] seconds.")
    if not math.isfinite(args.timeout) or not args.record_duration + 2 <= args.timeout <= 120:
        parser.error("timeout must be finite, at least record-duration + 2 and at most 120 seconds.")
    if not args.speech_provider.strip():
        parser.error("speech-provider must not be empty.")
    return args


class TranscribeSession:
    def __init__(self, args, cancelled):
        self.args, self.cancelled = args, cancelled
        self.node = Node("portable_hri_transcribe_app")
        self.executor = SingleThreadedExecutor(context=self.node.context)
        self.bond = None
        self.bond_id = ""
        self.attempted = False
        self.released = False
        self.request_id = uuid.uuid4().hex
        self.result = None
        self.lock = threading.Lock()
        self.message = "not_started"
        self.clients = {}
        try:
            for name, cls in (("establish_bond", EstablishBond), ("use_capability", UseCapability),
                              ("free_capability", FreeCapability),
                              ("get_running_capabilities", GetRunningCapabilities)):
                self.clients[name] = self.node.create_client(cls, "/capabilities/" + name)
            self.transcribe = self.node.create_client(TranscribeSpeech, "/hri/speech/transcribe")
            self.subscription = self.node.create_subscription(
                SpeechTranscript, "/hri/speech/transcript", self.on_result, 10)
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.thread.start()
        except Exception:
            self.executor.shutdown(timeout_sec=2)
            self.node.destroy_node()
            raise

    def on_result(self, result):
        with self.lock:
            if result.request_id != self.request_id or result.provider != self.args.speech_provider:
                return
            if self.result is None:
                self.result = result

    def call(self, client, request, seconds=3.0, cleanup=False):
        deadline = time.monotonic() + seconds
        while not client.service_is_ready() and time.monotonic() < deadline:
            if not rclpy.ok() or (self.cancelled.is_set() and not cleanup):
                raise RuntimeError("cancelled")
            time.sleep(0.02)
        if not client.service_is_ready():
            raise RuntimeError("service_unavailable")
        if not rclpy.ok() or (self.cancelled.is_set() and not cleanup):
            raise RuntimeError("cancelled")
        future = client.call_async(request)
        while not future.done() and time.monotonic() < deadline:
            if not rclpy.ok() or (self.cancelled.is_set() and not cleanup):
                raise RuntimeError("cancelled")
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError("service_timeout")
        return future.result()

    def establish(self, _blackboard):
        try:
            response = self.call(self.clients["establish_bond"], EstablishBond.Request(), 5)
            self.bond_id = response.bond_id
            if not self.bond_id:
                raise RuntimeError("empty_bond")
            self.bond = bondpy.Bond(self.node, "/capabilities/bonds", self.bond_id)
            self.bond.start()
            if not self.bond.wait_until_formed(Duration(seconds=5)) or self.bond.is_broken():
                raise RuntimeError("bond_unavailable")
            return OK
        except Exception as error:
            self.message = str(error)
            return FAIL

    def acquire(self, _blackboard):
        try:
            # Mark before the asynchronous request so a lost reply still triggers cleanup.
            self.attempted = True
            self.call(self.clients["use_capability"], UseCapability.Request(
                capability=CAPABILITY, preferred_provider=self.args.speech_provider,
                bond_id=self.bond_id))
            running = self.call(self.clients["get_running_capabilities"], GetRunningCapabilities.Request())
            if not any(item.capability.capability == CAPABILITY
                       and item.capability.provider == self.args.speech_provider
                       for item in running.running_capabilities):
                raise RuntimeError("requested_provider_not_running")
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and not self.cancelled.is_set():
                count = self.node.count_publishers("/hri/speech/transcript")
                if count > 1:
                    raise RuntimeError("ambiguous_transcript_publishers")
                if count == 1 and self.transcribe.service_is_ready():
                    return OK
                time.sleep(0.05)
            raise RuntimeError("speech_provider_unavailable")
        except Exception as error:
            self.message = str(error)
            return FAIL

    def listen(self, _blackboard):
        try:
            if self.cancelled.is_set():
                raise RuntimeError("cancelled")
            started = self.node.get_clock().now().nanoseconds
            deadline = time.monotonic() + self.args.timeout
            request = TranscribeSpeech.Request(
                duration_sec=self.args.record_duration, language=self.args.speech_language,
                request_id=self.request_id)
            admission_deadline = time.monotonic() + 2
            while True:
                response = self.call(self.transcribe, request)
                # Only an explicit non-admission is retryable; never retry a lost reply.
                if response.accepted or response.message != "lease_unavailable" or time.monotonic() >= admission_deadline:
                    break
                if self.cancelled.wait(0.1) or self.bond.is_broken():
                    raise RuntimeError("cancelled_or_bond_lost")
            if not response.accepted:
                raise RuntimeError(response.message or "transcription_rejected")
            if response.request_id != self.request_id or response.provider != self.args.speech_provider:
                raise RuntimeError("unexpected_transcription_admission")
            self.node.get_logger().info("capture_started: hable ahora; captura de audio acotada")
            while time.monotonic() < deadline and rclpy.ok() and not self.cancelled.is_set():
                if self.bond.is_broken() or self.node.count_publishers("/hri/speech/transcript") != 1:
                    raise RuntimeError("speech_provider_lost_or_ambiguous")
                with self.lock:
                    result = self.result
                if result is not None:
                    if not result.success:
                        raise RuntimeError(result.message or "transcription_failed")
                    stamp = result.header.stamp.sec * 1_000_000_000 + result.header.stamp.nanosec
                    if (stamp < started or stamp > self.node.get_clock().now().nanoseconds + 100_000_000
                            or not 0 <= result.header.stamp.nanosec < 1_000_000_000):
                        raise RuntimeError("invalid_audio_timestamp")
                    if not result.text.strip():
                        raise RuntimeError("empty_transcription")
                    self.message = "transcription_completed"
                    return OK
                self.cancelled.wait(0.05)
            raise RuntimeError("cancelled" if self.cancelled.is_set() else "transcription_timeout")
        except Exception as error:
            self.message = str(error)
            return FAIL

    def release(self, _blackboard=None):
        if not self.attempted or self.released:
            return OK
        try:
            self.call(self.clients["free_capability"], FreeCapability.Request(
                capability=CAPABILITY, bond_id=self.bond_id), cleanup=True)
            self.released = True
            return OK
        except Exception as error:
            self.message += "; release_failed: " + str(error)
            return FAIL

    def close(self):
        try:
            if self.bond is not None:
                self.bond.shutdown()
        finally:
            stopped = self.executor.shutdown(timeout_sec=2)
            self.thread.join(timeout=2)
            if not stopped or self.thread.is_alive():
                raise RuntimeError("speech_executor_did_not_stop")
            self.node.destroy_node()


def build_state_machine(session):
    machine = StateMachine(outcomes=["application_succeeded", "application_failed"], handle_sigint=False)
    machine.add_state("ESTABLISH_BOND", CbState({OK, FAIL}, session.establish),
                      transitions={OK: "USE_TRANSCRIBE_SPEECH", FAIL: "application_failed"})
    machine.add_state("USE_TRANSCRIBE_SPEECH", CbState({OK, FAIL}, session.acquire),
                      transitions={OK: "TRANSCRIBE", FAIL: "RELEASE_FAILURE"})
    machine.add_state("TRANSCRIBE", CbState({OK, FAIL}, session.listen),
                      transitions={OK: "RELEASE_SUCCESS", FAIL: "RELEASE_FAILURE"})
    for suffix in ("SUCCESS", "FAILURE"):
        machine.add_state("RELEASE_" + suffix, CbState({OK, FAIL}, session.release), transitions={
            OK: "application_succeeded" if suffix == "SUCCESS" else "application_failed",
            FAIL: "application_failed"})
    return machine


def run(args):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    cancelled = threading.Event()
    previous = {sig: signal.signal(sig, lambda *_: cancelled.set()) for sig in (signal.SIGINT, signal.SIGTERM)}
    session = None
    outcome = "application_failed"
    failure = "session_not_started"
    try:
        session = TranscribeSession(args, cancelled)
        outcome = build_state_machine(session)(Blackboard())
    except Exception as error:
        failure = str(error)
    finally:
        try:
            if session is not None:
                try:
                    if session.release() != OK:
                        outcome = "application_failed"
                finally:
                    session.close()
        except Exception as error:
            outcome = "application_failed"
            failure = "cleanup_failed: " + str(error)
        finally:
            rclpy.try_shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    result = {"mode": "transcribe_speech", "robot": args.robot, "outcome": outcome,
              "capability": CAPABILITY, "provider": args.speech_provider,
              "message": session.message if session and failure == "session_not_started" else failure,
              "released": bool(session and session.released), "physical_success_verified": False}
    if args.print_transcript and outcome == "application_succeeded":
        result["text"] = session.result.text
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0 if outcome == "application_succeeded" else 1

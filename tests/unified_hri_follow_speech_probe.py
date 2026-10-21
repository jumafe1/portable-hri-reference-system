#!/usr/bin/env python3
"""Isolated FollowPerson + real capability proxy + synthetic speech integration.

Requires --software-only, localhost DDS and an unused domain (example: 197).
Never starts a microphone, model, camera or physical robot driver. Synthetic
results prove coordination/privacy/routing, not recognition or physical safety.
"""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

import rclpy
from hri_capability_interfaces.msg import SpeechTranscript
from hri_capability_interfaces.srv import TranscribeSpeech
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from capabilities_naoqi_move_velocity_probe import software_guard, wait_until, zero
from unified_hri_follow_motion_probe import MotionFixture, MotionFrames, bounded
from unified_hri_velocity_probe import parse_result


PHRASE = "synthetic follow speech phrase"
INVALID_PHRASE = "invalid speech must not escape"
VOICE = "hri_capability_interfaces/TranscribeSpeech"


def assert_empty_graph(node):
    """Refuse even stray topic/service endpoints before creating fake backends."""
    time.sleep(1.5)
    others = list(node.get_node_names_and_namespaces())
    others.remove((node.get_name(), "/"))
    services = [name for name, _ in node.get_service_names_and_types()
                if not name.startswith(f"/{node.get_name()}/")]
    topics = [name for name, _ in node.get_topic_names_and_types()
              if name not in ("/rosout", "/parameter_events")]
    if others or services or topics:
        raise RuntimeError(f"DDS domain not empty: {others}, {services}, {topics}")


class SyntheticSpeech:
    def __init__(self, node, case):
        self.node, self.case = node, case
        self.lock = threading.Lock()
        self.requests, self.leases, self.results, self.pending = [], [], [], []
        self.capture_settings = []
        self.late_emitted = False
        self.publisher = node.create_publisher(SpeechTranscript, "/whisper/transcript", 10)
        self.subscription = node.create_subscription(
            SpeechTranscript, "/hri/speech/transcript", self.receive, 10)
        self.lease_subscription = node.create_subscription(String, "/whisper/lease", self.lease, 10)
        self.service = None if case == "absent" else node.create_service(
            TranscribeSpeech, "/whisper/transcribe", self.admit)
        self.timer = node.create_timer(.02, self.emit)

    def receive(self, message):
        with self.lock:
            self.results.append((time.monotonic(), message.request_id, message.text))

    def lease(self, message):
        with self.lock:
            self.leases.append((time.monotonic(), message.data))
            if self.case == "visual_loss" and not message.data and self.requests:
                self.pending.append((time.monotonic() + .2, self.requests[0][1],
                                     self.node.get_clock().now().nanoseconds,
                                     INVALID_PHRASE, True, "transcribed"))

    def admit(self, request, response):
        with self.lock:
            now = time.monotonic()
            self.requests.append((now, request.request_id))
            self.capture_settings.append((request.duration_sec, request.language))
            response.accepted = self.case != "reject"
            response.request_id = request.request_id
            response.message = "local_model_unavailable" if not response.accepted else "capture_started"
            if response.accepted and self.case not in ("pending_cancel", "visual_loss"):
                stamp = self.node.get_clock().now().nanoseconds
                if self.case == "wrong_id":
                    self.pending.append((now + request.duration_sec + .05, request.request_id + "-wrong", stamp,
                                         INVALID_PHRASE, True, "transcribed"))
                if self.case == "stale":
                    stamp -= 120_000_000_000
                silence = self.case == "repeated" and len(self.requests) == 1
                self.pending.append((now + request.duration_sec + .15, request.request_id, stamp,
                                     "" if silence else PHRASE,
                                     not silence, "no_speech" if silence else "transcribed"))
            return response

    def emit(self):
        with self.lock:
            due = [entry for entry in self.pending if entry[0] <= time.monotonic()]
            self.pending = [entry for entry in self.pending if entry[0] > time.monotonic()]
        for _, identifier, stamp, text, success, reason in due:
            result = SpeechTranscript()
            result.header.stamp.sec, result.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            result.request_id, result.text, result.language = identifier, text, "es"
            result.success, result.message = success, reason
            self.publisher.publish(result)
            if self.case == "visual_loss":
                self.late_emitted = True

    def snapshot(self):
        with self.lock:
            return list(self.requests), list(self.leases), list(self.results)

    def close(self):
        self.node.destroy_timer(self.timer)
        if self.service is not None:
            self.node.destroy_service(self.service)
        self.node.destroy_publisher(self.publisher)
        self.node.destroy_subscription(self.subscription)
        self.node.destroy_subscription(self.lease_subscription)


class VisualLossFrames(MotionFrames):
    def __init__(self, fixture, speech):
        self.speech, self.lost_at = speech, None
        super().__init__(fixture, "steady")

    def publish(self):
        requests = self.speech.snapshot()[0]
        if requests and self.fixture.moved() and time.monotonic() - requests[0][0] > .5:
            self.lost_at = self.lost_at or time.monotonic()
            self.case = "person_loss"
        super().publish()


def launch_case(case, path, fixture, speech):
    arguments = ["ros2", "launch", "hri_reference_app", "hri_app.launch.py", "robot:=nao",
                 "mode:=follow_person", "image_width:=320", "image_height:=240",
                 "expected_frame:=test_camera"]
    if case != "preview":
        arguments += ["enable_motion:=true", "confirm:=MOVER"]
    if case != "disabled":
        arguments += ["transcribe_speech:=true", "confirm_listen:=ESCUCHAR"]
    if case == "print":
        arguments += ["print_transcript:=true"]
    interrupted = None
    with path.open("w", encoding="utf-8") as output:
        process = subprocess.Popen(arguments, stdout=output, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            def observed():
                requests, leases, results = speech.snapshot()
                active = fixture.locked() if case == "preview" else fixture.moved()
                if not active:
                    return False
                if case == "disabled":
                    return len(fixture.snapshot(fixture.case_start)[0]) >= 50
                if case == "pending_cancel":
                    return bool(requests) and time.monotonic() - requests[0][0] > .25
                if case in ("reject", "absent", "stale"):
                    released = [stamp for stamp, owner in leases if not owner]
                    return bool(released) and time.monotonic() - released[0] > 1.0
                valid = [entry for entry in results if entry[2] == PHRASE]
                minimum = 2 if case == "repeated" else 1
                return len(valid) >= minimum and time.monotonic() - valid[-1][0] > .25

            if not wait_until(lambda: observed() or process.poll() is not None, 25):
                raise RuntimeError("Expected speech/follow observations missing before Ctrl+C")
            if process.poll() is None:
                interrupted = time.monotonic()
                os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=35)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
    return process.returncode, path.read_text(encoding="utf-8"), interrupted


def inspect_case(case, fixture, speech, head, output, interrupted, loss_at=None):
    result = parse_result(output)
    requests, leases, results = speech.snapshot()
    with speech.lock:
        capture_settings = list(speech.capture_settings)
    commands = fixture.recorders["/nao/cmd_vel"].since(fixture.case_start)
    checks = {
        "expected_termination": (result.get("message") == "person_missing" and interrupted is None
                                 if case == "visual_loss" else
                                 interrupted is not None and result.get("message") == "cancelled_or_ros_shutdown"),
        "released": result.get("released") is True,
        "not_physical_proof": result.get("physical_success_verified") is False,
        "bounded_commands": bounded(commands),
        "no_head_commands": not head,
        "text_privacy": (PHRASE in output) == (case == "print") and INVALID_PHRASE not in output,
        "internal_spanish_windows": all(duration == 3.0 and language == "es"
                                        for duration, language in capture_settings),
    }
    if case == "preview":
        checks["no_motion"] = not commands and result.get("commands_sent") == 0
    else:
        checks["motion_and_stop"] = (any(not zero(values) for _, values in commands)
                                     and bool(commands) and zero(commands[-1][1])
                                     and result.get("stop_acknowledged") is True)
        stop_at = loss_at if case == "visual_loss" else interrupted
        after = [stamp for stamp, values in commands if stop_at and stamp >= stop_at and zero(values)]
        checks["prompt_stop"] = bool(after) and after[0] - stop_at < 1.0
    if case == "disabled":
        checks["speech_unused"] = not requests and not leases
        checks["no_voice_acquisition"] = f"started capability: {VOICE}" not in output
    else:
        checks["voice_released"] = (bool(leases) and leases[-1][1] == ""
                                    and result.get("speech_released") is True)
    if case == "visual_loss":
        checks["capture_was_pending"] = len(requests) == 1 and loss_at is not None
        checks["late_result_ignored"] = (speech.late_emitted and not results
                                         and result.get("transcripts_received") == 0)
    if case in ("repeated", "print", "wrong_id", "preview"):
        valid = [entry for entry in results if entry[2] == PHRASE]
        checks["valid_results"] = len(valid) >= (2 if case == "repeated" else 1)
        checks["application_consumed_results"] = result.get("transcripts_received", 0) >= (
            2 if case == "repeated" else 1)
        checks["ids_mapped"] = all(identifier not in [request[1] for request in requests]
                                   for _, identifier, _ in results)
        checks["unique_requests"] = len({entry[1] for entry in requests}) == len(requests)
        checks["wrong_results_ignored"] = all(entry[2] != INVALID_PHRASE for entry in results)
        if case == "repeated":
            checks["silence_resumed"] = any(not entry[2] for entry in results) and len(requests) >= 3
            checks["three_complete_windows_without_trial_expiry"] = (
                len(results) >= 3 and valid[-1][0] - requests[0][0] >= 9.0)
            advancing = [stamp for stamp, values in commands if not zero(values)
                         and requests[0][0] <= stamp <= valid[-1][0]]
            gaps = [b - a for a, b in zip(advancing, advancing[1:])]
            checks["capture_does_not_block_motion"] = len(advancing) > 10 and bool(gaps) and max(gaps) < .5
    if case in ("reject", "absent", "stale"):
        released_at = next((stamp for stamp, owner in leases if not owner), None)
        checks["follow_survives_voice_fault"] = released_at is not None and any(
            stamp > released_at + .5 and not zero(values) for stamp, values in commands)
        checks["no_capture_after_disable"] = released_at is not None and all(
            stamp < released_at for stamp, _ in requests)
        expected = "disabled_invalid_audio_timestamp" if case == "stale" else "disabled_admission_rejected"
        checks["voice_fault_reported"] = result.get("speech_status") == expected
    return {"name": case, "passed": all(checks.values()), "checks": checks,
            "application": result, "requests": len(requests), "results": len(results),
            "capture_settings": capture_settings,
            "backend_commands": len(commands)}


def main():
    software_guard()
    evidence = Path(tempfile.mkdtemp(prefix="portable-hri-follow-speech-probe-"))
    report = {"started_utc": datetime.now(timezone.utc).isoformat(), "checks": [],
              "evidence_directory": str(evidence), "real_audio": False, "real_whisper": False,
              "physical_success_verified": False}
    rclpy.init()
    node = Node("software_follow_speech_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        assert_empty_graph(node)
        fixture = MotionFixture(node)
        head = []
        node.create_subscription(JointTrajectory, "/nao/joint_trajectory", head.append, 10)
        for case in ("disabled", "repeated", "print", "reject", "absent", "wrong_id",
                     "stale", "pending_cancel", "preview", "visual_loss"):
            fixture.begin("nao")
            head.clear()
            speech = SyntheticSpeech(node, case)
            frames = VisualLossFrames(fixture, speech) if case == "visual_loss" else MotionFrames(fixture, "steady")
            try:
                code, output, interrupted = launch_case(case, evidence / f"{case}.log", fixture, speech)
                time.sleep(.4)
                check = inspect_case(case, fixture, speech, head, output, interrupted,
                                     getattr(frames, "lost_at", None))
                check["exit_code"] = code
                report["checks"].append(check)
                print(json.dumps(check), flush=True)
            finally:
                frames.close()
                speech.close()
            time.sleep(.5)
    finally:
        executor.shutdown(timeout_sec=2)
        spin.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
        (evidence / "result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    passed = sum(check["passed"] for check in report["checks"])
    print(json.dumps({"passed": passed, "total": len(report["checks"]), "evidence": str(evidence)}))
    return 0 if passed == 10 else 1


if __name__ == "__main__":
    raise SystemExit(main())

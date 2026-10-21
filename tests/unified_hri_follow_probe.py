#!/usr/bin/env python3
"""Exercise follow_person preview through YASMIN and the real DetectPeople provider.

Synthetic YOLO frames only. Requires --software-only, localhost DDS and a fresh
ROS_DOMAIN_ID in 100..200. Never publishes Twist or contacts a robot. Results go
to a new temporary directory and cannot overwrite physical-test evidence.
"""

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from hri_capability_interfaces.msg import PersonDetection2DArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from yolo_msgs.msg import Detection, DetectionArray

from capabilities_naoqi_move_velocity_probe import Recorder, software_guard, wait_until
from unified_hri_velocity_probe import parse_result


COMMAND_TOPICS = ("/hri/cmd_vel", "/nao/cmd_vel", "/pepper/cmd_vel")


def preview_is_zero(sample):
    values = [sample.get("vx_m_s"), sample.get("wz_rad_s")]
    return all(isinstance(value, (int, float)) and math.isfinite(value)
               and abs(value) < 1e-9 for value in values)


class Fixture:
    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.previews = []
        self.invalid_previews = []
        self.publishers_seen = set()
        self.people_receipts = []
        self.telemetry_group = MutuallyExclusiveCallbackGroup()
        self.recorders = {topic: Recorder() for topic in COMMAND_TOPICS}
        self.subscriptions = [node.create_subscription(
            Twist, topic, recorder.command, 100)
            for topic, recorder in self.recorders.items()]
        self.subscriptions.append(node.create_subscription(
            String, "/hri/follow_person/preview", self.receive_preview,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)))
        self.graph_timer = node.create_timer(0.05, self.inspect_graph)
        self.subscriptions.append(node.create_subscription(
            PersonDetection2DArray, "/hri/people", self.receive_people,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.telemetry_group))

    def receive_people(self, message):
        received = time.monotonic()
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        sample = {"received_monotonic_s": received, "source_stamp_ns": stamp,
                  "source_age_s": (self.node.get_clock().now().nanoseconds - stamp) / 1e9,
                  "people_count": len(message.people)}
        with self.lock:
            self.people_receipts.append(sample)

    def receipt_telemetry(self):
        with self.lock:
            return list(self.people_receipts)

    def receive_preview(self, message):
        with self.lock:
            try:
                self.previews.append(json.loads(message.data))
            except json.JSONDecodeError:
                self.invalid_previews.append(message.data)

    def inspect_graph(self):
        for topic in COMMAND_TOPICS:
            if self.node.count_publishers(topic):
                with self.lock:
                    self.publishers_seen.add(topic)

    def reset(self):
        with self.lock:
            self.previews.clear()
            self.invalid_previews.clear()
            self.publishers_seen.clear()
            self.people_receipts.clear()

    def locked(self):
        with self.lock:
            return any(sample.get("locked") is True for sample in self.previews)

    def preview_connected(self):
        with self.lock:
            return bool(self.previews)

    def snapshot(self, start):
        with self.lock:
            previews = list(self.previews)
            invalid = list(self.invalid_previews)
            publishers = sorted(self.publishers_seen)
        counts = {topic: len(recorder.since(start))
                  for topic, recorder in self.recorders.items()}
        return previews, invalid, publishers, counts


class Frames:
    def __init__(self, fixture, case, topic):
        self.fixture = fixture
        self.node = fixture.node
        self.case = case
        self.fixed_stamp = None
        self.frame_count = 0
        self.telemetry_lock = threading.Lock()
        self.publish_samples = []
        self.publish_error = None
        self.publisher = self.node.create_publisher(DetectionArray, topic, 10)
        self.stop = threading.Event()
        # Keep the synthetic camera independent of diagnostic/graph callbacks.
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        try:
            while not self.stop.wait(0.1):
                if self.fixture.preview_connected():
                    self.publish()
        except Exception as error:
            with self.telemetry_lock:
                self.publish_error = repr(error)

    def publish(self):
        if self.case == "no_stream":
            return
        frame = DetectionArray()
        frame.header.frame_id = "other_camera" if self.case == "wrong_frame" else "test_camera"
        now_ns = self.node.get_clock().now().nanoseconds
        if self.case == "stale_stamp":
            now_ns -= 3_000_000_000
        elif self.case == "future_stamp":
            now_ns += 3_000_000_000
        elif self.case == "duplicate_stamp" and self.fixture.locked():
            if self.fixed_stamp is None:
                self.fixed_stamp = now_ns
            now_ns = self.fixed_stamp
        frame.header.stamp.sec, frame.header.stamp.nanosec = divmod(now_ns, 1_000_000_000)
        lost = self.case == "late_loss" and self.fixture.locked()
        if self.case != "empty" and not lost:
            person = Detection()
            person.class_name = "person"
            person.class_id = 0
            person.score = 0.92
            person.id = "person-1" if self.case == "tracked_id" else ""
            person.bbox.center.position.x = 160.0
            person.bbox.center.position.y = 120.0
            person.bbox.size.x = 60.0
            person.bbox.size.y = 100.0
            if self.case == "single_person_changes":
                # Abrupt center/size/ID changes are allowed under the declared
                # single-person assumption. Keep every box inside the frame.
                phase = (self.frame_count // 5) % 4
                person.id = ("", "temporary-left", "temporary-right", "")[phase]
                person.bbox.center.position.x = (160.0, 70.0, 250.0, 160.0)[phase]
                person.bbox.size.x = (60.0, 100.0, 50.0, 60.0)[phase]
                person.bbox.size.y = (100.0, 200.0, 200.0, 100.0)[phase]
            frame.detections = [person]
            if self.case == "ambiguous":
                other = Detection()
                other.class_name = "person"
                other.class_id = 0
                other.score = 0.91
                other.bbox.center.position.x = 250.0
                other.bbox.center.position.y = 120.0
                other.bbox.size.x = 60.0
                other.bbox.size.y = 100.0
                frame.detections.append(other)
        before_publish = time.monotonic()
        try:
            self.publisher.publish(frame)
            self.frame_count += 1
        finally:
            after_publish = time.monotonic()
            with self.telemetry_lock:
                self.publish_samples.append({
                    "started_monotonic_s": before_publish,
                    "finished_monotonic_s": after_publish,
                    "duration_s": after_publish - before_publish,
                    "source_stamp_ns": now_ns,
                    "people_count": len(frame.detections),
                })

    def telemetry(self):
        with self.telemetry_lock:
            return {"publish_count": len(self.publish_samples),
                    "publish_samples": list(self.publish_samples),
                    "publish_error": self.publish_error,
                    "thread_alive": self.thread.is_alive()}

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError("Synthetic publisher thread did not stop within 2s; "
                               "publisher retained to avoid destroying it during publish")
        self.node.destroy_publisher(self.publisher)
        if self.publish_error:
            raise RuntimeError(f"Synthetic publisher failed: {self.publish_error}")


def launch(arguments, path, fixture=None, interrupt=False, until=None):
    command = ["ros2", "launch", "hri_reference_app", "hri_app.launch.py",
               "mode:=follow_person", "image_width:=320", "image_height:=240",
               "expected_frame:=test_camera", *arguments]
    with path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   text=True, start_new_session=True)
        try:
            if interrupt:
                if not wait_until(lambda: (until or fixture.locked)()
                                  or process.poll() is not None, 12.0):
                    raise RuntimeError("Expected preview evidence absent before SIGINT test")
                if process.poll() is not None:
                    return process.returncode, path.read_text(encoding="utf-8")
                output = path.read_text(encoding="utf-8")
                match = re.search(r"\[hri_app-\d+\]: process started with pid \[(\d+)\]", output)
                if not match:
                    raise RuntimeError("Cannot resolve hri_app child PID from launch output")
                os.kill(int(match.group(1)), signal.SIGINT)
            process.wait(timeout=20)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
    return process.returncode, path.read_text(encoding="utf-8")


def main():
    software_guard()
    evidence = Path(tempfile.mkdtemp(prefix="portable-hri-follow-preview-probe-"))
    report = {"started_utc": datetime.now(timezone.utc).isoformat(), "checks": [],
              "evidence_directory": str(evidence),
              "limits": ["Synthetic YOLO detections; no physical following is verified.",
                         "Preview only: every Twist message, including zero, is forbidden."]}
    rclpy.init()
    node = Node("software_unified_follow_probe")
    # All observer callbacks are short. Avoid worker contention in the test
    # process starving its independent 10 Hz synthetic-camera thread.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    frames = None
    try:
        time.sleep(1.0)
        strangers = set(node.get_node_names()) - {node.get_name()}
        if strangers:
            raise RuntimeError(f"Domain is not isolated; refusing to run: {strangers}")
        fixture = Fixture(node)
        rejected = [("width", "image_width:=0"), ("height", "image_height:=-1"),
                    ("removed_timeout", "timeout:=120.0"),
                    ("removed_duration", "duration:=2.0"),
                    ("confidence", "minimum_confidence:=1.1"),
                    ("height_ratio", "target_height_ratio:=0")]
        for name, argument in rejected:
            fixture.reset()
            start = time.monotonic()
            code, output = launch(["robot:=nao", argument], evidence / f"invalid_{name}.log")
            previews, invalid, publishers, counts = fixture.snapshot(start)
            report["checks"].append({"name": f"rejects_invalid_{name}",
                "passed": code != 0 and "process started" not in output
                and not previews and not invalid and not publishers and not any(counts.values()),
                "launch_exit_code": code, "output_tail": output[-1000:]})

        cases = [("nao_valid", "valid", "nao", True),
                 ("pepper_valid", "valid", "pepper", True),
                 ("tracked_id", "tracked_id", "nao", True),
                 ("single_person_changes", "single_person_changes", "nao", True),
                 ("custom_topic", "valid", "nao", True),
                 ("no_stream", "no_stream", "nao", False),
                 ("empty", "empty", "nao", False),
                 ("late_loss", "late_loss", "nao", False),
                 ("stale_stamp", "stale_stamp", "nao", False),
                 ("future_stamp", "future_stamp", "nao", False),
                 ("duplicate_stamp", "duplicate_stamp", "nao", False),
                 ("wrong_frame", "wrong_frame", "nao", False),
                 ("ambiguous", "ambiguous", "nao", False),
                 ("interrupt", "valid", "nao", False)]
        for name, case, robot, success in cases:
            fixture.reset()
            topic = "/test_follow/detections" if name == "custom_topic" else "/yolo/detections"
            frames = Frames(fixture, case, topic)
            start = time.monotonic()
            try:
                arguments = [f"robot:={robot}", f"detections_topic:={topic}"]
                awaiting_target = name in ("empty", "ambiguous")
                controlled_stop = success or awaiting_target or name == "interrupt"
                # Harness deadlines bound this test, not the application's life.
                # Fresh frames without one unique person remain safely waiting
                # beyond the separate 5s initial-no-stream watchdog.
                minimum_samples = 120 if awaiting_target else 60
                until = (lambda: len(fixture.snapshot(start)[0]) >= minimum_samples) if (
                    success or awaiting_target) else None
                code, output = launch(arguments, evidence / f"{name}.log", fixture,
                                      interrupt=controlled_stop, until=until)
                result = parse_result(output)
                time.sleep(0.15)
                previews, invalid, publishers, counts = fixture.snapshot(start)
                passed = (result.get("outcome") == "application_failed"
                          and result.get("mode") == "follow_person"
                          and result.get("robot") == robot
                          and result.get("dry_run") is True
                          and result.get("motion_enabled") is False
                          and result.get("physical_success_verified") is False
                          and result.get("released") is True
                          and not invalid and not publishers and not any(counts.values())
                          and bool(previews) and all(p.get("dry_run") is True for p in previews)
                          and preview_is_zero(previews[-1]))
                if controlled_stop:
                    passed = passed and result.get("message") == "cancelled_or_ros_shutdown"
                if success:
                    passed = passed and result.get("tracked_frames", 0) >= 3
                    passed = passed and any(not preview_is_zero(p) for p in previews)
                elif awaiting_target:
                    passed = (passed and result.get("frames_received", 0) > 0
                              and len(previews) >= minimum_samples
                              and all(preview_is_zero(p) and not p.get("locked")
                                      for p in previews))
                if name in ("late_loss", "duplicate_stamp"):
                    passed = passed and any(p.get("locked") is True for p in previews)
                if name == "single_person_changes":
                    close = [p for p in previews if p.get("reason") == "preview_centering_close"]
                    passed = (passed
                              and any(p.get("vx_m_s", 0.0) > 0.0 for p in previews)
                              and any(p.get("wz_rad_s", 0.0) > 0.0 for p in close)
                              and any(p.get("wz_rad_s", 0.0) < 0.0 for p in close)
                              and all(p.get("vx_m_s") == 0.0 for p in close)
                              and not any(p.get("reason") == "target_discontinuity"
                                          for p in previews))
                report["checks"].append({"name": name, "passed": bool(passed),
                    "application": result, "launch_exit_code": code,
                    "velocity_messages": counts, "velocity_publishers_seen": publishers,
                    "preview_count": len(previews), "previews": previews,
                    "invalid_previews": invalid,
                    "source_telemetry": frames.telemetry(),
                    "portable_people_receipts": fixture.receipt_telemetry()})
            finally:
                frames.close()
    except Exception as error:
        report["execution_error"] = str(error)
        if frames is not None:
            report["last_source_telemetry"] = frames.telemetry()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2)
        if frames is not None and frames.thread.is_alive():
            # A blocked publish must not race native entity/context destruction.
            # Persist diagnostics and exit the failed probe; OS cleanup follows.
            report["execution_error"] = report.get("execution_error", "") + "; source thread still alive"
        else:
            node.destroy_node()
            rclpy.shutdown()
    report["passed"] = bool(report["checks"]) and "execution_error" not in report and all(
        check["passed"] for check in report["checks"])
    (evidence / "result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "evidence_directory": str(evidence),
                      "checks": [{"name": c["name"], "passed": c["passed"]}
                                 for c in report["checks"]],
                      "execution_error": report.get("execution_error")}, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""FollowPerson motion integration using synthetic images and fake robot subscribers.

Requires --software-only, ROS_LOCALHOST_ONLY=1, unused ROS_DOMAIN_ID 100..200.
Never run with robot drivers. Results go to a fresh temporary directory. Backend
Twist reception proves routing/stop commands, never physical movement or stopping.
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
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from yolo_msgs.msg import Detection, DetectionArray

from capabilities_naoqi_move_velocity_probe import software_guard, wait_until, zero
from unified_hri_follow_probe import Fixture
from unified_hri_velocity_probe import parse_result


class MotionFixture(Fixture):
    def __init__(self, node):
        super().__init__(node)
        # The motion protocol requires exactly one common command consumer: the
        # provider. Observing that topic would itself violate its exclusivity.
        node.destroy_subscription(self.subscriptions.pop(0))
        self.robot = "nao"
        self.case_start = time.monotonic()

    def moved(self):
        return any(not zero(values) for _, values in
                   self.recorders[f"/{self.robot}/cmd_vel"].since(self.case_start))

    def begin(self, robot):
        self.reset()
        self.robot = robot
        self.case_start = time.monotonic()
        self.interrupted_at = None


class MotionFrames:
    def __init__(self, fixture, case):
        self.fixture, self.node, self.case = fixture, fixture.node, case
        self.started = None
        self.sent = 0
        self.error = None
        self.publisher = self.node.create_publisher(DetectionArray, "/yolo/detections", 10)
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        try:
            while not self.done.wait(0.1):
                if not self.fixture.preview_connected():
                    continue
                self.started = self.started or time.monotonic()
                self.publish()
        except Exception as error:
            self.error = repr(error)

    def publish(self):
        moved = self.fixture.moved()
        if self.case == "stream_loss" and moved:
            return
        stamp = self.node.get_clock().now().nanoseconds
        if self.case == "stale_frame" and moved:
            stamp -= 2_000_000_000
        message = DetectionArray()
        message.header.frame_id = "test_camera"
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        person = Detection()
        person.class_name, person.class_id, person.score = "person", 0, 0.95
        person.bbox.center.position.x = 160.0
        person.bbox.center.position.y = 120.0
        person.bbox.size.x, person.bbox.size.y = 60.0, 100.0
        if self.case == "directions":
            elapsed = time.monotonic() - self.started
            # First acquire and advance, then turn either way without advancing.
            person.bbox.center.position.x = (160.0 if elapsed < 1.0 else
                                             80.0 if elapsed < 1.9 else
                                             240.0 if elapsed < 2.8 else 160.0)
        message.detections = [] if self.case == "person_loss" and moved else [person]
        if self.case == "ambiguous" and moved:
            other = Detection()
            other.class_name, other.class_id, other.score = "person", 0, 0.93
            other.bbox.center.position.x, other.bbox.center.position.y = 250.0, 120.0
            other.bbox.size.x, other.bbox.size.y = 50.0, 100.0
            message.detections.append(other)
        self.publisher.publish(message)
        self.sent += 1

    def close(self):
        self.done.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError("Synthetic image publisher did not stop")
        self.node.destroy_publisher(self.publisher)
        if self.error:
            raise RuntimeError(self.error)


def launch(arguments, path, fixture=None, interrupt=None):
    command = ["ros2", "launch", "hri_reference_app", "hri_app.launch.py",
               "mode:=follow_person", "enable_motion:=true", "confirm:=MOVER",
               "image_width:=320", "image_height:=240", "expected_frame:=test_camera",
               "timeout:=5.0", "duration:=4.0", *arguments]
    with path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   text=True, start_new_session=True)
        try:
            if interrupt:
                if not wait_until(fixture.moved, 15):
                    raise RuntimeError("No backend command before SIGINT")
                fixture.interrupted_at = time.monotonic()
                if interrupt == "process_group":
                    # A terminal sends Ctrl+C to its foreground process group,
                    # not solely to the application child.
                    os.killpg(process.pid, signal.SIGINT)
                elif interrupt == "launch":
                    process.send_signal(signal.SIGINT)
                elif interrupt == "application":
                    output = path.read_text(encoding="utf-8")
                    match = re.search(r"\[hri_app-\d+\]: process started with pid \[(\d+)\]", output)
                    if not match:
                        raise RuntimeError("Missing child PID for application-only interrupt")
                    os.kill(int(match.group(1)), signal.SIGINT)
                else:
                    raise ValueError(f"Unknown interrupt target: {interrupt}")
            process.wait(timeout=30)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
    return process.returncode, path.read_text(encoding="utf-8")


def bounded(commands):
    return all(all(math.isfinite(value) for value in values)
               and -1e-9 <= values[0] <= 0.030001
               and all(abs(values[index]) < 1e-9 for index in (1, 2, 3, 4))
               and abs(values[5]) <= 0.100001 for _, values in commands)


def main():
    software_guard()
    evidence = Path(tempfile.mkdtemp(prefix="portable-hri-follow-motion-probe-"))
    report = {"started_utc": datetime.now(timezone.utc).isoformat(), "checks": [],
              "evidence_directory": str(evidence),
              "limits": ["Fake backend subscribers, not physical robot validation.",
                         "Final zero proves a delivered command, not actuator stop."]}
    rclpy.init()
    node = Node("software_follow_motion_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    frames = None
    try:
        time.sleep(1)
        strangers = set(node.get_node_names()) - {node.get_name()}
        if strangers:
            raise RuntimeError(f"Domain is not isolated: {strangers}")
        fixture = MotionFixture(node)
        rejected = [("confirmation", "confirm:=NO"), ("empty_confirmation", "confirm:="),
                    ("malformed_opt_in", "enable_motion:=yes"),
                    ("duration_zero", "duration:=0"), ("duration_long", "duration:=301"),
                    ("duration_nan", "duration:=nan"), ("duration_inf", "duration:=inf"),
                    ("timeout_long", "timeout:=301")]
        for name, argument in rejected:
            fixture.begin("nao")
            code, output = launch(["robot:=nao", argument], evidence / f"invalid_{name}.log")
            previews, invalid, publishers, counts = fixture.snapshot(fixture.case_start)
            report["checks"].append({"name": f"rejects_{name}", "passed": code != 0
                and "process started" not in output and not previews and not invalid
                and not publishers and not any(counts.values()), "output_tail": output[-1000:]})

        fixture.begin("nao")
        frames = MotionFrames(fixture, "directions")
        try:
            code, output = launch(["robot:=nao", "enable_motion:=false", "timeout:=3.0"],
                                  evidence / "confirmation_without_opt_in.log")
            result = parse_result(output)
            previews, invalid, publishers, counts = fixture.snapshot(fixture.case_start)
            report["checks"].append({"name": "confirmation_without_opt_in_stays_preview",
                "passed": code == 0 and result.get("outcome") == "application_succeeded"
                and result.get("dry_run") is True and result.get("motion_enabled") is False
                and result.get("released") is True and result.get("commands_sent") == 0
                and bool(previews) and not invalid and not publishers and not any(counts.values()),
                "application": result, "velocity_publishers_seen": publishers,
                "backend_counts": counts})
        finally:
            frames.close()
            frames = None
        time.sleep(0.5)

        cases = [("nao", "directions"), ("pepper", "directions"),
                 ("nao", "person_loss"), ("nao", "stream_loss"),
                 ("nao", "stale_frame"), ("nao", "ambiguous"),
                 ("nao", "interrupt"), ("nao", "launch_interrupt_long"),
                 ("nao", "process_group_interrupt_long"), ("nao", "duplicate_commands"),
                 ("nao", "duplicate_status"), ("nao", "missing_backend")]
        for robot, case in cases:
            fixture.begin(robot)
            extra = None
            if case == "duplicate_commands":
                extra = node.create_publisher(Twist, "/hri/cmd_vel", 1)
            elif case == "duplicate_status":
                extra = node.create_publisher(String, "/hri/move_velocity/status", 1)
            elif case == "missing_backend":
                # After removing the common observer, backend indices are 0,1.
                node.destroy_subscription(fixture.subscriptions.pop(0))
            frames = MotionFrames(fixture, case)
            try:
                arguments = [f"robot:={robot}"]
                interrupt = {"interrupt": "application", "launch_interrupt_long": "launch",
                             "process_group_interrupt_long": "process_group"}.get(case)
                if case == "launch_interrupt_long":
                    arguments += ["duration:=300.0", "timeout:=120.0"]
                elif case == "process_group_interrupt_long":
                    arguments += ["duration:=300.0", "timeout:=300.0"]
                code, output = launch(arguments, evidence / f"{robot}_{case}.log",
                                      fixture, interrupt=interrupt)
                try:
                    result = parse_result(output)
                except RuntimeError as error:
                    # Preserve backend evidence even when shutdown loses the
                    # result JSON. Missing required result fields still fail.
                    result = {"parse_error": str(error)}
                time.sleep(0.2)
                commands = fixture.recorders[f"/{robot}/cmd_vel"].since(fixture.case_start)
                other = "pepper" if robot == "nao" else "nao"
                other_commands = fixture.recorders[f"/{other}/cmd_vel"].since(fixture.case_start)
                success = case == "directions"
                passed = (result.get("outcome") == ("application_succeeded" if success else "application_failed")
                          and result.get("robot") == robot and result.get("dry_run") is False
                          and result.get("motion_enabled") is True
                          and result.get("physical_success_verified") is False
                          and result.get("released") is True
                          and result.get("motion_released") is True
                          and result.get("people_released") is True
                          and bounded(commands) and not other_commands)
                if case not in ("duplicate_commands", "duplicate_status", "missing_backend"):
                    passed = (passed and bool(commands) and zero(commands[-1][1])
                              and any(not zero(values) for _, values in commands)
                              and result.get("stop_acknowledged") is True)
                else:
                    passed = passed and all(zero(values) for _, values in commands)
                if success:
                    passed = (passed and code == 0
                              and any(values[0] > 0 for _, values in commands)
                              and any(values[5] > 0 for _, values in commands)
                              and any(values[5] < 0 for _, values in commands))
                if case == "person_loss":
                    passed = passed and result.get("message") == "person_missing"
                if case in ("stream_loss", "stale_frame"):
                    passed = passed and result.get("message") in ("stale_or_future_image", "stale_detection")
                if interrupt:
                    passed = passed and result.get("message") == "cancelled_or_ros_shutdown"
                zero_delays = [stamp - fixture.interrupted_at for stamp, values in commands
                               if fixture.interrupted_at is not None
                               and stamp >= fixture.interrupted_at and zero(values)]
                first_zero_delay = min(zero_delays) if zero_delays else None
                remained_zero = None
                if interrupt:
                    remained_zero = bool(zero_delays) and all(
                        zero(values) for stamp, values in commands
                        if stamp >= fixture.interrupted_at + first_zero_delay)
                    passed = (passed and first_zero_delay is not None
                              and first_zero_delay <= 1.0 and remained_zero)
                report["checks"].append({"name": f"{robot}_{case}", "passed": bool(passed),
                    "launch_arguments": arguments, "interrupt_target": interrupt,
                    "first_zero_after_interrupt_s": first_zero_delay,
                    "zero_deadline_s": 1.0 if interrupt else None,
                    "remained_zero_after_stop": remained_zero,
                    "application": result, "launch_exit_code": code,
                    "backend_commands": commands, "other_backend_commands": other_commands,
                    "source_frames": frames.sent, "preview_count": len(fixture.previews)})
            finally:
                frames.close()
                frames = None
                if extra is not None:
                    node.destroy_publisher(extra)
            time.sleep(0.5)
    except Exception as error:
        report["execution_error"] = repr(error)
    finally:
        if frames is not None:
            frames.close()
        executor.shutdown(timeout_sec=3)
        worker.join(timeout=3)
        if worker.is_alive():
            report["execution_error"] = "Probe executor did not stop"
        else:
            node.destroy_node()
            rclpy.shutdown()
    report["passed"] = bool(report["checks"]) and "execution_error" not in report and all(
        check["passed"] for check in report["checks"])
    (evidence / "result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "evidence_directory": str(evidence),
                      "checks": [{"name": c["name"], "passed": c["passed"]} for c in report["checks"]],
                      "execution_error": report.get("execution_error")}, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

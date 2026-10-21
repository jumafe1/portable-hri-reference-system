#!/usr/bin/env python3
"""Software-only full FollowPerson / OrientCamera trial with fake robot endpoints.

Requires isolated localhost DDS, no physical drivers and --software-only. Fake
joint feedback proves software sequencing, never real head/base movement.
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
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, JointState
from trajectory_msgs.msg import JointTrajectory
from yolo_msgs.msg import Detection, DetectionArray

from capabilities_naoqi_move_velocity_probe import software_guard, wait_until, zero
from unified_hri_follow_motion_probe import MotionFixture, bounded
from unified_hri_velocity_probe import parse_result


class CameraRobot:
    def __init__(self, fixture, case):
        self.fixture, self.node, self.case = fixture, fixture.node, case
        self.lock = threading.Lock()
        self.commands = []
        self.elevation = 0.0
        self.pending = None
        self.moved_at = None
        self.error = None
        self.joints = self.node.create_publisher(JointState, "/nao/joint_states", 1)
        self.info = self.node.create_publisher(CameraInfo, "/nao/camera/front/camera_info", 1)
        self.images = self.node.create_publisher(DetectionArray, "/yolo/detections", 1)
        self.subscription = self.node.create_subscription(
            JointTrajectory, "/nao/joint_trajectory", self.on_command, 10)
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def on_command(self, message):
        with self.lock:
            self.commands.append({"received": time.monotonic(), "names": list(message.joint_names),
                                  "positions": list(message.points[0].positions),
                                  "velocities": list(message.points[0].velocities)})
            self.pending = (time.monotonic() + 0.15, -message.points[0].positions[0])

    def snapshot(self):
        with self.lock:
            return list(self.commands)

    def run(self):
        try:
            index = 0
            while not self.done.wait(0.02):
                now = time.monotonic()
                if self.fixture.moved() and self.moved_at is None:
                    self.moved_at = now
                with self.lock:
                    if self.pending and now >= self.pending[0]:
                        self.elevation = self.pending[1]
                        self.pending = None
                    elevation = self.elevation
                since_move = 0.0 if self.moved_at is None else now - self.moved_at
                stamp = self.node.get_clock().now().to_msg()
                if self.case != "missing_feedback" and not (
                        self.case == "stale_feedback" and since_move > 0.5):
                    joints = JointState()
                    joints.header.stamp = stamp
                    joints.name, joints.position = ["HeadYaw", "HeadPitch"], [0.0, -elevation]
                    self.joints.publish(joints)
                info = CameraInfo()
                info.header.stamp, info.header.frame_id = stamp, "test_camera"
                info.width, info.height = 320, 240
                info.k = [280.0, 0.0, 160.0, 0.0, 280.0, 120.0, 0.0, 0.0, 1.0]
                self.info.publish(info)
                index += 1
                if index % 5:
                    continue
                frame = DetectionArray()
                frame.header.stamp, frame.header.frame_id = stamp, "test_camera"
                person = Detection()
                person.class_name, person.class_id, person.score = "person", 0, 0.95
                person.bbox.center.position.x, person.bbox.center.position.y = 160.0, 120.0
                person.bbox.size.x, person.bbox.size.y = 60.0, 100.0
                if since_move > 0.5 and elevation < 0.025:
                    person.bbox.center.position.y = 50.0 if self.case == "clipped" else 80.0
                frame.detections = [person]
                self.images.publish(frame)
        except Exception as error:
            self.error = repr(error)

    def close(self):
        self.done.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError("Fake camera thread did not stop")
        for publisher in (self.joints, self.info, self.images):
            self.node.destroy_publisher(publisher)
        self.node.destroy_subscription(self.subscription)
        if self.error:
            raise RuntimeError(self.error)


def launch(case, path, camera):
    arguments = ["ros2", "launch", "hri_reference_app", "hri_app.launch.py", "robot:=nao",
                 "mode:=follow_person", "image_width:=320", "image_height:=240",
                 "expected_frame:=test_camera", "enable_motion:=true", "confirm:=MOVER",
                 "orient_camera:=true"]
    if case == "preview":
        arguments += ["enable_motion:=false"]
    if case == "unknown_provider":
        arguments += ["camera_provider:=missing/NotInstalled"]
    interrupted_at = None
    with path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(arguments, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            if case in ("integrated", "clipped", "interrupt", "preview"):
                def observations_complete():
                    if case == "preview":
                        return len(camera.fixture.snapshot(camera.fixture.case_start)[0]) >= 30
                    head = camera.snapshot()
                    if case == "interrupt":
                        return bool(head)
                    adjustments = [sample for sample in head if abs(sample["positions"][0]) > 0.005]
                    if not adjustments:
                        return False
                    first = adjustments[0]["received"]
                    return any(stamp > first + 0.35 and values[0] > 0 for stamp, values in
                               camera.fixture.recorders["/nao/cmd_vel"].since(first))
                if not wait_until(lambda: observations_complete() or process.poll() is not None, 15):
                    raise RuntimeError("Expected observations missing before operator stop")
                if process.poll() is None:
                    interrupted_at = time.monotonic()
                    os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=30)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
    return process.returncode, parse_result(path.read_text(encoding="utf-8")), interrupted_at


def main():
    software_guard()
    evidence = Path(tempfile.mkdtemp(prefix="portable-hri-follow-camera-probe-"))
    report = {"started_utc": datetime.now(timezone.utc).isoformat(), "checks": [],
              "limits": ["Software-only fake feedback; no physical actuation verified.",
                         "Synthetic recentering tests sequencing, not robot camera geometry."]}
    rclpy.init()
    node = Node("software_follow_camera_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    camera = None
    try:
        time.sleep(1)
        strangers = set(node.get_node_names()) - {node.get_name()}
        if strangers:
            raise RuntimeError(f"Domain not isolated: {strangers}")
        fixture = MotionFixture(node)
        for case in ("integrated", "clipped", "stale_feedback", "interrupt", "preview",
                     "missing_feedback", "unknown_provider"):
            fixture.begin("nao")
            camera = CameraRobot(fixture, case)
            try:
                code, result, interrupted_at = launch(case, evidence / f"{case}.log", camera)
                time.sleep(0.2)
                base = fixture.recorders["/nao/cmd_vel"].since(fixture.case_start)
                head = camera.snapshot()
                previews = fixture.snapshot(fixture.case_start)[0]
                moved = any(not zero(values) for _, values in base)
                passed = (result.get("physical_success_verified") is False and
                          result.get("released") is True and bounded(base))
                extra = {}
                if case in ("integrated", "clipped"):
                    passed = (passed and interrupted_at is not None
                              and result.get("outcome") == "application_failed"
                              and result.get("message") == "cancelled_or_ros_shutdown")
                    adjustments = [sample for sample in head if abs(sample["positions"][0]) > 0.005]
                    first = adjustments[0]["received"] if adjustments else None
                    before = [(stamp, values) for stamp, values in base if first and stamp < first]
                    during = [(stamp, values) for stamp, values in base if first and first <= stamp <= first + 0.35]
                    resumed = any(values[0] > 0 for stamp, values in base if first and stamp > first + 0.35)
                    paused = bool(before and during) and zero(before[-1][1]) and all(zero(v) for _, v in during)
                    passed = passed and moved and paused and resumed and bool(base) and zero(base[-1][1])
                    passed = passed and result.get("camera_released") is True
                    extra = {"base_zero_during_head_adjustment": paused, "forward_resumed": resumed}
                elif case == "preview":
                    passed = passed and not moved and not head and result.get("dry_run") is True
                    passed = (passed and interrupted_at is not None
                              and result.get("message") == "cancelled_or_ros_shutdown")
                elif case in ("missing_feedback", "unknown_provider"):
                    passed = passed and not moved and not head and result.get("outcome") == "application_failed"
                else:
                    passed = passed and moved and bool(base) and zero(base[-1][1])
                    passed = passed and result.get("outcome") == "application_failed"
                    passed = passed and result.get("motion_released") is True and result.get("camera_released") is True
                    if case == "interrupt":
                        passed = (passed and interrupted_at is not None and
                                  result.get("message") == "cancelled_or_ros_shutdown")
                        after = [(stamp, values) for stamp, values in base
                                 if interrupted_at is not None and stamp >= interrupted_at]
                        zeros = [stamp for stamp, values in after if zero(values)]
                        passed = passed and bool(zeros) and zeros[0] - interrupted_at < 1.0
                passed = passed and all(sample["names"] == ["HeadPitch"] and
                                       abs(sample["positions"][0]) <= 0.25 for sample in head)
                report["checks"].append({"name": case, "passed": bool(passed), "application": result,
                    "operator_stop_requested": interrupted_at is not None, "launch_exit_code": code,
                    "head_commands": head, "base_commands": base, "preview_samples": previews, **extra})
            finally:
                camera.close()
                camera = None
            time.sleep(0.5)
    except Exception as error:
        report["execution_error"] = repr(error)
    finally:
        if camera:
            camera.close()
        executor.shutdown(timeout_sec=3)
        worker.join(timeout=3)
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

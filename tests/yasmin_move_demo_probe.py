#!/usr/bin/env python3
"""Verify the YASMIN MoveRelative application without moving a robot."""

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from naoqi_utilities_msgs.srv import MoveTo
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


REQUEST = {"x": 0.1, "y": 0.05, "theta": 0.2}
PROVIDER = "hri_naoqi_providers/NaoqiMoveRelative"


def application_result(output):
    for line in reversed(output.splitlines()):
        start = line.find('{"outcome":')
        if start >= 0:
            return json.JSONDecoder().raw_decode(line[start:])[0]
    raise RuntimeError(f"Application did not print JSON. Output:\n{output[-3000:]}")


def run_launch(robot):
    command = [
        "ros2", "launch", "hri_reference_app", "move_demo.launch.py",
        f"robot:={robot}",
        f"x:={REQUEST['x']}",
        f"y:={REQUEST['y']}",
        f"theta:={REQUEST['theta']}",
        "confirm:=MOVER",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        output, _ = process.communicate(timeout=25)
    except subprocess.TimeoutExpired as error:
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5)
        raise RuntimeError(f"YASMIN move launch timed out: {output[-3000:]}") from error
    return process.returncode, application_result(output), output


def main():
    repository = Path(__file__).resolve().parents[1]
    evidence = repository / "results/yasmin_move_demo"
    evidence.mkdir(parents=True, exist_ok=True)
    report = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "checks": [],
        "limits": [
            "MoveTo and odometry are simulated; no physical robot is used.",
            "A provider timeout does not cancel an underlying NAOqi movement.",
        ],
    }

    rclpy.init()
    node = Node("yasmin_move_demo_probe")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    lock = threading.Lock()
    poses = {robot: {"x": 0.0, "y": 0.0, "yaw": 0.0} for robot in ("nao", "pepper")}
    behavior = {robot: "move" for robot in poses}
    events = []
    publishers = {}
    services = []
    timers = []

    for robot in poses:
        publishers[robot] = node.create_publisher(Odometry, f"/{robot}/odom", 10)

        def publish_pose(platform=robot):
            message = Odometry()
            message.header.frame_id = "odom"
            message.child_frame_id = "base_link"
            with lock:
                if behavior[platform] == "no_odometry":
                    return
                current = poses[platform].copy()
            message.pose.pose.position.x = current["x"]
            message.pose.pose.position.y = current["y"]
            message.pose.pose.orientation.z = math.sin(current["yaw"] / 2.0)
            message.pose.pose.orientation.w = math.cos(current["yaw"] / 2.0)
            publishers[platform].publish(message)

        def fake_move(request, response, platform=robot):
            target = (
                float(request.x_coordinate),
                float(request.y_coordinate),
                float(request.theta_coordinate),
            )
            with lock:
                events.append({"robot": platform, "target": target})
                if behavior[platform] == "move":
                    current = poses[platform]
                    yaw = current["yaw"]
                    current["x"] += math.cos(yaw) * target[0] - math.sin(yaw) * target[1]
                    current["y"] += math.sin(yaw) * target[0] + math.cos(yaw) * target[1]
                    current["yaw"] += target[2]
            return response

        timers.append(node.create_timer(0.05, publish_pose))
        services.append(node.create_service(MoveTo, f"/{robot}/move_to", fake_move))

    try:
        for robot in ("nao", "pepper"):
            with lock:
                poses[robot] = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                behavior[robot] = "move"
                events.clear()
            time.sleep(0.2)
            code, result, _ = run_launch(robot)
            with lock:
                observed = list(events)
            achieved = result["achieved"] or {}
            passed = (
                code == 0
                and result["outcome"] == "application_succeeded"
                and result["provider"] == PROVIDER
                and result["message"] == "motion_completed"
                and result["released"] is True
                and len(observed) == 1
                and observed[0]["robot"] == robot
                and all(abs(observed[0]["target"][index] - requested) < 1e-5
                        for index, requested in enumerate(REQUEST.values()))
                and all(abs(achieved.get(name, float("inf")) - requested) < 0.01
                        for name, requested in zip(
                            ("x_m", "y_m", "theta_rad"), REQUEST.values()
                        ))
            )
            report["checks"].append({
                "name": f"{robot}_movement_and_release",
                "passed": passed,
                "application": result,
                "backend_events": observed,
            })

        with lock:
            poses["nao"] = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            behavior["nao"] = "hold_position"
            events.clear()
        time.sleep(0.2)
        code, result, _ = run_launch("nao")
        with lock:
            observed = list(events)
        report["checks"].append({
            "name": "target_not_reached_and_released",
            "passed": code == 0
            and result["outcome"] == "application_failed"
            and result["message"] == "motion_target_not_reached"
            and result["released"] is True
            and len(observed) == 1,
            "application": result,
            "backend_events": observed,
        })

        with lock:
            behavior["nao"] = "no_odometry"
            events.clear()
        time.sleep(1.2)
        code, result, _ = run_launch("nao")
        with lock:
            observed = list(events)
        report["checks"].append({
            "name": "no_odometry_expires_without_backend_movement",
            "passed": code == 0
            and result["outcome"] == "application_failed"
            and result["message"] == "odometry_unavailable_before_motion"
            and result["released"] is True
            and result["odometry_retries"] >= 2
            and not observed,
            "application": result,
            "backend_events": observed,
        })

        command = [
            "ros2", "run", "hri_reference_app", "move_demo",
            "--robot", "nao", "--x", "0.1", "--y", "0", "--theta", "0",
        ]
        denied = subprocess.run(command, capture_output=True, text=True, timeout=5)
        invalid = subprocess.run(
            command + ["--confirm", "MOVER", "--x", "0.6"],
            capture_output=True, text=True, timeout=5,
        )
        report["checks"].append({
            "name": "unconfirmed_or_out_of_bounds_rejected_before_ros",
            "passed": denied.returncode != 0
            and invalid.returncode != 0
            and "required" in denied.stderr
            and "exceeds limits" in invalid.stderr,
        })
    except Exception as error:
        report["execution_error"] = str(error)
    finally:
        for timer in timers:
            node.destroy_timer(timer)
        for service in services:
            node.destroy_service(service)
        for publisher in publishers.values():
            node.destroy_publisher(publisher)
        executor.shutdown()
        executor_thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()

    report["passed"] = (
        bool(report["checks"])
        and "execution_error" not in report
        and all(check["passed"] for check in report["checks"])
    )
    (evidence / "result.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

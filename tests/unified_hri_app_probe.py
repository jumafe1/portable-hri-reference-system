#!/usr/bin/env python3
"""Exercise all six robot/mode combinations in an isolated software-only domain."""

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

import rclpy
from naoqi_utilities_msgs.srv import MoveTo, Say
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from yolo_msgs.msg import Detection, DetectionArray

from capabilities_naoqi_move_velocity_probe import software_guard, wait_until


EXPECTED_PROVIDERS = {
    "speak": "hri_naoqi_providers/NaoSpeak",
    "detect_people": "hri_yolo_providers/YoloDetectPeople",
    "move_relative": "hri_naoqi_providers/NaoqiMoveRelative",
}


def parse_result(output):
    for line in reversed(output.splitlines()):
        marker = line.find('{"outcome":')
        if marker >= 0:
            return json.JSONDecoder().raw_decode(line[marker:])[0]
    raise RuntimeError(f"Application result is missing:\n{output[-3000:]}")


def launch_application(robot, mode):
    command = [
        "ros2", "launch", "hri_reference_app", "hri_app.launch.py",
        f"robot:={robot}", f"mode:={mode}",
    ]
    if mode == "speak":
        command.append("text:=Prueba de voz simulada.")
    elif mode == "detect_people":
        command.append("timeout:=8.0")
    else:
        command.extend(("x:=0.1", "y:=0.0", "theta:=0.0", "confirm:=MOVER"))
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
        raise RuntimeError(f"{robot}/{mode} timed out:\n{output[-3000:]}") from error
    return process.returncode, parse_result(output), output


def rejected_launch(name, arguments, expected_message):
    command = [
        "ros2", "launch", "hri_reference_app", "hri_app.launch.py",
        "robot:=nao", "mode:=move_relative", *arguments,
    ]
    completed = subprocess.run(command, capture_output=True, text=True, timeout=10)
    output = completed.stdout + completed.stderr
    return {
        "name": name,
        "passed": completed.returncode != 0
        and expected_message in output
        and "process started" not in output,
        "output_tail": output[-1000:],
    }


def main():
    software_guard()
    evidence = Path(tempfile.mkdtemp(prefix="portable-hri-app-probe-"))
    report = {"started_utc": datetime.now(timezone.utc).isoformat(),
              "evidence_directory": str(evidence),
              "limits": ["Software-only; no physical movement verified.",
                         "Run sequentially: launchers share a SQLite path across ROS domains."]}
    rclpy.init()
    node = Node("unified_hri_app_probe")
    # Fixed synthetic streams and service callbacks do not need parallelism.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)
    strangers = set(node.get_node_names()) - {node.get_name()}
    if strangers:
        executor.shutdown()
        spin_thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(f"Domain is not isolated; refusing to run: {strangers}")
    lock = threading.Lock()
    events = []
    positions = {"nao": 0.0, "pepper": 0.0}
    services = []
    publishers = {}
    timers = []
    ticks = {"nao": [], "pepper": [], "detections": []}

    for robot in positions:
        publishers[robot] = node.create_publisher(Odometry, f"/{robot}/odom", 10)

        def say(request, response, platform=robot):
            with lock:
                events.append(("speak", platform, request.text))
            response.success = True
            response.message = "simulated_speech"
            return response

        def move(request, response, platform=robot):
            with lock:
                events.append(("move_relative", platform, request.x_coordinate))
                positions[platform] += request.x_coordinate
            return response

        def publish_odometry(platform=robot):
            message = Odometry()
            message.header.frame_id = "odom"
            message.child_frame_id = "base_link"
            message.pose.pose.orientation.w = 1.0
            with lock:
                message.pose.pose.position.x = positions[platform]
                ticks[platform].append(time.monotonic())
            publishers[platform].publish(message)

        services.append(node.create_service(Say, f"/{robot}/say", say))
        services.append(node.create_service(MoveTo, f"/{robot}/move_to", move))
        timers.append(node.create_timer(0.1, publish_odometry))

    detections = node.create_publisher(DetectionArray, "/yolo/detections", 10)

    def publish_person():
        frame = DetectionArray()
        frame.header.frame_id = "simulated_camera"
        frame.header.stamp = node.get_clock().now().to_msg()
        person = Detection()
        person.class_name = "person"
        person.score = 0.91
        person.id = "simulated-person"
        person.bbox.center.position.x = 100.0
        person.bbox.center.position.y = 80.0
        person.bbox.size.x = 50.0
        person.bbox.size.y = 100.0
        frame.detections = [person]
        detections.publish(frame)
        with lock:
            ticks["detections"].append(time.monotonic())

    timers.append(node.create_timer(0.1, publish_person))
    checks = []
    clients = []
    try:
        # Verify fixture readiness before launching the first application. This
        # is not a retry of a failed speech/movement request.
        for robot in positions:
            for kind, name in ((Say, "say"), (MoveTo, "move_to")):
                client = node.create_client(kind, f"/{robot}/{name}")
                clients.append(client)
                if not client.wait_for_service(timeout_sec=3.0):
                    raise RuntimeError(f"Fixture service did not become ready: /{robot}/{name}")
        if not wait_until(lambda: all(len(samples) >= 3 for samples in ticks.values())):
            raise RuntimeError("Fixture timers did not begin publishing.")
        report["fixture_ready"] = True
        checks.extend([
            rejected_launch(
                "movement_requires_confirmation",
                ("x:=0.1", "y:=0", "theta:=0"),
                "Movement requires confirm:=MOVER",
            ),
            rejected_launch(
                "movement_rejects_out_of_bounds",
                ("x:=0.6", "y:=0", "theta:=0", "confirm:=MOVER"),
                "Movement exceeds",
            ),
        ])
        for robot in positions:
            for mode in EXPECTED_PROVIDERS:
                with lock:
                    events.clear()
                    positions[robot] = 0.0
                time.sleep(0.2)
                code, result, output = launch_application(robot, mode)
                (evidence / f"{robot}_{mode}.log").write_text(output, encoding="utf-8")
                with lock:
                    observed = list(events)
                expected_events = [] if mode == "detect_people" else [mode]
                passed = (
                    code == 0
                    and result["outcome"] == "application_succeeded"
                    and result["capability"].endswith(
                        {"speak": "/Speak", "detect_people": "/DetectPeople",
                         "move_relative": "/MoveRelative"}[mode]
                    )
                    and result["requested_provider"] == EXPECTED_PROVIDERS[mode]
                    and result["provider"] == EXPECTED_PROVIDERS[mode]
                    and result["robot"] == robot
                    and result["released"] is True
                    and [event[0] for event in observed] == expected_events
                    and all(event[1] == robot for event in observed)
                )
                if mode == "detect_people":
                    passed = passed and len(result["people"]) == 1
                if mode == "move_relative":
                    passed = passed and math.isclose(
                        result["achieved"]["x_m"], 0.1, abs_tol=0.01
                    )
                checks.append({
                    "name": f"{robot}_{mode}", "passed": passed,
                    "result": result, "backend_events": observed,
                    "output_tail": output[-1000:] if not passed else "",
                })
    except Exception as error:
        report["execution_error"] = str(error)
    finally:
        shutdown_ok = executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2)
        if not shutdown_ok or spin_thread.is_alive():
            report["execution_error"] = report.get("execution_error", "") + "; fixture_executor_shutdown_timeout"
        else:
            for timer in timers:
                node.destroy_timer(timer)
            for service in services:
                node.destroy_service(service)
            for publisher in publishers.values():
                node.destroy_publisher(publisher)
            node.destroy_publisher(detections)
            for client in clients:
                node.destroy_client(client)
            node.destroy_node()
            rclpy.shutdown()

    report["fixture_timing"] = {
        name: {"samples": len(samples), "maximum_gap_s": max(
            (after - before for before, after in zip(samples, samples[1:])), default=0.0)}
        for name, samples in ticks.items()
    }
    report["checks"] = checks
    report["passed"] = len(checks) == 8 and "execution_error" not in report and all(
        item["passed"] for item in checks)
    (evidence / "result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

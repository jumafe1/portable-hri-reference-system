#!/usr/bin/env python3
"""Exercise the YASMIN DetectPeople state with a simulated YOLO backend."""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
from datetime import datetime, timezone

import rclpy
from capabilities2_msgs import srv as capabilities_srv
from capabilities2_msgs.msg import CapabilitySpec
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray

from capabilities_yolo_detect_people_probe import (
    CAPABILITY,
    PROVIDER,
    Probe,
    capabilities_server_command,
    detection,
)


APP_SUCCEEDED = "application_succeeded"
APP_NO_PERSON = "application_no_person"


def wait_for(condition, timeout_sec=5.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.05)
    return bool(condition())


def parse_application_result(output):
    for line in reversed(output.splitlines()):
        try:
            return json.loads(line)
        except json.JSONDecodeError:
            continue
    raise RuntimeError(f"Could not find the application JSON result in:\n{output}")


def run_application(publisher, frame, timeout_sec):
    command = [
        "ros2",
        "run",
        "hri_reference_app",
        "people_demo",
        "--robot",
        "nao",
        "--timeout",
        str(timeout_sec),
    ]
    application = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )
    try:
        if not wait_for(lambda: publisher.get_subscription_count() == 1):
            raise RuntimeError("YASMIN did not activate the YOLO provider.")
        deadline = time.monotonic() + timeout_sec + 5.0
        while application.poll() is None and time.monotonic() < deadline:
            if frame is not None:
                frame.header.stamp.sec = int(time.time())
                publisher.publish(frame)
            time.sleep(0.1)
        if application.poll() is None:
            raise RuntimeError("YASMIN application did not finish before its deadline.")
        output, _ = application.communicate(timeout=2.0)
        return application.returncode, parse_application_result(output), output
    finally:
        if application.poll() is None:
            os.killpg(application.pid, signal.SIGINT)
            try:
                application.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(application.pid, signal.SIGKILL)
                application.wait(timeout=5)


def main():
    repository = Path(__file__).resolve().parents[1]
    workspace = repository.parent
    interface_path = (
        workspace
        / "portable-hri-interfaces/hri_capability_interfaces/capability_interfaces/DetectPeople.yaml"
    )
    provider_path = (
        workspace
        / "portable-hri-providers/hri_yolo_providers/capability_providers/YoloDetectPeople.yaml"
    )
    evidence = repository / "results/yasmin_people_demo"
    evidence.mkdir(parents=True, exist_ok=True)
    results = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "checks": [],
        "limits": [
            "The YOLO stream is simulated; no camera or robot is used.",
            "The application monitors one portable capability and does not yet compose Speak or MoveRelative.",
        ],
    }
    if not interface_path.is_file() or not provider_path.is_file():
        raise SystemExit("Run the probe from sibling clones of interfaces and providers.")

    rclpy.init()
    node = Node("yasmin_people_demo_probe")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    publisher = node.create_publisher(DetectionArray, "/yolo/detections", 10)
    probe = Probe(node)

    person_frame = DetectionArray()
    person_frame.header.frame_id = "simulated_camera"
    person_frame.detections = [
        detection("person", 0.91, "person-7", 205.8, 119.7, 228.3, 239.3),
        detection("chair", 0.99, "chair-2", 50.0, 60.0, 40.0, 80.0),
        detection("person", 0.40, "person-low", 10.0, 20.0, 30.0, 40.0),
        detection("person", 0.99, "person-invalid", math.nan, 20.0, 30.0, 40.0),
    ]
    empty_frame = DetectionArray()
    empty_frame.header.frame_id = "simulated_camera"

    with tempfile.TemporaryDirectory(prefix="portable-hri-yasmin-people-") as temporary:
        with (evidence / "server.log").open("w", encoding="utf-8") as server_log:
            server = subprocess.Popen(
                capabilities_server_command(f"{temporary}/capabilities.sqlite3"),
                stdout=server_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
            try:
                for package, capability_type, path in (
                    ("hri_capability_interfaces", "capability_interface", interface_path),
                    ("hri_yolo_providers", "capability_provider", provider_path),
                ):
                    probe.api(
                        capabilities_srv.RegisterCapability,
                        "register_capability",
                        capability_spec=CapabilitySpec(
                            package=package,
                            type=capability_type,
                            content=path.read_text(encoding="utf-8"),
                        ),
                    )

                success_code, success, success_output = run_application(
                    publisher, person_frame, timeout_sec=4.0
                )
                success_cleaned = wait_for(
                    lambda: publisher.get_subscription_count() == 0
                    and not node.get_publishers_info_by_topic("/hri/people")
                    and probe.running() == {}
                )
                results["checks"].append(
                    {
                        "name": "person_found_and_released",
                        "passed": success_code == 0
                        and success["outcome"] == APP_SUCCEEDED
                        and success["provider"] == PROVIDER
                        and success["released"]
                        and len(success["people"]) == 1
                        and success["people"][0]["tracking_id"] == "person-7"
                        and success_cleaned,
                        "application": success,
                        "cleanup": success_cleaned,
                    }
                )

                no_person_code, no_person, no_person_output = run_application(
                    publisher, empty_frame, timeout_sec=1.2
                )
                no_person_cleaned = wait_for(
                    lambda: publisher.get_subscription_count() == 0
                    and not node.get_publishers_info_by_topic("/hri/people")
                    and probe.running() == {}
                )
                results["checks"].append(
                    {
                        "name": "empty_frames_timeout_and_released",
                        "passed": no_person_code == 0
                        and no_person["outcome"] == APP_NO_PERSON
                        and no_person["message"] == "observation_timeout"
                        and not no_person["people"]
                        and no_person["released"]
                        and no_person_cleaned,
                        "application": no_person,
                        "cleanup": no_person_cleaned,
                    }
                )
                results["application_logs"] = {
                    "success": success_output,
                    "no_person": no_person_output,
                }
            except Exception as error:
                results["execution_error"] = str(error)
            finally:
                if server.poll() is None:
                    os.killpg(server.pid, signal.SIGINT)
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(server.pid, signal.SIGKILL)
                    server.wait(timeout=5)

    results["passed"] = (
        bool(results["checks"])
        and "execution_error" not in results
        and all(item["passed"] for item in results["checks"])
    )
    (evidence / "result.json").write_text(
        json.dumps(results, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(results, indent=2, ensure_ascii=False))

    executor.shutdown()
    executor_thread.join(timeout=2)
    node.destroy_node()
    rclpy.shutdown()
    if not results["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

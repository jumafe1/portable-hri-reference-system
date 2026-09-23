#!/usr/bin/env python3
"""Run YoloDetectPeople against a live yolo_ros detection stream."""

import argparse
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
from capabilities2_msgs import srv as capabilities_srv
from capabilities2_msgs.msg import CapabilitySpec
from hri_capability_interfaces.msg import PersonDetection2DArray
from hri_capability_interfaces.srv import DetectPeople
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from capabilities_yolo_detect_people_probe import (
    CAPABILITY,
    PROVIDER,
    Probe,
    capabilities_server_command,
    response_dict,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Detect a visible person through Capabilities2 and live yolo_ros."
    )
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--minimum-confidence", type=float, default=0.5)
    parser.add_argument("--maximum-age", type=float, default=1.0)
    return parser.parse_args()


def wait_for_publishers(node, topic, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if node.count_publishers(topic) > 0:
            return True
        time.sleep(0.1)
    return False


def people_frame_dict(frame):
    return {
        "stamp": {
            "sec": frame.header.stamp.sec,
            "nanosec": frame.header.stamp.nanosec,
        },
        "frame_id": frame.header.frame_id,
        "provider": frame.provider,
        "people": [
            {
                "confidence": person.confidence,
                "tracking_id": person.tracking_id,
                "center_x": person.center_x,
                "center_y": person.center_y,
                "width": person.width,
                "height": person.height,
            }
            for person in frame.people
        ],
    }


def main():
    args = parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        raise SystemExit("--timeout must be greater than zero.")
    if not math.isfinite(args.minimum_confidence) or not (
        0.0 < args.minimum_confidence <= 1.0
    ):
        raise SystemExit("--minimum-confidence must be in the (0.0, 1.0] range.")
    if not math.isfinite(args.maximum_age) or args.maximum_age <= 0.0:
        raise SystemExit("--maximum-age must be greater than zero.")

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
    evidence = repository / "results/yolo_detect_people_physical"
    evidence.mkdir(parents=True, exist_ok=True)
    results = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "provider": PROVIDER,
        "minimum_confidence": args.minimum_confidence,
        "maximum_age_sec": args.maximum_age,
        "timeout_sec": args.timeout,
        "technical_success": False,
        "observer_confirmation": False,
        "released": False,
    }

    if not interface_path.is_file() or not provider_path.is_file():
        raise SystemExit("Clone interfaces, providers and reference-system as siblings.")

    rclpy.init()
    node = Node("yolo_detect_people_physical_probe")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    probe = Probe(node)
    people_lock = threading.Lock()
    latest_people_frame = None

    def people_callback(frame):
        nonlocal latest_people_frame
        with people_lock:
            latest_people_frame = frame

    people_subscription = node.create_subscription(
        PersonDetection2DArray, "/hri/people", people_callback, 10
    )

    if not wait_for_publishers(node, "/yolo/detections", 5.0):
        executor.shutdown()
        executor_thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(
            "/yolo/detections has no publisher. Start the robot driver and yolo_ros first."
        )

    with tempfile.TemporaryDirectory(prefix="portable-hri-yolo-physical-") as temporary:
        with (evidence / "server.log").open("w", encoding="utf-8") as server_log:
            server = subprocess.Popen(
                capabilities_server_command(f"{temporary}/capabilities.sqlite3"),
                stdout=server_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
            bond_id = None
            try:
                probe.api(
                    capabilities_srv.RegisterCapability,
                    "register_capability",
                    capability_spec=CapabilitySpec(
                        package="hri_capability_interfaces",
                        type="capability_interface",
                        content=interface_path.read_text(encoding="utf-8"),
                    ),
                )
                probe.api(
                    capabilities_srv.RegisterCapability,
                    "register_capability",
                    capability_spec=CapabilitySpec(
                        package="hri_yolo_providers",
                        type="capability_provider",
                        content=provider_path.read_text(encoding="utf-8"),
                    ),
                )
                bond_id = probe.api(
                    capabilities_srv.EstablishBond, "establish_bond"
                ).bond_id
                probe.api(
                    capabilities_srv.UseCapability,
                    "use_capability",
                    capability=CAPABILITY,
                    preferred_provider=PROVIDER,
                    bond_id=bond_id,
                )
                if not probe.wait_for_running({CAPABILITY: PROVIDER}):
                    raise RuntimeError("Capabilities2 did not activate YoloDetectPeople.")

                if not wait_for_publishers(node, "/hri/people", 5.0):
                    raise RuntimeError("YoloDetectPeople did not publish /hri/people.")

                deadline = time.monotonic() + args.timeout
                while time.monotonic() < deadline:
                    with people_lock:
                        frame = latest_people_frame
                    if frame is not None:
                        results["last_people_frame"] = people_frame_dict(frame)
                    people = [
                        person
                        for person in frame.people
                        if person.confidence >= args.minimum_confidence
                    ] if frame is not None else []
                    if people:
                        results["technical_success"] = True
                        results["detections"] = [
                            {
                                "confidence": person.confidence,
                                "tracking_id": person.tracking_id,
                                "center_x": person.center_x,
                                "center_y": person.center_y,
                                "width": person.width,
                                "height": person.height,
                            }
                            for person in people
                        ]
                        response = probe.call(
                            DetectPeople,
                            "/hri/detect_people",
                            minimum_confidence=args.minimum_confidence,
                            maximum_age_sec=args.maximum_age,
                        )
                        results["point_query"] = response_dict(response)
                        break
                    time.sleep(0.25)
            except Exception as error:
                results["execution_error"] = str(error)
            finally:
                if bond_id is not None:
                    try:
                        probe.api(
                            capabilities_srv.FreeCapability,
                            "free_capability",
                            capability=CAPABILITY,
                            bond_id=bond_id,
                        )
                        results["released"] = probe.wait_for_running({})
                    except Exception as error:
                        results["cleanup_error"] = str(error)
                if server.poll() is None:
                    os.killpg(server.pid, signal.SIGINT)
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(server.pid, signal.SIGKILL)
                    server.wait(timeout=5)

    if results["technical_success"]:
        confirmation = input(
            "¿Había una persona visible frente a la cámara durante la detección? [s/N]: "
        )
        results["observer_confirmation"] = confirmation.strip().lower() in {
            "s",
            "si",
            "sí",
        }

    results["passed"] = (
        results["technical_success"]
        and results["observer_confirmation"]
        and results["released"]
        and "cleanup_error" not in results
    )
    (evidence / "result.json").write_text(
        json.dumps(results, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(results, indent=2, ensure_ascii=False))

    executor.shutdown()
    executor_thread.join(timeout=2)
    node.destroy_subscription(people_subscription)
    node.destroy_node()
    rclpy.shutdown()
    if not results["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

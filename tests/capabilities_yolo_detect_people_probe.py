#!/usr/bin/env python3
"""Exercise Capabilities2 -> DetectPeople -> a simulated YOLO stream."""

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
from hri_capability_interfaces.msg import PersonDetection2DArray
from hri_capability_interfaces.srv import DetectPeople
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from yolo_msgs.msg import Detection, DetectionArray


CAPABILITY = "hri_capability_interfaces/DetectPeople"
PROVIDER = "hri_yolo_providers/YoloDetectPeople"


class Probe:
    def __init__(self, node: Node):
        self.node = node
        self.checks = []

    def call(self, service_type, name, timeout_sec=10.0, **fields):
        client = self.node.create_client(service_type, name)
        completed = threading.Event()
        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError(f"Service unavailable: {name}")
            future = client.call_async(service_type.Request(**fields))
            future.add_done_callback(lambda _: completed.set())
            if not completed.wait(timeout_sec):
                raise RuntimeError(f"Service timed out: {name}")
            if future.exception() is not None:
                raise RuntimeError(f"Service failed: {name}: {future.exception()}")
            return future.result()
        finally:
            self.node.destroy_client(client)

    def api(self, service_type, name, **fields):
        return self.call(service_type, f"/capabilities/{name}", **fields)

    def check(self, name, passed, **details):
        self.checks.append({"name": name, "passed": bool(passed), **details})

    def running(self):
        response = self.api(
            capabilities_srv.GetRunningCapabilities,
            "get_running_capabilities",
        )
        return {
            item.capability.capability: item.capability.provider
            for item in response.running_capabilities
        }

    def wait_for_running(self, expected, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.running() == expected:
                return True
            time.sleep(0.1)
        return False


def capabilities_server_command(database_file):
    return [
        "ros2",
        "run",
        "capabilities2_server",
        "capabilities2_server_node",
        "--ros-args",
        "-p",
        f"db_file:={database_file}",
    ]


def detection(class_name, score, tracking_id, center_x, center_y, width, height):
    message = Detection()
    message.class_name = class_name
    message.score = score
    message.id = tracking_id
    message.bbox.center.position.x = center_x
    message.bbox.center.position.y = center_y
    message.bbox.size.x = width
    message.bbox.size.y = height
    return message


def response_dict(response):
    return {
        "success": response.success,
        "provider": response.provider,
        "message": response.message,
        "people": [
            {
                "confidence": person.confidence,
                "tracking_id": person.tracking_id,
                "center_x": person.center_x,
                "center_y": person.center_y,
                "width": person.width,
                "height": person.height,
            }
            for person in response.people
        ],
    }


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
    evidence = repository / "results/yolo_detect_people"
    evidence.mkdir(parents=True, exist_ok=True)
    results = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "provider": PROVIDER,
        "checks": [],
        "limits": [
            "The YOLO DetectionArray stream is simulated; no camera or robot is used.",
            "The provider adapts detections but does not start or stop yolo_ros.",
        ],
    }

    if not interface_path.is_file() or not provider_path.is_file():
        raise SystemExit("Run the probe from sibling clones of interfaces and providers.")

    rclpy.init()
    node = Node("yolo_detect_people_capability_probe")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    publisher = node.create_publisher(DetectionArray, "/yolo/detections", 10)
    people_frames = []
    people_lock = threading.Lock()

    def people_callback(frame):
        with people_lock:
            people_frames.append(frame)

    people_subscription = node.create_subscription(
        PersonDetection2DArray, "/hri/people", people_callback, 10
    )

    def people_frame_count():
        with people_lock:
            return len(people_frames)

    def latest_people_frame():
        with people_lock:
            return people_frames[-1] if people_frames else None

    def wait_for_people_frame(after_count, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if people_frame_count() > after_count:
                return latest_people_frame()
            time.sleep(0.05)
        return None

    probe = Probe(node)

    with tempfile.TemporaryDirectory(prefix="portable-hri-yolo-") as temporary:
        with (evidence / "server.log").open("w", encoding="utf-8") as server_log:
            server = subprocess.Popen(
                capabilities_server_command(f"{temporary}/capabilities.sqlite3"),
                stdout=server_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
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

                interfaces = probe.api(
                    capabilities_srv.GetInterfaces, "get_interfaces"
                ).interfaces
                providers = probe.api(
                    capabilities_srv.GetProviders,
                    "get_providers",
                    interface=CAPABILITY,
                ).providers
                probe.check(
                    "catalog",
                    CAPABILITY in interfaces and PROVIDER in providers,
                    interfaces=list(interfaces),
                    providers=list(providers),
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
                probe.check(
                    "provider_selected",
                    probe.wait_for_running({CAPABILITY: PROVIDER}),
                    running=probe.running(),
                )

                deadline = time.monotonic() + 5.0
                while (
                    not node.get_publishers_info_by_topic("/hri/people")
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.05)
                probe.check(
                    "portable_topic_available",
                    len(node.get_publishers_info_by_topic("/hri/people")) == 1,
                    publisher_count=len(node.get_publishers_info_by_topic("/hri/people")),
                )

                unavailable = probe.call(
                    DetectPeople,
                    "/hri/detect_people",
                    minimum_confidence=0.5,
                    maximum_age_sec=1.0,
                )
                probe.check(
                    "detections_unavailable",
                    not unavailable.success
                    and unavailable.message == "detections_unavailable",
                    response=response_dict(unavailable),
                )

                deadline = time.monotonic() + 5.0
                while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
                    time.sleep(0.05)
                probe.check(
                    "backend_subscription",
                    publisher.get_subscription_count() == 1,
                    subscription_count=publisher.get_subscription_count(),
                )

                frame = DetectionArray()
                frame.header.stamp = node.get_clock().now().to_msg()
                frame.header.frame_id = "simulated_camera"
                frame.detections = [
                    detection("person", 0.91, "person-7", 205.8, 119.7, 228.3, 239.3),
                    detection("chair", 0.99, "chair-2", 50.0, 60.0, 40.0, 80.0),
                    detection("person", 0.40, "person-low", 10.0, 20.0, 30.0, 40.0),
                    detection("person", 0.99, "person-invalid", math.nan, 20.0, 30.0, 40.0),
                ]
                previous_people_frame_count = people_frame_count()
                publisher.publish(frame)
                time.sleep(0.25)

                streamed = wait_for_people_frame(previous_people_frame_count)
                streamed_person = streamed.people[0] if streamed and streamed.people else None
                probe.check(
                    "continuous_people_topic",
                    streamed is not None
                    and streamed.header == frame.header
                    and streamed.provider == PROVIDER
                    and len(streamed.people) == 1
                    and streamed_person.tracking_id == "person-7"
                    and abs(streamed_person.confidence - 0.91) < 1e-6,
                    frame=people_frame_dict(streamed) if streamed else None,
                )

                detected = probe.call(
                    DetectPeople,
                    "/hri/detect_people",
                    minimum_confidence=0.5,
                    maximum_age_sec=1.0,
                )
                person = detected.people[0] if detected.people else None
                probe.check(
                    "person_filtered_and_translated",
                    detected.success
                    and detected.message == "people_detected"
                    and len(detected.people) == 1
                    and person.tracking_id == "person-7"
                    and abs(person.confidence - 0.91) < 1e-6
                    and abs(person.center_x - 205.8) < 1e-6
                    and abs(person.height - 239.3) < 1e-6,
                    response=response_dict(detected),
                )

                filtered = probe.call(
                    DetectPeople,
                    "/hri/detect_people",
                    minimum_confidence=0.95,
                    maximum_age_sec=1.0,
                )
                probe.check(
                    "caller_threshold",
                    filtered.success
                    and filtered.message == "no_people"
                    and not filtered.people,
                    response=response_dict(filtered),
                )

                invalid = probe.call(
                    DetectPeople,
                    "/hri/detect_people",
                    minimum_confidence=1.1,
                    maximum_age_sec=1.0,
                )
                probe.check(
                    "invalid_threshold",
                    not invalid.success
                    and invalid.message == "minimum_confidence_out_of_range",
                    response=response_dict(invalid),
                )

                non_finite_threshold = probe.call(
                    DetectPeople,
                    "/hri/detect_people",
                    minimum_confidence=math.nan,
                    maximum_age_sec=1.0,
                )
                probe.check(
                    "non_finite_threshold",
                    not non_finite_threshold.success
                    and non_finite_threshold.message
                    == "request_contains_non_finite_value",
                    response=response_dict(non_finite_threshold),
                )

                non_finite_age = probe.call(
                    DetectPeople,
                    "/hri/detect_people",
                    minimum_confidence=0.5,
                    maximum_age_sec=math.inf,
                )
                probe.check(
                    "non_finite_age",
                    not non_finite_age.success
                    and non_finite_age.message == "request_contains_non_finite_value",
                    response=response_dict(non_finite_age),
                )

                previous_people_frame_count = people_frame_count()
                empty_frame = DetectionArray()
                empty_frame.header.stamp = node.get_clock().now().to_msg()
                empty_frame.header.frame_id = "simulated_camera"
                publisher.publish(empty_frame)
                time.sleep(0.25)
                streamed_empty = wait_for_people_frame(previous_people_frame_count)
                probe.check(
                    "continuous_empty_frame",
                    streamed_empty is not None
                    and streamed_empty.header == empty_frame.header
                    and streamed_empty.provider == PROVIDER
                    and not streamed_empty.people,
                    frame=people_frame_dict(streamed_empty) if streamed_empty else None,
                )
                empty = probe.call(
                    DetectPeople,
                    "/hri/detect_people",
                    minimum_confidence=0.5,
                    maximum_age_sec=1.0,
                )
                probe.check(
                    "valid_empty_frame",
                    empty.success and empty.message == "no_people" and not empty.people,
                    response=response_dict(empty),
                )

                publisher.publish(frame)
                time.sleep(0.25)
                stale = probe.call(
                    DetectPeople,
                    "/hri/detect_people",
                    minimum_confidence=0.5,
                    maximum_age_sec=0.05,
                )
                probe.check(
                    "stale_frame",
                    not stale.success and stale.message == "detections_stale",
                    response=response_dict(stale),
                )

                probe.api(
                    capabilities_srv.FreeCapability,
                    "free_capability",
                    capability=CAPABILITY,
                    bond_id=bond_id,
                )
                freed = probe.wait_for_running({})
                deadline = time.monotonic() + 5.0
                while (
                    (
                        publisher.get_subscription_count() != 0
                        or node.get_publishers_info_by_topic("/hri/people")
                    )
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.05)
                previous_people_frame_count = people_frame_count()
                publisher.publish(frame)
                time.sleep(0.25)
                probe.check(
                    "explicit_free",
                    freed
                    and publisher.get_subscription_count() == 0
                    and not node.get_publishers_info_by_topic("/hri/people")
                    and people_frame_count() == previous_people_frame_count,
                    running=probe.running(),
                    subscription_count=publisher.get_subscription_count(),
                    portable_publisher_count=len(
                        node.get_publishers_info_by_topic("/hri/people")
                    ),
                    portable_frame_count=people_frame_count(),
                )
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

    results["checks"] = probe.checks
    results["passed"] = (
        bool(probe.checks)
        and "execution_error" not in results
        and all(item["passed"] for item in probe.checks)
    )
    (evidence / "result.json").write_text(
        json.dumps(results, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(results, indent=2))

    executor.shutdown()
    executor_thread.join(timeout=2)
    node.destroy_subscription(people_subscription)
    node.destroy_node()
    rclpy.shutdown()
    if not results["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

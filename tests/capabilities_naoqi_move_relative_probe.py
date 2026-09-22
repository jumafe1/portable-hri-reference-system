#!/usr/bin/env python3
"""Exercise Capabilities2 -> MoveRelative against simulated NAOqi services."""

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
from hri_capability_interfaces.srv import MoveRelative
from naoqi_utilities_msgs.srv import MoveTo
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


CAPABILITY = "hri_capability_interfaces/MoveRelative"
PROVIDER = "hri_naoqi_providers/NaoqiMoveRelative"


class Probe:
    def __init__(self, node):
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


def response_dict(response):
    return {
        "success": response.success,
        "provider": response.provider,
        "message": response.message,
        "achieved_x_m": response.achieved_x_m,
        "achieved_y_m": response.achieved_y_m,
        "achieved_theta_rad": response.achieved_theta_rad,
    }


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    repository = Path(__file__).resolve().parents[1]
    workspace = repository.parent
    interface_path = (
        workspace
        / "portable-hri-interfaces/hri_capability_interfaces/capability_interfaces/MoveRelative.yaml"
    )
    provider_path = (
        workspace
        / "portable-hri-providers/hri_naoqi_providers/capability_providers/NaoqiMoveRelative.yaml"
    )
    evidence = repository / "results/naoqi_move_relative"
    evidence.mkdir(parents=True, exist_ok=True)
    results = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "provider": PROVIDER,
        "checks": [],
        "limits": [
            "All robot services and odometry are simulated; no robot moves.",
            "Timeout does not cancel an underlying NAOqi movement.",
        ],
    }

    if not interface_path.is_file() or not provider_path.is_file():
        raise SystemExit("Clone interfaces, providers and reference-system as siblings.")

    rclpy.init()
    node = Node("naoqi_move_relative_capability_probe")
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    probe = Probe(node)

    pose_lock = threading.Lock()
    pose = {"x": 1.0, "y": -0.5, "yaw": 0.4}
    events = []
    publish_odometry = threading.Event()
    slow_move_started = threading.Event()

    odometry_publisher = node.create_publisher(Odometry, "/nao/odom", 10)

    def publish_pose():
        if not publish_odometry.is_set():
            return
        message = Odometry()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        with pose_lock:
            message.pose.pose.position.x = pose["x"]
            message.pose.pose.position.y = pose["y"]
            message.pose.pose.orientation.z = math.sin(pose["yaw"] / 2.0)
            message.pose.pose.orientation.w = math.cos(pose["yaw"] / 2.0)
        odometry_publisher.publish(message)

    odometry_timer = node.create_timer(0.05, publish_pose)

    def fake_move(request, response):
        target = (
            float(request.x_coordinate),
            float(request.y_coordinate),
            float(request.theta_coordinate),
        )
        events.append(("move", target))
        if abs(target[0] - 0.2) < 1e-6:
            slow_move_started.set()
            time.sleep(1.0)
        if abs(target[0] - 0.3) < 1e-6:
            return response
        with pose_lock:
            initial_yaw = pose["yaw"]
            pose["x"] += math.cos(initial_yaw) * target[0] - math.sin(
                initial_yaw
            ) * target[1]
            pose["y"] += math.sin(initial_yaw) * target[0] + math.cos(
                initial_yaw
            ) * target[1]
            pose["yaw"] = normalize_angle(initial_yaw + target[2])
        return response

    move_service = node.create_service(MoveTo, "/nao/move_to", fake_move)

    with tempfile.TemporaryDirectory(prefix="portable-hri-motion-") as temporary:
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
                        package="hri_naoqi_providers",
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

                no_odometry = probe.call(
                    MoveRelative,
                    "/hri/move_relative",
                    x_m=0.1,
                    y_m=0.0,
                    theta_rad=0.0,
                )
                probe.check(
                    "odometry_required",
                    not no_odometry.success
                    and no_odometry.message == "odometry_unavailable_before_motion"
                    and not events,
                    response=response_dict(no_odometry),
                )

                publish_odometry.set()
                time.sleep(0.25)
                events.clear()
                moved = probe.call(
                    MoveRelative,
                    "/hri/move_relative",
                    x_m=0.1,
                    y_m=0.05,
                    theta_rad=0.2,
                )
                probe.check(
                    "motion_verified",
                    moved.success
                    and moved.message == "motion_completed"
                    and moved.provider == PROVIDER
                    and abs(moved.achieved_x_m - 0.1) < 0.01
                    and abs(moved.achieved_y_m - 0.05) < 0.01
                    and abs(moved.achieved_theta_rad - 0.2) < 0.01
                    and len(events) == 1
                    and events[0][0] == "move",
                    response=response_dict(moved),
                    backend_events=list(events),
                )

                backend_count = len(events)
                zero = probe.call(
                    MoveRelative,
                    "/hri/move_relative",
                    x_m=0.0,
                    y_m=0.0,
                    theta_rad=0.0,
                )
                excessive = probe.call(
                    MoveRelative,
                    "/hri/move_relative",
                    x_m=0.6,
                    y_m=0.0,
                    theta_rad=0.0,
                )
                non_finite = probe.call(
                    MoveRelative,
                    "/hri/move_relative",
                    x_m=math.nan,
                    y_m=0.0,
                    theta_rad=0.0,
                )
                probe.check(
                    "invalid_requests_rejected",
                    zero.message == "movement_is_zero"
                    and excessive.message == "movement_exceeds_provider_limits"
                    and non_finite.message == "request_contains_non_finite_value"
                    and len(events) == backend_count,
                    zero=response_dict(zero),
                    excessive=response_dict(excessive),
                    non_finite=response_dict(non_finite),
                )

                events.clear()
                slow_result = {}

                def call_slow_move():
                    slow_result["response"] = probe.call(
                        MoveRelative,
                        "/hri/move_relative",
                        timeout_sec=5.0,
                        x_m=0.2,
                        y_m=0.0,
                        theta_rad=0.0,
                    )

                slow_thread = threading.Thread(target=call_slow_move)
                slow_thread.start()
                if not slow_move_started.wait(3.0):
                    raise RuntimeError("The simulated slow movement did not start.")
                busy = probe.call(
                    MoveRelative,
                    "/hri/move_relative",
                    x_m=0.05,
                    y_m=0.0,
                    theta_rad=0.0,
                )
                slow_thread.join(timeout=5.0)
                slow_response = slow_result.get("response")
                probe.check(
                    "single_flight",
                    not busy.success
                    and busy.message == "provider_busy"
                    and slow_response is not None
                    and slow_response.success,
                    busy=response_dict(busy),
                    first_response=(
                        response_dict(slow_response) if slow_response else None
                    ),
                )

                not_reached = probe.call(
                    MoveRelative,
                    "/hri/move_relative",
                    x_m=0.3,
                    y_m=0.0,
                    theta_rad=0.0,
                )
                probe.check(
                    "target_not_reached",
                    not not_reached.success
                    and not_reached.message == "motion_target_not_reached",
                    response=response_dict(not_reached),
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
                    odometry_publisher.get_subscription_count() != 0
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.05)
                probe.check(
                    "explicit_free",
                    freed and odometry_publisher.get_subscription_count() == 0,
                    running=probe.running(),
                    odometry_subscription_count=(
                        odometry_publisher.get_subscription_count()
                    ),
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
        json.dumps(results, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(results, indent=2, ensure_ascii=False))

    node.destroy_service(move_service)
    node.destroy_timer(odometry_timer)
    executor.shutdown()
    executor_thread.join(timeout=2)
    node.destroy_node()
    rclpy.shutdown()
    if not results["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

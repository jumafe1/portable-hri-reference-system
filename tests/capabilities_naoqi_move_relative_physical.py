#!/usr/bin/env python3
"""Run one supervised MoveRelative request on a real NAO or Pepper."""

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
from hri_capability_interfaces.srv import MoveRelative
from naoqi_utilities_msgs.srv import MoveTo
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from capabilities_naoqi_move_relative_probe import (
    CAPABILITY,
    PROVIDER,
    Probe,
    response_dict,
)


MAXIMUM_X_METRES = 0.5
MAXIMUM_Y_METRES = 0.3
MAXIMUM_THETA_RADIANS = math.pi / 2.0
ROBOT_INTERFACES = {
    "nao": ("/nao/move_to", "/nao/odom"),
    "pepper": ("/pepper/move_to", "/pepper/odom"),
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Request one relative planar movement through Capabilities2."
    )
    parser.add_argument("--robot", choices=sorted(ROBOT_INTERFACES), required=True)
    parser.add_argument("--x", type=float, default=0.1, help="Forward/backward metres.")
    parser.add_argument("--y", type=float, default=0.0, help="Left/right metres.")
    parser.add_argument(
        "--theta", type=float, default=0.0, help="Yaw rotation in radians."
    )
    args = parser.parse_args()
    values = (args.x, args.y, args.theta)
    if not all(math.isfinite(value) for value in values):
        parser.error("--x, --y and --theta must be finite.")
    if all(abs(value) <= 1e-6 for value in values):
        parser.error("At least one movement component must be non-zero.")
    if (
        abs(args.x) > MAXIMUM_X_METRES
        or abs(args.y) > MAXIMUM_Y_METRES
        or abs(args.theta) > MAXIMUM_THETA_RADIANS
    ):
        parser.error("Movement exceeds provider limits: x=0.5, y=0.3, theta=pi/2.")
    return args


def capabilities_server_command(database_file, move_service, odometry_topic):
    command = [
        "ros2",
        "run",
        "capabilities2_server",
        "capabilities2_server_node",
        "--ros-args",
        "-p",
        f"db_file:={database_file}",
    ]
    if move_service != "/nao/move_to":
        command.extend(["-r", f"/nao/move_to:={move_service}"])
    if odometry_topic != "/nao/odom":
        command.extend(["-r", f"/nao/odom:={odometry_topic}"])
    return command


def wait_for_publishers(node, topic, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if node.count_publishers(topic) > 0:
            return True
        time.sleep(0.1)
    return False


def wait_for_subscribers(node, topic, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if node.count_subscribers(topic) > 0:
            return True
        time.sleep(0.1)
    return False


def main():
    args = parse_args()
    move_service, odometry_topic = ROBOT_INTERFACES[args.robot]
    robot_label = args.robot.upper() if args.robot == "nao" else "Pepper"
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
    evidence_root = Path(
        os.environ.get("PORTABLE_HRI_EVIDENCE_DIR", repository / "results")
    )
    evidence = evidence_root / f"{args.robot}_move_relative_physical"
    evidence.mkdir(parents=True, exist_ok=True)
    results = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "robot": robot_label,
        "provider": PROVIDER,
        "move_service": move_service,
        "odometry_topic": odometry_topic,
        "request": {"x_m": args.x, "y_m": args.y, "theta_rad": args.theta},
        "technical_success": False,
        "observer_confirmation": False,
        "released": False,
    }

    if not interface_path.is_file() or not provider_path.is_file():
        raise SystemExit("Clone interfaces, providers and reference-system as siblings.")

    rclpy.init()
    node = Node(f"{args.robot}_move_relative_physical_probe")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    probe = Probe(node)

    move_client = node.create_client(MoveTo, move_service)
    try:
        if not move_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                f"{move_service} is unavailable. Start and verify the "
                f"{robot_label} driver first."
            )
    finally:
        node.destroy_client(move_client)
    if not wait_for_publishers(node, odometry_topic, 5.0):
        raise RuntimeError(
            f"{odometry_topic} has no publisher. Verify the {robot_label} driver."
        )

    print(
        f"\nADVERTENCIA: {robot_label} ejecutará x={args.x:.3f} m, "
        f"y={args.y:.3f} m, theta={args.theta:.3f} rad.\n"
        "Deja espacio libre, supervisa el robot y mantén acceso al botón físico.\n"
        "Esta prueba no modifica la vida autónoma y no dispone de cancelación remota."
    )
    confirmation = input('Escribe "MOVER" para continuar: ')
    if confirmation.strip() != "MOVER":
        executor.shutdown()
        executor_thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit("Prueba cancelada antes de enviar movimiento.")

    with tempfile.TemporaryDirectory(prefix="portable-hri-motion-physical-") as temporary:
        with (evidence / "server.log").open("w", encoding="utf-8") as server_log:
            server = subprocess.Popen(
                capabilities_server_command(
                    f"{temporary}/capabilities.sqlite3",
                    move_service,
                    odometry_topic,
                ),
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
                    raise RuntimeError("Capabilities2 did not activate NaoqiMoveRelative.")
                if not wait_for_subscribers(node, odometry_topic, 5.0):
                    raise RuntimeError(
                        f"NaoqiMoveRelative did not subscribe to {odometry_topic}."
                    )
                time.sleep(0.5)

                response = probe.call(
                    MoveRelative,
                    "/hri/move_relative",
                    timeout_sec=40.0,
                    x_m=args.x,
                    y_m=args.y,
                    theta_rad=args.theta,
                )
                results["response"] = response_dict(response)
                results["technical_success"] = bool(
                    response.success and response.provider == PROVIDER
                )
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
        observed = input(
            f"¿{robot_label} ejecutó el desplazamiento solicitado y se detuvo? [s/N]: "
        )
        results["observer_confirmation"] = observed.strip().lower() in {
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
    node.destroy_node()
    rclpy.shutdown()
    if not results["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Run the real Capabilities2 -> /hri/speak -> /nao/say validation."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading

import rclpy
from capabilities2_msgs import srv as capabilities_srv
from capabilities2_msgs.msg import CapabilitySpec
from hri_capability_interfaces.srv import Speak
from naoqi_utilities_msgs.srv import Say
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from capabilities_nao_speak_probe import CAPABILITY, PROVIDER, Probe


def parse_args():
    parser = argparse.ArgumentParser(
        description="Speak once through the real NAO provider managed by Capabilities2."
    )
    parser.add_argument(
        "--text",
        default="Hola, esta es una prueba con Capabilities2.",
        help="Short phrase for the supervised physical validation.",
    )
    parser.add_argument("--language", default="Spanish")
    return parser.parse_args()


def main():
    args = parse_args()
    repository = Path(__file__).resolve().parents[1]
    workspace = repository.parent
    interface_path = (
        workspace
        / "portable-hri-interfaces/hri_capability_interfaces/capability_interfaces/Speak.yaml"
    )
    provider_path = (
        workspace
        / "portable-hri-providers/hri_naoqi_providers/capability_providers/NaoSpeak.yaml"
    )
    evidence = repository / "results/nao_speak_physical"
    evidence.mkdir(parents=True, exist_ok=True)
    results = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "text": args.text,
        "language": args.language,
        "technical_success": False,
        "audible_confirmation": False,
    }

    if not args.text.strip():
        raise SystemExit("The physical test phrase cannot be empty.")
    if not interface_path.is_file() or not provider_path.is_file():
        raise SystemExit("Clone interfaces, providers and reference-system as siblings.")

    rclpy.init()
    node = Node("nao_speak_physical_probe")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    probe = Probe(node)
    backend_client = node.create_client(Say, "/nao/say")

    try:
        if not backend_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                "/nao/say is unavailable. Start and verify the NAO driver first."
            )
    finally:
        node.destroy_client(backend_client)

    with tempfile.TemporaryDirectory(prefix="portable-hri-physical-") as temporary:
        with (evidence / "server.log").open("w", encoding="utf-8") as server_log:
            server = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "capabilities2_server",
                    "capabilities2_server_node",
                    "--ros-args",
                    "-p",
                    f"db_file:={temporary}/capabilities.sqlite3",
                ],
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
                    raise RuntimeError("Capabilities2 did not activate NaoSpeak.")

                response = probe.call(
                    Speak,
                    "/hri/speak",
                    text=args.text,
                    language=args.language,
                )
                results["response"] = {
                    "success": response.success,
                    "provider": response.provider,
                    "message": response.message,
                }
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
        confirmation = input("¿Escuchaste al NAO pronunciar la frase? [s/N]: ")
        results["audible_confirmation"] = confirmation.strip().lower() in {"s", "si", "sí"}

    results["passed"] = (
        results["technical_success"]
        and results["audible_confirmation"]
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

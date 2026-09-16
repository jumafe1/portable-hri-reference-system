#!/usr/bin/env python3
"""Exercise Capabilities2 -> portable Speak -> simulated /nao/say."""

import json
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
from hri_capability_interfaces.srv import Speak
from naoqi_utilities_msgs.srv import Say
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


CAPABILITY = "hri_capability_interfaces/Speak"
PROVIDER = "hri_naoqi_providers/NaoSpeak"
SLOW_BACKEND_DELAY_SEC = 5.5


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


def main():
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
    evidence = repository / "results/nao_speak"
    evidence.mkdir(parents=True, exist_ok=True)
    results = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "checks": [],
        "limits": [
            "The /nao/say backend is simulated; no robot or audio is used.",
            "Capabilities2 v0.1.0 retains its known missing-provider reporting defect.",
        ],
    }

    if not interface_path.is_file() or not provider_path.is_file():
        raise SystemExit("Run the probe from sibling clones of interfaces and providers.")

    rclpy.init()
    node = Node("nao_speak_capability_probe")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    requests = []

    def fake_say(request, response):
        requests.append(
            {
                "text": request.text,
                "language": request.language,
                "animated": request.animated,
                "asynchronous": request.asynchronous,
            }
        )
        if request.text == "__backend_timeout__":
            time.sleep(SLOW_BACKEND_DELAY_SEC)
        response.success = request.text != "__backend_failure__"
        response.message = "spoken" if response.success else "backend_rejected"
        return response

    backend_service = node.create_service(Say, "/nao/say", fake_say)
    probe = Probe(node)

    with tempfile.TemporaryDirectory(prefix="portable-hri-capabilities-") as temporary:
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
            try:
                interface_content = interface_path.read_text(encoding="utf-8")
                provider_content = provider_path.read_text(encoding="utf-8")
                probe.api(
                    capabilities_srv.RegisterCapability,
                    "register_capability",
                    capability_spec=CapabilitySpec(
                        package="hri_capability_interfaces",
                        type="capability_interface",
                        content=interface_content,
                    ),
                )
                probe.api(
                    capabilities_srv.RegisterCapability,
                    "register_capability",
                    capability_spec=CapabilitySpec(
                        package="hri_naoqi_providers",
                        type="capability_provider",
                        content=provider_content,
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

                spoken = probe.call(
                    Speak,
                    "/hri/speak",
                    text="Hola desde Capabilities2",
                    language="",
                )
                expected_request = {
                    "text": "Hola desde Capabilities2",
                    "language": "Spanish",
                    "animated": False,
                    "asynchronous": False,
                }
                probe.check(
                    "speak_translation",
                    spoken.success
                    and spoken.provider == PROVIDER
                    and requests[-1] == expected_request,
                    response={
                        "success": spoken.success,
                        "provider": spoken.provider,
                        "message": spoken.message,
                    },
                    backend_request=requests[-1],
                )

                timed_out = probe.call(
                    Speak,
                    "/hri/speak",
                    text="__backend_timeout__",
                    language="Spanish",
                )
                probe.check(
                    "backend_timeout",
                    not timed_out.success
                    and timed_out.provider == PROVIDER
                    and timed_out.message == "nao_say_timeout",
                    response={
                        "success": timed_out.success,
                        "provider": timed_out.provider,
                        "message": timed_out.message,
                    },
                )

                recovered = probe.call(
                    Speak,
                    "/hri/speak",
                    text="Solicitud posterior al timeout",
                    language="Spanish",
                )
                probe.check(
                    "request_after_timeout",
                    recovered.success
                    and recovered.provider == PROVIDER
                    and recovered.message == "spoken"
                    and requests[-1]["text"] == "Solicitud posterior al timeout",
                    response={
                        "success": recovered.success,
                        "provider": recovered.provider,
                        "message": recovered.message,
                    },
                    backend_request=requests[-1],
                )

                rejected = probe.call(
                    Speak,
                    "/hri/speak",
                    text="__backend_failure__",
                    language="Spanish",
                )
                probe.check(
                    "backend_error_propagated",
                    not rejected.success
                    and rejected.provider == PROVIDER
                    and rejected.message == "backend_rejected",
                    message=rejected.message,
                )

                probe.api(
                    capabilities_srv.FreeCapability,
                    "free_capability",
                    capability=CAPABILITY,
                    bond_id=bond_id,
                )
                probe.check(
                    "explicit_free",
                    probe.wait_for_running({}),
                    running=probe.running(),
                )

                node.destroy_service(backend_service)
                backend_service = None
                time.sleep(1.0)
                second_bond = probe.api(
                    capabilities_srv.EstablishBond, "establish_bond"
                ).bond_id
                probe.api(
                    capabilities_srv.UseCapability,
                    "use_capability",
                    capability=CAPABILITY,
                    preferred_provider=PROVIDER,
                    bond_id=second_bond,
                )
                unavailable = probe.call(
                    Speak,
                    "/hri/speak",
                    text="Backend ausente",
                    language="Spanish",
                )
                probe.check(
                    "backend_unavailable",
                    not unavailable.success
                    and unavailable.provider == PROVIDER
                    and unavailable.message == "nao_say_unavailable",
                    message=unavailable.message,
                )
                probe.api(
                    capabilities_srv.FreeCapability,
                    "free_capability",
                    capability=CAPABILITY,
                    bond_id=second_bond,
                )
                probe.check("final_free", probe.wait_for_running({}))
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
    node.destroy_node()
    rclpy.shutdown()
    if not results["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

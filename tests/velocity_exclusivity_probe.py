#!/usr/bin/env python3
"""Software-only provider graph/source exclusivity tests; no robot driver allowed."""

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
from bondpy import bondpy
from capabilities2_msgs import srv
from capabilities2_msgs.msg import CapabilitySpec
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from capabilities_naoqi_move_relative_probe import Probe, capabilities_server_command
from capabilities_naoqi_move_velocity_probe import (
    CAPABILITY, PROVIDER, Recorder, await_zero, publish_for, software_guard,
    twist, wait_until, zero,
)


def main():
    software_guard()
    siblings = Path(__file__).resolve().parents[2]
    evidence = Path(tempfile.mkdtemp(prefix="portable-hri-velocity-exclusivity-"))
    report = {"started_utc": datetime.now(timezone.utc).isoformat(), "checks": [],
              "evidence_directory": str(evidence),
              "limits": ["Software-only: no physical stop verified.",
                         "DDS graph discovery is asynchronous, not a distributed lock."]}
    rclpy.init()
    node = Node("software_velocity_exclusivity_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    recorder = Recorder()
    probe = Probe(node)
    server = heartbeat = publisher = None
    log = (evidence / "server.log").open("w", encoding="utf-8")
    try:
        time.sleep(1)
        if set(node.get_node_names()) - {node.get_name()}:
            raise RuntimeError("Domain is not isolated; refusing to run.")
        publisher = node.create_publisher(Twist, "/hri/cmd_vel", 1)
        node.create_subscription(Twist, "/nao/cmd_vel", recorder.command, 100)
        node.create_subscription(String, "/hri/move_velocity/status", recorder.status, 100)
        server = subprocess.Popen(capabilities_server_command(str(evidence / "capabilities.sqlite3")),
                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        for package, folder, kind, filename in (
            ("hri_capability_interfaces", "portable-hri-interfaces", "capability_interface", "MoveVelocity"),
            ("hri_naoqi_providers", "portable-hri-providers", "capability_provider", "NaoqiMoveVelocity"),
        ):
            path = siblings / folder / package / f"{kind}s" / f"{filename}.yaml"
            probe.api(srv.RegisterCapability, "register_capability", capability_spec=CapabilitySpec(
                package=package, type=kind, content=path.read_text(encoding="utf-8")))
        bond = probe.api(srv.EstablishBond, "establish_bond").bond_id
        heartbeat = bondpy.Bond(node, "/capabilities/bonds", bond)
        heartbeat.start()
        if not heartbeat.wait_until_formed(Duration(seconds=5)):
            raise RuntimeError("Bond failed to form.")

        acquisitions = 0

        def use():
            nonlocal acquisitions
            acquisitions += 1
            start = time.monotonic()
            probe.api(srv.UseCapability, "use_capability", capability=CAPABILITY,
                      preferred_provider=PROVIDER, bond_id=bond)
            if not await_zero(recorder, start, "ready", timeout=4):
                raise RuntimeError("Provider did not become ready.")
            start = publish_for(publisher, twist(), 0.7)
            samples = recorder.since(start)
            accepted = await_zero(recorder, start, "stopped") and bool(samples) and all(
                zero(values) for _, values in samples)
            report["checks"].append({"name": f"original_source_zero_accepted_{acquisitions}",
                                     "passed": accepted})
            if not accepted:
                raise RuntimeError("Provider did not acknowledge original-source startup zero.")
            start = publish_for(publisher, twist(0.03), 0.5)
            accepted = any(not zero(values) for _, values in recorder.since(start))
            report["checks"].append({"name": f"original_source_motion_accepted_{acquisitions}",
                                     "passed": accepted})
            if not accepted:
                raise RuntimeError("Provider did not move fake backend before fault injection.")

        def free():
            probe.api(srv.FreeCapability, "free_capability", capability=CAPABILITY, bond_id=bond)
            if not wait_until(lambda: node.count_publishers("/nao/cmd_vel") == 0):
                raise RuntimeError("Provider did not leave graph after release.")

        for topic, name in (("/hri/cmd_vel", "duplicate_input_latches_zero"),
                            ("/nao/cmd_vel", "duplicate_backend_latches_zero")):
            use()
            start = time.monotonic()
            extra = node.create_publisher(Twist, topic, 1)
            try:
                fault = await_zero(recorder, start, "multiple_velocity_publishers")
            finally:
                node.destroy_publisher(extra)
            # Removing the competing endpoint must not restart motion.
            continued = publish_for(publisher, twist(0.03), 0.8)
            samples = recorder.since(continued)
            report["checks"].append({"name": name, "passed": fault and bool(samples)
                                     and all(zero(values) for _, values in samples)
                                     and recorder.saw_status("multiple_velocity_publishers", continued)})
            free()

        use()
        start = time.monotonic()
        node.destroy_publisher(publisher)
        publisher = None
        absent = await_zero(recorder, start, "awaiting_velocity_source")
        publisher = node.create_publisher(Twist, "/hri/cmd_vel", 1)
        start = publish_for(publisher, twist(0.03), 1)
        samples = recorder.since(start)
        report["checks"].append({"name": "replacement_source_gid_rejected", "passed": absent
                                 and recorder.saw_status("velocity_source_changed", start)
                                 and bool(samples) and all(zero(values) for _, values in samples)})
        free()
    except Exception as error:
        report["execution_error"] = str(error)
    finally:
        if heartbeat is not None:
            heartbeat.shutdown()
        if server is not None and server.poll() is None:
            os.killpg(server.pid, signal.SIGINT)
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(server.pid, signal.SIGKILL)
                server.wait(timeout=5)
        executor.shutdown()
        thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
    with recorder.lock:
        report["observed_statuses"] = list(recorder.statuses)
    report["passed"] = len(report["checks"]) == 9 and "execution_error" not in report and all(
        check["passed"] for check in report["checks"])
    (evidence / "result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

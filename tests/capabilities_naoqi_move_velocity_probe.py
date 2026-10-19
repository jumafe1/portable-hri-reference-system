#!/usr/bin/env python3
"""Software-only MoveVelocity regression; never run alongside a robot driver.

Run in a fresh domain, e.g. ROS_DOMAIN_ID=171 ROS_LOCALHOST_ONLY=1 python3
tests/capabilities_naoqi_move_velocity_probe.py --software-only. Evidence is
written to a new temporary directory, never over existing physical results.
"""

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
from bondpy import bondpy
from capabilities2_msgs import srv as capabilities_srv
from capabilities2_msgs.msg import CapabilitySpec
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from capabilities_naoqi_move_relative_probe import Probe, capabilities_server_command


CAPABILITY = "hri_capability_interfaces/MoveVelocity"
PROVIDER = "hri_naoqi_providers/NaoqiMoveVelocity"
RELATIVE = "hri_capability_interfaces/MoveRelative"
RELATIVE_PROVIDER = "hri_naoqi_providers/NaoqiMoveRelative"


def software_guard():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--software-only", action="store_true", required=True)
    parser.parse_args()
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if not domain.isdigit() or not 100 <= int(domain) <= 200:
        raise SystemExit("Use a dedicated ROS_DOMAIN_ID between 100 and 200.")
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("Set ROS_LOCALHOST_ONLY=1; no physical hardware allowed.")


def wait_until(predicate, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return bool(predicate())


def twist(vx=0.0, vy=0.0, wz=0.0):
    message = Twist()
    message.linear.x, message.linear.y, message.angular.z = vx, vy, wz
    return message


def components(message):
    return [message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z]


def zero(values):
    return all(math.isfinite(value) and abs(value) < 1e-9 for value in values)


class Recorder:
    def __init__(self):
        self.lock = threading.Lock()
        self.commands = []
        self.statuses = []

    def command(self, message):
        with self.lock:
            self.commands.append((time.monotonic(), components(message)))

    def status(self, message):
        with self.lock:
            self.statuses.append((time.monotonic(), message.data))

    def since(self, start):
        with self.lock:
            return [(stamp, values) for stamp, values in self.commands if stamp >= start]

    def saw_status(self, name, start):
        with self.lock:
            return any(stamp >= start and value == name for stamp, value in self.statuses)


def publish_for(publisher, message, duration=0.6):
    start = time.monotonic()
    while time.monotonic() - start < duration:
        publisher.publish(message)
        time.sleep(0.05)
    return start


def await_zero(recorder, start, status, timeout=2.0):
    def confirmed():
        samples = recorder.since(start)
        return bool(samples) and zero(samples[-1][1]) and recorder.saw_status(status, start)
    return wait_until(confirmed, timeout)


def main():
    software_guard()
    repository = Path(__file__).resolve().parents[1]
    siblings = repository.parent
    evidence = Path(tempfile.mkdtemp(prefix="portable-hri-velocity-probe-"))
    report = {"started_utc": datetime.now(timezone.utc).isoformat(),
              "evidence_directory": str(evidence), "checks": [],
              "limits": ["Software-only; no physical movement or stop is proven.",
                         "Exclusivity applies only to cooperating providers on this host."]}
    rclpy.init()
    node = Node("software_move_velocity_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    probe = Probe(node)
    recorder = Recorder()
    server = None
    backend = None
    publisher = None
    server_log = None
    heartbeat = None
    try:
        time.sleep(1.0)
        strangers = set(node.get_node_names()) - {node.get_name()}
        if strangers:
            raise RuntimeError(f"Domain is not isolated; refusing to run: {strangers}")
        publisher = node.create_publisher(Twist, "/hri/cmd_vel", 1)
        node.create_subscription(String, "/hri/move_velocity/status", recorder.status, 10)
        server_log = (evidence / "server.log").open("w", encoding="utf-8")
        server = subprocess.Popen(
            capabilities_server_command(str(evidence / "capabilities.sqlite3")),
            stdout=server_log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        for name in ("MoveVelocity", "MoveRelative"):
            for package, folder, spec_type, filename in (
                ("hri_capability_interfaces", "portable-hri-interfaces", "capability_interface", name),
                ("hri_naoqi_providers", "portable-hri-providers", "capability_provider", f"Naoqi{name}"),
            ):
                path = siblings / folder / package / f"{spec_type}s" / f"{filename}.yaml"
                probe.api(capabilities_srv.RegisterCapability, "register_capability",
                          capability_spec=CapabilitySpec(package=package, type=spec_type,
                                                         content=path.read_text(encoding="utf-8")))
        bond = probe.api(capabilities_srv.EstablishBond, "establish_bond").bond_id
        heartbeat = bondpy.Bond(node, "/capabilities/bonds", bond)
        heartbeat.start()
        if not heartbeat.wait_until_formed(Duration(seconds=5)):
            raise RuntimeError("Capabilities2 bond heartbeat did not form.")

        def use(capability=CAPABILITY, provider=PROVIDER):
            return probe.api(capabilities_srv.UseCapability, "use_capability",
                             capability=capability, preferred_provider=provider, bond_id=bond)

        def free(capability=CAPABILITY):
            probe.api(capabilities_srv.FreeCapability, "free_capability",
                      capability=capability, bond_id=bond)

        def denied_use(capability, provider):
            try:
                use(capability, provider)
            except RuntimeError:
                pass  # Upstream may report rejection as a service failure.
            time.sleep(0.25)
            observed = probe.running()
            services = dict(node.get_service_names_and_types())
            if capability == RELATIVE:
                endpoint_absent = "/hri/move_relative" not in services
                start = publish_for(publisher, twist(0.03), 0.3)
                owner_active = any(not zero(values) for _, values in recorder.since(start))
                publish_for(publisher, twist(), 0.15)
            else:
                endpoint_absent = (publisher.get_subscription_count() == 0
                                   and node.count_publishers("/hri/move_velocity/status") == 0)
                owner_active = "/hri/move_relative" in services
            if capability in observed:
                report.setdefault("known_upstream_limitations", []).append({
                    "name": "upstream_failed_activation_remains_in_catalog",
                    "capability": capability, "reported_running": observed,
                    "operational_endpoint_absent": endpoint_absent,
                    "description": "RunnerCache inserts the plugin before start; a thrown start leaves a phantom running entry.",
                })
            # UseCapability binds before start. FreeCapability removes that bond
            # and the failed runner; its presence in the catalog is NOT success.
            free(capability)
            expected_owner = ({CAPABILITY: PROVIDER} if capability == RELATIVE
                              else {RELATIVE: RELATIVE_PROVIDER})
            cleaned = probe.wait_for_running(expected_owner)
            return server.poll() is None and endpoint_absent and owner_active and cleaned

        use()
        probe.check("registration_and_acquisition", probe.wait_for_running({CAPABILITY: PROVIDER}))
        if not wait_until(lambda: publisher.get_subscription_count() > 0):
            raise RuntimeError("Provider did not subscribe to the common velocity topic.")
        # The provider first settles publisher discovery. Test missing backend
        # rejection only once that independent precondition has completed.
        time.sleep(0.6)
        baseline = time.monotonic()
        if not wait_until(lambda: recorder.saw_status("backend_unavailable", baseline)):
            raise RuntimeError("Provider did not report the absent backend after graph settling.")
        start = publish_for(publisher, twist(0.04))
        probe.check("backend_absent_rejects_command",
                    recorder.saw_status("backend_unavailable", start))
        backend = node.create_subscription(Twist, "/nao/cmd_vel", recorder.command, 100)
        start = time.monotonic()
        ready = await_zero(recorder, start, "ready")
        samples = recorder.since(start)
        probe.check("backend_appearance_does_not_replay_command",
                    ready and all(zero(values) for _, values in samples))

        start = publish_for(publisher, twist(0.04, -0.02, 0.08), duration=0.9)
        samples = recorder.since(start)
        nonzero = [values for _, values in samples if not zero(values)]
        ramp_bounded = all(abs(second[axis] - first[axis]) <= step + 1e-5
                           for (_, first), (_, second) in zip(samples, samples[1:])
                           for axis, step in ((0, 0.005), (1, 0.005), (5, 0.020)))
        probe.check("directions_limits_and_ramp", bool(nonzero)
                    and ramp_bounded
                    and any(abs(values[0] - 0.04) < 0.002 for values in nonzero)
                    and any(0 < values[0] < 0.035 for values in nonzero)
                    and all(0 <= values[0] <= 0.040001 and -0.020001 <= values[1] <= 0
                            and 0 <= values[5] <= 0.080001
                            and values[2:5] == [0.0, 0.0, 0.0] for values in nonzero),
                    first_samples=samples[:8], last_samples=samples[-3:])
        stopped_at = time.monotonic()
        timed_out = wait_until(lambda: recorder.saw_status("command_timeout", stopped_at), 1.0)
        time.sleep(0.1)
        after = recorder.since(stopped_at + 0.45)
        if not after:
            time.sleep(0.35)
            after = recorder.since(stopped_at + 0.45)
        probe.check("input_loss_stops", timed_out and bool(after)
                    and all(zero(values) for _, values in after), samples=after[:3])

        invalid = [("nan", twist(math.nan)), ("infinity", twist(wz=math.inf)),
                   ("x_limit", twist(0.101)), ("y_limit", twist(vy=0.051)),
                   ("yaw_limit", twist(wz=0.201))]
        for name in ("linear.z", "angular.x", "angular.y"):
            message = twist()
            part, field = name.split(".")
            setattr(getattr(message, part), field, 0.01)
            invalid.append((name, message))
        for name, message in invalid:
            publish_for(publisher, twist(0.02), 0.3)
            start = publish_for(publisher, message, 0.2)
            stopped = await_zero(recorder, start, "invalid_command")
            settled = time.monotonic()
            time.sleep(0.15)
            samples = recorder.since(settled)
            probe.check(f"invalid_{name}_stops", stopped and bool(samples)
                        and all(zero(values) for _, values in samples))

        start = publish_for(publisher, twist(-0.03, 0.01, -0.06), 0.5)
        samples = recorder.since(start)
        probe.check("reverse_directions", bool(samples) and samples[-1][1][0] < 0
                    and samples[-1][1][1] > 0 and samples[-1][1][5] < 0)
        start = publish_for(publisher, twist(), 0.15)
        stopped = await_zero(recorder, start, "stopped")
        settled = time.monotonic()
        time.sleep(0.15)
        samples = recorder.since(settled)
        probe.check("explicit_zero_stops", stopped and bool(samples)
                    and all(zero(values) for _, values in samples))

        publish_for(publisher, twist(0.03), 0.4)
        node.destroy_subscription(backend)
        start = time.monotonic()
        disappeared = wait_until(lambda: recorder.saw_status("backend_unavailable", start))
        backend = node.create_subscription(Twist, "/nao/cmd_vel", recorder.command, 100)
        start = time.monotonic()
        ready = await_zero(recorder, start, "ready")
        samples = recorder.since(start)
        probe.check("backend_disconnection_invalidates_command", disappeared and ready and bool(samples)
                    and all(zero(values) for _, values in samples))
        probe.check("velocity_blocks_relative_activation",
                    denied_use(RELATIVE, RELATIVE_PROVIDER), running=probe.running())

        publish_for(publisher, twist(0.03), 0.5)
        start = time.monotonic()
        free()
        released = probe.wait_for_running({})
        wait_until(lambda: publisher.get_subscription_count() == 0)
        samples = recorder.since(start)
        probe.check("release_sends_zero", released and bool(samples)
                    and zero(samples[-1][1]), samples=samples[-5:])
        start = time.monotonic()
        use()
        ready = await_zero(recorder, start, "ready")
        samples = recorder.since(start)
        probe.check("reacquire_has_no_stale_command", ready and bool(samples)
                    and all(zero(values) for _, values in samples))
        free()
        if not probe.wait_for_running({}):
            raise RuntimeError("Velocity provider did not release before exclusivity test.")
        use(RELATIVE, RELATIVE_PROVIDER)
        relative_active = probe.wait_for_running({RELATIVE: RELATIVE_PROVIDER})
        probe.check("relative_blocks_velocity_activation", relative_active
                    and denied_use(CAPABILITY, PROVIDER), running=probe.running())
        free(RELATIVE)
        probe.check("relative_release", probe.wait_for_running({}))
        use()
        probe.check("velocity_after_relative_release", probe.wait_for_running({CAPABILITY: PROVIDER}))
        free()
    except Exception as error:
        report["execution_error"] = str(error)
    finally:
        if publisher is not None:
            publish_for(publisher, twist(), 0.15)
        if heartbeat is not None:
            heartbeat.shutdown()
        if server is not None:
            if server.poll() is None:
                os.killpg(server.pid, signal.SIGINT)
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(server.pid, signal.SIGKILL)
                server.wait(timeout=5)
        if server_log is not None:
            server_log.close()
        executor.shutdown()
        spin_thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
    report["checks"] = probe.checks
    report["passed"] = bool(probe.checks) and "execution_error" not in report and all(
        check["passed"] for check in probe.checks)
    (evidence / "result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

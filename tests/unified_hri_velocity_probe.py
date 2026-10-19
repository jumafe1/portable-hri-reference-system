#!/usr/bin/env python3
"""Verify the unified velocity mode with fake NAO/Pepper cmd_vel subscribers.

Requires --software-only, ROS_LOCALHOST_ONLY=1 and an unused ROS_DOMAIN_ID
between 100 and 200. Does not contact or move physical hardware.
"""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from capabilities_naoqi_move_velocity_probe import Recorder, software_guard, wait_until, zero


PROVIDER = "hri_naoqi_providers/NaoqiMoveVelocity"


def parse_result(output):
    for line in reversed(output.splitlines()):
        marker = line.find('{"outcome":')
        if marker >= 0:
            return json.JSONDecoder().raw_decode(line[marker:])[0]
    raise RuntimeError(f"Application result is missing:\n{output[-3000:]}")


def launch(arguments, timeout=20):
    command = ["ros2", "launch", "hri_reference_app", "hri_app.launch.py",
               "mode:=move_velocity", *arguments]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, start_new_session=True)
    try:
        output, _ = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired as error:
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5)
        raise RuntimeError(f"Application timed out: {output[-2500:]}") from error
    return process.returncode, output


def interrupt_during_motion(recorder, evidence, target, sig):
    """Signal only the selected child, not the provider and application together."""
    path = evidence / f"interrupt_{target}.log"
    command = ["ros2", "launch", "hri_reference_app", "hri_app.launch.py",
               "mode:=move_velocity", "robot:=nao", "vx:=0.04", "duration:=5",
               "confirm:=MOVER"]
    start = time.monotonic()
    with path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   text=True, start_new_session=True)
        try:
            if not wait_until(lambda: any(not zero(values) for _, values in recorder.since(start)), 10):
                raise RuntimeError("No velocity output before interruption test.")
            output = path.read_text(encoding="utf-8")
            matched = re.search(r"\[" + re.escape(target) + r"-\d+\]: process started with pid \[(\d+)\]", output)
            if not matched:
                raise RuntimeError(f"Cannot resolve child {target} from launch log.")
            os.kill(int(matched.group(1)), sig)
            process.wait(timeout=12)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
    output = path.read_text(encoding="utf-8")
    return parse_result(output), recorder.since(start)


def main():
    software_guard()
    evidence = Path(tempfile.mkdtemp(prefix="portable-hri-app-velocity-probe-"))
    report = {"started_utc": datetime.now(timezone.utc).isoformat(), "checks": [],
              "evidence_directory": str(evidence),
              "limits": ["Fake cmd_vel subscribers; no physical movement or stop is verified."]}
    rclpy.init()
    node = Node("software_unified_velocity_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    recorders = {robot: Recorder() for robot in ("nao", "pepper")}
    statuses = Recorder()
    node.create_subscription(String, "/hri/move_velocity/status", statuses.status, 100)
    subscriptions = {}
    try:
        time.sleep(1.0)
        strangers = set(node.get_node_names()) - {node.get_name()}
        if strangers:
            raise RuntimeError(f"Domain is not isolated; refusing to run: {strangers}")
        for robot, recorder in recorders.items():
            subscriptions[robot] = node.create_subscription(
                Twist, f"/{robot}/cmd_vel", recorder.command, 100)

        rejected = [
            ("requires_confirmation", ["vx:=0.04", "duration:=1.2"]),
            ("rejects_long_duration", ["vx:=0.04", "duration:=5.1", "confirm:=MOVER"]),
            ("rejects_zero_duration", ["vx:=0.04", "duration:=0", "confirm:=MOVER"]),
            ("rejects_x_limit", ["vx:=0.11", "duration:=1.2", "confirm:=MOVER"]),
            ("rejects_nan", ["vx:=nan", "duration:=1.2", "confirm:=MOVER"]),
            ("rejects_zero_velocity", ["vx:=0", "vy:=0", "wz:=0", "confirm:=MOVER"]),
        ]
        for name, arguments in rejected:
            start = time.monotonic()
            code, output = launch(["robot:=nao", *arguments], timeout=10)
            no_output = all(not recorder.since(start) for recorder in recorders.values())
            report["checks"].append({"name": name, "passed": code != 0
                                     and "process started" not in output and no_output,
                                     "output_tail": output[-1200:]})
            (evidence / f"{name}.log").write_text(output, encoding="utf-8")

        for robot in recorders:
            start = time.monotonic()
            code, output = launch([f"robot:={robot}", "vx:=0.04", "vy:=-0.02", "wz:=0.08",
                                   "duration:=1.2", "confirm:=MOVER"])
            result = parse_result(output)
            (evidence / f"{robot}.log").write_text(output, encoding="utf-8")
            commands = recorders[robot].since(start)
            other = "pepper" if robot == "nao" else "nao"
            moving = [values for _, values in commands if not zero(values)]
            passed = (code == 0 and result.get("outcome") == "application_succeeded"
                      and result.get("robot") == robot
                      and result.get("requested_provider") == PROVIDER
                      and result.get("released") is True
                      and result.get("stop_acknowledged") is True
                      and result.get("physical_success_verified") is False
                      and bool(moving) and len(moving) >= 5
                      and moving[-1][0] > 0 and moving[-1][1] < 0 and moving[-1][5] > 0
                      and zero(commands[-1][1]) and not recorders[other].since(start))
            report["checks"].append({"name": f"{robot}_routing_stream_stop_release", "passed": passed,
                                     "application": result, "sample_count": len(commands),
                                     "provider_statuses": [(stamp, value) for stamp, value in statuses.statuses
                                                           if stamp >= start],
                                     "first_commands": commands[:6], "last_commands": commands[-6:]})

        result, commands = interrupt_during_motion(recorders["nao"], evidence, "hri_app", signal.SIGINT)
        report["checks"].append({"name": "application_sigint_stops_and_releases",
                                 "passed": result.get("outcome") == "application_failed"
                                 and result.get("released") is True
                                 and result.get("stop_acknowledged") is True
                                 and bool(commands) and zero(commands[-1][1]),
                                 "application": result, "last_commands": commands[-5:]})

        extra_publisher = node.create_publisher(Twist, "/hri/cmd_vel", 1)
        try:
            if not wait_until(lambda: node.count_publishers("/hri/cmd_vel") == 1):
                raise RuntimeError("Duplicate-input fixture did not become visible in the graph.")
            time.sleep(0.6)
            start = time.monotonic()
            code, output = launch(["robot:=nao", "vx:=0.04", "duration:=1.2", "confirm:=MOVER"])
            result = parse_result(output)
            (evidence / "multiple_publishers.log").write_text(output, encoding="utf-8")
            commands = recorders["nao"].since(start)
            report["checks"].append({"name": "multiple_publishers_rejected",
                                     "passed": result.get("outcome") == "application_failed"
                                     and result.get("message") in {
                                         "multiple_velocity_publishers", "provider_not_ready",
                                         "provider_status_lost_or_faulted"}
                                     and result.get("released") is True
                                     and all(zero(values) for _, values in commands),
                                     "application": result})
        finally:
            node.destroy_publisher(extra_publisher)

        if not wait_until(lambda: node.count_publishers("/hri/cmd_vel") == 0):
            raise RuntimeError("Duplicate-input fixture did not leave the graph.")
        extra_backend = node.create_publisher(Twist, "/nao/cmd_vel", 1)
        try:
            if not wait_until(lambda: node.count_publishers("/nao/cmd_vel") == 1):
                raise RuntimeError("Duplicate-output fixture did not become visible in the graph.")
            time.sleep(0.6)
            start = time.monotonic()
            code, output = launch(["robot:=nao", "vx:=0.04", "duration:=1.2", "confirm:=MOVER"])
            result = parse_result(output)
            (evidence / "multiple_backend_publishers.log").write_text(output, encoding="utf-8")
            commands = recorders["nao"].since(start)
            report["checks"].append({"name": "multiple_backend_publishers_rejected",
                                     "passed": result.get("outcome") == "application_failed"
                                     and result.get("released") is True
                                     and bool(commands) and all(zero(values) for _, values in commands),
                                     "application": result, "sample_count": len(commands)})
        finally:
            node.destroy_publisher(extra_backend)
        if not wait_until(lambda: node.count_publishers("/nao/cmd_vel") == 0):
            raise RuntimeError("Duplicate-output fixture did not leave the graph.")

        node.destroy_subscription(subscriptions.pop("nao"))
        time.sleep(0.5)
        start = time.monotonic()
        code, output = launch(["robot:=nao", "vx:=0.04", "duration:=1.2", "confirm:=MOVER"])
        result = parse_result(output)
        (evidence / "backend_absent.log").write_text(output, encoding="utf-8")
        report["checks"].append({"name": "backend_absent_fails_closed",
                                 "passed": result.get("outcome") == "application_failed"
                                 and result.get("released") is True
                                 and not recorders["pepper"].since(start),
                                 "application": result, "launch_exit_code": code})
    except Exception as error:
        report["execution_error"] = str(error)
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
    report["passed"] = bool(report["checks"]) and "execution_error" not in report and all(
        check["passed"] for check in report["checks"])
    (evidence / "result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Passive, bounded diagnostics. No commands, services, image/audio pixels or transcripts."""

from collections import Counter, deque
import json
import math
import os
from pathlib import Path
import time


FOLLOW_FIELDS = (
    "reason", "vx_m_s", "vy_m_s", "wz_rad_s", "locked", "motion_enabled",
    "source_stamp_ns", "source_age_s", "receipt_age_s", "image_recovery_active",
    "image_recovery_frames", "image_recoveries", "person_wait_active",
    "person_wait_reason", "person_wait_frames", "person_recoveries", "speech_status",
    "transcripts_received", "camera_status",
)
PERCEPTION_FIELDS = ("age_at_receipt_s", "receipt_gap_s", "people_count", "eligible_count",
                     "invalid_count", "image_width", "image_height", "minimum_confidence")
BOX_FIELDS = ("center_x", "center_y", "width", "height", "confidence", "valid")


def selected_follow(payload):
    result = {key: payload[key] for key in FOLLOW_FIELDS if key in payload}
    perception = payload.get("perception") or {}
    result["perception"] = {key: perception[key] for key in PERCEPTION_FIELDS if key in perception}
    result["perception"]["people"] = [
        {key: box[key] for key in BOX_FIELDS if key in box}
        for box in perception.get("people", [])[:10]]
    return result


def percentile(values, fraction):
    values = sorted(value for value in values if isinstance(value, (int, float)) and math.isfinite(value))
    return values[min(len(values) - 1, math.ceil(fraction * len(values)) - 1)] if values else None


class Timing:
    def __init__(self):
        self.count = 0
        self.first = self.last = None
        self.max_gap = 0.0
        self.recent = deque(maxlen=1200)

    def add(self, now, age):
        if self.first is None:
            self.first = now
        if self.last is not None:
            self.max_gap = max(self.max_gap, now - self.last)
        self.last = now
        self.count += 1
        self.recent.append((now, age))

    def summary(self, now):
        recent = [(stamp, age) for stamp, age in self.recent if now - stamp <= 30.0]
        period = now - recent[0][0] if len(recent) > 1 else 0
        ages = [age for _, age in recent]
        return dict(samples=self.count, recent_hz=(len(recent) - 1) / period if period else None,
                    source_age_p95_s=percentile(ages, .95), source_age_max_s=max(
                        (age for age in ages if isinstance(age, (int, float)) and math.isfinite(age)), default=None),
                    max_receipt_gap_s=self.max_gap,
                    since_last_receipt_s=now - self.last if self.last is not None else None)


def proc_stat(text):
    # The command in parentheses may contain spaces or ')'. Field numbers are
    # from proc_pid_stat(5): ppid=4, utime=14, stime=15, threads=20, start=22, rss=24.
    fields = text.rsplit(")", 1)[1].split()
    return dict(parent=int(fields[1]), ticks=int(fields[11]) + int(fields[12]),
                threads=int(fields[17]), start=int(fields[19]), rss_pages=int(fields[21]))


class Resources:
    LABELS = {"yolo_node", "whisper_worker", "hri_app", "capabilities2_server_node", "hri_diagnostics"}

    def __init__(self, root_pid, proc_root="/proc"):
        self.root_pid = root_pid
        self.proc_root = Path(proc_root)
        self.previous = {}
        self.previous_cpu = None
        self.hz = os.sysconf("SC_CLK_TCK")
        self.page_bytes = os.sysconf("SC_PAGE_SIZE")

    def sample(self, now):
        stats = {}
        for path in self.proc_root.glob("[0-9]*/stat"):
            try:
                stats[int(path.parent.name)] = proc_stat(path.read_text())
            except (OSError, ValueError, IndexError):
                continue  # Processes can exit while being sampled.
        children = {self.root_pid}
        while True:
            expanded = children | {pid for pid, stat in stats.items() if stat["parent"] in children}
            if expanded == children:
                break
            children = expanded
        processes, previous = [], {}
        for pid in children:
            if pid not in stats:
                continue
            try:
                args = (self.proc_root / str(pid) / "cmdline").read_bytes().split(b"\0")
                # Only executable labels are retained; never save command lines.
                labels = {Path(arg.decode(errors="replace")).name for arg in args[:4]}
                label = next(iter(labels & self.LABELS), None)
                if label is None:
                    continue
                stat = stats[pid]
                key = (pid, stat["start"])
                old = self.previous.get(key)
                cpu = (100 * (stat["ticks"] - old[1]) / self.hz / (now - old[0])
                       if old and now > old[0] else None)
                processes.append(dict(process=label, pid=pid, cpu_percent_one_core=cpu,
                                      rss_mib=stat["rss_pages"] * self.page_bytes / 1048576,
                                      threads=stat["threads"]))
                previous[key] = (now, stat["ticks"])
            except OSError:
                continue
        self.previous = previous
        result = dict(processes=sorted(processes, key=lambda item: item["pid"]))
        try:
            cpu = [int(v) for v in (self.proc_root / "stat").read_text().splitlines()[0].split()[1:9]]
            total, idle = sum(cpu), cpu[3] + cpu[4]
            old = self.previous_cpu
            result["system_cpu_percent"] = (100 * (1 - (idle - old[1]) / (total - old[0]))
                                              if old and total > old[0] else None)
            self.previous_cpu = (total, idle)
            result["logical_cpus"] = os.cpu_count()
            memory = dict(line.split(":", 1) for line in (self.proc_root / "meminfo").read_text().splitlines())
            result["memory_available_mib"] = int(memory["MemAvailable"].split()[0]) / 1024
        except (OSError, KeyError, ValueError, IndexError):
            pass
        return result


class Recorder:
    def __init__(self, directory, root_pid, *, max_bytes=10 * 1024 * 1024):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.max_bytes = max_bytes
        self.path = self.directory / "events.jsonl"
        self.stream = self.path.open("a", encoding="utf-8")
        self.size = self.path.stat().st_size
        self.resources = Resources(root_pid)
        self.timings = {name: Timing() for name in ("follow", "yolo", "people")}
        self.reasons = Counter()
        self.reason_samples = Counter()
        self.last_follow_at = self.last_saved_at = None
        self.last_reason = ""
        self.unobserved = 0.0
        self.last_follow = self.last_speech = self.last_resources = {}
        self.started = time.monotonic()
        self.last_source = {"yolo": None, "people": None}
        self.duplicate_sources = Counter()
        self.malformed = 0

    def event(self, kind, data, now):
        line = json.dumps(dict(kind=kind, wall_time_s=time.time(), monotonic_s=now, data=data),
                          ensure_ascii=False, allow_nan=False) + "\n"
        size = len(line.encode("utf-8"))
        if self.size + size > self.max_bytes:
            self.stream.close()
            old = self.directory / "events.1.jsonl"
            if old.exists():
                old.replace(self.directory / "events.2.jsonl")
            self.path.replace(old)
            self.stream = self.path.open("w", encoding="utf-8")
            self.size = 0
        self.stream.write(line)
        self.size += size

    def follow(self, payload, now):
        data = selected_follow(payload)
        reason = str(data.get("reason", "unknown"))[:100]
        if self.last_follow_at is not None:
            delta = max(0.0, now - self.last_follow_at)
            # Do not attribute observer outages to the last controller decision.
            self.reasons[self.last_reason] += min(delta, .25)
            self.unobserved += max(0.0, delta - .25)
        self.timings["follow"].add(now, data.get("source_age_s"))
        self.reason_samples[reason] += 1
        if reason != self.last_reason or self.last_saved_at is None or now - self.last_saved_at >= .2:
            self.event("follow", data, now)
            self.last_saved_at = now
        self.last_follow, self.last_follow_at, self.last_reason = data, now, reason

    def source(self, name, stamp_ns, age, now):
        if self.last_source[name] == stamp_ns:
            self.duplicate_sources[name] += 1
        self.last_source[name] = stamp_ns
        self.timings[name].add(now, age)

    def speech(self, data, now):
        self.last_speech = data
        self.event("speech", data, now)

    def flush(self, now, *, closed=False):
        self.last_resources = self.resources.sample(now)
        self.event("resources", self.last_resources, now)
        self.stream.flush()
        summary = dict(elapsed_s=now - self.started, closed=closed, observation_only=True,
                       follow=self.last_follow, speech=self.last_speech, resources=self.last_resources,
                       streams={name: timing.summary(now) for name, timing in self.timings.items()},
                       reason_seconds_observed=dict(self.reasons), reason_samples=dict(self.reason_samples),
                       unknown_follow_time_s=self.unobserved, duplicate_source_stamps=dict(self.duplicate_sources),
                       malformed_messages=self.malformed,
                       limits=["Best-effort observer can miss samples; rates describe received messages.",
                               "CPU per process: 100% means one logical core; recent stream metrics cover 30 s.",
                               "No images, audio or transcript text saved. Three event files of at most 10 MiB.",
                               "Reason durations are observed estimates; final physical stop is not measured."])
        path = self.directory / "summary.json.tmp"
        path.write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        path.replace(self.directory / "summary.json")

    def close(self):
        try:
            self.flush(time.monotonic(), closed=True)
        finally:
            self.stream.close()


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from yolo_msgs.msg import DetectionArray
    from hri_capability_interfaces.msg import PersonDetection2DArray, SpeechCaptureStatus

    rclpy.init()
    node = Node("hri_diagnostics")
    node.declare_parameter("output_dir", "/tmp/hri-diagnostics")
    node.declare_parameter("launch_pid", os.getppid())
    node.declare_parameter("yolo_topic", "/yolo/detections")
    recorder = None
    try:
        recorder = Recorder(node.get_parameter("output_dir").value, node.get_parameter("launch_pid").value)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        def follow(message):
            try:
                recorder.follow(json.loads(message.data), time.monotonic())
            except (ValueError, TypeError, KeyError, AttributeError):
                recorder.malformed += 1

        def source(name, message):
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            age = (node.get_clock().now().nanoseconds - stamp) / 1e9
            recorder.source(name, stamp, age, time.monotonic())

        def speech(message):
            recorder.speech(dict(state=message.state, queued_segments=message.queued_segments,
                                 dropped_segments=message.dropped_segments), time.monotonic())

        node.create_subscription(String, "/hri/follow_person/preview", follow, qos)
        node.create_subscription(DetectionArray, node.get_parameter("yolo_topic").value,
                                 lambda message: source("yolo", message), qos)
        node.create_subscription(PersonDetection2DArray, "/hri/people", lambda message: source("people", message), qos)
        node.create_subscription(SpeechCaptureStatus, "/hri/speech/capture_status", speech, qos)
        node.create_timer(1.0, lambda: recorder.flush(time.monotonic()))
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if recorder is not None:
                recorder.close()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()

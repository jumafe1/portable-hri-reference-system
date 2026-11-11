"""Timing, redaction, rotation and CPU parsing without ROS or hardware."""

import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hri_reference_app/scripts"))
from hri_diagnostics import Recorder, Resources, Timing, proc_stat, selected_follow


class DiagnosticTest(unittest.TestCase):
    def test_follow_record_excludes_audio_image_and_transcript_fields(self):
        result = selected_follow(dict(reason="following", vx_m_s=.1, transcript="private words",
                                      audio=[1, 2], image=[3, 4], perception=dict(
                                          people_count=1, text="private words", people=[dict(
                                              confidence=.9, height=123, transcript="private words")])) )
        self.assertEqual(result["perception"]["people"], [dict(confidence=.9, height=123)])
        self.assertNotIn("private", json.dumps(result))
        self.assertNotIn("audio", result)
        self.assertNotIn("image", result)

    def test_recent_rates_decay_during_a_gap_and_old_ages_expire(self):
        timing = Timing()
        for now, age in ((0, .1), (1, .2), (2, .3)):
            timing.add(now, age)
        self.assertEqual(timing.summary(2)["recent_hz"], 1)
        self.assertEqual(timing.summary(4)["recent_hz"], .5)
        self.assertEqual(timing.summary(4)["source_age_p95_s"], .3)
        self.assertIsNone(timing.summary(40)["source_age_p95_s"])
        self.assertEqual(timing.summary(40)["samples"], 3)
        timing.add(41, .1)
        self.assertEqual(timing.summary(41)["max_receipt_gap_s"], 39)

    def test_reason_durations_do_not_blame_an_observer_gap_on_the_controller(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = Recorder(directory, 1)
            with patch.object(recorder.resources, "sample", return_value={}):
                recorder.follow(dict(reason="preview_following", source_age_s=.1), 10)
                recorder.follow(dict(reason="waiting_fresh_images", source_age_s=.8), 10.1)
                recorder.follow(dict(reason="preview_following", source_age_s=.1), 12.1)
                recorder.flush(12.1)
                result = json.loads((Path(directory) / "summary.json").read_text())
                recorder.close()
            self.assertAlmostEqual(result["reason_seconds_observed"]["preview_following"], .1)
            self.assertAlmostEqual(result["reason_seconds_observed"]["waiting_fresh_images"], .25)
            self.assertAlmostEqual(result["unknown_follow_time_s"], 1.75)
            self.assertEqual(result["streams"]["follow"]["samples"], 3)

    def test_event_files_rotate_and_summary_remains_readable_after_close(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = Recorder(directory, 1, max_bytes=1000)
            with patch.object(recorder.resources, "sample", return_value={}):
                for index in range(60):
                    recorder.speech(dict(state="processing", queued_segments=2, dropped_segments=index), index)
                recorder.close()
            events = list(Path(directory).glob("events*.jsonl"))
            self.assertEqual(len(events), 3)
            for path in events:
                self.assertLessEqual(path.stat().st_size, 1000)
                for line in path.read_text().splitlines():
                    json.loads(line)
            self.assertTrue(json.loads((Path(directory) / "summary.json").read_text())["closed"])
            self.assertFalse((Path(directory) / "summary.json.tmp").exists())

    def test_repeated_source_stamp_is_visible(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = Recorder(directory, 1)
            with patch.object(recorder.resources, "sample", return_value={}):
                recorder.source("yolo", 100, .1, 1)
                recorder.source("yolo", 100, .2, 2)
                recorder.source("yolo", 101, .1, 3)
                recorder.close()
            result = json.loads((Path(directory) / "summary.json").read_text())
            self.assertEqual(result["duplicate_source_stamps"]["yolo"], 1)

    def test_proc_stat_handles_spaces_and_parentheses_in_command(self):
        fields = ["0"] * 22
        fields[0], fields[1] = "S", "100"
        fields[11], fields[12], fields[17], fields[19], fields[21] = "200", "30", "4", "55", "256"
        result = proc_stat("123 (a complicated ) name) " + " ".join(fields))
        self.assertEqual(result, dict(parent=100, ticks=230, threads=4, start=55, rss_pages=256))

    def test_cpu_sampler_scopes_children_and_handles_pid_reuse_without_saving_arguments(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)

            def process(pid, parent, ticks, start=1):
                path = root / str(pid)
                path.mkdir(exist_ok=True)
                fields = ["0"] * 22
                fields[0], fields[1], fields[11] = "S", str(parent), str(ticks)
                fields[17], fields[19], fields[21] = "2", str(start), "256"
                (path / "stat").write_text(f"{pid} (python3) " + " ".join(fields))
                (path / "cmdline").write_bytes(b"python3\0/path/whisper_worker\0--secret\0private-value\0")

            process(101, 100, 10)
            process(201, 200, 10)  # Same executable belonging to a different launch.
            monitor = Resources(100, root)
            monitor.hz = 100
            first = monitor.sample(1)
            self.assertEqual([p["pid"] for p in first["processes"]], [101])
            self.assertIsNone(first["processes"][0]["cpu_percent_one_core"])
            process(101, 100, 110)
            second = monitor.sample(2)
            self.assertEqual(second["processes"][0]["cpu_percent_one_core"], 100)
            self.assertNotIn("private", json.dumps(second))
            process(101, 100, 2, start=2)
            self.assertIsNone(monitor.sample(3)["processes"][0]["cpu_percent_one_core"])


if __name__ == "__main__":
    unittest.main()

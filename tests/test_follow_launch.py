"""Launch argument validation only: never starts a graph or robot."""

import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


SCRIPT = Path(__file__).resolve().parents[1] / "hri_reference_app/launch/hri_app.launch.py"
SPEC = importlib.util.spec_from_file_location("hri_follow_launch_test", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FollowLaunchArgumentsTest(unittest.TestCase):
    def arguments(self, mode="follow_person", **overrides):
        values = dict(image_width="320", image_height="240", expected_frame="camera",
                      minimum_confidence="", target_height_ratio="0.6", timeout="",
                      enable_motion="false", confirm="", duration="2.0",
                      vx="0.02", vy="0", wz="0")
        values.update(overrides)
        with patch.object(MODULE, "argument", side_effect=lambda _context, name: values[name]):
            return MODULE.application_arguments(None, mode, "nao")

    def value(self, args, name):
        return args[args.index(name) + 1]

    def test_default_timeout_is_resolved_by_mode(self):
        self.assertEqual(self.value(self.arguments(), "--timeout"), "120.0")
        self.assertEqual(self.value(self.arguments("detect_people"), "--timeout"), "20.0")
        self.assertNotIn("--enable-motion", self.arguments())

    def test_launch_declares_empty_timeout_for_mode_specific_defaults(self):
        context = LaunchContext()
        declaration = next(action for action in MODULE.generate_launch_description().entities
                           if isinstance(action, DeclareLaunchArgument) and action.name == "timeout")
        declaration.execute(context)
        self.assertEqual(context.launch_configurations["timeout"], "")

    def test_long_follow_duration_and_timeout_are_preserved(self):
        for duration in ("0.1", "120", "300"):
            with self.subTest(duration=duration):
                args = self.arguments(enable_motion="true", confirm="MOVER",
                                      duration=duration, timeout="300")
                self.assertEqual(float(self.value(args, "--duration")), float(duration))
                self.assertEqual(self.value(args, "--timeout"), "300.0")

    def test_invalid_follow_duration_is_rejected(self):
        for duration in ("0.09", "300.01", "301", "nan", "inf", "-inf"):
            with self.subTest(duration=duration), self.assertRaises(RuntimeError):
                self.arguments(enable_motion="true", confirm="MOVER", duration=duration)

    def test_invalid_follow_timeout_is_rejected(self):
        for timeout in ("0.99", "300.01", "301", "nan", "inf", "-inf"):
            with self.subTest(timeout=timeout), self.assertRaises(RuntimeError):
                self.arguments(timeout=timeout)

    def test_move_velocity_still_rejects_more_than_five_seconds(self):
        for duration in ("5.01", "120", "300"):
            with self.subTest(duration=duration), self.assertRaises(RuntimeError):
                self.arguments("move_velocity", confirm="MOVER", duration=duration)
        args = self.arguments("move_velocity", confirm="MOVER", duration="5")
        self.assertEqual(self.value(args, "--duration"), "5.0")


if __name__ == "__main__":
    unittest.main()

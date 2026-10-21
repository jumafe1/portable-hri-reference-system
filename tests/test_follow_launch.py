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
                      max_forward_speed="0.03",
                      orient_camera="false", camera_provider="hri_naoqi_providers/NaoqiOrientCamera",
                      transcribe_speech="false", confirm_listen="",
                      record_duration="", speech_language="es",
                      speech_provider="hri_whisper_providers/WhisperTranscribeSpeech", print_transcript="false",
                      enable_motion="false", confirm="", duration="",
                      vx="0.02", vy="0", wz="0")
        values.update(overrides)
        with patch.object(MODULE, "argument", side_effect=lambda _context, name: values[name]):
            return MODULE.application_arguments(None, mode, "nao")

    def value(self, args, name):
        return args[args.index(name) + 1]

    def test_default_timeout_is_resolved_by_mode(self):
        self.assertNotIn("--timeout", self.arguments())
        self.assertNotIn("--duration", self.arguments())
        self.assertEqual(self.value(self.arguments("detect_people"), "--timeout"), "20.0")
        self.assertEqual(self.value(self.arguments("move_velocity", confirm="MOVER"), "--duration"), "2.0")
        self.assertNotIn("--enable-motion", self.arguments())

    def test_launch_declares_empty_timeout_for_mode_specific_defaults(self):
        context = LaunchContext()
        declaration = next(action for action in MODULE.generate_launch_description().entities
                           if isinstance(action, DeclareLaunchArgument) and action.name == "timeout")
        declaration.execute(context)
        self.assertEqual(context.launch_configurations["timeout"], "")

    def test_speed_default_and_explicit_values_are_forwarded_without_motion_opt_in(self):
        self.assertEqual(float(self.value(self.arguments(), "--max-forward-speed")), 0.03)
        for maximum in ("0.01", "0.05", "0.1"):
            with self.subTest(maximum=maximum):
                args = self.arguments(max_forward_speed=maximum)
                self.assertEqual(float(self.value(args, "--max-forward-speed")), float(maximum))
                self.assertNotIn("--enable-motion", args)

    def test_launch_declares_conservative_speed_default(self):
        context = LaunchContext()
        declaration = next(action for action in MODULE.generate_launch_description().entities
                           if isinstance(action, DeclareLaunchArgument)
                           and action.name == "max_forward_speed")
        declaration.execute(context)
        self.assertEqual(float(context.launch_configurations["max_forward_speed"]), 0.03)

    def test_invalid_speed_is_rejected_with_or_without_motion(self):
        for enabled in ("true", "false"):
            for maximum in ("0", "-0.1", "0.10001", "nan", "inf", "-inf", "true"):
                with self.subTest(enabled=enabled, maximum=maximum), self.assertRaises(RuntimeError):
                    self.arguments(max_forward_speed=maximum, enable_motion=enabled, confirm="MOVER")

    def test_follow_motion_has_no_total_time_flags(self):
        args = self.arguments(enable_motion="true", confirm="MOVER")
        self.assertIn("--enable-motion", args)
        self.assertNotIn("--duration", args)
        self.assertNotIn("--timeout", args)

    def test_previously_valid_temporal_options_are_rejected(self):
        for enabled in ("true", "false"):
            for options in ({"duration": "2"}, {"duration": "300"}, {"timeout": "120"}):
                with self.subTest(enabled=enabled, options=options), self.assertRaisesRegex(RuntimeError, "continuous"):
                    self.arguments(enable_motion=enabled, confirm="MOVER", **options)

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

    def test_optional_camera_flags(self):
        self.assertNotIn("--orient-camera", self.arguments())
        args = self.arguments(orient_camera="true")
        self.assertIn("--orient-camera", args)
        self.assertNotIn("--enable-motion", args)
        self.assertEqual(self.value(args, "--camera-provider"), "hri_naoqi_providers/NaoqiOrientCamera")
        with self.assertRaises(RuntimeError):
            self.arguments(orient_camera="true", camera_provider="")
        with self.assertRaises(RuntimeError):
            self.arguments(orient_camera="yes")

    def test_speech_defaults_and_consent(self):
        with self.assertRaises(RuntimeError):
            self.arguments("transcribe_speech")
        args = self.arguments("transcribe_speech", confirm="ESCUCHAR")
        self.assertEqual(self.value(args, "--record-duration"), "3.0")
        self.assertNotIn("--speech-language", args)
        self.assertEqual(self.value(args, "--timeout"), "60.0")
        self.assertNotIn("--print-transcript", args)
        self.assertNotIn("--enable-motion", args)
        self.assertIn("--print-transcript", self.arguments(
            "transcribe_speech", confirm="ESCUCHAR", print_transcript="true"))

    def test_invalid_speech_arguments(self):
        for options in ({"record_duration": "nan"}, {"record_duration": "0.49"},
                        {"record_duration": "10.1"}, {"timeout": "3"}, {"timeout": "inf"},
                        {"speech_language": "xx"}, {"speech_provider": " "}, {"print_transcript": "yes"}):
            with self.subTest(options=options), self.assertRaises(RuntimeError):
                self.arguments("transcribe_speech", confirm="ESCUCHAR", **options)

    def test_follow_speech_is_separate_opt_in_from_motion(self):
        self.assertNotIn("--transcribe-speech", self.arguments(confirm_listen="ESCUCHAR"))
        with self.assertRaisesRegex(RuntimeError, "confirm_listen"):
            self.arguments(transcribe_speech="true", confirm="MOVER")
        with self.assertRaises(RuntimeError):
            self.arguments(transcribe_speech="yes")
        args = self.arguments(transcribe_speech="true", confirm_listen="ESCUCHAR")
        self.assertIn("--transcribe-speech", args)
        self.assertNotIn("--enable-motion", args)
        self.assertNotIn("--timeout", args)
        self.assertNotIn("--duration", args)
        self.assertNotIn("--print-transcript", args)
        self.assertNotIn("--record-duration", args)
        self.assertNotIn("--speech-language", args)
        combined = self.arguments(transcribe_speech="true", confirm_listen="ESCUCHAR",
                                  enable_motion="true", confirm="MOVER", orient_camera="true",
                                  print_transcript="true")
        for flag in ("--transcribe-speech", "--enable-motion", "--orient-camera", "--print-transcript"):
            self.assertIn(flag, combined)

    def test_follow_speech_validates_options_and_rejects_legacy_duration(self):
        for options in ({"record_duration": "not-a-number"}, {"record_duration": "nan"}, {"record_duration": "0.49"},
                        {"record_duration": "10.1"}, {"speech_language": "xx"},
                        {"speech_provider": " "}, {"print_transcript": "yes"},
                        {"timeout": "60"}, {"duration": "3"}):
            with self.subTest(options=options), self.assertRaises(RuntimeError):
                self.arguments(transcribe_speech="true", confirm_listen="ESCUCHAR", **options)

    def test_follow_rejects_record_duration_even_when_voice_is_disabled(self):
        for enabled in ("true", "false"):
            for value in ("0.5", "3", "10", "nan"):
                with self.subTest(enabled=enabled, value=value), \
                        self.assertRaisesRegex(RuntimeError, "remove record_duration"):
                    self.arguments(transcribe_speech=enabled, confirm_listen="ESCUCHAR", record_duration=value)

    def test_capture_default_is_mode_specific_and_language_override_is_preserved(self):
        context = LaunchContext()
        declaration = next(action for action in MODULE.generate_launch_description().entities
                           if isinstance(action, DeclareLaunchArgument) and action.name == "record_duration")
        declaration.execute(context)
        self.assertEqual(context.launch_configurations["record_duration"], "")
        for language in ("en", ""):
            with self.subTest(language=language):
                args = self.arguments(transcribe_speech="true", confirm_listen="ESCUCHAR", speech_language=language)
                self.assertEqual(self.value(args, "--speech-language"), language)
                self.assertNotIn("--record-duration", args)
                standalone = self.arguments("transcribe_speech", confirm="ESCUCHAR",
                                            speech_language=language, record_duration="0.5")
                self.assertEqual(self.value(standalone, "--speech-language"), language)
                self.assertEqual(self.value(standalone, "--record-duration"), "0.5")

    def test_optional_missing_whisper_package_does_not_block_follow_launch(self):
        context = LaunchContext()
        context.launch_configurations.update(
            robot="nao", mode="follow_person", image_width="320", image_height="240",
            expected_frame="camera", transcribe_speech="true", confirm_listen="ESCUCHAR")
        for action in MODULE.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)

        def share(package):
            if package == "hri_whisper_providers":
                raise MODULE.PackageNotFoundError(package)
            return "/tmp/probe/" + package + "/share/" + package

        with patch.object(MODULE, "require_yasmin"), patch.object(MODULE, "find_spec", return_value=True), \
                patch.object(MODULE, "get_package_share_directory", side_effect=share), \
                patch.object(MODULE, "get_package_prefix", return_value="/tmp/probe"):
            self.assertEqual(len(MODULE.launch_setup(context)), 3)
            context.launch_configurations.update(mode="transcribe_speech", confirm="ESCUCHAR")
            with self.assertRaises(MODULE.PackageNotFoundError):
                MODULE.launch_setup(context)


if __name__ == "__main__":
    unittest.main()

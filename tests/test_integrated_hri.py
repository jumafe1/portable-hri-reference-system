"""Integrated app preflight and orchestration; no graph, microphone or robot."""

from concurrent.futures import Future
from contextlib import ExitStack, redirect_stderr
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription


ROOT = Path(__file__).resolve().parents[1]
SHARE = ROOT / "hri_reference_app"
sys.path.insert(0, str(SHARE / "scripts"))
import follow_mode
import startup_speak
from follow_motion import FollowMotion

SPEC = importlib.util.spec_from_file_location("integrated_hri_test", SHARE / "launch/aplicacion_hri.launch.py")
ENTRY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ENTRY)


def completed(value):
    future = Future()
    future.set_result(value)
    return future


class IntegratedLaunchTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.work = Path(self.directory.name)
        self.model = self.work / "model.pt"
        self.model.touch()
        self.profile = ENTRY.load_profile(SHARE / "config/nao.yaml", "nao")
        self.path = self.work / "profile.yaml"
        self.save_profile()
        self.app = ENTRY.application_module(SHARE)
        self.context = LaunchContext()
        self.context.launch_configurations.update(
            profile=str(self.path), confirm_listen="ESCUCHAR", yolo_model=str(self.model),
            whisper_model_path=str(self.model))
        for action in ENTRY.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(self.context)
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(ENTRY, "get_package_share_directory", side_effect=self.share))
        self.stack.enter_context(patch.object(ENTRY, "workspace_directory", return_value=self.work))
        self.stack.enter_context(patch.object(ENTRY, "application_module", return_value=self.app))
        self.stack.enter_context(patch.object(self.app, "get_package_share_directory", side_effect=self.share))
        self.stack.enter_context(patch.object(self.app, "get_package_prefix", return_value=str(self.work / "install/app")))
        self.stack.enter_context(patch.object(self.app, "require_yasmin"))
        self.stack.enter_context(patch.object(self.app, "find_spec", return_value=True))
        self.node = self.stack.enter_context(patch.object(self.app, "Node", return_value=MagicMock()))

    def share(self, package):
        return str(SHARE if package == "hri_reference_app" else self.work / package / "share" / package)

    def save_profile(self):
        self.path.write_text(yaml.safe_dump(self.profile), encoding="utf-8")

    def launch(self, **overrides):
        self.context.launch_configurations.update(overrides)
        return ENTRY.launch_setup(self.context)

    def node_options(self, executable):
        return next(call.kwargs for call in self.node.call_args_list if call.kwargs["executable"] == executable)

    def test_single_launch_starts_workers_server_and_preview_without_greeting(self):
        actions = self.launch()
        self.assertEqual([call.kwargs["executable"] for call in self.node.call_args_list],
                         ["capabilities2_server_node", "hri_app", "whisper_worker"])
        args = self.node_options("hri_app")["arguments"]
        parsed = follow_mode.parse_args(args[2:])
        self.assertFalse(parsed.enable_motion)
        self.assertTrue(parsed.transcribe_speech)
        self.assertFalse(parsed.print_transcript)
        for executable in ("hri_app", "whisper_worker", "capabilities2_server_node"):
            self.assertEqual(self.node_options(executable)["ros_arguments"], ["--log-level", "warn"])
        self.assertEqual(parsed.startup_text, "")
        self.assertEqual(parsed.target_height_ratio, .75)
        self.assertEqual(parsed.max_forward_speed, .1)
        group = next(action for action in actions if isinstance(action, GroupAction))
        inclusion = next(action for action in group.get_sub_entities() if isinstance(action, IncludeLaunchDescription))
        settings = dict(inclusion.launch_arguments)
        self.assertEqual(settings["device"], "cpu")
        self.assertEqual(settings["input_image_topic"], "/nao/camera/front/image_raw")
        self.assertEqual(settings["use_tracking"], "False")
        self.assertEqual(settings["log_level"], "warn")
        self.assertEqual(settings["enable_nnpack"], "False")
        self.assertEqual(settings["model"], str(self.model))
        self.assertEqual(self.node_options("whisper_worker")["parameters"][0]["audio_topic"], "/nao/mic_stamped")

    def test_motion_requires_both_flags_before_starting_any_worker(self):
        with self.assertRaisesRegex(RuntimeError, "MOVER"):
            self.launch(enable_motion="true")
        self.node.assert_not_called()
        self.launch(enable_motion="true", confirm="MOVER")
        self.assertIn("--enable-motion", self.node_options("hri_app")["arguments"])

    def test_optional_diagnostics_is_an_independent_observer_in_the_requested_directory(self):
        with patch.object(ENTRY, "Node") as observer:
            self.launch(record_diagnostics="true", diagnostics_dir=str(self.work / "diagnostics"))
        observer.assert_called_once()
        options = observer.call_args.kwargs
        self.assertEqual(options["executable"], "hri_diagnostics")
        self.assertEqual(options["parameters"][0]["output_dir"], str(self.work / "diagnostics"))
        self.assertGreater(options["parameters"][0]["launch_pid"], 0)
        self.assertFalse(follow_mode.parse_args(self.node_options("hri_app")["arguments"][2:]).enable_motion)

    def test_missing_listening_consent_starts_nothing(self):
        with self.assertRaisesRegex(RuntimeError, "ESCUCHAR"):
            self.launch(confirm_listen="")
        self.node.assert_not_called()

    def test_missing_model_starts_nothing(self):
        for key in ("yolo_model", "whisper_model_path"):
            with self.subTest(key=key), self.assertRaisesRegex(RuntimeError, "Local model not found"):
                self.launch(**{key: str(self.work / "missing.pt")})
            self.context.launch_configurations[key] = str(self.model)
        self.node.assert_not_called()

    def test_external_workers_do_not_require_local_weights_or_spawn_duplicates(self):
        actions = self.launch(start_yolo="false", start_whisper_worker="false", yolo_model="missing.pt",
                              whisper_model_path="missing.pt", greet="false")
        self.assertFalse(any(isinstance(action, GroupAction) for action in actions))
        self.assertEqual(self.node.call_count, 2)
        self.assertNotIn("--startup-text", self.node_options("hri_app")["arguments"])

    def test_disabled_voice_needs_neither_whisper_model_nor_consent(self):
        self.launch(transcribe_speech="false", confirm_listen="", whisper_model_path="missing.pt")
        self.assertEqual(self.node.call_count, 2)
        self.assertNotIn("--transcribe-speech", self.node_options("hri_app")["arguments"])

    def test_wrong_camera_mapping_or_profile_is_rejected_before_workers(self):
        self.profile["application"]["detections_topic"] = "/other/detections"
        self.save_profile()
        with self.assertRaisesRegex(RuntimeError, "namespace"):
            self.launch()
        self.node.assert_not_called()
        with self.assertRaisesRegex(RuntimeError, "selected robot"):
            self.launch(robot="other")

    def test_custom_robot_providers_reach_core_and_server_without_naoqi(self):
        self.profile["robot"] = "wheeled_test"
        options = self.profile["application"]
        options.update(people_provider="test_vision/Detect", motion_provider="test_base/Move",
                       speak_provider="test_voice/Speak", audio_topic="/wheels/mic",
                       audio_info_topic="/wheels/mic_info", backend_remappings=[["/backend/velocity", "/wheels/cmd_vel"]])
        self.save_profile()
        self.launch(robot="wheeled_test", start_yolo="false", enable_motion="true", confirm="MOVER")
        args = self.node_options("hri_app")["arguments"]
        parsed = follow_mode.parse_args(args[2:])
        self.assertEqual(parsed.robot, "wheeled_test")
        self.assertEqual(parsed.motion_provider, "test_base/Move")
        server = self.node_options("capabilities2_server_node")
        paths = server["parameters"][0]["package_paths"]
        self.assertFalse(any("hri_naoqi_providers" in path for path in paths))
        self.assertTrue(any("test_base" in path for path in paths))
        self.assertIn(("/backend/velocity", "/wheels/cmd_vel"), server["remappings"])
        self.assertEqual(self.node_options("whisper_worker")["parameters"][0]["audio_topic"], "/wheels/mic")

    def test_invalid_boolean_or_excessive_speed_starts_nothing(self):
        for options in ({"enable_motion": "yes"}, {"start_yolo": "yes"}, {"max_forward_speed": "0.5"}):
            before = dict(self.context.launch_configurations)
            with self.subTest(options=options), self.assertRaises(RuntimeError):
                self.launch(**options)
            self.context.launch_configurations.update(before)
        self.node.assert_not_called()


class StartupSpeakTest(unittest.TestCase):
    def setUp(self):
        self.session = MagicMock()
        self.session.args = SimpleNamespace(speak_provider=startup_speak.PROVIDER,
                                            startup_text="Hola.", speak_language="Spanish")
        self.session.cancelled.is_set.return_value = False
        self.session.bond.is_broken.return_value = False
        self.acquire, self.speak = MagicMock(), MagicMock()
        self.session.node.create_client.side_effect = [self.acquire, self.speak]
        self.acquire.call_async.return_value = completed(None)
        self.speak.call_async.return_value = completed(SimpleNamespace(success=True, provider=startup_speak.PROVIDER))
        self.session.running_client.call_async.return_value = completed(SimpleNamespace(running_capabilities=[
            SimpleNamespace(capability=SimpleNamespace(capability=startup_speak.CAPABILITY, provider=startup_speak.PROVIDER))]))
        self.session.release_client.call_async.return_value = completed(None)
        self.greeting = startup_speak.StartupSpeak(self.session)
        self.board = {"bond_id": "one-app-bond", "message": "starting"}
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(startup_speak.rclpy, "ok", return_value=True))
        self.clock = 0.0
        self.stack.enter_context(patch.object(startup_speak.time, "monotonic", side_effect=lambda: self.clock))
        self.stack.enter_context(patch.object(startup_speak.time, "sleep", side_effect=self.advance))

    def advance(self, seconds):
        self.clock += seconds

    def test_greeting_uses_same_bond_and_portable_service_then_releases(self):
        self.assertEqual(self.greeting.execute(self.board), startup_speak.SUCCEED)
        self.assertEqual(self.acquire.call_async.call_args.args[0].bond_id, "one-app-bond")
        self.assertEqual(self.speak.call_async.call_args.args[0].text, "Hola.")
        self.assertEqual(self.session.release_client.call_async.call_args.args[0].capability, startup_speak.CAPABILITY)
        self.assertEqual(self.greeting.status, "completed")
        self.assertEqual(self.greeting.release(), startup_speak.SUCCEED)
        self.session.release_client.call_async.assert_called_once()

    def test_lost_acquisition_reply_still_releases_and_never_speaks(self):
        self.acquire.call_async.return_value = Future()
        self.assertEqual(self.greeting.execute(self.board), startup_speak.ABORT)
        self.speak.call_async.assert_not_called()
        self.assertTrue(self.greeting.released)
        self.assertTrue(self.acquire.call_async.return_value.cancelled())

    def test_wrong_running_provider_does_not_get_a_speech_command(self):
        self.session.running_client.call_async.return_value = completed(SimpleNamespace(running_capabilities=[]))
        self.assertEqual(self.greeting.execute(self.board), startup_speak.ABORT)
        self.speak.call_async.assert_not_called()
        self.assertTrue(self.greeting.released)

    def test_failed_speak_or_unexpected_provider_aborts_and_releases(self):
        for response in (SimpleNamespace(success=False, provider=startup_speak.PROVIDER),
                         SimpleNamespace(success=True, provider="other/Voice")):
            with self.subTest(response=response):
                self.speak.call_async.return_value = completed(response)
                self.assertEqual(self.greeting.execute(self.board), startup_speak.ABORT)
                self.assertEqual(self.greeting.status, "failed")

    def test_cancelled_before_greeting_sends_no_speech(self):
        self.session.cancelled.is_set.return_value = True
        self.assertEqual(self.greeting.execute(self.board), startup_speak.ABORT)
        self.acquire.call_async.assert_not_called()
        self.speak.call_async.assert_not_called()
        self.assertTrue(self.greeting.released)

    def test_release_error_is_reported_and_can_be_retried_in_final_cleanup(self):
        self.session.release_client.call_async.side_effect = RuntimeError("transport")
        self.assertEqual(self.greeting.execute(self.board), startup_speak.ABORT)
        self.assertFalse(self.greeting.released)
        self.assertEqual(self.board["message"], "startup_speech_release_failed")
        self.session.release_client.call_async.side_effect = None
        self.assertEqual(self.greeting.release(), startup_speak.SUCCEED)

    def test_timeout_does_not_retry_possibly_already_spoken_text(self):
        self.speak.call_async.return_value = Future()
        self.assertEqual(self.greeting.execute(self.board), startup_speak.ABORT)
        self.speak.call_async.assert_called_once()
        self.assertTrue(self.speak.call_async.return_value.cancelled())
        self.assertTrue(self.greeting.released)


class CompositionTest(unittest.TestCase):
    def test_startup_speech_precedes_detection_listening_and_motion(self):
        session = SimpleNamespace(node=MagicMock(), startup_speak=MagicMock(), motion=MagicMock(),
                                  speech=MagicMock(), camera=None, start_bond=MagicMock(),
                                  wait_ready=MagicMock(), observe=MagicMock())
        machine = MagicMock()
        with patch.object(follow_mode, "StateMachine", return_value=machine), patch.object(follow_mode, "ServiceState"), \
                patch.object(follow_mode, "CbState", side_effect=lambda _, callback: callback):
            follow_mode.build_state_machine(session)
        states = {call.args[0]: call for call in machine.add_state.call_args_list}
        success, abort = follow_mode.SUCCEED, follow_mode.ABORT
        self.assertEqual(states["START_HEARTBEAT"].kwargs["transitions"][success], "STARTUP_SPEECH")
        self.assertEqual(states["STARTUP_SPEECH"].args[1], session.startup_speak.execute)
        self.assertEqual(states["STARTUP_SPEECH"].kwargs["transitions"][abort], follow_mode.APP_FAILED)
        self.assertEqual(states["STARTUP_SPEECH"].kwargs["transitions"][success], "USE_DETECT_PEOPLE")
        self.assertEqual(states["VERIFY_PROVIDER"].kwargs["transitions"][success], "CONFIGURE_SPEECH")
        self.assertEqual(states["CONFIGURE_SPEECH"].kwargs["transitions"][success], "USE_MOVE_VELOCITY")

    def test_motion_provider_is_selected_from_profile(self):
        session = SimpleNamespace(node=MagicMock(), args=SimpleNamespace(motion_provider="wheels/Velocity"))
        motion = FollowMotion(session)
        self.assertEqual(motion.request({"bond_id": "same-bond"}).preferred_provider, "wheels/Velocity")

    def test_bad_greeting_or_provider_rejected_by_direct_cli(self):
        base = ["--robot", "nao", "--image-width", "320", "--image-height", "240", "--expected-frame", "camera"]
        for extra in (["--startup-text", "x" * 121], ["--people-provider", "unknown"], ["--robot", "../bad"]):
            with self.subTest(extra=extra), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                follow_mode.parse_args(base + extra)


if __name__ == "__main__":
    unittest.main()

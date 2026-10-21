"""Optional follow transcription contracts: no graph, model, microphone or robot."""

from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hri_reference_app/scripts"))
import follow_speech as speech
import follow_mode
from follow_mode import parse_args
from hri_capability_interfaces.msg import SpeechTranscript


class ArgumentsTest(unittest.TestCase):
    BASE = ["--robot", "nao", "--image-width", "320", "--image-height", "240", "--expected-frame", "camera"]

    def test_disabled_default_and_separate_consent(self):
        args = parse_args(self.BASE)
        self.assertFalse(args.transcribe_speech)
        self.assertFalse(args.print_transcript)
        self.assertFalse(hasattr(args, "record_duration"))
        self.assertEqual(args.speech_language, "es")
        args = parse_args(self.BASE + ["--transcribe-speech", "--confirm-listen", "ESCUCHAR"])
        self.assertTrue(args.transcribe_speech)
        self.assertFalse(args.enable_motion)
        for more in (["--transcribe-speech"], ["--transcribe-speech", "--confirm", "MOVER"],
                     ["--transcribe-speech", "--confirm-listen", "escuchar"],
                     ["--transcribe-speech", "--confirm-listen", "ESCUCHAR", "--enable-motion"]):
            with self.subTest(more=more), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(self.BASE + more)

    def test_record_duration_is_not_a_follow_option(self):
        self.assertFalse(parse_args(self.BASE + ["--print-transcript"]).transcribe_speech)
        for enabled in ([], ["--transcribe-speech", "--confirm-listen", "ESCUCHAR"]):
            for value in ("0.5", "3", "10", "nan"):
                with self.subTest(enabled=enabled, value=value), redirect_stderr(io.StringIO()), \
                        self.assertRaises(SystemExit):
                    parse_args(self.BASE + enabled + ["--record-duration", value])
        for key, value in (("--speech-provider", " "), ("--speech-language", "xx")):
            with self.subTest(key=key, value=value), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(self.BASE + [key, value])


class FollowSpeechTest(unittest.TestCase):
    def setUp(self):
        self.session = MagicMock()
        self.session.args = SimpleNamespace(speech_provider=speech.PROVIDER,
                                            speech_language="es", print_transcript=False)
        self.session.cancelled = threading.Event()
        self.session.bond.is_broken.return_value = False
        self.session.node.count_publishers.return_value = 1
        self.session.node.get_clock().now().nanoseconds = 10_000_000_000
        self.voice = speech.FollowSpeech(self.session)
        self.voice.transcribe = MagicMock()
        self.voice.active = self.voice.requested = True
        self.voice.bond_id, self.voice.status = "bond", "ready"
        self.clock = 100.0
        self.patches = [patch.object(speech.time, "monotonic", side_effect=lambda: self.clock),
                        patch.object(speech.rclpy, "ok", return_value=True)]
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)

    def future(self, result=None, *, done=True, error=None):
        future = MagicMock()
        future.done.return_value = done
        future.exception.return_value = error
        future.result.return_value = result
        return future

    def start(self):
        future = self.future(done=False)
        self.voice.transcribe.call_async.return_value = future
        self.voice.tick()
        return future

    def admit(self, future=None, **overrides):
        future = future or self.start()
        data = dict(accepted=True, request_id=self.voice.request_id, provider=speech.PROVIDER, message="capture_started")
        data.update(overrides)
        future.result.return_value = SimpleNamespace(**data)
        future.done.return_value = True
        self.voice.tick()

    def result(self, **overrides):
        data = dict(request_id=self.voice.request_id, provider=speech.PROVIDER, text="hola", language="es",
                    success=True, message="transcribed")
        data.update(overrides)
        result = SpeechTranscript(**data)
        result.header.stamp.sec = 10
        return result

    def test_serial_windows_wait_for_result_without_blocking(self):
        self.admit()
        self.assertEqual(self.voice.status, "capturing")
        self.clock += 4
        for _ in range(20):
            self.voice.tick()
        self.assertEqual(self.voice.status, "processing")
        self.voice.transcribe.call_async.assert_called_once()
        self.voice.on_result(self.result())
        self.voice.tick()
        self.assertEqual(self.voice.transcripts_received, 1)
        self.assertEqual(self.voice.status, "ready")
        self.voice.transcribe.call_async.assert_called_once()
        self.voice.tick()
        self.assertEqual(self.voice.transcribe.call_async.call_count, 2)
        requests = [call.args[0] for call in self.voice.transcribe.call_async.call_args_list]
        self.assertNotEqual(requests[0].request_id, requests[1].request_id)
        self.assertEqual((requests[0].duration_sec, requests[0].language), (3.0, "es"))

    def test_wrong_identity_ignored_and_early_result_waits_for_admission(self):
        future = self.start()
        self.voice.on_result(self.result(request_id="foreign"))
        self.voice.on_result(self.result(provider="other"))
        self.assertIsNone(self.voice.result)
        self.voice.on_result(self.result())
        self.voice.on_result(self.result(text="replacement"))
        self.voice.tick()
        self.assertEqual(self.voice.transcripts_received, 0)
        self.admit(future)
        self.assertEqual(self.voice.transcripts_received, 1)

    def test_internal_windows_continue_beyond_five_minutes_without_session_deadline(self):
        started = self.clock
        for _ in range(7):
            self.admit()
            self.clock += 50  # Slow but within each independent inference deadline.
            self.voice.tick()
            self.assertTrue(self.voice.active)
            self.voice.on_result(self.result())
            self.voice.tick()
            self.assertEqual(self.voice.status, "ready")
        requests = [call.args[0] for call in self.voice.transcribe.call_async.call_args_list]
        self.assertEqual(len(requests), 7)
        self.assertEqual(len({request.request_id for request in requests}), 7)
        self.assertTrue(all(request.duration_sec == speech.CAPTURE_WINDOW_SECONDS for request in requests))
        self.assertEqual(self.voice.transcripts_received, 7)
        self.assertGreater(self.clock - started, 300)
        self.assertFalse(hasattr(self.voice.args, "record_duration"))
        self.session.controller.stop.assert_not_called()

    def test_silence_is_normal_and_has_no_transcript(self):
        for message in ("no_speech", "no_speech_energy"):
            with self.subTest(message=message):
                self.admit()
                self.voice.on_result(self.result(success=False, text="", message=message))
                self.voice.tick()
                self.assertEqual(self.voice.status, "ready")
                self.assertTrue(self.voice.active)
                self.assertEqual(self.voice.transcripts_received, 0)

    def test_default_output_contains_no_recognized_text(self):
        self.admit()
        self.voice.on_result(self.result(text="PRIVATE_PHRASE"))
        self.voice.tick()
        self.assertNotIn("PRIVATE_PHRASE", str(self.session.node.get_logger().mock_calls))
        self.assertNotIn("PRIVATE_PHRASE", str(self.voice.publisher.publish.mock_calls))

    def test_print_is_opt_in_and_escapes_terminal_controls(self):
        self.session.args.print_transcript = True
        self.admit()
        self.voice.on_result(self.result(text="hola\n\x1b[2J"))
        self.voice.tick()
        line = self.session.node.get_logger().info.call_args.args[0]
        self.assertEqual(json.loads(line.removeprefix("transcript: ")), "hola\n\x1b[2J")
        self.assertNotIn("\x1b", line)

    def test_rejection_disables_only_voice_and_starts_async_release(self):
        self.admit(accepted=False, message="model_missing")
        self.assertEqual(self.voice.status, "disabled_admission_rejected")
        self.assertFalse(self.voice.active)
        self.session.controller.stop.assert_not_called()
        self.session.release_client.call_async.assert_called_once()

    def test_lease_rejection_only_retry_same_uuid_and_bounded(self):
        self.admit(accepted=False, message="lease_unavailable")
        old = self.voice.request_id
        self.clock += 0.11
        self.voice.tick()
        self.assertEqual(self.voice.transcribe.call_async.call_count, 2)
        self.assertEqual(self.voice.request_id, old)
        self.clock += 2
        self.voice.tick()
        self.assertEqual(self.voice.status, "disabled_admission_rejected")

    def test_lost_admission_is_never_retried(self):
        self.start()
        self.clock += 3.1
        self.voice.tick()
        self.assertEqual(self.voice.status, "disabled_admission_timeout")
        for _ in range(5):
            self.voice.tick()
        self.voice.transcribe.call_async.assert_called_once()

    def test_wrong_admission_identity_disables_voice(self):
        self.admit(request_id="foreign")
        self.assertEqual(self.voice.status, "disabled_invalid_admission")

    def test_result_timestamp_and_size_validation(self):
        for kind in ("past", "future", "oversized", "failed", "empty"):
            with self.subTest(kind=kind):
                self.voice.active, self.voice.status, self.voice.result = True, "ready", None
                self.admit()
                result = self.result()
                if kind == "past":
                    result.header.stamp.sec = 9
                elif kind == "future":
                    result.header.stamp.sec = 11
                elif kind == "oversized":
                    result.text = "a" * 8193
                elif kind == "failed":
                    result.success, result.message = False, "inference_failed"
                else:
                    result.text = ""
                self.voice.on_result(result)
                self.voice.tick()
                self.assertFalse(self.voice.active)
                self.assertEqual(self.voice.transcripts_received, 0)

    def test_missing_result_has_bounded_watchdog_not_follow_timeout(self):
        self.admit()
        self.clock += 69
        self.voice.tick()
        self.assertEqual(self.voice.status, "disabled_result_timeout")
        self.session.controller.stop.assert_not_called()

    def test_cancel_freeze_and_late_results_cannot_start_capture(self):
        self.session.cancelled.set()
        self.voice.tick()
        self.voice.transcribe.call_async.assert_not_called()
        self.session.cancelled.clear()
        self.admit()
        self.voice.freeze("cancelled")
        self.voice.on_result(self.result())
        self.voice.tick()
        self.assertIsNone(self.voice.result)
        self.voice.transcribe.call_async.assert_called_once()
        self.assertEqual(self.voice.status, "cancelled")

    def test_lost_or_duplicate_publisher_disables_only_voice(self):
        for count in (0, 2):
            self.voice.active = True
            self.session.node.count_publishers.return_value = count
            self.voice.tick()
            self.assertEqual(self.voice.status, "disabled_provider_lost_or_ambiguous")
        self.session.controller.stop.assert_not_called()

    def test_release_ignores_cancel_and_is_idempotent(self):
        self.session.cancelled.set()
        self.voice._call = MagicMock()
        self.assertEqual(self.voice.release(), speech.SUCCEED)
        self.assertEqual(self.voice.release(), speech.SUCCEED)
        self.voice._call.assert_called_once()
        self.assertTrue(self.voice._call.call_args.kwargs["cleanup"])
        request = self.voice._call.call_args.args[1]
        self.assertEqual((request.capability, request.bond_id), (speech.CAPABILITY, "bond"))
        self.assertTrue(self.voice.released)

    def test_configure_catalog_absence_never_requests_or_releases(self):
        self.voice.requested = self.voice.active = False
        self.voice._call = MagicMock(return_value=SimpleNamespace(interfaces=[]))
        self.assertEqual(self.voice.configure({"bond_id": "bond"}), speech.SUCCEED)
        self.assertEqual(self.voice.status, "disabled_no_interface")
        self.assertFalse(self.voice.requested)
        self.session.release_client.call_async.assert_not_called()

    def test_configure_lost_acquisition_still_releases(self):
        self.voice.requested = self.voice.active = False
        self.voice._call = MagicMock(side_effect=[SimpleNamespace(interfaces=[speech.CAPABILITY]),
                                                 SimpleNamespace(providers=[speech.PROVIDER]),
                                                 RuntimeError("lost reply"), object()])
        self.assertEqual(self.voice.configure({"bond_id": "bond"}), speech.SUCCEED)
        self.assertTrue(self.voice.requested)
        self.assertTrue(self.voice.released)
        self.assertEqual(self.voice.status, "disabled_setup_failed")

    def test_diagnostic_publish_failure_cannot_skip_release(self):
        self.voice.publisher.publish.side_effect = RuntimeError("closed")
        self.voice._call = MagicMock()
        self.assertEqual(self.voice.release(), speech.SUCCEED)
        self.assertTrue(self.voice.released)

    def test_disable_transport_and_logger_errors_do_not_escape_visual_tick(self):
        self.voice.transcribe.call_async.side_effect = RuntimeError("capture transport")
        self.session.release_client.call_async.side_effect = RuntimeError("release transport")
        self.session.node.get_logger().warning.side_effect = RuntimeError("logger closed")
        self.voice.tick()
        self.assertFalse(self.voice.active)
        self.assertFalse(self.voice.released)
        self.assertEqual(self.voice.status, "disabled_transport_error")
        self.session.controller.stop.assert_not_called()

    def test_cancel_before_acquisition_does_not_request_any_capability(self):
        self.session.cancelled.set()
        self.voice.requested = False
        self.voice._call = MagicMock()
        self.assertEqual(self.voice.configure({"bond_id": "bond"}), speech.ABORT)
        self.voice._call.assert_not_called()
        self.assertFalse(self.voice.requested)


class CleanupTest(unittest.TestCase):
    def run_follow(self, *, motion=True, fail_stop=False, fail_voice=False):
        session = MagicMock()
        events = []
        session.frames = session.tracked_frames = session.samples = 0
        session.camera = None
        session.speech.requested, session.speech.released = True, False
        session.speech.status, session.speech.transcripts_received = "ready", 2

        def release_voice():
            events.append("release_voice")
            if fail_voice:
                raise RuntimeError("voice release unavailable")
            session.speech.released = True
            return speech.SUCCEED

        session.speech.release.side_effect = release_voice
        if motion:
            session.motion.requested, session.motion.released = True, False
            session.motion.stop_acknowledged, session.motion.commands_sent = False, 0

            def stop(_board):
                events.append("stop_base")
                if fail_stop:
                    raise RuntimeError("stop unavailable")
                return speech.SUCCEED

            def release_base(_board):
                events.append("release_base")
                session.motion.released = True
                return speech.SUCCEED

            session.motion.stop.side_effect = stop
            session.motion.release.side_effect = release_base
        else:
            session.motion = None

        def finish(board):
            board["released"], board["message"] = True, "cancelled_or_ros_shutdown"
            return follow_mode.APP_FAILED

        output = io.StringIO()
        with ExitStack() as stack:
            stack.enter_context(patch.object(follow_mode.rclpy, "init"))
            stack.enter_context(patch.object(follow_mode.rclpy, "ok", return_value=True))
            stack.enter_context(patch.object(follow_mode.rclpy, "shutdown"))
            stack.enter_context(patch.object(follow_mode.signal, "signal"))
            stack.enter_context(patch.object(follow_mode, "set_ros_loggers"))
            stack.enter_context(patch.object(follow_mode, "Blackboard", side_effect=dict))
            stack.enter_context(patch.object(follow_mode, "FollowSession", return_value=session))
            stack.enter_context(patch.object(follow_mode, "build_state_machine", return_value=finish))
            stack.enter_context(redirect_stdout(output))
            follow_mode.run(SimpleNamespace(robot="nao", enable_motion=motion,
                                           orient_camera=False, transcribe_speech=True))
        return session, events, json.loads(output.getvalue())

    def test_base_stop_precedes_voice_free_even_if_stop_throws(self):
        session, events, result = self.run_follow(fail_stop=True)
        self.assertLess(events.index("stop_base"), events.index("release_voice"))
        self.assertTrue(result["speech_released"])
        self.assertTrue(result["motion_released"])
        session.release_fallback.assert_called_once()
        session.close_perception.assert_called_once()

    def test_voice_free_failure_cannot_skip_base_or_people_cleanup(self):
        session, events, result = self.run_follow(fail_voice=True)
        self.assertEqual(events, ["stop_base", "release_base", "release_voice"])
        self.assertFalse(result["speech_released"])
        self.assertFalse(result["released"])
        self.assertTrue(result["people_released"])
        self.assertTrue(result["motion_released"])
        session.release_fallback.assert_called_once()
        session.close_perception.assert_called_once()

    def test_preview_voice_releases_without_creating_motion(self):
        session, events, result = self.run_follow(motion=False)
        self.assertEqual(events, ["release_voice"])
        self.assertTrue(result["dry_run"])
        self.assertTrue(result["speech_released"])
        self.assertEqual(result["transcripts_received"], 2)
        self.assertIsNone(session.motion)
        self.assertNotIn("text", result)


if __name__ == "__main__":
    unittest.main()

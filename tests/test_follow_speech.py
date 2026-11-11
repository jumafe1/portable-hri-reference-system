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
from hri_capability_interfaces.msg import SpeechCaptureStatus, SpeechTranscript


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
                    success=True, message="transcribed", segment_index=1, final=False)
        data.update(overrides)
        result = SpeechTranscript(**data)
        result.header.stamp.sec = 10
        return result

    def heartbeat(self, **overrides):
        data = dict(request_id=self.voice.request_id, provider=speech.PROVIDER,
                    state="listening", queued_segments=0, dropped_segments=0)
        data.update(overrides)
        status = SpeechCaptureStatus(**data)
        status.header.stamp.sec = 10
        self.voice.on_capture_status(status)

    def test_single_stream_admission_handles_repeated_results_without_blocking(self):
        self.admit()
        self.assertEqual(self.voice.status, "listening")
        for index in range(1, 5):
            self.clock += 1
            self.heartbeat(state="speech_and_processing", queued_segments=1)
            for _ in range(20):
                self.voice.tick()
            self.voice.on_result(self.result(segment_index=index))
            self.voice.tick()
            self.assertEqual(self.voice.transcripts_received, index)
            self.assertEqual(self.voice.status, "speech_and_processing")
        self.voice.transcribe.call_async.assert_called_once()
        request = self.voice.transcribe.call_async.call_args.args[0]
        self.assertEqual((request.duration_sec, request.language, request.continuous, request.silence_sec),
                         (10.0, "es", True, .8))

    def test_wrong_identity_ignored_and_early_result_waits_for_admission(self):
        future = self.start()
        self.voice.on_result(self.result(request_id="foreign"))
        self.voice.on_result(self.result(provider="other"))
        self.assertFalse(self.voice.results)
        self.voice.on_result(self.result())
        self.voice.on_result(self.result(text="replacement"))
        self.voice.tick()
        self.assertEqual(self.voice.transcripts_received, 0)
        self.admit(future)
        self.assertEqual(self.voice.transcripts_received, 1)

    def test_stream_continues_beyond_five_minutes_with_heartbeat(self):
        self.admit()
        for index in range(310):
            self.clock += 1
            self.heartbeat()
            self.voice.tick()
            self.assertTrue(self.voice.active)
        self.voice.on_result(self.result())
        self.voice.tick()
        self.voice.transcribe.call_async.assert_called_once()
        self.assertEqual(self.voice.transcripts_received, 1)
        self.session.controller.stop.assert_not_called()

    def test_silence_is_normal_and_does_not_restart_capture(self):
        self.admit()
        for index, message in enumerate(("no_speech", "no_speech_energy"), 1):
            self.voice.on_result(self.result(success=False, text="", message=message, segment_index=index))
            self.voice.tick()
            self.assertEqual(self.voice.status, "listening")
            self.assertTrue(self.voice.active)
            self.assertEqual(self.voice.transcripts_received, 0)
        self.voice.transcribe.call_async.assert_called_once()

    def test_duplicates_out_of_order_and_queue_are_bounded(self):
        self.admit()
        self.voice.on_result(self.result(segment_index=1))
        self.voice.on_result(self.result(segment_index=1))
        self.voice.on_result(self.result(segment_index=3))
        self.voice.on_result(self.result(segment_index=2))
        self.voice.tick()
        self.voice.tick()
        self.assertEqual(self.voice.transcripts_received, 2)
        for index in range(4, 13):
            self.voice.on_result(self.result(segment_index=index))
        self.voice.tick()
        self.assertEqual(self.voice.status, "disabled_result_queue_overflow")

    def test_foreign_or_stale_heartbeat_does_not_keep_dead_stream_alive(self):
        self.admit()
        self.clock += 3.1
        self.heartbeat(request_id="another-session")
        stale = SpeechCaptureStatus(request_id=self.voice.request_id, provider=speech.PROVIDER, state="listening")
        stale.header.stamp.sec = 1
        self.voice.on_capture_status(stale)
        self.voice.tick()
        self.assertEqual(self.voice.status, "disabled_stream_timeout")

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

    def test_cold_worker_startup_retries_then_automatically_captures(self):
        self.admit(accepted=False, message="whisper_worker_unavailable")
        self.assertEqual(self.voice.status, "waiting_worker")
        original_id = self.voice.request_id
        self.clock += 15
        self.voice.transcribe.call_async.return_value = self.future(done=False)
        self.voice.tick()
        self.assertEqual(self.voice.request_id, original_id)
        self.admit(self.voice.future, accepted=False, message="audio_info_unavailable")
        self.assertEqual(self.voice.status, "waiting_audio_info")
        self.clock += 1
        self.voice.transcribe.call_async.return_value = self.future(done=False)
        self.voice.tick()
        self.admit(self.voice.future)
        self.assertEqual(self.voice.status, "listening")
        self.assertEqual(self.voice.result_deadline, self.clock + 3)
        self.voice.on_result(self.result())
        self.voice.tick()
        self.assertEqual(self.voice.transcripts_received, 1)
        self.assertEqual(self.voice.status, "listening")
        self.session.controller.stop.assert_not_called()

    def test_startup_wait_is_bounded_and_cancellable(self):
        self.admit(accepted=False, message="whisper_worker_unavailable")
        self.session.cancelled.set()
        self.clock += 10
        self.voice.tick()
        self.voice.transcribe.call_async.assert_called_once()
        self.session.cancelled.clear()
        self.clock += 51
        self.voice.tick()
        self.assertEqual(self.voice.status, "disabled_startup_timeout")
        self.voice.transcribe.call_async.assert_called_once()

    def test_ambiguous_admission_timeout_is_not_a_startup_retry(self):
        self.admit(accepted=False, message="worker_admission_timeout")
        self.assertFalse(self.voice.active)
        self.assertEqual(self.voice.status, "disabled_admission_rejected")
        self.voice.transcribe.call_async.assert_called_once()

    def test_no_audio_failure_reports_cause_without_a_capture_timestamp(self):
        self.admit()
        result = self.result(success=False, text="", message="audio_capture_timeout")
        result.header.stamp.sec = 0
        self.voice.on_result(result)
        self.voice.tick()
        self.assertEqual(self.voice.status, "disabled_audio_capture_timeout")
        self.session.controller.stop.assert_not_called()

    def test_wrong_admission_identity_disables_voice(self):
        self.admit(request_id="foreign")
        self.assertEqual(self.voice.status, "disabled_invalid_admission")

    def test_result_timestamp_and_size_validation(self):
        for kind in ("past", "future", "oversized", "failed", "empty"):
            with self.subTest(kind=kind):
                self.voice.active, self.voice.status = True, "ready"
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

    def test_missing_heartbeat_disables_voice_without_stopping_follow(self):
        self.admit()
        self.clock += 69
        self.voice.tick()
        self.assertEqual(self.voice.status, "disabled_stream_timeout")
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
        self.assertFalse(self.voice.results)
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
        session.image_recoveries = 0
        session.person_recoveries = 0
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

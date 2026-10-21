"""Transcription application tests; no microphone, graph or model is started."""

from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import signal
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hri_reference_app/scripts"))
import transcribe_mode as mode
from hri_capability_interfaces.msg import SpeechTranscript


class ArgumentsTest(unittest.TestCase):
    def test_explicit_consent_and_no_print_default(self):
        args = mode.parse_args(["--robot", "nao", "--confirm", "ESCUCHAR"])
        self.assertFalse(args.print_transcript)
        self.assertEqual(args.record_duration, 3)
        for consent in ("MOVER", "", "escuchar"):
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                mode.parse_args(["--robot", "nao", "--confirm", consent])

    def test_bad_durations_timeouts_providers(self):
        for key, value in (("--record-duration", "nan"), ("--record-duration", "10.1"),
                           ("--record-duration", "0"), ("--timeout", "inf"),
                           ("--timeout", "4.9"), ("--timeout", "121"),
                           ("--speech-provider", ""), ("--speech-language", "xx")):
            with self.subTest(key=key, value=value), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                mode.parse_args(["--robot", "nao", "--confirm", "ESCUCHAR", key, value])


class SessionTest(unittest.TestCase):
    def make_session(self):
        session = mode.TranscribeSession.__new__(mode.TranscribeSession)
        session.args = mode.parse_args(["--robot", "nao", "--confirm", "ESCUCHAR"])
        session.cancelled = threading.Event()
        session.node = MagicMock()
        session.node.get_clock().now().nanoseconds = 10_000_000_000
        session.node.count_publishers.return_value = 1
        session.request_id = "owned-token"
        session.bond = MagicMock()
        session.bond.is_broken.return_value = False
        session.result = None
        session.lock = threading.Lock()
        session.transcribe = MagicMock()
        session.call = MagicMock(return_value=SimpleNamespace(
            accepted=True, request_id="owned-token", provider=mode.PROVIDER))
        session.message = "not_started"
        session.clients = {"free_capability": MagicMock()}
        session.attempted, session.released, session.bond_id = True, False, "bond"
        return session

    def transcript(self, **values):
        result = SpeechTranscript(request_id="owned-token", provider=mode.PROVIDER,
                                  text="Hola", language="es", success=True)
        result.header.stamp.sec = 10
        for key, value in values.items():
            setattr(result, key, value)
        return result

    def test_correlation_and_first_completion(self):
        session = self.make_session()
        session.on_result(self.transcript(request_id="someone-else"))
        session.on_result(self.transcript(provider="other/provider"))
        self.assertIsNone(session.result)
        session.on_result(self.transcript())
        session.on_result(self.transcript(text="late replacement"))
        self.assertEqual(session.result.text, "Hola")

    def test_success_and_rejected_or_mismatched_admission(self):
        session = self.make_session()
        session.result = self.transcript()
        with patch.object(mode.rclpy, "ok", return_value=True):
            self.assertEqual(session.listen({}), mode.OK)
            session.call.return_value.accepted = False
            session.call.return_value.message = "model_missing"
            self.assertEqual(session.listen({}), mode.FAIL)
            self.assertEqual(session.message, "model_missing")
            session.call.return_value.accepted = True
            session.call.return_value.request_id = "wrong"
            self.assertEqual(session.listen({}), mode.FAIL)

    def test_failed_empty_old_future_or_ambiguous_results(self):
        for scenario in ("failed", "empty", "old", "future", "publishers", "bond"):
            with self.subTest(scenario=scenario):
                session = self.make_session()
                session.result = self.transcript()
                if scenario == "failed":
                    session.result.success, session.result.message = False, "no_audio"
                elif scenario == "empty":
                    session.result.text = " "
                elif scenario == "old":
                    session.result.header.stamp.sec = 9
                elif scenario == "future":
                    session.result.header.stamp.sec = 11
                elif scenario == "publishers":
                    session.node.count_publishers.return_value = 2
                else:
                    session.bond.is_broken.return_value = True
                with patch.object(mode.rclpy, "ok", return_value=True):
                    self.assertEqual(session.listen({}), mode.FAIL)

    def test_cancel_and_cleanup_is_not_cancelled(self):
        session = self.make_session()
        session.cancelled.set()
        with patch.object(mode.rclpy, "ok", return_value=True):
            self.assertEqual(session.listen({}), mode.FAIL)
            self.assertEqual(session.message, "cancelled")
        self.assertEqual(session.release(), mode.OK)
        self.assertTrue(session.released)
        self.assertTrue(session.call.call_args.kwargs["cleanup"])
        session.call.reset_mock()
        self.assertEqual(session.release(), mode.OK)
        session.call.assert_not_called()

    def test_release_failure_reported(self):
        session = self.make_session()
        session.call.side_effect = RuntimeError("lost")
        self.assertEqual(session.release(), mode.FAIL)
        self.assertFalse(session.released)

    def test_cancel_before_ready_service_never_submits_capture(self):
        session = self.make_session()
        session.cancelled.set()
        with patch.object(mode.rclpy, "ok", return_value=True):
            with self.assertRaisesRegex(RuntimeError, "cancelled"):
                mode.TranscribeSession.call(session, session.transcribe, object())
        session.transcribe.call_async.assert_not_called()
        self.assertEqual(session.listen({}), mode.FAIL)
        session.call.assert_not_called()

    def test_explicit_lease_rejection_retries_same_token(self):
        session = self.make_session()
        session.result = self.transcript()
        accepted = session.call.return_value
        session.call.side_effect = [SimpleNamespace(accepted=False, message="lease_unavailable"), accepted]
        with patch.object(mode.rclpy, "ok", return_value=True):
            self.assertEqual(session.listen({}), mode.OK)
        self.assertEqual(session.call.call_count, 2)
        self.assertIs(session.call.call_args_list[0].args[1], session.call.call_args_list[1].args[1])

    def test_admission_timeout_is_never_retried(self):
        session = self.make_session()
        session.call.side_effect = RuntimeError("service_timeout")
        self.assertEqual(session.listen({}), mode.FAIL)
        session.call.assert_called_once()


class CleanupTest(unittest.TestCase):
    def test_cleanup_exception_closes_and_does_not_print_text(self):
        session = MagicMock()
        session.message, session.released = "done", False
        session.result.text = "sensitive text"
        session.release.side_effect = RuntimeError("cleanup failure")
        output = io.StringIO()
        with ExitStack() as stack:
            stack.enter_context(patch.object(mode.rclpy, "init"))
            stack.enter_context(patch.object(mode.rclpy, "try_shutdown"))
            stack.enter_context(patch.object(mode.signal, "signal", return_value=signal.SIG_DFL))
            stack.enter_context(patch.object(mode, "TranscribeSession", return_value=session))
            stack.enter_context(patch.object(mode, "build_state_machine", return_value=lambda _: "application_succeeded"))
            stack.enter_context(redirect_stdout(output))
            code = mode.run(mode.parse_args(["--robot", "nao", "--confirm", "ESCUCHAR"]))
        self.assertEqual(code, 1)
        session.close.assert_called_once()
        result = json.loads(output.getvalue())
        self.assertNotIn("text", result)
        self.assertIn("cleanup_failed", result["message"])


if __name__ == "__main__":
    unittest.main()

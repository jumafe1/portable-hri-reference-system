"""Command policy and nonblocking stop/Speak/resume sequencing, without a robot."""
from concurrent.futures import Future
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'hri_reference_app/scripts'))
from voice_command_policy import CommandPolicy
from voice_commands import VoiceCommands
from follow_controller import Decision

CONFIG = ROOT / 'hri_reference_app/config/voice_commands.yaml'


class VocabularyTest(unittest.TestCase):
    def setUp(self):
        self.policy = CommandPolicy(CONFIG)

    def test_synonyms_case_accents_and_punctuation(self):
        for phrase in ('Nao, detente.', 'NAO QUÉDATE QUIETO', 'nao no me sigas', 'nao no vengas conmigo'):
            self.assertEqual(self.policy.intent(phrase), 'pause')
        for phrase in ('Nao, sígueme', 'nao ven conmigo', 'NAO VUELVE A SEGUIRME'):
            self.assertEqual(self.policy.intent(phrase), 'follow')

    def test_no_substrings_unknown_tokens_or_implicit_wake(self):
        for phrase in ('sígueme', 'no me sigas', 'nao', 'nao no sígueme', 'nao no me gusta ese color',
                       'nao sígueme detente', 'dijo nao sígueme', '[unk] nao sígueme',
                       'nao [unk] sígueme', 'hola nao sígueme', 'te voy a seguir'):
            self.assertIsNone(self.policy.intent(phrase), phrase)


class ControlTest(unittest.TestCase):
    def setUp(self):
        self.now, self.ros = 10., 100_000_000_000
        self.clock = patch('voice_commands.time.monotonic', side_effect=lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.session = MagicMock()
        self.session.args = SimpleNamespace(voice_commands_config=str(CONFIG), speak_provider='test/Speak', speak_language='Spanish')
        self.session.cancelled = threading.Event()
        self.session.controller.halted = False
        self.session.node.get_clock().now.side_effect = lambda: SimpleNamespace(nanoseconds=self.ros)
        self.session.speech = SimpleNamespace(active=True, has_captured=True, closed=False)
        self.session._base_stop_acknowledged.return_value = False
        self.session.latest_stamp = self.ros
        self.voice = VoiceCommands(self.session)
        self.voice.speak = MagicMock()
        self.future = Future()
        self.voice.speak.call_async.return_value = self.future
        self.decision = Decision(.1, .1, 'preview_following', True)

    def advance(self, seconds=.1):
        self.now += seconds
        self.ros += int(seconds * 1e9)

    def issue(self, text):
        self.voice.receive(text, self.ros - 100_000_000)
        self.voice.tick()

    def ack_stop(self):
        self.voice.sent(self.voice.gate(self.decision))
        self.advance(.5)
        self.session._base_stop_acknowledged.return_value = True
        self.voice.tick()

    def finish_reply(self, success=True):
        self.future.set_result(SimpleNamespace(success=success, provider='test/Speak'))
        self.voice.tick()

    def test_starts_paused_and_vision_alone_cannot_resume(self):
        for _ in range(5):
            self.advance()
            self.assertEqual(self.voice.gate(self.decision).vx, 0)
        self.assertTrue(self.voice.paused)

    def test_pause_is_latched_before_response_or_ack(self):
        self.voice.paused = False
        self.issue('nao detente')
        self.assertTrue(self.voice.paused)
        self.assertEqual(self.voice.gate(self.decision).vx, 0)
        self.voice.speak.call_async.assert_not_called()
        self.ack_stop()
        self.voice.speak.call_async.assert_called_once()
        self.finish_reply()
        self.advance(2)
        self.assertTrue(self.voice.paused)
        self.assertEqual(self.voice.gate(self.decision).wz, 0)

    def test_follow_waits_for_stop_reply_echo_guard_and_three_new_frames(self):
        self.issue('nao sígueme')
        self.assertEqual(self.voice.gate(self.decision).vx, 0)
        self.ack_stop()
        self.assertEqual(self.voice.gate(self.decision).vx, 0)
        self.finish_reply()
        self.assertEqual(self.voice.gate(self.decision).vx, 0)
        self.advance(1.1)
        for count in range(3):
            self.session.latest_stamp = self.ros
            d = self.voice.gate(self.decision)
            self.assertEqual(d.vx, .1 if count == 2 else 0)
            # Repeated control ticks on the same frame do not count.
            self.assertEqual(self.voice.gate(self.decision).vx, d.vx)
            self.advance()

    def test_missing_or_ambiguous_vision_still_blocks_after_follow(self):
        self.issue('nao sígueme')
        self.ack_stop()
        self.finish_reply()
        self.advance(2)
        for _ in range(6):
            self.session.latest_stamp = self.ros
            self.assertEqual(self.voice.gate(Decision(0, 0, 'ambiguous_people', True)).vx, 0)
            self.advance()
        self.assertEqual(self.voice.resume_frames, 0)

    def test_old_future_duplicate_and_self_speech_results_are_ignored(self):
        for stamp in (self.ros - 6_000_000_000, self.ros + 100):
            self.voice.receive('nao sígueme', stamp)
        self.assertEqual(self.voice.accepted_commands, 0)
        self.issue('nao detente')
        self.voice.receive('nao sígueme', self.ros - 100_000_000)
        self.assertEqual(self.voice.pending, 'pause')
        self.ack_stop()
        self.voice.receive('nao sígueme', self.ros)
        self.assertEqual(self.voice.pending, 'pause')
        echo_stamp = self.ros
        self.finish_reply()
        self.advance(2)
        self.voice.receive('nao sígueme', echo_stamp)
        self.assertTrue(self.voice.paused)
        self.assertEqual(self.voice.accepted_commands, 1)

    def test_unknown_does_not_change_follow_state(self):
        self.voice.paused = False
        self.issue('nao no me gusta ese color')
        self.assertFalse(self.voice.paused)
        self.voice.speak.call_async.assert_not_called()

    def test_stop_supersedes_follow_before_response(self):
        self.issue('nao sígueme')
        self.advance(.2)
        self.issue('nao no me sigas')
        self.assertEqual(self.voice.pending, 'pause')

    def test_failed_reply_never_resumes(self):
        self.issue('nao sígueme')
        self.ack_stop()
        self.finish_reply(False)
        self.advance(2)
        self.assertTrue(self.voice.paused)
        self.assertEqual(self.voice.gate(self.decision).vx, 0)

    def test_timeout_and_late_success_never_resume(self):
        self.issue('nao sígueme')
        self.ack_stop()
        self.advance(8)
        self.voice.tick()
        self.assertEqual(self.voice.fault, 'voice_response_timeout')
        self.finish_reply()
        self.assertTrue(self.voice.paused)

    def test_recognizer_loss_latches_stop_and_recovery_does_not_resume(self):
        self.voice.paused = False
        self.session.speech.active = False
        self.voice.tick()
        self.assertTrue(self.voice.paused)
        self.assertEqual(self.voice.gate(self.decision).vx, 0)
        self.session.speech.active = True
        self.voice.tick()
        self.assertTrue(self.voice.paused)

    def test_configured_unpaused_start_waits_for_recognizer_admission(self):
        self.voice.paused = False
        self.session.speech.has_captured = False
        self.voice.tick()
        self.assertEqual(self.voice.gate(self.decision).vx, 0)
        self.assertFalse(self.voice.paused)
        self.session.speech.has_captured = True
        self.voice.tick()
        self.assertEqual(self.voice.gate(self.decision).vx, .1)

    def test_fatal_visual_fault_is_not_masked(self):
        self.session.controller.halted = True
        failure = Decision(0, 0, 'unexpected_camera_frame', False)
        self.assertIs(self.voice.gate(failure), failure)

    def test_cancel_does_not_start_a_pending_reply(self):
        self.issue('nao sígueme')
        self.session.cancelled.set()
        self.session._base_stop_acknowledged.return_value = True
        self.voice.tick()
        self.voice.speak.call_async.assert_not_called()
        self.assertTrue(self.voice.paused)

if __name__ == '__main__':
    unittest.main()

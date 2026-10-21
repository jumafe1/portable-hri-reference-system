"""Standalone startup handshake regression; mocks only, no ROS initialization."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hri_reference_app/scripts"))

from velocity_mode import ABORT, CAPABILITY, DEFAULT_PROVIDER, SUCCEED, VelocitySession


class VelocityStartupTest(unittest.TestCase):
    def setUp(self):
        self.session = VelocitySession.__new__(VelocitySession)
        self.session.node = MagicMock()
        self.session.node.count_publishers.return_value = 1
        self.session.publisher = MagicMock()
        self.session.publisher.get_subscription_count.return_value = 1
        self.session.cancelled = MagicMock()
        self.session.cancelled.is_set.return_value = False
        self.session.args = SimpleNamespace(provider=DEFAULT_PROVIDER, duration=0.1,
                                            vx=0.03, vy=0.0, wz=0.0)
        self.session.running_client = MagicMock()
        future = self.session.running_client.call_async.return_value
        future.done.return_value = True
        future.exception.return_value = None
        future.result.return_value = SimpleNamespace(running_capabilities=[SimpleNamespace(
            capability=SimpleNamespace(capability=CAPABILITY, provider=DEFAULT_PROVIDER))])
        self.session.acquired = False
        self.clock = 100.0
        self.session.status = ("ready", self.clock)
        self.board = {"message": "", "commands_sent": 0, "stop_acknowledged": False}
        for patcher in (patch("velocity_mode.rclpy.ok", return_value=True),
                        patch("velocity_mode.time.monotonic", side_effect=lambda: self.clock)):
            patcher.start()
            self.addCleanup(patcher.stop)

    def assert_only_zero(self):
        for call in self.session.publisher.publish.call_args_list:
            command = call.args[0]
            self.assertEqual((command.linear.x, command.linear.y, command.linear.z,
                              command.angular.x, command.angular.y, command.angular.z), (0.0,) * 6)

    def advance_with_ack(self, seconds):
        self.clock += seconds
        status = "stopped" if self.session.publisher.publish.called else "ready"
        self.session.status = (status, self.clock)

    def test_short_duration_starts_after_confirmed_zero_handshake(self):
        with patch("velocity_mode.time.sleep", side_effect=self.advance_with_ack):
            self.assertEqual(self.session.wait_ready(self.board), SUCCEED)
        self.assertGreaterEqual(self.clock, 101.0)
        self.assertTrue(self.session.acquired)
        self.assert_only_zero()
        self.session.publisher.publish.reset_mock()
        started = self.clock
        self.session.cancelled.wait.side_effect = self.advance_with_ack
        with patch("velocity_mode.time.sleep", side_effect=self.advance_with_ack):
            self.assertEqual(self.session.stream(self.board), SUCCEED)
        self.assertGreaterEqual(self.clock, started + self.session.args.duration)
        self.assertGreater(self.board["commands_sent"], 0)
        self.assertTrue(any(call.args[0].linear.x == 0.03
                            for call in self.session.publisher.publish.call_args_list))
        self.assertTrue(self.board["stop_acknowledged"])

    def test_ready_without_ack_cannot_arm(self):
        def advance(seconds):
            self.clock += seconds
            self.session.status = ("ready", self.clock)

        with patch("velocity_mode.time.sleep", side_effect=advance):
            self.assertEqual(self.session.wait_ready(self.board), ABORT)
        self.assertFalse(self.session.acquired)
        self.assertTrue(self.session.publisher.publish.called)
        self.assert_only_zero()

    def test_old_stopped_cannot_ack_startup(self):
        self.session.status = ("stopped", self.clock)

        def advance(seconds):
            self.clock += seconds
            if not self.session.publisher.publish.called:
                self.session.status = ("stopped", self.clock)

        with patch("velocity_mode.time.sleep", side_effect=advance):
            self.assertEqual(self.session.wait_ready(self.board), ABORT)
        self.assertFalse(self.session.acquired)
        self.assert_only_zero()

    def test_invalid_delivery_source_aborts_during_handshake(self):
        def advance(seconds):
            self.clock += seconds
            status = "invalid_velocity_source" if self.session.publisher.publish.called else "ready"
            self.session.status = (status, self.clock)

        with patch("velocity_mode.time.sleep", side_effect=advance):
            self.assertEqual(self.session.wait_ready(self.board), ABORT)
        self.assertFalse(self.session.acquired)
        self.assertEqual(self.board["message"], "invalid_velocity_source")
        self.assert_only_zero()


if __name__ == "__main__":
    unittest.main()

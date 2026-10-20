"""Opt-in follow motion validation; mocks only, never initializes ROS or a robot."""

from contextlib import redirect_stderr
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hri_reference_app/scripts"))

from follow_mode import parse_args
from follow_motion import ABORT, CAPABILITY, COMMAND_TOPIC, PROVIDER, STATUS_TOPIC, SUCCEED, FollowMotion


class FollowMotionArgumentsTest(unittest.TestCase):
    BASE = ["--robot", "nao", "--image-width", "320", "--image-height", "240",
            "--expected-frame", "test_camera"]

    def test_default_remains_preview(self):
        args = parse_args(self.BASE)
        self.assertFalse(args.enable_motion)
        self.assertEqual(args.duration, 2.0)
        self.assertEqual(args.timeout, 120.0)
        self.assertEqual(args.max_forward_speed, 0.03)

    def test_configured_speed_is_accepted_without_enabling_motion(self):
        for maximum in ("0.01", "0.03", "0.05", "0.1"):
            with self.subTest(maximum=maximum):
                args = parse_args(self.BASE + ["--max-forward-speed", maximum])
                self.assertEqual(args.max_forward_speed, float(maximum))
                self.assertFalse(args.enable_motion)

    def test_invalid_speed_is_rejected_before_motion(self):
        for maximum in ("0", "-0.1", "0.10001", "nan", "inf", "-inf", "true"):
            with self.subTest(maximum=maximum), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_args(self.BASE + ["--max-forward-speed", maximum])

    def test_confirmation_alone_does_not_enable_motion(self):
        args = parse_args(self.BASE + ["--confirm", "MOVER"])
        self.assertFalse(args.enable_motion)

    def test_explicit_opt_in_and_confirmation(self):
        args = parse_args(self.BASE + ["--enable-motion", "--confirm", "MOVER", "--duration", "2"])
        self.assertTrue(args.enable_motion)
        self.assertEqual(args.duration, 2.0)

    def test_motion_without_exact_confirmation_is_rejected(self):
        for confirmation in ([], ["--confirm", "mover"], ["--confirm", "YES"]):
            with self.subTest(confirmation=confirmation), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_args(self.BASE + ["--enable-motion"] + confirmation)

    def test_invalid_motion_duration_is_rejected(self):
        for duration in ("0", "-1", "0.09", "300.01", "301", "nan", "inf", "-inf"):
            with self.subTest(duration=duration), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_args(self.BASE + ["--enable-motion", "--confirm", "MOVER",
                                            "--duration", duration])

    def test_bounded_duration_endpoints_are_accepted(self):
        for duration in ("0.1", "5.0", "120.0", "300.0"):
            with self.subTest(duration=duration):
                args = parse_args(self.BASE + ["--enable-motion", "--confirm", "MOVER",
                                              "--duration", duration])
                self.assertEqual(args.duration, float(duration))

    def test_bounded_timeout_endpoints_are_accepted(self):
        for timeout in ("1", "120", "300"):
            with self.subTest(timeout=timeout):
                args = parse_args(self.BASE + ["--timeout", timeout])
                self.assertEqual(args.timeout, float(timeout))
                self.assertFalse(args.enable_motion)

    def test_invalid_timeout_is_rejected(self):
        for timeout in ("0", "0.99", "300.01", "301", "nan", "inf", "-inf"):
            with self.subTest(timeout=timeout), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_args(self.BASE + ["--timeout", timeout])


class FollowMotionTransportTest(unittest.TestCase):
    def setUp(self):
        self.session = MagicMock()
        self.session.cancelled.is_set.return_value = False
        self.session.node.count_publishers.return_value = 1
        self.session.await_future.return_value = True
        self.session.controller.config.max_forward_speed = 0.03
        self.motion = FollowMotion(self.session)
        self.motion.publisher.get_subscription_count.return_value = 1
        self.clock = 100.0
        self.motion.status = ("ready", self.clock)
        self.clock_patch = patch("follow_motion.time.monotonic", side_effect=lambda: self.clock)
        self.ok_patch = patch("follow_motion.rclpy.ok", return_value=True)
        self.clock_patch.start()
        self.ok_patch.start()
        self.addCleanup(self.clock_patch.stop)
        self.addCleanup(self.ok_patch.stop)
        self.board = {"bond_id": "test-bond", "message": "test"}

    def decision(self, vx=0.02, wz=-0.08):
        return SimpleNamespace(vx=vx, wz=wz)

    def test_request_marks_cleanup_needed_even_before_response(self):
        request = self.motion.request(self.board)
        self.assertTrue(self.motion.requested)
        self.assertEqual(request.capability, CAPABILITY)
        self.assertEqual(request.preferred_provider, PROVIDER)
        self.assertEqual(request.bond_id, "test-bond")

    def test_not_armed_never_publishes(self):
        self.assertEqual(self.motion.send(self.decision()), "motion_not_armed_or_cancelled")
        self.motion.publisher.publish.assert_not_called()

    def test_armed_send_routes_only_forward_and_yaw(self):
        self.motion.acquired = True
        self.assertEqual(self.motion.send(self.decision()), "")
        command = self.motion.publisher.publish.call_args.args[0]
        self.assertEqual(command.linear.x, 0.02)
        self.assertEqual(command.angular.z, -0.08)
        self.assertEqual((command.linear.y, command.linear.z,
                          command.angular.x, command.angular.y), (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(self.motion.commands_sent, 1)

    def test_released_and_cancelled_sessions_never_publish(self):
        self.motion.acquired = True
        self.motion.released = True
        self.assertEqual(self.motion.send(self.decision()), "motion_not_armed_or_cancelled")
        self.motion.released = False
        self.session.cancelled.is_set.return_value = True
        self.assertEqual(self.motion.send(self.decision()), "motion_not_armed_or_cancelled")
        self.motion.publisher.publish.assert_not_called()

    def test_invalid_velocity_values_never_publish(self):
        self.motion.acquired = True
        for vx, wz in ((-0.001, 0.0), (0.03001, 0.0), (0.0, 0.10001),
                       (0.0, -0.10001), (float("nan"), 0.0), (0.0, float("inf"))):
            with self.subTest(vx=vx, wz=wz):
                self.assertEqual(self.motion.send(self.decision(vx, wz)), "invalid_follow_velocity")
        self.motion.publisher.publish.assert_not_called()

    def test_configured_speed_is_accepted_but_never_exceeded(self):
        self.motion.acquired = True
        for maximum in (0.05, 0.1):
            with self.subTest(maximum=maximum):
                self.session.controller.config.max_forward_speed = maximum
                self.assertEqual(self.motion.send(self.decision(maximum, 0.0)), "")
                self.assertEqual(self.motion.publisher.publish.call_args.args[0].linear.x, maximum)
                sent = self.motion.commands_sent
                self.assertEqual(self.motion.send(self.decision(maximum + 0.00001, 0.0)),
                                 "invalid_follow_velocity")
                self.assertEqual(self.motion.commands_sent, sent)

    def test_absolute_transport_limit_survives_invalid_configuration(self):
        self.motion.acquired = True
        self.session.controller.config.max_forward_speed = 0.3
        self.assertEqual(self.motion.send(self.decision(0.11, 0.0)), "invalid_follow_velocity")
        self.motion.publisher.publish.assert_not_called()

    def test_stale_or_faulted_status_never_publishes(self):
        self.motion.acquired = True
        for status, stamp in (("ready", 99.5), ("moving", 99.0), ("fault", 100.0), ("", 100.0)):
            with self.subTest(status=status, stamp=stamp):
                self.motion.status = (status, stamp)
                self.assertEqual(self.motion.send(self.decision()), "motion_status_lost_or_faulted")
        self.motion.publisher.publish.assert_not_called()

    def test_duplicate_or_missing_command_publishers_never_publish(self):
        self.motion.acquired = True
        for count in (0, 2):
            self.session.node.count_publishers.side_effect = lambda topic: count if topic == COMMAND_TOPIC else 1
            self.assertEqual(self.motion.send(self.decision()), "multiple_or_missing_velocity_publishers")
        self.motion.publisher.publish.assert_not_called()

    def test_ambiguous_status_or_command_consumers_never_publish(self):
        self.motion.acquired = True
        self.session.node.count_publishers.side_effect = lambda topic: 2 if topic == STATUS_TOPIC else 1
        self.assertEqual(self.motion.send(self.decision()), "multiple_or_missing_velocity_status_publishers")
        self.session.node.count_publishers.side_effect = None
        self.motion.publisher.get_subscription_count.return_value = 2
        self.assertEqual(self.motion.send(self.decision()), "multiple_or_missing_velocity_consumers")
        self.motion.publisher.publish.assert_not_called()

    def wait_ready_fixture(self):
        capability = SimpleNamespace(capability=CAPABILITY, provider=PROVIDER)
        self.session.running_client.call_async.return_value.result.return_value = SimpleNamespace(
            running_capabilities=[SimpleNamespace(capability=capability)])

    def test_ready_handshake_requires_half_second_of_stable_graph(self):
        self.wait_ready_fixture()

        def advance(seconds):
            self.clock += seconds
            self.motion.status = ("ready", self.clock)

        with patch("follow_motion.time.sleep", side_effect=advance):
            self.assertEqual(self.motion.wait_ready(self.board), SUCCEED)
        self.assertGreaterEqual(self.clock, 100.5)
        self.assertTrue(self.motion.acquired)
        self.motion.publisher.publish.assert_not_called()

    def test_wrong_running_provider_cannot_arm(self):
        self.session.running_client.call_async.return_value.result.return_value = SimpleNamespace(
            running_capabilities=[])
        self.assertEqual(self.motion.wait_ready(self.board), ABORT)
        self.assertFalse(self.motion.acquired)
        self.assertEqual(self.board["message"], "motion_provider_not_running")

    def test_provider_fault_cannot_arm(self):
        self.wait_ready_fixture()
        self.motion.status = ("multiple_velocity_publishers", self.clock)
        self.assertEqual(self.motion.wait_ready(self.board), ABORT)
        self.assertFalse(self.motion.acquired)

    def test_stop_sends_only_zero_even_after_cancel_and_requires_new_ack(self):
        self.motion.acquired = True
        self.session.controller.config.max_forward_speed = 0.1
        self.assertEqual(self.motion.send(self.decision(0.1, 0.0)), "")
        self.motion.publisher.publish.reset_mock()
        self.session.cancelled.is_set.return_value = True
        self.assertEqual(self.motion.send(self.decision(0.1, 0.0)), "motion_not_armed_or_cancelled")

        def advance(seconds):
            self.clock += seconds
            self.motion.status = ("stopped", self.clock)

        with patch("follow_motion.time.sleep", side_effect=advance):
            self.assertEqual(self.motion.stop(self.board), SUCCEED)
        self.assertTrue(self.motion.stop_acknowledged)
        self.assertGreaterEqual(self.motion.publisher.publish.call_count, 2)
        for call in self.motion.publisher.publish.call_args_list:
            command = call.args[0]
            self.assertEqual((command.linear.x, command.linear.y, command.linear.z,
                              command.angular.x, command.angular.y, command.angular.z), (0.0,) * 6)

    def test_old_stop_status_is_not_an_acknowledgement(self):
        self.motion.acquired = True
        self.motion.status = ("stopped", 99.9)

        def advance(seconds):
            self.clock += seconds

        with patch("follow_motion.time.sleep", side_effect=advance):
            self.assertEqual(self.motion.stop(self.board), ABORT)
        self.assertFalse(self.motion.stop_acknowledged)
        self.assertIn("motion_stop_not_acknowledged", self.board["message"])

    def test_stop_ack_cannot_latch_before_a_later_provider_fault(self):
        self.motion.acquired = True

        def advance(seconds):
            self.clock += seconds
            self.motion.status = ("stopped" if self.clock < 100.2 else "fault", self.clock)

        with patch("follow_motion.time.sleep", side_effect=advance):
            self.assertEqual(self.motion.stop(self.board), ABORT)
        self.assertFalse(self.motion.stop_acknowledged)

    def test_stop_ack_requires_unambiguous_final_graph(self):
        self.motion.acquired = True

        def advance(seconds):
            self.clock += seconds
            self.motion.status = ("stopped", self.clock)
            if self.clock >= 100.2:
                self.session.node.count_publishers.return_value = 2

        with patch("follow_motion.time.sleep", side_effect=advance):
            self.assertEqual(self.motion.stop(self.board), ABORT)
        self.assertFalse(self.motion.stop_acknowledged)

    def test_stop_without_acquisition_never_publishes(self):
        self.assertEqual(self.motion.stop(self.board), SUCCEED)
        self.motion.publisher.publish.assert_not_called()

    def test_release_targets_motion_and_is_idempotent(self):
        self.motion.requested = self.motion.acquired = True
        self.assertEqual(self.motion.release(self.board), SUCCEED)
        self.assertEqual(self.motion.release(self.board), SUCCEED)
        self.session.release_client.call_async.assert_called_once()
        request = self.session.release_client.call_async.call_args.args[0]
        self.assertEqual((request.capability, request.bond_id), (CAPABILITY, "test-bond"))
        self.assertTrue(self.motion.released)
        self.assertFalse(self.motion.acquired)

    def test_release_failure_is_not_reported_as_released(self):
        self.motion.requested = True
        self.session.await_future.return_value = False
        self.assertEqual(self.motion.release(self.board), ABORT)
        self.assertFalse(self.motion.released)
        self.assertIn("motion_release_failed", self.board["message"])

    def test_unrequested_release_has_no_side_effects(self):
        self.assertEqual(self.motion.release(self.board), SUCCEED)
        self.session.release_client.call_async.assert_not_called()


if __name__ == "__main__":
    unittest.main()

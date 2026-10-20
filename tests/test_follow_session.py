"""Deterministic follow adapter regressions; requires sourced ROS interfaces.

No ROS initialization, live graph, network, driver or robot is used. The real
controller and generated message classes run against mocked session endpoints.
"""

from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hri_reference_app/scripts"))

from hri_capability_interfaces.msg import PersonDetection2D, PersonDetection2DArray
from follow_controller import Config, Controller, Person
from follow_session import ABORT, PROVIDER, SUCCEED, FollowSession


class FollowSessionTest(unittest.TestCase):
    def setUp(self):
        self.session = FollowSession.__new__(FollowSession)
        self.session.args = SimpleNamespace(expected_frame="camera_optical", timeout=0.1)
        self.session.cancelled = MagicMock()
        self.session.cancelled.is_set.return_value = False
        self.session.lock = threading.Lock()
        self.session.controller = Controller(Config(640, 480))
        self.session.node = MagicMock()
        self.session.node.get_clock.return_value.now.return_value.nanoseconds = 100_000_000_000
        self.session.node.count_publishers.return_value = 1
        self.session.bond = MagicMock()
        self.session.bond.is_broken.return_value = False
        self.session.frames = 0
        self.session.tracked_frames = 0
        self.session.samples = 0
        self.session.latest_stamp = 0
        self.session.latest_received = 0.0
        self.session.finished = False
        self.session.motion = None
        self.session.publish_preview = MagicMock()

    def frame(self):
        frame = PersonDetection2DArray()
        frame.header.stamp.sec = 100
        frame.header.frame_id = "camera_optical"
        frame.provider = PROVIDER
        frame.people = [PersonDetection2D(
            confidence=0.9, tracking_id="person-1", center_x=320.0,
            center_y=240.0, width=100.0, height=120.0,
        )]
        return frame

    def acquire(self):
        person = Person(0.9, "person-1", 320.0, 240.0, 100.0, 120.0)
        for now in (0.1, 0.2, 0.3):
            self.session.controller.update([person], now)
        self.session.tracked_frames = 1

    def test_constructor_owns_one_single_threaded_ros_executor(self):
        args = SimpleNamespace(image_width=640, image_height=480,
                               minimum_confidence=0.65, target_height_ratio=0.6)
        with patch("follow_session.Node") as node_type, \
                patch("follow_session.SingleThreadedExecutor") as executor_type, \
                patch("follow_session.threading.Thread") as thread_type:
            session = FollowSession(args, threading.Event())
        node = node_type.return_value
        node_type.assert_called_once_with("portable_hri_follow_app")
        executor_type.assert_called_once_with(context=node.context)
        executor_type.return_value.add_node.assert_called_once_with(node)
        thread_type.assert_called_once_with(target=executor_type.return_value.spin,
                                            name="follow-ros", daemon=True)
        thread_type.return_value.start.assert_called_once_with()
        node.create_subscription.assert_called_once()
        subscription_call = node.create_subscription.call_args
        self.assertIs(subscription_call.args[0], PersonDetection2DArray)
        self.assertEqual(subscription_call.args[1], "/hri/people")
        self.assertEqual(subscription_call.args[2], session.on_frame)
        self.assertIs(session.node, node)
        self.assertIs(session.executor, executor_type.return_value)
        self.assertIs(session.spin_thread, thread_type.return_value)
        self.assertEqual(node.create_client.call_count, 2)
        self.assertIsNone(session.motion)
        self.assertEqual(session.controller.config.max_forward_speed, 0.03)
        node.create_publisher.assert_called_once()
        self.assertEqual(node.create_publisher.call_args.args[1], "/hri/follow_person/preview")

    def test_motion_transport_is_created_only_after_explicit_opt_in(self):
        args = SimpleNamespace(image_width=640, image_height=480,
                               minimum_confidence=0.65, target_height_ratio=0.6,
                               enable_motion=True, max_forward_speed=0.1)
        with patch("follow_session.Node"), \
                patch("follow_session.SingleThreadedExecutor"), \
                patch("follow_session.threading.Thread"), \
                patch("follow_session.FollowMotion") as motion_type:
            session = FollowSession(args, threading.Event())
        motion_type.assert_called_once_with(session)
        self.assertIs(session.motion, motion_type.return_value)
        self.assertEqual(session.controller.config.max_forward_speed, 0.1)

    def test_preview_propagates_configured_speed_without_creating_motion_transport(self):
        args = SimpleNamespace(image_width=640, image_height=480,
                               minimum_confidence=0.65, target_height_ratio=0.6,
                               enable_motion=False, max_forward_speed=0.1)
        with patch("follow_session.Node"), \
                patch("follow_session.SingleThreadedExecutor"), \
                patch("follow_session.threading.Thread"), \
                patch("follow_session.FollowMotion") as motion_type:
            session = FollowSession(args, threading.Event())
        self.assertEqual(session.controller.config.max_forward_speed, 0.1)
        motion_type.assert_not_called()
        person = Person(0.9, "", 320.0, 240.0, 100.0, 120.0)
        for now in (0.1, 0.2, 0.3):
            decision = session.controller.update([person], now)
        self.assertEqual(decision.vx, 0.1)

    def test_constructor_cleans_owned_node_if_subscription_creation_fails(self):
        args = SimpleNamespace(image_width=640, image_height=480,
                               minimum_confidence=0.65, target_height_ratio=0.6)
        with patch("follow_session.Node") as receiver_type, \
                patch("follow_session.SingleThreadedExecutor") as executor_type, \
                patch("follow_session.threading.Thread") as thread_type:
            receiver_type.return_value.create_subscription.side_effect = RuntimeError("subscription failed")
            with self.assertRaisesRegex(RuntimeError, "subscription failed"):
                FollowSession(args, threading.Event())
        executor_type.return_value.shutdown.assert_called_once_with(timeout_sec=2.0)
        receiver_type.return_value.destroy_node.assert_called_once_with()
        thread_type.assert_not_called()

    def configure_perception_cleanup(self):
        self.session.ros_closed = False
        self.session.executor = MagicMock()
        self.session.executor.shutdown.return_value = True
        self.session.spin_thread = MagicMock()
        self.session.spin_thread.is_alive.return_value = False
        self.session.node = MagicMock()

    def test_perception_cleanup_stops_executor_before_destroy_and_is_idempotent(self):
        self.configure_perception_cleanup()
        events = MagicMock()
        events.attach_mock(self.session.executor.shutdown, "shutdown")
        events.attach_mock(self.session.spin_thread.join, "join")
        events.attach_mock(self.session.node.destroy_node, "destroy")
        self.session.close_perception()
        self.session.close_perception()
        self.assertEqual([call[0] for call in events.mock_calls], ["shutdown", "join", "destroy"])
        self.session.executor.shutdown.assert_called_once_with(timeout_sec=2.0)
        self.session.spin_thread.join.assert_called_once_with(timeout=2.0)
        self.assertTrue(self.session.ros_closed)

    def test_perception_cleanup_retains_node_if_thread_alive(self):
        self.configure_perception_cleanup()
        self.session.spin_thread.is_alive.return_value = True
        with self.assertRaisesRegex(RuntimeError, "did not stop"):
            self.session.close_perception()
        self.session.node.destroy_node.assert_not_called()
        self.assertFalse(self.session.ros_closed)

    def test_perception_cleanup_retains_node_if_executor_shutdown_times_out(self):
        self.configure_perception_cleanup()
        self.session.executor.shutdown.return_value = False
        with self.assertRaisesRegex(RuntimeError, "did not stop"):
            self.session.close_perception()
        self.session.node.destroy_node.assert_not_called()

    def test_callback_samples_monotonic_after_entering_lock(self):
        self.acquire()
        clock = [0.30]
        controller = self.session.controller

        class InterleavingLock:
            def __enter__(self):
                # Timer ran while callback was waiting to enter its critical section.
                controller.tick(0.31)
                clock[0] = 0.32

            def __exit__(self, *_args):
                return False

        self.session.lock = InterleavingLock()
        with patch("follow_session.time.monotonic", side_effect=lambda: clock[0]):
            self.session.on_frame(self.frame())
        self.assertFalse(controller.halted)
        self.assertEqual(self.session.latest_received, 0.32)
        self.assertEqual(self.session.tracked_frames, 2)

    def test_failure_during_last_wait_cannot_be_reported_successful(self):
        self.acquire()
        clock = [0.3]

        def last_wait(_seconds):
            self.session.controller.stop("unexpected_camera_frame")
            clock[0] = 0.5  # Deadline expired while callback latched the failure.

        self.session.cancelled.wait.side_effect = last_wait
        blackboard = {}
        with patch("follow_session.time.monotonic", side_effect=lambda: clock[0]), \
                patch("follow_session.rclpy.ok", return_value=True):
            outcome = self.session.observe(blackboard)
        self.assertEqual(outcome, ABORT)
        self.assertEqual(blackboard["message"], "unexpected_camera_frame")
        self.assertTrue(self.session.finished)
        final = self.session.publish_preview.call_args.args[0]
        self.assertEqual((final.vx, final.wz), (0.0, 0.0))

    def test_wrong_provider_permanently_stops(self):
        frame = self.frame()
        frame.provider = "other/Provider"
        self.session.on_frame(frame)
        self.assertTrue(self.session.controller.halted)
        self.assertEqual(self.session.controller.tick(1.0).reason, "unexpected_people_provider")

    def run_timed_follow(self, duration, timeout, *, motion=True, cancel_after=None):
        """Advance a fake clock and feed fresh detections; never wait in real time."""
        self.acquire()
        self.session.args.duration = duration
        self.session.args.timeout = timeout
        if motion:
            self.session.motion = MagicMock()
            self.session.motion.send.return_value = ""
        clock = [0.3]
        person = Person(0.9, "person-1", 320.0, 240.0, 100.0, 120.0)

        def advance(seconds):
            clock[0] += seconds
            self.session.controller.update([person], clock[0])
            if cancel_after is not None and clock[0] >= 0.3 + cancel_after:
                self.session.cancelled.is_set.return_value = True
                self.sends_at_cancel = self.session.motion.send.call_count

        self.session.cancelled.wait.side_effect = advance
        blackboard = {}
        with patch("follow_session.time.monotonic", side_effect=lambda: clock[0]), \
                patch("follow_session.rclpy.ok", return_value=True):
            outcome = self.session.observe(blackboard)
        return outcome, blackboard, clock[0] - 0.3

    def test_two_minute_timeout_caps_five_minute_duration(self):
        outcome, board, elapsed = self.run_timed_follow(300.0, 120.0)
        self.assertEqual(outcome, SUCCEED)
        self.assertEqual(board["message"], "follow_trial_completed")
        self.assertGreaterEqual(elapsed, 120.0)
        self.assertLess(elapsed, 120.051)

    def test_duration_caps_longer_timeout(self):
        outcome, board, elapsed = self.run_timed_follow(120.0, 300.0)
        self.assertEqual(outcome, SUCCEED)
        self.assertEqual(board["message"], "follow_trial_completed")
        self.assertGreaterEqual(elapsed, 120.0)
        self.assertLess(elapsed, 120.051)

    def test_five_minutes_are_possible_only_with_matching_timeout(self):
        outcome, _, elapsed = self.run_timed_follow(300.0, 300.0)
        self.assertEqual(outcome, SUCCEED)
        self.assertGreaterEqual(elapsed, 300.0)
        self.assertLess(elapsed, 300.051)
        self.assertEqual(self.session.publish_preview.call_args.args[0].vx, 0.0)

    def test_preview_uses_timeout_without_creating_motion(self):
        outcome, board, elapsed = self.run_timed_follow(2.0, 120.0, motion=False)
        self.assertEqual(outcome, SUCCEED)
        self.assertEqual(board["message"], "preview_completed")
        self.assertGreaterEqual(elapsed, 120.0)
        self.assertLess(elapsed, 120.051)
        self.assertIsNone(self.session.motion)

    def test_cancel_interrupts_long_trial_before_deadline(self):
        outcome, board, elapsed = self.run_timed_follow(300.0, 300.0, cancel_after=0.2)
        self.assertEqual(outcome, ABORT)
        self.assertEqual(board["message"], "cancelled_or_ros_shutdown")
        self.assertLess(elapsed, 0.251)
        self.assertTrue(self.session.finished)
        self.assertGreater(self.sends_at_cancel, 0)
        self.assertEqual(self.session.motion.send.call_count, self.sends_at_cancel)
        final = self.session.publish_preview.call_args.args[0]
        self.assertEqual((final.vx, final.wz), (0.0, 0.0))

    def test_wrong_camera_frame_permanently_stops(self):
        frame = self.frame()
        frame.header.frame_id = "another_camera"
        self.session.on_frame(frame)
        self.assertEqual(self.session.controller.tick(1.0).reason, "unexpected_camera_frame")

    def test_old_source_stamp_permanently_stops(self):
        frame = self.frame()
        frame.header.stamp.sec = 98
        self.session.on_frame(frame)
        self.assertEqual(self.session.controller.tick(1.0).reason, "stale_or_future_image")

    def test_future_source_stamp_permanently_stops(self):
        frame = self.frame()
        frame.header.stamp.sec = 101
        self.session.on_frame(frame)
        self.assertEqual(self.session.controller.tick(1.0).reason, "stale_or_future_image")

    def test_zero_source_stamp_permanently_stops(self):
        frame = self.frame()
        frame.header.stamp.sec = 0
        self.session.on_frame(frame)
        self.assertEqual(self.session.controller.tick(1.0).reason, "invalid_image_timestamp")

    def test_repeated_source_stamp_permanently_stops(self):
        self.session.latest_stamp = 100_000_000_000
        self.session.on_frame(self.frame())
        self.assertEqual(self.session.controller.tick(1.0).reason, "non_increasing_image_timestamp")

    def test_finished_preview_ignores_late_frames(self):
        self.session.finish_preview("preview_finished")
        frame = self.frame()
        frame.provider = "late/UnexpectedProvider"
        self.session.on_frame(frame)
        self.assertTrue(self.session.finished)
        self.assertEqual(self.session.frames, 0)
        self.assertEqual(self.session.latest_stamp, 0)
        self.assertEqual(self.session.controller.tick(1.0).reason, "preview_finished")


if __name__ == "__main__":
    unittest.main()

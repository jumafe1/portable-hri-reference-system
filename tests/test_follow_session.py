"""Deterministic follow adapter regressions; requires sourced ROS interfaces.

No ROS initialization, live graph, network, driver or robot is used. The real
controller and generated message classes run against mocked session endpoints.
"""

from pathlib import Path
import json
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hri_reference_app/scripts"))

from hri_capability_interfaces.msg import PersonDetection2D, PersonDetection2DArray
from follow_controller import Config, Controller, Person
from follow_motion import FollowMotion
from follow_session import ABORT, PROVIDER, FollowSession


class FollowSessionTest(unittest.TestCase):
    def setUp(self):
        self.session = FollowSession.__new__(FollowSession)
        self.session.args = SimpleNamespace(expected_frame="camera_optical")
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
        self.session.last_seen_stamp = 0
        self.session.image_recovery_started = None
        self.session.image_recovery_stamp = 0
        self.session.image_recovery_frames = 0
        self.session.image_recovery_zero_sent = None
        self.session.image_recoveries = 0
        self.session.person_wait_reason = ""
        self.session.person_wait_stamp = 0
        self.session.person_wait_frames = 0
        self.session.person_wait_zero_sent = None
        self.session.person_recoveries = 0
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
        self.assertEqual(session.controller.config.max_forward_speed, 0.10)
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

    def test_callback_failure_ends_unbounded_observation(self):
        self.acquire()
        clock = [0.3]

        def last_wait(_seconds):
            self.session.controller.stop("unexpected_camera_frame")
            clock[0] = 0.5

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

    def run_until_cancel(self, cancel_after, *, motion=True, empty=False):
        """Advance a fake clock and feed fresh detections; never wait in real time."""
        if not empty:
            self.acquire()
        if motion:
            self.session.motion = MagicMock()
            self.session.motion.send.return_value = ""
        clock = [0.3]

        def advance(seconds):
            clock[0] += seconds
            frame = self.frame()
            stamp = 100_000_000_000 + round(clock[0] * 1e9)
            frame.header.stamp.sec, frame.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            self.session.node.get_clock.return_value.now.return_value.nanoseconds = stamp
            if empty:
                frame.people = []
            self.session.on_frame(frame)
            if clock[0] >= 0.3 + cancel_after:
                self.session.cancelled.is_set.return_value = True
                self.sends_at_cancel = self.session.motion.send.call_count if motion else 0

        self.session.cancelled.wait.side_effect = advance
        blackboard = {}
        with patch("follow_session.time.monotonic", side_effect=lambda: clock[0]), \
                patch("follow_session.rclpy.ok", return_value=True):
            outcome = self.session.observe(blackboard)
        return outcome, blackboard, clock[0] - 0.3

    def test_motion_continues_past_old_limits_until_cancelled(self):
        outcome, board, elapsed = self.run_until_cancel(601.0)
        self.assertEqual(outcome, ABORT)
        self.assertEqual(board["message"], "cancelled_or_ros_shutdown")
        self.assertGreaterEqual(elapsed, 601.0)
        self.assertLess(elapsed, 601.051)
        self.assertGreater(self.session.motion.send.call_count, 12000)
        self.assertEqual(self.session.publish_preview.call_args.args[0].vx, 0.0)

    def test_preview_continues_past_old_limits_without_motion(self):
        outcome, board, elapsed = self.run_until_cancel(301.0, motion=False)
        self.assertEqual(outcome, ABORT)
        self.assertEqual(board["message"], "cancelled_or_ros_shutdown")
        self.assertGreaterEqual(elapsed, 301.0)
        self.assertLess(elapsed, 301.051)
        self.assertIsNone(self.session.motion)

    def test_fresh_empty_stream_can_wait_for_first_person_indefinitely(self):
        outcome, board, elapsed = self.run_until_cancel(301.0, empty=True)
        self.assertEqual(outcome, ABORT)
        self.assertEqual(board["message"], "cancelled_or_ros_shutdown")
        self.assertGreaterEqual(elapsed, 301.0)
        for call in self.session.motion.send.call_args_list:
            self.assertEqual((call.args[0].vx, call.args[0].wz), (0.0, 0.0))
            self.assertFalse(call.args[0].locked)

    def test_initial_stream_waits_at_zero_until_cancel_if_publisher_exists(self):
        clock = [10.0]
        def advance(seconds):
            clock[0] += seconds
            if clock[0] >= 20.0:
                self.session.cancelled.is_set.return_value = True
        self.session.cancelled.wait.side_effect = advance
        board = {}
        with patch("follow_session.time.monotonic", side_effect=lambda: clock[0]), \
                patch("follow_session.rclpy.ok", return_value=True):
            outcome = self.session.observe(board)
        self.assertEqual(outcome, ABORT)
        self.assertEqual(board["message"], "cancelled_or_ros_shutdown")
        self.assertGreaterEqual(clock[0], 20.0)
        self.assertTrue(all(call.args[0].vx == call.args[0].wz == 0
                            for call in self.session.publish_preview.call_args_list))
        self.assertTrue(self.session.finished)

    def test_cancel_interrupts_observation_without_further_motion(self):
        outcome, board, elapsed = self.run_until_cancel(0.2)
        self.assertEqual(outcome, ABORT)
        self.assertEqual(board["message"], "cancelled_or_ros_shutdown")
        self.assertLess(elapsed, 0.251)
        self.assertTrue(self.session.finished)
        self.assertGreater(self.sends_at_cancel, 0)
        self.assertEqual(self.session.motion.send.call_count, self.sends_at_cancel)
        final = self.session.publish_preview.call_args.args[0]
        self.assertEqual((final.vx, final.wz), (0.0, 0.0))

    def test_stale_stream_holds_zero_until_operator_cancels(self):
        self.acquire()
        self.session.latest_stamp = 100_000_000_000
        clock = [0.3]

        def stall_stream(_seconds):
            clock[0] += 0.8
            self.session.node.get_clock.return_value.now.return_value.nanoseconds = 100_800_000_000
            if clock[0] > 10.0:
                self.session.cancelled.is_set.return_value = True

        self.session.cancelled.wait.side_effect = stall_stream
        board = {}
        with patch("follow_session.time.monotonic", side_effect=lambda: clock[0]), \
                patch("follow_session.rclpy.ok", return_value=True):
            self.assertEqual(self.session.observe(board), ABORT)
        self.assertEqual(board["message"], "cancelled_or_ros_shutdown")
        self.assertGreater(clock[0], 10.0)
        pauses = [call.args[0] for call in self.session.publish_preview.call_args_list
                  if call.args[0].reason == "waiting_fresh_images"]
        self.assertGreater(len(pauses), 1)
        self.assertTrue(all(d.vx == d.wz == 0.0 for d in pauses))

    def test_already_cancelled_observation_never_calls_camera_or_motion(self):
        self.session.cancelled.is_set.return_value = True
        self.session.camera = MagicMock()
        self.session.motion = MagicMock()
        board = {}
        with patch("follow_session.rclpy.ok", return_value=True):
            self.assertEqual(self.session.observe(board), ABORT)
        self.assertEqual(board["message"], "cancelled_or_ros_shutdown")
        self.session.camera.gate.assert_not_called()
        self.session.motion.send.assert_not_called()

    def test_wrong_camera_frame_permanently_stops(self):
        frame = self.frame()
        frame.header.frame_id = "another_camera"
        self.session.on_frame(frame)
        self.assertEqual(self.session.controller.tick(1.0).reason, "unexpected_camera_frame")

    def test_old_source_stamp_pauses_without_accepting_the_frame(self):
        frame = self.frame()
        frame.header.stamp.sec = 98
        self.session.on_frame(frame)
        self.assertEqual(self.session.controller.tick(1.0).reason, "waiting_fresh_images")
        self.assertFalse(self.session.controller.halted)
        self.assertEqual(self.session.latest_stamp, 0)

    def test_future_source_stamp_permanently_stops(self):
        frame = self.frame()
        frame.header.stamp.sec = 101
        self.session.on_frame(frame)
        self.assertEqual(self.session.controller.tick(1.0).reason, "stale_or_future_image")

    def test_rejected_frame_diagnostic_preserves_actual_stamp_and_first_failure(self):
        self.session.latest_stamp = 99_900_000_000
        frame = self.frame()
        frame.header.stamp.sec = 98
        frame.provider = "wrong/provider"
        self.session.on_frame(frame)
        diagnostic = self.session.perception_diagnostic.copy()
        self.assertEqual(diagnostic['stamp_ns'], 98_000_000_000)
        self.assertEqual(diagnostic['age_at_receipt_s'], 2.0)
        self.assertEqual(self.session.latest_stamp, 99_900_000_000)
        self.session.on_frame(self.frame())
        self.assertEqual(self.session.perception_diagnostic, diagnostic)

    def test_ambiguity_diagnostic_counts_only_eligible_people(self):
        self.acquire()
        frame = self.frame()
        frame.people = [frame.people[0], frame.people[0], PersonDetection2D(
            confidence=0.6, center_x=320.0, center_y=240.0, width=100.0, height=120.0)]
        with patch('follow_session.time.monotonic', return_value=0.4):
            self.session.on_frame(frame)
        self.assertEqual(self.session.controller.tick(0.4).reason, 'ambiguous_people')
        self.assertEqual(self.session.perception_diagnostic['people_count'], 3)
        self.assertEqual(self.session.perception_diagnostic['eligible_count'], 2)

    def test_nonfinite_geometry_diagnostic_can_be_published_as_strict_json(self):
        self.acquire()
        frame = self.frame()
        frame.people[0].width = float('nan')
        with patch('follow_session.time.monotonic', return_value=0.4):
            self.session.on_frame(frame)
        self.session.args.robot = 'nao'
        self.session.camera = None
        self.session.publisher = MagicMock()
        self.session.last_reason = ''
        self.session.last_print = 0.0
        decision = self.session.controller.tick(0.4)
        self.assertEqual(decision.reason, 'invalid_detection')
        FollowSession.publish_preview(self.session, decision)
        payload = json.loads(self.session.publisher.publish.call_args.args[0].data)
        self.assertEqual(payload['perception']['invalid_count'], 1)
        self.assertEqual(payload['perception']['people'][0]['width'], 'nan')
        self.assertEqual(payload['source_age_s'], 0.0)

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

    def set_time(self, now):
        self.session.node.get_clock.return_value.now.return_value.nanoseconds = round((100 + now) * 1e9)

    def feed_at(self, now, *, age=0.12, people=None):
        self.set_time(now)
        frame = self.frame()
        stamp = round((100 + now - age) * 1e9)
        frame.header.stamp.sec, frame.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        if people is not None:
            frame.people = people
        with patch("follow_session.time.monotonic", return_value=now):
            self.session.on_frame(frame)

    def check_at(self, now):
        self.set_time(now)
        with patch("follow_session.time.monotonic", return_value=now):
            return self.session._checked_decision()

    def start_recovery(self):
        for now in (0.2, 0.3, 0.4):
            self.feed_at(now)
        self.assertGreater(self.check_at(0.4).vx, 0)
        decision = self.check_at(1.1)
        self.assertEqual((decision.vx, decision.wz), (0, 0))
        self.assertEqual(decision.reason, "waiting_fresh_images")
        self.assertFalse(self.session.controller.halted)

    def test_three_distinct_post_pause_images_recover_preview(self):
        self.start_recovery()
        for now in (1.3, 1.4):
            self.feed_at(now)
            for _ in range(5):  # Repeated ticks cannot count as new observations.
                decision = self.check_at(now)
                self.assertEqual((decision.vx, decision.wz), (0, 0))
        self.feed_at(1.5)
        self.assertGreater(self.check_at(1.5).vx, 0)
        self.assertIsNone(self.session.image_recovery_started)
        self.assertEqual(self.session.image_recoveries, 1)

    def test_callback_cannot_hide_gap_between_control_ticks(self):
        for now in (0.2, 0.3, 0.4):
            self.feed_at(now)
        self.feed_at(1.2)
        self.assertEqual(self.check_at(1.2).reason, "waiting_fresh_images")
        self.assertEqual(self.session.image_recovery_frames, 0)
        self.assertEqual(self.session.image_recovery_started, 1.2)

    def test_queued_images_from_before_pause_do_not_rearm(self):
        self.start_recovery()
        for now in (1.2, 1.3, 1.4):
            self.feed_at(now, age=0.5)
            self.assertEqual(self.check_at(now).vx, 0)
        self.assertEqual(self.session.image_recovery_frames, 0)

    def test_second_gap_resets_acquisition_and_can_recover_later(self):
        self.start_recovery()
        self.feed_at(1.3)
        self.feed_at(1.4)
        self.assertEqual(self.session.image_recovery_frames, 2)
        self.assertEqual(self.check_at(2.2).reason, "waiting_fresh_images")
        self.assertEqual(self.session.image_recovery_frames, 0)
        self.assertEqual(self.session.image_recovery_started, 1.1)
        self.feed_at(2.4)
        self.assertEqual(self.check_at(2.4).vx, 0)
        self.assertEqual(self.check_at(4.11).reason, "waiting_fresh_images")
        self.assertFalse(self.session.controller.halted)
        for now in (4.2, 4.3, 4.4):
            self.feed_at(now)
        self.assertGreater(self.check_at(4.4).vx, 0)

    def test_return_after_long_image_gap_still_requires_three_fresh_frames(self):
        self.start_recovery()
        for now in (30.0, 30.1):
            self.feed_at(now)
            self.assertEqual(self.check_at(now).vx, 0)
        self.feed_at(30.2)
        self.assertGreater(self.check_at(30.2).vx, 0)

    def test_invalid_geometry_during_recovery_still_latches(self):
        bad = self.frame().people[0]
        bad.width = float("nan")
        for people, reason in (([bad], "invalid_detection"),):
            with self.subTest(reason=reason):
                self.setUp()
                self.start_recovery()
                self.feed_at(1.3, people=people)
                self.assertEqual(self.check_at(1.3).reason, reason)
                self.feed_at(1.4)
                self.assertTrue(self.session.controller.halted)

    def test_duplicate_future_and_wrong_headers_during_recovery_still_latch(self):
        for fault, reason in (("duplicate", "non_increasing_image_timestamp"),
                              ("future", "stale_or_future_image"),
                              ("provider", "unexpected_people_provider"),
                              ("frame", "unexpected_camera_frame")):
            with self.subTest(fault=fault):
                self.setUp()
                self.start_recovery()
                self.feed_at(1.3)
                frame = self.frame()
                stamp = self.session.latest_stamp if fault == "duplicate" else 102_000_000_000
                frame.header.stamp.sec, frame.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                if fault == "provider":
                    frame.provider = "other/provider"
                if fault == "frame":
                    frame.header.frame_id = "other_camera"
                with patch("follow_session.time.monotonic", return_value=1.4):
                    self.session.on_frame(frame)
                self.assertEqual(self.check_at(1.4).reason, reason)

    def test_motion_requires_zero_publication_and_new_stop_ack_before_resume(self):
        self.start_recovery()
        self.session.motion = SimpleNamespace(status=("stopped", 1.0))
        for now in (1.3, 1.4, 1.5):
            self.feed_at(now)
        self.assertEqual(self.check_at(1.5).reason, "waiting_image_recovery_stop")
        self.session.image_recovery_zero_sent = 1.1
        self.assertEqual(self.check_at(1.6).vx, 0)  # Old acknowledgement is insufficient.
        self.session.motion.status = ("moving", 1.6)
        self.assertEqual(self.check_at(1.6).vx, 0)
        self.session.motion.status = ("stopped", 1.6)
        self.assertGreater(self.check_at(1.6).vx, 0)

    def test_no_resume_when_stop_ack_never_arrives(self):
        self.start_recovery()
        self.session.motion = SimpleNamespace(status=("moving", 1.1))
        self.session.image_recovery_zero_sent = 1.1
        for now in (1.3, 1.4, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0):
            self.feed_at(now)
            self.assertEqual(self.check_at(now).vx, 0)
        self.assertEqual(self.check_at(4.11).reason, "waiting_image_recovery_stop")

    def test_receipt_watchdog_pauses_even_when_ros_clock_does_not_advance(self):
        for now in (0.2, 0.3, 0.4):
            self.feed_at(now)
        with patch("follow_session.time.monotonic", return_value=1.2):
            decision = self.session._checked_decision()
        self.assertEqual(decision.reason, "waiting_fresh_images")

    def test_recovery_keeps_camera_health_checks_without_commands(self):
        self.start_recovery()
        self.session.camera = MagicMock(active=True)
        self.session.camera.health_error.return_value = ""
        self.assertEqual(self.check_at(1.2).reason, "waiting_fresh_images")
        self.session.camera.gate.assert_not_called()
        self.session.camera.health_error.return_value = "camera_feedback_or_calibration_stale"
        self.assertEqual(self.check_at(1.3).reason, "camera_feedback_or_calibration_stale")

    def test_pause_cannot_clear_a_permanent_fault(self):
        self.session.controller.stop("ambiguous_people")
        self.session.controller.pause_stale()
        self.assertEqual(self.session.controller.tick(1.0).reason, "ambiguous_people")

    def test_observe_sends_only_zero_through_real_motion_adapter_during_gap(self):
        for now in (0.2, 0.3, 0.4):
            self.feed_at(now)
        motion = self.session.motion = FollowMotion(self.session)
        motion.acquired = True
        motion.publisher.get_subscription_count.return_value = 1
        clock, commands = [0.4], []
        motion.status = ("moving", clock[0])
        motion.publisher.publish.side_effect = lambda msg: commands.append(
            (clock[0], msg.linear.x, msg.angular.z))

        def advance(seconds):
            clock[0] = round(clock[0] + seconds, 3)
            self.set_time(clock[0])
            # Fake backend acknowledges the last published Twist on the next tick.
            status = "stopped" if commands[-1][1:] == (0.0, 0.0) else "moving"
            motion.status = (status, clock[0])
            if clock[0] >= 1.6:
                self.feed_at(clock[0])
            if clock[0] >= 2.0:
                self.session.cancelled.is_set.return_value = True

        self.session.cancelled.wait.side_effect = advance
        board = {}
        with patch("follow_session.time.monotonic", side_effect=lambda: clock[0]), \
                patch("follow_session.rclpy.ok", return_value=True):
            self.assertEqual(self.session.observe(board), ABORT)
        self.assertEqual(board["message"], "cancelled_or_ros_shutdown")
        self.assertTrue(any(vx > 0 for t, vx, _ in commands if t < 1.05))
        held = [(vx, wz) for t, vx, wz in commands if 1.05 <= t < 1.7]
        self.assertGreater(len(held), 10)
        self.assertTrue(all(command == (0.0, 0.0) for command in held))
        self.assertTrue(any(vx > 0 for t, vx, _ in commands if t >= 1.7))
        self.assertEqual(self.session.image_recoveries, 1)
        self.assertFalse(any(t >= 2.0 for t, _, _ in commands))

    def test_lost_bond_or_publisher_during_pause_still_latches(self):
        for fault, reason in (("bond", "capability_bond_broken"),
                              ("publisher", "people_publisher_lost_or_ambiguous")):
            with self.subTest(fault=fault):
                self.setUp()
                self.start_recovery()
                if fault == "bond":
                    self.session.bond.is_broken.return_value = True
                else:
                    self.session.node.count_publishers.return_value = 0
                self.assertEqual(self.check_at(1.2).reason, reason)

    def test_cancel_during_pause_does_not_resume_or_send_new_commands(self):
        self.start_recovery()
        self.session.motion = MagicMock()
        self.session.cancelled.is_set.return_value = True
        board = {}
        with patch("follow_session.rclpy.ok", return_value=True):
            self.assertEqual(self.session.observe(board), ABORT)
        self.assertEqual(board["message"], "cancelled_or_ros_shutdown")
        self.session.motion.send.assert_not_called()
        self.assertEqual(self.session.image_recoveries, 0)

    def start_person_wait(self, count=0):
        for now in (0.2, 0.3, 0.4):
            self.feed_at(now)
        self.assertGreater(self.check_at(0.4).vx, 0)
        self.feed_at(0.6, people=[self.frame().people[0]] * count)
        decision = self.check_at(0.6)
        self.assertEqual(decision.reason, "person_missing" if count == 0 else "ambiguous_people")
        self.assertEqual((decision.vx, decision.wz), (0, 0))
        self.assertFalse(self.session.controller.halted)

    def test_missing_or_ambiguous_can_wait_a_minute_then_reacquire(self):
        for count in (0, 2):
            with self.subTest(count=count):
                self.setUp()
                self.start_person_wait(count)
                for i in range(1, 301):
                    now = round(0.6 + i * 0.2, 3)
                    self.feed_at(now, people=[self.frame().people[0]] * count)
                    decision = self.check_at(now)
                    self.assertEqual((decision.vx, decision.wz), (0, 0))
                    self.assertFalse(self.session.controller.halted)
                for now in (60.8, 61.0):
                    self.feed_at(now)
                    for _ in range(3):
                        self.assertEqual(self.check_at(now).reason, "reacquiring_person")
                self.feed_at(61.2)
                self.assertGreater(self.check_at(61.2).vx, 0)
                self.assertEqual(self.session.person_recoveries, 1)
                self.assertFalse(self.session.person_wait_reason)

    def test_unique_observations_must_be_consecutive_after_people_wait(self):
        self.start_person_wait()
        for now in (0.8, 0.9):
            self.feed_at(now)
        self.feed_at(1.0, people=[self.frame().people[0]] * 2)
        self.assertEqual(self.session.person_wait_frames, 0)
        for now in (1.1, 1.2):
            self.feed_at(now)
            self.assertEqual(self.check_at(now).vx, 0)
        self.feed_at(1.3)
        self.assertGreater(self.check_at(1.3).vx, 0)

    def test_pre_pause_queued_person_images_do_not_rearm(self):
        self.start_person_wait()
        for now in (0.7, 0.8, 0.9):
            self.feed_at(now, age=0.4)
            self.assertEqual(self.check_at(now).vx, 0)
        self.assertEqual(self.session.person_wait_frames, 0)

    def test_person_return_still_requires_new_stop_ack_and_hold(self):
        self.start_person_wait()
        self.session.motion = SimpleNamespace(status=("stopped", 0.5))
        for now in (0.8, 0.85, 0.9):
            self.feed_at(now)
        self.assertEqual(self.check_at(0.9).reason, "waiting_person_stop")
        self.session.person_wait_zero_sent = 0.6
        self.session.motion.status = ("stopped", 0.85)
        self.assertEqual(self.check_at(0.9).vx, 0)  # Less than 0.4 s of zero commands.
        self.session.motion.status = ("stopped", 0.5)
        self.assertEqual(self.check_at(1.1).vx, 0)  # Pre-pause acknowledgement.
        self.session.motion.status = ("moving", 1.1)
        self.assertEqual(self.check_at(1.1).vx, 0)
        self.session.motion.status = ("stopped", 1.1)
        self.assertGreater(self.check_at(1.1).vx, 0)

    def test_fresh_empty_or_ambiguous_images_recover_stream_but_keep_base_held(self):
        for count in (0, 2):
            with self.subTest(count=count):
                self.setUp()
                self.start_recovery()
                for now in (1.3, 1.4, 1.5):
                    self.feed_at(now, people=[self.frame().people[0]] * count)
                    self.assertEqual(self.check_at(now).vx, 0)
                self.assertIsNone(self.session.image_recovery_started)
                self.assertTrue(self.session.person_wait_reason)
                for i in range(1, 21):
                    now = 1.5 + i * 0.2
                    self.feed_at(now, people=[self.frame().people[0]] * count)
                    self.assertEqual(self.check_at(now).vx, 0)
                    self.assertFalse(self.session.controller.halted)
                for now in (5.7, 5.8):
                    self.feed_at(now)
                    self.assertEqual(self.check_at(now).vx, 0)
                self.feed_at(5.9)
                self.assertGreater(self.check_at(5.9).vx, 0)

    def test_image_gap_while_waiting_for_person_resets_unique_frame_count(self):
        self.start_person_wait()
        for now in (0.8, 0.9):
            self.feed_at(now)
        self.assertEqual(self.session.person_wait_frames, 2)
        self.assertEqual(self.check_at(1.6).reason, "waiting_fresh_images")
        self.assertEqual(self.session.person_wait_frames, 0)
        for now in (1.8, 1.9):
            self.feed_at(now)
            self.assertEqual(self.check_at(now).vx, 0)
        self.feed_at(2.0)
        self.assertGreater(self.check_at(2.0).vx, 0)
        self.assertEqual(self.session.image_recoveries, 1)
        self.assertEqual(self.session.person_recoveries, 1)

    def test_waiting_for_person_holds_on_stream_gap_but_invalid_geometry_is_fatal(self):
        for fault in ("stream", "geometry"):
            with self.subTest(fault=fault):
                self.setUp()
                self.start_person_wait()
                if fault == "stream":
                    self.check_at(1.4)
                    self.assertEqual(self.check_at(4.41).reason, "waiting_fresh_images")
                else:
                    bad = self.frame().people[0]
                    bad.width = -1.0
                    self.feed_at(0.8, people=[bad])
                    self.assertEqual(self.check_at(0.8).reason, "invalid_detection")
                self.assertEqual(self.session.controller.halted, fault == "geometry")

    def test_low_confidence_waits_and_two_eligible_people_prevent_resume(self):
        self.start_person_wait()
        low = self.frame().people[0]
        low.confidence = 0.5
        self.feed_at(0.8, people=[low])
        self.assertEqual(self.check_at(0.8).reason, "person_missing")
        for now in (0.9, 1.0, 1.1):
            self.feed_at(now, people=[self.frame().people[0], low])
        self.assertGreater(self.check_at(1.1).vx, 0)
        self.feed_at(1.2, people=[self.frame().people[0]] * 2)
        self.assertEqual(self.check_at(1.2).reason, "ambiguous_people")

    def test_person_wait_blocks_camera_commands_and_keeps_health_check(self):
        self.start_person_wait()
        self.session.camera = MagicMock(active=True)
        self.session.camera.health_error.return_value = ""
        self.assertEqual(self.check_at(0.7).vx, 0)
        self.session.camera.gate.assert_not_called()
        self.session.camera.health_error.return_value = "camera_feedback_or_calibration_stale"
        self.assertEqual(self.check_at(0.75).reason, "camera_feedback_or_calibration_stale")

    def test_motion_adapter_holds_during_people_wait_resumes_and_cancels(self):
        for count in (0, 2):
            with self.subTest(count=count):
                self.setUp()
                for now in (0.2, 0.3, 0.4):
                    self.feed_at(now)
                motion = self.session.motion = FollowMotion(self.session)
                motion.acquired = True
                motion.publisher.get_subscription_count.return_value = 1
                clock, commands = [0.4], []
                motion.status = ("moving", clock[0])
                motion.publisher.publish.side_effect = lambda msg: commands.append(
                    (clock[0], msg.linear.x, msg.angular.z))

                def advance(seconds):
                    clock[0] = round(clock[0] + seconds, 3)
                    status = "stopped" if commands[-1][1:] == (0.0, 0.0) else "moving"
                    motion.status = (status, clock[0])
                    people = [self.frame().people[0]] * count if 0.6 <= clock[0] < 5.0 else None
                    self.feed_at(clock[0], people=people)
                    if clock[0] >= 5.4:
                        self.session.cancelled.is_set.return_value = True

                self.session.cancelled.wait.side_effect = advance
                board = {}
                with patch("follow_session.time.monotonic", side_effect=lambda: clock[0]), \
                        patch("follow_session.rclpy.ok", return_value=True):
                    self.assertEqual(self.session.observe(board), ABORT)
                self.assertEqual(board["message"], "cancelled_or_ros_shutdown")
                held = [(vx, wz) for t, vx, wz in commands if 0.6 <= t < 5.1]
                self.assertGreater(len(held), 80)
                self.assertTrue(all(command == (0.0, 0.0) for command in held))
                self.assertTrue(any(vx > 0 for t, vx, _ in commands if t >= 5.1))
                self.assertEqual(self.session.person_recoveries, 1)
                self.assertFalse(any(t >= 5.4 for t, _, _ in commands))


if __name__ == "__main__":
    unittest.main()

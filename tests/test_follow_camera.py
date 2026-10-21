"""Optional-camera state/lifecycle checks with mocked ROS, never hardware."""

from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hri_reference_app/scripts"))

from hri_capability_interfaces.msg import CameraOrientation
from sensor_msgs.msg import CameraInfo
from camera_geometry import Calibration
from follow_camera import ABORT, CAPABILITY, SUCCEED, FollowCamera
from follow_controller import Config, Controller, Decision, Person
from follow_session import FollowSession


class FollowCameraTest(unittest.TestCase):
    def setUp(self):
        self.session = SimpleNamespace(
            node=MagicMock(), lock=threading.Lock(),
            args=SimpleNamespace(camera_provider="test/Camera", expected_frame="camera_optical",
                                 image_width=320, image_height=240),
            cancelled=threading.Event(), controller=Controller(Config(320, 240)),
            motion=SimpleNamespace(status=("stopped", 0.0)),
            running_client=MagicMock(), release_client=MagicMock(), await_future=MagicMock(return_value=True))
        self.session.node.create_client.side_effect = lambda *args: MagicMock()
        self.session.node.count_publishers.return_value = 1
        self.session.node.get_clock.return_value.now.return_value.nanoseconds = 100_000_000_000
        self.camera = FollowCamera(self.session)
        self.camera.command.service_is_ready.return_value = True
        self.board = {"bond_id": "test-bond", "message": "test"}
        self.decision = Decision(0.02, 0.01, "preview_following", True)

    def feedback(self, sec=100, elevation=0.0, ready=True, moving=False):
        message = CameraOrientation()
        message.header.frame_id = "camera_mount"
        message.header.stamp.sec = sec
        message.provider = "test/Camera"
        message.elevation, message.azimuth = elevation, 0.0
        message.ready, message.moving = ready, moving
        message.status = "ready" if ready else "awaiting_camera_backend"
        return message

    def activate(self):
        self.camera.on_status(self.feedback())
        self.camera.calibration = Calibration(320, 240, 280.0, 280.0, 160.0, 120.0)
        self.camera.calibration_stamp = 100_000_000_000
        self.camera.active = self.camera.requested = True
        self.camera.target = 0.0
        self.camera.image_stamp = 100_000_000_000
        self.camera.settled_stamp = 99_000_000_000

    def test_initial_unmeasured_status_is_not_latched_as_failure(self):
        self.camera.on_status(self.feedback(sec=0, ready=False))
        self.assertEqual(self.camera.error, "")
        self.assertIsNone(self.camera.feedback)
        self.camera.on_status(self.feedback())
        self.assertEqual(self.camera.error, "")
        self.assertTrue(self.camera.feedback.ready)

    def test_zero_stamp_after_activation_or_ready_zero_stamp_is_rejected(self):
        for active, ready in ((True, False), (False, True)):
            self.camera.error, self.camera.active = "", active
            self.camera.on_status(self.feedback(sec=0, ready=ready))
            self.assertEqual(self.camera.error, "stale_or_invalid_camera_timestamp")

    def test_wrong_identity_even_on_startup_is_rejected(self):
        message = self.feedback(sec=0, ready=False)
        message.provider = "other/Camera"
        self.camera.on_status(message)
        self.assertEqual(self.camera.error, "unexpected_camera_provider_or_mount_frame")

    def test_catalog_missing_interface_falls_back_without_acquisition(self):
        with patch.object(self.camera, "_call", return_value=SimpleNamespace(interfaces=[])) as call:
            self.assertEqual(self.camera.configure(self.board), SUCCEED)
        self.assertEqual(call.call_count, 1)
        self.assertFalse(self.camera.requested)
        self.assertEqual(self.camera.status, "fixed_camera_fallback_no_interface")

    def test_catalog_empty_providers_falls_back_without_acquisition(self):
        with patch.object(self.camera, "_call", side_effect=[SimpleNamespace(interfaces=[CAPABILITY]),
                                                           SimpleNamespace(providers=[])]):
            self.assertEqual(self.camera.configure(self.board), SUCCEED)
        self.assertFalse(self.camera.requested)
        self.assertEqual(self.camera.status, "fixed_camera_fallback_no_provider")

    def test_catalog_error_is_not_misrepresented_as_absence(self):
        with patch.object(self.camera, "_call", side_effect=RuntimeError("catalog failed")):
            self.assertEqual(self.camera.configure(self.board), ABORT)
        self.assertEqual(self.board["message"], "catalog failed")

    def test_missing_requested_provider_is_configuration_error(self):
        with patch.object(self.camera, "_call", side_effect=[SimpleNamespace(interfaces=[CAPABILITY]),
                                                           SimpleNamespace(providers=["other/Camera"])]):
            self.assertEqual(self.camera.configure(self.board), ABORT)
        self.assertFalse(self.camera.requested)

    def test_acquisition_failure_marks_request_for_cleanup(self):
        with patch.object(self.camera, "_call", side_effect=[SimpleNamespace(interfaces=[CAPABILITY]),
                          SimpleNamespace(providers=["test/Camera"]), RuntimeError("acquire failed")]):
            self.assertEqual(self.camera.configure(self.board), ABORT)
        self.assertTrue(self.camera.requested)
        self.assertFalse(self.camera.active)

    def test_preview_never_constructs_camera_transport_even_when_requested(self):
        args = SimpleNamespace(image_width=320, image_height=240, minimum_confidence=0.65,
                               target_height_ratio=0.8, orient_camera=True, enable_motion=False)
        with patch("follow_session.Node"), patch("follow_session.SingleThreadedExecutor"), \
                patch("follow_session.threading.Thread"), patch("follow_session.FollowCamera") as camera:
            session = FollowSession(args, threading.Event())
        camera.assert_not_called()
        self.assertIsNone(session.camera)
        self.assertIsNone(session.motion)

    def test_active_health_failure_latches_stop(self):
        self.activate()
        self.camera.feedback_stamp = 98_000_000_000
        decision = self.camera.gate(self.decision)
        self.assertEqual(decision.vx, 0.0)
        self.assertTrue(self.session.controller.halted)
        self.assertEqual(decision.reason, "camera_feedback_or_calibration_stale")

    def test_health_checks_resolved_remapped_subscription_names(self):
        self.activate()
        self.camera.status_sub.topic_name = "/robot/camera/orientation"
        self.camera.info_sub.topic_name = "/robot/camera/front/camera_info"
        self.session.node.count_publishers.side_effect = lambda name: int(name in {
            "/robot/camera/orientation", "/robot/camera/front/camera_info"})
        self.assertEqual(self.camera.health_error(), "")

    def test_yaw_motion_is_rejected(self):
        self.activate()
        feedback = self.feedback()
        feedback.azimuth = 0.1
        self.camera.on_status(feedback)
        self.assertEqual(self.camera.gate(self.decision).reason, "camera_orientation_out_of_bounds")

    def test_moving_camera_outside_command_is_rejected(self):
        self.activate()
        self.camera.feedback.moving = True
        self.assertEqual(self.camera.gate(self.decision).reason, "unexpected_camera_motion")

    def test_clipped_frame_retains_zero_decision_but_can_request_framing(self):
        self.activate()
        person = Person(0.9, "", 160.0, 95.0, 80.0, 190.0)
        self.assertIsNone(self.camera.process_frame([person], 100_000_000_000))
        self.assertEqual(self.camera.error, "")
        with patch("follow_camera.time.monotonic", return_value=10.0):
            decision = self.camera.gate(Decision(0.0, 0.0, "clipped_detection", True))
        self.assertEqual((decision.vx, decision.wz), (0.0, 0.0))
        self.assertEqual(self.camera.pending, 0.03)
        self.camera.command.call_async.assert_not_called()

    def test_adjustment_requires_base_stop_feedback_settling_and_fresh_image(self):
        self.activate()
        self.camera.person = Person(0.9, "", 160.0, 70.0, 50.0, 80.0)
        with patch("follow_camera.time.monotonic", return_value=10.0):
            self.assertEqual(self.camera.gate(self.decision).vx, 0.0)
        self.assertEqual(self.camera.pending, 0.03)
        self.session.motion.status = ("stopped", 10.1)
        with patch("follow_camera.time.monotonic", return_value=10.3):
            self.camera.gate(self.decision)
        self.camera.command.call_async.assert_not_called()
        future = MagicMock()
        future.done.return_value = True
        future.exception.return_value = None
        future.result.return_value.accepted = True
        self.camera.command.call_async.return_value = future
        with patch("follow_camera.time.monotonic", return_value=10.5):
            self.assertEqual(self.camera.gate(self.decision).vx, 0.0)
        self.assertEqual(self.camera.commands_sent, 1)
        self.camera.feedback.elevation = 0.03
        self.camera.feedback_stamp = 100_100_000_000
        self.session.node.get_clock.return_value.now.return_value.nanoseconds = 100_100_000_000
        with patch("follow_camera.time.monotonic", return_value=10.6):
            self.camera.gate(self.decision)
        self.camera.feedback_stamp = 100_200_000_000
        self.session.node.get_clock.return_value.now.return_value.nanoseconds = 100_200_000_000
        with patch("follow_camera.time.monotonic", return_value=10.9):
            self.camera.gate(self.decision)
        self.assertIsNone(self.camera.pending)
        self.camera.person = Person(0.9, "", 160.0, 120.0, 50.0, 80.0)
        with patch("follow_camera.time.monotonic", return_value=11.0):
            self.assertEqual(self.camera.gate(self.decision).reason, "camera_waiting_fresh_image")
        self.camera.image_stamp = 100_210_000_000
        with patch("follow_camera.time.monotonic", return_value=11.1):
            self.assertEqual(self.camera.gate(self.decision), self.decision)

    def test_base_stop_not_acknowledged_aborts_without_head_command(self):
        self.activate()
        self.camera.pending, self.camera.hold_started = 0.03, 10.0
        with patch("follow_camera.time.monotonic", return_value=12.1):
            self.assertEqual(self.camera.gate(self.decision).reason, "base_stop_before_camera_not_acknowledged")
        self.camera.command.call_async.assert_not_called()

    def test_rejected_camera_command_aborts(self):
        self.activate()
        self.camera.future = MagicMock()
        self.camera.future.done.return_value = True
        self.camera.future.exception.return_value = None
        self.camera.future.result.return_value.accepted = False
        self.assertEqual(self.camera.gate(self.decision).reason, "camera_command_rejected")

    def test_final_deadline_and_cancellation_never_send_new_head_commands(self):
        self.activate()
        self.camera.pending, self.camera.hold_started = 0.03, 10.0
        self.session.motion.status = ("stopped", 10.1)
        with patch("follow_camera.time.monotonic", return_value=10.5):
            self.assertEqual(self.camera.gate(self.decision, allow_commands=False).vx, 0.0)
            self.session.cancelled.set()
            self.assertEqual(self.camera.gate(self.decision).vx, 0.0)
        self.camera.command.call_async.assert_not_called()

    def test_hold_and_release_are_independent_and_idempotent(self):
        self.activate()
        with patch.object(self.camera, "_call", side_effect=RuntimeError("hold failed")):
            self.assertEqual(self.camera.stop(self.board), ABORT)
        with patch.object(self.camera, "_call", return_value=SimpleNamespace()) as call:
            self.assertEqual(self.camera.release(self.board), SUCCEED)
            self.assertEqual(self.camera.release(self.board), SUCCEED)
        self.assertEqual(call.call_count, 1)
        self.assertFalse(self.camera.active)
        self.assertTrue(self.camera.released)

    def test_changed_active_calibration_latches_failure(self):
        self.activate()
        message = CameraInfo()
        message.header.frame_id, message.header.stamp.sec = "camera_optical", 100
        message.width, message.height = 320, 240
        message.k = [290.0, 0.0, 160.0, 0.0, 290.0, 120.0, 0.0, 0.0, 1.0]
        self.camera.on_info(message)
        self.assertEqual(self.camera.error, "camera_calibration_changed")


if __name__ == "__main__":
    unittest.main()

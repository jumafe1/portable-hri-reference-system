"""Follow-mode teardown tests with mocked ROS; no graph or hardware is started."""

from contextlib import ExitStack, redirect_stdout
import io
import json
from pathlib import Path
import signal
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hri_reference_app/scripts"))

import follow_mode


class FollowModeCleanupTest(unittest.TestCase):
    def run_preview(self, cleanup_error=None, *, diagnostic_error=None,
                    bond_error=None, motion_stop_error=None, motion_release_error=None,
                    motion_enabled=False, camera_enabled=False, camera_stop_error=None,
                    camera_release_error=None):
        session = MagicMock()
        session.frames = 10
        session.tracked_frames = 8
        session.samples = 20
        session.close_perception.side_effect = cleanup_error
        session.finish_preview.side_effect = diagnostic_error
        session.bond.shutdown.side_effect = bond_error
        if motion_enabled:
            session.motion.requested = True
            session.motion.released = False
            session.motion.stop_acknowledged = False
            session.motion.commands_sent = 2
            session.motion.stop.return_value = follow_mode.SUCCEED
            session.motion.stop.side_effect = motion_stop_error

            def release_motion(_blackboard):
                if motion_release_error:
                    raise motion_release_error
                session.motion.released = True
                return follow_mode.SUCCEED

            session.motion.release.side_effect = release_motion
        else:
            session.motion = None
        session.camera = None
        if camera_enabled:
            session.camera = MagicMock()
            session.camera.requested, session.camera.released = True, False
            session.camera.status = "camera_ready"
            session.camera.commands_sent = 2
            session.camera.hold_acknowledged = False
            session.camera.stop.return_value = follow_mode.SUCCEED
            session.camera.stop.side_effect = camera_stop_error

            def release_camera(_blackboard):
                if camera_release_error:
                    raise camera_release_error
                session.camera.released = True
                return follow_mode.SUCCEED

            session.camera.release.side_effect = release_camera
        events = MagicMock()
        events.attach_mock(session.close_perception, "close_perception")
        events.attach_mock(session.bond.shutdown, "close_bond")

        def finish(blackboard):
            blackboard["released"] = True
            blackboard["message"] = "preview_completed"
            return follow_mode.APP_SUCCEEDED

        output = io.StringIO()
        with ExitStack() as stack:
            init = stack.enter_context(patch("follow_mode.rclpy.init"))
            stack.enter_context(patch("follow_mode.rclpy.ok", return_value=True))
            shutdown = stack.enter_context(patch("follow_mode.rclpy.shutdown"))
            loggers = stack.enter_context(patch("follow_mode.set_ros_loggers"))
            handlers = stack.enter_context(patch("follow_mode.signal.signal", return_value=signal.SIG_DFL))
            stack.enter_context(patch("follow_mode.Blackboard", side_effect=dict))
            stack.enter_context(patch("follow_mode.FollowSession", return_value=session))
            stack.enter_context(patch("follow_mode.build_state_machine", return_value=finish))
            events.attach_mock(shutdown, "shutdown_ros")
            with redirect_stdout(output):
                code = follow_mode.run(SimpleNamespace(robot="nao", enable_motion=motion_enabled,
                                                       orient_camera=camera_enabled))
        return SimpleNamespace(code=code, result=json.loads(output.getvalue()),
                               session=session, init=init, shutdown=shutdown,
                               loggers=loggers, handlers=handlers, events=events)

    def test_successful_receiver_cleanup_allows_context_destruction(self):
        observed = self.run_preview()
        self.assertEqual(observed.code, 0)
        self.assertEqual(observed.result["outcome"], follow_mode.APP_SUCCEEDED)
        self.assertTrue(observed.result["released"])
        self.assertEqual([call[0] for call in observed.events.mock_calls],
                         ["close_bond", "close_perception", "shutdown_ros"])
        observed.loggers.assert_called_once_with(observed.session.node)
        observed.session.finish_preview.assert_called_once_with("preview_finished")
        observed.init.assert_called_once()
        self.assertEqual(observed.handlers.call_count, 4)

    def test_diagnostic_failure_cannot_skip_people_release(self):
        observed = self.run_preview(diagnostic_error=RuntimeError("diagnostic publisher failed"))
        self.assertEqual(observed.code, 1)
        observed.session.release_fallback.assert_called_once()
        observed.session.bond.shutdown.assert_called_once()
        observed.session.close_perception.assert_called_once()
        observed.shutdown.assert_called_once()
        self.assertIn("diagnostic publisher failed", observed.result["message"])

    def test_motion_stop_failure_cannot_skip_either_release(self):
        observed = self.run_preview(motion_enabled=True,
                                    motion_stop_error=RuntimeError("stop transport failed"))
        self.assertEqual(observed.code, 1)
        observed.session.motion.release.assert_called_once()
        observed.session.release_fallback.assert_called_once()
        observed.session.close_perception.assert_called_once()
        self.assertTrue(observed.result["motion_released"])
        self.assertTrue(observed.result["people_released"])
        self.assertFalse(observed.result["stop_acknowledged"])

    def test_motion_release_exception_cannot_skip_people_release(self):
        observed = self.run_preview(motion_enabled=True,
                                    motion_release_error=RuntimeError("motion release transport failed"))
        self.assertEqual(observed.code, 1)
        observed.session.release_fallback.assert_called_once()
        observed.session.close_perception.assert_called_once()
        self.assertFalse(observed.result["released"])
        self.assertFalse(observed.result["motion_released"])
        self.assertTrue(observed.result["people_released"])

    def test_bond_failure_still_closes_executor_and_context(self):
        observed = self.run_preview(bond_error=RuntimeError("bond shutdown failed"))
        self.assertEqual(observed.code, 1)
        observed.session.close_perception.assert_called_once()
        observed.shutdown.assert_called_once()
        self.assertIn("bond_cleanup_exception: bond shutdown failed", observed.result["message"])

    def test_camera_stop_failure_cannot_skip_camera_motion_or_people_release(self):
        observed = self.run_preview(motion_enabled=True, camera_enabled=True,
                                    camera_stop_error=RuntimeError("camera hold transport failed"))
        self.assertEqual(observed.code, 1)
        observed.session.camera.release.assert_called_once()
        observed.session.motion.release.assert_called_once()
        observed.session.release_fallback.assert_called_once()
        self.assertTrue(observed.result["camera_released"])
        self.assertFalse(observed.result["camera_hold_acknowledged"])
        self.assertTrue(observed.result["motion_released"])

    def test_camera_release_failure_cannot_skip_motion_or_people_release(self):
        observed = self.run_preview(motion_enabled=True, camera_enabled=True,
                                    camera_release_error=RuntimeError("camera release transport failed"))
        self.assertEqual(observed.code, 1)
        observed.session.motion.release.assert_called_once()
        observed.session.release_fallback.assert_called_once()
        self.assertFalse(observed.result["camera_released"])
        self.assertFalse(observed.result["released"])

    def test_receiver_cleanup_failure_retains_context_and_reports_failure(self):
        observed = self.run_preview(RuntimeError("receiver thread remains alive"))
        self.assertEqual(observed.code, 1)
        self.assertEqual(observed.result["outcome"], follow_mode.APP_FAILED)
        self.assertIn("perception_cleanup_exception: receiver thread remains alive",
                      observed.result["message"])
        self.assertTrue(observed.result["released"])
        self.assertTrue(observed.result["dry_run"])
        self.assertFalse(observed.result["motion_enabled"])
        observed.shutdown.assert_not_called()
        observed.session.bond.shutdown.assert_called_once_with()
        self.assertEqual(observed.handlers.call_count, 4)


if __name__ == "__main__":
    unittest.main()

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
    def run_preview(self, cleanup_error=None):
        session = MagicMock()
        session.frames = 10
        session.tracked_frames = 8
        session.samples = 20
        session.close_perception.side_effect = cleanup_error
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
                code = follow_mode.run(SimpleNamespace(robot="nao"))
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

"""Real POSIX subprocess shutdown tests; no ROS graph or hardware."""

import importlib.util
from contextlib import redirect_stderr
import io
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


SCRIPT = Path(__file__).resolve().parents[1] / "hri_reference_app/scripts/graceful_server.py"
SPEC = importlib.util.spec_from_file_location("follow_server_test", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@unittest.skipUnless(os.name == "posix", "POSIX signals and sessions required")
class FollowServerShutdownTest(unittest.TestCase):
    def test_invalid_cli_grace_never_spawns(self):
        for args in (["--grace-seconds"], ["--grace-seconds", "32"],
                     ["--grace-seconds", "nan", "--", "unused"],
                     ["--grace-seconds", "61", "--", "unused"],
                     ["--grace-seconds", "0", "--", "unused"],
                     ["--grace-seconds", "32", "wrong", "unused"]):
            with self.subTest(args=args), patch.object(MODULE.subprocess, "Popen") as spawn, \
                    redirect_stderr(io.StringIO()):
                self.assertEqual(MODULE.main(args), 2)
                spawn.assert_not_called()

    def test_cli_grace_keeps_child_arguments_and_exit_code(self):
        self.assertEqual(MODULE.main(["--grace-seconds", "32", "--", sys.executable,
                                      "-c", "raise SystemExit(7)"]), 7)

    def start(self, child_code, *, grace=0.3, interrupt=0.15, terminate=0.15):
        runner = (
            "import importlib.util; "
            f"s=importlib.util.spec_from_file_location('wrapper',{str(SCRIPT)!r}); "
            "m=importlib.util.module_from_spec(s); s.loader.exec_module(m); "
            f"raise SystemExit(m.main({[sys.executable, '-u', '-c', child_code]!r}, "
            f"grace_seconds={grace}, interrupt_seconds={interrupt}, "
            f"terminate_seconds={terminate}, poll_seconds=0.005))"
        )
        process = subprocess.Popen([sys.executable, "-u", "-c", runner],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   text=True, start_new_session=True)
        self.addCleanup(self.cleanup, process)
        self.assertTrue(select.select([process.stdout], [], [], 3.0)[0], "child not ready")
        line = process.stdout.readline()
        self.assertTrue(line.startswith("READY "), line)
        return process, int(line.split()[1])

    @staticmethod
    def cleanup(process):
        if process.poll() is None:
            process.terminate()
            try:
                process.communicate(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate(timeout=3.0)
        else:
            process.communicate()

    @staticmethod
    def child(ignore=False, return_code=0):
        handler = "signal.SIG_IGN" if ignore else f"lambda *_: sys.exit({return_code})"
        return (
            "import os, signal, sys, time; "
            f"signal.signal(signal.SIGINT, {handler}); "
            f"signal.signal(signal.SIGTERM, {handler}); "
            "print('READY',os.getpid(),flush=True); "
            "time.sleep(30)"
        )

    def assert_reaped(self, pid):
        with self.assertRaises(ProcessLookupError):
            os.kill(pid, 0)

    def test_terminal_group_sigint_gives_one_grace_period(self):
        process, pid = self.start(self.child())
        started = time.monotonic()
        os.killpg(process.pid, signal.SIGINT)
        time.sleep(0.06)
        self.assertIsNone(process.poll())
        os.kill(pid, 0)  # Child did not receive the terminal's process-group SIGINT.
        process.send_signal(signal.SIGINT)
        time.sleep(0.06)
        self.assertIsNone(process.poll())  # Repeated SIGINT cannot shorten grace.
        _, errors = process.communicate(timeout=2.0)
        elapsed = time.monotonic() - started
        self.assertEqual(process.returncode, 0, errors)
        self.assertGreaterEqual(elapsed, 0.27)
        self.assertLess(elapsed, 0.65)  # It cannot restart/extend grace either.
        self.assert_reaped(pid)

    def test_sigterm_ends_pending_grace_immediately(self):
        process, pid = self.start(self.child(), grace=8.0)
        process.send_signal(signal.SIGINT)
        started = time.monotonic()
        process.terminate()
        _, errors = process.communicate(timeout=2.0)
        self.assertEqual(process.returncode, 0, errors)
        self.assertLess(time.monotonic() - started, 1.0)
        self.assert_reaped(pid)

    def test_missing_client_exit_falls_back_to_kill_and_reaps(self):
        process, pid = self.start(self.child(ignore=True), grace=0.1)
        started = time.monotonic()
        process.send_signal(signal.SIGINT)
        process.communicate(timeout=2.0)
        self.assertEqual(process.returncode, 128 + signal.SIGKILL)
        self.assertGreaterEqual(time.monotonic() - started, 0.38)
        self.assert_reaped(pid)

    def test_child_exit_code_is_preserved(self):
        code = "import os,sys; print('READY',os.getpid(),flush=True); sys.exit(7)"
        process, pid = self.start(code)
        process.communicate(timeout=2.0)
        self.assertEqual(process.returncode, 7)
        self.assert_reaped(pid)

    def test_exception_still_stops_and_reaps_owned_child(self):
        child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
        try:
            clock = SimpleNamespace(monotonic=time.monotonic,
                                    sleep=Mock(side_effect=RuntimeError("loop failure")))
            with patch.object(MODULE.subprocess, "Popen", return_value=child), \
                    patch.object(MODULE, "time", clock):
                with self.assertRaisesRegex(RuntimeError, "loop failure"):
                    MODULE.main(["unused"], interrupt_seconds=0.1, terminate_seconds=0.1)
            self.assertIsNotNone(child.returncode)
            self.assert_reaped(child.pid)
        finally:
            if child.poll() is None:
                child.kill()
                child.wait()


if __name__ == "__main__":
    unittest.main()

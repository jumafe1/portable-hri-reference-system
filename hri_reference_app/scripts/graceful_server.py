#!/usr/bin/env python3
"""Keep Capabilities2 alive briefly while the HRI client stops and releases.

SIGINT (including terminal process-group delivery) starts one grace period.
SIGTERM means the client has exited, or launch requires shutdown: end that grace
immediately. The isolated child then gets SIGINT, SIGTERM and SIGKILL, each with
a bounded wait. This wrapper never changes robot motion or safety settings.
"""

import math
import signal
import subprocess
import sys
import time


def _send(child, signum):
    try:
        child.send_signal(signum)
    except ProcessLookupError:
        pass


def _reap(child, interrupt_seconds, terminate_seconds):
    """Also used on exceptions: never leave the owned child running or a zombie."""
    for signum, seconds in ((signal.SIGINT, interrupt_seconds),
                           (signal.SIGTERM, terminate_seconds)):
        if child.poll() is not None:
            child.wait()
            return
        _send(child, signum)
        try:
            child.wait(timeout=seconds)
            return
        except subprocess.TimeoutExpired:
            pass
    _send(child, signal.SIGKILL)
    child.wait()


def main(argv=None, *, grace_seconds=8.0, interrupt_seconds=2.0,
         terminate_seconds=2.0, poll_seconds=0.02):
    command = sys.argv[1:] if argv is None else list(argv)
    if command and command[0] == "--grace-seconds":
        try:
            grace_seconds = float(command[1])
            if command[2] != "--" or not math.isfinite(grace_seconds) or not 0 < grace_seconds <= 60:
                raise ValueError()
        except (IndexError, ValueError):
            print("Use --grace-seconds (0,60] -- executable [arguments ...]", file=sys.stderr)
            return 2
        command = command[3:]
    if not command:
        print("Usage: graceful_server.py executable [arguments ...]", file=sys.stderr)
        return 2

    # Handlers only change state, including when a signal arrives during Popen.
    deadline = None
    phase = 0

    def shutdown(signum, _frame):
        nonlocal deadline
        if phase:
            return  # Duplicate/group signals must not restart escalation timers.
        now = time.monotonic()
        if signum == signal.SIGTERM:
            deadline = now
        elif deadline is None:
            deadline = now + grace_seconds

    previous = {sig: signal.signal(sig, shutdown) for sig in (signal.SIGINT, signal.SIGTERM)}
    child = None
    try:
        # The terminal/launch process group must not interrupt the server before
        # the application has had a chance to stop motion and release ownership.
        child = subprocess.Popen(command, start_new_session=True)
        while True:
            code = child.poll()
            if code is not None:
                return code if code >= 0 else 128 - code
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                if phase == 0:
                    _send(child, signal.SIGINT)
                    phase, deadline = 1, now + interrupt_seconds
                elif phase == 1:
                    _send(child, signal.SIGTERM)
                    phase, deadline = 2, now + terminate_seconds
                else:
                    _send(child, signal.SIGKILL)
                    code = child.wait()
                    return code if code >= 0 else 128 - code
            time.sleep(poll_seconds)
    except OSError as error:
        print(f"Unable to run capability server: {error}", file=sys.stderr)
        return 127
    finally:
        try:
            if child is not None:
                _reap(child, interrupt_seconds, terminate_seconds)
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)


if __name__ == "__main__":
    raise SystemExit(main())

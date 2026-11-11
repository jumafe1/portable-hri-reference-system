"""Short startup greeting through Speak, before microphone or motion acquisition."""

import time

import rclpy
from capabilities2_msgs.srv import FreeCapability, GetRunningCapabilities, UseCapability
from hri_capability_interfaces.srv import Speak
from yasmin_ros.basic_outcomes import ABORT, SUCCEED


CAPABILITY = "hri_capability_interfaces/Speak"
PROVIDER = "hri_naoqi_providers/NaoSpeak"


class StartupSpeak:
    def __init__(self, session):
        self.session, self.node = session, session.node
        self.provider = session.args.speak_provider
        self.requested = self.released = False
        self.bond_id = ""
        self.status = "not_started"
        self.acquire = self.node.create_client(UseCapability, "/capabilities/use_capability")
        self.speak = self.node.create_client(Speak, "/hri/speak")

    def _call(self, client, request, seconds=3.0, *, cleanup=False):
        deadline = time.monotonic() + seconds
        future = None
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                if not cleanup and (self.session.cancelled.is_set() or self.session.bond.is_broken()):
                    raise RuntimeError("cancelled_or_bond_broken")
                if future is None and client.service_is_ready():
                    future = client.call_async(request)
                if future is not None and future.done():
                    return future.result()  # Service exceptions also trigger cleanup.
                time.sleep(.02)
            raise RuntimeError("service_timeout")
        finally:
            if future is not None and not future.done():
                future.cancel()  # Never retry a possibly already spoken greeting.

    def execute(self, blackboard):
        self.bond_id = blackboard["bond_id"]
        result = ABORT
        try:
            self.requested = True  # A lost acquisition reply still needs release.
            self._call(self.acquire, UseCapability.Request(
                capability=CAPABILITY, preferred_provider=self.provider, bond_id=self.bond_id))
            running = self._call(self.session.running_client, GetRunningCapabilities.Request())
            if not any(item.capability.capability == CAPABILITY and item.capability.provider == self.provider
                       for item in running.running_capabilities):
                raise RuntimeError("provider_not_running")
            response = self._call(self.speak, Speak.Request(
                text=self.session.args.startup_text, language=self.session.args.speak_language), seconds=7.0)
            if not response.success or response.provider != self.provider:
                raise RuntimeError("speak_not_acknowledged")
            self.status = "completed"
            self.node.get_logger().info("startup_speech_completed: saludo terminado")
            result = SUCCEED
        except Exception:
            self.status = "failed"
            blackboard["message"] = "startup_speech_failed"
        finally:
            if self.release() != SUCCEED:
                blackboard["message"] = "startup_speech_release_failed"
                result = ABORT
        return result

    def release(self):
        if not self.requested or self.released:
            return SUCCEED
        try:
            self._call(self.session.release_client, FreeCapability.Request(
                capability=CAPABILITY, bond_id=self.bond_id), cleanup=True)
            self.released = True
            return SUCCEED
        except Exception:
            return ABORT

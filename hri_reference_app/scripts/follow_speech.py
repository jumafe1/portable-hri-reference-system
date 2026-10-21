"""Optional serial speech windows alongside visual follow; no inference in this process."""

import json
import threading
import time
import uuid

import rclpy
from capabilities2_msgs.srv import FreeCapability, GetInterfaces, GetProviders, GetRunningCapabilities, UseCapability
from hri_capability_interfaces.msg import SpeechTranscript
from hri_capability_interfaces.srv import TranscribeSpeech
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from yasmin_ros.basic_outcomes import ABORT, SUCCEED


CAPABILITY = "hri_capability_interfaces/TranscribeSpeech"
PROVIDER = "hri_whisper_providers/WhisperTranscribeSpeech"
STATUS_TOPIC = "/hri/follow_person/speech_status"
# Internal bounded buffers, not a limit on how long follow speech stays enabled.
CAPTURE_WINDOW_SECONDS = 3.0


class FollowSpeech:
    def __init__(self, session):
        self.session, self.node, self.args = session, session.node, session.args
        self.provider = self.args.speech_provider
        self.requested = self.active = self.released = self.closed = False
        self.status = "not_configured"
        self.transcripts_received = 0
        self.bond_id = self.request_id = ""
        self.future = self.release_future = self.result = None
        self.started_stamp = self.result_deadline = self.admission_deadline = 0
        self.retry_until = self.retry_at = self.capture_until = 0
        self.release_deadline = 0
        self.lock = threading.Lock()
        self.interfaces = self.node.create_client(GetInterfaces, "/capabilities/get_interfaces")
        self.providers = self.node.create_client(GetProviders, "/capabilities/get_providers")
        self.acquire = self.node.create_client(UseCapability, "/capabilities/use_capability")
        self.transcribe = self.node.create_client(TranscribeSpeech, "/hri/speech/transcribe")
        self.subscription = self.node.create_subscription(
            SpeechTranscript, "/hri/speech/transcript", self.on_result, 10)
        self.publisher = self.node.create_publisher(
            String, STATUS_TOPIC, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _status(self, status):
        self.status = status
        try:
            self.publisher.publish(String(data=json.dumps({
                "speech_status": status, "provider": self.provider,
                "request_id": self.request_id, "transcripts_received": self.transcripts_received,
                "speech_released": not self.requested or self.released,
            })))
        except Exception:
            pass  # Diagnostic delivery must never prevent base stop or capability cleanup.

    def _call(self, client, request, *, cleanup=False):
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < deadline:
            if self.session.cancelled.is_set() and not cleanup:
                raise RuntimeError("cancelled")
            if client.service_is_ready():
                break
            time.sleep(0.02)
        if not rclpy.ok() or not client.service_is_ready():
            raise RuntimeError("service_unavailable")
        if self.session.cancelled.is_set() and not cleanup:
            raise RuntimeError("cancelled")
        future = client.call_async(request)
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            if self.session.cancelled.is_set() and not cleanup:
                raise RuntimeError("cancelled")
            time.sleep(0.02)
        if not future.done() or future.exception() is not None:
            raise RuntimeError("service_failed")
        return future.result()

    def configure(self, blackboard):
        """Bounded optional setup before motion acquisition; errors never start audio."""
        self.bond_id = blackboard["bond_id"]
        try:
            if self.session.cancelled.is_set():
                self.freeze("cancelled")
                return ABORT
            if CAPABILITY not in self._call(self.interfaces, GetInterfaces.Request()).interfaces:
                self._disable("disabled_no_interface", release_async=False)
                return SUCCEED
            providers = self._call(self.providers, GetProviders.Request(interface=CAPABILITY)).providers
            if self.provider not in providers:
                self._disable("disabled_no_provider", release_async=False)
                return SUCCEED
            self.requested = True  # Lost acquisition replies still require release.
            self._call(self.acquire, UseCapability.Request(
                capability=CAPABILITY, preferred_provider=self.provider, bond_id=self.bond_id))
            running = self._call(self.session.running_client, GetRunningCapabilities.Request())
            if not any(item.capability.capability == CAPABILITY and item.capability.provider == self.provider
                       for item in running.running_capabilities):
                raise RuntimeError("provider_not_running")
            deadline = time.monotonic() + 3.0
            while rclpy.ok() and not self.session.cancelled.is_set() and time.monotonic() < deadline:
                count = self.node.count_publishers(self.subscription.topic_name)
                if count > 1:
                    raise RuntimeError("ambiguous_transcript_publishers")
                if count == 1 and self.transcribe.service_is_ready():
                    self.active = True
                    self._status("ready")
                    if self.args.print_transcript:
                        self.node.get_logger().warning(
                            "Transcript printing enabled: terminal/ROS launch logs may retain recognized text.")
                    return SUCCEED
                time.sleep(0.02)
            raise RuntimeError("provider_not_ready")
        except Exception:
            # Do not copy exception/provider payloads into logs: they may contain user data.
            self._disable("disabled_setup_failed", release_async=False)
            self.release()
            return ABORT if self.session.cancelled.is_set() else SUCCEED

    def on_result(self, result):
        with self.lock:
            if (self.closed or not self.active or self.session.cancelled.is_set()
                    or not self.request_id or result.request_id != self.request_id
                    or result.provider != self.provider or self.result is not None):
                return
            # Keep only one bounded result, not an unbounded transcript history.
            self.result = result if len(result.text) <= 8192 and len(result.message) <= 256 else False

    def _start(self, now, *, retry=False):
        if self.closed or self.session.cancelled.is_set():
            return
        if not retry:
            with self.lock:
                self.request_id, self.result = uuid.uuid4().hex, None
            self.started_stamp = self.node.get_clock().now().nanoseconds
            self.result_deadline = now + CAPTURE_WINDOW_SECONDS + 65.0
            self.retry_until = now + 2.0
        self.future = self.transcribe.call_async(TranscribeSpeech.Request(
            duration_sec=CAPTURE_WINDOW_SECONDS, language=self.args.speech_language,
            request_id=self.request_id))
        self.admission_deadline = now + 3.0
        self.retry_at = 0
        self._status("awaiting_admission")

    def _disable(self, reason, *, release_async=True):
        self.active = False
        with self.lock:
            self.result = None
        self._status(reason)
        try:
            self.node.get_logger().warning("Optional transcription disabled: " + reason)
        except Exception:
            pass
        try:
            if release_async and self.requested and not self.released and self.release_future is None:
                if self.session.release_client.service_is_ready():
                    self.release_future = self.session.release_client.call_async(FreeCapability.Request(
                        capability=CAPABILITY, bond_id=self.bond_id))
                    self.release_deadline = time.monotonic() + 3.0
        except Exception:
            self.release_future = None  # Final independent cleanup will retry; visual control continues.

    def _poll_release(self, now):
        if self.release_future is None:
            return
        if self.release_future.done():
            self.released = self.release_future.exception() is None
            self.release_future = None
            self._status(self.status)
        elif now >= self.release_deadline:
            self.release_future = None  # Final cleanup retries; no blocking/retry loop during motion.

    def tick(self):
        """Only ready futures are read; never waits for microphone, inference or ROS services."""
        try:
            self._tick(time.monotonic())
        except Exception:
            self._disable("disabled_transport_error")

    def _tick(self, now):
        self._poll_release(now)
        if self.closed or self.session.cancelled.is_set() or not self.active:
            return
        if (self.session.bond.is_broken() or not self.transcribe.service_is_ready()
                or self.node.count_publishers(self.subscription.topic_name) != 1):
            self._disable("disabled_provider_lost_or_ambiguous")
            return
        if self.status == "ready":
            self._start(now)
            return
        if self.retry_at:
            if now >= self.retry_at:
                if now >= self.retry_until:
                    self._disable("disabled_lease_unavailable")
                else:
                    self._start(now, retry=True)
            return
        if self.future is not None:
            if not self.future.done():
                if now >= self.admission_deadline:
                    self._disable("disabled_admission_timeout")
                return  # A lost admission reply is never retried.
            response = self.future.result()
            self.future = None
            if not response.accepted:
                if response.message == "lease_unavailable" and now < self.retry_until:
                    self.retry_at = now + 0.1
                    return
                self._disable("disabled_admission_rejected")
                return
            if response.request_id != self.request_id or response.provider != self.provider:
                self._disable("disabled_invalid_admission")
                return
            self.capture_until = now + CAPTURE_WINDOW_SECONDS
            self._status("capturing")
            self.node.get_logger().info("capture_started: hable ahora; captura de audio acotada")
        with self.lock:
            result = self.result
        if result is False:
            self._disable("disabled_oversized_result")
            return
        if result is not None:
            stamp = result.header.stamp.sec * 1_000_000_000 + result.header.stamp.nanosec
            if (not 0 <= result.header.stamp.nanosec < 1_000_000_000 or stamp <= 0
                    or stamp < self.started_stamp
                    or stamp > self.node.get_clock().now().nanoseconds + 100_000_000):
                self._disable("disabled_invalid_audio_timestamp")
                return
            if result.success and result.text.strip():
                self.transcripts_received += 1
                if self.args.print_transcript:
                    # JSON escaping avoids terminal control sequences in model-generated output.
                    self.node.get_logger().info("transcript: " + json.dumps(result.text, ensure_ascii=False))
            elif result.success or result.message not in ("no_speech", "no_speech_energy") or result.text:
                self._disable("disabled_transcription_failed")
                return
            with self.lock:
                self.request_id, self.result = "", None
            self._status("ready")  # Next tick, never an overlapping queued capture.
        elif now >= self.result_deadline:
            self._disable("disabled_result_timeout")
        elif self.status == "capturing" and now >= self.capture_until:
            self._status("processing")  # Approximate phase; only the result permits another request.

    def freeze(self, reason="stopped"):
        """Immediately prevent new captures and ignore late results; release follows base stop."""
        with self.lock:
            self.closed = True
            self.result = None
        if self.active:
            self.active = False
            self._status(reason)

    def release(self, _blackboard=None):
        self.freeze()
        if not self.requested or self.released:
            return SUCCEED
        try:
            self._call(self.session.release_client, FreeCapability.Request(
                capability=CAPABILITY, bond_id=self.bond_id), cleanup=True)
            self.released = True
            self._status(self.status)
            return SUCCEED
        except Exception:
            self._status("disabled_release_failed")
            return ABORT

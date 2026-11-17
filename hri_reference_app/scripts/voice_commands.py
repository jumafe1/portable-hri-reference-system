"""Latched voice pause and nonblocking Speak replies within the follow session."""
import random
import time

from capabilities2_msgs.srv import GetRunningCapabilities, UseCapability
from hri_capability_interfaces.srv import Speak
from yasmin_ros.basic_outcomes import ABORT, SUCCEED

from follow_controller import Decision
from startup_speak import StartupSpeak, CAPABILITY
from voice_command_policy import CommandPolicy


class VoiceCommands(StartupSpeak):
    def __init__(self, session):
        super().__init__(session)
        self.policy = CommandPolicy(session.args.voice_commands_config)
        self.paused = self.policy.config['start_paused']
        self.pending = ''
        self.reply_future = None
        self.reply_deadline = self.cooldown_until = 0.0
        self.ignore_before_ns = self.last_command_stamp = 0
        self.zero_sent = None
        self.fault = ''
        self.last_intent, self.last_intent_at = '', -float('inf')
        self.resume_stamp = self.resume_seen = self.resume_frames = 0
        self.accepted_commands = self.responses_sent = 0

    def configure(self, blackboard):
        self.bond_id = blackboard['bond_id']
        try:
            self.requested = True
            self._call(self.acquire, UseCapability.Request(
                capability=CAPABILITY, preferred_provider=self.provider, bond_id=self.bond_id))
            running = self._call(self.session.running_client, GetRunningCapabilities.Request())
            if not any(item.capability.capability == CAPABILITY and item.capability.provider == self.provider
                       for item in running.running_capabilities):
                raise RuntimeError('speak_provider_not_running')
            if not self.speak.wait_for_service(timeout_sec=3.0):
                raise RuntimeError('speak_unavailable')
            if not self.session.speech.active:
                raise RuntimeError('recognizer_unavailable')
            self.status = 'paused' if self.paused else 'following'
            return SUCCEED
        except Exception:
            self.fault = self.status = 'voice_setup_failed'
            blackboard['message'] = self.status
            return ABORT

    def receive(self, text, stamp_ns):
        now = time.monotonic()
        ros_now = self.node.get_clock().now().nanoseconds
        if (self.session.cancelled.is_set() or self.fault or self.reply_future is not None
                or now < self.cooldown_until or stamp_ns <= max(self.ignore_before_ns, self.last_command_stamp)
                or not 0 <= (ros_now - stamp_ns) / 1e9 <= self.policy.config['max_command_age_s']):
            return
        intent = self.policy.intent(text)
        if intent is None:
            return
        self.last_command_stamp = stamp_ns
        if intent == self.last_intent and (now - self.last_intent_at < 2 or self.pending == intent):
            return
        self.last_intent, self.last_intent_at = intent, now
        self.accepted_commands += 1
        # Both commands first hold the base. A follow request is committed only
        # after stop acknowledgment and a successful completed Speak response.
        self.paused = True
        self.pending = intent
        self.zero_sent = None
        self.status = 'stopping_for_' + intent

    def healthy(self):
        speech = self.session.speech
        return bool(speech and speech.active and speech.has_captured and not speech.closed)

    def tick(self):
        if self.session.cancelled.is_set() or self.session.controller.halted:
            self.paused, self.pending = True, ''
            return
        now = time.monotonic()
        if not self.healthy():
            # Startup admission is a temporary hold. Once a running recognizer
            # fails, latch the pause; recovered vision cannot restart following.
            if self.session.speech.has_captured or not self.session.speech.active:
                self.paused = True
            self.pending = ''
        if self.reply_future is not None:
            if self.reply_future.done():
                try:
                    response = self.reply_future.result()
                    if not response.success or response.provider != self.provider:
                        raise RuntimeError('speak_failed')
                except Exception:
                    self.fault = 'voice_response_failed'
                self.reply_future = None
                intent, self.pending = self.pending, ''
                self.paused = intent != 'follow' or bool(self.fault) or not self.healthy()
                self.cooldown_until = now + self.policy.config['echo_guard_s']
                self.ignore_before_ns = self.node.get_clock().now().nanoseconds + int(self.policy.config['echo_guard_s'] * 1e9)
                self.resume_stamp = self.ignore_before_ns
                self.resume_frames, self.resume_seen = 0, 0
                self.status = self.fault or ('paused' if self.paused else 'reacquiring_after_voice')
            elif now >= self.reply_deadline:
                # Do not queue more speech or movement when completion is unknown.
                self.fault, self.paused = 'voice_response_timeout', True
            return
        if (self.pending and not self.fault and self.healthy()
                and self.session._base_stop_acknowledged(self.zero_sent, now)):
            try:
                if not self.speak.service_is_ready():
                    raise RuntimeError('speak_unavailable')
                self.reply_future = self.speak.call_async(Speak.Request(
                    text=random.choice(self.policy.responses[self.pending]), language=self.session.args.speak_language))
                self.reply_deadline = now + 7.0
                self.responses_sent += 1
                self.status = 'responding_' + self.pending
            except Exception:
                self.fault, self.paused = 'voice_response_failed', True

    def gate(self, decision):
        if self.session.controller.halted:
            return decision
        reason = ''
        if not self.healthy():
            reason = 'voice_unavailable'
        elif self.fault:
            reason = self.fault
        elif self.pending or self.reply_future is not None or time.monotonic() < self.cooldown_until:
            reason = 'voice_reply_hold'
        elif self.paused:
            reason = 'paused_by_voice'
        elif self.resume_stamp:
            stamp = self.session.latest_stamp
            if stamp != self.resume_seen:
                self.resume_seen = stamp
                if (stamp > self.resume_stamp and decision.locked and decision.reason in {
                        'preview_following', 'target_in_visual_deadband', 'target_too_close_visual', 'preview_centering_close'}):
                    self.resume_frames += 1
                else:
                    self.resume_frames = 0
            if self.resume_frames < 3:
                reason = 'reacquiring_after_voice'
            else:
                self.resume_stamp = 0
                self.status = 'following'
        return Decision(0.0, 0.0, reason, decision.locked) if reason else decision

    def sent(self, decision):
        if self.pending and self.zero_sent is None and decision.vx == 0 and decision.wz == 0:
            self.zero_sent = time.monotonic()

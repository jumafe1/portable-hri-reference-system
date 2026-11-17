#!/usr/bin/env python3
"""Real Vosk + real Capabilities2 + synthetic Spanish PCM and fake NAO services.
No hardware. Requires --software-only, localhost DDS, empty domain 100..200.
Uses libespeak-ng only to synthesize audio in memory, never the sound device.
"""
import audioop
import ctypes as c
import json
from pathlib import Path
import queue
import signal
import subprocess
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from audio_common_msgs.msg import AudioInfo, AudioDataStamped
from naoqi_utilities_msgs.srv import Say
import yaml

from capabilities_naoqi_move_velocity_probe import software_guard, wait_until, zero
from unified_hri_follow_speech_probe import assert_empty_graph
from unified_hri_follow_motion_probe import MotionFixture, MotionFrames


class SyntheticMic:
    def __init__(self, node):
        self.node = node
        self.queue = queue.Queue(maxsize=100)
        self.lib = c.CDLL('libespeak-ng.so.1')
        self.rate = self.lib.espeak_Initialize(1, 0, None, 0)  # retrieval, no playback
        self.lib.espeak_SetVoiceByName(b'es')
        self.buffer = bytearray()
        @c.CFUNCTYPE(c.c_int, c.POINTER(c.c_short), c.c_int, c.c_void_p)
        def callback(wav, count, _events):
            if wav:
                self.buffer.extend(c.string_at(wav, count * 2))
            return 0
        self.callback = callback
        self.lib.espeak_SetSynthCallback(callback)
        self.info = node.create_publisher(AudioInfo, '/test/mic_info',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.info.publish(AudioInfo(channels=1, sample_rate=16000, sample_format='S16LE', coding_format='wave'))
        self.publisher = node.create_publisher(AudioDataStamped, '/test/mic', 4)
        self.timer = node.create_timer(.1, self.send)

    def say(self, phrase):
        self.buffer.clear()
        text = phrase.encode('utf-8')
        self.lib.espeak_Synth(text, len(text)+1, 0, 1, 0, 1 | 0x1000, None, None)
        self.lib.espeak_Synchronize()
        pcm = audioop.ratecv(bytes(self.buffer), 2, 1, self.rate, 16000, None)[0] + bytes(32000)
        for i in range(0, len(pcm), 3200):
            self.queue.put(pcm[i:i+3200].ljust(3200, b'\0'), timeout=5)

    def send(self):
        try:
            pcm = self.queue.get_nowait()
        except queue.Empty:
            pcm = bytes(3200)
        msg = AudioDataStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.audio.data = list(pcm)
        self.publisher.publish(msg)


def main():
    software_guard()
    root = Path(__file__).resolve().parents[1]
    evidence = Path(tempfile.mkdtemp(prefix='hri-vosk-voice-probe-'))
    profile = yaml.safe_load((root/'hri_reference_app/config/nao.yaml').read_text())
    profile['application'].update(expected_frame='test_camera', audio_topic='/test/mic', audio_info_topic='/test/mic_info')
    path = evidence/'profile.yaml'
    path.write_text(yaml.safe_dump(profile))
    rclpy.init()
    node = Node('software_vosk_voice_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    process = frames = mic = None
    checks, spoken = [], []
    def check(name, predicate, seconds=10):
        passed = wait_until(predicate, seconds)
        checks.append({'check': name, 'passed': passed})
        print(json.dumps(checks[-1]), flush=True)
        if not passed:
            raise RuntimeError(name)
    try:
        assert_empty_graph(node)
        fixture = MotionFixture(node)
        fixture.begin('nao')
        frames = MotionFrames(fixture, 'steady')
        mic = SyntheticMic(node)
        def speak(req, response):
            samples = fixture.recorders['/nao/cmd_vel'].since(time.monotonic() - .5)
            spoken.append({'text': req.text, 'base_zero': bool(samples) and all(zero(v) for _, v in samples)})
            response.success, response.message = True, 'synthetic speech completed'
            return response
        node.create_service(Say, '/nao/say', speak)
        output = (evidence/'launch.log').open('w')
        process = subprocess.Popen(['ros2', 'launch', 'hri_reference_app', 'aplicacion_hri.launch.py',
            'start_yolo:=false', 'profile:='+str(path)], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        def latest():
            values = fixture.snapshot(fixture.case_start)[0]
            return values[-1] if values else {}
        def moving():
            return any(not zero(v) for _, v in fixture.recorders['/nao/cmd_vel'].since(time.monotonic()-.3))
        check('ready_and_paused', lambda: latest().get('reason') == 'paused_by_voice', 30)
        check('no_motion_before_command', lambda: not fixture.moved())
        mic.say('nao sígueme')
        check('follow_command_moves_fake_base', moving)
        mic.say('nao detente')
        check('pause_command_stops', lambda: latest().get('reason') == 'paused_by_voice' and len(spoken) >= 2)
        time.sleep(1.2)
        check('valid_vision_does_not_clear_voice_pause', lambda: not moving())
        mic.say('te voy a seguir')
        time.sleep(3.5)
        check('response_without_wake_does_not_resume', lambda: not moving())
        mic.say('nao ven conmigo')
        check('synonym_resumes', moving)
        frames.case = 'person_loss'
        check('person_missing_still_stops', lambda: latest().get('reason') == 'person_missing' and not moving())
        frames.case = 'steady'
        check('vision_recovers_while_follow_enabled', moving)
        mic.say('nao no me sigas')
        check('negative_command_pauses', lambda: latest().get('reason') == 'paused_by_voice' and len(spoken) >= 4)
        check('all_responses_after_stop', lambda: len(spoken) >= 4 and all(x['base_zero'] for x in spoken))
        check('application_stays_alive', lambda: process.poll() is None)
    finally:
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=40)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if frames:
            frames.close()
        executor.shutdown(timeout_sec=2)
        spin.join(timeout=2)
        if mic:
            mic.lib.espeak_Terminate()
        node.destroy_node()
        rclpy.shutdown()
        report = {'checks': checks, 'spoken': spoken, 'real_vosk': True, 'real_audio': False,
                  'physical_success_verified': False, 'evidence': str(evidence)}
        (evidence/'result.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
        print('Evidence: '+str(evidence), flush=True)
    return 0 if checks and all(x['passed'] for x in checks) else 1

if __name__ == '__main__':
    raise SystemExit(main())

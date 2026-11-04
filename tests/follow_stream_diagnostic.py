#!/usr/bin/env python3
"""Passive timing observer: no capabilities, services, inference or motion commands.

Stores timestamps only, never image pixels. Best-effort reception can miss
samples: gaps localize the next investigation, not prove a producer failure.
Run in the same domain as the driver/YOLO, optionally alongside follow preview.
"""

import argparse
from collections import deque
import json
from pathlib import Path
import tempfile
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from yolo_msgs.msg import DetectionArray
from hri_capability_interfaces.msg import PersonDetection2DArray


def summary(samples, now):
    if not samples:
        return {"samples": 0}
    gaps = [b['receipt_monotonic_s'] - a['receipt_monotonic_s']
            for a, b in zip(samples, list(samples)[1:])]
    ages = [s['source_age_s'] for s in samples]
    return {"samples": len(samples), "max_receipt_gap_s": max(gaps, default=0.0),
            "since_last_receipt_s": now - samples[-1]['receipt_monotonic_s'],
            "source_age_min_s": min(ages), "source_age_max_s": max(ages)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=30, choices=range(1, 301), metavar='1..300')
    args = parser.parse_args()
    streams = {'camera': ('/nao/camera/front/image_raw', Image),
               'yolo': ('/yolo/detections', DetectionArray),
               'people': ('/hri/people', PersonDetection2DArray)}
    samples = {key: deque(maxlen=5000) for key in streams}
    path = Path(tempfile.mkdtemp(prefix='hri-stream-timing-')) / 'result.json'
    node = None
    report = {'topics': {key: value[0] for key, value in streams.items()},
              'observation_only': True, 'cancelled': False,
              'limits': ['Best-effort observer may miss messages.',
                         'No /hri/people messages are expected without active DetectPeople.',
                         'At most 5000 timestamp samples retained per topic; no image/audio content.']}
    print(f'Observando {args.seconds} s sin mover el robot. Resultado: {path}', flush=True)
    rclpy.init()
    try:
        node = Node('follow_stream_diagnostic')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        subscriptions = []

        def receive(key, message):
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            samples[key].append({'receipt_monotonic_s': time.monotonic(),
                                 'stamp_ns': stamp,
                                 'source_age_s': (node.get_clock().now().nanoseconds - stamp) / 1e9})

        for key, (topic, kind) in streams.items():
            subscriptions.append(node.create_subscription(
                kind, topic, lambda msg, key=key: receive(key, msg), qos))
        start = last_print = last_spin = time.monotonic()
        max_spin_gap = 0.0
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            max_spin_gap = max(max_spin_gap, now - last_spin)
            last_spin = now
            if now - last_print >= 5:
                print(json.dumps({k: summary(v, now) for k, v in samples.items()}), flush=True)
                last_print = now
        report['max_observer_spin_gap_s'] = max_spin_gap
    except (KeyboardInterrupt, ExternalShutdownException):
        report['cancelled'] = True
    except Exception as error:
        report['error'] = str(error)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        now = time.monotonic()
        report['summary'] = {k: summary(v, now) for k, v in samples.items()}
        report['samples'] = {k: list(v) for k, v in samples.items()}
        # Receipt differences for identical image stamps separate upstream
        # capture gaps from delay between observable processing stages.
        report['matched_stage_delays'] = {}
        for source, target in (('camera', 'yolo'), ('yolo', 'people')):
            receipts = {s['stamp_ns']: s['receipt_monotonic_s'] for s in samples[source]}
            delays = [s['receipt_monotonic_s'] - receipts[s['stamp_ns']]
                      for s in samples[target] if s['stamp_ns'] in receipts]
            report['matched_stage_delays'][f'{source}_to_{target}'] = {
                'matched_samples': len(delays), 'min_s': min(delays, default=None),
                'max_s': max(delays, default=None)}
        path.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report['summary'], indent=2), flush=True)
        print(f'Resultado guardado: {path}', flush=True)
    return 1 if 'error' in report else 0


if __name__ == '__main__':
    raise SystemExit(main())

"""Pure geometry checks: no ROS graph, network or hardware."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hri_reference_app/scripts"))

from camera_geometry import Calibration, calibration_from_info, elevation_step
from follow_controller import Config, Controller, Person


class CameraGeometryTest(unittest.TestCase):
    def setUp(self):
        self.calibration = Calibration(320, 240, 280.0, 280.0, 160.0, 120.0)

    def person(self, top=50.0, bottom=190.0, x=160.0, width=50.0):
        return Person(0.9, "", x, (top + bottom) / 2, width, bottom - top)

    def info(self):
        return SimpleNamespace(header=SimpleNamespace(frame_id="camera_optical"),
                               width=320, height=240, binning_x=0, binning_y=0,
                               roi=SimpleNamespace(x_offset=0, y_offset=0, width=0, height=0),
                               k=[280.0, 0.0, 160.0, 0.0, 280.0, 120.0, 0.0, 0.0, 1.0],
                               d=[0.0] * 5, distortion_model="plumb_bob")

    def test_centered_ratio_matches_original_box_height(self):
        self.assertAlmostEqual(self.calibration.equivalent_height_ratio(self.person()), 140 / 240)

    def test_pitch_reprojection_preserves_angular_height_not_raw_occupancy(self):
        original = self.person(65, 175)
        original_ratio = self.calibration.equivalent_height_ratio(original)
        for pitch in (-0.15, -0.10, 0.05, 0.15):
            top = 120 + 280 * math.tan(math.atan((65 - 120) / 280) + pitch)
            bottom = 120 + 280 * math.tan(math.atan((175 - 120) / 280) + pitch)
            tilted = self.person(top, bottom)
            self.assertNotAlmostEqual(tilted.height / 240, original.height / 240)
            self.assertAlmostEqual(self.calibration.equivalent_height_ratio(tilted), original_ratio, places=12)

    def test_pitch_only_changes_do_not_change_controller_forward_velocity(self):
        speeds = []
        for pitch in (-0.1, 0.0, 0.1):
            top = 120 + 280 * math.tan(math.atan(-55 / 280) + pitch)
            bottom = 120 + 280 * math.tan(math.atan(55 / 280) + pitch)
            person = self.person(top, bottom)
            controller = Controller(Config(320, 240, target_height_ratio=0.8))
            for now in (0.1, 0.2, 0.3):
                decision = controller.update([person], now,
                    height_ratio_override=self.calibration.equivalent_height_ratio(person))
            speeds.append(decision.vx)
        self.assertAlmostEqual(min(speeds), max(speeds), places=12)

    def test_distortion_inverse_recovers_known_ray(self):
        d = (0.12, -0.03, 0.002, -0.001, 0.001)
        calibration = Calibration(320, 240, 280.0, 280.0, 160.0, 120.0, d)
        x, y = 0.25, -0.30
        r2 = x * x + y * y
        radial = 1 + d[0] * r2 + d[1] * r2 ** 2 + d[4] * r2 ** 3
        xd = x * radial + 2 * d[2] * x * y + d[3] * (r2 + 2 * x * x)
        yd = y * radial + d[2] * (r2 + 2 * y * y) + 2 * d[3] * x * y
        recovered = calibration.ray(160 + 280 * xd, 120 + 280 * yd)
        self.assertAlmostEqual(recovered[0], x, places=8)
        self.assertAlmostEqual(recovered[1], y, places=8)

    def test_bad_distortion_rejects_instead_of_using_unconverged_value(self):
        for coefficient in (-100.0, 100.0):
            calibration = Calibration(320, 240, 280.0, 280.0, 160.0, 120.0,
                                      (coefficient, 0.0, 0.0, 0.0, 0.0))
            with self.assertRaises(ValueError):
                calibration.ray(300, 220)

    def test_clipped_boxes_are_rejected(self):
        for person in (self.person(0, 180), self.person(60, 240), self.person(x=25), self.person(x=295)):
            with self.assertRaisesRegex(ValueError, "clipped_detection"):
                self.calibration.equivalent_height_ratio(person)

    def test_controller_does_not_allow_override_to_bypass_clipping(self):
        controller = Controller(Config(320, 240))
        for now in (0.1, 0.2, 0.3):
            decision = controller.update([self.person(0, 180)], now, height_ratio_override=0.1)
        self.assertEqual((decision.vx, decision.wz, decision.reason), (0.0, 0.0, "clipped_detection"))

    def test_invalid_override_halts_after_acquisition(self):
        for value in (float("nan"), float("inf"), -0.1, 0.0):
            controller = Controller(Config(320, 240))
            for now in (0.1, 0.2, 0.3):
                decision = controller.update([self.person()], now, height_ratio_override=value)
            self.assertTrue(controller.halted)
            self.assertEqual(decision.reason, "invalid_angular_height_ratio")

    def test_info_accepts_matching_calibration(self):
        self.assertEqual(calibration_from_info(self.info(), 320, 240, "camera_optical"), self.calibration)

    def test_info_rejects_frame_size_intrinsics_and_unsupported_distortion(self):
        for mutation in (lambda i: setattr(i, "width", 640),
                         lambda i: setattr(i.header, "frame_id", "other"),
                         lambda i: i.k.__setitem__(0, 0.0),
                         lambda i: i.k.__setitem__(1, 0.2),
                         lambda i: i.d.__setitem__(0, float("nan")),
                         lambda i: setattr(i, "d", [0.1] * 4),
                         lambda i: setattr(i, "binning_y", 2)):
            info = self.info()
            mutation(info)
            with self.assertRaises(ValueError):
                calibration_from_info(info, 320, 240, "camera_optical")
        info = self.info()
        info.d, info.distortion_model = [0.1] * 5, "equidistant"
        with self.assertRaises(ValueError):
            calibration_from_info(info, 320, 240, "camera_optical")
        info.d = [0.0] * 4
        with self.assertRaises(ValueError):
            calibration_from_info(info, 320, 240, "camera_optical")

    def test_vertical_framing_sign_deadband_limits_and_both_ends_clipped(self):
        self.assertEqual(elevation_step(self.person(20, 100), 240, 0.0), 0.03)
        self.assertEqual(elevation_step(self.person(140, 220), 240, 0.0), -0.03)
        self.assertEqual(elevation_step(self.person(), 240, 0.0), 0.0)
        self.assertEqual(elevation_step(self.person(0, 200), 240, 0.24), 0.25)
        self.assertEqual(elevation_step(self.person(40, 240), 240, -0.24), -0.25)
        self.assertEqual(elevation_step(self.person(0, 240), 240, 0.12), 0.12)


if __name__ == "__main__":
    unittest.main()

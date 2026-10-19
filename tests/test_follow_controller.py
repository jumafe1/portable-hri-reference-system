"""Pure controller tests: no ROS, network, robot, or motion publisher."""

from dataclasses import replace
import importlib.util
from pathlib import Path
import sys
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "hri_reference_app/scripts/follow_controller.py"
SPEC = importlib.util.spec_from_file_location("follow_controller", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
Config = MODULE.Config
Controller = MODULE.Controller
Person = MODULE.Person


class FollowControllerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = Controller(Config(640, 480))
        self.person = Person(0.9, "person-1", 320.0, 240.0, 100.0, 120.0)

    def acquire(self, person=None):
        person = person or self.person
        self.controller.update([person], 0.0)
        self.controller.update([person], 0.1)
        return self.controller.update([person], 0.2)

    def assert_zero(self, decision) -> None:
        self.assertEqual((decision.vx, decision.wz), (0.0, 0.0))

    def test_awaits_three_observations(self) -> None:
        self.assert_zero(self.controller.tick(0.0))
        for now in (0.0, 0.1):
            decision = self.controller.update([self.person], now)
            self.assert_zero(decision)
            self.assertFalse(decision.locked)
        self.assertTrue(self.controller.update([self.person], 0.2).locked)

    def test_forward_saturates(self) -> None:
        decision = self.acquire()
        self.assertEqual(decision.vx, 0.03)
        self.assertEqual(decision.wz, 0.0)

    def test_right_turn_sign_and_saturation(self) -> None:
        decision = self.acquire(replace(self.person, center_x=540.0))
        self.assertEqual((decision.vx, decision.wz), (0.0, -0.10))

    def test_left_turn_sign_and_saturation(self) -> None:
        decision = self.acquire(replace(self.person, center_x=100.0))
        self.assertEqual((decision.vx, decision.wz), (0.0, 0.10))

    def test_moderate_heading_disables_forward(self) -> None:
        decision = self.acquire(replace(self.person, center_x=384.0))
        self.assertEqual(decision.vx, 0.0)
        self.assertAlmostEqual(decision.wz, -0.04)

    def test_small_heading_allows_forward_and_turn(self) -> None:
        decision = self.acquire(replace(self.person, center_x=352.0))
        self.assertEqual(decision.vx, 0.03)
        self.assertAlmostEqual(decision.wz, -0.02)

    def test_yaw_deadband(self) -> None:
        self.assertEqual(self.acquire(replace(self.person, center_x=330.0)).wz, 0.0)

    def test_visual_target_deadband(self) -> None:
        decision = self.acquire(replace(self.person, height=288.0))
        self.assert_zero(decision)
        self.assertEqual(decision.reason, "target_in_visual_deadband")

    def test_too_close_centered_stops(self) -> None:
        decision = self.acquire(replace(self.person, height=340.0))
        self.assert_zero(decision)
        self.assertEqual(decision.reason, "target_too_close_visual")

    def test_too_close_left_allows_centering_only(self) -> None:
        decision = self.acquire(replace(self.person, height=340.0, center_x=100.0))
        self.assertEqual((decision.vx, decision.wz), (0.0, 0.10))
        self.assertEqual(decision.reason, "preview_centering_close")

    def test_too_close_right_allows_centering_only(self) -> None:
        decision = self.acquire(replace(self.person, height=340.0, center_x=540.0))
        self.assertEqual((decision.vx, decision.wz), (0.0, -0.10))
        self.assertEqual(decision.reason, "preview_centering_close")

    def test_large_clipped_box_stops_before_close_centering(self) -> None:
        decision = self.acquire(replace(self.person, height=340.0, center_x=50.0))
        self.assert_zero(decision)
        self.assertEqual(decision.reason, "clipped_detection")

    def test_clipping_stops_all_motion(self) -> None:
        self.assert_zero(self.acquire(replace(self.person, center_x=50.0)))

    def test_outside_frame_fails_closed(self) -> None:
        decision = self.controller.update([replace(self.person, center_x=49.0)], 0.0)
        self.assertEqual(decision.reason, "invalid_detection")
        self.assert_zero(decision)

    def test_missing_after_lock_latches(self) -> None:
        self.acquire()
        self.assertEqual(self.controller.update([], 0.3).reason, "person_missing")
        self.assert_zero(self.controller.update([self.person], 0.4))

    def test_ambiguity_after_lock_latches(self) -> None:
        self.acquire()
        decision = self.controller.update([self.person, self.person], 0.3)
        self.assertEqual(decision.reason, "ambiguous_people")
        self.assert_zero(self.controller.update([self.person], 0.4))

    def test_id_change_does_not_interrupt_single_person(self) -> None:
        self.acquire()
        decision = self.controller.update([replace(self.person, tracking_id="other")], 0.3)
        self.assertEqual(decision.vx, 0.03)
        self.assertTrue(decision.locked)
        self.assertFalse(self.controller.halted)

    def test_tracked_to_untracked_preserves_lock(self) -> None:
        self.acquire()
        decision = self.controller.update([replace(self.person, tracking_id="")], 0.3)
        self.assertTrue(decision.locked)
        self.assertEqual(decision.vx, 0.03)

    def test_untracked_to_tracked_preserves_lock(self) -> None:
        untracked = replace(self.person, tracking_id="")
        self.assertTrue(self.acquire(untracked).locked)
        self.assertEqual(self.controller.update([self.person], 0.3).vx, 0.03)

    def test_large_centroid_jump_changes_heading_without_latching(self) -> None:
        self.acquire()
        decision = self.controller.update([replace(self.person, center_x=450.0)], 0.3)
        self.assertEqual(decision.vx, 0.0)
        self.assertAlmostEqual(decision.wz, -0.08125)
        self.assertFalse(self.controller.halted)

    def test_large_size_jump_updates_forward_proposal(self) -> None:
        self.acquire()
        decision = self.controller.update([replace(self.person, height=240.0)], 0.3)
        self.assertAlmostEqual(decision.vx, 0.01)
        self.assertFalse(self.controller.halted)

    def test_prelock_identity_changes_still_acquire_after_three_observations(self) -> None:
        self.controller.update([self.person], 0.0)
        other = replace(self.person, tracking_id="other")
        self.assertFalse(self.controller.update([other], 0.1).locked)
        self.assertTrue(self.controller.update([replace(other, tracking_id="")], 0.2).locked)

    def test_prelock_geometric_changes_still_acquire_after_three_observations(self) -> None:
        self.controller.update([self.person], 0.0)
        other = replace(self.person, center_x=450.0)
        self.controller.update([other], 0.1)
        self.assertTrue(self.controller.update([replace(other, height=240.0)], 0.2).locked)

    def test_observed_nao_width_jump_is_not_identity_failure(self) -> None:
        self.controller = Controller(Config(320, 240))
        narrow = Person(0.84, "", 20.65, 110.98, 41.28, 218.41)
        wide = Person(0.90, "", 38.50, 111.36, 77.0, 219.45)
        self.acquire(narrow)
        decision = self.controller.update([wide], 0.3)
        self.assert_zero(decision)
        self.assertEqual(decision.reason, "clipped_detection")
        self.assertTrue(decision.locked)
        self.assertFalse(self.controller.halted)
        # An unclipped observation can resume proposals without restarting.
        decision = self.controller.update([replace(wide, center_x=50.0)], 0.4)
        self.assertEqual((decision.vx, decision.wz), (0.0, 0.10))
        self.assertEqual(decision.reason, "preview_centering_close")

    def test_missing_after_geometry_and_id_changes_still_latches(self) -> None:
        self.acquire()
        self.controller.update([replace(self.person, tracking_id="other", center_x=450.0,
                                        height=240.0)], 0.3)
        self.assertEqual(self.controller.update([], 0.4).reason, "person_missing")
        decision = self.controller.update([self.person], 0.5)
        self.assert_zero(decision)
        self.assertEqual(decision.reason, "person_missing")

    def test_missing_prelock_resets_count(self) -> None:
        self.controller.update([self.person], 0.0)
        self.controller.update([], 0.1)
        self.controller.update([self.person], 0.2)
        self.assertFalse(self.controller.update([self.person], 0.3).locked)

    def test_low_confidence_is_filtered(self) -> None:
        low = replace(self.person, confidence=0.2)
        for now in (0.0, 0.1, 0.2):
            decision = self.controller.update([self.person, low], now)
        self.assertTrue(decision.locked)

    def test_low_confidence_only_after_lock_stops(self) -> None:
        self.acquire()
        self.assert_zero(self.controller.update([replace(self.person, confidence=0.2)], 0.3))

    def test_invalid_low_confidence_not_silently_ignored(self) -> None:
        invalid = replace(self.person, confidence=0.1, width=-1.0)
        self.assertEqual(self.controller.update([self.person, invalid], 0.0).reason, "invalid_detection")

    def test_nan_infinity_and_malformed_geometry(self) -> None:
        for field, value in (("confidence", float("nan")), ("center_x", float("inf")),
                             ("width", 0.0), ("height", -1.0), ("confidence", 1.1)):
            with self.subTest(field=field, value=value):
                controller = Controller(Config(640, 480))
                self.assert_zero(controller.update([replace(self.person, **{field: value})], 0.0))

    def test_invalid_after_lock_latches(self) -> None:
        self.acquire()
        self.controller.update([replace(self.person, width=-1.0)], 0.3)
        self.assert_zero(self.controller.update([self.person], 0.4))

    def test_stale_tick_latches(self) -> None:
        self.acquire()
        self.assertEqual(self.controller.tick(1.0).reason, "stale_detection")
        self.assert_zero(self.controller.update([self.person], 1.1))

    def test_new_frame_cannot_hide_stale_gap(self) -> None:
        self.acquire()
        self.assertEqual(self.controller.update([self.person], 1.0).reason, "stale_detection")

    def test_stale_prelock_restarts_acquisition(self) -> None:
        self.controller.update([self.person], 0.0)
        self.controller.update([self.person], 0.1)
        self.assertFalse(self.controller.update([self.person], 1.0).locked)
        self.assertFalse(self.controller.update([self.person], 1.1).locked)
        self.assertTrue(self.controller.update([self.person], 1.2).locked)

    def test_duplicate_time_not_fresh(self) -> None:
        self.controller.update([self.person], 0.1)
        self.assertEqual(self.controller.update([self.person], 0.1).reason, "nonfresh_detection")

    def test_backwards_clock_after_tick_stops(self) -> None:
        self.acquire()
        self.controller.tick(0.4)
        self.assertEqual(self.controller.update([self.person], 0.3).reason, "invalid_time")

    def test_invalid_clock_fails_closed(self) -> None:
        for now in (float("nan"), float("inf"), -1.0):
            self.assert_zero(self.controller.tick(now))

    def test_explicit_stop_is_permanent_before_lock(self) -> None:
        self.controller.stop("operator_stop")
        self.assertEqual(self.acquire().reason, "operator_stop")

    def test_stop_preserves_first_cause(self) -> None:
        self.controller.stop("first")
        self.assertEqual(self.controller.stop("second").reason, "first")

    def test_halted_property_only_tracks_permanent_stops(self) -> None:
        self.controller.update([], 0.0)
        self.assertFalse(self.controller.halted)
        self.controller.stop("operator_stop")
        self.assertTrue(self.controller.halted)

    def test_config_validation(self) -> None:
        for kwargs in ({"image_width": 0}, {"image_height": -1}, {"image_width": True},
                       {"max_age": 0}, {"max_age": float("nan")},
                       {"minimum_confidence": 1.1}, {"target_height_ratio": 1.0}):
            with self.subTest(kwargs=kwargs):
                values = {"image_width": 640, "image_height": 480, **kwargs}
                with self.assertRaises(ValueError):
                    Config(**values)


if __name__ == "__main__":
    unittest.main()

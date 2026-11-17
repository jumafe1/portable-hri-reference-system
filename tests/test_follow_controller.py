"""Pure controller tests: no ROS, network, robot, or motion publisher."""

from dataclasses import replace
import importlib.util
from pathlib import Path
import sys
import struct
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "hri_reference_app/scripts/follow_controller.py"
SPEC = importlib.util.spec_from_file_location("_follow_controller_unit", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
Config = MODULE.Config
Controller = MODULE.Controller
Person = MODULE.Person


class FollowControllerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = Controller(Config(640, 480, target_height_ratio=0.6, max_forward_speed=0.03))
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

    def test_default_target_075_preserves_forward_deadband_and_close_stop(self) -> None:
        for height, reason in ((160.0, 'preview_following'),
                               (180.0, 'target_in_visual_deadband'),
                               (200.0, 'target_too_close_visual')):
            with self.subTest(height=height):
                controller = Controller(Config(320, 240))
                self.assertEqual(controller.config.target_height_ratio, 0.75)
                person = Person(0.9, '', 160.0, 120.0, 80.0, height)
                for now in (0.0, 0.1, 0.2):
                    decision = controller.update([person], now)
                self.assertEqual(decision.reason, reason)
                if height == 160.0:
                    self.assertGreater(decision.vx, 0.0)
                    self.assertLessEqual(decision.vx, 0.1)
                else:
                    self.assert_zero(decision)

    def test_configured_speed_scales_gain_and_saturates_at_its_limit(self) -> None:
        for maximum in (0.01, 0.03, 0.05, 0.10):
            for height, expected in ((240.0, maximum / 3.0), (120.0, maximum)):
                with self.subTest(maximum=maximum, height=height):
                    self.controller = Controller(Config(640, 480, target_height_ratio=0.6, max_forward_speed=maximum))
                    decision = self.acquire(replace(self.person, height=height))
                    self.assertAlmostEqual(decision.vx, expected)
                    self.assertEqual(decision.wz, 0.0)

    def test_higher_speed_preserves_heading_clipping_and_distance_gates(self) -> None:
        for changes, expected_vx, expected_wz in (
            ({"center_x": 384.0}, 0.1 * 23 / 27, -0.04),
            ({"center_x": 540.0}, 0.1 / 3, -0.1),
            ({"height": 288.0}, 0.0, 0.0),
            ({"height": 340.0}, 0.0, 0.0),
            ({"height": 340.0, "center_x": 540.0}, 0.0, -0.1),
            ({"center_x": 50.0}, 0.0, 0.0),
        ):
            with self.subTest(changes=changes):
                self.controller = Controller(Config(640, 480, target_height_ratio=0.6, max_forward_speed=0.1))
                decision = self.acquire(replace(self.person, **changes))
                self.assertAlmostEqual(decision.vx, expected_vx)
                self.assertAlmostEqual(decision.wz, expected_wz)

    def test_higher_speed_still_stops_on_person_loss_and_stale_input(self) -> None:
        for failure in ("person_missing", "stale_detection"):
            with self.subTest(failure=failure):
                self.controller = Controller(Config(640, 480, target_height_ratio=0.6, max_forward_speed=0.1))
                self.assertEqual(self.acquire().vx, 0.1)
                decision = (self.controller.update([], 0.3) if failure == "person_missing"
                            else self.controller.tick(1.0))
                self.assert_zero(decision)
                self.assertEqual(decision.reason, failure)
                self.assertEqual(self.controller.halted, failure == "stale_detection")
                self.assert_zero(self.controller.update([self.person], 1.1))

    def test_speed_configuration_rejects_invalid_values(self) -> None:
        self.assertEqual(Config(640, 480, target_height_ratio=0.6).max_forward_speed, 0.10)
        for maximum in (0, -0.1, 0.10001, float("nan"), float("inf"),
                        -float("inf"), True, False, "0.1", None):
            with self.subTest(maximum=maximum), self.assertRaises(ValueError):
                Config(640, 480, target_height_ratio=0.6, max_forward_speed=maximum)

    def test_right_turn_sign_and_saturation(self) -> None:
        decision = self.acquire(replace(self.person, center_x=540.0))
        self.assertAlmostEqual(decision.vx, 0.03 / 3)
        self.assertEqual(decision.wz, -0.10)

    def test_left_turn_sign_and_saturation(self) -> None:
        decision = self.acquire(replace(self.person, center_x=100.0))
        self.assertAlmostEqual(decision.vx, 0.03 / 3)
        self.assertEqual(decision.wz, 0.10)

    def test_moderate_heading_slows_forward_while_turning(self) -> None:
        decision = self.acquire(replace(self.person, center_x=384.0))
        self.assertAlmostEqual(decision.vx, 0.03 * 23 / 27)
        self.assertAlmostEqual(decision.wz, -0.04)

    def test_forward_tapers_to_one_third_from_24_to_60_pixels_and_scales_with_resolution(self) -> None:
        for scale in (1, 2):
            for sign in (-1, 1):
                for offset, multiplier in ((0, 1), (24, 1), (25, 53 / 54), (42, 2 / 3),
                                           (60, 1 / 3), (61, 1 / 3), (100, 1 / 3), (139, 1 / 3)):
                    with self.subTest(scale=scale, sign=sign, offset=offset):
                        self.controller = Controller(Config(320 * scale, 240 * scale))
                        person = Person(0.9, '', (160 + sign * offset) * scale,
                                        120 * scale, 40 * scale, 120 * scale)
                        decision = self.acquire(person)
                        # A centered half-height target requests 1/12 m/s.
                        self.assertAlmostEqual(decision.vx, multiplier / 12)
                        expected_wz = 0 if offset == 0 else -sign * min(0.1, 0.2 * offset / 160)
                        self.assertAlmostEqual(decision.wz, expected_wz)

    def test_wider_heading_zone_still_obeys_close_and_clipped_box_guards(self) -> None:
        for height, width, expected_wz, reason in (
            (168, 40, -0.06, 'preview_following'),
            (180, 40, -0.06, 'preview_following'),
            (200, 40, -0.06, 'preview_centering_close'),
            (120, 224, 0.0, 'clipped_detection'),
        ):
            with self.subTest(height=height, width=width):
                self.controller = Controller(Config(320, 240))
                decision = self.acquire(Person(0.9, '', 208, 120, width, height))
                self.assertEqual(decision.vx, 0.0)
                self.assertAlmostEqual(decision.wz, expected_wz)
                self.assertEqual(decision.reason, reason)

    def test_wider_heading_zone_stops_on_lost_ambiguous_or_stale_person(self) -> None:
        for failure in ('person_missing', 'ambiguous_people', 'stale_detection'):
            with self.subTest(failure=failure):
                self.controller = Controller(Config(320, 240))
                person = Person(0.9, '', 202, 120, 40, 120)
                self.assertGreater(self.acquire(person).vx, 0.0)
                if failure == 'stale_detection':
                    decision = self.controller.tick(1.0)
                else:
                    people = [] if failure == 'person_missing' else [person, person]
                    decision = self.controller.update(people, 0.3)
                self.assert_zero(decision)
                self.assertEqual(decision.reason, failure)

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

    def test_float32_border_roundtrip_stops_without_latching(self) -> None:
        f32 = lambda value: struct.unpack('f', struct.pack('f', value))[0]
        top, bottom = f32(16.01), f32(240.0)
        center = f32(f32(top + bottom) / 2)
        height = f32(bottom - top)
        self.assertGreater(center + height / 2, 240.0)
        self.controller = Controller(Config(320, 240, target_height_ratio=0.6))
        self.person = Person(0.9, '', 160.0, 120.0, 100.0, 100.0)
        self.acquire()
        decision = self.controller.update([replace(self.person, center_y=center, height=height)], 0.3)
        self.assertEqual(decision.reason, 'clipped_detection')
        self.assert_zero(decision)
        self.assertFalse(self.controller.halted)
        self.assertGreater(self.controller.update([self.person], 0.4).vx, 0)

    def test_roundoff_on_all_edges_is_clipped_but_real_overflow_latches(self) -> None:
        for offset in (-0.00001, 0.00001, 0.01):
            for changes in ({'center_x': 50.0 - offset},
                            {'center_x': 590.0 + offset},
                            {'center_y': 60.0 - offset},
                            {'center_y': 420.0 + offset}):
                with self.subTest(offset=offset, changes=changes):
                    self.controller = Controller(Config(640, 480, target_height_ratio=0.6))
                    self.acquire()
                    decision = self.controller.update([replace(self.person, **changes)], 0.3)
                    self.assert_zero(decision)
                    self.assertEqual(decision.reason, 'invalid_detection' if offset == 0.01 else 'clipped_detection')
                    self.assertEqual(self.controller.halted, offset == 0.01)

    def test_missing_after_lock_requires_three_new_observations(self) -> None:
        self.acquire()
        self.assertEqual(self.controller.update([], 0.3).reason, "person_missing")
        self.assert_zero(self.controller.update([self.person], 0.4))
        self.assertFalse(self.controller.halted)
        self.assert_zero(self.controller.update([self.person], 0.5))
        self.assertGreater(self.controller.update([self.person], 0.6).vx, 0)

    def test_ambiguity_after_lock_requires_three_new_observations(self) -> None:
        self.acquire()
        decision = self.controller.update([self.person, self.person], 0.3)
        self.assertEqual(decision.reason, "ambiguous_people")
        self.assert_zero(self.controller.update([self.person], 0.4))
        self.assertFalse(self.controller.halted)
        self.assert_zero(self.controller.update([self.person], 0.5))
        self.assertGreater(self.controller.update([self.person], 0.6).vx, 0)

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
        self.assertAlmostEqual(decision.vx, 0.03 / 3)
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
        self.controller = Controller(Config(320, 240, target_height_ratio=0.6))
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

    def test_missing_after_geometry_and_id_changes_restarts_acquisition(self) -> None:
        self.acquire()
        self.controller.update([replace(self.person, tracking_id="other", center_x=450.0,
                                        height=240.0)], 0.3)
        self.assertEqual(self.controller.update([], 0.4).reason, "person_missing")
        decision = self.controller.update([self.person], 0.5)
        self.assert_zero(decision)
        self.assertEqual(decision.reason, "acquiring_target")
        self.assertFalse(self.controller.halted)

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
                controller = Controller(Config(640, 480, target_height_ratio=0.6))
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

    def acquire_width(self, person, **config):
        self.controller = Controller(Config(320, 240, target_width_ratio=0.33, **config))
        return self.acquire(person)

    def test_width_proxy_is_disabled_by_default(self) -> None:
        self.assertEqual(Config(320, 240).target_width_ratio, 0.0)

    def test_width_proxy_advances_until_the_width_target_and_then_holds(self) -> None:
        # Box height is identical in every case: only width decides the advance.
        for width, expected_vx, reason in (
            (48.0, 0.10, 'preview_following'),               # far: forward ceiling
            (64.0, 0.10 * (0.33 - 0.20) / 0.15, 'preview_following'),
            (88.0, 0.10 * (0.33 - 0.275) / 0.15, 'preview_following'),
            (100.0, 0.0, 'target_in_visual_deadband'),
            (112.0, 0.0, 'target_in_visual_deadband'),
            (120.0, 0.0, 'target_too_close_visual'),
            (176.0, 0.0, 'target_too_close_visual'),
        ):
            with self.subTest(width=width):
                decision = self.acquire_width(Person(0.9, '', 160.0, 120.0, width, 200.0))
                self.assertAlmostEqual(decision.vx, expected_vx)
                self.assertEqual(decision.wz, 0.0)
                self.assertEqual(decision.reason, reason)

    def test_width_proxy_scales_with_the_configured_speed_ceiling(self) -> None:
        for maximum in (0.03, 0.05, 0.10):
            with self.subTest(maximum=maximum):
                person = Person(0.9, '', 160.0, 120.0, 64.0, 200.0)
                decision = self.acquire_width(person, max_forward_speed=maximum)
                self.assertAlmostEqual(decision.vx, maximum * (0.33 - 0.20) / 0.15)
                self.assertLessEqual(self.acquire_width(replace(person, width=40.0),
                                                        max_forward_speed=maximum).vx, maximum)

    def test_width_proxy_accepts_a_box_cut_at_top_or_bottom(self) -> None:
        for center_y, height in ((120.0, 240.0), (110.0, 220.0), (130.0, 220.0)):
            with self.subTest(center_y=center_y, height=height):
                person = Person(0.9, '', 160.0, center_y, 64.0, height)
                decision = self.acquire_width(person)
                self.assertGreater(decision.vx, 0.0)
                self.assertEqual(decision.reason, 'preview_following')
                # The height proxy keeps refusing the same box.
                self.controller = Controller(Config(320, 240))
                decision = self.acquire(person)
                self.assert_zero(decision)
                self.assertEqual(decision.reason, 'clipped_detection')

    def test_width_proxy_still_stops_both_axes_on_a_side_clipped_box(self) -> None:
        for center_x in (32.0, 288.0):
            with self.subTest(center_x=center_x):
                decision = self.acquire_width(Person(0.9, '', center_x, 120.0, 64.0, 240.0))
                self.assert_zero(decision)
                self.assertEqual(decision.reason, 'clipped_detection')

    def test_width_proxy_too_close_allows_centering_only(self) -> None:
        for center_x, expected_wz in ((100.0, 0.075), (220.0, -0.075)):
            with self.subTest(center_x=center_x):
                decision = self.acquire_width(Person(0.9, '', center_x, 120.0, 150.0, 240.0))
                self.assertEqual(decision.vx, 0.0)
                self.assertAlmostEqual(decision.wz, expected_wz)
                self.assertEqual(decision.reason, 'preview_centering_close')

    def test_width_proxy_keeps_the_heading_taper(self) -> None:
        for offset, multiplier in ((0, 1), (24, 1), (42, 2 / 3), (60, 1 / 3), (100, 1 / 3)):
            with self.subTest(offset=offset):
                decision = self.acquire_width(Person(0.9, '', 160.0 + offset, 120.0, 64.0, 240.0))
                self.assertAlmostEqual(decision.vx, multiplier * 0.10 * (0.33 - 0.20) / 0.15)

    def test_width_proxy_stops_on_lost_ambiguous_or_stale_person(self) -> None:
        for failure in ('person_missing', 'ambiguous_people', 'stale_detection'):
            with self.subTest(failure=failure):
                person = Person(0.9, '', 160.0, 120.0, 64.0, 240.0)
                self.assertGreater(self.acquire_width(person).vx, 0.0)
                if failure == 'stale_detection':
                    decision = self.controller.tick(1.0)
                else:
                    people = [] if failure == 'person_missing' else [person, person]
                    decision = self.controller.update(people, 0.3)
                self.assert_zero(decision)
                self.assertEqual(decision.reason, failure)
                if failure != 'stale_detection':
                    # Three new single-person observations are required again.
                    self.assert_zero(self.controller.update([person], 0.4))
                    self.assert_zero(self.controller.update([person], 0.5))
                    self.assertGreater(self.controller.update([person], 0.6).vx, 0.0)

    def test_width_proxy_ignores_the_angular_height_override(self) -> None:
        self.controller = Controller(Config(320, 240, target_width_ratio=0.33))
        person = Person(0.9, '', 160.0, 120.0, 64.0, 200.0)
        for now in (0.0, 0.1, 0.2):
            decision = self.controller.update([person], now, height_ratio_override=5.0)
        self.assertAlmostEqual(decision.vx, 0.10 * (0.33 - 0.20) / 0.15)

    def test_width_configuration_rejects_invalid_values(self) -> None:
        for value in (-0.1, 0.05, 0.95, 1.0, float("nan"), float("inf"), True, "0.33", None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                Config(320, 240, target_width_ratio=value)

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

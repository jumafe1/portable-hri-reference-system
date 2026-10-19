"""ROS-independent, preview-only visual following controller.

Outputs are suggestions, not authorized robot commands. Box size is a visual
proxy, NOT metric distance or collision clearance. This controlled experiment
assumes exactly one participant; the sole eligible detection is the target.
No identity association is attempted, even when tracking IDs are supplied.
After acquiring a target, missing/ambiguous/invalid input requires a new Controller.
The turn sign assumes a forward-facing camera aligned with the robot base;
this stage does not compensate for head motion or camera extrinsics.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Config:
    image_width: int
    image_height: int
    minimum_confidence: float = 0.65
    target_height_ratio: float = 0.6
    max_age: float = 0.75

    def __post_init__(self) -> None:
        for value in (self.image_width, self.image_height):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError("Image dimensions must be positive integers")
        if not _finite(self.minimum_confidence) or not 0 <= self.minimum_confidence <= 1:
            raise ValueError("minimum_confidence must be within [0, 1]")
        if not _finite(self.target_height_ratio) or not 0.05 < self.target_height_ratio < 0.95:
            raise ValueError("target_height_ratio must be within (0.05, 0.95)")
        if not _finite(self.max_age) or self.max_age <= 0:
            raise ValueError("max_age must be positive and finite")


@dataclass(frozen=True)
class Person:
    confidence: float
    tracking_id: str
    center_x: float
    center_y: float
    width: float
    height: float


@dataclass(frozen=True)
class Decision:
    vx: float
    wz: float
    reason: str
    locked: bool


def _finite(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class Controller:
    """Require three consecutive observations; fail closed on target loss.

    ``now`` must be monotonic receipt time. The caller must independently reject
    stale/repeated image stamps: receipt timestamps cannot establish sensor age.
    ``locked`` records acquisition, including latched-stop decisions. ``stop``
    is permanent. Re-arming is explicit: construct a new controller.
    """

    def __init__(self, config: Config) -> None:
        self.config = config
        self._count = 0
        self._locked = False
        self._latched: str | None = None
        self._last_update: float | None = None
        self._last_clock: float | None = None
        self._decision = Decision(0.0, 0.0, "awaiting_detection", False)

    @property
    def halted(self) -> bool:
        """Whether an explicit or post-acquisition stop has been latched."""
        return self._latched is not None

    def stop(self, reason: str) -> Decision:
        """Permanently stop, retaining the original latched reason."""
        if self._latched is None:
            self._latched = reason or "stopped"
        return self._zero(self._latched)

    def _zero(self, reason: str) -> Decision:
        self._decision = Decision(0.0, 0.0, reason, self._locked)
        return self._decision

    def _reject(self, reason: str) -> Decision:
        if self._locked:
            return self.stop(reason)
        self._count = 0
        return self._zero(reason)

    def _clock_valid(self, now: float) -> bool:
        if not _finite(now) or now < 0:
            return False
        if self._last_clock is not None and now < self._last_clock:
            return False
        self._last_clock = now
        return True

    def tick(self, now: float) -> Decision:
        """Run even without incoming frames, so input loss produces zero."""
        if self._latched is not None:
            return self._zero(self._latched)
        if not self._clock_valid(now):
            return self._reject("invalid_time")
        if self._last_update is not None and now - self._last_update > self.config.max_age:
            return self._reject("stale_detection")
        return self._decision

    def _valid_person(self, person: Person) -> bool:
        if not isinstance(person, Person) or not isinstance(person.tracking_id, str):
            return False
        values = (person.confidence, person.center_x, person.center_y, person.width, person.height)
        if not all(_finite(value) for value in values):
            return False
        if not 0 <= person.confidence <= 1 or person.width <= 0 or person.height <= 0:
            return False
        return (
            person.center_x - person.width / 2 >= 0
            and person.center_y - person.height / 2 >= 0
            and person.center_x + person.width / 2 <= self.config.image_width
            and person.center_y + person.height / 2 <= self.config.image_height
        )

    def update(self, people: list[Person], now: float) -> Decision:
        """Accept one fresh image's people; do not concatenate robot streams."""
        if self._latched is not None:
            return self._zero(self._latched)
        if not self._clock_valid(now):
            return self._reject("invalid_time")
        if self._last_update is not None:
            if now <= self._last_update:
                return self._reject("nonfresh_detection")
            # Check before overwriting receipt time; a new frame cannot erase a gap.
            if now - self._last_update > self.config.max_age:
                stale = self._reject("stale_detection")
                if self._latched is not None:
                    return stale
        self._last_update = now
        if not isinstance(people, list) or any(not self._valid_person(p) for p in people):
            return self._reject("invalid_detection")
        eligible = [p for p in people if p.confidence >= self.config.minimum_confidence]
        if not eligible:
            return self._reject("person_missing")
        if len(eligible) != 1:
            return self._reject("ambiguous_people")
        person = eligible[0]
        self._count += 1
        if self._count < 3:
            return self._zero("acquiring_target")
        self._locked = True
        height_ratio = person.height / self.config.image_height
        clipped = (
            person.center_x - person.width / 2 <= 0
            or person.center_y - person.height / 2 <= 0
            or person.center_x + person.width / 2 >= self.config.image_width
            or person.center_y + person.height / 2 >= self.config.image_height
        )
        if clipped:
            return self._zero("clipped_detection")
        error = (person.center_x - self.config.image_width / 2) / (self.config.image_width / 2)
        wz = 0.0 if abs(error) <= 0.08 else max(-0.10, min(0.10, -0.20 * error))
        # Preview only: apparent size gates advance, not centering. This is
        # not clearance to rotate physically; clipped boxes remain zero above.
        if height_ratio > self.config.target_height_ratio + 0.05:
            reason = "preview_centering_close" if wz != 0 else "target_too_close_visual"
            self._decision = Decision(0.0, wz, reason, True)
            return self._decision
        vx = 0.0
        if abs(error) <= 0.15 and height_ratio < self.config.target_height_ratio - 0.05:
            vx = min(0.03, 0.10 * (self.config.target_height_ratio - height_ratio))
        reason = "preview_following" if vx != 0 or wz != 0 else "target_in_visual_deadband"
        self._decision = Decision(vx, wz, reason, True)
        return self._decision

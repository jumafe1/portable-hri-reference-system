"""Calibrated framing for a pitch-only, controlled visual-follow experiment.

Vertical angular span is invariant to ideal camera pitch (rotation about the
optical x axis). It is NOT metric range. Neck translation, articulation and
changing detector boxes remain approximation errors; clipped boxes are unusable.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Calibration:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple = ()

    def __post_init__(self):
        values = (self.fx, self.fy, self.cx, self.cy, *self.distortion)
        if not all(math.isfinite(v) for v in values) or self.fx <= 0 or self.fy <= 0:
            raise ValueError("invalid_camera_calibration")
        if self.width <= 0 or self.height <= 0 or not (
            0 <= self.cx <= self.width and 0 <= self.cy <= self.height
        ) or len(self.distortion) not in (0, 5):
            raise ValueError("invalid_camera_calibration")

    def ray(self, u, v):
        """Invert Brown/plumb-bob distortion, rejecting nonconvergent inversions."""
        xd, yd = (u - self.cx) / self.fx, (v - self.cy) / self.fy
        x, y = xd, yd
        k1, k2, p1, p2, k3 = self.distortion or (0.0,) * 5
        for _ in range(30):
            r2 = x * x + y * y
            radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
            if not math.isfinite(radial) or radial <= 1e-8:
                raise ValueError("invalid_camera_distortion")
            dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
            dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
            nx, ny = (xd - dx) / radial, (yd - dy) / radial
            if not math.isfinite(nx + ny):
                raise ValueError("invalid_camera_distortion")
            if max(abs(nx - x), abs(ny - y)) < 1e-9:
                return nx, ny
            x, y = nx, ny
        raise ValueError("camera_distortion_did_not_converge")

    def equivalent_height_ratio(self, person):
        """Return angular size expressed as a centered pinhole box-height ratio."""
        top, bottom = person.center_y - person.height / 2, person.center_y + person.height / 2
        if not (0 < top < bottom < self.height
                and 0 < person.center_x - person.width / 2
                and person.center_x + person.width / 2 < self.width):
            raise ValueError("clipped_detection")
        _, yt = self.ray(person.center_x, top)
        _, yb = self.ray(person.center_x, bottom)
        span = math.atan(yb) - math.atan(yt)
        if not 0 < span < math.pi:
            raise ValueError("invalid_camera_angular_span")
        return 2 * self.fy * math.tan(span / 2) / self.height


def calibration_from_info(info, width, height, frame):
    if info.header.frame_id != frame or (info.width, info.height) != (width, height):
        raise ValueError("camera_calibration_frame_or_size_mismatch")
    if info.binning_x not in (0, 1) or info.binning_y not in (0, 1) or any(
        (info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height)
    ):
        raise ValueError("camera_calibration_roi_or_binning_unsupported")
    k = tuple(info.k)
    if len(k) != 9 or not all(math.isfinite(v) for v in k) or any(
        abs(k[i]) > 1e-9 for i in (1, 3, 6, 7)
    ) or abs(k[8] - 1.0) > 1e-9:
        raise ValueError("invalid_camera_intrinsics")
    distortion = tuple(info.d)
    if not ((info.distortion_model == "plumb_bob" and len(distortion) == 5)
            or (info.distortion_model == "" and not distortion)):
        raise ValueError("unsupported_camera_distortion_model")
    if not all(math.isfinite(value) for value in distortion):
        raise ValueError("invalid_camera_calibration")
    if not any(distortion):
        distortion = ()
    return Calibration(width, height, k[0], k[4], k[2], k[5], distortion)


def elevation_step(person, image_height, elevation):
    """Positive elevation looks up; never infer direction from occupancy alone."""
    top = person.center_y - person.height / 2
    bottom = person.center_y + person.height / 2
    if top <= 1 and bottom >= image_height - 1:
        return elevation  # Both ends clipped: moving the camera cannot fit the person.
    if top <= 1:
        delta = 0.03
    elif bottom >= image_height - 1:
        delta = -0.03
    else:
        error = person.center_y / image_height - 0.5
        delta = -0.03 if error > 0.12 else 0.03 if error < -0.12 else 0.0
    return max(-0.25, min(0.25, elevation + delta))

"""Depth-image statistics and the obstacle stop gate.

Pure NumPy/Python on purpose. The depth camera runs in Gazebo and the gate runs
in a ROS node, but the two decisions that matter are testable without either:

* "what is the nearest surface in the part of the image I care about?"
* "should the vehicle hold position, given that distance and its own past?"

The image is a float32 array of **z-depth** in metres, as Gazebo publishes it:
the distance along the camera's optical axis, not the Euclidean range. With a
camera tilted down, the two differ by up to 1/cos(tilt) at the frame centre; the
thresholds in ``config/depth.yaml`` are therefore z-depth thresholds and the
docs say so.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# A pixel is "no return": Gazebo writes 0 for rays that hit nothing, and the
# simulator also emits inf/nan at clipping boundaries. None of those is an
# obstacle, and treating 0 as "0 m away" would stop on the first empty sky pixel.
MIN_FINITE_DISTANCE_M = 0.0


def depth_from_bytes(data: bytes | bytearray | memoryview, width: int, height: int) -> np.ndarray:
    """Decode a Gazebo ``R_FLOAT32`` depth image into a ``height x width`` array.

    The wire format is 4 little-endian bytes per pixel. The row stride in the
    Gazebo message is ``width * 4`` for this format; a mismatch is a broken
    frame, not something to reshape optimistically.
    """
    width, height = int(width), int(height)
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid depth image size {width}x{height}")

    expected = width * height * 4
    if len(data) != expected:
        raise ValueError(
            f"depth image is {len(data)} bytes, expected {expected} for {width}x{height} float32"
        )

    array = np.frombuffer(data, dtype="<f4")
    return array.reshape(height, width)


@dataclass(frozen=True)
class Roi:
    """Pixel bounds of the region that is allowed to command a stop."""

    left: int
    top: int
    width: int
    height: int

    @property
    def pixels(self) -> int:
        return self.width * self.height


def roi_bounds(
    image_width: int,
    image_height: int,
    width_fraction: float = 0.6,
    height_fraction: float = 0.5,
    center_x: float = 0.5,
    center_y: float = 0.5,
) -> Roi:
    """Centre a fractional window in the image, clamped to the frame.

    Fractions are validated rather than clamped silently: a config typo such as
    ``1.5`` should fail at startup, not quietly select the whole frame and make
    the "centre region" claim false.
    """
    width, height = int(image_width), int(image_height)
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid image size {width}x{height}")
    for name, value in (
        ("width_fraction", width_fraction),
        ("height_fraction", height_fraction),
    ):
        if not 0.0 < float(value) <= 1.0:
            raise ValueError(f"{name} must be in (0, 1], got {value!r}")
    for name, value in (("center_x", center_x), ("center_y", center_y)):
        if not 0.0 <= float(value) <= 1.0:
            raise ValueError(f"{name} must be in [0, 1], got {value!r}")

    window_w = max(1, min(width, int(round(width * float(width_fraction)))))
    window_h = max(1, min(height, int(round(height * float(height_fraction)))))
    left = int(round(width * float(center_x) - window_w / 2.0))
    top = int(round(height * float(center_y) - window_h / 2.0))
    left = min(max(left, 0), width - window_w)
    top = min(max(top, 0), height - window_h)
    return Roi(left=left, top=top, width=window_w, height=window_h)


@dataclass(frozen=True)
class DepthStats:
    """What one depth frame says about the ROI."""

    nearest_m: float | None
    valid_pixels: int
    roi_pixels: int


def analyse_depth(
    depth: np.ndarray,
    *,
    width_fraction: float = 0.6,
    height_fraction: float = 0.5,
    center_x: float = 0.5,
    center_y: float = 0.5,
    min_distance_m: float = MIN_FINITE_DISTANCE_M,
    max_distance_m: float = math.inf,
) -> DepthStats:
    """Nearest valid surface in the ROI, plus how much of the ROI was valid.

    ``None`` means "no usable return" (all sky, all clipped, an empty frame).
    The gate deliberately treats that as *no new evidence*, not as "clear".
    """
    if depth.ndim != 2:
        raise ValueError(f"depth image must be 2-D, got shape {depth.shape}")
    if not math.isfinite(float(min_distance_m)):
        raise ValueError("min_distance_m must be finite")
    if max_distance_m <= min_distance_m:
        raise ValueError("max_distance_m must be greater than min_distance_m")

    roi = roi_bounds(
        depth.shape[1],
        depth.shape[0],
        width_fraction=width_fraction,
        height_fraction=height_fraction,
        center_x=center_x,
        center_y=center_y,
    )
    region = depth[roi.top : roi.top + roi.height, roi.left : roi.left + roi.width]

    finite = np.isfinite(region)
    valid = finite & (region > float(min_distance_m)) & (region <= float(max_distance_m))
    count = int(np.count_nonzero(valid))
    if count == 0:
        return DepthStats(nearest_m=None, valid_pixels=0, roi_pixels=roi.pixels)

    # Mask first, then reduce: np.min on the masked region would return 0 for
    # every invalid pixel and make a wall look like it is touching the lens.
    masked = np.where(valid, region, np.inf)
    return DepthStats(
        nearest_m=float(np.min(masked)),
        valid_pixels=count,
        roi_pixels=roi.pixels,
    )


@dataclass
class StopGate:
    """Stop below ``stop_distance_m``, release only above ``clear_distance_m``.

    The gap between the two thresholds is the hysteresis: one frame of noise at
    the boundary must not toggle the command at 50 Hz. The gate latches, so a
    vehicle that has stopped does not resume merely because the measured
    distance ticks a centimetre past the stop threshold while it brakes.
    """

    stop_distance_m: float = 2.0
    clear_distance_m: float = 2.6
    _stopped: bool = field(default=False, init=False)
    transitions: int = field(default=0, init=False)
    last_distance_m: float | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        stop = float(self.stop_distance_m)
        clear = float(self.clear_distance_m)
        if not math.isfinite(stop) or stop <= 0.0:
            raise ValueError(f"stop_distance_m must be positive and finite, got {stop!r}")
        if not math.isfinite(clear) or clear <= stop:
            raise ValueError(
                f"clear_distance_m ({clear!r}) must be greater than stop_distance_m ({stop!r})"
            )
        self.stop_distance_m = stop
        self.clear_distance_m = clear

    @property
    def stopped(self) -> bool:
        return self._stopped

    def reset(self) -> None:
        self._stopped = False
        self.transitions = 0
        self.last_distance_m = None

    def update(self, distance_m: float | None) -> bool:
        """Feed the newest nearest distance, get the stop decision.

        ``None``, NaN and infinity mean "no usable return this frame": the gate
        holds its previous decision. Holding a stop is the safe direction; this
        is not a place to guess that the obstacle went away.
        """
        if distance_m is None:
            return self._stopped

        distance = float(distance_m)
        if not math.isfinite(distance):
            return self._stopped
        self.last_distance_m = distance

        if self._stopped:
            if distance >= self.clear_distance_m:
                self._stopped = False
                self.transitions += 1
        elif distance <= self.stop_distance_m:
            self._stopped = True
            self.transitions += 1
        return self._stopped


@dataclass
class StopWatchdog:
    """Turn a stream of stop booleans into one decision, with a freshness rule.

    Before the first message the watchdog is inactive, so a mission that does
    not run the avoidance node keeps working unchanged. Once messages have been
    seen, a gap longer than ``timeout_s`` is treated as "stop": a dead depth
    node must not be indistinguishable from a clear sky.
    """

    timeout_s: float = 1.0
    _stop: bool = field(default=False, init=False)
    _last_s: float | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        timeout = float(self.timeout_s)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError(f"timeout_s must be positive and finite, got {timeout!r}")
        self.timeout_s = timeout

    @property
    def seen(self) -> bool:
        return self._last_s is not None

    def reset(self) -> None:
        self._stop = False
        self._last_s = None

    def update(self, stop: bool, now_s: float) -> None:
        now = float(now_s)
        if not math.isfinite(now):
            raise ValueError(f"now_s must be finite, got {now!r}")
        self._stop = bool(stop)
        self._last_s = now

    def active(self, now_s: float) -> bool:
        """True when the vehicle must hold position.

        No message has ever arrived -> False (avoidance is optional).
        A fresh message -> its value.
        A stale message -> True (fail safe).
        """
        if self._last_s is None:
            return False
        now = float(now_s)
        if not math.isfinite(now):
            raise ValueError(f"now_s must be finite, got {now!r}")
        if now - self._last_s > self.timeout_s:
            return True
        return self._stop

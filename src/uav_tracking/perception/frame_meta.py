"""Pose metadata that travels with a recorded depth frame.

The recorder used to save only the depth array, so the gate had to guess which
CSV row belonged to the frame. That guess is why the avoidance frame failed the
second-pair check. Every new frame now gets a JSON sidecar written at the same
instant as the array.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class FrameMeta:
    source: str
    t_s: float
    east_m: float
    north_m: float
    altitude_m: float
    yaw_deg: float
    nearest_m: float | None
    stop_event: int
    width: int
    height: int

    def __post_init__(self) -> None:
        for name in ("t_s", "east_m", "north_m", "altitude_m", "yaw_deg"):
            value = getattr(self, name)
            if not math.isfinite(float(value)):
                raise ValueError(f"{name} must be finite")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("width and height must be positive")
        if self.stop_event < 0:
            raise ValueError("stop_event must be non-negative")
        if self.nearest_m is not None and not math.isfinite(float(self.nearest_m)):
            raise ValueError("nearest_m must be finite when set")


def sidecar_path(frame_path: str | Path) -> Path:
    path = Path(frame_path)
    return path.with_suffix(path.suffix + ".json")


def write_frame_meta(frame_path: str | Path, meta: FrameMeta) -> Path:
    path = sidecar_path(frame_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(asdict(meta), indent=2) + "\n", encoding="utf-8")
    return path


def read_frame_meta(path: str | Path) -> FrameMeta:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return FrameMeta(**data)

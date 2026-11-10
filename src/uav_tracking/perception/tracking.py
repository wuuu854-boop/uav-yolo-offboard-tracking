"""Cross-frame target identity.

Without this, every frame is an independent set of boxes: the mission cannot say
"this is the same object as before", cannot count how many distinct targets it
has seen, and a momentary detection gap resets whatever the selector was
following.

The tracker is deliberately simple — greedy association with a short memory,
plus a constant-velocity guess. The camera is nadir and the frame rate is 5 Hz,
so a full Kalman filter would add parameters without adding accuracy. What the
guess buys is the moving target: at 5 Hz a fast target shifts by more than a box
width between frames, and on the first pair there is no velocity to predict with,
so association also has a nearest-centre pass to bootstrap it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class Track:
    """One target followed over time."""

    track_id: int
    bbox_cx: float
    bbox_cy: float
    bbox_w: float
    bbox_h: float
    hits: int = 1
    last_seen_s: float = 0.0
    # Pixels per second, smoothed over hits. Zero for a static target, which is
    # why adding this does not change the static-target behaviour at all.
    vx: float = 0.0
    vy: float = 0.0

    def as_detection(self) -> dict:
        return {
            "track_id": self.track_id,
            "bbox_cx": self.bbox_cx,
            "bbox_cy": self.bbox_cy,
            "bbox_w": self.bbox_w,
            "bbox_h": self.bbox_h,
        }

    def predicted(self, dt: float, max_extrapolation_s: float = 1.0) -> dict:
        """Where this box is expected to be ``dt`` seconds after the last hit.

        The extrapolation is capped: after a long gap a constant-velocity guess
        is worse than the last known position, and an unbounded prediction can
        throw the box across the image and steal another track's detection.
        """
        horizon = max(0.0, min(dt, max_extrapolation_s))
        return {
            "track_id": self.track_id,
            "bbox_cx": self.bbox_cx + self.vx * horizon,
            "bbox_cy": self.bbox_cy + self.vy * horizon,
            "bbox_w": self.bbox_w,
            "bbox_h": self.bbox_h,
        }


def iou(first: dict, second: dict) -> float:
    """Intersection over union for boxes given as centre, width and height."""
    left = max(first["bbox_cx"] - first["bbox_w"] / 2, second["bbox_cx"] - second["bbox_w"] / 2)
    right = min(first["bbox_cx"] + first["bbox_w"] / 2, second["bbox_cx"] + second["bbox_w"] / 2)
    top = max(first["bbox_cy"] - first["bbox_h"] / 2, second["bbox_cy"] - second["bbox_h"] / 2)
    bottom = min(first["bbox_cy"] + first["bbox_h"] / 2, second["bbox_cy"] + second["bbox_h"] / 2)

    width = max(0.0, right - left)
    height = max(0.0, bottom - top)
    intersection = width * height
    if intersection <= 0.0:
        return 0.0

    first_area = first["bbox_w"] * first["bbox_h"]
    second_area = second["bbox_w"] * second["bbox_h"]
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


@dataclass
class IouTracker:
    """Greedy association with a short memory and a constant-velocity guess.

    ``iou_threshold`` is low on purpose: at 5 Hz a moving target can shift by
    half a box between frames, and an ID switch is worse than a slightly loose
    match for counting distinct targets.

    ``max_age_s`` is how long a track survives without a detection. It should be
    longer than one missed frame and shorter than the mission's target-loss
    timeout, otherwise a returning target silently becomes a new one.

    ``velocity_alpha`` is the smoothing on the estimated velocity. Static targets
    keep a velocity of zero, so the motion model is inert for them; moving
    targets get their predicted box used as a second association candidate, which
    is what keeps the ID through a frame where the box moved too far for IoU.
    """

    iou_threshold: float = 0.2
    max_age_s: float = 1.5
    velocity_alpha: float = 0.6
    max_extrapolation_s: float = 1.0
    center_gate_factor: float = 1.5
    tracks: list[Track] = field(default_factory=list)
    _next_id: int = 1

    def __post_init__(self) -> None:
        if not 0.0 <= self.iou_threshold <= 1.0:
            raise ValueError("iou_threshold must be in [0, 1]")
        if self.max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive")
        if not 0.0 < self.velocity_alpha <= 1.0:
            raise ValueError("velocity_alpha must be in (0, 1]")
        if self.center_gate_factor <= 0.0:
            raise ValueError("center_gate_factor must be positive")

    def _absorb(self, track: Track, detection: dict, now_s: float) -> None:
        """Take the detection into the track and update the velocity estimate."""
        elapsed = now_s - track.last_seen_s
        if elapsed > 0.0:
            alpha = self.velocity_alpha
            track.vx = (
                alpha * (detection["bbox_cx"] - track.bbox_cx) / elapsed + (1.0 - alpha) * track.vx
            )
            track.vy = (
                alpha * (detection["bbox_cy"] - track.bbox_cy) / elapsed + (1.0 - alpha) * track.vy
            )
        track.bbox_cx = detection["bbox_cx"]
        track.bbox_cy = detection["bbox_cy"]
        track.bbox_w = detection["bbox_w"]
        track.bbox_h = detection["bbox_h"]
        track.hits += 1
        track.last_seen_s = now_s

    def update(self, detections: list[dict], now_s: float) -> list[dict]:
        """Annotate detections with ``track_id`` and age out old tracks.

        Returns the detections in their input order, each with a ``track_id``
        key. A track that is not matched this frame keeps its box and stays
        available for ``max_age_s``, which is what makes a single missed
        detection harmless.

        Association runs twice: overlap first (the box as last seen and where it
        is predicted to be), then nearest predicted centre within a gate. The
        second pass is what lets a target bootstrap its velocity — on the first
        pair there is no velocity to predict with, and a fast target would
        otherwise be a new track on every frame.
        """
        self.tracks = [
            track for track in self.tracks if now_s - track.last_seen_s <= self.max_age_s
        ]

        unmatched = list(range(len(detections)))
        matches: dict[int, Track] = {}
        used_tracks: set[int] = set()

        # Pass 1: greedy overlap, so two nearby targets do not steal each
        # other's ID when one is clearly the better match. A pair scores on the
        # box where the track was last seen and on where it is expected to be
        # now; for a static target the two are identical.
        pairs = sorted(
            (
                (
                    max(
                        iou(track.as_detection(), detection),
                        iou(
                            track.predicted(now_s - track.last_seen_s, self.max_extrapolation_s),
                            detection,
                        ),
                    ),
                    track_index,
                    detection_index,
                )
                for track_index, track in enumerate(self.tracks)
                for detection_index, detection in enumerate(detections)
            ),
            reverse=True,
        )
        for score, track_index, detection_index in pairs:
            if score < self.iou_threshold:
                break
            if track_index in used_tracks or detection_index not in unmatched:
                continue
            self._absorb(self.tracks[track_index], detections[detection_index], now_s)
            matches[detection_index] = self.tracks[track_index]
            used_tracks.add(track_index)
            unmatched.remove(detection_index)

        # Pass 2: nearest predicted centre, gated by the box size. This is the
        # bootstrap path; it never overrides an overlap match, because those
        # detections are already consumed.
        candidates = []
        for track_index, track in enumerate(self.tracks):
            if track_index in used_tracks:
                continue
            predicted = track.predicted(now_s - track.last_seen_s, self.max_extrapolation_s)
            gate = self.center_gate_factor * max(track.bbox_w, track.bbox_h, 1.0)
            for detection_index in unmatched:
                detection = detections[detection_index]
                distance = math.hypot(
                    detection["bbox_cx"] - predicted["bbox_cx"],
                    detection["bbox_cy"] - predicted["bbox_cy"],
                )
                if distance <= gate:
                    candidates.append((distance, track_index, detection_index))

        for _distance, track_index, detection_index in sorted(candidates):
            if track_index in used_tracks or detection_index not in unmatched:
                continue
            self._absorb(self.tracks[track_index], detections[detection_index], now_s)
            matches[detection_index] = self.tracks[track_index]
            used_tracks.add(track_index)
            unmatched.remove(detection_index)

        for detection_index in unmatched:
            detection = detections[detection_index]
            track = Track(
                track_id=self._next_id,
                bbox_cx=detection["bbox_cx"],
                bbox_cy=detection["bbox_cy"],
                bbox_w=detection["bbox_w"],
                bbox_h=detection["bbox_h"],
                last_seen_s=now_s,
            )
            self._next_id += 1
            self.tracks.append(track)
            matches[detection_index] = track

        annotated: list[dict] = []
        for detection_index, detection in enumerate(detections):
            enriched = dict(detection)
            enriched["track_id"] = matches[detection_index].track_id
            annotated.append(enriched)
        return annotated

    @property
    def active(self) -> list[Track]:
        return list(self.tracks)

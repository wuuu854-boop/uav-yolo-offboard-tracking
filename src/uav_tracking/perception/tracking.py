"""Cross-frame target identity.

Without this, every frame is an independent set of boxes: the mission cannot say
"this is the same object as before", cannot count how many distinct targets it
has seen, and a momentary detection gap resets whatever the selector was
following.

The tracker is deliberately simple — greedy IoU association, no Kalman filter.
The camera is nadir, the targets are static, and the frame rate is 5 Hz, so a
motion model would add parameters without adding accuracy. What it does need is
to tolerate a few missed frames, which ``max_age_s`` provides.
"""

from __future__ import annotations

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

    def as_detection(self) -> dict:
        return {
            "track_id": self.track_id,
            "bbox_cx": self.bbox_cx,
            "bbox_cy": self.bbox_cy,
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
    """Greedy association with a short memory.

    ``iou_threshold`` is low on purpose: at 5 Hz a moving target can shift by
    half a box between frames, and an ID switch is worse than a slightly loose
    match for counting distinct targets.

    ``max_age_s`` is how long a track survives without a detection. It should be
    longer than one missed frame and shorter than the mission's target-loss
    timeout, otherwise a returning target silently becomes a new one.
    """

    iou_threshold: float = 0.2
    max_age_s: float = 1.5
    tracks: list[Track] = field(default_factory=list)
    _next_id: int = 1

    def __post_init__(self) -> None:
        if not 0.0 <= self.iou_threshold <= 1.0:
            raise ValueError("iou_threshold must be in [0, 1]")
        if self.max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive")

    def update(self, detections: list[dict], now_s: float) -> list[dict]:
        """Annotate detections with ``track_id`` and age out old tracks.

        Returns the detections in their input order, each with a ``track_id``
        key. A track that is not matched this frame keeps its box and stays
        available for ``max_age_s``, which is what makes a single missed
        detection harmless.
        """
        self.tracks = [
            track for track in self.tracks if now_s - track.last_seen_s <= self.max_age_s
        ]

        unmatched = list(range(len(detections)))
        matches: dict[int, Track] = {}

        # Greedy: take the best overlapping pair first, so two nearby targets do
        # not steal each other's ID when one is clearly the better match.
        pairs = sorted(
            (
                (iou(track.as_detection(), detection), track_index, detection_index)
                for track_index, track in enumerate(self.tracks)
                for detection_index, detection in enumerate(detections)
            ),
            reverse=True,
        )
        used_tracks: set[int] = set()
        for score, track_index, detection_index in pairs:
            if score < self.iou_threshold:
                break
            if track_index in used_tracks or detection_index not in unmatched:
                continue
            track = self.tracks[track_index]
            track.bbox_cx = detections[detection_index]["bbox_cx"]
            track.bbox_cy = detections[detection_index]["bbox_cy"]
            track.bbox_w = detections[detection_index]["bbox_w"]
            track.bbox_h = detections[detection_index]["bbox_h"]
            track.hits += 1
            track.last_seen_s = now_s
            matches[detection_index] = track
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

"""Rate limiting with a true speed bound.

The previous implementation clipped each axis independently, which allowed a
diagonal setpoint to move at sqrt(2) times the configured speed while the README
claimed 1 m/s. Clipping the vector instead keeps the promise.
"""

from __future__ import annotations

import math
from collections.abc import Sequence


def limited_step(
    current: Sequence[float], target: Sequence[float], max_step: float
) -> tuple[float, ...]:
    """Move from ``current`` towards ``target`` by at most ``max_step``.

    Returns ``target`` unchanged when it is already within reach.
    """
    if max_step <= 0.0:
        raise ValueError("max_step must be positive")
    if len(current) != len(target):
        raise ValueError(f"dimension mismatch: {len(current)} vs {len(target)}")

    deltas = [float(t) - float(c) for c, t in zip(current, target, strict=True)]
    distance = math.sqrt(sum(delta * delta for delta in deltas))
    if distance <= max_step or distance == 0.0:
        return tuple(float(value) for value in target)

    ratio = max_step / distance
    return tuple(float(c) + delta * ratio for c, delta in zip(current, deltas, strict=True))

"""Joint angle <-> ring index.

Two mappings, both taken from the SO-2 repo (``control/mapping.py``, ``math/circular.py``):

* circular: the angle is the ring angle, ``index = angle / 2π · N`` (the paper's setting);
* bounded: the joint's limits lie on an arc that covers ``arc_fraction`` of the ring and is
  centred on index ``N / 2``; the rest of the ring, around the seam, is out of range (the
  bounded ring of architecture v3 / v4).

An encoder stimulates the nearest index, so a ring holds an angle to within half an index.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .kinematics import Joint

ARC_FRACTION = 0.70
WALL_MARGIN = 15


@dataclass(frozen=True)
class RingMap:
    size: int
    lower: float
    upper: float
    circular: bool
    arc_fraction: float = ARC_FRACTION

    @classmethod
    def for_joint(cls, joint: Joint, size: int, arc_fraction: float = ARC_FRACTION) -> "RingMap":
        return cls(int(size), float(joint.lower), float(joint.upper), bool(joint.circular), float(arc_fraction))

    @property
    def span(self) -> float:
        """Ring indices the joint range covers."""

        return float(self.size) if self.circular else float(round(self.arc_fraction * self.size))

    @property
    def first(self) -> float:
        """Ring index of the lower limit (0 for the circular mapping, where index 0 is angle 0)."""

        return 0.0 if self.circular else (self.size - self.span) / 2.0

    @property
    def arc(self) -> Tuple[float, float]:
        return self.first, self.first + self.span

    @property
    def radians_per_index(self) -> float:
        return 2.0 * np.pi / self.size if self.circular else (self.upper - self.lower) / self.span

    def index(self, angle: float) -> int:
        """Nearest ring index of a joint angle (angles beyond the limits land on the arc's ends)."""

        if self.circular:
            return int(round(np.mod(float(angle), 2.0 * np.pi) / (2.0 * np.pi) * self.size)) % self.size
        phase = min(max((float(angle) - self.lower) / (self.upper - self.lower), 0.0), 1.0)
        return int(round(self.first + phase * self.span))

    def position(self, angle: float) -> float:
        """Ring position of a joint angle without rounding (a fractional index)."""

        if self.circular:
            return float(np.mod(float(angle), 2.0 * np.pi) / (2.0 * np.pi) * self.size)
        phase = min(max((float(angle) - self.lower) / (self.upper - self.lower), 0.0), 1.0)
        return float(self.first + phase * self.span)

    def angle(self, index: float) -> float:
        """Joint angle of a ring index (fractional indices allowed)."""

        if self.circular:
            return float(np.angle(np.exp(2j * np.pi * float(index) / self.size)))
        return self.lower + (float(index) - self.first) * self.radians_per_index

    def held_angle(self, angle: float, fractional: bool = False) -> float:
        """The angle the ring holds when an encoder writes ``angle`` on it: that of the nearest
        index, or the angle itself (within the limits) for an encoder that writes between indices."""

        return self.angle(self.position(angle) if fractional else self.index(angle))

    def neuron_angles(self) -> np.ndarray:
        """Joint angle each ring neuron stands for.

        On a bounded ring the line through the arc is continued beyond its ends, so a
        bump that sits on a limit and reaches a few neurons past it is still read at
        its centre, not pulled inward.
        """

        indices = np.arange(self.size, dtype=float)
        if self.circular:
            return 2.0 * np.pi * indices / self.size
        return self.lower + (indices - self.first) * self.radians_per_index

    def wall(self, margin: int = WALL_MARGIN) -> Tuple[int, int]:
        """Ring indices between which a bump may rest on a bounded ring."""

        first, last = self.arc
        return max(int(round(first)) - int(margin), 0), min(int(round(last)) + int(margin), self.size - 1)

    def population_vector_index(self, counts: np.ndarray) -> float:
        """Fractional ring index of the centre of the activity (NaN for a silent ring)."""

        values = np.asarray(counts, dtype=float)
        total = float(values.sum())
        if total <= 0.0:
            return float("nan")
        vector = np.sum(values * np.exp(2j * np.pi * np.arange(self.size) / self.size)) / total
        index = float((np.angle(vector) % (2.0 * np.pi)) * self.size / (2.0 * np.pi))
        if self.circular:
            return index
        middle = self.size / 2.0                    # express it on the side of the seam nearer the arc
        return middle + ((index - middle + self.size / 2.0) % self.size - self.size / 2.0)

    def index_error(self, a: float, b: float) -> float:
        """``a - b`` in ring indices (shortest way round on a circular ring)."""

        difference = float(a) - float(b)
        if self.circular:
            difference = (difference + self.size / 2.0) % self.size - self.size / 2.0
        return difference


__all__ = ["ARC_FRACTION", "RingMap", "WALL_MARGIN"]

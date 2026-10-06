"""Start poses and goals of the inverse-kinematics trials.

A goal is the orientation of another random joint pose of the same arm, so every goal can
be reached.  The seeds are kept apart: the reported trials, the trials used while the
gain and the lead were set, and the exact-rule study never share a pair.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

from ..experiments import POSE_SEED
from ..kinematics import Chain, lift_pitch_yaw

TEST_SEED = POSE_SEED + 21            # the reported trials
DEVELOPMENT_SEED = POSE_SEED + 31     # setting gain and lead
STUDY_SEED = POSE_SEED + 41           # the exact-rule study


@dataclass(frozen=True)
class Pair:
    index: int
    start: np.ndarray                 # joint angles at the start
    goal_pose: np.ndarray             # a joint pose that has the goal orientation (one of many)
    goal: np.ndarray                  # lift, pitch, yaw of the goal


def pairs(chain: Chain, count: int, seed: int = TEST_SEED, steep_deg: float = 0.0) -> List[Pair]:
    """``steep_deg`` > 0: only goals whose pitch is at least that far from level (the hard case near +-90 deg);
    those pairs are drawn from their own stream, so they are not the ordinary pairs."""

    key = [int(seed), chain.n, 0 if chain.joints[0].circular else 1] + ([int(round(steep_deg))] if steep_deg > 0 else [])
    rng = np.random.default_rng(key)
    out = []
    for index in range(int(count)):
        start = chain.random_pose(rng)
        while True:
            goal_pose = chain.random_pose(rng)
            goal = np.array(lift_pitch_yaw(chain.rotation(goal_pose)))
            if abs(np.degrees(goal[1])) >= steep_deg:
                break
        out.append(Pair(index, start, goal_pose, goal))
    return out


__all__ = ["DEVELOPMENT_SEED", "Pair", "STUDY_SEED", "TEST_SEED", "pairs"]

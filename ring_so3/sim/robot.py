"""The light robot of the simulator and of the tests: joint angles that move toward a target.

No dynamics: every joint approaches its target with a speed proportional to the
remaining distance, limited to ``max_speed`` (rad/s), so a move starts at the
limit and slows down smoothly at the end.  Joints on the whole circle take the
shorter way round.  :class:`GoalSequence` hands out one goal after another.
"""

from __future__ import annotations

from typing import List, Optional, Sequence

import numpy as np

from ..kinematics import Chain

APPROACH_GAIN = 6.0          # 1/s: the speed is this times the remaining distance, up to the limit
ARRIVED_RAD = 2.0e-3


class SimRobot:
    def __init__(self, chain: Chain, q: Optional[Sequence[float]] = None, max_speed: float = 1.0) -> None:
        self.chain = chain
        limits = chain.limits()
        self.lower, self.upper = limits[:, 0], limits[:, 1]
        self.circular = np.array([j.circular for j in chain.joints])
        self.q = self._legal(np.zeros(chain.n) if q is None else np.asarray(q, dtype=float))
        self.target = self.q.copy()
        self.max_speed = float(max_speed)
        self.velocity = np.zeros(chain.n)

    def _legal(self, q: np.ndarray) -> np.ndarray:
        wrapped = np.where(self.circular, np.angle(np.exp(1j * q)), q)
        return np.where(self.circular, wrapped, np.clip(wrapped, self.lower, self.upper))

    def set_target(self, q: Sequence[float]) -> None:
        self.target = self._legal(np.asarray(q, dtype=float).reshape(-1))

    def remaining(self) -> np.ndarray:
        difference = self.target - self.q
        return np.where(self.circular, np.angle(np.exp(1j * difference)), difference)

    def arrived(self) -> bool:
        return bool(np.all(np.abs(self.remaining()) < ARRIVED_RAD))

    def step(self, dt_s: float) -> np.ndarray:
        remaining = self.remaining()
        speed_limit = np.minimum(self.max_speed, np.array([j.max_velocity for j in self.chain.joints]))
        self.velocity = np.clip(APPROACH_GAIN * remaining, -speed_limit, speed_limit)
        move = self.velocity * float(dt_s)
        move = np.where(np.abs(remaining) < ARRIVED_RAD, remaining, move)          # land exactly
        self.q = self._legal(self.q + move)
        return self.q


class GoalSequence:
    """Random goals inside the joint limits, one after another, with a pause at each."""

    def __init__(self, chain: Chain, seed: int = 0, dwell_s: float = 1.0) -> None:
        self.chain = chain
        self.rng = np.random.default_rng(int(seed))
        self.dwell_s = float(dwell_s)
        self._waited = 0.0
        self.goals: List[np.ndarray] = []

    def next_goal(self) -> np.ndarray:
        goal = self.chain.random_pose(self.rng)
        self.goals.append(goal)
        return goal

    def update(self, robot: SimRobot, dt_s: float) -> bool:
        """Give the robot a new goal when it has rested at the current one; True when it did."""

        if not self.goals:
            robot.set_target(self.next_goal())
            return True
        if robot.arrived():
            self._waited += float(dt_s)
            if self._waited >= self.dwell_s:
                self._waited = 0.0
                robot.set_target(self.next_goal())
                return True
        return False


__all__ = ["ARRIVED_RAD", "GoalSequence", "SimRobot"]

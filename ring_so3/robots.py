"""The robot behind the inverse kinematics: the light robot, or the TIAGo arm in Gazebo.

Both have the same few calls, so the closed loop (:mod:`ring_so3.ik.session`), the simulator
and the experiments do not care which one runs:

* ``q``: the joint angles now (measured);
* ``follow(targets, time_s, dt)``: every joint moves toward its target as if it should arrive
  after ``time_s``, for one tick of ``dt``.  With the target a joint's goal-ring angle and
  ``time_s = 1 / kappa`` this is "the joint follows its goal ring with gain kappa";
* ``reset(q)``: put the arm at ``q`` (the light robot at once; Gazebo moves there);
* ``displace(joint, angle)``: push one joint (light robot only).
"""

from __future__ import annotations

from typing import Any, Optional, Sequence

import numpy as np

from .ik import rules
from .kinematics import Chain


def start_pose(chain: Chain) -> np.ndarray:
    return np.zeros(chain.n) if chain.joints[0].circular else chain.limits().mean(axis=1)


class LightRobot:
    """No dynamics: a joint moves exactly as told.  ``motor_scale``: it moves this much of what it is told."""

    name = "light"
    #: how close to a pose counts as there, rad
    tolerance = 1e-9
    #: how fast the arm is moved by hand between trials, rad/s
    place_speed = 3.0

    def __init__(self, chain: Chain, q: Optional[Sequence[float]] = None, motor_scale: float = 1.0) -> None:
        self.chain = chain
        self.circular = np.array([j.circular for j in chain.joints])
        self.q = np.array(q, dtype=float) if q is not None else start_pose(chain)
        self.motor_scale = float(motor_scale)
        self.target = self.q.copy()

    def difference(self, target: Sequence[float]) -> np.ndarray:
        d = np.asarray(target, dtype=float) - self.q
        return np.where(self.circular, rules.wrap(d), d)

    def follow(self, targets: Sequence[float], time_s: float, dt: float, exact: bool = False) -> np.ndarray:
        difference = self.difference(targets)
        difference = np.where(np.isfinite(difference), difference, 0.0)          # a silent goal ring: stay
        self.target = self.q + difference
        velocity = difference / max(float(time_s), 1e-9)
        self.q = rules.step_joints(self.chain, self.q, (1.0 if exact else self.motor_scale) * velocity, dt)
        return self.q

    def reset(self, q: Sequence[float]) -> None:
        self.q = rules.step_joints(self.chain, np.asarray(q, dtype=float), np.zeros(self.chain.n), 0.0)
        self.target = self.q.copy()

    def displace(self, joint: int, angle: float) -> None:
        moved = self.q.copy()
        moved[int(joint)] += float(angle)
        self.reset(moved)

    def describe(self) -> dict:
        return {"robot": self.name, "motor_scale": self.motor_scale}


class GazeboRobot:
    """The TIAGo arm in Gazebo, advanced in lock step: physics is paused, and every ``follow`` sends the
    targets to the arm's trajectory controller and then lets exactly one tick of physics pass.

    ``link``: a :class:`ring_so3.gazebo.link.RosLink` (or a stand-in with the same calls).  The chain's
    joints are the first joints of the arm; the others are held where they are.
    """

    name = "gazebo"
    tolerance = 0.01
    place_speed = 1.0

    def __init__(self, chain: Chain, link: Any, physics_step_s: float = 0.001) -> None:
        if not chain.name.startswith("tiago"):
            raise ValueError("Gazebo has the TIAGo arm only; %s is not one of its chains" % chain.name)
        self.chain = chain
        self.link = link
        self.physics_step_s = float(physics_step_s)
        self.circular = np.zeros(chain.n, dtype=bool)
        self.limits = chain.limits()
        self.q = np.zeros(chain.n)
        self.velocity = np.zeros(chain.n)
        self.target = np.zeros(chain.n)
        self.others = np.zeros(0)
        self.lag = np.zeros(chain.n)                 # target of the last tick minus where the joint is now
        self.sim_time_s = 0.0
        self.read()
        self.target = self.q.copy()

    def read(self) -> np.ndarray:
        positions, velocities, self.sim_time_s = self.link.joint_state()
        self.q = np.array(positions[: self.chain.n], dtype=float)
        self.velocity = np.array(velocities[: self.chain.n], dtype=float)
        self.others = np.array(positions[self.chain.n:], dtype=float)
        return self.q

    def difference(self, target: Sequence[float]) -> np.ndarray:
        return np.asarray(target, dtype=float) - self.q

    def follow(self, targets: Sequence[float], time_s: float, dt: float, exact: bool = False) -> np.ndarray:
        wanted = np.asarray(targets, dtype=float)
        wanted = np.where(np.isfinite(wanted), wanted, self.q)
        self.target = np.clip(wanted, self.limits[:, 0], self.limits[:, 1])
        self.link.send(list(self.target) + list(self.others), max(float(time_s), float(dt)))
        self.link.step(int(round(float(dt) / self.physics_step_s)))
        self.read()
        self.lag = self.target - self.q
        return self.q

    def reset(self, q: Sequence[float], speed: float = 1.0, dt: float = 0.05, settle_s: float = 1.0) -> None:
        """Move to ``q`` at up to ``speed`` rad/s and wait until the arm is there and still."""

        goal = np.clip(np.asarray(q, dtype=float), self.limits[:, 0], self.limits[:, 1])
        self.link.put_base_back()
        for _ in range(2000):
            difference = goal - self.q
            if np.max(np.abs(difference)) < self.tolerance and np.max(np.abs(self.velocity)) < 0.02:
                break
            self.follow(self.q + np.clip(difference, -speed * 4 * dt, speed * 4 * dt), 4 * dt, dt)
        for _ in range(int(round(settle_s / dt))):
            self.follow(goal, 4 * dt, dt)

    def displace(self, joint: int, angle: float) -> None:
        raise NotImplementedError("a joint cannot be pushed from outside in Gazebo here")

    def describe(self) -> dict:
        return {"robot": self.name, "physics_step_s": self.physics_step_s, "base": self.link.base_pose()}


__all__ = ["GazeboRobot", "LightRobot", "start_pose"]

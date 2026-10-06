"""The closed loop: the light robot follows each joint's goal ring, and the network sees where the robot is.

Every tick:

1. the goal written on the three goal rings travels toward the final goal (a bump cannot jump);
2. every joint moves with a speed of ``kappa`` times the distance from its own angle to the
   angle its goal ring held at the end of the last tick (no dynamics: the light robot);
3. the joint angles and the written goal are handed to the network, which is simulated for one tick.

A trial ends when the written goal has arrived and the hand has turned less than ``rest_deg``
within the last ``settle_s``, or after ``trial_s``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..kinematics import Chain, lift_pitch_yaw, rotation_angle
from ..robots import LightRobot
from . import rules


@dataclass
class Trial:
    t: np.ndarray                 # (ticks,) s
    q: np.ndarray                 # (ticks, n)
    error_deg: np.ndarray         # true orientation against the final goal
    shares: np.ndarray            # (ticks, n) decoded shares, rad
    lead: np.ndarray              # (ticks, n) lead of each goal ring over its state ring, indices
    left: np.ndarray              # (ticks,) length of what the arm could not take, rad
    written: np.ndarray           # (ticks, 3) the goal on the goal rings
    decoded_error_deg: np.ndarray  # decoded orientation against the true one
    goal_ring_error_deg: np.ndarray  # the orientation the three goal rings hold against the one written on them
    start_error_deg: float
    path_deg: float
    settled_s: Optional[float]
    at_limit: bool
    #: Gazebo only: how far every joint was from the target it was sent, per tick, rad
    lag: Optional[np.ndarray] = None

    @property
    def final_error_deg(self) -> float:
        return float(self.error_deg[-1])

    def time_to(self, degrees: float) -> float:
        below = np.flatnonzero(self.error_deg <= degrees)
        return float(self.t[below[0]]) if below.size else float("nan")


class IkSession:
    def __init__(self, network: Any, q: Optional[Sequence[float]] = None, motor_scale: float = 1.0, robot: Any = None) -> None:
        """``robot``: a :class:`ring_so3.robots.LightRobot` (the default) or ``GazeboRobot``."""

        self.network = network
        self.chain: Chain = network.chain
        self.design = network.ik_design
        self.dt = float(network.options.tick_ms) / 1000.0
        self.ring_size = int(network.calibration.settings["ring_size"])
        chain = self.chain
        self.robot = robot if robot is not None else LightRobot(chain, q, motor_scale)
        self.written = np.array(lift_pitch_yaw(chain.rotation(self.q)))
        self.goal = self.written.copy()
        self.follow = True
        self.last: Any = None
        self.circular = np.array([j.circular for j in chain.joints])
        #: another way to turn the comparison into joint targets (a standard method): ``targets = method(session)``
        self.method: Any = None

    @property
    def q(self) -> np.ndarray:
        return self.robot.q

    @q.setter
    def q(self, value: Sequence[float]) -> None:
        self.robot.reset(value)

    # -- one tick ------------------------------------------------------------
    def targets(self) -> np.ndarray:
        """Where every joint is told to go: the angle its goal ring holds (or what ``method`` says)."""

        if self.method is not None:
            return np.asarray(self.method(self), dtype=float)
        if self.last is None or not self.follow:
            return self.q.copy()
        return np.where(np.isfinite(self.last.joint_goal), self.last.joint_goal, self.q)

    def tick(self):
        self.written = rules.travel(self.written, self.goal, self.design.goal_speed * self.dt)
        self.robot.follow(self.targets(), 1.0 / self.design.kappa, self.dt)
        self.last = self.network.step_ik(self.q, self.written)
        return self.last

    def warm_up(self, ticks: int = 20) -> None:
        for _ in range(int(ticks)):
            self.hold()

    def hold(self, move: bool = True):
        """One tick with the goal kept on the hand: nothing is asked of the arm.  The goal on the rings
        travels to where the hand is at its usual speed (a bump does not follow a jump).
        ``move=False``: the robot has already made its step of this tick."""

        if move:
            self.robot.follow(self.q.copy(), self.dt, self.dt, exact=True)          # stay; in Gazebo this lets the tick pass
        self.goal = np.array(lift_pitch_yaw(self.chain.rotation(self.q)))
        self.written = rules.travel(self.written, self.goal, self.design.goal_speed * self.dt)
        self.last = self.network.step_ik(self.q, self.written)
        return self.last

    def place(self, q: Sequence[float], speed: Optional[float] = None, settle_ticks: int = 12) -> None:
        """Move the arm by hand to ``q``, let the goal on the rings arrive at the hand, then let the network settle."""

        target = np.asarray(q, dtype=float)
        speed = float(speed if speed is not None else self.robot.place_speed)
        for _ in range(2400):
            difference = self.robot.difference(target)
            there = np.max(np.abs(difference)) < self.robot.tolerance
            if there and np.max(np.abs(rules.wrap(self.written - self.goal))) < 1e-6:
                break
            if not there:
                self.robot.follow(self.q + np.clip(difference, -speed * self.dt, speed * self.dt), self.dt, self.dt, exact=True)
            else:
                self.robot.follow(target, self.dt, self.dt, exact=True)
            self.hold(move=False)
        for _ in range(int(settle_ticks)):
            self.hold()

    # -- one trial -----------------------------------------------------------
    def reach(self, goal_lpy: Sequence[float], push: Optional[Tuple[float, int, float]] = None,
              goal_change: Optional[Tuple[float, Sequence[float]]] = None, stop_when_settled: bool = True) -> Trial:
        """From where the arm is to the orientation ``goal_lpy``.  ``push``: ``(time, joint, angle)``;
        ``goal_change``: ``(time, new goal)``."""

        design, chain = self.design, self.chain
        self.goal = np.asarray(goal_lpy, dtype=float)
        final = rules.goal_matrix(*self.goal)
        previous = chain.rotation(self.q)
        start_error = float(np.degrees(rotation_angle(previous, final)))
        rows: Dict[str, List[Any]] = {key: [] for key in ("t", "q", "error", "shares", "lead", "left", "written", "decoded", "held", "lag")}
        path, quiet, settled = 0.0, 0, None
        recent: List[np.ndarray] = []
        for step in range(int(round(design.trial_s / self.dt))):
            now = step * self.dt
            if goal_change is not None and now >= goal_change[0]:
                self.goal, goal_change = np.asarray(goal_change[1], dtype=float), None
                final = rules.goal_matrix(*self.goal)
            if push is not None and now >= push[0]:
                self.robot.displace(int(push[1]), float(push[2]))
                push = None
            tick = self.tick()
            rotation = chain.rotation(self.q)
            path += rotation_angle(previous, rotation)
            previous = rotation
            rows["t"].append(now + self.dt)
            rows["q"].append(self.q.copy())
            rows["error"].append(float(np.degrees(rotation_angle(rotation, final))))
            rows["shares"].append(tick.shares.copy())
            rows["lead"].append(tick.lead.copy())
            rows["left"].append(float(np.linalg.norm(tick.left)))
            rows["written"].append(self.written.copy())
            if hasattr(self.robot, "lag"):
                rows["lag"].append(np.array(self.robot.lag, dtype=float))
            rows["decoded"].append(float(np.degrees(rotation_angle(tick.tick.tool_rotation(), rotation))))
            rows["held"].append(float(np.degrees(rotation_angle(rules.goal_matrix(*tick.goal_held), rules.goal_matrix(*self.written))))
                                if np.all(np.isfinite(tick.goal_held)) else float("nan"))
            arrived = bool(np.all(np.abs(rules.wrap(self.written - self.goal)) < 1e-6)) and goal_change is None and push is None
            quiet = quiet + 1 if arrived else 0
            window = int(round(design.settle_s / self.dt))
            recent.append(rotation)
            recent = recent[-(window + 1):]
            at_rest = len(recent) > window and np.degrees(rotation_angle(recent[0], rotation)) < design.rest_deg
            if stop_when_settled and quiet >= window and at_rest:
                settled = now + self.dt
                break
        at_limit = bool(np.any(rules.limit_room(chain, self.q, design, self.ring_size) < 0.5))
        return Trial(np.array(rows["t"]), np.array(rows["q"]), np.array(rows["error"]), np.array(rows["shares"]), np.array(rows["lead"]),
                     np.array(rows["left"]), np.array(rows["written"]), np.array(rows["decoded"]), np.array(rows["held"]), start_error,
                     float(np.degrees(path)), settled, at_limit, np.array(rows["lag"]) if rows["lag"] else None)


def trial_row(trial: Trial, reach_deg: float = 5.0, every: int = 4) -> Dict[str, Any]:
    """A JSON-ready record of one trial (the time courses thinned to every ``every``-th tick)."""

    return {
        "start_error_deg": trial.start_error_deg, "final_error_deg": trial.final_error_deg,
        "reached": bool(trial.final_error_deg <= reach_deg), "time_to_5_deg": trial.time_to(5.0),
        "path_deg": trial.path_deg, "path_ratio": trial.path_deg / trial.start_error_deg if trial.start_error_deg > 1e-9 else float("nan"),
        "settled_s": trial.settled_s, "duration_s": float(trial.t[-1]), "at_limit": bool(trial.at_limit),
        "final_q": trial.q[-1].tolist(),
        "lag_median_rad": float(np.median(np.max(np.abs(trial.lag), axis=1))) if trial.lag is not None else None,
        "lag_final_rad": [float(v) for v in trial.lag[-1]] if trial.lag is not None else None,
        "decoded_error_median_deg": float(np.median(trial.decoded_error_deg)),
        "decoded_error_final_deg": float(np.mean(trial.decoded_error_deg[-10:])),
        "goal_ring_error_final_deg": float(np.nanmean(trial.goal_ring_error_deg[-10:])),
        "goal_ring_error_deg": trial.goal_ring_error_deg[::every].tolist(),
        "largest_lead": float(np.nanmax(np.abs(trial.lead))) if np.any(np.isfinite(trial.lead)) else float("nan"),
        "t": trial.t[::every].tolist(), "error_deg": trial.error_deg[::every].tolist(),
        "left": trial.left[::every].tolist(), "shares": trial.shares[::every].tolist(), "lead": trial.lead[::every].tolist(),
        "q": trial.q[::every].tolist(),
    }


__all__ = ["IkSession", "Trial", "trial_row"]

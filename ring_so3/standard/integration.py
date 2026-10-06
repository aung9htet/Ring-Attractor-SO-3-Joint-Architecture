"""Path integration of the hand's orientation: the standard way to follow an orientation from velocities.

The arm moves; only the joint velocities are measured, each with noise and a constant bias.
The orientation is carried along by integrating: the joint angles from the velocities, and the
orientation from the angular velocity ``omega = sum_k a_k qdot_k`` with the exponential map
``R <- exp([omega dt]x) R``.  Nothing ties the estimate to the true pose, so its error grows.
A correction every ``correct_s`` seconds (the estimate is set to the truth, as a position
sensor or an anchor would do) bounds it.
"""

from __future__ import annotations

from typing import Any, Dict, Optional

import numpy as np

from ..ik import rules
from ..kinematics import Chain, rotation_angle
from ..sim.robot import GoalSequence, SimRobot


def run(chain: Chain, seconds: float = 60.0, dt: float = 0.05, speed: float = 0.5, noise: float = 0.02, bias: float = 0.005,
        correct_s: Optional[float] = None, seed: int = 0, motion_seed: int = 3) -> Dict[str, Any]:
    """Error over time of the integrated orientation.  ``noise``: standard deviation of a velocity
    reading, rad/s; ``bias``: size of each joint's constant velocity offset, rad/s (sign drawn per joint)."""

    rng = np.random.default_rng([int(seed), chain.n])
    robot = SimRobot(chain, np.zeros(chain.n) if chain.joints[0].circular else chain.limits().mean(axis=1), max_speed=speed)
    goals = GoalSequence(chain, motion_seed, dwell_s=1.0)
    offsets = bias * rng.choice([-1.0, 1.0], chain.n)
    q_hat = robot.q.copy()
    r_hat = chain.rotation(robot.q)
    t, by_exp, by_angles = [], [], []
    since = 0.0
    for step in range(int(round(seconds / dt))):
        goals.update(robot, dt)
        robot.step(dt)
        measured = robot.velocity + offsets + rng.normal(0.0, noise, chain.n)
        omega = rules.joint_axes(chain, q_hat + 0.5 * measured * dt).T @ measured     # axes as the estimate has them, at mid-step
        r_hat = rules.small_rotation(omega * dt) @ r_hat
        q_hat = q_hat + measured * dt
        since += dt
        if correct_s is not None and since >= correct_s:
            q_hat, r_hat, since = robot.q.copy(), chain.rotation(robot.q), 0.0
        true = chain.rotation(robot.q)
        t.append((step + 1) * dt)
        by_exp.append(float(np.degrees(rotation_angle(r_hat, true))))
        by_angles.append(float(np.degrees(rotation_angle(chain.rotation(q_hat), true))))
    by_exp, by_angles = np.array(by_exp), np.array(by_angles)
    return {"chain": chain.name, "seconds": float(seconds), "dt": float(dt), "speed": float(speed), "noise": float(noise),
            "bias": float(bias), "correct_s": correct_s, "t": t[::4], "error_deg": by_exp[::4].tolist(),
            "error_from_angles_deg": by_angles[::4].tolist(),
            "error_median_deg": float(np.median(by_exp)), "error_final_deg": float(by_exp[-1]),
            "error_at_10s_deg": float(by_exp[int(round(10.0 / dt)) - 1]) if seconds >= 10 else None,
            "error_at_30s_deg": float(by_exp[int(round(30.0 / dt)) - 1]) if seconds >= 30 else None,
            "error_from_angles_final_deg": float(by_angles[-1])}


__all__ = ["run"]

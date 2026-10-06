"""Standard inverse kinematics for an orientation goal, in the shape of the ring IK's loop.

Every method answers the same question each tick: given the joints' turning axes ``a_k`` (the
rows of ``axes``, the columns of the Jacobian ``J`` for orientation) and the turn ``e`` that is
still to make, how far should each joint move?  The answer is a vector of shares, as
:func:`ring_so3.ik.rules.cascade` gives, so the methods run in the same loop
(:func:`ring_so3.ik.rules.simulate`, argument ``rule``), with the same caps and joint limits.

* ``transpose``: ``t = J^T e``.  The gradient of the error; every joint gets the whole turn.
* ``dls``: damped least squares, ``t = J^T (J J^T + lambda^2 I)^-1 e``: the pseudo-inverse, kept
  finite near poses where the axes do not span every direction.  The smallest joint motion that
  makes the turn.  The usual choice in robot controllers.
* ``ccd``: cyclic coordinate descent.  Joint after joint, each turned by the angle that brings
  the hand closest to the goal with all others held; the next joint sees the result.
* :func:`solve`: the joint angles in one go, by bounded non-linear least squares from the start
  pose (no trajectory; says whether a solution is found, not how to get there).

What each needs per tick (n joints): transpose 3n multiplications; damped least squares a
3 x 3 matrix built from 6n products, its inverse, and 3n + 9 more; coordinate descent one
3 x 3 matrix product and an arctangent per joint.
"""

from __future__ import annotations

from typing import Callable, Dict, Optional, Sequence, Tuple

import numpy as np

from ..ik import rules
from ..kinematics import Chain, rotation_angle

METHODS = ("transpose", "dls", "ccd")
LABELS = {"transpose": "Jacobian transpose", "dls": "damped least squares", "ccd": "cyclic coordinate descent",
          "cascade": "exact cascade", "ring": "ring IK (spikes)"}
DAMPING = 0.1


def _within(shares: np.ndarray, caps: np.ndarray, room: Optional[np.ndarray]) -> np.ndarray:
    """Scale the whole step so that no joint exceeds its cap (the direction is kept), then fade at the limits."""

    scale = float(np.max(np.abs(shares) / caps)) if shares.size else 0.0
    out = shares / scale if scale > 1.0 else shares.copy()
    if room is not None:
        out = np.array([rules.faded(out[k], caps[k], room[k, 0], room[k, 1], "scale") for k in range(out.size)])
    return out


def transpose(axes, e, caps, room=None, rotation=None, goal=None) -> np.ndarray:
    shares = np.clip(np.asarray(axes) @ np.asarray(e), -caps, caps)
    if room is None:
        return shares
    return np.array([rules.faded(shares[k], caps[k], room[k, 0], room[k, 1], "scale") for k in range(shares.size)])


def dls(axes, e, caps, room=None, rotation=None, goal=None, damping: float = DAMPING) -> np.ndarray:
    a = np.asarray(axes, dtype=float)
    weights = np.ones(a.shape[0]) if room is None else np.array([room[k, 1] if a[k] @ e > 0 else room[k, 0] for k in range(a.shape[0])])
    jacobian = (a * weights[:, None]).T                           # a joint at its limit is left out of the solution
    shares = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping ** 2 * np.eye(3), np.asarray(e, dtype=float))
    return _within(shares, np.asarray(caps, dtype=float), room)


def best_angle(axis: np.ndarray, rotation: np.ndarray, goal: np.ndarray) -> float:
    """The turn about ``axis`` that brings ``rotation`` closest to ``goal``: it maximises trace(goal^T Rot(axis, x) rotation)."""

    m = rotation @ goal.T
    cross = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return float(np.arctan2(np.trace(cross @ m), np.trace(m) - axis @ m @ axis))


def ccd(axes, e, caps, room=None, rotation=None, goal=None) -> np.ndarray:
    a = np.asarray(axes, dtype=float)
    current = np.asarray(rotation, dtype=float).copy()
    shares = np.zeros(a.shape[0])
    for k in range(a.shape[0]):
        share = float(np.clip(best_angle(a[k], current, goal), -caps[k], caps[k]))
        if room is not None:
            share = rules.faded(share, caps[k], room[k, 0], room[k, 1], "scale")
        shares[k] = share
        current = rules.small_rotation(a[k] * share) @ current    # the next joint sees the hand where this one will have put it
    return shares


RULES: Dict[str, Callable[..., np.ndarray]] = {"transpose": transpose, "dls": dls, "ccd": ccd}
#: loop gain of each method, 1/s (the transpose makes the turn several times over and needs a lower one)
GAINS = {"transpose": 1.0, "dls": 1.25, "ccd": 1.25, "cascade": 1.25}


def solve(chain: Chain, start: Sequence[float], goal: np.ndarray, tolerance_deg: float = 0.01) -> Tuple[np.ndarray, float, int]:
    """Joint angles with the goal orientation, by bounded least squares from ``start``.
    Returns ``(q, error in degrees, evaluations of the kinematics)``."""

    from scipy.optimize import least_squares

    circular = np.array([j.circular for j in chain.joints])
    limits = chain.limits()
    lower = np.where(circular, -np.inf, limits[:, 0])
    upper = np.where(circular, np.inf, limits[:, 1])

    def residual(q):
        return (chain.rotation(q) - goal).reshape(-1)
    result = least_squares(residual, np.clip(np.asarray(start, dtype=float), lower, upper), bounds=(lower, upper), xtol=1e-12, ftol=1e-12)
    q = np.where(circular, rules.wrap(result.x), result.x)
    return q, float(np.degrees(rotation_angle(chain.rotation(q), goal))), int(result.nfev)


__all__ = ["DAMPING", "GAINS", "LABELS", "METHODS", "RULES", "best_angle", "ccd", "dls", "solve", "transpose"]

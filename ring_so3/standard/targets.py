"""A standard method as the thing a robot follows: joint targets from the exact joint angles."""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

from ..ik import rules
from . import ik as standard

#: the methods that read the exact joint angles (everything but the ring IK)
EXACT_METHODS = ("cascade", "dls", "transpose", "ccd")


def exact_targets(method: str) -> Callable[[Any], np.ndarray]:
    """Joint targets of a method that reads the exact joint angles (everything but the ring IK)."""

    def targets(session: Any) -> np.ndarray:
        chain, design, q = session.chain, session.design, session.q
        rotation = chain.rotation(q)
        e = rules.turn(rotation, session.written, "matrix")
        caps = rules.share_caps(chain, design, session.ring_size)
        room = rules.limit_room(chain, q, design, session.ring_size)
        axes = rules.joint_axes(chain, q)
        if method == "cascade":
            shares, _ = rules.cascade(axes, e, caps, room)
        else:
            shares = standard.RULES[method](axes, e, caps, room, rotation, rules.goal_matrix(*session.written))
        session.method_shares = np.asarray(shares, dtype=float)
        return q + session.method_shares * standard.GAINS.get(method, design.kappa) / design.kappa
    return targets


__all__ = ["EXACT_METHODS", "exact_targets"]

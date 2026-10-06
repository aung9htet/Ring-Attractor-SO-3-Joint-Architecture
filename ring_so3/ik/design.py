"""The settings of the inverse-kinematics layer (one place for the exact rule and the spiking network)."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Dict

COMPARISONS = ("matrix", "angles")
GOAL_RINGS = ("written", "attractor", "none")


@dataclass(frozen=True)
class IkDesign:
    #: how goal and decoded orientation are compared: ``matrix`` (the two orientation matrices) or
    #: ``angles`` (the differences of lift, pitch and yaw); both through cos and sin of the goal rings
    comparison: str = "matrix"
    #: True: joint k takes its share of what the joints before it left.  False: every joint gets the whole comparison
    cascade: bool = True
    #: loop gain, 1/s: a joint moves at ``kappa`` times its share (the light robot follows its goal ring with this gain)
    kappa: float = 1.25
    #: the largest lead of a joint's goal bump over its state bump, in ring indices: caps the share a joint takes
    lead_indices: float = 8.0
    #: ring indices before a joint limit over which the share that points beyond the limit fades to nothing
    limit_taper_indices: float = 5.0
    #: how a share fades near a joint limit: "subtract" (inhibition from the state ring) or "scale"
    limit_fade: str = "subtract"
    #: the goal written on the goal rings travels to a new goal at up to this speed (a bump cannot jump), rad/s
    goal_speed: float = 1.0
    #: each joint's goal ring: "written" (a gain pair gated by the state bump writes beside it, no attractor),
    #: "attractor" (a ring attractor with a share pair and a body pair), or "none" (comparison and shares only)
    goal_ring: str = "written"
    #: ticks of goal-ring spikes summed for one reading of where a joint's goal bump is
    goal_window: int = 3
    #: a trial ends when every joint's share has stayed below ``quiet_share`` (rad) for ``settle_s``, or after ``trial_s``
    quiet_share: float = 0.02
    settle_s: float = 2.0
    trial_s: float = 20.0
    #: in the spiking network the decoded shares never fall quiet (their readout is a few hundredths of a
    #: radian off), so there a trial ends when the hand has turned less than this within ``settle_s``, deg
    rest_deg: float = 0.5
    #: a goal counts as reached when the true orientation is within this of it at the end of the trial, deg
    reach_deg: float = 5.0

    def __post_init__(self) -> None:
        if self.comparison not in COMPARISONS:
            raise ValueError("comparison must be one of %r" % (COMPARISONS,))
        if self.goal_ring not in GOAL_RINGS:
            raise ValueError("goal_ring must be one of %r" % (GOAL_RINGS,))

    def describe(self) -> Dict[str, object]:
        return asdict(self)


__all__ = ["COMPARISONS", "GOAL_RINGS", "IkDesign"]

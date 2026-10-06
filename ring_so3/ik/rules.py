"""The inverse-kinematics rule in exact arithmetic (no neurons).

**The comparison** turns goal and current orientation into one vector ``e`` in the base
frame: the hand still has to turn about the direction of ``e``, by about its length (rad).

* ``matrix``: ``e = ½ Σ_i r_i × g_i`` over the columns of the current matrix ``R`` and the goal
  matrix ``G``.  This is ``sin(Δ) · n`` for the rotation by ``Δ`` about ``n`` that takes ``R``
  into ``G``; turning the hand along ``e`` raises ``trace(Gᵀ R)``.
* ``angles``: the three differences of lift, pitch and yaw as sines, each from two products
  of a goal cos / sin with an entry of ``R``, then expressed in the base frame::

      e_lift  = sin φ*·r33 − cos φ*·r32 = cos ϑ · sin(φ* − φ)
      e_pitch = sin ϑ*·N   + cos ϑ*·r31 =         sin(ϑ* − ϑ)      N = |(r11, r21)| = cos ϑ
      e_yaw   = sin ψ*·r11 − cos ψ*·r21 = cos ϑ · sin(ψ* − ψ)
      e = e_lift · r_1 + e_pitch · (−r21, r11, 0) + e_yaw · ẑ

  ``r_1`` (the first column), ``(−sin ψ, cos ψ, 0)`` and ``ẑ`` are the axes about which lift,
  pitch and yaw turn the hand; the whole is ``cos ϑ`` times the small turn those three
  differences describe, so it fades where lift and yaw lose their meaning.

**The cascade** hands ``e`` down the arm.  Joint ``k`` turns about its axis ``a_k`` (a unit
vector in the base frame, a column of link ``k``'s orientation); its *share* is the part of
what is left that lies about that axis, capped at what the joint can take::

    e_1 = e
    t_k = clip(a_k · e_k, ±cap_k)   (nothing toward a joint limit that is reached)
    e_(k+1) = e_k − a_k · t_k

so that ``Σ_k a_k t_k = e − e_(n+1)``: together the joints make the turn that is asked for,
less what none of them could take.  Without the cascade every joint gets the whole ``e``.

:func:`simulate` closes the loop with the light robot as the spiking network does: each
joint's goal leads its angle by its share, and the joint follows its goal.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from ..kinematics import AXES, Chain, Joint, from_lift_pitch_yaw, lift_pitch_yaw, rotation_angle
from ..mapping import RingMap
from .design import IkDesign

#: yaw about z, then pitch about y, then lift about x: the chain whose orientation is the goal matrix
GOAL_CHAIN = Chain("goal", (Joint("yaw", "z"), Joint("pitch", "y"), Joint("lift", "x")),
                   description="the three goal rings as a chain: R = Rot(z, yaw) Rot(y, pitch) Rot(x, lift)")


def goal_matrix(lift: float, pitch: float, yaw: float) -> np.ndarray:
    return from_lift_pitch_yaw(lift, pitch, yaw)


def joint_axes(chain: Chain, q: Sequence[float]) -> np.ndarray:
    """Turning axis of every joint in the base frame, ``(n, 3)``: the column of link k that joint k leaves alone."""

    rotations = chain.link_rotations(q)
    return np.array([rotations[k][:, AXES.index(joint.axis)] for k, joint in enumerate(chain.joints)])


def turn_from_matrices(rotation: np.ndarray, goal: np.ndarray) -> np.ndarray:
    r, g = np.asarray(rotation, dtype=float), np.asarray(goal, dtype=float)
    return 0.5 * sum(np.cross(r[:, i], g[:, i]) for i in range(3))


def angle_differences(rotation: np.ndarray, goal_lpy: Sequence[float]) -> np.ndarray:
    """``(e_lift, e_pitch, e_yaw)`` as the network forms them: products of goal cos / sin and entries of ``R``."""

    r = np.asarray(rotation, dtype=float)
    lift, pitch, yaw = [float(v) for v in goal_lpy]
    length = float(np.hypot(r[0, 0], r[1, 0]))
    return np.array([
        np.sin(lift) * r[2, 2] - np.cos(lift) * r[2, 1],
        np.sin(pitch) * length + np.cos(pitch) * r[2, 0],
        np.sin(yaw) * r[0, 0] - np.cos(yaw) * r[1, 0],
    ])


def turn_from_angles(rotation: np.ndarray, goal_lpy: Sequence[float]) -> np.ndarray:
    r = np.asarray(rotation, dtype=float)
    e_lift, e_pitch, e_yaw = angle_differences(r, goal_lpy)
    return e_lift * r[:, 0] + e_pitch * np.array([-r[1, 0], r[0, 0], 0.0]) + e_yaw * np.array([0.0, 0.0, 1.0])


def turn(rotation: np.ndarray, goal_lpy: Sequence[float], comparison: str = "matrix") -> np.ndarray:
    """The comparison: about which axis, and how far, the hand still has to turn."""

    if comparison == "matrix":
        return turn_from_matrices(rotation, goal_matrix(*goal_lpy))
    if comparison == "angles":
        return turn_from_angles(rotation, goal_lpy)
    raise ValueError("comparison must be 'matrix' or 'angles', got %r" % (comparison,))


def cascade(axes: np.ndarray, e: Sequence[float], caps: Optional[Sequence[float]] = None,
            room: Optional[np.ndarray] = None, pass_on: bool = True, fade: str = "subtract") -> Tuple[np.ndarray, np.ndarray]:
    """Shares ``t`` (n,) and what is left before each joint and after the last, ``rests`` (n + 1, 3).

    ``caps``: the largest share each joint takes.  ``room``: ``(n, 2)`` factors in [0, 1] on a
    negative and on a positive share (0 at a joint limit in that direction).  ``pass_on=False``:
    no cascade, every joint gets the whole ``e``.
    """

    a = np.asarray(axes, dtype=float)
    n = a.shape[0]
    rest = np.asarray(e, dtype=float).copy()
    shares, rests = np.zeros(n), [rest.copy()]
    for k in range(n):
        share = float(a[k] @ rest)
        if caps is not None:
            share = float(np.clip(share, -caps[k], caps[k]))
        if room is not None:
            share = faded(share, caps[k] if caps is not None else abs(share), room[k, 0], room[k, 1], fade)
        shares[k] = share
        if pass_on:
            rest = rest - a[k] * share
        rests.append(rest.copy())
    return shares, np.array(rests)


def faded(share: float, cap: float, room_minus: float, room_plus: float, fade: str = "subtract") -> float:
    """A share near a joint limit.  ``room``: 1 away from the limit the share points to, 0 at it.
    ``scale``: the share times the room.  ``subtract``: the share less ``(1 - room)`` caps, never below
    nothing (what inhibition from the state ring does to a population)."""

    room = float(room_plus if share > 0 else room_minus)
    if fade == "scale":
        return float(share) * room
    return float(np.sign(share) * max(abs(share) - (1.0 - room) * float(cap), 0.0))


def ring_maps(chain: Chain, ring_size: int = 200) -> List[RingMap]:
    return [RingMap.for_joint(joint, int(ring_size)) for joint in chain.joints]


def share_caps(chain: Chain, design: IkDesign, ring_size: int = 200) -> np.ndarray:
    """The largest share of every joint, rad: ``lead_indices`` ring indices of its own ring."""

    return np.array([design.lead_indices * m.radians_per_index for m in ring_maps(chain, ring_size)])


def limit_room(chain: Chain, q: Sequence[float], design: IkDesign, ring_size: int = 200) -> np.ndarray:
    """``(n, 2)``: how much of a negative / positive share a joint may take: 1 away from its limits,
    falling to 0 over ``limit_taper_indices`` ring indices before the limit it would move toward."""

    out = np.ones((chain.n, 2))
    for k, (joint, mapping) in enumerate(zip(chain.joints, ring_maps(chain, ring_size))):
        if joint.circular:
            continue
        taper = max(design.limit_taper_indices * mapping.radians_per_index, 1e-9)
        out[k, 0] = np.clip((float(q[k]) - joint.lower) / taper, 0.0, 1.0)
        out[k, 1] = np.clip((joint.upper - float(q[k])) / taper, 0.0, 1.0)
    return out


def wrap(angle):
    return np.angle(np.exp(1j * np.asarray(angle, dtype=float)))


def travel(current_lpy: np.ndarray, goal_lpy: Sequence[float], step: float) -> np.ndarray:
    """Move lift, pitch, yaw toward a goal by at most ``step`` each; lift and yaw the short way round."""

    goal = np.asarray(goal_lpy, dtype=float)
    difference = np.array([wrap(goal[0] - current_lpy[0]), goal[1] - current_lpy[1], wrap(goal[2] - current_lpy[2])])
    moved = current_lpy + np.clip(difference, -step, step)
    return np.array([wrap(moved[0]), moved[1], wrap(moved[2])])


def step_joints(chain: Chain, q: np.ndarray, velocity: np.ndarray, dt: float) -> np.ndarray:
    limits = chain.limits()
    circular = np.array([j.circular for j in chain.joints])
    moved = q + velocity * dt
    return np.where(circular, wrap(moved), np.clip(moved, limits[:, 0], limits[:, 1]))


def small_rotation(vector: Sequence[float]) -> np.ndarray:
    """The rotation by ``|vector|`` rad about ``vector`` (Rodrigues)."""

    w = np.asarray(vector, dtype=float)
    angle = float(np.linalg.norm(w))
    if angle < 1e-12:
        return np.eye(3)
    k = w / angle
    cross = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(angle) * cross + (1.0 - np.cos(angle)) * (cross @ cross)


@dataclass
class Trace:
    """One closed-loop trial of the exact rule."""

    t: np.ndarray                 # (ticks,) s
    q: np.ndarray                 # (ticks, n) joint angles
    error_deg: np.ndarray         # (ticks,) true orientation against the final goal
    shares: np.ndarray            # (ticks, n) rad
    left: np.ndarray              # (ticks,) length of what is left after the last joint
    start_error_deg: float
    path_deg: float               # how far the hand turned in all
    settled_s: Optional[float]    # when all shares had been quiet for the settle time (None: ran to the end)
    at_limit: bool                # a bounded joint ended at a limit (inside the zone where its share fades)

    @property
    def final_error_deg(self) -> float:
        return float(self.error_deg[-1])

    def time_to(self, degrees: float) -> float:
        hit = np.flatnonzero(self.error_deg <= float(degrees))
        return float(self.t[hit[0]]) if hit.size else float("nan")


def simulate(chain: Chain, q0: Sequence[float], goal_lpy: Sequence[float], design: Optional[IkDesign] = None, *,
             ring_size: int = 200, dt: float = 0.05, delay_s: float = 0.2, noise: float = 0.0, dead_zone: float = 0.0,
             readout_bias: Optional[np.ndarray] = None, motor_scale: float = 1.0, turn_taking: bool = False,
             push: Optional[Tuple[float, int, float]] = None, goal_change: Optional[Tuple[float, Sequence[float]]] = None,
             rng: Optional[np.random.Generator] = None, stop_when_settled: bool = True,
             turn_clip: Optional[float] = None, rule: Optional[Callable[..., np.ndarray]] = None) -> Trace:
    """Close the loop on the light robot.

    ``delay_s``: the comparison sees the arm as it was that long ago (the forward network's delay).
    ``noise``: standard deviation added to every share (rad); ``dead_zone``: shares below it move
    nothing (and are not passed on); ``readout_bias``: a fixed 3 x 3 rotation by which the decoded
    orientation is off.  ``motor_scale``: the joint moves this much of what it is told.
    ``push``: ``(time, joint, angle)`` displaces one joint; ``goal_change``: ``(time, new goal)``.
    ``turn_taking``: only one joint moves at a time, in sweeps down the arm (for comparison).
    ``turn_clip``: every component of the comparison is bounded by it (a population that is full).
    ``rule``: another way to hand out the turn (the standard methods of :mod:`ring_so3.standard.ik`):
    ``rule(axes, e, caps, room, seen_rotation, goal_rotation)`` returns every joint's share.
    """

    design = design or IkDesign()
    rng = rng or np.random.default_rng(0)
    n = chain.n
    caps = share_caps(chain, design, ring_size)
    q = np.asarray(q0, dtype=float).copy()
    history = [q.copy() for _ in range(int(round(delay_s / dt)) + 1)]
    final_goal = np.asarray(goal_lpy, dtype=float)
    written = np.array(lift_pitch_yaw(chain.rotation(q)))            # the goal rings start where the hand is
    bias = np.eye(3) if readout_bias is None else np.asarray(readout_bias, dtype=float)
    ticks = int(round(design.trial_s / dt))
    record_q, record_e, record_t, record_left = [], [], [], []
    previous = chain.rotation(q)
    path, quiet, settled, token, token_quiet = 0.0, 0, None, 0, 0
    start_error = float(np.degrees(rotation_angle(previous, goal_matrix(*final_goal))))
    for step in range(ticks):
        now = step * dt
        if goal_change is not None and now >= goal_change[0]:
            final_goal, goal_change = np.asarray(goal_change[1], dtype=float), None
        if push is not None and now >= push[0]:
            q = q.copy()
            q[int(push[1])] += float(push[2])
            q = step_joints(chain, q, np.zeros(n), dt)
            push = None
        written = travel(written, final_goal, design.goal_speed * dt)
        seen = history[0]
        axes = joint_axes(chain, seen)
        e = turn(bias @ chain.rotation(seen), written, design.comparison)
        if turn_clip is not None:
            e = np.clip(e, -float(turn_clip), float(turn_clip))
        room = limit_room(chain, q, design, ring_size)
        if turn_taking:
            shares, rests = np.zeros(n), np.array([e] * (n + 1))
            share = float(np.clip(axes[token] @ e + rng.normal(0.0, noise) if noise else axes[token] @ e, -caps[token], caps[token]))
            share = faded(share, caps[token], room[token, 0], room[token, 1], design.limit_fade)
            if abs(share) > max(dead_zone, design.quiet_share):
                shares[token], token_quiet = share, 0
            else:
                token_quiet += 1
                if token_quiet >= 4:
                    token, token_quiet = (token + 1) % n, 0
        elif rule is not None:
            shares = np.asarray(rule(axes, e, caps, room, bias @ chain.rotation(seen), goal_matrix(*written)), dtype=float).copy()
            if noise:
                shares = np.clip(shares + rng.normal(0.0, noise, n), -caps, caps)
            shares[np.abs(shares) < dead_zone] = 0.0
            rests = np.array([e] * n + [e - axes.T @ shares])
        else:
            rest = e.copy()
            shares, rest_list = np.zeros(n), [rest.copy()]
            for k in range(n):
                share = float(axes[k] @ rest) + (float(rng.normal(0.0, noise)) if noise else 0.0)
                share = float(np.clip(share, -caps[k], caps[k]))
                share = faded(share, caps[k], room[k, 0], room[k, 1], design.limit_fade)
                if abs(share) < dead_zone:
                    share = 0.0
                shares[k] = share
                if design.cascade:
                    rest = rest - axes[k] * share
                rest_list.append(rest.copy())
            rests = np.array(rest_list)
        q = step_joints(chain, q, motor_scale * design.kappa * shares, dt)
        history = history[1:] + [q.copy()]
        rotation = chain.rotation(q)
        path += rotation_angle(previous, rotation)
        previous = rotation
        record_q.append(q.copy())
        record_e.append(np.degrees(rotation_angle(rotation, goal_matrix(*final_goal))))
        record_t.append(shares.copy())
        record_left.append(float(np.linalg.norm(rests[-1])))
        arrived = bool(np.all(np.abs(wrap(written - final_goal)) < 1e-6))
        quiet = quiet + 1 if (arrived and not turn_taking and np.all(np.abs(shares) < design.quiet_share)) else 0
        if stop_when_settled and quiet >= int(round(design.settle_s / dt)):
            settled = now + dt
            break
    at_limit = bool(np.any(limit_room(chain, q, design, ring_size) < 0.5))          # a joint ended within reach of a limit
    count = len(record_e)
    return Trace(np.arange(1, count + 1) * dt, np.array(record_q), np.array(record_e), np.array(record_t), np.array(record_left),
                 start_error, float(np.degrees(path)), settled, at_limit)


__all__ = ["GOAL_CHAIN", "Trace", "angle_differences", "cascade", "faded", "goal_matrix", "joint_axes", "limit_room", "ring_maps",
           "share_caps", "simulate", "small_rotation", "step_joints", "travel", "turn", "turn_from_angles",
           "turn_from_matrices", "wrap"]

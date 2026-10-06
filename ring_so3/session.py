"""A network and a robot stepped together, and the measurement of one held pose.

:class:`Session` moves the light robot of :mod:`ring_so3.sim.robot` and the spiking
network in step: every tick the robot moves a little, its joint angles are written on the
rings, the network runs for one tick.  :meth:`Session.go_to` moves the arm to a pose,
holds it and averages the readout over the end of the hold; :func:`pose_row` compares
that readout with the exact kinematics and returns one result row.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from .kinematics import (Chain, circular_difference, lift_pitch_yaw, orientation_error_deg, orthonormality_defect)
from .sim.robot import SimRobot

#: The readout is late by about this much for the last joint (ring, cos / sin population, readout:
#: measured, docs/results.md section 6) and by about LEAD_PER_JOINT_S more for every joint that follows.
LEAD_BASE_S = 0.14
LEAD_PER_JOINT_S = 0.01


def lead_times(joints: int) -> np.ndarray:
    """How far ahead each joint's angle can be written so that the readout is on time while the arm moves."""

    return LEAD_BASE_S + LEAD_PER_JOINT_S * (int(joints) - 1 - np.arange(int(joints)))


HOLD_TICKS = 20          # 1 s at 50 ms ticks
MEASURE_TICKS = 10       # the readout is averaged over the last 0.5 s of the hold
MAX_MOVE_TICKS = 400


@dataclass
class Measurement:
    q: np.ndarray
    move_ticks: int
    tool: np.ndarray             # raw decoded end-effector matrix, mean over the measured ticks
    links: np.ndarray            # (n, 3, 3) raw decoded link matrices, mean
    angles: np.ndarray           # lift, pitch, yaw of the output rings at the end of the hold
    ring_spikes: np.ndarray      # spikes of the three output rings in their reading window
    held: np.ndarray             # angle each joint ring holds, mean over the measured ticks
    ring_error: np.ndarray       # largest distance of each ring's bump from the written position, ring indices
    arrived: bool


class Session:
    """``lead``: write on each ring the angle its joint will have :func:`lead_times` later (its angle
    plus velocity times the lead), instead of the angle it has now.  Off by default."""

    def __init__(self, network: Any, robot: SimRobot, lead: bool = False) -> None:
        self.network = network
        self.robot = robot
        self.dt_s = float(network.options.tick_ms) / 1000.0
        self.lead = bool(lead)
        self.lead_s = lead_times(robot.chain.n)

    def written(self) -> np.ndarray:
        """The angles handed to the encoders this tick."""

        if not self.lead:
            return self.robot.q
        ahead = self.robot.q + self.lead_s * self.robot.velocity
        return np.where(self.robot.circular, ahead, self.robot.chain.clip(ahead))      # a joint with limits is not written beyond them

    def tick(self):
        self.robot.step(self.dt_s)
        return self.network.step(self.written())

    def warm_up(self, ticks: int = 20) -> None:
        for _ in range(int(ticks)):
            self.network.step(self.robot.q)

    def go_to(self, q: Sequence[float], hold_ticks: int = HOLD_TICKS, measure_ticks: int = MEASURE_TICKS,
              max_move_ticks: int = MAX_MOVE_TICKS) -> Measurement:
        self.robot.set_target(q)
        moved = 0
        while not self.robot.arrived() and moved < max_move_ticks:
            self.tick()
            moved += 1
        arrived = self.robot.arrived()
        ticks = [self.tick() for _ in range(int(hold_ticks))]
        measured = ticks[-int(measure_ticks):]
        held = np.array([np.angle(np.mean(np.exp(1j * np.array([t.held[k] for t in measured]))))
                         for k in range(self.robot.chain.n)])
        return Measurement(
            q=self.robot.q.copy(), move_ticks=moved,
            tool=np.mean([t.tool for t in measured], axis=0), links=np.mean([t.links for t in measured], axis=0),
            angles=ticks[-1].angles.copy(), ring_spikes=ticks[-1].ring_spikes.copy(), held=held,
            ring_error=np.nanmax(np.abs([t.ring_error for t in measured]), axis=0), arrived=arrived,
        )


def written_angles(network: Any, q: Sequence[float]) -> np.ndarray:
    """What the encoders wrote on the rings for pose ``q`` (the pose itself, or the nearest ring indices)."""

    fractional = network.options.encoder == "fractional"
    return np.array([ring.mapping.held_angle(float(angle), fractional) for ring, angle in zip(network.rings, q)])


def pose_row(chain: Chain, network: Any, m: Measurement) -> Dict[str, Any]:
    """Compare one held pose's readout with the exact kinematics."""

    true = chain.rotation(m.q)
    true_links = chain.link_rotations(m.q)
    written = written_angles(network, m.q)
    lift, pitch, yaw = lift_pitch_yaw(true)
    ring_angle_error = [float(np.degrees(circular_difference(a, b))) if np.isfinite(a) else float("nan")
                        for a, b in zip(m.angles, (lift, pitch, yaw))]
    return {
        "q": [float(v) for v in m.q],
        "arrived": bool(m.arrived),
        "move_ticks": int(m.move_ticks),
        "error_deg": orientation_error_deg(m.tool, true),
        #: the same readout against the pose the rings were given (differs from the true pose only by the encoder's rounding)
        "decoder_error_deg": orientation_error_deg(m.tool, chain.rotation(written)),
        #: exact orientation of the written angles against the true pose: what rounding to a ring index costs
        "written_error_deg": orientation_error_deg(chain.rotation(written), true),
        #: exact orientation of the angles read back from the rings' activity against the true pose
        "held_error_deg": orientation_error_deg(chain.rotation(m.held), true) if np.all(np.isfinite(m.held)) else float("nan"),
        "link_error_deg": [orientation_error_deg(m.links[k], true_links[k]) for k in range(len(true_links))] if len(m.links) else [],
        "defect": orthonormality_defect(m.tool),
        "tool": [[float(v) for v in row] for row in m.tool],
        "true_lift_pitch_yaw": [float(lift), float(pitch), float(yaw)],
        "ring_lift_pitch_yaw": [float(v) for v in m.angles],
        "ring_angle_error_deg": ring_angle_error,
        "ring_spikes": [float(v) for v in m.ring_spikes],
        "cos_pitch": float(np.cos(pitch)),
        "joint_ring_error_index": [float(v) for v in m.ring_error],
    }


__all__ = ["HOLD_TICKS", "LEAD_BASE_S", "LEAD_PER_JOINT_S", "MEASURE_TICKS", "Measurement", "Session", "lead_times", "pose_row",
           "written_angles"]

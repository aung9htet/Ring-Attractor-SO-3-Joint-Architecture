"""The forward network with the inverse-kinematics layer on top (open loop: the arm is moved from outside).

:class:`IkNetwork` is an :class:`~ring_so3.nest.network.FkNetwork` that also holds

* three goal rings (yaw, pitch, lift) with their encoders and a cos / sin population each,
* the populations of an :class:`~ring_so3.ik.wiring.IkWiring`: the goal matrix, the
  comparison, and per joint the share (two halves) and what is left after it,
* near a joint limit, inhibition from that joint's ring onto the half that points beyond it,
* readout cells for the comparison, the shares and what is left.

:meth:`IkNetwork.step_ik` writes joint angles and a goal, simulates one tick and returns both
the forward readout and the layer's values.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..design import Design
from ..ik import rules
from ..ik.design import IkDesign
from ..ik.wiring import Half, IkWiring, Mul, Relay, TURN_SCALE, ik_wiring
from ..kinematics import Chain
from ..mapping import RingMap
from ..wiring import ONE, Node
from .calibrate import Calibration, feature_tuning
from .ik_calibrate import IkCalibration
from .network import FkNetwork, NetworkOptions, Tick
from .populations import Population, ReadoutCells, connect_weights, create_population, create_readout_cells
from .goal_ring import GoalRing, GoalRingSettings, WrittenRingSettings, build_goal_ring, build_written_ring
from .ring import JointRing, build_joint_ring

GOAL_NAMES = ("yaw", "pitch", "lift")          # the order of the goal chain
#: the inhibition near a limit keeps rising beyond the limit up to this many times the full hold
LIMIT_HOLD_MAX = 2.0


@dataclass
class IkTick:
    tick: Tick
    #: the comparison (rad), each joint's share (rad), what is left after every joint (n, 3) (rad)
    turn: np.ndarray
    shares: np.ndarray
    rests: np.ndarray
    #: lift, pitch, yaw the goal rings hold (NaN while silent)
    goal_held: np.ndarray
    #: the angle each joint's goal ring holds (NaN while silent, or without goal rings) and its lead
    #: over the joint's state ring in ring indices
    joint_goal: np.ndarray
    lead: np.ndarray

    @property
    def left(self) -> np.ndarray:
        return self.rests[-1]


class IkNetwork(FkNetwork):
    def __init__(self, backend: Any, chain: Chain, calibration: Calibration, ik_calibration: IkCalibration,
                 design: Optional[Design] = None, options: Optional[NetworkOptions] = None,
                 ik_design: Optional[IkDesign] = None, goal_ring_settings: Any = None) -> None:
        self.ik_design = ik_design or IkDesign()
        self.goal_ring_settings = goal_ring_settings
        self.joint_goal_rings: List[GoalRing] = []
        self._joint_goal_history: List[np.ndarray] = []
        self.ik_calibration = ik_calibration
        self.goal_rings: List[JointRing] = []
        self.goal_features: List[Population] = []
        self.ik_populations: Dict[Node, Population] = {}
        self.ik_cells: Optional[ReadoutCells] = None
        options = options or NetworkOptions()
        if options.mode != "chain" or not options.relay or not options.output_rings:
            raise ValueError("the IK layer needs the joint-by-joint forward network with its link populations and output rings")
        super().__init__(backend, chain, calibration, design, options)

    # -- construction --------------------------------------------------------
    def _build(self) -> None:
        super()._build()
        b, o, cal = self.backend, self.options, self.calibration
        size = int(cal.settings["ring_size"])
        self.ik_wiring: IkWiring = ik_wiring(self.chain, self.wiring, self.ik_design, size)
        tuning = feature_tuning(self.design, cal.settings["feature_tau"])
        mapping = RingMap(size, -np.pi, np.pi, True)
        angles = mapping.neuron_angles()
        basis = cal.feature_scale(mapping) * np.stack([np.cos(angles), np.sin(angles)], axis=1)
        for c, name in enumerate(GOAL_NAMES):
            ring = build_joint_ring(b, "goal_" + name, mapping, mode=o.encoder)
            feature = create_population(b, tuning, "goal_feature_" + name, seed=60 + c, record=o.record)
            connect_weights(b, ring.neurons, feature.neurons, tuning.input_weights(np.eye(2)) @ basis.T / feature.charge)
            self.goal_rings.append(ring)
            self.goal_features.append(feature)
            for part in (0, 1):
                self.sources[("gfeat", c, part)] = (feature, cal.decoders["feature"][:, part:part + 1])

        half_of = {node: (k, sign) for k, pair in enumerate(self.ik_wiring.shares) for node, sign in zip(pair, (1, -1))}
        for index, step in enumerate(self.ik_wiring.steps):
            seed = 20000 + index
            if isinstance(step, Mul):
                population = create_population(b, self.design.product, "ik_product%d" % index, seed=seed, record=o.record)
                self._feed(population, dict(step.a), 0)
                self._feed(population, dict(step.b), 1)
                decoders = cal.decoders["product"]
            else:
                population = create_population(b, self.design.entry, "ik_%s" % "_".join(str(p) for p in step.node), seed=seed, record=o.record)
                self._feed(population, dict(step.terms), 0)
                if isinstance(step, Half):
                    decoders = self.ik_calibration.decoders["half"]
                else:           # the goal matrix stays within what a population holds; comparison and rest can ask for more
                    decoders = cal.decoders["scalar"] if step.node[0] == "g" else self.ik_calibration.decoders["bounded"]
                if isinstance(step, Half) and step.node in half_of:
                    self._hold_at_limit(population, *half_of[step.node])
            self.ik_populations[step.node] = population
            self.sources[step.node] = (population, decoders)

        w = self.ik_wiring
        self._ik_nodes: List[Node] = list(w.turn) + [node for pair in w.shares for node in pair] + [node for rest in w.rests for node in rest]
        self.ik_cells = create_readout_cells(b, len(self._ik_nodes), {"tau_m": float(o.readout_tau_ms)})
        for cell, node in enumerate(self._ik_nodes):
            population, decoders = self.sources[node]
            self.ik_cells.connect(b, population, decoders, [cell])
        if self.ik_design.goal_ring != "none":
            written = self.ik_design.goal_ring == "written"
            settings = self.goal_ring_settings or (WrittenRingSettings() if written else GoalRingSettings())
            for k, (plus, minus) in enumerate(w.shares):
                build = build_written_ring if written else build_goal_ring
                self.joint_goal_rings.append(build(b, "joint_goal%d" % (k + 1), self.rings[k].mapping, self.rings[k],
                                                   self.sources[plus], self.sources[minus], settings))
        if o.noise_pa > 0.0:
            noise = b.Create("noise_generator", params={"mean": 0.0, "std": float(o.noise_pa), "dt": 1.0})
            for population in self.goal_features + list(self.ik_populations.values()):
                b.Connect(noise, population.neurons)
        summary = w.summary()
        self.build_log.append("IK layer (%s, %s): %d products, %d sum populations, %d share halves, %d neurons in all" % (
            self.ik_design.comparison, "cascade" if self.ik_design.cascade else "all at once",
            summary["products"], summary["sum_populations"], summary["share_populations"], self.neuron_count()))

    def _hold_at_limit(self, population: Population, joint: int, sign: int) -> None:
        """The joint's own ring holds down the half of its share that points beyond a limit."""

        ring = self.rings[joint]
        mapping = ring.mapping
        if mapping.circular:
            return
        taper = float(self.ik_design.limit_taper_indices)
        first, last = mapping.arc
        indices = np.arange(mapping.size, dtype=float)
        ramp = (indices - (last - taper)) / taper if sign > 0 else ((first + taper) - indices) / taper
        ramp = np.clip(ramp, 0.0, LIMIT_HOLD_MAX)
        per_hz = ramp / float(self.calibration.ring_profile.sum())            # a bump at index i gives about ramp(i)
        transform = np.array([[-1.0]])
        connect_weights(self.backend, ring.neurons, population.neurons,
                        population.tuning.input_weights(transform) @ per_hz[None, :] / population.charge)

    # -- running -------------------------------------------------------------
    def step_ik(self, q: Sequence[float], goal_lpy: Sequence[float]) -> IkTick:
        """Write joint angles and a goal (lift, pitch, yaw), simulate one tick, return what the network holds."""

        lift, pitch, yaw = [float(v) for v in goal_lpy]
        for ring, angle in zip(self.goal_rings, (yaw, pitch, lift)):
            ring.drive(self.backend, angle)
        if self.ik_design.goal_ring == "attractor":               # written again at the joint's angle when it has lost the state bump
            for goal, angle in zip(self.joint_goal_rings, np.asarray(q, dtype=float).reshape(-1)):
                goal.ring.drive(self.backend, float(angle))
        tick = self.step(q)
        held = []
        for ring in self.goal_rings:
            ring.read(self.backend)
            held.append(ring.held_angle())
        n = self.chain.n
        joint_goal, lead = np.full(n, np.nan), np.full(n, np.nan)
        if self.joint_goal_rings:
            counts = np.array([goal.ring.read(self.backend) for goal in self.joint_goal_rings])
            self._joint_goal_history = (self._joint_goal_history + [counts])[-int(self.ik_design.goal_window):]
            window = np.sum(self._joint_goal_history, axis=0)
            for k, (goal, state) in enumerate(zip(self.joint_goal_rings, self.rings)):
                index = goal.ring.mapping.population_vector_index(window[k])
                if np.isfinite(index):
                    joint_goal[k] = goal.ring.mapping.angle(index)
                    if np.isfinite(state.index):
                        lead[k] = goal.ring.mapping.index_error(index, state.index)
        values = dict(zip(self._ik_nodes, self.ik_cells.values(self.backend)))
        w = self.ik_wiring
        rests = np.array([[values[node] for node in rest] for rest in w.rests]) / TURN_SCALE
        return IkTick(tick, w.turn_vector(values), w.share_values(values), rests, np.array([held[2], held[1], held[0]]),
                      joint_goal, lead)

    def joint_goal_counts(self) -> np.ndarray:
        """Spike counts of every joint's goal ring in the last tick, (joints, ring size)."""

        if not self._joint_goal_history:
            return np.zeros((self.chain.n, int(self.calibration.settings["ring_size"])))
        return self._joint_goal_history[-1]

    def goal_counts(self) -> np.ndarray:
        """Spike counts of the goal rings in the last tick, rows lift, pitch, yaw."""

        return np.array([self.goal_rings[c].counts for c in (2, 1, 0)])

    def neuron_count(self) -> int:
        n = super().neuron_count()
        n += sum(ring.size for ring in self.goal_rings) + sum(p.n for p in self.goal_features)
        n += sum(p.n for p in self.ik_populations.values()) + sum(goal.neurons for goal in self.joint_goal_rings)
        return int(n)

    def silence_ik(self, fraction: float, seed: int = 0) -> int:
        """Switch off a random ``fraction`` of the neurons of comparison and cascade for good; returns how many."""

        rng = np.random.default_rng(int(seed))
        dead = 0
        for node in sorted(self.ik_populations, key=repr):
            population = self.ik_populations[node]
            for index in np.flatnonzero(rng.uniform(size=population.n) < float(fraction)):
                self.backend.SetStatus(population.neurons[int(index)], {"I_e": -1.0e6})
                dead += 1
        return dead

    def exact(self, q: Sequence[float], goal_lpy: Sequence[float]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """What the layer's own wiring gives with exact products and sums: comparison, shares, what is left."""

        w = self.ik_wiring
        room = rules.limit_room(self.chain, q, self.ik_design, int(self.calibration.settings["ring_size"]))
        values = w.evaluate(q, goal_lpy, room=room)
        return w.turn_vector(values), w.share_values(values), w.left_vector(values)


__all__ = ["GOAL_NAMES", "IkNetwork", "IkTick"]

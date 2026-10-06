"""The spiking network for one chain: joint rings in, link orientations and lift / pitch / yaw rings out.

:class:`FkNetwork` builds, in one NEST kernel,

* a joint ring per joint (:mod:`ring_so3.nest.ring`) with its encoder,
* a cos / sin population per joint that reads the ring with cos and sin weights,
* the product and link-orientation populations of a :class:`~ring_so3.wiring.Wiring`,
* nine readout cells per link and for the end effector (the decoded matrix entries),
* three output rings (lift, pitch, yaw) driven by the end-effector entries,

and then runs it tick by tick: :meth:`FkNetwork.step` writes the joint angles on the
rings, simulates one tick and returns what the network holds.  The kernel is persistent:
nothing is rebuilt between ticks, so the readout follows a moving arm.

All readout weights come from a :class:`~ring_so3.nest.calibrate.Calibration`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..design import Design
from ..kinematics import Chain, lift_pitch_yaw, nearest_rotation
from ..mapping import RingMap
from ..wiring import ONE, Combo, Entry, Node, Product, Wiring, make_wiring
from .calibrate import Calibration, feature_tuning
from .populations import (Population, ReadoutCells, connect_weights, create_population, create_readout_cells,
                          decoded_weights)
from .ring import JointRing, build_joint_ring

OUTPUT_NAMES = ("lift", "pitch", "yaw")
#: an output ring neuron starts to fire when the pair it is driven with, projected on its
#: preferred direction, exceeds this; a ring driven by a pair shorter than this stays silent
OUTPUT_THRESHOLD = 0.25
OUTPUT_PEAK_HZ = 150.0
#: ticks of output-ring spikes summed for one reading of its angle
OUTPUT_WINDOW = 4


@dataclass
class NetworkOptions:
    mode: str = "chain"              # wiring: "chain" (joint by joint) or "flat" (all at once)
    relay: bool = True               # link-orientation populations in the path
    encoder: str = "fractional"      # how a joint angle is written on its ring (ring.ENCODER_MODES)
    threads: int = 8
    seed: int = 1                    # kernel seed (the encoders' Poisson generators)
    tick_ms: float = 50.0
    readout_tau_ms: float = 50.0     # smoothing of the decoded matrix entries
    output_rings: bool = True
    record: bool = False             # spike recorders on product and link populations (diagnostics)
    noise_pa: float = 0.0            # standard deviation of a noise current into every neuron between the rings, pA


@dataclass
class Tick:
    t_ms: float
    #: angle each joint ring holds (NaN while silent) and its distance from the written index
    held: np.ndarray
    ring_error: np.ndarray
    #: raw decoded orientation of every link (n, 3, 3) and of the end effector (3, 3)
    links: np.ndarray
    tool: np.ndarray
    #: lift, pitch, yaw read from the output rings (rad; NaN while a ring is silent) and their spike counts
    angles: np.ndarray
    ring_spikes: np.ndarray

    def tool_rotation(self) -> np.ndarray:
        return nearest_rotation(self.tool)


class FkNetwork:
    def __init__(self, backend: Any, chain: Chain, calibration: Calibration, design: Optional[Design] = None,
                 options: Optional[NetworkOptions] = None) -> None:
        self.backend = backend
        self.chain = chain
        self.calibration = calibration
        self.design = design or Design()
        self.options = options or NetworkOptions()
        self.wiring: Wiring = make_wiring(chain, self.options.mode, self.options.relay)
        self.t_ms = 0.0
        self.rings: List[JointRing] = []
        self.features: List[Population] = []
        self.sources: Dict[Node, Tuple[Population, np.ndarray]] = {}
        self.products: Dict[Node, Population] = {}
        self.entries: Dict[Node, Population] = {}
        self.outputs: Dict[str, Population] = {}
        self.cells: Optional[ReadoutCells] = None
        self.build_log: List[str] = []
        self._output_history: List[np.ndarray] = []
        self._build()

    # -- construction --------------------------------------------------------
    def _build(self) -> None:
        b, o, cal = self.backend, self.options, self.calibration
        size = int(cal.settings["ring_size"])
        b.ResetKernel()
        b.set_verbosity("M_ERROR")
        b.SetKernelStatus({"local_num_threads": int(o.threads), "rng_seed": int(o.seed)})
        tuning = feature_tuning(self.design, cal.settings["feature_tau"])
        d_feature = cal.decoders["feature"]
        for k, joint in enumerate(self.chain.joints):
            mapping = RingMap.for_joint(joint, size)
            ring = build_joint_ring(b, "ring%d" % (k + 1), mapping, mode=o.encoder)
            feature = create_population(b, tuning, "feature%d" % (k + 1), seed=10 + k, record=o.record)
            angles = mapping.neuron_angles()
            basis = cal.feature_scale(mapping) * np.stack([np.cos(angles), np.sin(angles)], axis=1)
            connect_weights(b, ring.neurons, feature.neurons, tuning.input_weights(np.eye(2)) @ basis.T / feature.charge)
            self.rings.append(ring)
            self.features.append(feature)
            for part in (0, 1):
                self.sources[("feat", k, part)] = (feature, d_feature[:, part:part + 1])

        for step in self.wiring.steps:
            if isinstance(step, Product):
                population = create_population(b, self.design.product, "product%d" % step.index, seed=100 + step.index, record=o.record)
                self._feed(population, dict(step.operand), 0)
                self._feed(population, {step.feature: 1.0}, 1)
                self.products[step.node] = population
                self.sources[step.node] = (population, cal.decoders["product"])
            else:
                population = create_population(b, self.design.entry, "link%d_%d%d" % (step.link, step.row, step.col),
                                               seed=5000 + 100 * step.link + 10 * step.row + step.col, record=o.record)
                self._feed(population, dict(step.terms), 0)
                self.entries[step.node] = population
                self.sources[step.node] = (population, cal.decoders["scalar"])

        matrices = list(self.wiring.links) + [self.wiring.tool]
        self.cells = create_readout_cells(b, 9 * len(matrices), {"tau_m": float(o.readout_tau_ms)})
        constants = np.zeros(self.cells.n)
        for m, matrix in enumerate(matrices):
            for i in range(3):
                for j in range(3):
                    cell = 9 * m + 3 * i + j
                    for node, coefficient in matrix[i][j].items():
                        if node == ONE:
                            constants[cell] += coefficient
                        else:
                            population, decoders = self.sources[node]
                            self.cells.connect(b, population, decoders * coefficient, [cell])
        self.cells.set_constants(b, constants)

        if o.output_rings:
            self._build_output_rings()
        if o.noise_pa > 0.0:
            noise = b.Create("noise_generator", params={"mean": 0.0, "std": float(o.noise_pa), "dt": 1.0})
            populations = self.features + list(self.products.values()) + list(self.entries.values())
            populations += [self.pitch_norm] if o.output_rings else []
            for population in populations:
                b.Connect(noise, population.neurons)
        status = b.GetKernelStatus()
        self.build_log.append("%s (%s%s): %d joints, %d products, %d link populations, %d neurons, %d synapses" % (
            self.chain.name, o.mode, "" if o.relay or o.mode == "flat" else ", no link populations in the path",
            self.chain.n, len(self.products), len(self.entries), self.neuron_count(), int(status["num_connections"])))

    def _feed(self, population: Population, combo: Combo, dim: int) -> None:
        """Hand the value of ``combo`` to dimension ``dim`` of ``population``."""

        transform = np.zeros((population.tuning.dims, 1))
        transform[dim, 0] = 1.0
        for node, coefficient in combo.items():
            if node == ONE:
                extra = population.tuning.input_weights(transform)[:, 0] * float(coefficient)
                current = np.asarray(self.backend.GetStatus(population.neurons, "I_e"), dtype=float) + extra
                self.backend.SetStatus(population.neurons, [{"I_e": float(c)} for c in current])
            else:
                source, decoders = self.sources[node]
                connect_weights(self.backend, source.neurons, population.neurons,
                                decoded_weights(decoders * float(coefficient), transform, population))

    def _build_output_rings(self) -> None:
        b, cal, model = self.backend, self.calibration, self.design.model
        size = int(self.design.sizes.output_ring)
        theta = 2.0 * np.pi * np.arange(size) / size
        gain = float(model.current(np.array([OUTPUT_PEAK_HZ]))[0] - model.rheobase) / (1.0 - OUTPUT_THRESHOLD)
        tool = self.wiring.tool
        # pitch = atan2(-r31, cos(pitch)), and cos(pitch) is the length of the pair (r11, r21):
        # one more population holds that pair and gives its length
        norm = create_population(b, self.design.norm, "pitch_norm", seed=9000, record=self.options.record)
        self._feed(norm, tool[0][0], 0)
        self._feed(norm, tool[1][0], 1)
        self.pitch_norm = norm
        norm_node = ("norm",)
        self.sources[norm_node] = (norm, cal.decoders["norm"])
        minus_r31 = {node: -coefficient for node, coefficient in tool[2][0].items()}
        drives = {
            "lift": (tool[2][2], tool[2][1]),          # atan2(r32, r33)
            "pitch": ({norm_node: 1.0}, minus_r31),    # atan2(-r31, |(r11, r21)|)
            "yaw": (tool[0][0], tool[1][0]),           # atan2(r21, r11)
        }
        for name in OUTPUT_NAMES:
            neurons = b.Create("iaf_psc_alpha", size, params=model.nest_params())
            bias = np.full(size, model.rheobase - gain * OUTPUT_THRESHOLD)
            for combo, direction in zip(drives[name], (np.cos(theta), np.sin(theta))):
                for node, coefficient in combo.items():
                    if node == ONE:
                        bias = bias + gain * coefficient * direction
                    else:
                        source, decoders = self.sources[node]
                        connect_weights(b, source.neurons, neurons,
                                        gain * coefficient * np.outer(direction, decoders[:, 0]) / model.charge)
            start = np.random.default_rng([7, OUTPUT_NAMES.index(name)]).uniform(0.0, model.V_th, size)
            b.SetStatus(neurons, [{"I_e": float(c), "V_m": float(v)} for c, v in zip(bias, start)])
            recorder = b.Create("spike_recorder")
            b.Connect(neurons, recorder)
            self.outputs[name] = Population(name, neurons, size, int(np.min(b.GetStatus(neurons, "global_id"))), None, recorder, model.charge)

    # -- running -------------------------------------------------------------
    def step(self, q: Sequence[float]) -> Tick:
        """Write the joint angles ``q`` on the rings, simulate one tick, return the readout."""

        b = self.backend
        angles = np.asarray(q, dtype=float).reshape(-1)
        for ring, angle in zip(self.rings, angles):
            ring.drive(b, float(angle))
        b.Simulate(float(self.options.tick_ms))
        self.t_ms += float(self.options.tick_ms)
        held, ring_error = [], []
        for ring in self.rings:
            ring.read(b)
            held.append(ring.held_angle())
            ring_error.append(ring.mapping.index_error(ring.index, ring.target_index) if np.isfinite(ring.index) else float("nan"))
        values = self.cells.values(b).reshape(-1, 3, 3)
        out_angles, out_spikes = np.full(3, np.nan), np.zeros(3)
        if self.outputs:
            counts = np.array([self.outputs[name].counts(b) for name in OUTPUT_NAMES])
            self._output_history = (self._output_history + [counts])[-OUTPUT_WINDOW:]
            window = np.sum(self._output_history, axis=0)
            size = window.shape[1]
            vector = window @ np.exp(2j * np.pi * np.arange(size) / size)
            out_spikes = window.sum(axis=1)
            out_angles = np.where(out_spikes > 0, np.angle(vector), np.nan)
        return Tick(self.t_ms, np.array(held), np.array(ring_error), values[:-1], values[-1], out_angles, out_spikes)

    def run(self, q: Sequence[float], ticks: int) -> List[Tick]:
        return [self.step(q) for _ in range(int(ticks))]

    def ring_counts(self) -> np.ndarray:
        """Spike counts of every joint ring in the last tick, (joints, ring size)."""

        return np.array([ring.counts for ring in self.rings])

    def output_counts(self) -> np.ndarray:
        """Spike counts of the three output rings in the last tick, (3, ring size)."""

        return self._output_history[-1] if self._output_history else np.zeros((3, int(self.design.sizes.output_ring)))

    def neuron_count(self) -> int:
        n = sum(ring.size for ring in self.rings) + sum(p.n for p in self.features)
        n += sum(p.n for p in self.products.values()) + sum(p.n for p in self.entries.values())
        n += sum(p.n for p in self.outputs.values()) + (self.pitch_norm.n if self.outputs else 0)
        return int(n)

    def silence(self, fraction: float, seed: int = 0) -> int:
        """Switch off a random ``fraction`` of the product neurons for good (robustness test); returns how many."""

        rng = np.random.default_rng(int(seed))
        dead = 0
        for population in self.products.values():
            chosen = np.flatnonzero(rng.uniform(size=population.n) < float(fraction))
            for index in chosen:
                self.backend.SetStatus(population.neurons[int(index)], {"I_e": -1.0e6})
            dead += int(chosen.size)
        return dead


def decoded_angles(tick: Tick) -> Tuple[float, float, float]:
    """Lift, pitch, yaw of the decoded matrix (the output rings give them as bumps; this is the matrix's own)."""

    return lift_pitch_yaw(tick.tool_rotation())


__all__ = ["FkNetwork", "NetworkOptions", "OUTPUT_NAMES", "Tick", "decoded_angles"]

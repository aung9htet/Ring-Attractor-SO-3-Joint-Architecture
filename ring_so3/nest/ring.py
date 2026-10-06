"""The joint ring: the ring attractor of the SO-2 repo, and the encoder that writes a joint angle on it.

Ported from the SO-2 repo (branch ``velocity-calibration``), unchanged in what it computes:

* connectivity: ``math/ring.py`` (``legacy_ring_distances``, ``legacy_ring_weight``,
  ``ring_weight_matrix``): every neuron excites its neighbours and inhibits the rest with a
  difference of two Gaussians over the ring distance (0 .. 50 for the far side);
* neuron: ``config/model_params/neuron_params.json``, entry ``ring`` (Table 1 of the paper);
* bounded ring: ``nest/populations.py`` (``wall_profile``) and ``graph/templates.py``
  (``BOUNDED``): neurons more than 15 indices outside the joint's arc get 90% of the bias
  current, so no bump rests there;
* stimulus: ``nest/populations.py`` (``build_stimulus_generators``): one Poisson generator
  per neuron, 200 Hz on the 11 neurons around the target index, weight 4500;
* corrective encoder: ``blocks/encoder.py`` (mode ``corrective``): stimulate for one tick
  only when the centre of the ring's activity is further than ``dead_band`` indices from
  the target index (the state-ring setting of architecture v4);
* continuous encoder: the same stimulus every tick (SO-2's mode ``continuous``).

New here: the ``fractional`` encoder.  It stimulates every tick like ``continuous``, but
the 11-neuron window is placed at the unrounded ring position of the angle: the two
neurons at the window's ends get the share of the rate that the fraction leaves them, so
the centre of the stimulus is the angle itself and not the nearest index.

Different from SO-2: one spike recorder per ring instead of one per neuron (counts per
tick are taken from the recorded senders), and the weights are connected as one matrix.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

import numpy as np

from ..mapping import RingMap, WALL_MARGIN

#: ``iaf_psc_alpha`` parameters of a ring neuron (SO-2 ``neuron_params.json``, "ring")
RING_NEURON: Dict[str, float] = {
    "V_m": 0.0, "E_L": 0.0, "V_th": 0.8, "V_reset": 0.0, "tau_syn_ex": 27.0, "tau_syn_in": 50.0,
    "I_e": 450000.0, "tau_m": 7.04, "C_m": 3960000.0, "t_ref": 0.0,
}
MAX_DISTANCE = 50.0
EXCITATION_SD = 10.0
INHIBITION_SD = 5.0
OUTSIDE_CURRENT_SCALE = 0.9
STIMULUS_RATE_HZ = 200.0
STIMULUS_WEIGHT = 4.5e3
STIMULUS_HALF_WIDTH = 5
DEAD_BAND = 1


def ring_distances(size: int, max_distance: float = MAX_DISTANCE) -> np.ndarray:
    """Distance of every neuron from neuron 0 along the ring (SO-2 ``legacy_ring_distances``)."""

    half = int(size / 2)
    if size % 2 == 0:
        up = np.linspace(0, max_distance, half + 1)[1:half]
        down = np.linspace(max_distance, 0, half + 1)[1:half]
        return np.concatenate([[0.0], up, [max_distance], down])
    up = np.linspace(0, max_distance, half + 1)[1:]
    down = np.linspace(max_distance, 0, half + 1)[:half]
    return np.concatenate([[0.0], up, down])


def ring_weight(distance: np.ndarray, sd_1: float = EXCITATION_SD, sd_2: float = INHIBITION_SD) -> np.ndarray:
    """Recurrent weight for a ring distance (SO-2 ``legacy_ring_weight``), rounded as there."""

    d = np.asarray(distance, dtype=float)
    weight = 3.0 * ((sd_2 * np.exp(-(d ** 2) / (2 * sd_1 ** 2)) - sd_1 * np.exp(-(d ** 2) / (2 * sd_2 ** 2))) / (sd_2 - sd_1))
    return np.around(weight, 3)


def ring_weight_matrix(size: int) -> np.ndarray:
    """``(pre, post)`` recurrent weights, one per pair, self-connection included."""

    profile = ring_weight(ring_distances(int(size)))
    pre = np.arange(int(size))[:, None]
    shift = np.arange(int(size))[None, :]
    matrix = np.zeros((int(size), int(size)))
    matrix[pre, (pre + shift) % int(size)] = profile[None, :]
    return matrix


def wall_profile(size: int, low: float, high: float, outside_scale: float = OUTSIDE_CURRENT_SCALE) -> np.ndarray:
    """Bias-current factor per neuron: 1 between ``low`` and ``high``, ``outside_scale`` elsewhere."""

    indices = np.arange(int(size), dtype=float)
    outside = (np.maximum(low - indices, 0.0) + np.maximum(indices - high, 0.0)) > 0
    return 1.0 - outside * (1.0 - float(outside_scale))


def bump_rates(size: int, center: int, half_width: int = STIMULUS_HALF_WIDTH, rate_hz: float = STIMULUS_RATE_HZ) -> np.ndarray:
    rates = np.zeros(int(size))
    for offset in range(-int(half_width), int(half_width) + 1):
        rates[(int(center) + offset) % int(size)] = float(rate_hz)
    return rates


def fractional_bump_rates(size: int, position: float, half_width: int = STIMULUS_HALF_WIDTH,
                          rate_hz: float = STIMULUS_RATE_HZ) -> np.ndarray:
    """The stimulus window centred on a fractional ring position: a blend of the two nearest whole windows."""

    low = int(np.floor(position))
    fraction = float(position) - low
    return (1.0 - fraction) * bump_rates(size, low, half_width, rate_hz) + fraction * bump_rates(size, low + 1, half_width, rate_hz)


ENCODER_MODES = ("fractional", "continuous", "corrective")


@dataclass
class JointRing:
    """One ring in the NEST kernel, with its stimulus generators and recorder."""

    name: str
    mapping: RingMap
    neurons: Any
    generators: Any
    recorder: Any
    first_id: int
    dead_band: int = DEAD_BAND
    half_width: int = STIMULUS_HALF_WIDTH
    rate_hz: float = STIMULUS_RATE_HZ
    #: ``fractional`` or ``continuous`` (stimulate every tick) or ``corrective`` (as v4's state ring)
    mode: str = "fractional"

    def __post_init__(self) -> None:
        if self.mode not in ENCODER_MODES:
            raise ValueError("encoder mode must be one of %r" % (ENCODER_MODES,))
        self.stimulated = False
        self.target_index: Optional[float] = None
        self.counts = np.zeros(self.mapping.size)
        self.index = float("nan")

    @property
    def size(self) -> int:
        return self.mapping.size

    def drive(self, backend: Any, angle: float) -> bool:
        """Call before a tick: decide from the last tick's counts whether to stimulate. True when stimulating."""

        fractional = self.mode == "fractional"
        target = self.mapping.position(angle) if fractional else self.mapping.index(angle)
        self.target_index = target
        needed = True
        if self.mode == "corrective" and np.isfinite(self.index):
            needed = abs(self.mapping.index_error(self.index, target)) > self.dead_band
        if needed:
            rates = (fractional_bump_rates(self.size, target, self.half_width, self.rate_hz) if fractional
                     else bump_rates(self.size, int(target), self.half_width, self.rate_hz))
            backend.SetStatus(self.generators, [{"rate": float(r)} for r in rates])
            self.stimulated = True
        elif self.stimulated:
            backend.SetStatus(self.generators, {"rate": 0.0})
            self.stimulated = False
        return needed

    def release(self, backend: Any) -> None:
        if self.stimulated:
            backend.SetStatus(self.generators, {"rate": 0.0})
            self.stimulated = False

    def read(self, backend: Any) -> np.ndarray:
        """Call after a tick: spike counts of the tick, and the centre of the activity."""

        events = backend.GetStatus(self.recorder, "events")[0]
        senders = np.asarray(events["senders"], dtype=int)
        self.counts = np.bincount(senders - self.first_id, minlength=self.size).astype(float)[: self.size]
        backend.SetStatus(self.recorder, {"n_events": 0})
        self.index = self.mapping.population_vector_index(self.counts)
        return self.counts

    def held_angle(self) -> float:
        """The joint angle the ring holds now (NaN while it is silent)."""

        return self.mapping.angle(self.index) if np.isfinite(self.index) else float("nan")


def build_joint_ring(backend: Any, name: str, mapping: RingMap, mode: str = "fractional",
                     dead_band: int = DEAD_BAND, neuron: Optional[Dict[str, float]] = None) -> JointRing:
    size = mapping.size
    params = dict(neuron or RING_NEURON)
    neurons = backend.Create("iaf_psc_alpha", size, params=params)
    if not mapping.circular:
        low, high = mapping.wall(WALL_MARGIN)
        factors = wall_profile(size, low, high)
        backend.SetStatus(neurons, [{"I_e": params["I_e"] * float(f)} for f in factors])
    backend.Connect(neurons, neurons, {"rule": "all_to_all"}, {"weight": np.ascontiguousarray(ring_weight_matrix(size).T)})
    generators = backend.Create("poisson_generator", size, params={"rate": 0.0})
    backend.Connect(generators, neurons, "one_to_one", {"weight": STIMULUS_WEIGHT})
    recorder = backend.Create("spike_recorder")
    backend.Connect(neurons, recorder)
    first_id = int(np.min(backend.GetStatus(neurons, "global_id")))
    return JointRing(name, mapping, neurons, generators, recorder, first_id, dead_band=int(dead_band), mode=mode)


__all__ = ["DEAD_BAND", "ENCODER_MODES", "JointRing", "RING_NEURON", "STIMULUS_HALF_WIDTH", "STIMULUS_RATE_HZ", "STIMULUS_WEIGHT",
           "build_joint_ring", "bump_rates", "fractional_bump_rates", "ring_distances", "ring_weight", "ring_weight_matrix", "wall_profile"]

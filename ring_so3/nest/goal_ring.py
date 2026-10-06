"""One joint's goal ring: where the joint should be next.

A ring attractor ``G`` beside the joint's state ring ``S``, and two gain pairs on it that
compete, as the two pairs of architecture v4 (SO-2 repo) do on its predictive ring:

* the **share pair** pushes the bump of ``G`` in the direction of the joint's share of the
  turn (reach the goal).  Its neurons are gated by the bump of ``G`` and driven by the two
  halves of the share;
* the **body pair** pulls the bump of ``G`` back to the bump of ``S`` (stay with the body).
  Its neurons are gated by the bump of ``G`` and steered by ``S`` index by index
  (SO-2 ``connect_gain_reference``).

Both act through feedback that is shifted by one index (SO-2
``connect_gain_feedback_populations``).  Where push and pull balance is the lead of the
goal bump over the state bump: the next step of the joint.

Neuron and weights are those of the SO-2 repo unless :class:`GoalRingSettings` says otherwise.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np

from ..mapping import RingMap
from .populations import Population
from .ring import RING_NEURON, JointRing, build_joint_ring

#: ``iaf_psc_alpha`` parameters of a gain neuron (SO-2 ``neuron_params.json``, "gain")
GAIN_NEURON: Dict[str, float] = {
    "V_m": 0.0, "E_L": 0.0, "V_th": 1.35, "V_reset": 0.0, "tau_syn_ex": 27.0, "tau_syn_in": 50.0,
    "I_e": 0.0, "tau_m": 7.04, "C_m": 3960000.0, "t_ref": 0.0,
}


@dataclass(frozen=True)
class GoalRingSettings:
    ring_to_gain: float = 450000.0          # ring neuron i -> gain neuron i of both pairs (SO-2)
    #: share pair: current (pA) into the side a half of the share points to, at a full half
    share_drive_pa: float = 300000.0
    #: share pair: a constant current into both sides (0: silent without a share)
    share_bias_pa: float = 0.0
    #: share pair: True: the other half is taken away from each side (push-pull about the resting rate)
    push_pull: bool = False
    share_feedback: float = -0.6            # share pair -> goal ring, shifted by one index (SO-2 goal pair)
    body_exc: float = 2.0e4                 # state ring -> body pair, from the side the bump should move to (SO-2)
    body_inh: float = 6.0e4
    body_window: int = 10
    body_feedback: float = -3.0             # body pair -> goal ring (SO-2 body pair)
    #: the goal bump is written again at the state bump when it is further than this from it, ring indices
    rewrite_beyond: int = 14

    def describe(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class GoalRing:
    ring: JointRing
    share_left: Any
    share_right: Any
    body_left: Any
    body_right: Any
    recorders: Tuple[Any, Any, Any, Any]
    settings: Any

    def gain_counts(self, backend: Any) -> np.ndarray:
        """Spikes of the gain populations since the last call: share left, share right (and body left, body right)."""

        out = []
        for recorder in self.recorders:
            out.append(float(backend.GetStatus(recorder, "n_events")[0]))
            backend.SetStatus(recorder, {"n_events": 0})
        return np.array(out)

    @property
    def neurons(self) -> int:
        return self.ring.size * (1 + len(self.recorders))


def connect_shifted(backend: Any, pre: Any, post: Any, shift: int, weight: float, wrap: bool) -> None:
    """``pre[i] -> post[i + shift]``; around the seam only when ``wrap``."""

    size = len(pre)
    shift = int(shift)
    if shift == 0:
        backend.Connect(pre, post, "one_to_one", {"weight": float(weight)})
        return
    s = abs(shift)
    if shift > 0:
        backend.Connect(pre[0:size - s], post[s:size], "one_to_one", {"weight": float(weight)})
        if wrap:
            backend.Connect(pre[size - s:size], post[0:s], "one_to_one", {"weight": float(weight)})
    else:
        backend.Connect(pre[s:size], post[0:size - s], "one_to_one", {"weight": float(weight)})
        if wrap:
            backend.Connect(pre[0:s], post[size - s:size], "one_to_one", {"weight": float(weight)})


def decoded_current(backend: Any, source: Population, decoders: np.ndarray, target: Any, pa_per_unit: float) -> None:
    """Hand ``pa_per_unit`` x (decoded value of ``source``) to every neuron of ``target`` as a current."""

    per_hz = float(pa_per_unit) * np.asarray(decoders, dtype=float)[:, 0]
    ex = np.e * GAIN_NEURON["tau_syn_ex"] * 1e-3           # an excitatory and an inhibitory spike carry different charge
    inh = np.e * GAIN_NEURON["tau_syn_in"] * 1e-3
    weights = np.where(per_hz >= 0.0, per_hz / ex, per_hz / inh)
    backend.Connect(source.neurons, target, {"rule": "all_to_all"},
                    {"weight": np.ascontiguousarray(np.tile(weights[None, :], (len(target), 1))), "delay": 1.0})


def build_goal_ring(backend: Any, name: str, mapping: RingMap, state: JointRing,
                    plus: Tuple[Population, np.ndarray], minus: Tuple[Population, np.ndarray],
                    settings: Optional[GoalRingSettings] = None) -> GoalRing:
    """``plus`` / ``minus``: the two halves of the share (population, readout), each between 0 and 1.
    The plus half moves the bump up the ring (to larger joint angles)."""

    s = settings or GoalRingSettings()
    size = mapping.size
    wrap = bool(mapping.circular)
    ring = build_joint_ring(backend, name, mapping, mode="corrective", dead_band=int(s.rewrite_beyond))
    pops, recorders = [], []
    for _ in range(4):
        neurons = backend.Create("iaf_psc_alpha", size, params=dict(GAIN_NEURON))
        backend.Connect(ring.neurons, neurons, "one_to_one", {"weight": float(s.ring_to_gain)})
        recorder = backend.Create("spike_recorder")
        backend.Connect(neurons, recorder)
        pops.append(neurons)
        recorders.append(recorder)
    share_left, share_right, body_left, body_right = pops

    if s.share_bias_pa:
        backend.SetStatus(share_left, {"I_e": float(s.share_bias_pa)})
        backend.SetStatus(share_right, {"I_e": float(s.share_bias_pa)})
    decoded_current(backend, plus[0], plus[1], share_right, s.share_drive_pa)
    decoded_current(backend, minus[0], minus[1], share_left, s.share_drive_pa)
    if s.push_pull:
        decoded_current(backend, plus[0], plus[1], share_left, -s.share_drive_pa)
        decoded_current(backend, minus[0], minus[1], share_right, -s.share_drive_pa)

    # the body pair: the side whose direction leads to the state bump is excited, the other inhibited
    for shift in range(1, int(s.body_window) + 1):
        connect_shifted(backend, state.neurons, body_right, -shift, float(s.body_exc), wrap)      # state above -> move up
        connect_shifted(backend, state.neurons, body_right, +shift, -abs(float(s.body_inh)), wrap)
        connect_shifted(backend, state.neurons, body_left, +shift, float(s.body_exc), wrap)       # state below -> move down
        connect_shifted(backend, state.neurons, body_left, -shift, -abs(float(s.body_inh)), wrap)

    # feedback shifted by one index: the left populations push the bump down the ring, the right ones up
    for left, right, weight in ((share_left, share_right, s.share_feedback), (body_left, body_right, s.body_feedback)):
        connect_shifted(backend, left, ring.neurons, +1, float(weight), wrap)
        connect_shifted(backend, right, ring.neurons, -1, float(weight), wrap)
    return GoalRing(ring, share_left, share_right, body_left, body_right, tuple(recorders), s)


@dataclass(frozen=True)
class WrittenRingSettings:
    """The goal ring without an attractor (:func:`build_written_ring`)."""

    #: state ring neuron i -> gain neuron i of both sides: below what makes a gain neuron fire on its own
    gate_weight: float = 137000.0
    #: a constant current into both sides: with the gate, both fire at a resting rate under the state bump
    rest_pa: float = 304000.0
    #: current (pA) at a full half of the share: added to the side it points to, taken from the other
    share_drive_pa: float = 228000.0
    #: ring indices by which each side writes beside the state bump: the lead at a full share
    shift: int = 8
    write_weight: float = 84000.0           # gain neuron -> goal ring neuron
    ring_bias_pa: float = 0.0               # constant current into the goal ring's neurons (they fire from 450000 pA on)

    def describe(self) -> Dict[str, Any]:
        return asdict(self)


def build_written_ring(backend: Any, name: str, mapping: RingMap, state: JointRing,
                       plus: Tuple[Population, np.ndarray], minus: Tuple[Population, np.ndarray],
                       settings: Optional[WrittenRingSettings] = None) -> GoalRing:
    """The goal ring written from the state ring through one gain pair, with no attractor of its own.

    Both sides of the pair are gated by the state bump.  The right side writes the bump ``shift``
    indices up the ring, the left side as far down.  At rest both fire alike and the goal ring's
    activity is centred on the state bump; a share moves the two rates apart (push-pull), and the
    centre of the goal ring's activity moves with it, up to ``shift`` indices.
    """

    s = settings or WrittenRingSettings()
    size = mapping.size
    wrap = bool(mapping.circular)
    params = dict(RING_NEURON, I_e=float(s.ring_bias_pa))
    neurons = backend.Create("iaf_psc_alpha", size, params=params)
    recorder = backend.Create("spike_recorder")
    backend.Connect(neurons, recorder)
    ring = JointRing(name, mapping, neurons, None, recorder, int(np.min(backend.GetStatus(neurons, "global_id"))), mode="continuous")
    sides, recorders = [], []
    for _ in range(2):
        gain = backend.Create("iaf_psc_alpha", size, params=dict(GAIN_NEURON, I_e=float(s.rest_pa)))
        backend.Connect(state.neurons, gain, "one_to_one", {"weight": float(s.gate_weight)})
        gain_recorder = backend.Create("spike_recorder")
        backend.Connect(gain, gain_recorder)
        sides.append(gain)
        recorders.append(gain_recorder)
    left, right = sides
    decoded_current(backend, plus[0], plus[1], right, s.share_drive_pa)
    decoded_current(backend, minus[0], minus[1], right, -s.share_drive_pa)
    decoded_current(backend, minus[0], minus[1], left, s.share_drive_pa)
    decoded_current(backend, plus[0], plus[1], left, -s.share_drive_pa)
    connect_shifted(backend, right, neurons, +int(s.shift), float(s.write_weight), wrap)
    connect_shifted(backend, left, neurons, -int(s.shift), float(s.write_weight), wrap)
    return GoalRing(ring, left, right, None, None, tuple(recorders), s)


def bump_spread(counts: np.ndarray, index: float) -> float:
    """Share of a ring's spikes further than 20 indices from the centre of its activity (two bumps: large)."""

    total = float(np.sum(counts))
    if total <= 0 or not np.isfinite(index):
        return float("nan")
    size = counts.size
    distance = np.abs((np.arange(size) - index + size / 2.0) % size - size / 2.0)
    return float(np.sum(counts[distance > 20]) / total)


__all__ = ["GAIN_NEURON", "GoalRing", "GoalRingSettings", "WrittenRingSettings", "build_goal_ring", "build_written_ring", "bump_spread", "connect_shifted", "decoded_current"]

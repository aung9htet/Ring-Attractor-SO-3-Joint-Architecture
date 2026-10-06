"""The network as firing rates, without spikes (NumPy only).

It has the same populations, the same tuning and the same wiring as the NEST
network; each neuron is replaced by its rate for a constant input and the
readouts are fitted to those rates.  What it leaves out: spikes (so no ripple,
no delay), the dynamics of the ring attractor (the ring simply holds the
nearest index) and the output rings (the three angles are taken from the
decoded matrix).  It is used to choose population sizes, as a fast test of the
wiring, and as the level the spiking network is compared with.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

from ..design import Design
from ..kinematics import lift_pitch_yaw
from ..mapping import RingMap
from ..wiring import Entry, Node, Product, Wiring


@dataclass
class RateReadout:
    values: Dict[Node, float]
    tool: np.ndarray                 # raw 3 x 3 readout of the end effector
    links: List[np.ndarray]
    held: np.ndarray                 # the joint angles the rings hold
    lift_pitch_yaw: tuple


class RateNetwork:
    def __init__(self, wiring: Wiring, design: Optional[Design] = None,
                 decoders: Optional[Dict[str, np.ndarray]] = None, ring_size: Optional[int] = None) -> None:
        self.wiring = wiring
        self.design = design or Design()
        self.decoders = decoders or self.design.analytic_decoders()
        size = int(ring_size or self.design.sizes.ring)
        self.maps = [RingMap.for_joint(joint, size) for joint in wiring.chain.joints]

    def held_angles(self, q: Sequence[float], fractional: bool = True) -> np.ndarray:
        """The angles the rings hold: the angles themselves, or those of the nearest ring index."""

        return np.array([m.held_angle(angle, fractional) for m, angle in zip(self.maps, np.asarray(q, dtype=float).reshape(-1))])

    def _read(self, kind: str, x: np.ndarray, noise: float, rng: Optional[np.random.Generator]) -> np.ndarray:
        tuning = self.design.tuning(kind)
        rates = tuning.rates(x)[0]
        if noise and rng is not None:
            rates = np.maximum(rates + rng.normal(0.0, noise * float(np.max(tuning.model.rate(tuning.gain + tuning.bias))), rates.shape), 0.0)
        return rates @ self.decoders[kind]

    def decode(self, q: Sequence[float], amplitude: float = 1.0, noise: float = 0.0,
               rng: Optional[np.random.Generator] = None, fractional: bool = True) -> RateReadout:
        """``amplitude``: strength of the ring bumps relative to the calibrated one.
        ``noise``: Gaussian noise on every rate, as a fraction of the population's largest rate.
        ``fractional``: the encoder writes between ring indices (False: on the nearest index)."""

        angles = np.asarray(q, dtype=float).reshape(-1)
        held = self.held_angles(angles, fractional)

        def feature(k: int, _angle: float):
            pair = float(amplitude) * np.array([[np.cos(held[k]), np.sin(held[k])]])
            c, s = self._read("phasor", pair, noise, rng)
            return float(c), float(s)

        def product(a: float, b: float, _step: Product) -> float:
            return float(self._read("product", np.array([[a, b]]), noise, rng)[0])

        def entry(x: float, _step: Entry) -> float:
            return float(self._read("scalar", np.array([[x]]), noise, rng)[0])

        values = self.wiring.evaluate(angles, feature, product, entry)
        tool = self.wiring.tool_matrix(values)
        return RateReadout(values, tool, self.wiring.link_matrices(values), held, lift_pitch_yaw(tool))


__all__ = ["RateNetwork", "RateReadout"]

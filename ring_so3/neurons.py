"""Populations that hold a value: tuning of the neurons and least-squares readouts.

A population of ``n`` leaky integrate-and-fire neurons holds a value ``x`` (one
number, or a pair) in its firing rates.  Neuron ``i`` gets the current::

    J_i = gain_i · (encoder_i · x) / radius + bias_i

so each neuron responds to one direction of ``x`` (its encoder) above its own
threshold (its intercept) and reaches its own maximum rate at the edge of the
range.  A value, or a function of it, is read back as a weighted sum of the
rates (:func:`solve_decoders`: least squares with a noise term).  This is the
population code of Eliasmith (2005), which the paper cites for its Fourier
readout.

Three kinds are used (:func:`make_tuning`):

* ``scalar``: one number in [-1, 1]; encoders +1 and -1 (the push-pull halves of the paper);
* ``phasor``: a pair (cos, sin); encoders all around the circle;
* ``product``: a pair (a, b), encoders on the four diagonals, so every neuron responds
  to ``a + b`` or ``a - b``; the product is read out through ``ab = ((a+b)² - (a-b)²) / 4``.

The neuron is NEST's ``iaf_psc_alpha`` (:class:`NeuronModel`); its rate for a
constant current is the closed form used by :meth:`NeuronModel.rate`, which is
what the rate copy of the network runs on.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, Optional, Tuple

import numpy as np

KINDS = ("scalar", "phasor", "product")


@dataclass(frozen=True)
class NeuronModel:
    """``iaf_psc_alpha`` parameters of the populations between the rings (pF, ms, mV, pA)."""

    C_m: float = 250.0
    tau_m: float = 20.0
    t_ref: float = 2.0
    V_th: float = 15.0          # above the resting and reset potential, which are 0
    tau_syn: float = 5.0

    @property
    def rheobase(self) -> float:
        """Smallest constant current that makes the neuron fire, pA."""

        return self.C_m * self.V_th / self.tau_m

    def rate(self, current: np.ndarray) -> np.ndarray:
        """Firing rate (Hz) for a constant input current (pA)."""

        j = np.asarray(current, dtype=float)
        out = np.zeros_like(j)
        above = j > self.rheobase * (1.0 + 1e-12)
        out[above] = 1000.0 / (self.t_ref + self.tau_m * np.log(j[above] / (j[above] - self.rheobase)))
        return out

    def current(self, rate_hz: np.ndarray) -> np.ndarray:
        """Constant current (pA) that gives a firing rate (Hz); inverse of :meth:`rate` above 0."""

        r = np.asarray(rate_hz, dtype=float)
        period = 1000.0 / r - self.t_ref
        return self.rheobase / (1.0 - np.exp(-period / self.tau_m))

    @property
    def charge(self) -> float:
        """Mean current (pA) one input firing at 1 Hz delivers per pA of synaptic weight.

        NEST's alpha current is ``w·(e/τ)·t·exp(-t/τ)`` (peak ``w``), whose integral is ``w·e·τ``.
        """

        return float(np.e * self.tau_syn * 1e-3)

    def nest_params(self) -> Dict[str, float]:
        return {"C_m": self.C_m, "tau_m": self.tau_m, "t_ref": self.t_ref, "E_L": 0.0, "V_reset": 0.0, "V_m": 0.0,
                "V_th": self.V_th, "tau_syn_ex": self.tau_syn, "tau_syn_in": self.tau_syn, "I_e": 0.0}


@dataclass
class Tuning:
    kind: str
    n: int
    dims: int
    radius: float
    encoders: np.ndarray        # (n, dims)
    gain: np.ndarray            # (n,) pA per unit of encoder · x / radius
    bias: np.ndarray            # (n,) pA
    model: NeuronModel = field(default_factory=NeuronModel)
    seed: int = 0

    def currents(self, x: np.ndarray) -> np.ndarray:
        values = np.atleast_2d(np.asarray(x, dtype=float))
        if values.shape[1] != self.dims:
            values = values.reshape(-1, self.dims)
        return self.gain[None, :] * (values @ self.encoders.T) / self.radius + self.bias[None, :]

    def rates(self, x: np.ndarray) -> np.ndarray:
        """Rates (samples x neurons) for constant inputs: the rate copy's neuron."""

        return self.model.rate(self.currents(x))

    def input_weights(self, transform: np.ndarray) -> np.ndarray:
        """``(n, k)`` currents per unit of each of ``k`` decoded values that enter through ``transform`` (dims x k)."""

        return self.gain[:, None] * (self.encoders @ np.asarray(transform, dtype=float)) / self.radius


def make_tuning(kind: str, n: int, seed: int, model: Optional[NeuronModel] = None,
                intercepts: Tuple[float, float] = (-0.9, 0.9), max_rates: Tuple[float, float] = (100.0, 200.0),
                radius: Optional[float] = None) -> Tuning:
    """A population of ``n`` neurons with thresholds and maximum rates drawn from ``seed``."""

    if kind not in KINDS:
        raise ValueError("kind must be one of %r" % (KINDS,))
    model = model or NeuronModel()
    rng = np.random.default_rng(int(seed))
    n = int(n)
    if kind == "scalar":
        encoders = np.where(np.arange(n) % 2 == 0, 1.0, -1.0)[:, None]
        default_radius = 1.0
    elif kind == "phasor":
        angles = 2.0 * np.pi * (np.arange(n) + rng.uniform(0.0, 1.0)) / n
        encoders = np.stack([np.cos(angles), np.sin(angles)], axis=1)
        default_radius = 1.3            # room for a pair somewhat longer than 1
    else:
        signs = np.array([[1.0, 1.0], [1.0, -1.0], [-1.0, 1.0], [-1.0, -1.0]])
        encoders = signs[np.arange(n) % 4] / 2.0        # encoder · (a, b) stays in [-1, 1] on the unit square
        default_radius = 1.0
    cut = rng.uniform(intercepts[0], intercepts[1], n)
    top = rng.uniform(max_rates[0], max_rates[1], n)
    j_top = model.current(top)
    gain = (j_top - model.rheobase) / (1.0 - cut)
    bias = model.rheobase - gain * cut
    return Tuning(kind, n, encoders.shape[1], float(radius or default_radius), encoders, gain, bias, model, int(seed))


def solve_decoders(rates: np.ndarray, targets: np.ndarray, regularisation: float = 0.1) -> np.ndarray:
    """Readout weights ``D`` (neurons x outputs) with ``rates @ D ≈ targets``.

    Least squares with a ridge term that stands for noise on the rates of
    ``regularisation`` times the largest rate, so no weight relies on a
    difference of rates smaller than that noise.
    """

    a = np.asarray(rates, dtype=float)
    y = np.asarray(targets, dtype=float)
    if y.ndim == 1:
        y = y[:, None]
    sigma = float(regularisation) * float(np.max(a)) if a.size else 0.0
    gram = a.T @ a + (sigma ** 2) * a.shape[0] * np.eye(a.shape[1])
    return np.linalg.solve(gram, a.T @ y)


# -- what each kind is asked to give back -------------------------------------
def scalar_samples(points: int = 121, span: float = 1.1) -> Tuple[np.ndarray, np.ndarray]:
    """Inputs a little beyond [-1, 1]; the readout returns the value, held at +-1 outside."""

    x = np.linspace(-span, span, int(points))[:, None]
    return x, np.clip(x, -1.0, 1.0)


def product_samples(points: int = 33, span: float = 1.1) -> Tuple[np.ndarray, np.ndarray]:
    axis = np.linspace(-span, span, int(points))
    a, b = [g.ravel() for g in np.meshgrid(axis, axis, indexing="ij")]
    return np.stack([a, b], axis=1), (np.clip(a, -1.0, 1.0) * np.clip(b, -1.0, 1.0))[:, None]


def phasor_samples(angles: int = 144, lengths: Tuple[float, ...] = (0.94, 0.97, 1.0, 1.03, 1.06)) -> Tuple[np.ndarray, np.ndarray]:
    """Pairs a few percent longer and shorter than 1 (a joint ring's bump varies by about 2% in
    strength, measured in NEST); the readout returns the pair with length 1."""

    theta = 2.0 * np.pi * np.arange(int(angles)) / int(angles)
    unit = np.stack([np.cos(theta), np.sin(theta)], axis=1)
    x = np.concatenate([length * unit for length in lengths])
    return x, np.concatenate([unit for _ in lengths])


SAMPLES: Dict[str, Callable[[], Tuple[np.ndarray, np.ndarray]]] = {
    "scalar": scalar_samples, "product": product_samples, "phasor": phasor_samples,
}


def analytic_decoders(tuning: Tuning, regularisation: float = 0.1) -> np.ndarray:
    """Readout weights from the closed-form rates (what the rate copy uses)."""

    x, y = SAMPLES[tuning.kind]()
    return solve_decoders(tuning.rates(x), y, regularisation)


__all__ = ["KINDS", "NeuronModel", "SAMPLES", "Tuning", "analytic_decoders", "make_tuning", "phasor_samples",
           "product_samples", "scalar_samples", "solve_decoders"]

"""NEST building blocks between the rings: populations that hold values, and readout cells."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from ..neurons import NeuronModel, Tuning

#: readout cell: a leaky integrator that never fires; its membrane potential is the weighted,
#: smoothed sum of the spikes it receives
READOUT = {"tau_m": 50.0, "C_m": 250.0, "tau_syn": 5.0, "mv_per_unit": 10.0}


@dataclass
class Population:
    name: str
    neurons: Any
    n: int
    first_id: int
    tuning: Optional[Tuning] = None
    recorder: Any = None
    #: charge one input spike delivers per pA of weight (pA s), for weights into this population
    charge: float = 0.0

    def counts(self, backend: Any) -> np.ndarray:
        """Spikes of every neuron since the last call (needs ``record=True``; not inside Prepare/Run)."""

        events = backend.GetStatus(self.recorder, "events")[0]
        senders = np.asarray(events["senders"], dtype=int)
        out = np.bincount(senders - self.first_id, minlength=self.n).astype(float)[: self.n]
        backend.SetStatus(self.recorder, {"n_events": 0})
        return out


def _first_id(backend: Any, neurons: Any) -> int:
    return int(np.min(backend.GetStatus(neurons, "global_id")))


def _record(backend: Any, neurons: Any) -> Any:
    recorder = backend.Create("spike_recorder")
    backend.Connect(neurons, recorder)
    return recorder


def create_population(backend: Any, tuning: Tuning, name: str, seed: int = 0, record: bool = False) -> Population:
    """``tuning.n`` ``iaf_psc_alpha`` neurons with the population's bias currents; start potentials spread out."""

    model: NeuronModel = tuning.model
    neurons = backend.Create("iaf_psc_alpha", tuning.n, params=model.nest_params())
    start = np.random.default_rng([int(seed), tuning.seed]).uniform(0.0, model.V_th, tuning.n)
    backend.SetStatus(neurons, [{"I_e": float(b), "V_m": float(v)} for b, v in zip(tuning.bias, start)])
    return Population(name, neurons, tuning.n, _first_id(backend, neurons), tuning,
                      _record(backend, neurons) if record else None, model.charge)


def set_value(backend: Any, population: Population, x: Sequence[float]) -> None:
    """Drive a population with a constant value directly (calibration sources only)."""

    current = population.tuning.currents(np.asarray(x, dtype=float).reshape(1, -1))[0]
    backend.SetStatus(population.neurons, [{"I_e": float(c)} for c in current])


def connect_weights(backend: Any, pre: Any, post: Any, weights_post_by_pre: np.ndarray, delay: float = 1.0) -> None:
    """All-to-all with one weight per pair, in pA; ``weights_post_by_pre`` has shape (post, pre)."""

    matrix = np.ascontiguousarray(np.asarray(weights_post_by_pre, dtype=float))
    backend.Connect(pre, post, {"rule": "all_to_all"}, {"weight": matrix, "delay": float(delay)})


def decoded_weights(decoders: np.ndarray, transform: np.ndarray, post: Population) -> np.ndarray:
    """Weights (post, pre) that hand ``transform @ decoded values`` of the sender to ``post``."""

    return post.tuning.input_weights(transform) @ np.asarray(decoders, dtype=float).T / post.charge


@dataclass
class ReadoutCells:
    """Cells whose membrane potential is a weighted sum of spikes, smoothed with ``tau_m``."""

    neurons: Any
    n: int
    settings: Dict[str, float] = field(default_factory=lambda: dict(READOUT))
    offsets: np.ndarray = field(default_factory=lambda: np.zeros(0))

    @property
    def pa_per_unit(self) -> float:
        """Input current that holds the potential at one unit of value."""

        return self.settings["mv_per_unit"] * self.settings["C_m"] / self.settings["tau_m"]

    @property
    def charge(self) -> float:
        return float(np.e * self.settings["tau_syn"] * 1e-3)

    def connect(self, backend: Any, pre: Population, decoders: np.ndarray, cells: Optional[Sequence[int]] = None) -> None:
        """``decoders``: (pre neurons, cells listed in ``cells``), in value per Hz."""

        d = np.asarray(decoders, dtype=float)
        targets = self.neurons if cells is None else self.neurons[list(cells)] if len(cells) > 1 else self.neurons[int(cells[0])]
        connect_weights(backend, pre.neurons, targets, (d * self.pa_per_unit / self.charge).T)

    def set_constants(self, backend: Any, constants: Sequence[float]) -> None:
        """A constant part of each cell's value, as a bias current."""

        values = np.asarray(constants, dtype=float)
        backend.SetStatus(self.neurons, [{"I_e": float(v * self.pa_per_unit)} for v in values])

    def values(self, backend: Any) -> np.ndarray:
        return np.asarray(backend.GetStatus(self.neurons, "V_m"), dtype=float) / self.settings["mv_per_unit"]


def create_readout_cells(backend: Any, n: int, settings: Optional[Dict[str, float]] = None) -> ReadoutCells:
    s = dict(READOUT, **(settings or {}))
    params = {"C_m": s["C_m"], "tau_m": s["tau_m"], "t_ref": 0.0, "E_L": 0.0, "V_reset": 0.0, "V_m": 0.0,
              "V_th": 1.0e12, "tau_syn_ex": s["tau_syn"], "tau_syn_in": s["tau_syn"], "I_e": 0.0}
    neurons = backend.Create("iaf_psc_alpha", int(n), params=params)
    return ReadoutCells(neurons, int(n), s)


__all__ = ["Population", "READOUT", "ReadoutCells", "connect_weights", "create_population",
           "create_readout_cells", "decoded_weights", "set_value"]

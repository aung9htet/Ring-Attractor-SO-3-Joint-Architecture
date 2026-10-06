"""Readout weights from measured responses in NEST (section 1.3 of the plan, docs/architecture.md).

Every kind of population is measured once, in small rigs that run side by side in one
kernel, and its readout is the least-squares fit to those measured rates:

* ``source``: a scalar population driven by a constant current (only used as the sender
  in the other rigs);
* ``scalar``: a link-orientation population driven by the spikes of a ``source``; fitted to
  give back the value;
* ``product``: a product population driven by the spikes of two sources, on a grid of
  pairs; fitted to give ``a·b``;
* ``norm``: a population that holds a pair, driven by two sources on a grid inside the unit
  disc; fitted to give the pair's length (the pitch ring needs ``cos(pitch)``, which is the
  length of the pair ``(r11, r21)`` of the end-effector matrix);
* ``feature``: the cos / sin population of a joint, driven by a real joint ring whose bump
  is written at many positions; fitted to give ``(cos q, sin q)`` of the written angle.

The fit never sees a test pose: sources take values on a grid, the ring is swept index by
index.  :func:`run` does all of it and returns a :class:`Calibration`, which
``scripts/calibrate.py`` stores in ``artifacts/``.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..design import Design
from ..mapping import RingMap
from ..neurons import NeuronModel, Tuning, make_tuning, solve_decoders
from .populations import Population, connect_weights, create_population, decoded_weights, set_value
from .ring import build_joint_ring

#: raised when what a calibration file holds changes; older files are then refused
FORMAT = 2
SETTLE_MS = 300.0
MEASURE_MS = 1000.0
TICK_MS = 50.0


@dataclass
class Calibration:
    design: Dict[str, Any]
    settings: Dict[str, Any]
    decoders: Dict[str, np.ndarray]
    #: mean rate of every ring neuron with the bump centred on index 0, Hz
    ring_profile: np.ndarray
    fit: Dict[str, Any] = field(default_factory=dict)

    def feature_scale(self, mapping: RingMap) -> float:
        """Factor that makes the cos / sin sums of a held bump a pair of length 1 for this mapping."""

        size = mapping.size
        if self.ring_profile.size != size:
            raise ValueError("calibrated for rings of %d neurons, asked for %d" % (self.ring_profile.size, size))
        offsets = (np.arange(size) + size // 2) % size - size // 2
        return 1.0 / float(abs(np.sum(self.ring_profile * np.exp(1j * offsets * mapping.radians_per_index))))

    def save(self, path: str) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        arrays = {"decoders_" + key: value for key, value in self.decoders.items()}
        np.savez_compressed(path, ring_profile=self.ring_profile, **arrays)
        with open(os.path.splitext(path)[0] + ".json", "w", encoding="utf-8") as stream:
            json.dump({"design": self.design, "settings": self.settings, "fit": self.fit}, stream, indent=1, sort_keys=True)

    @classmethod
    def load(cls, path: str) -> "Calibration":
        data = np.load(path)
        with open(os.path.splitext(path)[0] + ".json", encoding="utf-8") as stream:
            meta = json.load(stream)
        decoders = {key[len("decoders_"):]: data[key] for key in data.files if key.startswith("decoders_")}
        return cls(meta["design"], meta["settings"], decoders, data["ring_profile"], meta.get("fit", {}))


def feature_tuning(design: Design, tau_syn: float) -> Tuning:
    """The cos / sin population: the design's phasor tuning with the slower synapse that reads a ring."""

    model = NeuronModel(design.model.C_m, design.model.tau_m, design.model.t_ref, design.model.V_th, float(tau_syn))
    return make_tuning("phasor", design.sizes.feature, design.feature.seed, model, radius=design.feature.radius)


def _kernel(backend: Any, threads: int, seed: int) -> None:
    backend.ResetKernel()
    backend.set_verbosity("M_ERROR")
    backend.SetKernelStatus({"local_num_threads": int(threads), "rng_seed": int(seed)})


def _rates(backend: Any, populations: Sequence[Population], settle_ms: float, measure_ms: float) -> np.ndarray:
    backend.Simulate(float(settle_ms))
    for population in populations:
        population.counts(backend)
    backend.Simulate(float(measure_ms))
    return np.array([population.counts(backend) for population in populations]) / (measure_ms / 1000.0)


def measure_source(backend: Any, design: Design, values: np.ndarray, threads: int = 4, seed: int = 1,
                   settle_ms: float = SETTLE_MS, measure_ms: float = MEASURE_MS) -> np.ndarray:
    """Rates (values x neurons) of the scalar population driven by constant currents."""

    _kernel(backend, threads, seed)
    populations = []
    for i, x in enumerate(values):
        population = create_population(backend, design.entry, "source%d" % i, seed=i, record=True)
        set_value(backend, population, [x])
        populations.append(population)
    return _rates(backend, populations, settle_ms, measure_ms)


def measure_driven(backend: Any, design: Design, kind: str, inputs: np.ndarray, source_decoders: np.ndarray,
                   threads: int = 4, seed: int = 1, settle_ms: float = SETTLE_MS, measure_ms: float = MEASURE_MS) -> np.ndarray:
    """Rates (inputs x neurons) of a ``scalar``, ``product`` or ``norm`` population driven by sources' spikes.

    ``inputs``: (points, 1) for ``scalar``, (points, 2) for ``product`` and ``norm``.
    """

    tuning = design.tuning(kind)
    _kernel(backend, threads, seed)
    targets = []
    for i, point in enumerate(np.atleast_2d(inputs)):
        target = create_population(backend, tuning, "%s%d" % (kind, i), seed=1000 + i, record=True)
        for d, value in enumerate(point):
            source = create_population(backend, design.entry, "source%d_%d" % (i, d), seed=2000 + 7 * i + d)
            set_value(backend, source, [value])
            transform = np.zeros((tuning.dims, 1))
            transform[d, 0] = 1.0
            connect_weights(backend, source.neurons, target.neurons, decoded_weights(source_decoders, transform, target))
        targets.append(target)
    return _rates(backend, targets, settle_ms, measure_ms)


def measure_ring_profile(backend: Any, size: int, mode: str, threads: int = 4, seed: int = 1,
                         positions: Sequence[int] = (20, 60, 100, 140, 180), ticks: int = 40) -> np.ndarray:
    """Mean rate of every ring neuron around a held bump, rolled so the bump is centred on index 0."""

    profiles = []
    for position in positions:
        _kernel(backend, threads, seed + int(position))
        mapping = RingMap(int(size), -np.pi, np.pi, True)
        ring = build_joint_ring(backend, "ring", mapping, mode=mode)
        collected = []
        for t in range(int(ticks)):
            ring.drive(backend, mapping.angle(position))
            backend.Simulate(TICK_MS)
            counts = ring.read(backend)
            if t >= ticks // 2:
                collected.append(counts / (TICK_MS / 1000.0))
        profiles.append(np.roll(np.mean(collected, axis=0), -int(position)))        # whole positions: the same for every encoder
    return np.mean(profiles, axis=0)


def measure_feature(backend: Any, design: Design, tuning: Tuning, scale: float, positions: Sequence[int], mode: str,
                    size: int, threads: int = 4, seed: int = 1, settle_ticks: int = 6, hold_ticks: int = 14,
                    step: float = 0.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Sweep a bump over a circular ring and record the cos / sin population.

    Returns ``(angles, rates, ring_index_error)``: one row per tick of the hold period.
    ``step``: ring indices the written angle advances per tick (0: held).
    """

    _kernel(backend, threads, seed)
    mapping = RingMap(int(size), -np.pi, np.pi, True)
    ring = build_joint_ring(backend, "ring", mapping, mode=mode)
    population = create_population(backend, tuning, "feature", record=True)
    neuron_angles = mapping.neuron_angles()
    basis = np.stack([np.cos(neuron_angles), np.sin(neuron_angles)], axis=1)
    connect_weights(backend, ring.neurons, population.neurons,
                    tuning.input_weights(np.eye(2)) @ (scale * basis).T / population.charge)
    angles, rates, index_error = [], [], []
    for position in positions:
        for t in range(int(settle_ticks + hold_ticks)):
            target = float(position) + float(step) * t
            ring.drive(backend, mapping.angle(target))
            backend.Simulate(TICK_MS)
            ring.read(backend)
            counts = population.counts(backend)
            if t >= settle_ticks:
                angles.append(mapping.angle(target))
                rates.append(counts / (TICK_MS / 1000.0))
                index_error.append(mapping.index_error(ring.index, target))
    return np.array(angles), np.array(rates), np.array(index_error)


def _rms(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.mean((np.asarray(a) - np.asarray(b)) ** 2)))


def run(backend: Any, design: Optional[Design] = None, ring_size: Optional[int] = None, encoder_mode: str = "fractional",
        feature_tau: float = 15.0, threads: int = 8, seed: int = 1, regularisation: float = 0.02,
        scalar_points: int = 61, product_points: int = 25, log: Callable[[str], None] = print) -> Calibration:
    """Measure every kind of population and fit its readout."""

    design = design or Design()
    size = int(ring_size or design.sizes.ring)
    started = time.time()
    fit: Dict[str, Any] = {}

    x = np.linspace(-1.1, 1.1, int(scalar_points))
    held = np.clip(x, -1.0, 1.0)[:, None]
    source_rates = measure_source(backend, design, x, threads, seed)
    d_source = solve_decoders(source_rates, held, regularisation)
    fit["source_rms"] = _rms(source_rates @ d_source, held)
    log("source: %d values, fit rms %.4f (%.0f s)" % (len(x), fit["source_rms"], time.time() - started))

    scalar_rates = measure_driven(backend, design, "scalar", x[:, None], d_source, threads, seed)
    d_scalar = solve_decoders(scalar_rates, held, regularisation)
    fit["scalar_rms"] = _rms(scalar_rates @ d_scalar, held)
    log("scalar: fit rms %.4f (%.0f s)" % (fit["scalar_rms"], time.time() - started))

    axis = np.linspace(-1.1, 1.1, int(product_points))
    a, b = [g.ravel() for g in np.meshgrid(axis, axis, indexing="ij")]
    pairs = np.stack([a, b], axis=1)
    product_target = (np.clip(a, -1.0, 1.0) * np.clip(b, -1.0, 1.0))[:, None]
    product_rates = measure_driven(backend, design, "product", pairs, d_source, threads, seed)
    d_product = solve_decoders(product_rates, product_target, regularisation)
    fit["product_rms"] = _rms(product_rates @ d_product, product_target)
    log("product: %d pairs, fit rms %.4f (%.0f s)" % (len(pairs), fit["product_rms"], time.time() - started))

    disc = pairs[np.hypot(pairs[:, 0], pairs[:, 1]) <= 1.15]
    norm_target = np.minimum(np.hypot(disc[:, 0], disc[:, 1]), 1.0)[:, None]
    norm_rates = measure_driven(backend, design, "norm", disc, d_source, threads, seed)
    d_norm = solve_decoders(norm_rates, norm_target, regularisation)
    fit["norm_rms"] = _rms(norm_rates @ d_norm, norm_target)
    log("norm: %d pairs, fit rms %.4f (%.0f s)" % (len(disc), fit["norm_rms"], time.time() - started))

    profile = measure_ring_profile(backend, size, encoder_mode, threads, seed)
    scale = 1.0 / float(abs(np.sum(profile * np.exp(1j * ((np.arange(size) + size // 2) % size - size // 2) * 2.0 * np.pi / size))))
    tuning = feature_tuning(design, feature_tau)
    # whole indices for the encoders that write on the nearest index; in between as well for the fractional one
    positions = list(np.arange(0.0, size, 1.37 if encoder_mode == "fractional" else max(size // 100, 1)))
    angles, feature_rates, index_error = measure_feature(backend, design, tuning, scale, positions, encoder_mode, size, threads, seed)
    targets = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    d_feature = solve_decoders(feature_rates, targets, regularisation)
    fit["feature_rms"] = _rms(feature_rates @ d_feature, targets)
    fit["ring_total_rate_hz"] = float(profile.sum())
    fit["ring_active_neurons"] = int(np.sum(profile > 0.05 * profile.max()))
    fit["ring_index_error_mean"] = float(np.mean(index_error))
    fit["ring_index_error_sd"] = float(np.std(index_error))
    log("feature: %d ring positions, fit rms %.4f per tick; ring holds the written index to %+.2f +- %.2f (%.0f s)"
        % (len(positions), fit["feature_rms"], fit["ring_index_error_mean"], fit["ring_index_error_sd"], time.time() - started))

    settings = {"format": FORMAT, "ring_size": size, "encoder_mode": encoder_mode, "feature_tau": float(feature_tau), "seed": int(seed),
                "regularisation": float(regularisation), "scalar_points": int(scalar_points), "product_points": int(product_points),
                "settle_ms": SETTLE_MS, "measure_ms": MEASURE_MS, "seconds": round(time.time() - started, 1)}
    decoders = {"source": d_source, "scalar": d_scalar, "norm": d_norm, "product": d_product, "feature": d_feature}
    return Calibration(design.describe(), settings, decoders, profile, fit)


ARTIFACTS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "artifacts"))


def artifact_name(design: Design, ring_size: int, encoder_mode: str) -> str:
    s = design.sizes
    return "calibration_N%d_f%d_p%d_e%d_seed%d_%s.npz" % (ring_size, s.feature, s.product, s.entry, design.seed, encoder_mode)


def load(design: Optional[Design] = None, ring_size: Optional[int] = None, encoder_mode: str = "fractional",
         directory: Optional[str] = None) -> Calibration:
    """The stored calibration of a design (``scripts/calibrate.py`` writes it)."""

    design = design or Design()
    path = os.path.join(directory or ARTIFACTS, artifact_name(design, int(ring_size or design.sizes.ring), encoder_mode))
    if not os.path.exists(path):
        raise FileNotFoundError("no calibration %s; run scripts/calibrate.py with these sizes and --encoder %s"
                                % (os.path.relpath(path), encoder_mode))
    calibration = Calibration.load(path)
    if calibration.settings.get("format") != FORMAT:
        raise FileNotFoundError("calibration %s was written by an older version; run scripts/calibrate.py again"
                                % os.path.relpath(path))
    return calibration


__all__ = ["ARTIFACTS", "Calibration", "FORMAT", "artifact_name", "feature_tuning", "load", "measure_driven", "measure_feature", "measure_ring_profile",
           "measure_source", "run"]

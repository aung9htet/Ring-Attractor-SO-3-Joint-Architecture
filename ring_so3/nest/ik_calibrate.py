"""Readouts the inverse-kinematics layer needs beyond those of the forward network.

Both are the forward network's scalar population, driven by the spikes of a source
population over a wider range than the forward calibration covers (the comparison can ask
for more than a population holds, and then the readout has to stay at the bound):

* ``half``: read out as the positive part of the value, held at 1 above 1.  A joint's share
  is held by two of them, one per direction;
* ``bounded``: read out as the value, held at -1 and +1 outside.

The forward calibration is only read.  The result is stored under its own name
(``ik_`` + the forward calibration's name), so no forward calibration file is written again.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional

import numpy as np

from ..design import Design
from ..neurons import solve_decoders
from . import calibrate
from .calibrate import ARTIFACTS, Calibration

FORMAT = 1
#: both readouts are fitted for inputs up to this size (twice what a population holds, and a little more)
WIDEST = 2.2


@dataclass
class IkCalibration:
    settings: Dict[str, Any]
    decoders: Dict[str, np.ndarray]
    fit: Dict[str, Any] = field(default_factory=dict)

    def save(self, path: str) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        np.savez_compressed(path, **{"decoders_" + key: value for key, value in self.decoders.items()})
        with open(os.path.splitext(path)[0] + ".json", "w", encoding="utf-8") as stream:
            json.dump({"settings": self.settings, "fit": self.fit}, stream, indent=1, sort_keys=True)

    @classmethod
    def load(cls, path: str) -> "IkCalibration":
        data = np.load(path)
        with open(os.path.splitext(path)[0] + ".json", encoding="utf-8") as stream:
            meta = json.load(stream)
        return cls(meta["settings"], {key[len("decoders_"):]: data[key] for key in data.files}, meta.get("fit", {}))


def artifact_name(design: Design, ring_size: int, encoder_mode: str) -> str:
    return "ik_" + calibrate.artifact_name(design, ring_size, encoder_mode)


def run(backend: Any, base: Calibration, design: Optional[Design] = None, threads: int = 8, seed: int = 1,
        points: int = 133, log: Callable[[str], None] = print) -> IkCalibration:
    design = design or Design()
    started = time.time()
    # the whole range, and the small values once more: near nothing is where a share has to be right
    x = np.concatenate([np.linspace(-WIDEST, WIDEST, int(points)), np.linspace(-0.3, 0.3, int(points) // 2)])
    # a source holds values up to 1: a larger value is the sum of two sources
    inputs = np.stack([np.clip(x, -1.1, 1.1), x - np.clip(x, -1.1, 1.1)], axis=1)
    rates = _measure_sum(backend, design, inputs, base.decoders["source"], threads, seed)
    decoders, fit = {}, {}
    for name, target in (("half", np.clip(x, 0.0, 1.0)[:, None]), ("bounded", np.clip(x, -1.0, 1.0)[:, None])):
        decoders[name] = solve_decoders(rates, target, float(base.settings["regularisation"]))
        fit[name + "_rms"] = float(np.sqrt(np.mean((rates @ decoders[name] - target) ** 2)))
        small = np.abs(x) <= 0.3
        fit[name + "_small_rms"] = float(np.sqrt(np.mean((rates[small] @ decoders[name] - target[small]) ** 2)))
        log("%s: %d values, fit rms %.4f (%.0f s)" % (name, len(x), fit[name + "_rms"], time.time() - started))
    settings = {"format": FORMAT, "seed": int(seed), "points": int(points), "widest": WIDEST,
                "seconds": round(time.time() - started, 1)}
    return IkCalibration(settings, decoders, fit)


def _measure_sum(backend: Any, design: Design, inputs: np.ndarray, source_decoders: np.ndarray, threads: int, seed: int) -> np.ndarray:
    """Rates (points x neurons) of a scalar population that gets the sum of two sources' values."""

    from .populations import connect_weights, create_population, decoded_weights, set_value

    calibrate._kernel(backend, threads, seed)
    targets = []
    for i, point in enumerate(inputs):
        target = create_population(backend, design.entry, "sum%d" % i, seed=1000 + i, record=True)
        for d, value in enumerate(point):
            source = create_population(backend, design.entry, "source%d_%d" % (i, d), seed=2000 + 7 * i + d)
            set_value(backend, source, [value])
            connect_weights(backend, source.neurons, target.neurons, decoded_weights(source_decoders, np.array([[1.0]]), target))
        targets.append(target)
    return calibrate._rates(backend, targets, calibrate.SETTLE_MS, calibrate.MEASURE_MS)


def load(design: Optional[Design] = None, ring_size: Optional[int] = None, encoder_mode: str = "fractional",
         directory: Optional[str] = None) -> IkCalibration:
    design = design or Design()
    path = os.path.join(directory or ARTIFACTS, artifact_name(design, int(ring_size or design.sizes.ring), encoder_mode))
    if not os.path.exists(path):
        raise FileNotFoundError("no IK calibration %s; run scripts/calibrate_ik.py" % os.path.relpath(path))
    calibration = IkCalibration.load(path)
    if calibration.settings.get("format") != FORMAT:
        raise FileNotFoundError("IK calibration %s was written by an older version; run scripts/calibrate_ik.py again"
                                % os.path.relpath(path))
    return calibration


__all__ = ["FORMAT", "IkCalibration", "artifact_name", "load", "run"]

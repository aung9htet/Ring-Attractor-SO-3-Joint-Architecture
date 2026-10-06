"""Sizes and tuning of the populations: one design is shared by the rate copy and the NEST network."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Dict, Optional

import numpy as np

from .neurons import NeuronModel, Tuning, analytic_decoders, make_tuning


@dataclass(frozen=True)
class Sizes:
    """Neurons per population (docs/architecture.md says how they were chosen)."""

    ring: int = 200             # joint ring (the SO-2 ring attractor)
    feature: int = 120          # cos and sin of one joint
    product: int = 200          # one product
    entry: int = 60             # one entry of a link's orientation
    output_ring: int = 100      # lift, pitch, yaw
    norm: int = 120             # cos(pitch): the length of the pair (r11, r21), for the pitch ring


@dataclass
class Design:
    sizes: Sizes = field(default_factory=Sizes)
    model: NeuronModel = field(default_factory=NeuronModel)
    seed: int = 1
    regularisation: float = 0.02

    def __post_init__(self) -> None:
        base = 1000 * int(self.seed)
        self.feature: Tuning = make_tuning("phasor", self.sizes.feature, base + 1, self.model)
        self.product: Tuning = make_tuning("product", self.sizes.product, base + 2, self.model)
        self.entry: Tuning = make_tuning("scalar", self.sizes.entry, base + 3, self.model)
        self.norm: Tuning = make_tuning("phasor", self.sizes.norm, base + 4, self.model, radius=1.2)

    def tuning(self, kind: str) -> Tuning:
        return {"phasor": self.feature, "product": self.product, "scalar": self.entry, "norm": self.norm}[kind]

    def analytic_decoders(self) -> Dict[str, np.ndarray]:
        """Readouts from the closed-form rates, keyed by kind (the rate copy's weights)."""

        return {kind: analytic_decoders(self.tuning(kind), self.regularisation) for kind in ("phasor", "product", "scalar")}

    def describe(self) -> Dict[str, object]:
        return {"sizes": asdict(self.sizes), "model": asdict(self.model), "seed": int(self.seed),
                "regularisation": float(self.regularisation)}


__all__ = ["Design", "Sizes"]

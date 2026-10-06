"""Standard forward kinematics, and what it costs, beside the ring network."""

from __future__ import annotations

from typing import Dict

from ..kinematics import Chain
from ..wiring import chain_wiring


def cost(chain: Chain) -> Dict[str, int]:
    """Arithmetic of the exact orientation (one 3 x 3 product with a joint rotation per joint: 12
    multiplications that are not by 0 or 1, plus the constant frame change) against the ring network's parts."""

    wiring = chain_wiring(chain, relay=True)
    return {"joints": chain.n, "trigonometric": 2 * chain.n, "multiplications_exact": 12 * chain.n + 27 * chain.n,
            "products_network": len(wiring.products), "sum_populations_network": len(wiring.entries)}


__all__ = ["cost"]

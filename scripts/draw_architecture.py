#!/usr/bin/env python3
"""Draw the network as it is built (docs/figures/architecture.png), from the wiring of a chain.

    python3 -B scripts/draw_architecture.py                    # generic3, the picture of the docs
    python3 -B scripts/draw_architecture.py --chain tiago7 --out /out/tiago7.png

The numbers in the picture (products and link populations per joint, neurons)
are counted from the same wiring the NEST network is built from.
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.chains import get_chain  # noqa: E402
from ring_so3.design import Sizes  # noqa: E402
from ring_so3.wiring import Entry, Product, chain_wiring  # noqa: E402

INK, GREY, LIGHT, BLUE, GREEN, ORANGE = "#1c2330", "#8a94a3", "#eef1f5", "#2a78d6", "#1baf7a", "#eb6834"


def box(ax, x, y, w, h, text, color, sub=None):
    from matplotlib.patches import FancyBboxPatch

    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0.02,rounding_size=0.08", fc=LIGHT, ec=color, lw=1.6))
    ax.text(x, y + (0.1 if sub else 0.0), text, ha="center", va="center", fontsize=9.5, color=INK, fontweight="bold")
    if sub:
        ax.text(x, y - 0.17, sub, ha="center", va="center", fontsize=8, color=GREY)


def ring(ax, x, y, r, color, bump):
    angles = np.linspace(0, 2 * np.pi, 16, endpoint=False)
    for a in angles:
        near = abs(np.angle(np.exp(1j * (a - bump)))) < 0.6
        ax.plot(x + r * np.cos(a), y + r * np.sin(a), "o", ms=5.5 if near else 4, color=color if near else "white", mec=color, mew=1)


def arrow(ax, a, b, color=GREY):
    ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="-|>", color=color, lw=1.3, shrinkA=2, shrinkB=2))


def draw(chain, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    wiring = chain_wiring(chain)
    sizes = Sizes()
    n = chain.n
    products = {k: sum(1 for s in wiring.steps if isinstance(s, Product) and s.joint == k) for k in range(n)}
    links = {k + 1: sum(1 for s in wiring.steps if isinstance(s, Entry) and s.link == k + 1) for k in range(n)}
    width = 3.0 + 2.6 * n
    fig, ax = plt.subplots(figsize=(width, 5.6))
    ax.set_xlim(-0.6, 2.6 * n + 2.6)
    ax.set_ylim(-0.5, 5.4)
    ax.axis("off")
    for k in range(n):
        x = 0.9 + 2.6 * k
        ring(ax, x, 4.6, 0.36, BLUE, 0.7 + 1.3 * k)
        ax.text(x, 5.2, "joint ring %d" % (k + 1), ha="center", fontsize=9.5, color=INK, fontweight="bold")
        ax.text(x + 0.5, 4.6, "%d neurons\nbump at q%d" % (sizes.ring, k + 1), ha="left", va="center", fontsize=8, color=GREY)
        box(ax, x, 3.35, 2.1, 0.62, "cos q%d, sin q%d" % (k + 1, k + 1), BLUE, "%d neurons" % sizes.feature)
        arrow(ax, (x, 4.18), (x, 3.68))
        if k == 0:
            box(ax, x, 1.6, 2.1, 0.9, "link 1", GREEN, "its entries are\ncos q1, sin q1, 0, 1")
            arrow(ax, (x, 3.02), (x, 2.08))
        else:
            box(ax, x, 2.3, 2.1, 0.62, "x joint %d" % (k + 1), ORANGE, "%d products x %d neurons" % (products[k], sizes.product))
            arrow(ax, (x, 3.02), (x, 2.63))
            box(ax, x, 1.15, 2.1, 0.7, "link %d" % (k + 1), GREEN,
                "%d new entries x %d neurons" % (links[k + 1], sizes.entry) if links[k + 1] else "read from the products")
            arrow(ax, (x, 1.98), (x, 1.52))
            previous = 0.9 + 2.6 * (k - 1)
            arrow(ax, (previous + 1.07, 1.6 if k == 1 else 1.15), (x - 1.07, 2.2))
    last = 0.9 + 2.6 * (n - 1)
    out_x = last + 2.6
    for i, name in enumerate(("lift", "pitch", "yaw")):
        y = 3.6 - 1.25 * i
        ring(ax, out_x, y, 0.3, GREEN, 2.0 + i)
        ax.text(out_x + 0.45, y, "%s ring\n%d neurons" % (name, sizes.output_ring), ha="left", va="center", fontsize=8.5, color=INK)
        arrow(ax, (last + 1.07, 1.15 if n > 1 else 1.6), (out_x - 0.4, y))
    ax.text(out_x, 0.42, "and the 3 x 3 matrix\nof every link (readout cells)", ha="center", va="center", fontsize=8.5, color=INK)
    arrow(ax, (last + 0.5, 0.78), (out_x - 0.95, 0.45))
    ax.text(-0.5, -0.25, "%s: %d joints, %d products, %d link populations.  Blue: one joint.  Orange: one multiplying stage per further joint.  Green: orientation."
            % (chain.name, n, len(wiring.products), len(wiring.entries)), fontsize=8.5, color=GREY, va="center")
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--chain", default="generic3")
    parser.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "figures", "architecture.png"))
    args = parser.parse_args(argv)
    draw(get_chain(args.chain), args.out)
    print("written:", os.path.relpath(args.out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

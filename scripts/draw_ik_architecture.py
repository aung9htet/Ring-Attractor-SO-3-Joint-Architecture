#!/usr/bin/env python3
"""Draw the inverse-kinematics layer as it is built (docs/ik/figures/architecture.png), from the wiring of a chain.

    python3 -B scripts/draw_ik_architecture.py                      # generic3, the picture of the docs
    python3 -B scripts/draw_ik_architecture.py --chain tiago7 --out /out/ik_tiago7.png
    python3 -B scripts/draw_ik_architecture.py --comparison angles --out /out/ik_angles.png

The numbers in the picture (products, sums and neurons of every part) are counted from the
same wiring the NEST network is built from (ring_so3/ik/wiring.py).
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.chains import get_chain  # noqa: E402
from ring_so3.design import Sizes  # noqa: E402
from ring_so3.ik.design import COMPARISONS, IkDesign  # noqa: E402
from ring_so3.ik.wiring import Half, Mul, Relay, ik_wiring  # noqa: E402
from ring_so3.wiring import chain_wiring  # noqa: E402

INK, GREY, LIGHT = "#1c2330", "#8a94a3", "#eef1f5"
BLUE, GREEN, ORANGE, PURPLE = "#2a78d6", "#1baf7a", "#eb6834", "#7559c7"


def count(chain, design):
    """Products and sums of every part of the layer, in the order the wiring lists them."""

    wiring = ik_wiring(chain, design=design)
    parts = {"goal": [0, 0], "comparison": [0, 0], "share": [[0, 0] for _ in range(chain.n)], "rest": [[0, 0] for _ in range(chain.n)]}
    where, joint = "comparison", 0
    for step in wiring.steps:
        if isinstance(step, Mul):
            if step.name is not None:
                parts["goal"][0] += 1
            elif where == "comparison":
                parts["comparison"][0] += 1
            else:
                parts[where][joint][0] += 1
        elif isinstance(step, Relay):
            kind = step.node[0]
            if kind == "g":
                parts["goal"][1] += 1
            elif kind in ("turn", "adiff"):
                parts["comparison"][1] += 1
                if step.node == ("turn", 2):
                    where, joint = "share", 0
            else:                                   # ("rest", k + 1, i)
                parts["rest"][joint][1] += 1
                if step.node[2] == 2:
                    where, joint = "share", joint + 1
        elif isinstance(step, Half):
            parts["share"][joint][1] += 1
            if step.node[2] < 0:
                where = "rest"
    return wiring, parts


def neurons(chain, wiring, sizes):
    forward = chain_wiring(chain)
    fk = chain.n * (sizes.ring + sizes.feature) + len(forward.products) * sizes.product + len(forward.entries) * sizes.entry
    fk += 3 * sizes.output_ring + sizes.norm
    ik = 3 * (sizes.ring + sizes.feature) + len(wiring.muls) * sizes.product + (len(wiring.relays) + len(wiring.halves)) * sizes.entry
    ik += chain.n * 3 * sizes.ring                                  # per joint: a goal ring and the two sides of its gain pair
    return fk, ik


def box(ax, x, y, w, h, text, color, sub=None, size=9.5):
    from matplotlib.patches import FancyBboxPatch

    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0.02,rounding_size=0.08", fc=LIGHT, ec=color, lw=1.6))
    lines = 0 if not sub else sub.count("\n") + 1
    ax.text(x, y + 0.11 * lines, text, ha="center", va="center", fontsize=size, color=INK, fontweight="bold")
    if sub:
        ax.text(x, y - 0.16, sub, ha="center", va="center", fontsize=7.8, color=GREY, linespacing=1.25)


def ring(ax, x, y, r, color, bump, ghost=None):
    """A ring of dots with a bump at ``bump``; ``ghost``: where another ring's bump is (drawn as a tick outside)."""

    for a in np.linspace(0, 2 * np.pi, 16, endpoint=False):
        near = abs(np.angle(np.exp(1j * (a - bump)))) < 0.6
        ax.plot(x + r * np.cos(a), y + r * np.sin(a), "o", ms=5.3 if near else 3.8, color=color if near else "white", mec=color, mew=1)
    if ghost is not None:
        ax.plot([x + 1.25 * r * np.cos(ghost), x + 1.5 * r * np.cos(ghost)], [y + 1.25 * r * np.sin(ghost), y + 1.5 * r * np.sin(ghost)],
                color=BLUE, lw=2, solid_capstyle="round")


def arrow(ax, a, b, color=GREY, rad=0.0, style="-|>", lw=1.3, dash=None):
    ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle=style, color=color, lw=lw, shrinkA=2, shrinkB=2,
                                                    connectionstyle="arc3,rad=%g" % rad, linestyle=dash or "-"))


def label(ax, x, y, text, color=GREY, size=7.8, **kwargs):
    ax.text(x, y, text, ha=kwargs.pop("ha", "center"), va=kwargs.pop("va", "center"), fontsize=size, color=color, linespacing=1.25, **kwargs)


def draw(chain, design, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    sizes = Sizes()
    wiring, parts = count(chain, design)
    fk_neurons, ik_neurons = neurons(chain, wiring, sizes)
    n = chain.n
    step, first = 3.4, 8.3                           # width of one joint's column, x of the first joint's share
    row_left, row_share, bus = 5.3, 3.85, 2.9        # y of "what is left", of the shares, of the line of joint axes
    x_min, x_max, y_min, y_max = -0.1, first + step * (n - 1) + 4.6, -2.45, 7.45
    unit = 0.78                                      # inches per unit, the same both ways
    fig, ax = plt.subplots(figsize=(unit * (x_max - x_min), unit * (y_max - y_min)))
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_aspect("equal")
    ax.axis("off")

    # -- the goal, top left ------------------------------------------------------
    for i, name in enumerate(("lift", "pitch", "yaw")):
        x = 0.8 + 1.1 * i
        ring(ax, x, 6.55, 0.3, PURPLE, 0.9 + 1.7 * i)
        label(ax, x, 7.08, name, INK, 8.5, fontweight="bold")
    label(ax, 3.55, 6.55, "the goal: 3 goal rings,\n%d neurons each" % sizes.ring, INK, 8.3, ha="left")
    box(ax, 1.9, 5.55, 3.4, 0.62, "cos, sin of each goal angle", PURPLE, "3 x %d neurons" % sizes.feature)
    arrow(ax, (1.9, 6.15), (1.9, 5.88))
    if design.comparison == "matrix":
        box(ax, 1.9, 4.45, 3.4, 0.86, "goal orientation  G", PURPLE,
            "%d products x %d neurons,\n%d sums x %d neurons" % (parts["goal"][0], sizes.product, parts["goal"][1], sizes.entry))
        arrow(ax, (1.9, 5.22), (1.9, 4.9))
        source_goal = (3.62, 4.6)
    else:
        source_goal = (3.62, 5.5)

    # -- the forward network, bottom left ---------------------------------------
    box(ax, 1.9, 2.0, 3.4, 1.5, "forward network", GREEN,
        "unchanged, %s neurons:\njoint rings  >  cos, sin  >\norientation of every link  >  R" % format(fk_neurons, ","))

    # -- the comparison ----------------------------------------------------------
    cx = 5.75
    box(ax, cx, row_left, 3.2, 1.06, "comparison  e", ORANGE,
        "%d products x %d, %d sums x %d:\nabout which axis, and how far,\nthe hand still has to turn" % (
            parts["comparison"][0], sizes.product, parts["comparison"][1], sizes.entry))
    arrow(ax, source_goal, (cx - 1.62, row_left - 0.1 if design.comparison == "matrix" else row_left + 0.1))
    arrow(ax, (3.0, 2.77), (cx - 0.75, row_left - 0.55))
    label(ax, 3.95, 3.72, "decoded R", rotation=42)

    # -- the joint axes: one line under the cascade -------------------------------
    last_share = first + step * (n - 1)
    if n > 1:
        ax.plot([3.62, 4.35, 4.35, last_share - 0.6], [2.2, 2.2, bus, bus], color=GREEN, lw=1.3, solid_capstyle="round")
    label(ax, 4.5, bus - 0.36, "axis of every joint,  a_k:\none column of link k", GREEN, ha="left")

    # -- the cascade, the goal rings of the joints, the robot ---------------------
    robot_y = -1.1
    previous = cx                                   # x of the box on the upper row that holds what is left so far
    previous_half = 1.6                             # half its width
    for k in range(n):
        x = first + step * k
        share_products, halves = parts["share"][k]
        rest_products, rest_sums = parts["rest"][k]
        box(ax, x, row_share, 2.25, 0.86, "share  t%d" % (k + 1), ORANGE,
            ("%d products x %d\n" % (share_products, sizes.product) if share_products else "axis is fixed: no product\n")
            + "%d halves x %d neurons" % (halves, sizes.entry))
        arrow(ax, (previous + 0.55, row_left - 0.5), (x - 0.55, row_share + 0.45))          # what is left so far, into the share
        if share_products:                          # the first joint's axis is a constant and comes from nowhere
            arrow(ax, (x - 0.6, bus), (x - 0.6, row_share - 0.43), GREEN)
        if design.cascade:
            rx = x + step / 2.0
            box(ax, rx, row_left, 2.0, 0.86, "left  e%d" % (k + 2), ORANGE,
                ("%d products x %d\n" % (rest_products, sizes.product) if rest_products else "no product\n")
                + "%d sums x %d" % (rest_sums, sizes.entry))
            arrow(ax, (previous + previous_half, row_left), (rx - 1.02, row_left))          # e_k itself
            arrow(ax, (x + 0.6, row_share + 0.45), (rx - 0.35, row_left - 0.45))           # minus the share that was taken
            previous, previous_half = rx, 1.0

        # the joint's own rings
        box(ax, x, 2.05, 2.3, 0.66, "gain pair %d" % (k + 1), PURPLE, "left and right: 2 x %d" % sizes.ring)
        arrow(ax, (x + 0.5, row_share - 0.43), (x + 0.5, 2.4), PURPLE)
        label(ax, x + 0.62, 3.1, "its two halves move\nthe two sides apart", PURPLE, ha="left", size=7.3)
        state, goal = (x - 0.8, 0.6), (x + 0.8, 0.6)
        bump = 0.9 + 1.1 * k
        ring(ax, state[0], state[1], 0.3, BLUE, bump)
        ring(ax, goal[0], goal[1], 0.3, PURPLE, bump + 0.55, ghost=bump)
        label(ax, state[0], 0.0, "joint ring %d\n(where it is)" % (k + 1), INK, 7.8)
        label(ax, goal[0], 0.0, "goal ring %d\n(where next)" % (k + 1), INK, 7.8)
        arrow(ax, (state[0] + 0.05, 0.97), (x - 0.5, 1.7), BLUE)
        label(ax, x - 0.98, 1.38, "gates", BLUE, size=7.5, ha="right")
        arrow(ax, (x + 0.5, 1.7), (goal[0] - 0.05, 0.99), PURPLE)
        label(ax, x + 0.98, 1.38, "writes", PURPLE, size=7.5, ha="left")
        arrow(ax, (goal[0], -0.34), (goal[0], robot_y + 0.36), PURPLE)
        arrow(ax, (state[0], robot_y + 0.36), (state[0], -0.34), BLUE)

    if design.cascade:
        label(ax, previous + 1.12, row_left, "what no\njoint took", ha="left")
    box(ax, first + step * (n - 1) / 2.0, robot_y, max(step * (n - 1) + 3.6, 10.4), 0.66,
        "robot: the light robot, or the TIAGo arm in Gazebo", INK,
        "every joint moves toward the angle its goal ring holds (purple);  its angle is written on its joint ring (blue)")
    # the joint rings are the forward network's own
    arrow(ax, (first - 1.2, 0.6), (3.62, 1.45), BLUE, dash=(0, (4, 3)))
    label(ax, 5.25, 0.52, "the joint rings are the\nforward network's own", BLUE, size=7.6)

    title = "%s, %s comparison%s" % (chain.name, "matrix" if design.comparison == "matrix" else "angle",
                                    "" if design.cascade else ", every joint gets the whole comparison")
    ax.text(0.0, -2.05, "%s: %s neurons in all, %s of them in this layer (%d products, %d sums, %d share halves; 3 goal rings; per joint a gain pair and a goal ring).\n"
                        "Purple: goal.  Orange: comparison and cascade.  Blue: joint rings.  Green: the forward network and what is taken from it."
            % (title, format(fk_neurons + ik_neurons, ","), format(ik_neurons, ","), len(wiring.muls), len(wiring.relays), len(wiring.halves)),
            fontsize=8.3, color=GREY, va="center", linespacing=1.4)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    return fk_neurons, ik_neurons


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--chain", default="generic3")
    parser.add_argument("--comparison", choices=COMPARISONS, default=IkDesign().comparison)
    parser.add_argument("--all-at-once", action="store_true", help="draw the variant without the cascade")
    parser.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "ik", "figures", "architecture.png"))
    args = parser.parse_args(argv)
    fk_neurons, ik_neurons = draw(get_chain(args.chain), IkDesign(comparison=args.comparison, cascade=not args.all_at_once), args.out)
    print("written: %s (%s: forward network %d neurons, IK layer %d, together %d)" % (
        os.path.relpath(args.out), args.chain, fk_neurons, ik_neurons, fk_neurons + ik_neurons))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

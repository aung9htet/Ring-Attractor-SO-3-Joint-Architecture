#!/usr/bin/env python3
"""Record an inverse-kinematics session without a browser: a video, a figure and the log.

    python3 -B scripts/record_ik_session.py --chain tiago7 --goals 5 --out docs/ik/results/videos/tiago7_ik

The hand is given one random orientation goal after another (the orientation of a random
pose, so it can be reached), ``--dwell`` seconds each, and the spiking network turns the arm
to it.  Written: ``<out>.mp4`` (the arm; at the hand the true frame, solid, and the goal,
dotted; lift, pitch, yaw and the distance from the goal over time), ``<out>.png`` (the whole
session, with every joint's share) and ``<out>.json`` (every tick).  ``--stub`` uses the exact
rule instead of the network.
"""

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.chains import MODEL_NAMES  # noqa: E402
from ring_so3.design import Design  # noqa: E402
from ring_so3.ik.design import IkDesign  # noqa: E402
from ring_so3.sim.server import Simulator, StubIkNetwork  # noqa: E402

INK, GREY, LIGHT, GOAL, ERROR = "#1c2330", "#8a94a3", "#d9dee6", "#8a56c2", "#eb6834"
AXIS_COLORS = ("#d9443a", "#2f9e5b", "#2a78d6")


def _angles(values):
    out = np.array([np.nan if v is None else np.degrees(v) for v in values], dtype=float)
    out[1:][np.abs(np.diff(out)) > 180.0] = np.nan                # no line across the jump from +180 to -180
    return out


def series(ticks):
    t = np.array([x["t"] for x in ticks]) - ticks[0]["t"]
    true = [_angles([x["lpy_true"][a] for x in ticks]) for a in range(3)]
    goal = [_angles([x["ik"]["written"][a] for x in ticks]) for a in range(3)]
    error = np.array([x["ik"]["goal_error"] for x in ticks])
    shares = np.degrees(np.array([[0.0 if v is None else v for v in x["ik"]["shares"]] for x in ticks]))
    return t, true, goal, error, shares


def figure(plt, ticks, changes, title, path):
    t, true, goal, error, shares = series(ticks)
    fig, axes = plt.subplots(5, 1, figsize=(8.5, 9.0), sharex=True)
    for a, name in enumerate(("lift", "pitch", "yaw")):
        axes[a].plot(t, goal[a], color=GOAL, lw=1.6, label="goal on the goal ring")
        axes[a].plot(t, true[a], color=INK, lw=1.6, label="true")
        half = 90 if a == 1 else 180
        axes[a].set_ylim(-half - 10, half + 10)
        axes[a].set_yticks([-half, 0, half])
        axes[a].set_ylabel(name + " (deg)")
    axes[3].plot(t, error, color=ERROR, lw=1.5)
    axes[3].axhline(5.0, color=GREY, lw=0.8, ls="--")
    axes[3].set_ylabel("from the goal (deg)")
    axes[3].set_ylim(0, None)
    colors = plt.cm.viridis(np.linspace(0.05, 0.9, shares.shape[1]))
    for k in range(shares.shape[1]):
        axes[4].plot(t, shares[:, k], color=colors[k], lw=1.1, label="joint %d" % (k + 1))
    axes[4].set_ylabel("share of the turn (deg)")
    axes[4].set_xlabel("time (s)")
    axes[4].legend(loc="upper right", ncol=4, fontsize=7, frameon=False)
    for axis in axes:
        for when in changes:
            axis.axvline(when, color=LIGHT, lw=1.0, zorder=0)
    fig.suptitle(title, x=0.01, y=0.995, ha="left", fontsize=10.5, fontweight="bold")
    fig.legend(*axes[0].get_legend_handles_labels(), loc="upper right", ncol=2, bbox_to_anchor=(0.99, 0.975), frameon=False)
    fig.subplots_adjust(top=0.93)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def video(plt, animation, ticks, config, title, path, fps=20):
    t, true, goal, error, shares = series(ticks)
    reach = max(config["reach"], 0.2)
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    grid = fig.add_gridspec(4, 2, width_ratios=[1.15, 1.0], hspace=0.35, wspace=0.12, left=0.02, right=0.97, top=0.92, bottom=0.08)
    view = fig.add_subplot(grid[:, 0], projection="3d")
    panels = [fig.add_subplot(grid[a, 1]) for a in range(4)]
    fig.suptitle(title, x=0.02, ha="left", fontsize=12, fontweight="bold", color=INK)
    points = np.array([x["points"] for x in ticks])
    flat = points.reshape(-1, 3)
    middle = (flat.min(axis=0) + flat.max(axis=0)) / 2.0
    span = max(float(np.max(flat.max(axis=0) - flat.min(axis=0))) / 2.0, reach * 0.3) * 1.25
    view.set_xlim(middle[0] - span, middle[0] + span)
    view.set_ylim(middle[1] - span, middle[1] + span)
    view.set_zlim(middle[2] - span, middle[2] + span)
    view.set_box_aspect((1, 1, 1))
    view.view_init(elev=22, azim=-55)
    for axis in (view.xaxis, view.yaxis, view.zaxis):
        axis.set_ticklabels([])
        axis.pane.set_facecolor("#f6f7f9")
        axis.pane.set_edgecolor(LIGHT)
    arm, = view.plot([], [], [], "-o", color=GREY, lw=4, ms=4, mfc=INK, mec=INK)
    length = reach * 0.22
    true_lines = [view.plot([], [], [], "-", color=AXIS_COLORS[a], lw=2.8)[0] for a in range(3)]
    goal_lines = [view.plot([], [], [], ":", color=AXIS_COLORS[a], lw=2.6)[0] for a in range(3)]
    view.plot([], [], [], "-", color=INK, lw=2, label="true frame of the hand")
    view.plot([], [], [], ":", color=INK, lw=2, label="goal")
    view.legend(loc="upper left", frameon=False, fontsize=9)
    label = view.text2D(0.02, 0.02, "", transform=view.transAxes, fontsize=13, color=INK)
    cursors, growing = [], []
    for a, name in enumerate(("lift", "pitch", "yaw")):
        ax = panels[a]
        half = 90 if a == 1 else 180
        ax.plot(t, goal[a], color=LIGHT, lw=1.2)
        growing.append((ax.plot([], [], color=GOAL, lw=1.6)[0], goal[a], ax.plot([], [], color=INK, lw=1.6)[0], true[a]))
        ax.set_ylim(-half - 10, half + 10)
        ax.set_yticks([-half, 0, half])
        ax.set_ylabel(name + " (deg)", fontsize=9)
        ax.set_xlim(t[0], t[-1])
        ax.tick_params(labelbottom=False)
        cursors.append(ax.axvline(0.0, color=GREY, lw=0.8))
    panels[0].plot([], [], color=GOAL, label="goal on the goal ring")
    panels[0].plot([], [], color=INK, label="true")
    panels[0].legend(loc="lower right", ncol=2, frameon=False, fontsize=9, bbox_to_anchor=(1.0, 1.0))
    panels[3].set_xlim(t[0], t[-1])
    panels[3].set_ylim(0, max(10.0, float(np.max(error)) * 1.05))
    panels[3].axhline(5.0, color=GREY, lw=0.8, ls="--")
    panels[3].set_ylabel("from the goal (deg)", fontsize=9)
    panels[3].set_xlabel("time (s)", fontsize=9)
    error_line, = panels[3].plot([], [], color=ERROR, lw=1.5)
    cursors.append(panels[3].axvline(0.0, color=GREY, lw=0.8))
    for ax in panels:
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.grid(True, color=LIGHT, lw=0.8)

    def draw(i):
        x = ticks[i]
        p = np.array(x["points"])
        arm.set_data_3d(p[:, 0], p[:, 1], p[:, 2])
        origin = p[-1]
        for lines, rotation, scale in ((true_lines, np.array(x["true"][-1]).reshape(3, 3), 1.0),
                                       (goal_lines, np.array(x["ik"]["goal_matrix"]).reshape(3, 3), 1.25)):
            for a in range(3):
                tip = origin + length * scale * rotation[:, a]
                lines[a].set_data_3d([origin[0], tip[0]], [origin[1], tip[1]], [origin[2], tip[2]])
        for goal_line, goal_values, true_line, true_values in growing:
            goal_line.set_data(t[: i + 1], goal_values[: i + 1])
            true_line.set_data(t[: i + 1], true_values[: i + 1])
        error_line.set_data(t[: i + 1], error[: i + 1])
        for cursor in cursors:
            cursor.set_xdata([t[i], t[i]])
        label.set_text("%.1f deg from the goal" % error[i])
        return []

    writer = animation.FFMpegWriter(fps=fps, codec="libx264", extra_args=["-pix_fmt", "yuv420p", "-crf", "24"])
    with writer.saving(fig, path, dpi=100):
        for i in range(len(ticks)):
            draw(i)
            writer.grab_frame()
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--chain", default="tiago7", choices=MODEL_NAMES)
    parser.add_argument("--goals", type=int, default=5)
    parser.add_argument("--dwell", type=float, default=12.0, help="seconds per goal")
    parser.add_argument("--seed", type=int, default=3, help="seed of the random goals")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--stub", action="store_true")
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--out", required=True, help="path without extension")
    args = parser.parse_args(argv)

    if args.stub:
        make_network = lambda chain, cascade=True: StubIkNetwork(chain, IkDesign(cascade=cascade))  # noqa: E731
    else:
        import nest
        from ring_so3.nest import calibrate, ik_calibrate
        from ring_so3.nest.ik_network import IkNetwork
        from ring_so3.nest.network import NetworkOptions

        design = Design()
        calibration, ik_cal = calibrate.load(design), ik_calibrate.load(design)
        make_network = lambda chain, cascade=True: IkNetwork(  # noqa: E731
            nest, chain, calibration, ik_cal, design, NetworkOptions(threads=args.threads), IkDesign(cascade=cascade))

    simulator = Simulator(make_network, args.chain, mode="manual", seed=args.seed, realtime=False)
    for _ in range(30):                                              # let the rings form their bumps
        simulator.tick()
    ticks, changes, per_goal = [], [], int(round(args.dwell / simulator.dt_s))
    for _ in range(args.goals):
        simulator.handle({"command": "random_goal"})
        changes.append(len(ticks) * simulator.dt_s)
        ticks += [simulator.tick() for _ in range(per_goal)]
    config = simulator.config()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    finals = [ticks[(g + 1) * per_goal - 1]["ik"]["goal_error"] for g in range(args.goals)]
    with open(args.out + ".json", "w", encoding="utf-8") as stream:
        json.dump({"chain": args.chain, "goals": args.goals, "dwell_s": args.dwell, "seed": args.seed, "stub": bool(args.stub),
                   "neurons": config["neurons"], "final_error_deg": finals,
                   "ticks": [{"t": x["t"], "q": x["q"], "lpy_true": x["lpy_true"], "error": x["error"],
                              "ik": {k: x["ik"][k] for k in ("goal", "written", "goal_error", "turn", "shares", "left", "lead")}} for x in ticks]},
                  stream, separators=(",", ":"))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation

    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "axes.edgecolor": GREY,
                         "xtick.color": GREY, "ytick.color": GREY, "text.color": INK, "axes.labelcolor": INK,
                         "axes.grid": True, "grid.color": LIGHT, "grid.linewidth": 0.8})
    title = "%s turns its hand to %d orientation goals%s" % (
        args.chain, args.goals, " (exact rule, no spikes)" if args.stub else ", %s spiking neurons" % format(config["neurons"], ","))
    figure(plt, ticks, changes, title, args.out + ".png")
    if not args.no_video:
        video(plt, animation, ticks, config, title, args.out + ".mp4")
    print("written: %s.{json,png%s} | %d ticks | distance from each goal at the end of its %.0f s: %s deg" % (
        args.out, "" if args.no_video else ",mp4", len(ticks), args.dwell, ", ".join("%.1f" % v for v in finals)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

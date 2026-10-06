#!/usr/bin/env python3
"""Record a session of the simulator without a browser: a video, a figure and the log.

    python3 -B scripts/record_session.py --chain tiago7 --seconds 45 --out docs/results/videos/tiago7_goals

The arm keeps moving from one random goal to the next (as the page's goal mode
does) while the spiking network runs.  Written: ``<out>.mp4`` (the arm with the
true frame, solid, and the frame read from the network, dashed, at every link;
lift, pitch, yaw and the orientation error over time), ``<out>.png`` (the same
curves for the whole session) and ``<out>.json`` (every tick).  ``--stub`` uses
exact kinematics with a little noise instead of the network.
"""

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.chains import MODEL_NAMES  # noqa: E402
from ring_so3.design import Design  # noqa: E402
from ring_so3.sim.server import Simulator, StubNetwork  # noqa: E402

INK, GREY, LIGHT, DECODED, ERROR = "#1c2330", "#8a94a3", "#d9dee6", "#2a78d6", "#eb6834"
AXIS_COLORS = ("#d9443a", "#2f9e5b", "#2a78d6")


def collect(simulator, seconds):
    ticks = []
    for _ in range(int(round(seconds / simulator.dt_s))):
        ticks.append(simulator.tick())
    return ticks


def _angles(ticks, key, a):
    values = np.array([np.nan if t[key][a] is None else np.degrees(t[key][a]) for t in ticks], dtype=float)
    values[1:][np.abs(np.diff(values)) > 180.0] = np.nan            # no line across the jump from +180 to -180
    return values


def figure(plt, ticks, title, path):
    t = np.array([x["t"] for x in ticks]) - ticks[0]["t"]
    fig, axes = plt.subplots(4, 1, figsize=(8.5, 7.0), sharex=True)
    for a, name in enumerate(("lift", "pitch", "yaw")):
        axes[a].plot(t, _angles(ticks, "lpy_true", a), color=INK, lw=1.6, label="true")
        axes[a].plot(t, _angles(ticks, "lpy_ring", a), color=DECODED, lw=1.6, label="output ring")
        half = 90 if a == 1 else 180
        axes[a].set_ylim(-half - 10, half + 10)
        axes[a].set_yticks([-half, 0, half])
        axes[a].set_ylabel(name + " (deg)")
    fig.suptitle(title, x=0.01, y=0.995, ha="left", fontsize=10.5, fontweight="bold")
    fig.legend(*axes[0].get_legend_handles_labels(), loc="upper right", ncol=2, bbox_to_anchor=(0.99, 0.965), frameon=False)
    fig.subplots_adjust(top=0.9)
    axes[3].plot(t, [x["error"] for x in ticks], color=ERROR, lw=1.4)
    axes[3].set_ylabel("orientation error (deg)")
    axes[3].set_ylim(0, None)
    axes[3].set_xlabel("time (s)")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def video(plt, animation, ticks, config, title, path, fps=20):
    t = np.array([x["t"] for x in ticks]) - ticks[0]["t"]
    reach = max(config["reach"], 0.2)
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    grid = fig.add_gridspec(4, 2, width_ratios=[1.15, 1.0], hspace=0.35, wspace=0.12, left=0.02, right=0.97, top=0.92, bottom=0.08)
    view = fig.add_subplot(grid[:, 0], projection="3d")
    panels = [fig.add_subplot(grid[a, 1]) for a in range(4)]
    fig.suptitle(title, x=0.02, ha="left", fontsize=12, fontweight="bold", color=INK)
    points = np.array([x["points"] for x in ticks])
    middle = (points.reshape(-1, 3).min(axis=0) + points.reshape(-1, 3).max(axis=0)) / 2.0
    span = max(float(np.max(points.reshape(-1, 3).max(axis=0) - points.reshape(-1, 3).min(axis=0))) / 2.0, reach * 0.3) * 1.15
    view.set_xlim(middle[0] - span, middle[0] + span)
    view.set_ylim(middle[1] - span, middle[1] + span)
    view.set_zlim(middle[2] - span, middle[2] + span)
    view.set_box_aspect((1, 1, 1))
    view.view_init(elev=22, azim=-55)
    for axis in (view.xaxis, view.yaxis, view.zaxis):
        axis.set_ticklabels([])
    view.set_xlabel("x"), view.set_ylabel("y"), view.set_zlabel("z")
    for axis in (view.xaxis, view.yaxis, view.zaxis):
        axis.pane.set_facecolor("#f6f7f9")
        axis.pane.set_edgecolor(LIGHT)
    arm, = view.plot([], [], [], "-o", color=GREY, lw=4, ms=4, mfc=INK, mec=INK)
    n_frames = len(ticks[0]["true"])
    length = reach * 0.13
    true_lines = [[view.plot([], [], [], "-", color=AXIS_COLORS[a], lw=2.6 if k == n_frames - 1 else 1.8)[0] for a in range(3)] for k in range(n_frames)]
    read_lines = [[view.plot([], [], [], "--", color=AXIS_COLORS[a], lw=1.8 if k == n_frames - 1 else 1.2)[0] for a in range(3)] for k in range(n_frames)]
    view.plot([], [], [], "-", color=INK, lw=2, label="true frame")
    view.plot([], [], [], "--", color=INK, lw=1.4, label="read from the network")
    view.legend(loc="upper left", frameon=False, fontsize=9)
    label = view.text2D(0.02, 0.02, "", transform=view.transAxes, fontsize=13, color=INK)
    cursors, growing = [], []
    for a, name in enumerate(("lift", "pitch", "yaw")):
        ax = panels[a]
        half = 90 if a == 1 else 180
        ax.plot(t, _angles(ticks, "lpy_true", a), color=LIGHT, lw=1.2)
        growing.append((ax.plot([], [], color=INK, lw=1.6)[0], _angles(ticks, "lpy_true", a),
                        ax.plot([], [], color=DECODED, lw=1.6)[0], _angles(ticks, "lpy_ring", a)))
        ax.set_ylim(-half - 10, half + 10)
        ax.set_yticks([-half, 0, half])
        ax.set_ylabel(name + " (deg)", fontsize=9)
        ax.set_xlim(t[0], t[-1])
        ax.tick_params(labelbottom=False)
        cursors.append(ax.axvline(0.0, color=GREY, lw=0.8))
    panels[0].plot([], [], color=INK, label="true")
    panels[0].plot([], [], color=DECODED, label="output ring")
    panels[0].legend(loc="lower right", ncol=2, frameon=False, fontsize=9, bbox_to_anchor=(1.0, 1.0))
    errors = np.array([x["error"] for x in ticks])
    panels[3].set_xlim(t[0], t[-1])
    panels[3].set_ylim(0, max(5.0, float(np.max(errors)) * 1.05))
    panels[3].set_ylabel("error (deg)", fontsize=9)
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
        for k in range(n_frames):
            origin = p[k + 1]
            for lines, key, scale in ((true_lines, "true", 1.0), (read_lines, "decoded", 1.18)):
                rotation = np.array(x[key][k]).reshape(3, 3)
                size = length * scale * (1.5 if k == n_frames - 1 else 1.0)
                for a in range(3):
                    tip = origin + size * rotation[:, a]
                    lines[k][a].set_data_3d([origin[0], tip[0]], [origin[1], tip[1]], [origin[2], tip[2]])
        for true_line, true_values, ring_line, ring_values in growing:
            true_line.set_data(t[: i + 1], true_values[: i + 1])
            ring_line.set_data(t[: i + 1], ring_values[: i + 1])
        error_line.set_data(t[: i + 1], errors[: i + 1])
        for cursor in cursors:
            cursor.set_xdata([t[i], t[i]])
        label.set_text("orientation error %.1f deg%s" % (x["error"], "   (moving)" if x["moving"] else "   (at rest)"))
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
    parser.add_argument("--seconds", type=float, default=45.0)
    parser.add_argument("--speed", type=float, default=0.5, help="joint speed limit, rad/s")
    parser.add_argument("--seed", type=int, default=3, help="seed of the random goals")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--lead", action="store_true", help="write each joint's angle ahead by the readout's delay")
    parser.add_argument("--stub", action="store_true")
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--out", required=True, help="path without extension")
    args = parser.parse_args(argv)

    if args.stub:
        make_network = lambda chain: StubNetwork(chain)  # noqa: E731
    else:
        import nest
        from ring_so3.nest import calibrate
        from ring_so3.nest.network import FkNetwork, NetworkOptions

        design = Design()
        calibration = calibrate.load(design)
        make_network = lambda chain: FkNetwork(nest, chain, calibration, design, NetworkOptions(threads=args.threads))  # noqa: E731

    simulator = Simulator(make_network, args.chain, speed=args.speed, mode="goals", seed=args.seed, realtime=False, lead=args.lead)
    for _ in range(20):                                              # let the rings form their bumps
        simulator.network.step(simulator.robot.q)
    ticks = collect(simulator, args.seconds)
    config = simulator.config()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    keep = ("t", "q", "target", "moving", "error", "link_error", "lpy_true", "lpy_ring", "lpy_matrix")
    with open(args.out + ".json", "w", encoding="utf-8") as stream:
        json.dump({"chain": args.chain, "speed": args.speed, "seed": args.seed, "stub": bool(args.stub), "lead": bool(args.lead),
                   "neurons": config["neurons"],
                   "ticks": [{k: x[k] for k in keep} for x in ticks]}, stream, separators=(",", ":"))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation

    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "axes.edgecolor": GREY,
                         "xtick.color": GREY, "ytick.color": GREY, "text.color": INK, "axes.labelcolor": INK,
                         "axes.grid": True, "grid.color": LIGHT, "grid.linewidth": 0.8})
    title = "%s keeps moving from goal to goal, joint speed limit %.2f rad/s%s%s" % (
        args.chain, args.speed, " (stand-in network)" if args.stub else ", %s spiking neurons" % format(config["neurons"], ","),
        ", angles written ahead" if args.lead else "")
    figure(plt, ticks, title, args.out + ".png")
    if not args.no_video:
        video(plt, animation, ticks, config, title, args.out + ".mp4")
    errors = np.array([x["error"] for x in ticks])
    moving = np.array([x["moving"] for x in ticks])
    print("written: %s.{json,png%s} | %d ticks | error median %.2f deg at rest, %.2f deg while moving" % (
        args.out, "" if args.no_video else ",mp4", len(ticks),
        np.median(errors[~moving]) if (~moving).any() else float("nan"), np.median(errors[moving]) if moving.any() else float("nan")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

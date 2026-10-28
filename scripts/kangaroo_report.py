#!/usr/bin/env python3
"""Figures and tables of the Kangaroo measurements, from docs/kangaroo/results/data/.

    python3 -B scripts/kangaroo_report.py [--data DIR] [--out DIR]

Writes <out>/tables.md, <out>/figures/*.{png,pdf} and <out>/figures/manifest.json. Missing inputs are listed in the
manifest and skipped. Numbers come from the JSON files through the helpers of scripts/report.py.
"""

import argparse
import datetime
import hashlib
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from report import fmt, rows_of, tracking_summary, triple  # noqa: E402
from ring_so3.ik.experiments import reach_summary  # noqa: E402

ROOT = os.path.normpath(os.path.join(HERE, ".."))
SIDES, SPEEDS = ("left", "right"), ("0.25", "0.50", "1.00")
METHODS = (("light_ring", "light ring"), ("mujoco_ring", "MuJoCo ring"), ("mujoco_dls", "MuJoCo dls"))
COLORS = {"left": "#2a78d6", "right": "#eb6834"}
METHOD_COLORS = {"light_ring": "#1baf7a", "mujoco_ring": "#2a78d6", "mujoco_dls": "#eb6834"}


def inputs():
    """Every expected input, relative to --data."""
    out = ["static/kangaroo_%s4.json" % s for s in SIDES]
    out += ["tracking/kangaroo_%s4_%s.json" % (s, v) for s in SIDES for v in SPEEDS]
    out += ["reach/%s_%s.json" % (m, s) for s in SIDES for m, _ in METHODS]
    return out + ["calibration/comparison.json"]


def load(data):
    docs, missing, manifest = {}, [], []
    for rel in inputs():
        path = os.path.join(data, rel)
        if not os.path.isfile(path):
            missing.append(rel)
            continue
        with open(path, "rb") as stream:
            raw = stream.read()
        docs[rel] = json.loads(raw.decode("utf-8"))
        entry = {"path": rel, "size": len(raw), "sha256": hashlib.sha256(raw).hexdigest()}
        if isinstance(docs[rel], dict):
            options, design = docs[rel].get("options", {}), docs[rel].get("design", {})
            entry["seeds_threads"] = {k: v for k, v in (
                ("design_seed", design.get("seed")), ("seed", options.get("seed")), ("threads", options.get("threads")),
                ("pose_seed", docs[rel].get("pose_seed")), ("goal_seed", docs[rel].get("goal_seed")),
                ("pair_seed", docs[rel].get("pair_seed"))) if v is not None}
        manifest.append(entry)
    return docs, missing, manifest


def commit():
    try:
        return subprocess.run(["git", "describe", "--always", "--dirty"], cwd=ROOT, capture_output=True, text=True, timeout=10, check=True).stdout.strip() or "unknown"
    except Exception:
        return "unknown"


def num(v):
    return fmt(float("nan") if v is None else v)


def tracking_runs(docs):
    return {(s, v): tracking_summary(docs[k]) for s in SIDES for v in SPEEDS
            for k in ["tracking/kangaroo_%s4_%s.json" % (s, v)] if k in docs}


# -- tables -----------------------------------------------------------------------
def tables(docs):
    out = ["# Tables of the Kangaroo results", "", "Written by `scripts/kangaroo_report.py`; do not edit by hand. Errors in degrees.", ""]
    static = [(s, rows_of(docs["static/kangaroo_%s4.json" % s])) for s in SIDES if "static/kangaroo_%s4.json" % s in docs]
    if static:
        out += ["## Held poses", "", "| Arm | Poses | Error, median / p90 / max (deg) |", "| --- | ---: | --- |"]
        out += ["| %s | %d | %s |" % (s, len(r), triple([x["error_deg"] for x in r])) for s, r in static] + [""]
    runs = tracking_runs(docs)
    if runs:
        out += ["## Tracking", "", "| Arm | Joint speed limit (rad/s) | Delay (s) | Error while moving, median / p90 (deg) | Error at rest, median / p90 (deg) | Error 1 s after a stop, median / p90 (deg) |",
                "| --- | ---: | ---: | --- | --- | --- |"]
        out += ["| %s | %s | %s | %s / %s | %s / %s | %s / %s |" % (s, v, fmt(m["delay_s"]), fmt(m["error_moving"][0]), fmt(m["error_moving"][1]),
                                                       fmt(m["error_rest"][0]), fmt(m["error_rest"][1]),
                                                       fmt(m["error_1s_after_stop"][0]), fmt(m["error_1s_after_stop"][1])) for (s, v), m in runs.items()] + [""]
    reach = [(s, m, label, docs["reach/%s_%s.json" % (m, s)]) for s in SIDES for m, label in METHODS if "reach/%s_%s.json" % (m, s) in docs]
    if reach:
        out += ["## Reaching", "", "| Arm | Method | Trials | Reached (%) | Final error of reached, median (deg) | Time to 5 deg, median (s) | Path ratio, median | Missed at a limit | Missed, joints far from target | Missed, joints on target |",
                "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
        for s, _, label, d in reach:
            r = d["rows"]
            skipped = len(d.get("skipped", []))
            q = reach_summary(r)
            if not r:
                continue
            miss = [x for x in r if not x["reached"] and not x["at_limit"]]
            blocked = sum(1 for x in miss if x.get("lag_final_rad") and max(abs(v) for v in x["lag_final_rad"]) > 0.005)
            out.append("| %s | %s | %s | %s | %s | %s | %s | %d | %d | %d |" % (
                s, label, "%d (%d not run)" % (q["trials"], skipped), fmt(100 * q["reached"]), num(q["final_error_reached_median_deg"]),
                num(q["time_to_5_deg_median_s"]), num(q["path_ratio_median"]),
                q["missed_at_limit"], blocked, len(miss) - blocked))
        out += ["", "Missed, joints far from target: max |lag_final_rad| over joints > 0.005 rad; a miss without lag data (the light robot has none) is counted under on target. Time to 5 deg median over all trials that got there, "
                "final error and path ratio over reached trials (as reach_summary).", ""]
    cal = docs.get("calibration/comparison.json")
    if cal:
        out += ["## Calibration comparison", "", "| Array | Shape | Largest absolute difference | Relative difference | Identical |", "| --- | --- | ---: | ---: | --- |"]
        out += ["| %s | %s | %.3g | %.3g | %s |" % (c["array"], "x".join(str(n) for n in c["shape"]),
                                          c["max_abs_difference"], c["relative"], "yes" if c["identical"] else "no") for c in cal] + [""]
    return "\n".join(out).rstrip() + "\n"


# -- figures ----------------------------------------------------------------------
def save(plt, fig, out_dir, name):
    for ext in ("png", "pdf"):
        fig.savefig(os.path.join(out_dir, name + "." + ext))
    plt.close(fig)
    return name


def panels(plt, n, **kw):
    fig, axes = plt.subplots(1, n, figsize=(4.6 * n, 3.6), squeeze=False, **kw)
    return fig, axes[0]


def figures(docs, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    plt.rcParams.update({"axes.grid": True, "grid.alpha": 0.3, "savefig.bbox": "tight", "figure.dpi": 150})
    os.makedirs(out_dir, exist_ok=True)
    done = []
    static = {s: rows_of(docs["static/kangaroo_%s4.json" % s]) for s in SIDES if "static/kangaroo_%s4.json" % s in docs}
    static = {s: r for s, r in static.items() if r}
    if static:
        fig, (ax,) = panels(plt, 1)
        for s, r in static.items():
            e = np.sort([x["error_deg"] for x in r])
            ax.plot(e, np.arange(1, e.size + 1) / e.size * 100, color=COLORS[s], label=s)
        for mark in (5, 10):
            ax.axvline(mark, color="grey", ls=":")
        ax.set(xlabel="orientation error (deg)", ylabel="poses with at most this error (%)", title="Held poses")
        ax.legend()
        done.append(save(plt, fig, out_dir, "static_error"))
        fig, (ax,) = panels(plt, 1)
        for s, r in static.items():
            med = np.median([x["link_error_deg"] for x in r], axis=0)
            ax.plot(range(1, len(med) + 1), med, "-o", color=COLORS[s], label=s)
        ax.set(xlabel="link (number of joints passed)", ylabel="median orientation error (deg)", title="Error along the arm")
        ax.legend()
        done.append(save(plt, fig, out_dir, "error_along_arm"))
    runs = tracking_runs(docs)
    if runs:
        fig, axes = panels(plt, 2)
        for s in SIDES:
            pts = [(float(v), m) for (side, v), m in runs.items() if side == s]
            for ax, key, label in ((axes[0], "delay_s", "delay (s)"), (axes[1], None, "median error while moving (deg)")):
                ax.plot([p[0] for p in pts], [p[1][key] if key else p[1]["error_moving"][0] for p in pts], "-o", color=COLORS[s], label=s)
                ax.set(xlabel="joint speed limit (rad/s)", ylabel=label)
        axes[0].legend()
        done.append(save(plt, fig, out_dir, "tracking"))
    reach = {(s, m): docs["reach/%s_%s.json" % (m, s)]["rows"] for s in SIDES for m, _ in METHODS
             if "reach/%s_%s.json" % (m, s) in docs and docs["reach/%s_%s.json" % (m, s)]["rows"]}
    if reach:
        fig, axes = panels(plt, 2, sharey=True)
        for ax, s in zip(axes, SIDES):
            ax.set(title=s, xlabel="time (s)", ylabel="distance from the goal (deg), trials still running")
            for m, label in METHODS:
                rows = reach.get((s, m))
                if not rows:
                    continue
                grid = np.linspace(0, max(r["t"][-1] for r in rows), 100)
                e = np.array([np.interp(grid, r["t"], r["error_deg"], right=np.nan) for r in rows])   # NaN once a trial has ended
                lo, med, hi = np.nanpercentile(e, [25, 50, 75], axis=0)    # over the trials still running
                ax.plot(grid, med, color=METHOD_COLORS[m], label=label)
                ax.fill_between(grid, lo, hi, color=METHOD_COLORS[m], alpha=0.2, lw=0)
            ax.legend()
        done.append(save(plt, fig, out_dir, "reach_error_over_time"))
    mj = {(s, m): docs["reach/%s_%s.json" % (m, s)] for s in SIDES for m in ("mujoco_ring", "mujoco_dls") if (s, m) in reach}
    if mj:
        fig, axes = panels(plt, 3)
        keys = list(mj)
        for i, k in enumerate(keys):
            rows = mj[k]["rows"]
            col = COLORS[k[0]]
            hatch = "//" if k[1] == "mujoco_dls" else None
            med = [r["lag_median_rad"] for r in rows if r.get("lag_median_rad") is not None]
            fin = [[abs(v) for v in r["lag_final_rad"]] for r in rows if r.get("lag_final_rad")]
            if med:
                axes[0].boxplot([med], positions=[i], widths=0.6, patch_artist=True, boxprops={"facecolor": col, "alpha": 0.5, "hatch": hatch})
            for j, col_vals in enumerate(zip(*fin)):
                axes[1].boxplot([col_vals], positions=[j + (i - len(keys) / 2) * 0.2], widths=0.15, patch_artist=True, manage_ticks=False,
                                boxprops={"facecolor": col, "alpha": 0.5, "hatch": hatch})
            stale = mj[k]["robot"].get("joint_state_stale_max_s") if isinstance(mj[k].get("robot"), dict) else None
            if stale is not None:
                axes[2].bar(i, stale, color=col, hatch=hatch)
        names = ["%s %s" % (k[0], "ring" if k[1] == "mujoco_ring" else "dls") for k in keys]
        axes[0].set_xticks(range(len(keys)), names, rotation=30)
        axes[2].set_xticks(range(len(keys)), names, rotation=30)
        axes[0].set(ylabel="median over a trial of the largest joint lag (rad)", title="Joint lag per trial on MuJoCo")
        axes[1].set(xlabel="joint", ylabel="|lag| at the end of the trial (rad)", title="Final lag per joint")
        n = max(len(r) for k in keys for r in [mj[k]["rows"][0].get("lag_final_rad") or []]) or 1
        axes[1].set_xticks(range(n), [str(j + 1) for j in range(n)])
        axes[1].legend([Patch(facecolor=COLORS[k[0]], alpha=0.5, hatch="//" if k[1] == "mujoco_dls" else None) for k in keys], names, fontsize=7)
        for ax in axes[:2]:
            ax.axhline(0.005, color="grey", ls=":")   # KangarooRobot tolerance
        axes[2].set(ylabel="largest joint-state age (s)", title="Joint-state staleness")
        done.append(save(plt, fig, out_dir, "mujoco_lag"))
    return done


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--data", default=os.path.join(ROOT, "docs", "kangaroo", "results", "data"))
    parser.add_argument("--out", default=os.path.join(ROOT, "docs", "kangaroo", "results"))
    args = parser.parse_args(argv)
    docs, missing, manifest = load(args.data)
    fig_dir = os.path.join(args.out, "figures")
    os.makedirs(fig_dir, exist_ok=True)
    written = figures(docs, fig_dir)
    with open(os.path.join(args.out, "tables.md"), "w", encoding="utf-8") as stream:
        stream.write(tables(docs))
    with open(os.path.join(fig_dir, "manifest.json"), "w", encoding="utf-8") as stream:
        json.dump({"commit": commit(), "date": datetime.date.today().isoformat(), "inputs": manifest, "missing": missing, "figures": written}, stream, indent=1)
    print("figures: %s; %d inputs, %d missing" % (", ".join(written) or "none", len(manifest), len(missing)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

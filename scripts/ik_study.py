#!/usr/bin/env python3
"""Test 2 of the inverse kinematics: the rule itself, with exact kinematics and no neurons (NumPy only).

    python3 -B scripts/ik_study.py --out docs/ik/results/data/study.json

For every arm 100 random start poses and goals (not the pairs of the reported trials). The
loop is closed as the spiking network closes it: the comparison sees the arm 0.2 s late,
every share carries noise, the decoded orientation is off by a fixed 1 deg, and shares
below a dead zone move nothing.  Compared: the cascade, every joint getting the whole
comparison, and turn-taking (one joint at a time); the two comparisons; dead zones; gains.
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.chains import get_chain  # noqa: E402
from ring_so3.experiments import write_json  # noqa: E402
from ring_so3.ik import rules  # noqa: E402
from ring_so3.ik.design import IkDesign  # noqa: E402
from ring_so3.ik.pairs import STUDY_SEED, pairs  # noqa: E402

MODELS = ("generic3", "generic4", "generic7", "tiago4", "tiago7")
#: as the spiking network has them (docs/results.md of the forward network): delay, noise of a product, readout error
BASE = dict(delay_s=0.2, noise=0.008, dead_zone=0.01, bias_deg=1.0)


def run(name, design, trials, label, turn_taking=False, **settings):
    chain = get_chain(name)
    conditions = dict(BASE, **settings)
    bias_deg = conditions.pop("bias_deg")
    rng = np.random.default_rng([STUDY_SEED, chain.n, 99])
    final, to5, straight, limit, start = [], [], [], [], []
    for pair in pairs(chain, trials, STUDY_SEED):
        axis = rng.normal(size=3)
        bias = rules.small_rotation(np.radians(bias_deg) * axis / np.linalg.norm(axis))
        trace = rules.simulate(chain, pair.start, pair.goal, design, readout_bias=bias, rng=rng, turn_taking=turn_taking, **conditions)
        final.append(trace.final_error_deg), to5.append(trace.time_to(5.0)), limit.append(trace.at_limit), start.append(trace.start_error_deg)
        if trace.final_error_deg <= design.reach_deg and trace.start_error_deg > 20.0:
            straight.append(trace.path_deg / trace.start_error_deg)
    final, limit = np.array(final), np.array(limit)
    missed = final > design.reach_deg
    row = {
        "model": name, "label": label, "comparison": design.comparison, "cascade": design.cascade, "turn_taking": bool(turn_taking),
        "kappa": design.kappa, "trials": int(trials), "conditions": dict(conditions, bias_deg=bias_deg),
        "reached_pct": float(100.0 * np.mean(~missed)), "final_median_deg": float(np.median(final)),
        "final_p90_deg": float(np.percentile(final, 90)), "to_5_deg_s": float(np.nanmedian(to5)) if np.any(np.isfinite(to5)) else None,
        "path_over_shortest": float(np.median(straight)) if straight else None,
        "missed": int(missed.sum()), "missed_at_limit": int((missed & limit).sum()), "start_median_deg": float(np.median(start)),
    }
    print("%-9s %-34s reached %5.1f%% | final %5.2f / %6.2f deg | to 5 deg %s s | path %s | missed %d (at a limit %d)" % (
        name, label, row["reached_pct"], row["final_median_deg"], row["final_p90_deg"],
        "%4.1f" % row["to_5_deg_s"] if row["to_5_deg_s"] is not None else "   -",
        "%.2f" % row["path_over_shortest"] if row["path_over_shortest"] else "-", row["missed"], row["missed_at_limit"]), flush=True)
    return row


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--trials", type=int, default=100)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    started = time.time()
    rows = []
    for name in MODELS:
        rows.append(run(name, IkDesign(), args.trials, "cascade"))
        rows.append(run(name, IkDesign(cascade=False, kappa=1.0), args.trials, "all joints at once"))
        rows.append(run(name, IkDesign(), args.trials, "one joint at a time", turn_taking=True))
        rows.append(run(name, IkDesign(comparison="angles"), args.trials, "cascade, angle comparison"))
    for name in ("generic7", "tiago7"):
        for dead_zone in (0.0, 0.005, 0.015, 0.02, 0.03, 0.05):
            rows.append(run(name, IkDesign(), args.trials, "cascade, dead zone %.3f" % dead_zone, dead_zone=dead_zone))
        for kappa in (0.75, 3.0, 5.0):
            rows.append(run(name, IkDesign(kappa=kappa), args.trials, "cascade, gain %.2f" % kappa))
        for delay in (0.03, 0.3, 0.4):
            rows.append(run(name, IkDesign(), args.trials, "cascade, delay %.2f s" % delay, delay_s=delay))
        rows.append(run(name, IkDesign(), args.trials, "cascade, nothing in the way", delay_s=0.0, noise=0.0, dead_zone=0.0, bias_deg=0.0))
    write_json(args.out, {"experiment": "ik_study", "seed": STUDY_SEED, "base": BASE, "design": IkDesign().describe(),
                          "rows": rows, "wall_s": round(time.time() - started, 1)})
    print("written: %s (%.0f s)" % (args.out, time.time() - started))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

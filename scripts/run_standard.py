#!/usr/bin/env python3
"""The standard methods on the test pairs of the ring IK, and standard path integration (NumPy; no NEST).

    python3 -B scripts/run_standard.py --out docs/ik/results/data/standard.json

* Inverse kinematics, step by step: the exact cascade, Jacobian transpose, damped least squares
  and cyclic coordinate descent, in the loop the ring IK runs in (same pairs, joint limits, largest
  step per joint, light robot), once with perfect sensing and once with the delay, noise, dead
  zone and readout error the ring network has.
* Inverse kinematics in one go: bounded least squares from the start pose.
* Path integration: the orientation carried along from noisy joint velocities, for 60 s.
* What forward kinematics costs.
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
from ring_so3.ik.pairs import TEST_SEED, pairs  # noqa: E402
from ring_so3.standard import fk, ik, integration  # noqa: E402

ARMS = ("generic3", "generic7", "tiago7")
CONDITIONS = {
    "perfect": dict(delay_s=0.0, noise=0.0, dead_zone=0.0, turn_clip=None, bias_deg=0.0),
    "as the network": dict(delay_s=0.2, noise=0.005, dead_zone=0.01, turn_clip=0.5, bias_deg=1.0),
}


def reach(chain, method, condition, count):
    settings = dict(CONDITIONS[condition])
    bias_deg = settings.pop("bias_deg")
    design = IkDesign(kappa=ik.GAINS[method])
    rng = np.random.default_rng([TEST_SEED, chain.n, 5])
    rows = []
    for pair in pairs(chain, count, TEST_SEED):
        axis = rng.normal(size=3)
        bias = rules.small_rotation(np.radians(bias_deg) * axis / np.linalg.norm(axis))
        trace = rules.simulate(chain, pair.start, pair.goal, design, readout_bias=bias, rng=np.random.default_rng(pair.index),
                               rule=None if method == "cascade" else ik.RULES[method], **settings)
        rows.append({"index": pair.index, "start_error_deg": trace.start_error_deg, "final_error_deg": trace.final_error_deg,
                     "reached": bool(trace.final_error_deg <= design.reach_deg), "time_to_5_deg": trace.time_to(5.0),
                     "path_ratio": trace.path_deg / trace.start_error_deg if trace.start_error_deg > 1e-9 else float("nan"),
                     "at_limit": bool(trace.at_limit), "settled_s": trace.settled_s})
    reached = [r for r in rows if r["reached"]]
    times = [r["time_to_5_deg"] for r in rows if np.isfinite(r["time_to_5_deg"])]
    return {"chain": chain.name, "method": method, "condition": condition, "kappa": design.kappa, "trials": len(rows),
            "reached": float(np.mean([r["reached"] for r in rows])),
            "final_error_reached_median_deg": float(np.median([r["final_error_deg"] for r in reached])) if reached else None,
            "time_to_5_deg_median_s": float(np.median(times)) if times else None,
            "path_ratio_median": float(np.median([r["path_ratio"] for r in reached])) if reached else None,
            "missed": len(rows) - len(reached), "missed_at_limit": int(sum(1 for r in rows if not r["reached"] and r["at_limit"])),
            "rows": rows}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--pairs", type=int, default=100)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    started = time.time()
    document = {"experiment": "standard", "pair_seed": TEST_SEED, "pairs": args.pairs, "conditions": CONDITIONS, "gains": ik.GAINS,
                "damping": ik.DAMPING, "reach": [], "solve": [], "integration": [], "fk_cost": []}
    for name in ARMS:
        chain = get_chain(name)
        document["fk_cost"].append(dict(fk.cost(chain), chain=name))
        for condition in CONDITIONS:
            for method in ("cascade",) + ik.METHODS:
                row = reach(chain, method, condition, args.pairs)
                document["reach"].append(row)
                print("%-9s %-26s %-15s reached %3.0f%% | final %s deg | 5 deg after %s s | path %s | missed %d (at a limit %d) | %.0f s" % (
                    name, ik.LABELS[method], condition, 100 * row["reached"],
                    "%.2f" % row["final_error_reached_median_deg"] if row["reached"] else "-",
                    "%.1f" % row["time_to_5_deg_median_s"] if row["time_to_5_deg_median_s"] is not None else "-",
                    "%.2f" % row["path_ratio_median"] if row["reached"] else "-", row["missed"], row["missed_at_limit"], time.time() - started), flush=True)
        errors, evaluations = [], []
        for pair in pairs(chain, args.pairs, TEST_SEED):
            _, error, count = ik.solve(chain, pair.start, rules.goal_matrix(*pair.goal))
            errors.append(error), evaluations.append(count)
        document["solve"].append({"chain": name, "trials": len(errors), "solved": float(np.mean(np.array(errors) <= 5.0)),
                                  "exact": float(np.mean(np.array(errors) <= 0.1)), "error_deg": errors,
                                  "evaluations_median": float(np.median(evaluations))})
        print("%-9s solved in one go: %.0f%% within 5 deg, %.0f%% within 0.1 deg, %d evaluations of the kinematics (median)" % (
            name, 100 * document["solve"][-1]["solved"], 100 * document["solve"][-1]["exact"], np.median(evaluations)), flush=True)
    for name in ("generic7", "tiago7"):
        chain = get_chain(name)
        for noise, bias in ((0.0, 0.0), (0.01, 0.002), (0.02, 0.005), (0.05, 0.01)):
            for correct_s in (None, 5.0, 1.0):
                runs = [integration.run(chain, 60.0, noise=noise, bias=bias, correct_s=correct_s, seed=seed) for seed in range(8)]
                summary = {"chain": name, "noise": noise, "bias": bias, "correct_s": correct_s, "seeds": len(runs),
                           "error_at_10s_deg": float(np.median([r["error_at_10s_deg"] for r in runs])),
                           "error_at_30s_deg": float(np.median([r["error_at_30s_deg"] for r in runs])),
                           "error_final_deg": float(np.median([r["error_final_deg"] for r in runs])),
                           "error_median_deg": float(np.median([r["error_median_deg"] for r in runs])),
                           "t": runs[0]["t"], "error_deg": [r["error_deg"] for r in runs]}
                document["integration"].append(summary)
                print("%-9s path integration, noise %.3f bias %.3f rad/s, correction %s: after 10 s %.1f, 30 s %.1f, 60 s %.1f deg; median %.1f deg" % (
                    name, noise, bias, "never" if correct_s is None else "every %.0f s" % correct_s, summary["error_at_10s_deg"],
                    summary["error_at_30s_deg"], summary["error_final_deg"], summary["error_median_deg"]), flush=True)
    document["wall_s"] = round(time.time() - started, 1)
    write_json(args.out, document)
    print("written: %s (%.0f s)" % (args.out, time.time() - started))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

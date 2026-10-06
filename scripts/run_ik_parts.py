#!/usr/bin/env python3
"""Tests 3a and 3b: comparison and shares of the spiking IK layer against the exact ones, arm moved by hand.

    python3 -B scripts/run_ik_parts.py --chain generic7 --out docs/ik/results/data/parts_generic7_matrix.json
    python3 -B scripts/run_ik_parts.py --chain tiago7 --comparison angles --out /out/parts_tiago7_angles.json
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.experiments import write_json  # noqa: E402
from ring_so3.ik import experiments  # noqa: E402
from ring_so3.ik.design import COMPARISONS, IkDesign  # noqa: E402
from ring_so3.ik.pairs import TEST_SEED  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--chain", default="generic7")
    parser.add_argument("--comparison", choices=COMPARISONS, default="matrix")
    parser.add_argument("--all-at-once", action="store_true", help="no cascade: every joint gets the whole comparison")
    parser.add_argument("--pairs", type=int, default=100)
    parser.add_argument("--pair-seed", type=int, default=TEST_SEED)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)

    import nest

    design = IkDesign(comparison=args.comparison, cascade=not args.all_at_once)
    document = experiments.open_loop_run(nest, args.chain, args.pairs, design, args.pair_seed, threads=args.threads)
    write_json(args.out, document)
    s = document["summary"]
    print("%s, %s: %d neurons, %.2f of real time" % (args.chain, args.comparison, document["neurons"], document["speed"]))
    print("comparison: direction error median %.1f deg, p90 %.1f deg (turn median %.2f rad); rms %.4f rad" % (
        s["turn_direction_error_median_deg"], s["turn_direction_error_p90_deg"], s["turn_size_median_rad"], s["turn_rms_rad"]))
    print("shares: right sign in %.0f%% of %d shares above %.2f rad; rms %.4f rad; %.4f rad where none is asked" % (
        100 * s["share_right_sign"], s["shares_large"], s["share_floor_rad"], s["share_rms_rad"], s["share_when_none_asked_rms_rad"] or float("nan")))
    for row in s["per_joint"]:
        print("  joint %d: rms %.4f rad, tick-to-tick sd %.4f rad, right sign %s of %d" % (
            row["joint"], row["rms_rad"], row["sd_rad"], "-" if row["right_sign"] is None else "%.0f%%" % (100 * row["right_sign"]), row["large"]))
    print("written:", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

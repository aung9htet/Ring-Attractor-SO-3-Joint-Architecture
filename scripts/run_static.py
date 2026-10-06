#!/usr/bin/env python3
"""Test 4 (and its variants): held poses.  The arm moves to each pose, holds it for 1 s, the readout of the last 0.5 s is compared with the exact kinematics.

    python3 -B scripts/run_static.py --chain generic3 --out docs/results/data/static/generic3.json
    python3 -B scripts/run_static.py --chain generic3 --mode flat --out /out/flat3.json          # all at once (test 8)
    python3 -B scripts/run_static.py --chain generic4 --silence 0.1 --out /out/silence10.json    # robustness (test 10)

300 random poses (uniform in the joint ranges, seed fixed) and a sweep of every
joint over its range with the others at one fixed pose.  Needs the calibration
of the chosen sizes, seed and encoder in artifacts/ (scripts/calibrate.py).
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3 import experiments  # noqa: E402


def add_network_arguments(parser):
    parser.add_argument("--chain", required=True, help="generic2..generic7, tiago2..tiago7")
    parser.add_argument("--mode", choices=("chain", "flat"), default="chain", help="joint by joint, or all product features at once")
    parser.add_argument("--no-relay", action="store_true", help="no link-orientation populations in the path")
    parser.add_argument("--encoder", choices=("fractional", "continuous", "corrective"), default="fractional")
    parser.add_argument("--product", type=int, default=None, help="neurons per product population")
    parser.add_argument("--entry", type=int, default=None, help="neurons per link-orientation population")
    parser.add_argument("--feature", type=int, default=None, help="neurons per cos/sin population")
    parser.add_argument("--ring", type=int, default=None, help="neurons per joint ring")
    parser.add_argument("--design-seed", type=int, default=1, help="seed of the populations' thresholds")
    parser.add_argument("--seed", type=int, default=1, help="kernel seed (the encoders' Poisson generators)")
    parser.add_argument("--noise", type=float, default=0.0, help="noise current into every neuron between the rings, pA (standard deviation)")
    parser.add_argument("--readout-tau", type=float, default=50.0, help="smoothing of the decoded matrix entries, ms")
    parser.add_argument("--threads", type=int, default=4)


def network_options(args):
    return dict(mode=args.mode, relay=not args.no_relay, encoder=args.encoder, threads=args.threads, seed=args.seed,
                noise_pa=args.noise, readout_tau_ms=args.readout_tau)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    add_network_arguments(parser)
    parser.add_argument("--poses", type=int, default=300)
    parser.add_argument("--sweep-points", type=int, default=24, help="values per joint in the sweeps (0: no sweeps)")
    parser.add_argument("--silence", type=float, default=0.0, help="share of the product neurons switched off")
    parser.add_argument("--max-speed", type=float, default=1.0, help="joint speed limit of the moves between poses, rad/s")
    parser.add_argument("--legacy-sweeps", type=int, default=0, metavar="POINTS",
                        help="two-joint chains: also the old decoder's two sweeps with this many points each")
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)

    import nest

    design = experiments.make_design(args.product, args.entry, args.feature, args.ring, args.design_seed)
    document = experiments.static_run(nest, args.chain, args.poses, args.sweep_points, design, args.silence,
                                      max_speed=args.max_speed, legacy_sweep_points=args.legacy_sweeps,
                                      **network_options(args))
    experiments.write_json(args.out, document)
    print("written: %s (%d rows, %.0f s simulated in %.0f s)" % (args.out, len(document["rows"]), document["simulated_s"], document["wall_s"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

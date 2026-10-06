#!/usr/bin/env python3
"""Step 2 of the work order: the error the network should reach, from its rate copy.

    python3 -B scripts/rate_study.py                       # the table behind the chosen sizes
    python3 -B scripts/rate_study.py --out docs/results/data/rate_study.json

For every model and several population sizes: orientation error over random
poses, with and without link-orientation populations in the path, without
noise and with noise on every rate, and for an encoder that writes between ring
indices or on the nearest index.  ``ring only`` is the error of the exact
orientation of the angles the rings hold (up to half an index of rounding per
joint for the nearest-index encoder): no decoder of those rings can do better.
"""

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.chains import MODEL_NAMES, get_chain  # noqa: E402
from ring_so3.design import Design, Sizes  # noqa: E402
from ring_so3.kinematics import orientation_error_deg  # noqa: E402
from ring_so3.rate.network import RateNetwork  # noqa: E402
from ring_so3.wiring import make_wiring  # noqa: E402


def errors(chain, wiring, design, poses, noise, seed, fractional):
    network = RateNetwork(wiring, design)
    rng = np.random.default_rng(seed)
    decoded, ring_only = [], []
    for q in poses:
        readout = network.decode(q, noise=noise, rng=rng, fractional=fractional)
        true = chain.rotation(q)
        decoded.append(orientation_error_deg(readout.tool, true))
        ring_only.append(orientation_error_deg(chain.rotation(readout.held), true))
    return np.array(decoded), np.array(ring_only)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--poses", type=int, default=200)
    parser.add_argument("--seed", type=int, default=11, help="pose seed (not the seed of the test poses)")
    parser.add_argument("--models", nargs="*", default=MODEL_NAMES)
    parser.add_argument("--out", default=None)
    args = parser.parse_args(argv)
    settings = [
        ("product 100, entry 40", Sizes(product=100, entry=40)),
        ("product 200, entry 60", Sizes(product=200, entry=60)),
        ("product 400, entry 100", Sizes(product=400, entry=100)),
    ]
    rows = []
    print("%-9s %-24s %-8s %-6s %-10s %-22s %-10s" % ("model", "sizes", "relay", "noise", "encoder", "error median/p90/max", "ring only"))
    for name in args.models:
        chain = get_chain(name)
        poses = [chain.random_pose(np.random.default_rng([args.seed, i])) for i in range(args.poses)]
        for label, sizes in settings:
            design = Design(sizes=sizes)
            for relay, noise, fractional in ((True, 0.0, True), (False, 0.0, True), (True, 0.02, True), (True, 0.0, False)):
                wiring = make_wiring(chain, "chain", relay)
                if True:
                    decoded, ring_only = errors(chain, wiring, design, poses, noise, args.seed, fractional)
                    row = {"model": name, "joints": chain.n, "sizes": label, "product": sizes.product, "entry": sizes.entry,
                           "relay": relay, "noise": noise, "fractional": fractional, "poses": len(poses),
                           "median": float(np.median(decoded)), "p90": float(np.percentile(decoded, 90)),
                           "max": float(np.max(decoded)), "ring_median": float(np.median(ring_only)),
                           "ring_p90": float(np.percentile(ring_only, 90))}
                    rows.append(row)
                    print("%-9s %-24s %-8s %-6.2f %-10s %5.2f / %5.2f / %5.2f   %5.2f / %5.2f"
                          % (name, label, "yes" if relay else "no", noise, "between" if fractional else "nearest",
                             row["median"], row["p90"], row["max"], row["ring_median"], row["ring_p90"]), flush=True)
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as stream:
            json.dump({"poses": args.poses, "seed": args.seed, "rows": rows}, stream, indent=1)
        print("written:", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

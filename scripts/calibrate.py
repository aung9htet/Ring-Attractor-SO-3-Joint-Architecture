#!/usr/bin/env python3
"""Measure every kind of population in NEST and store its readout weights in artifacts/.

    python3 -B scripts/calibrate.py                          # the default design
    python3 -B scripts/calibrate.py --product 400 --entry 100
    python3 -B scripts/calibrate.py --encoder corrective --ring 100

One run takes a few minutes.  The file name carries the sizes, the seed and the
encoder mode; a network built with the same design loads it (ring_so3/nest/network.py).
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.design import Design, Sizes  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    defaults = Sizes()
    parser.add_argument("--ring", type=int, default=defaults.ring)
    parser.add_argument("--feature", type=int, default=defaults.feature)
    parser.add_argument("--product", type=int, default=defaults.product)
    parser.add_argument("--entry", type=int, default=defaults.entry)
    parser.add_argument("--seed", type=int, default=1, help="seed of the populations' thresholds and maximum rates")
    parser.add_argument("--encoder", choices=("fractional", "continuous", "corrective"), default="fractional")
    parser.add_argument("--feature-tau", type=float, default=15.0, help="synaptic time constant of the cos/sin population, ms")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--product-points", type=int, default=25, help="grid points per input of the product rig")
    parser.add_argument("--out", default=None, help="file to write; default artifacts/<name from the design>.npz")
    args = parser.parse_args(argv)

    import nest
    from ring_so3.nest import calibrate

    design = Design(sizes=Sizes(args.ring, args.feature, args.product, args.entry, defaults.output_ring), seed=args.seed)
    calibration = calibrate.run(nest, design, args.ring, args.encoder, args.feature_tau, args.threads, args.seed,
                                product_points=args.product_points)
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    path = args.out or os.path.join(root, "artifacts", calibrate.artifact_name(design, args.ring, args.encoder))
    calibration.save(path)
    print("written:", os.path.relpath(path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Measure the readouts of the inverse-kinematics layer and store them in artifacts/ under their own name.

    python3 -B scripts/calibrate_ik.py

The forward calibration of the same design is read, not written.  Takes about a minute.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.design import Design  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--encoder", choices=("fractional", "continuous", "corrective"), default="fractional")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--out", default=None, help="file to write; default artifacts/ik_<name of the forward calibration>.npz")
    parser.add_argument("--force", action="store_true", help="write even when the file exists")
    args = parser.parse_args(argv)

    import nest
    from ring_so3.nest import calibrate, ik_calibrate

    design = Design()
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    path = args.out or os.path.join(root, "artifacts", ik_calibrate.artifact_name(design, design.sizes.ring, args.encoder))
    if os.path.exists(path) and not args.force:
        print("exists, not written again (results depend on it):", os.path.relpath(path), "- use --out for another file")
        return 1
    base = calibrate.load(design, design.sizes.ring, args.encoder)
    ik_calibrate.run(nest, base, design, args.threads).save(path)
    print("written:", os.path.relpath(path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

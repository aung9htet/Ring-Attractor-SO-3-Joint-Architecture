#!/usr/bin/env python3
"""Test 9: the old two-joint decoder of the SO-2 repo on the poses of this repo's two-joint test.

    docker/run.bash nest --so2 /path/to/Ring-Attractor-SO-2-Joint-Architecture -- \\
        'python3 -B scripts/run_old_decoder.py --out docs/results/data/old_decoder/old2.json'

It runs the frozen legacy code unchanged (``legacy/multi_ring_component.py``: class
``MultiRingDecodeAnalysis``, its fitted weights in ``config/ring_decoding_weights/``): for
every pose the network is rebuilt, the two bumps are injected and the lift, pitch and yaw
rings are read as the legacy analysis reads them.  The old decoder gives three angles, no
matrix; its orientation is the rotation built from those three angles.  Poses: the 300
random poses of ``scripts/run_static.py --chain generic2`` (same seed) and the two sweeps
the old decoder was tested on.  ``--shard I/N`` runs one of N parts; the report joins them.
"""

import argparse
import contextlib
import io
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3 import experiments  # noqa: E402
from ring_so3.chains import get_chain  # noqa: E402
from ring_so3.kinematics import circular_difference, from_lift_pitch_yaw, lift_pitch_yaw, orientation_error_deg  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--so2", default="/so2", help="checkout of the SO-2 repo")
    parser.add_argument("--poses", type=int, default=300)
    parser.add_argument("--sweep-points", type=int, default=24)
    parser.add_argument("--shard", default="1/1")
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)

    package = os.path.join(args.so2, "src", "tiago_ring_controller")
    legacy = os.path.join(package, "legacy")
    if not os.path.isdir(legacy):
        raise SystemExit("no legacy code in %s (mount the SO-2 repo with docker/run.bash nest --so2 DIR)" % args.so2)
    sys.path.insert(0, os.path.join(package, "src"))
    sys.path.insert(0, legacy)
    os.chdir(legacy)                                         # the legacy code reads ./config/...
    from multi_ring_component import MultiRingDecodeAnalysis

    chain = get_chain("generic2")
    items = [{"kind": "random", "index": i, "q": q} for i, q in enumerate(experiments.test_poses(chain, args.poses))]
    items += [{"kind": "legacy_sweep", "index": i, "sweep_joint": item["joint"], "q": item["q"]}
              for i, item in enumerate(experiments.legacy_sweep_poses(args.sweep_points))]
    part, parts = [int(v) for v in args.shard.split("/")]
    items = [item for k, item in enumerate(items) if k % parts == part - 1]

    analysis = MultiRingDecodeAnalysis(reset_kernel=True)
    rows, started = [], time.time()
    for n, item in enumerate(items):
        q = np.asarray(item["q"], dtype=float)
        with contextlib.redirect_stdout(io.StringIO()):
            decoder, delta = analysis._run_single_q(q)
        angles = [float(decoder._decode_angle_sawtooth_from_profile(delta[name])) for name in ("lift", "pitch", "yaw")]
        true = chain.rotation(q)
        true_angles = lift_pitch_yaw(true)
        row = {key: value for key, value in item.items() if key != "q"}
        row.update(
            q=q.tolist(), error_deg=orientation_error_deg(from_lift_pitch_yaw(*angles), true),
            true_lift_pitch_yaw=list(true_angles), ring_lift_pitch_yaw=angles,
            ring_angle_error_deg=[float(np.degrees(circular_difference(a, b))) for a, b in zip(angles, true_angles)],
            ring_contrast=[float(np.max(delta[name]) - np.min(delta[name])) for name in ("lift", "pitch", "yaw")],
            cos_pitch=float(np.cos(true_angles[1])),
        )
        rows.append(row)
        if (n + 1) % 10 == 0:
            print("%d / %d poses, error median %.1f deg, %.0f s" % (n + 1, len(items), np.median([r["error_deg"] for r in rows]), time.time() - started), flush=True)
    experiments.write_json(args.out, {
        "experiment": "old_decoder", "chain": "generic2", "shard": args.shard, "pose_seed": experiments.POSE_SEED,
        "neurons": 16384 + 2 * 100 + 3 * 100, "wall_s": round(time.time() - started, 1), "rows": rows,
        "note": "frozen SO-2 legacy code, MultiRingDecodeAnalysis._run_single_q; orientation = rotation of the three decoded angles",
    })
    print("written: %s (%d poses, %.0f s)" % (args.out, len(rows), time.time() - started))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

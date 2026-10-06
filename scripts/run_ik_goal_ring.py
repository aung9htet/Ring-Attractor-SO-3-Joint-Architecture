#!/usr/bin/env python3
"""Test 3c: one joint's goal ring on its own: the lead of the goal bump over the state bump against the share.

    python3 -B scripts/run_ik_goal_ring.py --scan a --out docs/ik/results/data/goal_ring_scan_a.json
    python3 -B scripts/run_ik_goal_ring.py --scan b --out docs/ik/results/data/goal_ring_scan_b.json
    python3 -B scripts/run_ik_goal_ring.py --set share_feedback=-3 --set body_feedback=-1.5 --out /out/one.json

``--scan a``: the share pair is silent without a share (the picture of architecture v4).
``--scan b``: both sides of the share pair fire at rest and the share moves them apart (push-pull).
``--scan c`` or ``--written``: no attractor in the goal ring: one gain pair, gated by the state bump,
writes beside it (ring_so3/nest/goal_ring.py, ``build_written_ring``).
Marks (fixed in the plan): no lead without a share (below 1 index); the lead rises with the share
to 6 indices or more; the right sign for a share of 0.02 rad in 90% of the trials; never two bumps.
"""

import argparse
import itertools
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.design import Design  # noqa: E402
from ring_so3.experiments import write_json  # noqa: E402

SCANS = {
    "a": {"share_feedback": (-1.5, -3.0, -6.0), "body_feedback": (-1.5, -3.0), "share_drive_pa": (3.0e5, 6.0e5)},
    "b": {"share_feedback": (-1.5, -3.0, -6.0), "body_feedback": (-1.5, -3.0), "share_drive_pa": (3.0e5, 6.0e5),
          "share_bias_pa": (1.5e5, 2.31e5), "push_pull": (True,)},
    "c": {"shift": (6, 8, 10), "rest_pa": (2.3e5, 3.04e5), "share_drive_pa": (1.5e5, 2.28e5, 3.0e5)},
}
WRITTEN = ("c",)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--scan", choices=sorted(SCANS), default=None)
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="one setting of the goal ring")
    parser.add_argument("--written", action="store_true", help="the goal ring without an attractor")
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--position", type=float, default=100.37, help="ring position of the state bump")
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)

    import nest
    from ring_so3.nest import calibrate, goal_rig, ik_calibrate
    from ring_so3.nest.goal_ring import GoalRingSettings, WrittenRingSettings

    design = Design()
    base, ik = calibrate.load(design), ik_calibrate.load(design)
    fixed = {name: json.loads(value.lower()) for name, value in (item.split("=", 1) for item in args.set)}
    grid = SCANS[args.scan] if args.scan else {}
    names = sorted(grid)
    started, runs = time.time(), []
    for values in itertools.product(*[grid[name] for name in names]):
        kind = WrittenRingSettings if (args.written or args.scan in WRITTEN) else GoalRingSettings
        settings = kind(**dict(fixed, **dict(zip(names, values))))
        rows = goal_rig.measure_lead(nest, base, ik, goal_rig.TEST_SHARES, settings, design, args.position, args.seconds, args.threads)
        marks = goal_rig.judge(rows)
        runs.append({"settings": settings.describe(), "marks": marks, "summary": goal_rig.summarise(rows),
                     "lead": [row["lead"] for row in rows]})
        print("%s %s | lead without share %+.2f, at full share %.1f, small share right %.0f%%, spread %.2f, rewritten %d%s (%.0f s)" % (
            "PASS" if marks["passed"] else "fail", dict(zip(names, values)), marks["lead_without_share"], marks["lead_at_full_share"],
            100 * marks["small_share_right_sign"], marks["spread_max"], marks["rewritten"],
            "" if marks["rising"] else ", not rising", time.time() - started), flush=True)
    write_json(args.out, {"experiment": "ik_goal_ring", "scan": args.scan, "written": bool(args.written or args.scan in WRITTEN), "fixed": fixed, "shares": list(goal_rig.TEST_SHARES),
                          "seconds": args.seconds, "position": args.position, "threads": args.threads, "runs": runs,
                          "passed": int(sum(run["marks"]["passed"] for run in runs))})
    print("written:", args.out, "- %d of %d settings pass" % (sum(run["marks"]["passed"] for run in runs), len(runs)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

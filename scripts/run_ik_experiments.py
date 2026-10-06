#!/usr/bin/env python3
"""Every closed-loop run behind docs/ik/results.md, in batches that each stay under two hours.

    python3 -B scripts/run_ik_experiments.py --list
    python3 -B scripts/run_ik_experiments.py --batch reach
    python3 -B scripts/run_ik_experiments.py --batch disturbances --max-minutes 100

A run is cut into shards (every n-th pair) that run side by side; each shard writes
``docs/ik/results/data/<run>_<i>of<n>.json``.  A shard whose file exists is not run again.
``--max-minutes``: no new trial is started after that long; pairs that were not run are listed
in the file.  The next call of the same batch runs just those pairs, into ``<run>_<i>of<n>_rest<k>.json``.
"""

import argparse
import json
import os
import subprocess
import sys
import time

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
DATA = os.path.join(ROOT, "docs", "ik", "results", "data")

# name: (arguments of run_ik_reach.py, pairs, shards)
RUNS = {
    # test 4 and 5: reaching and trajectory
    "reach_generic3": ("--chain generic3", 100, 1),
    "reach_generic7": ("--chain generic7", 100, 3),
    "reach_tiago7": ("--chain tiago7", 100, 3),
    # test 6: the other comparison, on the first 50 pairs of test 4
    "angles_generic3": ("--chain generic3 --comparison angles", 50, 1),
    "angles_generic7": ("--chain generic7 --comparison angles", 50, 1),
    "angles_tiago7": ("--chain tiago7 --comparison angles", 50, 1),
    # test 7: disturbances, on the first 25 pairs of test 4
    "weak_generic7": ("--chain generic7 --motor-scale 0.7", 25, 1),
    "strong_generic7": ("--chain generic7 --motor-scale 1.3", 25, 1),
    "push_generic7": ("--chain generic7 --push 2.0 0.3", 25, 1),
    "newgoal_generic7": ("--chain generic7 --goal-change 2.0", 25, 1),
    "weak_tiago7": ("--chain tiago7 --motor-scale 0.7", 25, 1),
    "strong_tiago7": ("--chain tiago7 --motor-scale 1.3", 25, 1),
    "push_tiago7": ("--chain tiago7 --push 2.0 0.3", 25, 1),
    "newgoal_tiago7": ("--chain tiago7 --goal-change 2.0", 25, 1),
    # test 8: goals near pitch +-90 deg, both comparisons
    "steep_generic7": ("--chain generic7 --steep 75", 20, 1),
    "steep_angles_generic7": ("--chain generic7 --steep 75 --comparison angles", 20, 1),
    # test 9: variants, on the first 25 pairs of test 4
    "allatonce_generic7": ("--chain generic7 --all-at-once --kappa 1.0", 25, 1),
    "silence10_generic7": ("--chain generic7 --silence 0.1", 25, 1),
    "noise_generic7": ("--chain generic7 --noise-pa 50", 25, 1),
    "seed2_generic7": ("--chain generic7 --seed 2", 25, 1),
    "seed3_generic7": ("--chain generic7 --seed 3", 25, 1),
}
BATCHES = {
    "reach": ("reach_generic3", "reach_generic7", "reach_tiago7"),
    "angles": ("angles_generic3", "angles_generic7", "angles_tiago7"),
    "disturbances_generic": ("weak_generic7", "strong_generic7", "push_generic7", "newgoal_generic7"),
    "disturbances_tiago": ("weak_tiago7", "strong_tiago7", "push_tiago7", "newgoal_tiago7"),
    "steep": ("steep_generic7", "steep_angles_generic7"),
    "variants": ("allatonce_generic7", "silence10_generic7", "noise_generic7", "seed2_generic7", "seed3_generic7"),
}


def shard_path(name, i, n):
    return os.path.join(DATA, "%s_%dof%d.json" % (name, i, n))


def left_over(path):
    """Pairs a shard and its later completions did not get to, and the number of the next completion."""

    with open(path, encoding="utf-8") as stream:
        left = set(json.load(stream).get("skipped", []))
    k = 1
    while os.path.exists("%s_rest%d.json" % (os.path.splitext(path)[0], k)):
        with open("%s_rest%d.json" % (os.path.splitext(path)[0], k), encoding="utf-8") as stream:
            left = set(json.load(stream).get("skipped", []))
        k += 1
    return sorted(left), k


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--batch", choices=sorted(BATCHES), default=None)
    parser.add_argument("--run", action="append", default=[], choices=sorted(RUNS))
    parser.add_argument("--jobs", type=int, default=3, help="shards side by side (jobs x threads above the free cores makes every job several times slower)")
    parser.add_argument("--threads", type=int, default=6, help="NEST threads per shard")
    parser.add_argument("--max-minutes", type=float, default=100.0, help="per shard, and for starting new shards")
    args = parser.parse_args(argv)
    if args.list:
        for batch, names in BATCHES.items():
            print(batch)
            for name in names:
                arguments, pairs, shards = RUNS[name]
                paths = [shard_path(name, i, shards) for i in range(shards)]
                done = sum(os.path.exists(path) and not left_over(path)[0] for path in paths)
                print("  %-24s %-48s %3d pairs, %d of %d shards complete" % (name, arguments, pairs, done, shards))
        return 0
    names = list(args.run) + list(BATCHES[args.batch] if args.batch else ())
    if not names:
        parser.error("give --batch or --run (or --list)")
    os.makedirs(DATA, exist_ok=True)
    queue = []
    for name in names:
        arguments, pairs, shards = RUNS[name]
        for i in range(shards):
            out = shard_path(name, i, shards)
            extra = []
            if os.path.exists(out):
                left, k = left_over(out)
                if not left:
                    print("complete, not run again:", os.path.relpath(out, ROOT))
                    continue
                out = "%s_rest%d.json" % (os.path.splitext(out)[0], k)
                extra = ["--only", ",".join(str(index) for index in left)]
                print("%d pairs left over from %s shard %d" % (len(left), name, i))
            command = [sys.executable, "-u", "-B", os.path.join(ROOT, "scripts", "run_ik_reach.py")] + arguments.split() + [
                "--pairs", str(pairs), "--shard", "%d/%d" % (i, shards), "--threads", str(args.threads),
                "--max-minutes", str(args.max_minutes), "--out", out] + extra
            queue.append((name, i, shards, command, out))
    started, running, failed = time.time(), [], 0
    while queue or running:
        for item in list(running):
            if item[0].poll() is not None:
                running.remove(item)
                failed += item[0].returncode != 0
                print("%s shard %d: %s after %.0f min" % (item[1], item[2], "done" if item[0].returncode == 0 else "FAILED", (time.time() - started) / 60.0), flush=True)
        while queue and len(running) < args.jobs:
            if (time.time() - started) / 60.0 > args.max_minutes:
                print("time is up: %d shards not started" % len(queue), flush=True)
                queue = []
                break
            name, i, shards, command, out = queue.pop(0)
            log = open(os.path.splitext(out)[0] + ".log", "w")
            running.append((subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT), name, i))
        time.sleep(5)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())

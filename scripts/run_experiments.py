#!/usr/bin/env python3
"""Every run behind docs/results.md, started side by side.

    docker/run.bash nest --so2 /path/to/Ring-Attractor-SO-2-Joint-Architecture -- \\
        'python3 -B scripts/run_experiments.py --jobs 6 --threads 4'

    python3 -B scripts/run_experiments.py --list                 # print the commands, run nothing
    python3 -B scripts/run_experiments.py --only static tracking # some groups only

Groups: ``calibrate`` (the readout weights of every design used below), ``static`` (tests 4-6:
the twelve models), ``cross`` (test 8: all at once, 2 and 3 joints), ``old`` (test 9: the old
two-joint decoder; needs ``--so2``), ``tracking`` (test 7), ``robust`` (test 10: variants on
generic4 and tiago7).  A run whose output file exists is skipped, so an interrupted
session can be started again.  Logs go to ``docs/results/data/logs/``.
"""

import argparse
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
DATA = os.path.join(ROOT, "docs", "results", "data")
GROUPS = ("calibrate", "static", "cross", "old", "tracking", "robust")
MODELS = ["generic%d" % n for n in range(2, 8)] + ["tiago%d" % n for n in range(2, 8)]
TRACKING_MODELS = ("generic3", "generic7", "tiago3", "tiago7")
TRACKING_SPEEDS = (0.25, 0.5, 1.0)
ROBUST_MODELS = ("generic4", "tiago7")
ROBUST_POSES = 100
#: name -> extra arguments of scripts/run_static.py
VARIANTS = {
    "silence05": ["--silence", "0.05"], "silence10": ["--silence", "0.10"], "silence20": ["--silence", "0.20"],
    "noise050": ["--noise", "50"], "noise150": ["--noise", "150"],
    "ring100": ["--ring", "100"],
    "seed2": ["--design-seed", "2"], "seed3": ["--design-seed", "3"],
    "nearest": ["--encoder", "continuous"], "corrective": ["--encoder", "corrective"],
    "direct": ["--no-relay"],
    "product400": ["--product", "400", "--entry", "100"],
}
#: the 7-joint arm at 0.5 rad/s with a shorter path: where the readout's delay comes from
TRACKING_VARIANTS = {
    "direct": ["--no-relay"], "tau20": ["--readout-tau", "20"], "direct_tau20": ["--no-relay", "--readout-tau", "20"],
    "lead": ["--lead"],
    # the same pair on goals that were not used to measure the delay
    "othergoals": ["--goal-seed", "777"], "othergoals_lead": ["--goal-seed", "777", "--lead"],
}
#: calibrations the runs need: arguments of scripts/calibrate.py
CALIBRATIONS = {
    "default": [], "nearest": ["--encoder", "continuous"], "corrective": ["--encoder", "corrective"],
    "seed2": ["--seed", "2"], "seed3": ["--seed", "3"], "ring100": ["--ring", "100"],
    "product400": ["--product", "400", "--entry", "100"],
}


def artifact(extra):
    """File name scripts/calibrate.py writes for these arguments (mirrors ring_so3.nest.calibrate.artifact_name)."""

    values = {"--ring": "200", "--feature": "120", "--product": "200", "--entry": "60", "--seed": "1", "--encoder": "fractional"}
    for key, value in zip(extra[::2], extra[1::2]):
        values[key] = value
    return os.path.join(ROOT, "artifacts", "calibration_N%s_f%s_p%s_e%s_seed%s_%s.npz" % (
        values["--ring"], values["--feature"], values["--product"], values["--entry"], values["--seed"], values["--encoder"]))


def plan(threads, so2):
    py = [sys.executable, "-B"]
    runs = []
    for name, extra in CALIBRATIONS.items():
        runs.append(("calibrate", "calibrate_" + name, artifact(extra),
                     py + ["scripts/calibrate.py", "--threads", str(max(threads, 8))] + extra))
    for model in MODELS:
        out = os.path.join(DATA, "static", model + ".json")
        extra = ["--legacy-sweeps", "24", "--sweep-points", "72"] if model == "generic2" else []
        runs.append(("static", "static_" + model, out, py + ["scripts/run_static.py", "--chain", model, "--threads", str(threads), "--out", out] + extra))
    for model in ("generic2", "generic3"):
        out = os.path.join(DATA, "cross", model + "_flat.json")
        runs.append(("cross", "cross_" + model, out, py + ["scripts/run_static.py", "--chain", model, "--mode", "flat", "--sweep-points", "0",
                                                           "--threads", str(threads), "--out", out]))
    for shard in range(1, 7):
        out = os.path.join(DATA, "old_decoder", "old2_%d.json" % shard)
        runs.append(("old", "old_%d" % shard, out, py + ["scripts/run_old_decoder.py", "--so2", so2, "--shard", "%d/6" % shard, "--out", out]))
    for model in TRACKING_MODELS:
        for speed in TRACKING_SPEEDS:
            out = os.path.join(DATA, "tracking", "%s_%.2f.json" % (model, speed))
            runs.append(("tracking", "tracking_%s_%.2f" % (model, speed), out,
                         py + ["scripts/run_tracking.py", "--chain", model, "--speed", str(speed), "--threads", str(threads), "--out", out]))
    for name, extra in TRACKING_VARIANTS.items():
        out = os.path.join(DATA, "tracking_variants", "tiago7_%s.json" % name)
        runs.append(("tracking", "tracking_tiago7_" + name, out,
                     py + ["scripts/run_tracking.py", "--chain", "tiago7", "--speed", "0.5", "--threads", str(threads), "--out", out] + extra))
    for model in ROBUST_MODELS:
        for name, extra in VARIANTS.items():
            out = os.path.join(DATA, "robust", "%s_%s.json" % (model, name))
            runs.append(("robust", "robust_%s_%s" % (model, name), out,
                         py + ["scripts/run_static.py", "--chain", model, "--poses", str(ROBUST_POSES), "--sweep-points", "0",
                               "--threads", str(threads), "--out", out] + extra))
    return runs


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--jobs", type=int, default=6, help="runs side by side")
    parser.add_argument("--threads", type=int, default=4, help="NEST threads per run")
    parser.add_argument("--only", nargs="*", choices=GROUPS, default=list(GROUPS))
    parser.add_argument("--so2", default="/so2", help="checkout of the SO-2 repo (group 'old')")
    parser.add_argument("--list", action="store_true", help="print the commands and stop")
    args = parser.parse_args(argv)

    runs = [r for r in plan(args.threads, args.so2) if r[0] in args.only]
    if "old" in args.only and not os.path.isdir(os.path.join(args.so2, "src", "tiago_ring_controller", "legacy")):
        print("group 'old' skipped: no SO-2 checkout at %s (docker/run.bash nest --so2 DIR)" % args.so2)
        runs = [r for r in runs if r[0] != "old"]
    if args.list:
        for group, name, out, command in runs:
            print("%-10s %s" % (group, " ".join(command[1:])))
        return 0
    logs = os.path.join(DATA, "logs")
    os.makedirs(logs, exist_ok=True)
    started = time.time()

    def current(group, out):
        """Is this output there already?  A calibration also has to be of the current format."""

        if not os.path.exists(out):
            return False
        if group != "calibrate":
            return True
        sys.path.insert(0, ROOT)
        from ring_so3.nest.calibrate import FORMAT

        with open(os.path.splitext(out)[0] + ".json", encoding="utf-8") as stream:
            return json.load(stream).get("settings", {}).get("format") == FORMAT

    def execute(run):
        group, name, out, command = run
        if current(group, out):
            return name, "exists"
        t0 = time.time()
        with open(os.path.join(logs, name + ".log"), "w", encoding="utf-8") as log:
            code = subprocess.call(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        status = "ok" if code == 0 and os.path.exists(out) else "FAILED (see docs/results/data/logs/%s.log)" % name
        print("%-28s %s  %.0f s  (%.0f s since start)" % (name, status, time.time() - t0, time.time() - started), flush=True)
        return name, status

    failed = []
    # calibrations first, one after another (each uses many threads); then everything else side by side
    for run in [r for r in runs if r[0] == "calibrate"]:
        failed += [n for n, s in [execute(run)] if s.startswith("FAILED")]
    rest = [r for r in runs if r[0] != "calibrate"]
    rest.sort(key=lambda r: 0 if ("7" in r[1] or "6" in r[1] or r[0] == "old") else 1)       # the long ones first
    with ThreadPoolExecutor(max_workers=max(args.jobs, 1)) as pool:
        for name, status in pool.map(execute, rest):
            if status.startswith("FAILED"):
                failed.append(name)
    print("done in %.0f s; %d runs, %d failed%s" % (time.time() - started, len(runs), len(failed), ": " + ", ".join(failed) if failed else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())

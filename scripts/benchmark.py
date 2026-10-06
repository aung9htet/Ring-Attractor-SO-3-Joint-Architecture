#!/usr/bin/env python3
"""How fast each model runs: simulated time per wall-clock time, with nothing else on the machine.

    python3 -B scripts/benchmark.py --out docs/results/data/speed.json

Each model is built and run for 20 s of simulated time (the arm moving from goal
to goal) with 4 and with 8 NEST threads.  A value of 1 means the network keeps
up with real time.  Run it alone: other jobs on the machine change the result.
"""

import argparse
import os
import platform
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3 import experiments  # noqa: E402
from ring_so3.chains import MODEL_NAMES  # noqa: E402
from ring_so3.sim.robot import GoalSequence  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--models", nargs="*", default=MODEL_NAMES)
    parser.add_argument("--threads", nargs="*", type=int, default=[4, 8])
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)

    import nest

    rows = []
    for name in args.models:
        for threads in args.threads:
            started = time.time()
            chain, design, network, session = experiments.build(nest, name, max_speed=0.5, threads=threads)
            build_s = time.time() - started
            session.warm_up()
            goals = GoalSequence(chain, seed=1, dwell_s=1.0)
            ticks = int(round(args.seconds / session.dt_s))
            started = time.time()
            for _ in range(ticks):
                goals.update(session.robot, session.dt_s)
                session.tick()
            wall = time.time() - started
            rows.append({"model": name, "joints": chain.n, "threads": threads, "neurons": network.neuron_count(),
                         "synapses": int(nest.GetKernelStatus("num_connections")), "build_s": build_s,
                         "simulated_s": ticks * session.dt_s, "wall_s": wall, "speed": ticks * session.dt_s / wall,
                         "ms_per_tick": 1000.0 * wall / ticks})
            print("%-9s %d threads: %6d neurons, built in %.1f s, %.2f x real time (%.0f ms per 50 ms tick)"
                  % (name, threads, rows[-1]["neurons"], build_s, rows[-1]["speed"], rows[-1]["ms_per_tick"]), flush=True)
    cpu = ""
    try:
        with open("/proc/cpuinfo", encoding="utf-8") as stream:
            cpu = next((line.split(":", 1)[1].strip() for line in stream if line.startswith("model name")), "")
    except OSError:
        pass
    experiments.write_json(args.out, {"experiment": "speed", "cpu": cpu or platform.processor(), "cores": os.cpu_count(), "rows": rows})
    print("written:", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

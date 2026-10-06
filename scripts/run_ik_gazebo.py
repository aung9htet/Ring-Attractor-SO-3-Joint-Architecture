#!/usr/bin/env python3
"""The closed loop with the TIAGo in Gazebo as the robot (run inside a container started with "gazebo up").

    docker/run.bash gazebo up ik-gazebo
    docker/run.bash gazebo exec ik-gazebo -- 'python3 -B scripts/run_ik_gazebo.py --pairs 30 --out docs/ik/results/data/gazebo_ring_0of1.json'
    docker/run.bash gazebo exec ik-gazebo -- 'python3 -B scripts/run_ik_gazebo.py --pairs 30 --method dls --out docs/ik/results/data/gazebo_dls_0of1.json'
    docker/run.bash gazebo down ik-gazebo

The first ``--pairs`` test pairs of ``tiago7``.  Physics is advanced in lock step with the network
(50 ms per tick), so the network being slower than real time does not matter.  ``--method``: ``ring``
(the spiking IK) or a standard method that reads the exact joint angles.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.experiments import write_json  # noqa: E402
from ring_so3.ik import experiments  # noqa: E402
from ring_so3.ik.design import IkDesign  # noqa: E402
from ring_so3.ik.pairs import DEVELOPMENT_SEED, TEST_SEED  # noqa: E402
from ring_so3.standard.targets import EXACT_METHODS  # noqa: E402


def main(argv=None):
    defaults = IkDesign()
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--chain", default="tiago7", choices=["tiago%d" % n for n in range(2, 8)])
    parser.add_argument("--method", default="ring", choices=("ring",) + EXACT_METHODS)
    parser.add_argument("--kappa", type=float, default=defaults.kappa)
    parser.add_argument("--pairs", type=int, default=30)
    parser.add_argument("--development", action="store_true")
    parser.add_argument("--only", default=None, metavar="I,J,...")
    parser.add_argument("--max-minutes", type=float, default=100.0)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)

    import nest
    from ring_so3.gazebo.link import RosLink
    from ring_so3.robots import GazeboRobot

    link = RosLink()
    document = experiments.reach_run(
        nest, args.chain, args.pairs, IkDesign(kappa=args.kappa), DEVELOPMENT_SEED if args.development else TEST_SEED, (0, 1),
        args.max_minutes, only=[int(v) for v in args.only.split(",")] if args.only else None,
        make_robot=lambda chain: GazeboRobot(chain, link), method=args.method, checkpoint=args.out, threads=args.threads)
    write_json(args.out, document)
    s = document["summary"]
    print("%s in Gazebo, %s: %d trials (%d not run)" % (args.chain, args.method, s["trials"], len(document["skipped"])))
    if s["trials"]:
        print("reached %.0f%% (exact rule on the light robot %.0f%%); final error of reached %.2f deg; 5 deg after %.1f s; path %.2f x shortest; missed %d (%d at a limit)" % (
            100 * s["reached"], 100 * s["exact_reached"], s["final_error_reached_median_deg"] or float("nan"),
            s["time_to_5_deg_median_s"] or float("nan"), s["path_ratio_median"] or float("nan"), s["missed"], s["missed_at_limit"]))
    print("written:", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

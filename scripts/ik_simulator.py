#!/usr/bin/env python3
"""The IK simulator: give the hand an orientation in a browser and watch the arm turn to it.

    docker/run.bash sim -- 'python3 -B scripts/ik_simulator.py --chain tiago7'
    then open http://localhost:8765/

With the Kangaroo in MuJoCo as the robot (see README 11.6):

    docker/run.bash kangaroo up ik-kangaroo
    docker/run.bash kangaroo exec ik-kangaroo -- 'python3 -B scripts/ik_simulator.py --chain kangaroo_left4 --kangaroo'
    docker/run.bash kangaroo down ik-kangaroo

With Gazebo as the robot (the TIAGo in an empty world; --gpu for the camera picture):

    docker/run.bash gazebo up ik-gazebo --gpu
    docker/run.bash gazebo exec ik-gazebo -- 'python3 -B scripts/ik_simulator.py --chain tiago7 --gazebo'
    docker/run.bash gazebo down ik-gazebo

The page shows the arm with the goal at its hand (drag a ring of the goal to turn it), a choice of
methods (the ring IK; the exact cascade; damped least squares, Jacobian transpose and cyclic
coordinate descent as standard methods), the robot (light robot, Gazebo or the Kangaroo), the distance from the
goal for the last runs side by side, every joint's share, and the spikes of all rings.

``--stub`` replaces the spiking network by the exact rule, to try the page where NEST is not
installed.  ``--ticks N`` stops after N ticks.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.chains import KANGAROO_MODELS, MODEL_NAMES  # noqa: E402
from ring_so3.design import Design  # noqa: E402
from ring_so3.ik.design import IkDesign  # noqa: E402
from ring_so3.sim.ik_server import METHODS, IkSimulator  # noqa: E402
from ring_so3.sim.server import SimServer, SimState, StubIkNetwork  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--chain", default="tiago7", choices=MODEL_NAMES + KANGAROO_MODELS)
    parser.add_argument("--method", default="ring", choices=METHODS)
    parser.add_argument("--gazebo", action="store_true", help="a Gazebo with the TIAGo runs beside this process: offer it as the robot")
    parser.add_argument("--kangaroo", action="store_true", help="the Kangaroo in MuJoCo runs beside this process (kangaroo_* chains): offer it as the robot")
    parser.add_argument("--robot", default=None, choices=("light", "gazebo", "kangaroo"), help="default: the robot started beside, else light")
    parser.add_argument("--port", type=int, default=int(os.environ.get("SIM_PORT", "8765")))
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--stub", action="store_true", help="the exact rule instead of the spiking network")
    parser.add_argument("--ticks", type=int, default=None, help="stop after this many ticks")
    args = parser.parse_args(argv)

    if args.stub:
        make_network = lambda chain, cascade=True: StubIkNetwork(chain, IkDesign(cascade=cascade))  # noqa: E731
    else:
        import nest
        from ring_so3.nest import calibrate, ik_calibrate
        from ring_so3.nest.ik_network import IkNetwork
        from ring_so3.nest.network import NetworkOptions

        design = Design()
        calibration, ik_cal = calibrate.load(design), ik_calibrate.load(design)
        make_network = lambda chain, cascade=True: IkNetwork(  # noqa: E731
            nest, chain, calibration, ik_cal, design, NetworkOptions(threads=args.threads), IkDesign(cascade=cascade))

    state = SimState()
    make_robot, robots = None, ("light",)
    if args.gazebo:
        from ring_so3.gazebo.link import RosLink
        from ring_so3.robots import GazeboRobot

        link = RosLink()
        make_robot = lambda name, chain: GazeboRobot(chain, link)  # noqa: E731
        robots = ("light", "gazebo")
        state.image = link.jpeg
    links = {}
    if args.kangaroo:
        from ring_so3.kangaroo.link import Ros2Link
        from ring_so3.robots import KangarooRobot

        def link_for(chain):                      # one link per side, opened when first needed
            side = "left" if "left" in chain.name else "right"
            if side not in links:
                links[side] = Ros2Link(side)
            return links[side]

        previous = make_robot
        make_robot = lambda name, chain: KangarooRobot(chain, link_for(chain)) if name == "kangaroo" else previous(name, chain)  # noqa: E731
        robots += ("kangaroo",)
    chain = args.chain
    robot = args.robot or ("kangaroo" if args.kangaroo and chain.startswith("kangaroo_") else "gazebo" if args.gazebo and chain.startswith("tiago") else "light")
    server = SimServer(state, args.host, args.port)
    server.start()
    simulator = IkSimulator(make_network, chain, state, make_robot=make_robot, robots=robots, robot=robot, method=args.method,
                            realtime=True)
    print("IK simulator ready: http://localhost:%d/  (chain %s, robot %s, Ctrl-C to stop)" % (server.port, chain, simulator.robot_name), flush=True)
    try:
        simulator.run_loop(args.ticks)
    except KeyboardInterrupt:
        pass
    finally:
        for close in [server.stop] + [link.close for link in links.values()]:
            try:
                close()
            except Exception as error:            # report it, and still close the rest
                print("closing failed: %r" % (error,), file=sys.stderr, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

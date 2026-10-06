#!/usr/bin/env python3
"""The simulator: change the joint angles in a browser and watch the network's readout next to the true orientation.

    docker/run.bash sim -- 'python3 -B scripts/simulator.py --chain tiago7'
    then open http://localhost:8765/

The page has a slider per joint, a mode in which the arm keeps moving from one
random goal to the next, a 3D view of the arm with the true frame (solid) and
the frame read from the network (dashed) at every link, lift / pitch / yaw over
time, and the spikes of the joint rings and the output rings.  The spiking
network runs live, one 50 ms tick at a time, in step with the robot.

``--stub`` replaces the network by exact kinematics with a little noise, to try
the page where NEST is not installed.  ``--ticks N`` stops after N ticks.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.chains import MODEL_NAMES  # noqa: E402
from ring_so3.design import Design  # noqa: E402
from ring_so3.ik.design import IkDesign  # noqa: E402
from ring_so3.sim.server import SimServer, SimState, Simulator, StubIkNetwork, StubNetwork  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--chain", default="tiago7", choices=MODEL_NAMES)
    parser.add_argument("--port", type=int, default=int(os.environ.get("SIM_PORT", "8765")))
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--speed", type=float, default=0.5, help="joint speed limit at the start, rad/s")
    parser.add_argument("--mode", choices=("manual", "goals"), default=None, help="default: goals; with --ik: manual")
    parser.add_argument("--lead", action="store_true", help="start with 'write ahead' on (the page has a switch for it)")
    parser.add_argument("--ik", action="store_true", help="with the inverse-kinematics layer: give an orientation goal and the arm reaches it")
    parser.add_argument("--stub", action="store_true", help="exact kinematics instead of the spiking network")
    parser.add_argument("--ticks", type=int, default=None, help="stop after this many ticks")
    args = parser.parse_args(argv)

    if args.stub and args.ik:
        make_network = lambda chain, cascade=True: StubIkNetwork(chain, IkDesign(cascade=cascade))  # noqa: E731
    elif args.stub:
        make_network = lambda chain: StubNetwork(chain)  # noqa: E731
    elif args.ik:
        import nest
        from ring_so3.nest import calibrate, ik_calibrate
        from ring_so3.nest.ik_network import IkNetwork
        from ring_so3.nest.network import NetworkOptions

        design = Design()
        calibration, ik_cal = calibrate.load(design), ik_calibrate.load(design)
        make_network = lambda chain, cascade=True: IkNetwork(  # noqa: E731
            nest, chain, calibration, ik_cal, design, NetworkOptions(threads=args.threads), IkDesign(cascade=cascade))
    else:
        import nest
        from ring_so3.nest import calibrate
        from ring_so3.nest.network import FkNetwork, NetworkOptions

        design = Design()
        calibration = calibrate.load(design)
        make_network = lambda chain: FkNetwork(nest, chain, calibration, design, NetworkOptions(threads=args.threads))  # noqa: E731

    state = SimState()
    server = SimServer(state, args.host, args.port)
    server.start()
    simulator = Simulator(make_network, args.chain, state, speed=args.speed, mode=args.mode or ("manual" if args.ik else "goals"), lead=args.lead)
    print("simulator ready: http://localhost:%d/  (chain %s, Ctrl-C to stop)" % (server.port, args.chain), flush=True)
    try:
        simulator.run(args.ticks)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

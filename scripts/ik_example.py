#!/usr/bin/env python3
"""The shortest way into the IK code: a 3-joint arm turns its hand to one orientation goal.

    python3 -B scripts/ik_example.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import nest  # noqa: E402

from ring_so3.ik import experiments  # noqa: E402
from ring_so3.ik.design import IkDesign  # noqa: E402
from ring_so3.ik.session import IkSession  # noqa: E402
from ring_so3.kinematics import lift_pitch_yaw  # noqa: E402


def main():
    # the forward network of the chain with the IK layer on top; calibrations come from artifacts/
    chain, network = experiments.build_ik(nest, "generic3", IkDesign(), threads=4)
    print(network.build_log[-1])
    session = IkSession(network)                     # the light robot, at the chain's rest pose
    session.warm_up()
    goal = lift_pitch_yaw(chain.rotation([0.6, -0.4, 0.8]))          # an orientation the arm can reach
    print("goal: lift %.0f, pitch %.0f, yaw %.0f deg" % tuple(np.degrees(goal)))
    trial = session.reach(goal)
    for i in range(9, len(trial.t), 10):
        print("t %4.1f s   %5.1f deg from the goal   shares (deg) %s" % (trial.t[i], trial.error_deg[i], np.round(np.degrees(trial.shares[i]), 1)))
    print("ended after %.1f s, %.1f deg from the goal; joint angles %s" % (trial.t[-1], trial.final_error_deg, np.round(trial.q[-1], 2)))


if __name__ == "__main__":
    main()

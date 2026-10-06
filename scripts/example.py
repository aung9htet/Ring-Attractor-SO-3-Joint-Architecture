#!/usr/bin/env python3
"""The example of README.md, section 7: build a network once, step it, read the result.

    docker/run.bash nest -- 'python3 -B scripts/example.py'
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import nest
import numpy as np
from ring_so3.chains import get_chain
from ring_so3.design import Design
from ring_so3.nest import calibrate
from ring_so3.nest.network import FkNetwork

chain = get_chain("tiago7")
design = Design()
network = FkNetwork(nest, chain, calibrate.load(design), design)   # built once, then kept
q = chain.limits().mean(axis=1)                                    # seven joint angles, rad
for _ in range(40):                                                # 40 ticks of 50 ms
    tick = network.step(q)
print(tick.tool_rotation())      # 3 x 3 rotation of the end effector
print(chain.rotation(q))         # the exact one
print(tick.links.shape)          # (7, 3, 3): the raw matrix of every link
print(np.degrees(tick.angles))   # lift, pitch, yaw from the output rings

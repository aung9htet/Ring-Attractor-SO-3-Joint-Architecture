#!/usr/bin/env bash
# Task 7, MuJoCo reaches, right arm: ring IK then DLS on the same 30 test pairs, each on a fresh isolated sim.
# Commit 9e3ed24 + staged work. Run from the repo root on the host.
set -u
for m in ring dls; do
  docker/run.bash kangaroo up km_right --port 8782
  docker/run.bash kangaroo exec km_right -- "python3 -B scripts/run_ik_gazebo.py --chain kangaroo_right4 --pairs 30 --method $m --max-minutes 1000 --threads 8 --out docs/kangaroo/results/data/reach/mujoco_${m}_right.json"
  docker/run.bash kangaroo down km_right
done

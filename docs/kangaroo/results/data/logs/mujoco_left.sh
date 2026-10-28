#!/usr/bin/env bash
# Task 7, MuJoCo reaches, left arm: ring IK then DLS on the same 30 test pairs, each on a fresh isolated sim.
# Commit 9e3ed24 + staged work. Run from the repo root on the host.
set -u
for m in ring dls; do
  docker/run.bash kangaroo up km_left --port 8781
  docker/run.bash kangaroo exec km_left -- "python3 -B scripts/run_ik_gazebo.py --chain kangaroo_left4 --pairs 30 --method $m --max-minutes 1000 --threads 8 --out docs/kangaroo/results/data/reach/mujoco_${m}_left.json"
  docker/run.bash kangaroo down km_left
done

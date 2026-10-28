# Task 7, light robot, right arm (commit 9e3ed24 + staged work); run in docker/run.bash kangaroo run
set -e
python3 -B scripts/run_static.py --chain kangaroo_right4 --threads 6 --out docs/kangaroo/results/data/static/kangaroo_right4.json
for v in 0.25 0.50 1.00; do python3 -B scripts/run_tracking.py --chain kangaroo_right4 --speed $v --threads 6 --out docs/kangaroo/results/data/tracking/kangaroo_right4_$v.json; done
python3 -B scripts/run_ik_reach.py --chain kangaroo_right4 --pairs 30 --threads 6 --out docs/kangaroo/results/data/reach/light_ring_right.json

# 2026-10-26 Kangaroo measurements: held poses, tracking, reaches

**Hypothesis.** The 4-joint readout of a Kangaroo arm meets the marks set for the other chains (held poses: median
≤ 5°, p90 ≤ 10°; tracking: delay ≤ 0.5 s at 0.5 rad/s, and 1 s after a stop the median error is back at or below the
held-pose p90), and reaches on the MuJoCo Kangaroo come close to reaches on the light robot (no dynamics).

**Code and data.** Commit 9e3ed24 plus the staged Kangaroo work, saved as
`docs/kangaroo/results/data/code_at_runs.patch` (`git apply` on 9e3ed24). Image and package versions:
`2026-10-26_kangaroo_plant_checks.md`. Checked-in calibration (`artifacts/`), unchanged
(`2026-10-26_kangaroo_calibration_rerun.md`). Tables: `docs/kangaroo/results/tables.md`; figures and the sha256 of
every input: `docs/kangaroo/results/figures/` (`manifest.json`); all from `scripts/kangaroo_report.py`.

## Commands

Light robot, per arm, in `docker/run.bash kangaroo run` (`docs/kangaroo/results/data/logs/light_<side>.sh`, log
beside it), 6 threads, network seed 1, pose seed 20261005, tracking goal seed 20261006, IK test-pair seed 20261026:

```bash
python3 -B scripts/run_static.py --chain kangaroo_left4 --threads 6 --out docs/kangaroo/results/data/static/kangaroo_left4.json
python3 -B scripts/run_tracking.py --chain kangaroo_left4 --speed 0.50 --threads 6 --out docs/kangaroo/results/data/tracking/kangaroo_left4_0.50.json   # also 0.25, 1.00
python3 -B scripts/run_ik_reach.py --chain kangaroo_left4 --pairs 30 --threads 6 --out docs/kangaroo/results/data/reach/light_ring_left.json
```

MuJoCo, per arm and method, on a fresh isolated sim each (`logs/mujoco_<side>.sh`), 8 threads, network seed 1, the
same 30 test pairs:

```bash
docker/run.bash kangaroo up km_left --port 8781
docker/run.bash kangaroo exec km_left -- 'python3 -B scripts/run_ik_gazebo.py --chain kangaroo_left4 --pairs 30 --method ring --max-minutes 1000 --threads 8 --out docs/kangaroo/results/data/reach/mujoco_ring_left.json'
docker/run.bash kangaroo down km_left     # then the same with --method dls -> mujoco_dls_left.json
docker/run.bash kangaroo run -- 'python3 -B scripts/kangaroo_report.py'
```

The light and MuJoCo runs differ in thread count (6 and 8); results depend slightly on it (README section 9).

## Results

Held poses (300 per arm): median / p90 / max 0.79 / 1.31 / 1.91° (left), 0.78 / 1.32 / 1.91° (right). Mark met.

Tracking (tables.md, Tracking): delay 0.18 s on both arms at every speed (mark ≤ 0.5 s at 0.5 rad/s met). 1 s after
a stop the median error is 0.74 to 0.95° over both arms and three speeds, at or below the held-pose p90 (1.31°,
1.32°): mark met. While the arm moves the error grows with speed (median 3.14°, 5.77°, 10.92° on the left at 0.25,
0.5, 1.0 rad/s); that is the 0.18 s delay: shifted by the delay, the error is 0.93 to 1.56°.

Reaches, 30 test pairs per arm, 20 s each:

| Arm | Robot, method | Reached | Final error of reached, median | Time to 5°, median | Path ratio, median |
| --- | --- | ---: | ---: | ---: | ---: |
| left | light, ring | 76.67% | 1.29° | 7.70 s | 1.31 |
| left | MuJoCo, ring | 76.67% | 1.10° | 7.60 s | 1.29 |
| left | MuJoCo, DLS | 76.67% | 0.39° | 7.60 s | 1.02 |
| right | light, ring | 76.67% | 1.48° | 7.85 s | 1.32 |
| right | MuJoCo, ring | 76.67% | 1.19° | 7.60 s | 1.29 |
| right | MuJoCo, DLS | 76.67% | 0.39° | 7.70 s | 1.02 |

The exact rule on the light robot also reaches 76.67%. On MuJoCo the ring reaches as often as on the light robot, a
little closer (1.10° and 1.19° against 1.29° and 1.48°) and as fast; the hypothesis holds for reaches. DLS ends closer
(0.39°) on a shorter path (1.02 × shortest), in a similar time to 5° (7.60 and 7.70 s).

**Misses.** 7 of 30 per run; in every miss a joint was at a limit at the end of the trial, so the table's columns
for misses away from a limit are 0. Same count, not the same pairs (test-pair indices):
- light ring and the exact rule: 2, 5, 7, 15, 19, 20, 29 (both arms);
- MuJoCo ring: 2, 5, 15, 19, 20, 28, 29 (it reaches pair 7; it misses pair 28 by a hair: 5.1° left and 5.3° right
  at the end, 5° is the mark);
- MuJoCo DLS: 0, 2, 5, 15, 19, 20, 29 (it reaches pair 7; on pair 0 it stops with a joint at a limit 113° from the
  goal on both arms, where the exact rule ends 0.15° (left) and 0.16° (right) away and the ring reaches it).

In `reach_error_over_time` the DLS median rises after about 13 s because mostly unreached trials are still running
then (7 of the 12 still running at 13 s on the left, pair 0 among them); the curves are over trials still running.

**Lock step.** Every tick advanced the simulation by exactly 100 physics steps (0.05 s). After each step the link
waits up to 1 s for a joint state stamped within the tick's last 20.5 ms; when none came, it used the newest one it had
and counted a miss: 227 of 8366 ticks (2.7%) and 239 of 8307 (2.9%) in the ring runs (left, right), 42 of 7749 (0.5%)
and 81 of 7789 (1.0%) in the DLS runs. The oldest joint state used was 62.5 to 87.5 ms old (up to 20 ms by design, at
50 Hz). The machine ran other jobs at the same time, most during the ring runs. Lock step is not bit-reproducible:
when PAL's controller receives a command inside a tick depends on DDS delivery in wall time, worth a step or two of
physics.

**Run time** (`wall_s`, `simulated_s` in the reach files; 0.058 to 0.066 of real time): ring 9.3 h (left) and 9.1 h
(right), DLS 8.1 h and 7.9 h; with start-up, 17.4 h and 17.0 h per arm for both methods (`real` in the logs).

## Negative results
- The calibration is not reproducible bit for bit in this image (`2026-10-26_kangaroo_calibration_rerun.md`).
- Every run misses 7 of 30 pairs (23%), always with a joint at a limit; six pairs (2, 5, 15, 19, 20, 29) are missed by
  every run and by the exact rule. Why (goals out of the arm's joint ranges from these starts, or the
  rules' paths running into a limit) was not tested.
- DLS can stop at a joint limit far from the goal (pair 0, 113°) where the ring and the exact rule do not.
- Lock-step runs do not repeat digit for digit (above).

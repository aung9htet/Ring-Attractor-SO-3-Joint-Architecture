# The Kangaroo migration

Branch `kangaroo-migration` (from `ik-cascade`, 2026-10-26). The robot side moves from the TIAGo in Gazebo to the two
4-joint arms of PAL's Kangaroo in MuJoCo, lock-stepped through ROS 2. The network, its calibration and the TIAGo code
are unchanged. Design: [`../superpowers/specs/2026-10-26-kangaroo-migration-design.md`](../superpowers/specs/2026-10-26-kangaroo-migration-design.md);
plan: [`../superpowers/plans/2026-10-26-kangaroo-migration.md`](../superpowers/plans/2026-10-26-kangaroo-migration.md).

## What was added

| Part | Where |
| --- | --- |
| Arm chains `kangaroo_left4`, `kangaroo_right4`, from PAL's URDF (tested against it) | `ring_so3/chains.py` |
| ROS 2 link: pause, step 100 physics steps per 50 ms tick, joint states, arm commands | `ring_so3/kangaroo/link.py` |
| The robot for the IK loop | `KangarooRobot` in `ring_so3/robots.py` |
| Container: `docker/run.bash kangaroo up NAME / exec NAME -- CMD / down NAME / run -- CMD` | `docker/run.bash`, `docker/kangaroo/` |
| Control: the IK simulator with "Kangaroo in MuJoCo" as the robot | `scripts/ik_simulator.py --kangaroo`, README section 11.6 |
| Closed-loop reaches on MuJoCo (ring or DLS) | `scripts/run_ik_gazebo.py --chain kangaroo_<side>4` |
| Tables and figures | `scripts/kangaroo_report.py` → [`results/tables.md`](results/tables.md), `results/figures/` |
| Tests | `tests/test_kangaroo_*.py` (the sim tests need `kangaroo up`) |

## Decisions

1. **Negative axes.** Joints 1 and 3 turn about −z; the chain folds them, `Rot(−z, q) = Rx(π)·Rz(q)·Rx(π)`, so joint
   angles equal the robot's. Links 2, 4 and the tool frame equal the URDF; links 1 and 3 are the URDF frames turned
   half a turn about their x axis.
2. **Rounding.** The URDF writes π/2 and π as 1.5708 and 3.14159; matrix entries within 2e-5 of 0 or ±1 are snapped
   (the plan said 1e-5; the URDF's own rounding reaches 1.17e-5). The URDF test compares at 1e-4.
3. **No command priming.** The arms get absolute targets, so there is no command state to prime (SO-2 D13); the
   robot's tolerance is set from the measured steady offset under gravity, 0.005 rad.
4. **Stale joint states are recorded, not hidden.** After each step the link waits up to 1 s for a joint state
   stamped within the tick's last 20.5 ms; when none comes it uses the newest and counts a miss. The count and the
   oldest state used go into every result file.

The owner's fallback for a sim without pause and step was not needed: the image's `mujoco_ros2_control` 0.1.2 has
both. Concurrent sims in separate containers shared one ROS graph until `ROS_LOCALHOST_ONLY=1` was set per container.

The spec's calls, as built: chain names as above; joint limits from the URDF, joint 2's upper limit narrowed to
SO-2's measured value (source in `chains.py`); PAL home is the first pose only, reaches use the test pairs' own
starts; experiments load the checked-in `artifacts/`; the pair set is not filtered and misses are classified; PAL's
gains untouched; results under `docs/kangaroo/results/`, notes under `docs/experiments/`; one container mode, not a
second script; no camera panel (MuJoCo's window shows the robot).

## Results

All numbers: [`results/tables.md`](results/tables.md). Notes with commands, seeds and interpretation:
[plant checks](../experiments/2026-10-26_kangaroo_plant_checks.md),
[calibration re-run](../experiments/2026-10-26_kangaroo_calibration_rerun.md),
[measurements](../experiments/2026-10-26_kangaroo_measurements.md).

- Held poses: median / p90 0.79 / 1.31° (left), 0.78 / 1.32° (right); marks 5° / 10° met.
- Tracking: delay 0.18 s; 1 s after a stop back to held-pose level; marks met.
- Reaches (30 test pairs per arm): 76.67% on both arms with the light robot, MuJoCo ring and MuJoCo DLS, as the exact
  rule; every miss has a joint at a limit. Final error of reached: ring 1.10° / 1.19° on MuJoCo (1.29° / 1.48° light),
  DLS 0.39°.
- Lock step: 0.5 to 2.9% of ticks used an older joint state (up to 87.5 ms old).

## Open

- The calibration re-run differs from `artifacts/` beyond the fourth digit (`decoders_feature` up to 28% of its
  largest weight, `ring_profile` 0.6 Hz in one bin); most likely from the environment the artifacts were made in
  (the re-run is deterministic in this image). Whether
  to replace `artifacts/` is the owner's call; until then everything uses the checked-in files.
- The work is staged, not committed; the data cite commit 9e3ed24 plus `results/data/code_at_runs.patch`.
- Lock-step runs are not bit-reproducible (command delivery is in wall time).
- PAL's packages are not pinned in the Dockerfile; the versions used are in the plant-checks note.

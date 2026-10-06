# The data behind the results

Everything here was written by the scripts named below, in the container. `scripts/report.py`
reads it and writes [`../tables.md`](../tables.md) and [`../figures/`](../figures/). To redo
all of it: top-level [`README.md`](../../../README.md), section 3.6.

| Path | Test | Written by | Holds |
| --- | --- | --- | --- |
| `rate_study.json` | 2 | `scripts/rate_study.py` | the rate copy's error for every model and three sets of sizes |
| `parts.json` | 3 | `scripts/run_parts.py` | every part of the network measured on its own |
| `static/<model>.json` | 4, 5, 6 | `scripts/run_static.py --chain <model>` | 300 random held poses and the joint sweeps of one model |
| `cross/<model>_flat.json` | 8 | `scripts/run_static.py --mode flat` | the same 300 poses with the all-at-once version (2 and 3 joints) |
| `old_decoder/old2_<i>.json` | 9 | `scripts/run_old_decoder.py --shard <i>/6` | the old two-joint decoder on the two-joint poses, in six parts |
| `tracking/<model>_<speed>.json` | 7 | `scripts/run_tracking.py` | every tick of a session in which the arm keeps moving |
| `tracking_variants/tiago7_<variant>.json` | 7 | `scripts/run_tracking.py` with `--no-relay`, `--readout-tau 20`, `--lead`, `--goal-seed 777` | the 7-joint arm at 0.5 rad/s: a shorter path through the network (`direct`, `tau20`, `direct_tau20`), the angles written ahead (`lead`), and both on other goals (`othergoals`, `othergoals_lead`) |
| `robust/<model>_<variant>.json` | 10 | `scripts/run_static.py` with one option | 100 poses of a variant (names: `VARIANTS` in `scripts/run_experiments.py`) |
| `speed.json` | | `scripts/benchmark.py` | simulated time per wall-clock time of every model, with 4 and 8 threads |
| `logs/` | | `scripts/run_experiments.py` | the output of every run (not kept in git) |

## The calibration file and these data

The default weights file in `artifacts/` was written again by `scripts/calibrate.py` after
these data were recorded (2026-10-05). Calibration repeats exactly now (two runs give the
same file), but the file differs in the fourth digit from the one the data were made with
(fit error of the cos / sin readout 0.01791 against 0.01792), so a rerun does not give the
recorded numbers digit for digit. Three runs were repeated with the new file to see how
much it matters:

| Run | Recorded | Repeated with the new file |
| --- | --- | --- |
| `static/generic2`, 300 poses, median / p90 / max (deg) | 0.43 / 0.78 / 1.29 | 0.44 / 0.76 / 1.14 |
| `static/tiago7`, 300 poses | 1.14 / 1.69 / 2.62 | 1.13 / 1.74 / 2.52 |
| `tracking/tiago7` at 0.5 rad/s, median error while moving (deg) | 6.85 | 6.87 |

The median of the differences pose by pose is -0.005° and -0.010°. The recorded data were kept.

## A static file

Top level: the chain, the design (sizes, neuron model, seed), the options of the network,
its size (`neurons`, `synapses`, `products`, `link_populations`), simulated and wall-clock
time, the fit errors of the calibration it used, and `rows`.

One row is one held pose:

| Field | Meaning |
| --- | --- |
| `kind`, `index` | `random` (the 300 test poses), `sweep` (with `sweep_joint`) or `legacy_sweep`; `index` counts within the kind and pairs runs on the same poses |
| `q` | joint angles, rad |
| `arrived`, `move_ticks` | whether the arm reached the pose, and how many 50 ms ticks the move took |
| `error_deg` | orientation error of the end effector against the true pose |
| `decoder_error_deg` | the same readout against the pose the encoders wrote (differs only for an encoder that rounds to an index) |
| `written_error_deg` | what rounding to a ring index alone costs (0 for the fractional encoder) |
| `held_error_deg` | exact orientation of the angles read back from the joint rings' activity, against the true pose |
| `link_error_deg` | orientation error of every link |
| `defect` | how far the raw 3 x 3 readout is from a rotation (norm of `MᵀM - I`) |
| `tool` | the raw readout of the end effector |
| `true_lift_pitch_yaw`, `ring_lift_pitch_yaw`, `ring_angle_error_deg` | true angles, the output rings' angles (null for a silent ring), their difference in degrees |
| `ring_spikes` | spikes of the three output rings in their reading window (200 ms) |
| `cos_pitch` | of the true orientation; lift and yaw are undefined where it is 0 |
| `joint_ring_error_index` | largest distance of each joint ring's bump from the written position during the measurement, in ring indices |

## A tracking file

`ticks`: one entry per 50 ms: time, joint angles, whether the arm is moving, the orientation
error at that instant, true and decoded lift / pitch / yaw, and the raw readout `tool`.
`scripts/report.py` (`tracking_summary`) derives the delay from it.

## The old decoder's files

`rows` as above with `error_deg`, the three decoded angles and their errors. The old decoder
gives three angles and no matrix; `error_deg` is for the rotation built from the three angles.

# Inverse kinematics: tables

Written by `scripts/ik_report.py` from `docs/ik/results/data/`. Do not edit by hand.

## 0. The marks, set before the runs

| Test | Mark | Result | |
| --- | --- | --- | --- |
| 3a | direction of the comparison within 10° (median) | 5.0° on the worst of 7 runs | met |
| 3b | right sign in 95% of the shares above 0.05 rad | 100% on the worst run | met |
| 3c | one joint's goal ring: no lead without a share, rising to 6 indices, right sign at 0.02 rad, one bump | settings that meet it: a 0 of 12, b 0 of 24, c 12 of 18 | met |
| 4 | generic7: 95% of the goals reached | 100% of 67 | met |
| 4 | generic7: 90% of the goals the exact rule reaches | 100% | met |
| 4 | generic7: final error of reached goals 3° (median) | 1.32° | met |
| 5 | generic7: path at most 1.3 times the shortest (median) | 1.20 | met |
| 5 | generic7: 5° within 6 s (median) | 4.5 s | met |
| 4 | tiago7: 85% of the goals reached | 94% of 83 | met |
| 4 | tiago7: 90% of the goals the exact rule reaches | 98% | met |
| 4 | tiago7: final error of reached goals 3° (median) | 1.32° | met |
| 5 | tiago7: path at most 1.3 times the shortest (median) | 1.19 | met |
| 5 | tiago7: 5° within 6 s (median) | 5.5 s | met |
| 4 | generic3: reported, no mark | 91% of 69 reached (exact rule 91%) | – |

## 1. The rule itself (exact kinematics, no neurons)

100 start and goal pairs per arm (seed 20261046, not the test pairs). Delay 0.2 s, share noise 0.008 rad, dead zone 0.01 rad, readout off by 1°. Loop gain 1.25 per second (all joints at once: 1.00).

| Arm | Rule | Reached | Final error, median | 5° after | Path / shortest | Missed (at a joint limit) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| generic3 | cascade | 89% | 1.04° | 6.8 s | 1.17 | 11 (0) |
| generic3 | all joints at once | 86% | 1.10° | 9.0 s | 1.20 | 14 (0) |
| generic3 | one joint at a time | 33% | 11.67° | 12.8 s | 1.48 | 67 (0) |
| generic3 | cascade, angle comparison | 85% | 1.05° | 7.2 s | 1.22 | 15 (0) |
| generic4 | cascade | 100% | 1.02° | 5.7 s | 1.22 | 0 (0) |
| generic4 | all joints at once | 97% | 1.01° | 7.2 s | 1.24 | 3 (0) |
| generic4 | one joint at a time | 43% | 8.63° | 13.6 s | 1.60 | 57 (0) |
| generic4 | cascade, angle comparison | 97% | 1.03° | 6.7 s | 1.29 | 3 (0) |
| generic7 | cascade | 100% | 1.00° | 3.7 s | 1.23 | 0 (0) |
| generic7 | all joints at once | 100% | 1.01° | 3.8 s | 1.31 | 0 (0) |
| generic7 | one joint at a time | 36% | 16.82° | 13.9 s | 1.67 | 64 (0) |
| generic7 | cascade, angle comparison | 100% | 0.98° | 4.5 s | 1.27 | 0 (0) |
| tiago4 | cascade | 72% | 1.07° | 6.2 s | 1.19 | 28 (28) |
| tiago4 | all joints at once | 71% | 1.10° | 8.0 s | 1.19 | 29 (29) |
| tiago4 | one joint at a time | 22% | 29.30° | 14.5 s | 1.60 | 78 (55) |
| tiago4 | cascade, angle comparison | 71% | 1.09° | 6.8 s | 1.23 | 29 (28) |
| tiago7 | cascade | 92% | 1.00° | 4.3 s | 1.21 | 8 (8) |
| tiago7 | all joints at once | 92% | 1.03° | 5.0 s | 1.25 | 8 (8) |
| tiago7 | one joint at a time | 28% | 32.56° | 13.4 s | 1.59 | 72 (63) |
| tiago7 | cascade, angle comparison | 93% | 1.03° | 4.8 s | 1.24 | 7 (7) |

What the rule needs (7 joints, cascade):

| Arm | Condition | Reached | Final error, median | 5° after | Path / shortest |
| --- | --- | ---: | ---: | ---: | ---: |
| generic7 | dead zone 0.000 | 100% | 1.00° | 3.7 s | 1.25 |
| generic7 | dead zone 0.005 | 100% | 0.99° | 3.7 s | 1.24 |
| generic7 | dead zone 0.015 | 100% | 1.03° | 3.7 s | 1.18 |
| generic7 | dead zone 0.020 | 100% | 1.03° | 3.7 s | 1.12 |
| generic7 | dead zone 0.030 | 100% | 1.39° | 3.7 s | 1.10 |
| generic7 | dead zone 0.050 | 100% | 2.61° | 3.8 s | 1.08 |
| generic7 | gain 0.75 | 100% | 1.00° | 6.0 s | 1.14 |
| generic7 | gain 3.00 | 100% | 1.02° | 2.3 s | 1.60 |
| generic7 | gain 5.00 | 100% | 1.05° | 2.4 s | 2.37 |
| generic7 | delay 0.03 s | 100% | 1.00° | 4.2 s | 1.20 |
| generic7 | delay 0.30 s | 100% | 1.01° | 3.4 s | 1.26 |
| generic7 | delay 0.40 s | 100% | 0.98° | 3.4 s | 1.31 |
| generic7 | nothing in the way | 100% | 0.11° | 4.2 s | 1.07 |
| tiago7 | dead zone 0.000 | 92% | 1.02° | 4.3 s | 1.23 |
| tiago7 | dead zone 0.005 | 92% | 1.02° | 4.3 s | 1.23 |
| tiago7 | dead zone 0.015 | 92% | 1.02° | 4.3 s | 1.16 |
| tiago7 | dead zone 0.020 | 91% | 1.05° | 4.2 s | 1.11 |
| tiago7 | dead zone 0.030 | 91% | 1.51° | 4.3 s | 1.09 |
| tiago7 | dead zone 0.050 | 89% | 2.72° | 4.2 s | 1.07 |
| tiago7 | gain 0.75 | 91% | 1.01° | 7.2 s | 1.12 |
| tiago7 | gain 3.00 | 92% | 1.04° | 2.2 s | 1.53 |
| tiago7 | gain 5.00 | 94% | 1.19° | 2.1 s | 2.19 |
| tiago7 | delay 0.03 s | 92% | 1.02° | 4.6 s | 1.19 |
| tiago7 | delay 0.30 s | 92% | 1.01° | 4.2 s | 1.24 |
| tiago7 | delay 0.40 s | 92% | 1.01° | 4.1 s | 1.28 |
| tiago7 | nothing in the way | 91% | 0.12° | 4.7 s | 1.07 |

## 2. Comparison and shares in spikes, arm held by hand (tests 3a, 3b)

100 poses each; the goal is part of the way to another pose. Marks: direction of the comparison within 10° (median); right sign in 95% of the shares above 0.05 rad.

| Arm | Comparison | Neurons | Direction error, median | p90 | Comparison error, rms | Shares above 0.05 rad | Right sign | Share error, rms | Share where none is asked, rms | 3a | 3b |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| generic3 | angles | 12,640 | 3.5° | 12.2° | 0.013 rad | 187 | 100% | 0.013 rad | 0.013 rad | met | met |
| generic3 | matrix | 15,100 | 3.1° | 9.8° | 0.013 rad | 219 | 100% | 0.012 rad | 0.011 rad | met | met |
| generic7 | angles | 33,360 | 4.8° | 12.1° | 0.021 rad | 360 | 100% | 0.016 rad | 0.012 rad | met | met |
| generic7 | matrix | 33,420 | 3.5° | 11.3° | 0.016 rad | 421 | 100% | 0.015 rad | 0.011 rad | met | met |
| generic7 | matrix, all at once | 32,960 | 3.5° | 11.3° | 0.016 rad | 608 | 100% | 0.014 rad | 0.012 rad | met | met |
| tiago7 | angles | 33,360 | 5.0° | 17.2° | 0.018 rad | 330 | 100% | 0.016 rad | 0.013 rad | met | met |
| tiago7 | matrix | 33,420 | 3.4° | 10.9° | 0.016 rad | 404 | 100% | 0.017 rad | 0.013 rad | met | met |

Share error along the arm (rms, rad): does the cascade get worse from joint to joint?

- generic3: joint 1 0.011, joint 2 0.012, joint 3 0.011
- generic7: joint 1 0.014, joint 2 0.013, joint 3 0.015, joint 4 0.015, joint 5 0.016, joint 6 0.016, joint 7 0.012
- tiago7: joint 1 0.018, joint 2 0.017, joint 3 0.019, joint 4 0.016, joint 5 0.015, joint 6 0.014, joint 7 0.018

## 3. One joint's goal ring (test 3c)

Marks: lead without a share below 1 index; lead at a full share 6 indices or more, with the right sign; rising with the share; right sign for a share of 0.02 rad in 90% of the trials (0.5 s each); never two bumps; never silent; never written again.

| Version | Settings tried | Meet all marks | Best: lead without a share | at a full share | 0.02 rad share, right sign | Bump lost and written again |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| (a) attractor ring, share pair silent without a share | 12 | 0 | -0.56 | 0.1 | 65% | 0 |
| (b) attractor ring, share pair in push-pull | 24 | 0 | -0.70 | 0.2 | 45% | 5 |
| (c) no attractor: a gain pair gated by the state bump writes beside it | 18 | 12 | 0.00 | 9.9 | 100% | 0 |

The setting that is used (shift 8, rest current 304,000 pA, share drive 228,000 pA): lead in ring indices against the share (in units of the largest share a joint takes).

| Share | -1.00 | -0.50 | -0.20 | -0.08 | +0.00 | +0.08 | +0.20 | +0.50 | +1.00 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Lead | -7.88 | -5.83 | -2.14 | -0.86 | 0.01 | 0.98 | 2.10 | 6.01 | 8.05 |

## 4. Reaching and trajectory in closed loop (tests 4, 5)

Spiking network, light robot, 100 start and goal pairs per arm (test seed). Reached: true error at most 5° at the end of the trial.

| Run | Trials | Reached | Exact rule reaches | Reached, of those the exact rule reaches | Final error of reached, median | 5° after, median | Path / shortest, median | Missed (with a joint at a limit) | Data |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| generic3 | 69 | 91% | 91% | 100% | 1.07° | 9.3 s | 1.23 | 6 (0) | 31 pairs not run (time guard) |
| generic7 | 67 | 100% | 100% | 100% | 1.32° | 4.5 s | 1.20 | 0 (0) | 1 of 3 shards missing |
| tiago7 | 83 | 94% | 96% | 98% | 1.32° | 5.5 s | 1.19 | 5 (5) | 17 pairs not run (time guard) |

| Arm | Neurons | Speed (of real time, shards side by side) | Trial length, median | Decoded orientation off by, median | Exact rule: 5° after | Exact rule: path / shortest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| generic3 | 16,900 | 0.18 | 12.8 s | 2.31° | 8.1 s | 1.16 |
| generic7 | 37,620 | 0.06 | 8.2 s | 3.18° | 3.9 s | 1.09 |
| tiago7 | 37,620 | 0.06 | 9.5 s | 2.94° | 4.5 s | 1.11 |

## 7. Hard cases (test 8)

TIAGo, 7 joints, by how the trial ended:

| Ended | Trials | Reached | Exact rule reaches | Final error, median |
| --- | ---: | ---: | ---: | ---: |
| with a joint at a limit | 40 | 88% | 92% | 1.35° |
| with no joint at a limit | 43 | 100% | 100% | 1.38° |

## 9. Ring IK beside the standard methods

The same 100 test pairs per arm, the same joint limits and the same largest step per joint, on the light robot. "As the network": the method sees the arm 0.2 s late, with share noise 0.005 rad, a dead zone of 0.01 rad, the turn bounded at 0.5 rad per component and the orientation off by 1°. "Perfect": none of that. Loop gain 1.25 per second (transpose: 1.00). Last column: of the goals this method reaches, how many the ring IK reaches too (same pairs).

| Arm | Method | Sensing | Reached | Final error of reached, median | 5° after, median | Path / shortest, median | Missed (at a limit) | Ring IK reaches of these |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| generic3 | **ring IK (spikes)** | its own | 91% of 69 | 1.07° | 9.3 s | 1.23 | 6 (0) | – |
| generic3 | damped least squares | as the network | 100% | 1.03° | 7.9 s | 1.06 | 0 (0) | 63 of 69 |
| generic3 | Jacobian transpose | as the network | 81% | 1.23° | 10.0 s | 1.16 | 19 (0) | 55 of 55 |
| generic3 | cyclic coordinate descent | as the network | 92% | 1.09° | 7.7 s | 1.19 | 8 (0) | 61 of 62 |
| generic3 | exact cascade (the ring IK's rule without neurons) | as the network | 90% | 1.09° | 7.7 s | 1.15 | 10 (0) | 62 of 62 |
| generic3 | damped least squares | perfect | 100% | 0.10° | 7.8 s | 1.03 | 0 (0) | 63 of 69 |
| generic3 | Jacobian transpose | perfect | 81% | 0.77° | 10.2 s | 1.14 | 19 (0) | 54 of 54 |
| generic3 | cyclic coordinate descent | perfect | 90% | 0.26° | 7.8 s | 1.14 | 10 (0) | 61 of 62 |
| generic3 | exact cascade (the ring IK's rule without neurons) | perfect | 90% | 0.25° | 7.8 s | 1.12 | 10 (0) | 61 of 62 |
| generic7 | **ring IK (spikes)** | its own | 100% of 67 | 1.32° | 4.5 s | 1.20 | 0 (0) | – |
| generic7 | damped least squares | as the network | 100% | 1.06° | 4.0 s | 1.06 | 0 (0) | 67 of 67 |
| generic7 | Jacobian transpose | as the network | 100% | 1.02° | 3.5 s | 1.17 | 0 (0) | 67 of 67 |
| generic7 | cyclic coordinate descent | as the network | 100% | 1.05° | 3.5 s | 1.11 | 0 (0) | 67 of 67 |
| generic7 | exact cascade (the ring IK's rule without neurons) | as the network | 100% | 1.01° | 3.9 s | 1.09 | 0 (0) | 67 of 67 |
| generic7 | damped least squares | perfect | 100% | 0.21° | 4.2 s | 1.05 | 0 (0) | 67 of 67 |
| generic7 | Jacobian transpose | perfect | 100% | 0.06° | 3.7 s | 1.10 | 0 (0) | 67 of 67 |
| generic7 | cyclic coordinate descent | perfect | 100% | 0.11° | 4.0 s | 1.07 | 0 (0) | 67 of 67 |
| generic7 | exact cascade (the ring IK's rule without neurons) | perfect | 100% | 0.11° | 4.1 s | 1.06 | 0 (0) | 67 of 67 |
| tiago7 | **ring IK (spikes)** | its own | 94% of 83 | 1.32° | 5.5 s | 1.19 | 5 (5) | – |
| tiago7 | damped least squares | as the network | 94% | 1.05° | 5.4 s | 1.07 | 6 (6) | 78 of 79 |
| tiago7 | Jacobian transpose | as the network | 91% | 1.04° | 4.8 s | 1.18 | 9 (9) | 77 of 77 |
| tiago7 | cyclic coordinate descent | as the network | 93% | 1.04° | 4.3 s | 1.15 | 7 (7) | 77 of 78 |
| tiago7 | exact cascade (the ring IK's rule without neurons) | as the network | 94% | 1.02° | 4.5 s | 1.12 | 6 (6) | 78 of 80 |
| tiago7 | damped least squares | perfect | 94% | 0.17° | 5.6 s | 1.06 | 6 (6) | 78 of 79 |
| tiago7 | Jacobian transpose | perfect | 93% | 0.13° | 5.0 s | 1.15 | 7 (7) | 77 of 78 |
| tiago7 | cyclic coordinate descent | perfect | 93% | 0.12° | 4.7 s | 1.10 | 7 (7) | 77 of 78 |
| tiago7 | exact cascade (the ring IK's rule without neurons) | perfect | 95% | 0.11° | 4.8 s | 1.10 | 5 (5) | 78 of 80 |

Solving for the joint angles in one go (bounded least squares from the start pose; no trajectory):

| Arm | Solved within 0.1° | Within 5° | Evaluations of the kinematics, median |
| --- | ---: | ---: | ---: |
| generic3 | 100% | 100% | 8 |
| generic7 | 100% | 100% | 21 |
| tiago7 | 97% | 97% | 8 |

## 10. Forward kinematics: the ring network beside the exact computation

The exact orientation is a product of 3 x 3 matrices in double precision: no error, no delay. The ring network's numbers are those of the forward results (`docs/results/`), 300 held poses and a moving arm at 0.5 rad/s.

| Arm | Exact: sines and cosines | Exact: multiplications | Network: neurons | Network: product populations | Network: error at a held pose, median | p90 | Network: error while moving at 0.5 rad/s, median |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| generic3 | 6 | 117 | 4,420 | 14 | 0.60° | 1.04° | 5.77° |
| generic7 | 14 | 273 | 16,740 | 62 | 1.06° | 1.80° | 9.14° |
| tiago7 | 14 | 273 | 16,740 | 62 | 1.14° | 1.69° | 6.85° |

## 11. Path integration: orientation from joint velocities

The standard way to follow an orientation without position sensing: integrate the joint velocities. Arm moving from goal to goal at up to 0.5 rad/s for 60 s; every velocity reading has noise and a constant offset; median over 8 runs. The ring network reads joint positions, so its error does not grow; its row is the forward network's tracking run at the same speed.

| Arm | Method | Velocity noise, offset (rad/s) | Corrected | Error after 10 s | 30 s | 60 s | Median over the run | Largest |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| generic7 | integration | 0.000, 0.000 | never | 0.2° | 0.4° | 0.4° | 0.4° | 0.7° |
| generic7 | integration | 0.000, 0.000 | every 5 s | – | – | – | 0.1° | 0.2° |
| generic7 | integration | 0.000, 0.000 | every 1 s | – | – | – | 0.0° | 0.1° |
| generic7 | integration | 0.010, 0.002 | never | 3.1° | 10.5° | 16.2° | 7.8° | 24.2° |
| generic7 | integration | 0.010, 0.002 | every 5 s | – | – | – | 0.8° | 2.6° |
| generic7 | integration | 0.010, 0.002 | every 1 s | – | – | – | 0.2° | 0.8° |
| generic7 | integration | 0.020, 0.005 | never | 7.6° | 25.9° | 41.0° | 19.0° | 58.2° |
| generic7 | integration | 0.020, 0.005 | every 5 s | – | – | – | 1.9° | 6.4° |
| generic7 | integration | 0.020, 0.005 | every 1 s | – | – | – | 0.5° | 1.8° |
| generic7 | integration | 0.050, 0.010 | never | 15.3° | 51.5° | 85.1° | 37.5° | 112.0° |
| generic7 | integration | 0.050, 0.010 | every 5 s | – | – | – | 4.1° | 13.2° |
| generic7 | integration | 0.050, 0.010 | every 1 s | – | – | – | 1.1° | 4.0° |
| tiago7 | integration | 0.000, 0.000 | never | 0.2° | 0.4° | 0.2° | 0.3° | 0.7° |
| tiago7 | integration | 0.000, 0.000 | every 5 s | – | – | – | 0.1° | 0.3° |
| tiago7 | integration | 0.000, 0.000 | every 1 s | – | – | – | 0.0° | 0.2° |
| tiago7 | integration | 0.010, 0.002 | never | 2.9° | 10.9° | 22.9° | 9.0° | 27.5° |
| tiago7 | integration | 0.010, 0.002 | every 5 s | – | – | – | 0.7° | 2.5° |
| tiago7 | integration | 0.010, 0.002 | every 1 s | – | – | – | 0.2° | 0.8° |
| tiago7 | integration | 0.020, 0.005 | never | 7.3° | 27.1° | 58.3° | 21.2° | 68.7° |
| tiago7 | integration | 0.020, 0.005 | every 5 s | – | – | – | 1.7° | 6.2° |
| tiago7 | integration | 0.020, 0.005 | every 1 s | – | – | – | 0.5° | 1.7° |
| tiago7 | integration | 0.050, 0.010 | never | 14.8° | 54.9° | 116.3° | 42.3° | 135.8° |
| tiago7 | integration | 0.050, 0.010 | every 5 s | – | – | – | 3.7° | 13.1° |
| tiago7 | integration | 0.050, 0.010 | every 1 s | – | – | – | 1.1° | 3.8° |
| generic7 | **ring network** (positions) | – | not needed | 8.6° (first third) | 7.8° (second) | 7.2° (last) | 7.9° | 25.6° |
| tiago7 | **ring network** (positions) | – | not needed | 6.8° (first third) | 5.3° (second) | 6.0° (last) | 6.2° | 21.7° |

# Tables of the results

Written by `scripts/report.py` from `docs/results/data/`; do not edit by hand. Orientation error is the angle of
the single rotation that turns the decoded orientation into the true one, in degrees. A readout that knows nothing
would be 126.5 deg off on average. Median / p90 / max: half of the poses are at or below the median, 90% at or below
p90. Reading and figures: [`../results.md`](../results.md).

## 0. The parts on their own (test 3)

Each part measured in NEST on inputs its readout was not fitted on (`scripts/run_parts.py`).

| Part | Measured on | Result | Pass mark | Met |
| --- | --- | --- | --- | --- |
| cos / sin population | 80 ring positions between indices | largest error of cos or sin 0.018; angle of the pair off by 0.17 deg (median) / 1.00 deg (largest); length off by at most 0.019 | within 0.03 | yes |
| product population | 300 random pairs | root mean square error 0.0077, largest 0.041 | rms at most 0.02 | yes |
| link-orientation population | 150 random values | root mean square error 0.0062, largest 0.026 | (none set) | yes |
| length population (pitch ring) | 200 random pairs in the unit disc | root mean square error 0.0143, largest 0.043 | (none set) | yes |
| one stage: direction | 64 vectors longer than 0.3, two-joint chain | direction off by 0.30 deg (median) / 1.87 deg (largest) | within 2 deg | yes |
| one stage: length | the same vectors | length off by at most 0.029; as a share of the vector's own length at most 4.7% | within 3% of its length | NO |

The joint ring itself (from the calibration run): a held bump has 27 active neurons and 1014 spikes per second in all; the centre of its activity is -0.01 ring indices from the written position on average, 0.35 from tick to tick (standard deviation).

Fit error of the readouts on their own calibration points (root mean square): link population 0.0081, product 0.0112, length 0.0157, cos / sin 0.0179 per 50 ms tick.

A joint ring's bump behind an angle that moves at a steady speed (one ring, encoder at its usual rate and four times as strong):

| Encoder rate (Hz) | Speed (ring indices per tick) | Bump behind by (indices) | The same in time (s) |
| ---: | ---: | ---: | ---: |
| 200 | 0.5 | 0.89 | 0.089 |
| 200 | 1.5 | 2.67 | 0.089 |
| 200 | 3.0 | 5.51 | 0.092 |
| 800 | 0.5 | 0.72 | 0.072 |
| 800 | 1.5 | 2.27 | 0.076 |
| 800 | 3.0 | 4.76 | 0.079 |

## 1. The models

Sizes of the spiking network and how fast it runs.

| Model | Joints | Products | Link populations | Neurons | Synapses | Speed with 4 threads | Speed with 8 threads |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| generic2 | 2 | 4 | 0 | 1,860 | 458,540 | 8.11 | 9.28 |
| generic3 | 3 | 14 | 4 | 4,420 | 1,130,780 | 3.75 | 4.79 |
| generic4 | 4 | 26 | 10 | 7,500 | 1,949,380 | 2.10 | 2.87 |
| generic5 | 5 | 38 | 16 | 10,580 | 2,615,380 | 1.46 | 2.04 |
| generic6 | 6 | 50 | 22 | 13,660 | 3,256,320 | 1.08 | 1.52 |
| generic7 | 7 | 62 | 28 | 16,740 | 3,897,260 | 0.78 | 1.14 |
| tiago2 | 2 | 4 | 0 | 1,860 | 448,140 | 8.08 | 8.92 |
| tiago3 | 3 | 14 | 4 | 4,420 | 1,119,980 | 3.77 | 4.74 |
| tiago4 | 4 | 26 | 10 | 7,500 | 1,932,580 | 2.12 | 2.81 |
| tiago5 | 5 | 38 | 16 | 10,580 | 2,615,380 | 1.47 | 2.01 |
| tiago6 | 6 | 50 | 22 | 13,660 | 3,256,320 | 1.09 | 1.50 |
| tiago7 | 7 | 62 | 28 | 16,740 | 3,897,260 | 0.76 | 1.17 |

Speed: simulated time per wall-clock time with the arm moving, measured alone on the machine (AMD Ryzen 9 7900X 12-Core Processor, 24 logical cores; `scripts/benchmark.py`). 1.00 means the network keeps up with real time.

## 2. What the readout costs

Counted from the wiring, no simulation. The old decoder used a 32 x 32 grid of cells for each of 16 signed features of two joints.

| Joints | Possible product features (3^n - 1) | Features that occur | Pairwise products, all at once | Pairwise products, joint by joint | Cells of the old decoder's design (16 x 32^n) |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 2 | 8 | 8 | 4 | 4 | 16,384 |
| 3 | 26 | 13 | 14 | 14 | 524,288 |
| 4 | 80 | 21 | 30 | 26 | 16,777,216 |
| 5 | 242 | 34 | 56 | 38 | 5.4e+08 |
| 6 | 728 | 55 | 98 | 50 | 1.7e+10 |
| 7 | 2,186 | 89 | 166 | 62 | 5.5e+11 |

## 3. Held poses (test 4)

300 random poses per model; each is reached by moving the arm, held for 1 s, and the readout of the last 0.5 s is compared with the exact kinematics. *Rings alone*: the exact orientation of the angles read back from the joint rings' activity, against the true pose: what the rings themselves lose.

| Model | Poses | Error, median / p90 / max (deg) | 95% interval of the median | Within 5 deg | Within 10 deg | Similar (median <= 5, p90 <= 10) | Rings alone, median / p90 (deg) | Not arrived |
| --- | ---: | --- | --- | ---: | ---: | --- | --- | ---: |
| generic2 | 300 | 0.43 / 0.78 / 1.29 | 0.41 - 0.47 | 100% | 100% | yes | 0.17 / 0.33 | 0 |
| generic3 | 300 | 0.60 / 1.04 / 1.44 | 0.56 - 0.65 | 100% | 100% | yes | 0.21 / 0.38 | 0 |
| generic4 | 300 | 0.77 / 1.22 / 2.06 | 0.71 - 0.81 | 100% | 100% | yes | 0.25 / 0.44 | 0 |
| generic5 | 300 | 0.91 / 1.45 / 2.24 | 0.84 - 0.97 | 100% | 100% | yes | 0.28 / 0.49 | 0 |
| generic6 | 300 | 0.97 / 1.57 / 2.48 | 0.93 - 1.03 | 100% | 100% | yes | 0.31 / 0.53 | 0 |
| generic7 | 300 | 1.06 / 1.80 / 2.74 | 0.99 - 1.12 | 100% | 100% | yes | 0.34 / 0.59 | 0 |
| tiago2 | 300 | 0.44 / 0.75 / 1.26 | 0.41 - 0.46 | 100% | 100% | yes | 0.10 / 0.19 | 0 |
| tiago3 | 300 | 0.62 / 1.10 / 1.72 | 0.58 - 0.64 | 100% | 100% | yes | 0.16 / 0.32 | 0 |
| tiago4 | 300 | 0.80 / 1.29 / 1.96 | 0.74 - 0.85 | 100% | 100% | yes | 0.19 / 0.35 | 0 |
| tiago5 | 300 | 0.91 / 1.47 / 2.63 | 0.84 - 0.98 | 100% | 100% | yes | 0.23 / 0.42 | 0 |
| tiago6 | 300 | 1.03 / 1.66 / 2.48 | 0.95 - 1.09 | 100% | 100% | yes | 0.26 / 0.45 | 0 |
| tiago7 | 300 | 1.14 / 1.69 / 2.62 | 1.07 - 1.21 | 100% | 100% | yes | 0.28 / 0.48 | 0 |

## 4. Error of every link (test 5)

Median orientation error of each link's frame over the same poses (deg).

| Model | Link 1 | 2 | 3 | 4 | 5 | 6 | 7 | End effector |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| generic2 | 0.26 | 0.43 | - | - | - | - | - | 0.43 |
| generic3 | 0.23 | 0.45 | 0.60 | - | - | - | - | 0.60 |
| generic4 | 0.20 | 0.43 | 0.59 | 0.77 | - | - | - | 0.77 |
| generic5 | 0.24 | 0.43 | 0.61 | 0.77 | 0.91 | - | - | 0.91 |
| generic6 | 0.25 | 0.45 | 0.60 | 0.76 | 0.89 | 0.97 | - | 0.97 |
| generic7 | 0.21 | 0.47 | 0.60 | 0.74 | 0.85 | 0.98 | 1.06 | 1.06 |
| tiago2 | 0.24 | 0.44 | - | - | - | - | - | 0.44 |
| tiago3 | 0.26 | 0.46 | 0.62 | - | - | - | - | 0.62 |
| tiago4 | 0.24 | 0.44 | 0.60 | 0.80 | - | - | - | 0.80 |
| tiago5 | 0.24 | 0.45 | 0.62 | 0.79 | 0.91 | - | - | 0.91 |
| tiago6 | 0.25 | 0.44 | 0.64 | 0.83 | 0.93 | 1.03 | - | 1.03 |
| tiago7 | 0.23 | 0.43 | 0.60 | 0.81 | 0.90 | 1.01 | 1.14 | 1.14 |

Growth per joint from link 2 to link 7 (difference of the medians divided by 5): generic7 0.12 deg, tiago7 0.14 deg.

## 5. Output rings (test 6)

Angle of each output ring (centre of its activity) against the true lift, pitch and yaw, for the poses where lift and yaw are defined well.

| Model | Poses with cos(pitch) > 0.3 | Lift ring error, median / p90 / max (deg) | Pitch ring | Yaw ring | p90 of the three <= 10 deg |
| --- | ---: | --- | --- | --- | --- |
| generic2 | 294 of 300 | 0.37 / 0.87 / 1.80 | 0.35 / 0.87 / 2.22 | 0.34 / 0.93 / 2.36 | yes |
| generic3 | 269 of 300 | 0.50 / 1.27 / 4.13 | 0.47 / 1.18 / 2.02 | 0.44 / 1.21 / 3.13 | yes |
| generic4 | 278 of 300 | 0.60 / 1.57 / 3.51 | 0.43 / 1.21 / 2.57 | 0.42 / 1.17 / 3.38 | yes |
| generic5 | 284 of 300 | 0.69 / 1.58 / 4.71 | 0.56 / 1.53 / 3.30 | 0.62 / 1.79 / 6.54 | yes |
| generic6 | 284 of 300 | 0.68 / 1.87 / 4.62 | 0.63 / 1.57 / 3.25 | 0.74 / 2.00 / 9.72 | yes |
| generic7 | 281 of 300 | 0.80 / 2.37 / 4.27 | 0.69 / 1.83 / 3.07 | 0.76 / 1.90 / 4.32 | yes |
| tiago2 | 265 of 300 | 0.00 / 0.00 / 0.00 | 0.45 / 1.06 / 1.89 | 0.35 / 0.97 / 1.62 | yes |
| tiago3 | 286 of 300 | 0.36 / 0.87 / 1.89 | 0.45 / 1.09 / 2.35 | 0.45 / 1.36 / 3.20 | yes |
| tiago4 | 282 of 300 | 0.51 / 1.35 / 4.15 | 0.56 / 1.27 / 2.30 | 0.64 / 1.58 / 3.32 | yes |
| tiago5 | 287 of 300 | 0.58 / 1.54 / 5.44 | 0.57 / 1.59 / 2.57 | 0.74 / 1.68 / 4.14 | yes |
| tiago6 | 281 of 300 | 0.55 / 1.87 / 4.10 | 0.65 / 1.68 / 3.01 | 0.83 / 2.05 / 4.91 | yes |
| tiago7 | 285 of 300 | 0.83 / 2.01 / 5.53 | 0.62 / 1.61 / 2.56 | 0.79 / 2.03 / 5.37 | yes |

tiago2: the first two joints of the TIAGo arm cannot change lift, so its lift ring is driven by a constant and shows no error.

## 6. Output rings near pitch = 90 deg

All models and all poses (random and sweeps) pooled, by cos(pitch). Where cos(pitch) is small, lift and yaw are not defined; the matrix stays right.

| cos(pitch) | Poses | Lift ring error, median (deg) | Yaw ring error, median (deg) | Lift ring spikes, median | Pitch ring error, median (deg) | Matrix error, median (deg) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.0 - 0.1 | 43 | silent (silent in 100%) | silent | 0 | 2.88 | 0.59 |
| 0.1 - 0.2 | 95 | 4.95 (silent in 99%) | silent | 0 | 0.97 | 0.77 |
| 0.2 - 0.3 | 162 | 1.42 (silent in 24%) | 1.69 | 22 | 0.72 | 0.70 |
| 0.3 - 0.5 | 559 | 1.08 | 1.00 | 237 | 0.63 | 0.78 |
| 0.5 - 0.8 | 1364 | 0.64 | 0.66 | 574 | 0.59 | 0.78 |
| 0.8 - 1.0 | 2817 | 0.43 | 0.49 | 857 | 0.49 | 0.76 |

## 7. Sweeps of single joints

One joint over its whole range, the others at one fixed pose (24 values per joint; 72 for generic2).

| Model | Swept joint | Poses | Error, median / max (deg) |
| --- | ---: | ---: | --- |
| generic2 | 1 | 72 | 0.29 / 1.07 |
| generic2 | 2 | 72 | 0.41 / 1.00 |
| generic3 | 1 | 24 | 0.80 / 1.19 |
| generic3 | 2 | 24 | 0.48 / 0.84 |
| generic3 | 3 | 24 | 0.51 / 1.38 |
| generic4 | 1 | 24 | 0.58 / 1.09 |
| generic4 | 2 | 24 | 0.55 / 1.19 |
| generic4 | 3 | 24 | 0.53 / 1.14 |
| generic4 | 4 | 24 | 0.56 / 1.12 |
| generic5 | 1 | 24 | 1.37 / 2.21 |
| generic5 | 2 | 24 | 1.13 / 1.58 |
| generic5 | 3 | 24 | 1.14 / 1.90 |
| generic5 | 4 | 24 | 1.65 / 2.23 |
| generic5 | 5 | 24 | 1.42 / 2.40 |
| generic6 | 1 | 24 | 0.86 / 1.75 |
| generic6 | 2 | 24 | 0.98 / 1.60 |
| generic6 | 3 | 24 | 0.79 / 1.54 |
| generic6 | 4 | 24 | 0.95 / 1.59 |
| generic6 | 5 | 24 | 0.52 / 1.40 |
| generic6 | 6 | 24 | 0.67 / 1.28 |
| generic7 | 1 | 24 | 0.85 / 1.54 |
| generic7 | 2 | 24 | 0.81 / 1.69 |
| generic7 | 3 | 24 | 0.94 / 2.34 |
| generic7 | 4 | 24 | 1.19 / 1.98 |
| generic7 | 5 | 24 | 1.12 / 2.14 |
| generic7 | 6 | 24 | 1.02 / 1.57 |
| generic7 | 7 | 24 | 0.74 / 1.47 |
| tiago2 | 1 | 24 | 0.37 / 0.75 |
| tiago2 | 2 | 24 | 0.47 / 1.03 |
| tiago3 | 1 | 24 | 0.42 / 0.91 |
| tiago3 | 2 | 24 | 0.39 / 0.93 |
| tiago3 | 3 | 24 | 0.61 / 1.45 |
| tiago4 | 1 | 24 | 0.60 / 1.19 |
| tiago4 | 2 | 24 | 0.45 / 1.69 |
| tiago4 | 3 | 24 | 0.47 / 1.27 |
| tiago4 | 4 | 24 | 0.40 / 1.03 |
| tiago5 | 1 | 24 | 1.29 / 2.02 |
| tiago5 | 2 | 24 | 1.52 / 2.13 |
| tiago5 | 3 | 24 | 1.11 / 1.86 |
| tiago5 | 4 | 24 | 1.12 / 2.11 |
| tiago5 | 5 | 24 | 1.01 / 1.72 |
| tiago6 | 1 | 24 | 1.22 / 2.09 |
| tiago6 | 2 | 24 | 0.90 / 1.85 |
| tiago6 | 3 | 24 | 1.09 / 2.01 |
| tiago6 | 4 | 24 | 1.46 / 2.65 |
| tiago6 | 5 | 24 | 1.17 / 2.03 |
| tiago6 | 6 | 24 | 1.16 / 2.33 |
| tiago7 | 1 | 24 | 1.11 / 1.64 |
| tiago7 | 2 | 24 | 0.84 / 2.07 |
| tiago7 | 3 | 24 | 1.04 / 2.14 |
| tiago7 | 4 | 24 | 1.00 / 2.03 |
| tiago7 | 5 | 24 | 0.82 / 2.00 |
| tiago7 | 6 | 24 | 1.04 / 1.55 |
| tiago7 | 7 | 24 | 0.76 / 1.12 |

## 8. A moving arm (test 7)

The arm goes from one random goal to the next and rests 1 s at each. *Delay*: the shift in time for which the readout matches the true orientation best while the arm moves. *Error at rest* and *1 s after a stop* are single 50 ms readings (a held pose in section 3 is the mean of ten). Pass marks: delay at most 0.5 s at 0.5 rad/s; 1 s after a stop the error is back at the level of held poses, taken here as: its median is at most the 90th percentile of the same model's held poses.

| Model | Joint speed limit (rad/s) | Ticks | Delay (s) | Error while moving, median / p90 (deg) | Error at rest, median / p90 (deg) | Error 1 s after a stop, median / max (deg) | Held poses of the same model, p90 (deg) | Delay <= 0.5 s | Back at the held-pose level 1 s after a stop |
| --- | ---: | ---: | ---: | --- | --- | --- | ---: | --- | --- |
| generic3 | 0.25 | 4138 | 0.16 | 3.00 / 4.83 | 0.62 / 1.21 | 0.57 / 1.76 | 1.04 | yes | yes |
| generic3 | 0.50 | 2358 | 0.16 | 5.77 / 9.48 | 0.68 / 1.15 | 0.70 / 1.67 | 1.04 | yes | yes |
| generic3 | 1.00 | 1485 | 0.16 | 10.58 / 18.11 | 0.67 / 1.21 | 0.64 / 1.29 | 1.04 | yes | yes |
| generic7 | 0.25 | 4753 | 0.20 | 4.80 / 8.32 | 1.14 / 1.87 | 1.09 / 1.91 | 1.80 | yes | yes |
| generic7 | 0.50 | 2661 | 0.20 | 9.14 / 16.15 | 1.07 / 1.75 | 1.08 / 2.62 | 1.80 | yes | yes |
| generic7 | 1.00 | 1634 | 0.20 | 16.26 / 30.87 | 1.13 / 1.81 | 1.35 / 2.19 | 1.80 | yes | yes |
| tiago3 | 0.25 | 4134 | 0.17 | 2.91 / 4.73 | 0.73 / 1.37 | 0.64 / 1.62 | 1.10 | yes | yes |
| tiago3 | 0.50 | 2354 | 0.16 | 5.35 / 9.14 | 0.78 / 1.36 | 0.78 / 1.46 | 1.10 | yes | yes |
| tiago3 | 1.00 | 1482 | 0.17 | 9.96 / 17.52 | 0.67 / 1.37 | 0.71 / 1.53 | 1.10 | yes | yes |
| tiago7 | 0.25 | 4860 | 0.20 | 3.88 / 8.06 | 1.08 / 1.69 | 1.20 / 1.77 | 1.69 | yes | yes |
| tiago7 | 0.50 | 2715 | 0.20 | 6.85 / 15.63 | 1.14 / 1.62 | 1.19 / 1.67 | 1.69 | yes | yes |
| tiago7 | 1.00 | 1663 | 0.20 | 12.59 / 30.08 | 1.12 / 1.63 | 0.93 / 1.87 | 1.69 | yes | yes |

## 8b. Where the delay comes from

tiago7 at 0.5 rad/s with a shorter path through the network. What remains with both changes is mostly the joint rings' own lag behind a moving angle and the cos / sin populations.

| Variant | Delay (s) | Error while moving, median (deg) | The same against the true orientation one delay earlier (deg) | Error at rest, median / p90 (deg) |
| --- | ---: | ---: | ---: | --- |
| as built (link populations in the path, readout smoothed with 50 ms) | 0.20 | 6.85 | 1.46 | 1.14 / 1.62 |
| no link populations in the path | 0.18 | 6.32 | 1.61 | 1.03 / 1.68 |
| readout smoothed with 20 ms | 0.16 | 6.11 | 1.77 | 1.28 / 2.08 |
| both | 0.15 | 5.49 | 1.51 | 1.26 / 2.02 |
| as built, but the encoders write the angle each joint will have one delay later | 0.03 | 1.80 | 1.64 | 1.10 / 1.59 |
| as built, other goals | 0.20 | 7.14 | 1.49 | 1.22 / 1.87 |
| writing ahead, other goals | 0.03 | 1.87 | 1.59 | 1.22 / 1.89 |

## 9. All at once against joint by joint (test 8)

The same poses for both versions.

| Model | Poses | Joint by joint, median / p90 / max (deg) | All at once | Median of (joint by joint - all at once) (deg) | Joint by joint closer | All at once closer | Sign test p |
| --- | ---: | --- | --- | ---: | ---: | ---: | --- |
| generic2 | 300 | 0.43 / 0.78 / 1.29 | 0.43 / 0.78 / 1.29 | +0.00 | 4% | 5% | 0.690 |
| generic3 | 300 | 0.60 / 1.04 / 1.44 | 0.60 / 1.01 / 1.54 | -0.01 | 55% | 45% | 0.094 |

## 10. Against the old two-joint decoder (test 9)

The old decoder is the frozen legacy code of the SO-2 repo with its stored weights (16,884 neurons); this decoder is `generic2` (1,860 neurons). The old decoder gives three angles; its orientation is the rotation built from them.

| Poses | Number | This decoder, median / p90 / max (deg) | Old decoder | Median of (this - old) (deg) | This closer | Old closer | Sign test p |
| --- | ---: | --- | --- | ---: | ---: | ---: | --- |
| 300 random poses | 300 | 0.43 / 0.78 / 1.29 | 4.96 / 9.72 / 18.78 | -4.51 | 99% | 1% | under 0.001 |
| the old decoder's two sweeps | 48 | 0.61 / 0.83 / 1.05 | 3.19 / 5.48 / 6.71 | -2.61 | 92% | 8% | under 0.001 |

Error of each of the three angles on the random poses with cos(pitch) > 0.3, median / p90 / max (deg):

| Decoder | Poses | Lift | Pitch | Yaw |
| --- | ---: | --- | --- | --- |
| this decoder (output rings) | 294 | 0.37 / 0.87 / 1.80 | 0.35 / 0.87 / 2.22 | 0.34 / 0.93 / 2.36 |
| old decoder (output rings) | 294 | 2.75 / 7.31 / 29.21 | 1.80 / 6.17 / 15.09 | 2.10 / 7.50 / 27.53 |

## 11. Variants and robustness (test 10)

The first 100 random poses of the static test.

| Model | Variant | Poses | Error, median / p90 / max (deg) | Median of (variant - as built) on the same poses (deg) | Similar |
| --- | --- | ---: | --- | ---: | --- |
| generic4 | as built | 100 | 0.80 / 1.16 / 2.06 | 0 | yes |
| generic4 | 5% of product neurons off | 100 | 3.91 / 6.59 / 9.14 | +3.16 | yes |
| generic4 | 10% of product neurons off | 100 | 5.07 / 7.80 / 11.77 | +4.33 | NO |
| generic4 | 20% of product neurons off | 100 | 8.31 / 13.88 / 19.97 | +7.71 | NO |
| generic4 | noise current 50 pA | 100 | 0.77 / 1.21 / 2.30 | +0.03 | yes |
| generic4 | noise current 150 pA | 100 | 0.96 / 1.52 / 2.00 | +0.27 | yes |
| generic4 | joint rings of 100 neurons | 100 | 1.00 / 1.57 / 2.81 | +0.26 | yes |
| generic4 | other neurons (seed 2) | 100 | 0.73 / 1.30 / 1.95 | -0.06 | yes |
| generic4 | other neurons (seed 3) | 100 | 0.73 / 1.25 / 1.77 | -0.02 | yes |
| generic4 | encoder writes on the nearest index | 100 | 1.12 / 1.84 / 3.33 | +0.36 | yes |
| generic4 | v4's corrective encoder | 100 | 1.50 / 2.40 / 4.23 | +0.68 | yes |
| generic4 | no link populations in the path | 100 | 0.72 / 1.16 / 1.87 | -0.00 | yes |
| generic4 | product 400, link 100 neurons | 100 | 0.63 / 1.03 / 1.34 | -0.14 | yes |
| tiago7 | as built | 100 | 1.14 / 1.62 / 2.17 | 0 | yes |
| tiago7 | 5% of product neurons off | 100 | 5.25 / 7.79 / 15.27 | +4.14 | NO |
| tiago7 | 10% of product neurons off | 100 | 6.75 / 10.62 / 23.56 | +5.55 | NO |
| tiago7 | 20% of product neurons off | 100 | 15.02 / 23.43 / 30.35 | +13.96 | NO |
| tiago7 | noise current 50 pA | 100 | 1.10 / 1.82 / 2.49 | +0.03 | yes |
| tiago7 | noise current 150 pA | 100 | 1.38 / 2.25 / 3.13 | +0.25 | yes |
| tiago7 | joint rings of 100 neurons | 100 | 1.29 / 2.07 / 2.82 | +0.09 | yes |
| tiago7 | other neurons (seed 2) | 100 | 1.03 / 1.82 / 3.14 | -0.04 | yes |
| tiago7 | other neurons (seed 3) | 100 | 0.98 / 1.83 / 2.32 | -0.12 | yes |
| tiago7 | encoder writes on the nearest index | 100 | 1.57 / 2.36 / 3.61 | +0.35 | yes |
| tiago7 | v4's corrective encoder | 100 | 2.11 / 3.51 / 4.93 | +1.01 | yes |
| tiago7 | no link populations in the path | 100 | 1.16 / 1.65 / 2.30 | -0.01 | yes |
| tiago7 | product 400, link 100 neurons | 100 | 0.93 / 1.33 / 1.70 | -0.21 | yes |

## 11b. The three encoders

*Rounding alone*: the exact orientation of the written ring indices against the true pose. *Rings read back*: the exact orientation of the angles read from the joint rings' activity against the true pose. Each encoder has its own calibration.

| Model | Encoder | Error against the true pose, median / p90 / max (deg) | Rounding to a ring index alone, median (deg) | Rings read back, median (deg) | Readout against the pose that was written, median (deg) |
| --- | --- | --- | ---: | ---: | ---: |
| generic4 | between indices, every tick (used) | 0.80 / 1.16 / 2.06 | 0.00 | 0.24 | 0.80 |
| generic4 | nearest index, every tick | 1.12 / 1.84 / 3.33 | 0.95 | 0.96 | 0.71 |
| generic4 | nearest index, only when the bump is off (v4) | 1.50 / 2.40 / 4.23 | 0.95 | 1.04 | 1.27 |
| tiago7 | between indices, every tick (used) | 1.14 / 1.62 / 2.17 | 0.00 | 0.29 | 1.14 |
| tiago7 | nearest index, every tick | 1.57 / 2.36 / 3.61 | 0.95 | 0.96 | 1.04 |
| tiago7 | nearest index, only when the bump is off (v4) | 2.11 / 3.51 / 4.93 | 0.95 | 1.23 | 1.73 |

## 12. Rate copy against the spiking network

The rate copy has the same populations as constant firing rates, with readouts fitted to the closed-form rates.

| Model | Rate copy, median / p90 (deg) | Spiking network, median / p90 (deg) |
| --- | --- | --- |
| generic2 | 0.56 / 0.91 | 0.43 / 0.78 |
| generic3 | 0.91 / 1.51 | 0.60 / 1.04 |
| generic4 | 1.29 / 1.85 | 0.77 / 1.22 |
| generic5 | 1.46 / 2.33 | 0.91 / 1.45 |
| generic6 | 1.74 / 2.76 | 0.97 / 1.57 |
| generic7 | 1.92 / 3.13 | 1.06 / 1.80 |
| tiago2 | 0.54 / 1.02 | 0.44 / 0.75 |
| tiago3 | 0.97 / 1.64 | 0.62 / 1.10 |
| tiago4 | 1.30 / 2.24 | 0.80 / 1.29 |
| tiago5 | 1.57 / 2.60 | 0.91 / 1.47 |
| tiago6 | 1.77 / 2.93 | 1.03 / 1.66 |
| tiago7 | 1.82 / 3.16 | 1.14 / 1.69 |

## 13. Rate copy: the sizes considered

Error median / p90 (deg) predicted by the rate copy on 200 poses (not the test poses). The middle column was chosen before the test poses were run.

| Model | Product 100, link 40 | Product 200, link 60 (chosen) | Product 400, link 100 |
| --- | --- | --- | --- |
| generic2 | 0.75 / 1.14 | 0.56 / 0.91 | 0.51 / 0.86 |
| generic4 | 1.70 / 2.72 | 1.29 / 1.85 | 1.10 / 1.84 |
| generic7 | 2.64 / 4.72 | 1.92 / 3.13 | 1.63 / 2.78 |
| tiago7 | 2.53 / 4.29 | 1.82 / 3.16 | 1.65 / 2.77 |

## 14. The pass marks

Set before the tests were run (docs/tests.md). Tests without a mark (2, 5, 8, 9, 10) are reported in their sections.

| Test | Pass mark | Outcome | Met |
| --- | --- | --- | --- |
| 1. Maths | equal to 1e-9; URDF to 1e-6 | asserted by `tests/test_kinematics.py` | yes |
| 3. Parts: cos / sin | within 0.03 | largest error 0.018 | yes |
| 3. Parts: product | rms at most 0.02 | rms 0.0077 | yes |
| 3. Parts: one stage, direction | within 2 deg | largest 1.87 deg | yes |
| 3. Parts: one stage, length | within 3% of the vector's length | largest 4.7% | NO |
| 4. Held poses | median at most 5 deg and p90 at most 10 deg, every model | 12 of 12 models; medians 0.43 to 1.14 deg, largest p90 1.80 deg, largest error 2.74 deg | yes |
| 6. Output rings | p90 at most 10 deg where cos(pitch) > 0.3 | largest p90 of any ring and model 2.37 deg | yes |
| 7. Moving arm: delay | at most 0.5 s at 0.5 rad/s | 0.16 to 0.20 s (4 models) | yes |
| 7. Moving arm: after a stop | back at the held-pose level 1 s after a stop | 12 of 12 sessions | yes |

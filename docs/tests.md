# The tests

Ten tests answer "does the readout of the rings give the orientation?" from different
sides. Each has a question, a procedure, and (where one makes sense) a pass mark that was
written down before the test was run. The outcomes, with numbers and figures, are in
[`results.md`](results.md); every table in full is in [`results/tables.md`](results/tables.md).

All commands run in the container: `docker/run.bash nest -- '<command>'` (top-level
[`README.md`](../README.md), section 2).

## How an error is measured

- **Orientation error**: the angle of the single rotation that turns the decoded
  orientation into the true one, in degrees. 0 is perfect; a readout that knows nothing
  is 126.5° off on average. The network gives a 3 x 3 matrix that is close to a rotation;
  the nearest proper rotation is taken before the angle is computed.
- **True orientation**: the exact kinematics of the chain at the robot's joint angles
  (`ring_so3/kinematics.py`), not anything the network produces.
- **"Similar"**, fixed before the runs: median error at most 5° and 90th percentile at
  most 10°, per model.
- **A held pose**: the arm moves to the pose at up to 1 rad/s per joint, stays 1 s, and
  the readout is averaged over the last 0.5 s.

## The tests

| # | Question | Procedure | Pass mark (set beforehand) | Run with |
| --- | --- | --- | --- | --- |
| 1 | Is computing joint by joint the paper's readout? Is the TIAGo chain right? | Exact maths, no neurons: `T Φ = R`; joint by joint and all at once against the direct matrix product; the TIAGo table against the robot's URDF | equal to 1e-9; URDF to 1e-6 | `python3 -B -m unittest tests.test_kinematics` |
| 2 | What error should the spiking network reach, and which sizes are enough? | The rate copy on 200 poses per model, three sets of sizes, with and without noise | none; it sets the expectation and the sizes | `scripts/rate_study.py` |
| 3 | Does each part do its job on inputs it was not fitted on? | In NEST: cos / sin population on a real ring at 80 positions between indices; product, link and length populations at random values; one multiplying stage | cos and sin within 0.03; product rms at most 0.02; one stage within 2° and 3% of the vector's length | `scripts/run_parts.py`, `tests/test_nest_parts.py` |
| 4 | Is the decoded orientation similar to the true one? | 300 random held poses per model, 12 models; plus a sweep of every joint over its range | "similar", for every model | `scripts/run_static.py` |
| 5 | How does the error grow along the arm? | The same poses: error of every link's orientation | none: reported | the same data |
| 6 | Do the lift, pitch, yaw rings show the three angles, and what happens at pitch = 90°? | The same poses: angle of each output ring against the true angle, by cos(pitch) | 90th percentile at most 10° where cos(pitch) > 0.3; nearer to 90° only the matrix is judged | the same data |
| 7 | Does the readout follow a moving arm? | The arm keeps moving from one random goal to the next (1 s rest at each) at 0.25, 0.5 and 1.0 rad/s; every tick logged | delay at most 0.5 s at 0.5 rad/s; back to the error at rest 1 s after a stop | `scripts/run_tracking.py` |
| 8 | Does the all-at-once version of the paper give the same? | 2 and 3 joints, the same 300 poses, paired | none: paired comparison reported | `scripts/run_static.py --mode flat` |
| 9 | Is it better than what existed? | The old two-joint decoder (frozen SO-2 code and weights) on the same 300 two-joint poses and on its own two sweeps, paired | none: paired comparison reported | `scripts/run_old_decoder.py` |
| 10 | What does it take to break it? | 100 poses, two models (generic4, tiago7): 5 / 10 / 20% of the product neurons switched off; noise current into every neuron; joint rings of 100 neurons; other neurons (2 more seeds); the two other encoders; no link populations in the path; larger populations | none: reported | `scripts/run_static.py` with options |

`scripts/run_experiments.py` runs tests 4 to 10 (70 runs, seven of them calibrations, about an
hour and a half on 24 cores); `scripts/report.py` turns the data into the tables and figures.

## What is and is not a test pose

- Readout weights are fitted on grids of values and on a ring swept index by index
  (`scripts/calibrate.py`). No pose of any chain is used.
- Population sizes were chosen on 200 poses with their own seed (`scripts/rate_study.py`).
- Test poses come from one fixed seed (`ring_so3/experiments.py`, `POSE_SEED`) and are
  uniform in the joint ranges. They were first run after sizes, encoder and readouts
  were fixed.

## Unit tests

`python3 -B -m unittest discover -s tests` (about half a minute):

| File | Covers |
| --- | --- |
| `tests/test_kinematics.py` | test 1: rotations, the paper's readout matrix, the wiring, the TIAGo chain against the URDF |
| `tests/test_rate.py` | neuron tuning and readouts, ring mapping, the copied ring against the SO-2 original (when mounted), the rate copy |
| `tests/test_nest_parts.py` | test 3 with fewer points, a 3-joint chain end to end, all-at-once and no-link-population versions, the other encoders |
| `tests/test_sim.py` | the light robot, the simulator's loop and its HTTP interface on a stand-in network |
| `tests/test_report.py` | the report's statistics, the delay estimate on a log with a known delay, and that the documents quote only numbers from the generated tables |

## A pass mark that was missed

Test 3 asked that one stage turn a vector "to within 2° and 3% of its length". The
direction mark is met. The length mark is not: over 64 vectors longer than 0.3 the
largest length error is 4.7% of the vector's own length in the recorded run, and 3.3%
to 4.7% over the runs made (they differ in the number of threads, and with it in the
encoders' random spike trains). In absolute terms it is at most 0.029 of the full
scale (the figures are in [`results/tables.md`](results/tables.md), section 0).
The cause is the product population's largest errors, which lie at the corners of its
input range. The sizes were not changed for it, because the test that matters (test 4)
is met with a wide margin; the unit test keeps a guard at 0.04 of the full scale.

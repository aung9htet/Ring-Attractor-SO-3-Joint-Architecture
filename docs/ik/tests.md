# Inverse kinematics: the tests and their marks

The marks were fixed in the plan of 2026-10-06, before any closed-loop run. A goal counts as
**reached** when the true orientation of the hand is within 5° of it at the end of a trial.
Test goals are the orientations of random joint poses, so each one can be reached.

Seeds: the test pairs (`TEST_SEED`), the development pairs on which the loop gain was set
(`DEVELOPMENT_SEED`) and the pairs of the exact-rule study (`STUDY_SEED`) are three different
streams (`ring_so3/ik/pairs.py`).

| Test | What it answers | Pass mark | Run with |
| --- | --- | --- | --- |
| 1. Maths | comparison formulas, goal matrix, shares add up to the turn | equal to 1e-9 | `tests/test_ik_rules.py`, `tests/test_ik_wiring.py` |
| 2. Exact rule | cascade, all at once, one joint at a time; both comparisons; delay, noise, dead zone | none: sets the expectation | `scripts/ik_study.py` |
| 3a. Comparison in spikes | decoded comparison against the exact one, 100 poses | direction within 10° (median) | `scripts/run_ik_parts.py` |
| 3b. Shares in spikes | decoded shares against the exact cascade | right sign in 95% of the shares above 0.05 rad | `scripts/run_ik_parts.py` |
| 3c. One joint's goal ring | lead of the goal bump against the share | no lead without a share; rising with the share to 6 indices or more; right sign for 0.02 rad in 90% of trials; never two bumps | `scripts/run_ik_goal_ring.py` |
| 4. Reaching | 100 start and goal pairs: generic3, generic7, tiago7 | reached: generic7 95%, tiago7 85%; 90% of the goals the exact rule reaches; final error of reached goals 3° (median); generic3 reported | `scripts/run_ik_experiments.py --batch reach` |
| 5. Trajectory | path against the shortest turn; time to 5° | path at most 1.3 times the shortest; 5° within 6 s (medians, 7 joints) | same data as test 4 |
| 6. The two comparisons | angles against matrices, same pairs | reported; decides the default | `--batch angles` |
| 7. Disturbances | robot moves 30% less or more than told; a joint pushed 0.3 rad; goal changed on the way | reached in 90% of the trials reached without | `--batch disturbances_generic`, `disturbances_tiago` |
| 8. Hard cases | goals near pitch ±90°; trials that end at a joint limit | reported | `--batch steep`; limits from test 4 |
| 9. Variants | all joints at once; 10% of the IK neurons off; noise; other seeds | reported | `--batch variants` |

## How a trial runs

1. The arm is moved by hand to the start pose. The goal on the goal rings follows the hand
   at its usual speed, and the trial starts only when it has arrived.
2. The goal is given. It travels on the goal rings at up to 1 rad/s.
3. The trial ends when the goal has arrived and the hand has turned less than 0.5° within
   2 s, or after 20 s.
4. Recorded: true error over time, path of the hand, decoded shares, lead of every goal
   ring, whether a joint ended within 5 ring indices of a limit.
5. The exact rule runs on the same pair (0.2 s delay, share noise 0.005 rad, dead zone
   0.01 rad, comparison bounded at 0.5 rad), so that a goal the network misses can be told
   from a goal the rule misses.

## Changes against the plan, made before the test runs

- **End of a trial.** Planned: "all shares quiet for 2 s". The decoded shares are off zero by
  about 0.01 rad all the time, so most trials would have run the full 20 s. Used instead:
  the hand at rest (step 3). Reaching is still judged by the true error at the end.
- **Fewer pairs for the secondary tests**, to keep the machine time within reach (the 7-joint
  network runs at about a quarter of real time, and jobs side by side do not add up): the
  other comparison on the first 50 test pairs, disturbances and variants on the first 25,
  steep goals on 20 pairs of their own. Test 4, which carries the marks, keeps its 100 pairs per arm.
- **Goal ring (c).** Planned: the state's cos and sin turned by the share and written as a
  bump. Built: a gain pair gated by the state bump that writes beside it. It needs no
  products and keeps the gain pair of the SO-2 model.

# Inverse kinematics for orientation: what is built

The hand gets an orientation goal on three goal rings (lift, pitch, yaw). The network works
out, all the time, about which axis and how far the hand still has to turn, hands every
joint a share of that turn, joint after joint down the arm, and each joint follows its own
goal ring. The mathematics is in [`TECHNICAL.md`](../../TECHNICAL.md), part II; what was
measured is in [`results.md`](results.md); the tests and their marks are in
[`tests.md`](tests.md).

Everything here sits on top of the forward network ([`../architecture.md`](../architecture.md)),
which is not changed.

## 1. The picture

![The inverse-kinematics layer of the three-joint chain: goal rings, goal orientation, comparison, the cascade of shares, and per joint a gain pair and a goal ring](figures/architecture.png)

The three-joint chain, drawn from the wiring the network is built from, with the neurons of
every part. Read it from the top left: the goal on three rings becomes the goal orientation;
the comparison with the decoded orientation gives the turn that is still to make; along the
upper row that turn is handed down the arm, and each joint takes its share (lower row); the
share moves the two sides of the joint's gain pair apart, which writes the joint's goal ring
beside its state ring; the robot moves every joint toward its goal ring. The same for seven
joints: [`figures/architecture_tiago7.png`](figures/architecture_tiago7.png).

```bash
docker/run.bash nest --out /tmp/ring-so3 -- 'python3 -B scripts/draw_ik_architecture.py --chain tiago7 --out /out/ik_tiago7.png'
```

draws it for any chain (`--comparison angles` and `--all-at-once` for the two variants).
How each box runs as spikes is part III of [`TECHNICAL.md`](../../TECHNICAL.md).

The same as text, with the signals named:

```
 goal rings                      forward network (unchanged)
 lift  pitch  yaw                joint rings ─► cos/sin ─► link orientations ─► decoded orientation R
   │     │     │                      │                         │                        │
   ▼     ▼     ▼                      │                         │ axis of joint k        │
  cos / sin of each                   │                         │ = one column of link k │
   │                                  │                         │                        │
   ▼                                  │                         ▼                        ▼
  goal matrix G ──────────────────────┼──────────────► comparison e = ½ Σ r_i × g_i  (3 populations)
                                      │                         │
                                      │                         ▼
                                      │        joint 1: share t1 = a1·e      rest e2 = e  − a1 t1
                                      │        joint 2: share t2 = a2·e2     rest e3 = e2 − a2 t2
                                      │        ...
                                      │                         │ two halves per share (+ and −)
                                      ▼                         ▼
                             state ring of joint k ──gates──► gain pair of joint k ──writes──► goal ring of joint k
                                      ▲                                                             │
                                      │ encoder                                                     │ read as an angle
                                  joint angle q_k  ◄───── the joint moves toward its goal ring ─────┘
```

## 2. The parts

| Part | Per | Neurons | Holds | File |
| --- | --- | ---: | --- | --- |
| Goal ring with encoder | goal angle (3) | 200 | the goal lift, pitch or yaw as a bump (the joint ring of the forward network) | `nest/ring.py` |
| cos / sin population | goal ring (3) | 120 | cos and sin of the goal angle | `nest/ik_network.py` |
| Goal matrix | once | 14 × 200 + 4 × 60 | the nine entries of the goal orientation | `ik/wiring.py`, `_goal_steps` |
| Comparison | once | 18 × 200 + 3 × 60 | twice the turn that is still to make, three numbers | `ik/wiring.py`, `ik_wiring` |
| Share | joint | 3 × 200 (none for joint 1, 2 × 200 for joint 2) + 2 × 60 | the joint's part of the turn, as a + half and a − half | `ik/wiring.py` |
| Rest | joint | 3 × 200 (none for joint 1, 2 × 200 for joint 2) + 3 × 60 | what is left for the joints after it | `ik/wiring.py` |
| Hold at a limit | joint with limits | synapses only | the joint's own ring inhibits the half that points beyond a limit | `nest/ik_network.py`, `_hold_at_limit` |
| Gain pair | joint | 2 × 200 | left and right side, gated by the state bump, moved apart by the share | `nest/goal_ring.py`, `build_written_ring` |
| Goal ring of a joint | joint | 200 | where the joint should be next | `nest/goal_ring.py` |
| Readout cells | once | 6 + 5 per joint | comparison, shares, rests, as numbers for the tests and the page | `nest/ik_network.py` |

- 200-neuron populations multiply two values, 60-neuron populations hold a sum (sections 5
  to 8 of the technical reference).
- The comparison "angles" replaces goal matrix and the 18 products by 11 products and 3 sums.
- Joint 1 turns about a fixed axis and joint 2's axis has a zero component, so their shares
  and rests need fewer products (the picture shows the counts).
- Seven joints: 37,620 neurons with the matrix comparison (forward network: 16,740). Three
  joints: 16,900 (forward network: 4,420).

## 3. One tick (50 ms)

1. The goal written on the three goal rings moves toward the goal that was given, at up to
   1 rad/s (a bump does not jump; lift and yaw take the short way round).
2. Every joint moves at 1.25 per second times the distance from its angle to the angle its
   goal ring held at the end of the last tick (the light robot: no dynamics).
3. The joint angles are written on the joint rings and the goal on the goal rings; NEST
   simulates 50 ms.
4. Read: the decoded orientation, the comparison, each joint's share and what is left, and
   the centre of each joint's goal ring (summed over the last 3 ticks).

`ring_so3/ik/session.py` is this loop. `IkSession.reach(goal)` runs one trial,
`IkSession.place(q)` moves the arm by hand (the goal then stays on the hand, so nothing is
asked of the arm).

## 3a. The robot

`ring_so3/robots.py` has two robots with the same calls, so the loop does not care which runs.

| Robot | What it is | Where |
| --- | --- | --- |
| Light robot | joint angles that move exactly as told, inside their limits | `LightRobot` |
| TIAGo in Gazebo | the simulated arm with its trajectory controller, in a world with nothing but the ground, a light and a camera (`gazebo/ring_ik.world`); physics advances 50 ms per network tick, in lock step | `GazeboRobot`, `ring_so3/gazebo/link.py`, `docker/run.bash gazebo` |

Each tick the robot is sent, per joint, the angle its goal ring holds, to be approached
with a time constant of 1 / 1.25 s. With a standard method chosen, it is sent that method's
targets instead; the network keeps running and showing its rings.

## 3b. The simulator

`scripts/ik_simulator.py` serves one page (`ring_so3/sim/ik_page.py`); the loop behind it is
`ring_so3/sim/ik_server.py`.

- **The goal** is drawn at the hand with a ring around each of its axes. Dragging a ring
  turns the goal about that axis; dragging inside turns it freely; three sliders do the same.
- **Method**: ring IK (spikes); ring IK without the cascade; the exact cascade; damped least
  squares; Jacobian transpose; cyclic coordinate descent.
- **Robot**: light robot, or the TIAGo in Gazebo when the simulator runs beside one; its
  camera picture is shown in a panel (needs the GPU container).
- **Runs**: every goal starts a run; "same start and goal, next method" puts the arm back and
  repeats the goal with another method. The distance from the goal of the last six runs is
  drawn in one chart, with a table (time to 5°, final error, path against the shortest turn).
- Below: each joint's share and lead, and the spikes of every ring.

## 4. Choices, and where each was decided

| Choice | Decided by |
| --- | --- |
| Comparison enters at the base joint and is passed down the arm; all joints move together | you (2026-10-06); checked against "every joint gets all" and "one joint at a time" with exact kinematics (`results/tables.md`, section 1) |
| Goal and decoded orientation are compared through cos and sin | you; two variants built, the default by a rule fixed before the runs (section 5 of the tables) |
| The light robot follows each joint's goal ring | you; the v4 loop under the goal rings comes later |
| A share is two halves, each capped at a lead of 8 ring indices | the cap is the largest lead of a goal ring |
| Near a limit the share is reduced by subtraction, not by a factor | this is what inhibition does; with exact kinematics it reaches the same goals |
| The comparison is held as twice the turn and bounded at 0.5 rad per component | resolution for small turns; with exact kinematics the bound costs at most 2 points of goals reached |
| The goal ring of a joint is written through a gain pair and is not an attractor | the two attractor versions did not meet the marks of test 3c (`results/tables.md`, section 3) |
| Loop gain 1.25 per second | set on 10 development pairs per arm (another seed than the test pairs), then frozen |
| A trial ends when the hand rests, not when the shares are quiet | the decoded shares are never exactly quiet |

## 5. What is not built

- The v4 controller under each goal ring (predictive ring, two gain pairs, spike decoder).
  In Gazebo the joint is driven by the TIAGo's own trajectory controller.
- Position goals. [`future-se3.md`](future-se3.md) says what they need.
- A way out of poses where no joint's axis lies along the turn, and a pull away from joint
  limits. Both show in the results.

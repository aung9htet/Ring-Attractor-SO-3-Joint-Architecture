# From orientation to pose: what SE(3) needs

Nothing on this page is built. It says what has to be added so that the hand reaches a
position as well as an orientation, what can be taken over as it is, and what the
orientation results say about the risks. The orientation IK is described in
[`architecture.md`](architecture.md) and measured in [`results.md`](results.md); the
forward network for position is in the SE-3 repo (`Ring-Attractor-SE-3-Joint-Architecture`).

## 1. What carries over unchanged

| Part | Why it carries over |
| --- | --- |
| The cascade | It takes a list of error cells and, per joint, the direction in which that joint moves them. It does not know that the error is a turn (section 24 of [`TECHNICAL.md`](../../TECHNICAL.md)). |
| The share as two halves, the hold at a joint limit | A share is a joint angle whatever the error was. |
| The goal ring of a joint, written through a gain pair | It turns a share into a lead; nothing about it is orientation. |
| The robot interface, light robot and Gazebo | `ring_so3/robots.py` knows joints only. |
| The simulator's method switch, runs and comparison | They compare a distance from the goal over time. |
| The standard methods | Damped least squares, transpose and coordinate descent are written for a Jacobian; it gets three more rows. |

## 2. What has to be added

1. **A position goal.** Three goal values (x, y, z of the hand). In the SE-3 network position
   is held on three 48 x 48 sheets with an anchor from the joint rings; a goal can be held the
   same way, or as three scalar populations, which is enough for a goal that does not have to
   be remembered. It has to travel, as the orientation goal does (a bump does not jump).
2. **A position error.** `p* - p`, three populations, from the goal and the decoded position.
   The SE-3 readout of position is good to about 1% of the reach (median), 1 to 1.4 cm on the
   TIAGo arm; that is the floor for the final position error.
3. **Per joint, the direction in which it moves the hand.** For orientation this is the axis
   `a_k`. For position it is the lever `a_k x (p - p_k)`: the axis crossed with the vector from
   the joint to the hand. The SE-3 network already builds exactly this product for its velocity
   path (hand velocity from joint velocities), so the populations exist; they have to be read
   out as directions instead of being multiplied by a joint velocity.
4. **The share with six error cells.** With `J_k = (a_k, lambda * a_k x (p - p_k))` and the
   stacked error `E_k`, the share is `t_k = J_k . E_k / |J_k|^2` and the rest
   `E_(k+1) = E_k - J_k t_k`. Two things are new against orientation:
   - the division by `|J_k|^2 = 1 + lambda^2 |lever|^2`. A population that divides is not in
     the tool box. Ways around it: fold `1 / |J_k|^2` into the product through a second
     readout of the lever populations (a function of the same values), or leave it out and
     accept a transpose-like step for the position part;
   - 6 more products per joint for the share and 6 for the rest, about 2,400 neurons per
     joint, so roughly 17,000 more for seven joints, on top of the SE-3 forward network.
5. **A weight between turning and moving**, `lambda` (per metre). It decides whether the hand
   first turns or first travels. It should be on the page as a slider.
6. **A draggable position** in the simulator: the goal frame gets three arrows beside its
   three rings.

## 3. What the orientation results say about the risks

- **The readout offset becomes a distance.** A share is read with an offset of about 0.01 rad.
  For orientation this ends as about 1° of final error. Through a lever of 0.5 m the same
  offset is 5 mm at the hand, on top of the 1 to 1.4 cm of the position readout.
- **Delay.** The comparison sees the arm about 0.2 s late; the position path of the SE-3
  network is slower (sheets and their anchor). The loop gain that is safe is about
  `0.37 / delay` without overshoot; expect a gain below 1 per second and so slower reaches
  than the 4 to 6 s measured for orientation.
- **Joint limits.** About 6 in 100 TIAGo orientation goals end at a joint limit, with exact
  kinematics as well. A pose goal uses all six freedoms of a seven-joint arm, so there is one
  spare joint instead of four: expect clearly more trials that end at a limit. A pull of each
  joint toward the middle of its range, in the freedom that is left, becomes necessary; it is
  optional for orientation.
- **Poses where the arm cannot move the hand in some direction** (a stretched arm) are far
  more common for position than for orientation. The cascade does not blow up there (it never
  divides by a small number if the division above is bounded), it gets slow.
- **Speed of the simulation.** The orientation IK runs at about a quarter of real time for
  seven joints (37,620 neurons). Pose IK on the SE-3 network will be near 60,000 neurons
  with the sheets, so about a seventh of real time on this machine.
- **Path integration.** The SE-3 repo found the ring path integration for position weak
  (the packet's speed off by more than 10%, drift at rest). The standard version drifts by
  tens of degrees a minute with realistic velocity noise (section 11 of the tables). A pose
  IK should therefore be closed on the anchored readout (positions), as the orientation IK is,
  and use integration only to bridge gaps.

## 4. A ring path integration for orientation

Not built, and not needed for the IK, but the missing piece of the comparison in section 11
of the tables. What it would take: the three output rings (lift, pitch, yaw) as attractors
that hold their bumps without input; the angular velocity `omega = sum_k a_k * qdot_k` as
populations (products of link entries and joint-velocity cells, which the SO-2 model has);
the rates of lift, pitch and yaw from `omega` (three more products, with a division by
`cos(pitch)` that fails near pitch +-90 deg); and gain pairs that shift the three bumps at
those rates. The SO-2 ring shifts a bump only above a threshold of gain spikes
([`results.md`](results.md), goal ring), so slow turns would be lost: the same weakness the
SE-3 position sheets showed.

## 5. Order in which I would build it

1. Exact check first, as for orientation: the six-cell cascade with exact kinematics on
   random pose goals, with and without the division, for a few values of `lambda`. This says
   how many goals a seven-joint arm reaches at all and what gain is safe.
2. Position error and lever directions read from the SE-3 forward network, arm held by hand
   (the counterpart of tests 3a and 3b).
3. The six-cell cascade in spikes; closed loop on the light robot; then Gazebo.
4. The pull toward mid-range for the spare joint.
5. The simulator: a goal with arrows and rings.

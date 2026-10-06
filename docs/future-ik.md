# Future work: inverse kinematics toward an orientation goal

Nothing on this page is built. It collects ways to go from "the end effector should have
this orientation" to "these joints should move", using what the forward network of this
repo already holds. The forward network is described in [`architecture.md`](architecture.md),
what it achieves in [`results.md`](results.md).

## What is there to build on

- **The orientation of every link**, as populations: nine matrix entries per link, in
  the network, updated all the time.
- **Each joint's turning axis.** A joint leaves one column of its link's matrix
  unchanged: the column of the axis it turns about. That column *is* the joint's axis
  in the base frame. So the three numbers that say "which way does the end effector
  turn when joint k moves" are already entries of link k.
- **A population that multiplies**, with its readout fitted once and reused everywhere.
- **Per joint, the controller of the SO-2 repo** (v4): a goal ring whose bump the joint
  follows, a predictive ring, a state ring. Moving a goal bump is how a joint is told
  to move.

## Idea 1: correct each joint by "orientation error times joint axis"

This is the transpose-of-the-Jacobian rule, and every part of it is a sum of products.

1. **Goal orientation as a matrix.** If the goal is given as three goal rings (lift,
   pitch, yaw), run a second, small forward network on them: `R_goal = Rz(yaw) · Ry(pitch)
   · Rx(lift)` is a three-joint chain with axes z, y, x, so the same code builds it
   (about 4,500 neurons).
2. **Error as a vector.** `e = ½ Σ_i (r_i × g_i)`, with `r_i` and `g_i` the columns of
   the current and the goal matrix. `e` points along the axis about which the end
   effector has to turn, and its length grows with the angle. Each of its three
   components is a sum of six products of one current and one goal entry: 18 product
   populations and three link-type populations.
3. **Each joint's share.** `d_k = axis_k · e`: three more products per joint. `d_k` is
   positive when turning joint k forward reduces the error, negative when turning it
   back does.
4. **Into the joint controller.** `d_k` drives the pair of gain populations that shift
   joint k's *goal* bump (the same mechanism that moves a bump in the SO-2 model).
   The joint's own loop then follows its goal bump.

Why it should work: with small steps the rule can only reduce the error (it is the
gradient of `trace(I - R_goalᵀ R)`), and it never divides by anything, so it does not
blow up near singular poses; it only gets slow there. Only the *direction* of `e` and
of the axes has to be right, and the forward network gives link orientations to about
a degree. This is the `TaskGain` block of the SO-2 plan ("configuration features AND
task-error direction") made concrete.

Known weak spots, to be measured:

- It stalls where no joint's axis has a component along `e` (singular poses).
- At an error of exactly 180° the vector `e` is zero. Remedy: do not jump the goal;
  let the goal orientation itself move toward the target in small steps.
- The forward readout lags a moving arm by a fraction of a second
  ([`results.md`](results.md), section 6), so this outer loop has to be slower than
  the joint loops.

Cost for 7 joints, on top of the forward network: 18 + 21 products, a few thousand
neurons.

## Idea 2: compare rings instead of matrices

Compare the lift, pitch and yaw output rings with three goal rings using the SO-2
comparator (which gives a signed mismatch of two bumps), and turn the three mismatches
into joint corrections. It is closer to the SO-2 circuit, but lift and yaw lose their
meaning near pitch = ±90°, where the rings fall silent. Use it for display and for
coarse goals; take the error for control from the matrix (idea 1).

## Idea 3: try the move in imagination first

The v4 controller has a predictive ring per joint. Run the forward network on the
predictive rings instead of the state rings: the result is the orientation the arm
*would* have. Then idea 1 can adjust the predictive bumps until the imagined
orientation matches the goal, without the arm moving, and only then hand the result to
the goal rings. That allows checking joint limits (the walls of the bounded rings) and
choosing between solutions before anything moves. It is the "internal simulation" the
paper's conclusion points to.

## Idea 4: learn the inverse by association

While the arm moves at random, strengthen connections from the cells that hold the
orientation (and the current joint rings) to the joint goal rings, as RatSLAM
associates local views with pose cells. Afterwards a goal orientation activates joint
goal bumps directly. Many joint poses give the same orientation, so the association
has to depend on the current pose, and the rings' own competition has to settle on one
answer. Fast at run time; needs training and only covers poses that were visited.

## Idea 5: seven joints, three numbers to satisfy

An orientation fixes three degrees of freedom; a 7-joint arm has four to spare. Without
a second aim the rule of idea 1 simply moves every joint a little. Natural second aims,
each a weak extra drive on the goal bumps:

- toward the middle of each joint's range (away from the walls of the bounded ring);
- as little motion as possible (prefer the joints with the largest share `d_k`);
- later, with SE(3): position first, orientation second, by the strength of the two drives.

## Idea 6: move the goal, not only the arm

Keep the goal orientation as bumps on rings and let those bumps travel toward the final
goal at a limited speed (the bump transport of the paper, applied to the goal). The
error the joints see then stays small, which keeps idea 1 in the range where it is
exact and avoids the 180° case.

## What to measure first

1. **Rate copy, closed loop.** The rule of idea 1 on the rate copy with the light
   robot: random start and goal orientations; final error, time to arrive, how often
   it stalls. Cheap, and it shows whether direction errors of a degree matter.
2. **Spiking network with the light robot.** The same, with the forward network of
   this repo in the loop; adds the delay.
3. **With the SO-2 joint controllers.** Each joint driven by its v4 controller, the
   corrections applied to the goal bumps.

Pass marks to write down before running, as for the forward network: final orientation
error, share of goals reached, time against a plain numerical solver on the same goals.

## Later: position goals

In SE(3) the same rule needs, per joint, `axis_k × (p_end - p_joint_k)` for the
position part: again products of entries the chain holds once it also holds positions.
That is the subject of the SE-3 repo.

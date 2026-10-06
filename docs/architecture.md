# How the network computes the orientation

What the network is made of and why. How to run it: the top-level
[`README.md`](../README.md). What it achieves: [`results.md`](results.md).

![the network for three joints](figures/architecture.png)

Drawn by `scripts/draw_architecture.py` from the same wiring the network is built from.

## 1. The task

A chain of revolute joints. Each joint angle sits on its own ring attractor as a bump
of activity (the SO-2 repo: one ring per joint). From those rings the network has to
give the orientation of the end effector, and of every link on the way: a rotation,
an element of SO(3).

A chain is described in [`ring_so3/chains.py`](../ring_so3/chains.py): for each joint
the axis it turns about, the fixed rotation `A_k` of its frame in the previous link,
its limits, and the offset of its origin (for drawing, and for the translation in
SE(3) later).

```
R_k  = R_(k-1) · A_k · Rot(axis_k, q_k)        orientation of link k
R_ee = R_n · A_tool                            orientation of the end effector
```

## 2. The maths

**The paper's readout.** With `φ_k = (1, cos q_k, sin q_k)` for every joint, every
entry of `R_ee` is a sum of products that take one factor from each joint. The paper
(Section 2.5, Appendix E) writes this as `vec(R) = T Φ`, with `Φ = φ_1 ⊗ … ⊗ φ_n` the
vector of all `3^n` products and `T` a fixed matrix. `ring_so3/kinematics.py` builds
`T` and `Φ`; the test `tests/test_kinematics.py` confirms `T Φ = R` to 1e-9 for all
14 chains.

**Most products never occur.** For 2 to 7 joints only 8, 13, 21, 34, 55, 89 of the
`3^n - 1` products have a non-zero column in `T` (for 7 joints: 89 of 2,186). This
holds for both chain families here.

**Joint by joint.** A joint changes two columns of the matrix and leaves the third:

```
col_u' =  col_u · cos q + col_v · sin q
col_v' = -col_u · sin q + col_v · cos q        (u, v: the two axes the joint does not turn about)
```

So each joint costs four products per matrix row, twelve per joint (fewer for the
first joints, whose entries are still constants). The fixed rotation `A_k` only
re-labels columns: it needs no neurons. This is the same sum of products as `T Φ`,
evaluated in the order in which the cost grows by a fixed amount per joint, and it
gives the orientation of every link.

| Joints | Products that occur | Pairwise products, all at once | Pairwise products, joint by joint |
| ---: | ---: | ---: | ---: |
| 2 | 8 | 4 | 4 |
| 3 | 13 | 14 | 14 |
| 4 | 21 | 30 | 26 |
| 5 | 34 | 56 | 38 |
| 6 | 55 | 98 | 50 |
| 7 | 89 | 166 | 62 |

[`ring_so3/wiring.py`](../ring_so3/wiring.py) turns a chain into this list of steps
(`chain_wiring`), and into the all-at-once list (`flat_wiring`) used as a cross-check
for 2 and 3 joints. The rate copy and the NEST network are both built from it, and a
test checks that evaluating the steps exactly gives the exact matrix.

**Lift, pitch, yaw.** As in the SO-2 code: `lift = atan2(r32, r33)`,
`pitch = -asin(r31)`, `yaw = atan2(r21, r11)`. That is the paper's Eq. 26 and means
`R = Rz(yaw) · Ry(pitch) · Rx(lift)`. The paper's Eq. 25 writes the product in the
opposite order, which does not agree with Eq. 26; the code and this repo follow
Eq. 26. Lift and yaw are not defined when pitch is ±90°.

## 3. The populations

Every population is NEST's `iaf_psc_alpha`, as in the paper.

| Population | Holds | Neurons | Code |
| --- | --- | ---: | --- |
| Joint ring, one per joint | the joint angle, as a bump | 200 | `nest/ring.py` |
| Cos / sin population, one per joint | `cos q`, `sin q` | 120 | `nest/network.py`, `nest/calibrate.py` |
| Product population, one per product | one product, e.g. entry x `cos q` | 200 | `neurons.py` (kind `product`) |
| Link-orientation population, one per new matrix entry that is a sum | one matrix entry of a link | 60 | `neurons.py` (kind `scalar`) |
| Readout cells, nine per link and for the end effector | the decoded matrix entries | 9 | `nest/populations.py` |
| Length population, one | `cos(pitch)` | 120 | `neurons.py` (kind `norm`) |
| Output rings: lift, pitch, yaw | the three angles, as bumps | 100 each | `nest/network.py` |

Three joints need 4,420 neurons, seven need 16,740. Values between the rings are held
by populations, not by rings: joint rings go in, orientation rings come out.

### 3.1 The joint ring and how an angle is written on it

The ring is the ring attractor of the SO-2 repo, unchanged: 200 neurons, every neuron
excites its neighbours and inhibits a surround, neuron parameters of the paper's
Table 1. A joint's range lies on the whole ring (generic chains) or on 70% of it with
a wall around the seam (TIAGo arm), as in architecture v3 / v4.

An encoder writes the joint angle: Poisson input at 200 Hz to the 11 neurons around
the angle's position. Three ways are implemented (`ENCODER_MODES` in `nest/ring.py`):

| Encoder | What it does | Where from |
| --- | --- | --- |
| `fractional` (used here) | every tick, at the unrounded position: the two neurons at the window's ends get the share of the rate that the fraction leaves them | new in this repo |
| `continuous` | every tick, at the nearest ring index | SO-2 |
| `corrective` | only when the bump is more than one index from the nearest index | SO-2, the state ring of v4 |

Measured in NEST: a bump can follow a target that moves a few indices per tick, but it
cannot jump across the ring (two bumps then coexist), so in every test the arm moves
to a pose, it is never placed there. How the three encoders compare is in
[`results.md`](results.md), section 9.

**Writing ahead (option, off by default).** The readout trails a moving arm by 0.16 to
0.20 s, the same at every speed. `Session(..., lead=True)` (`ring_so3/session.py`) hands
the encoder of joint k the angle `q_k + lead_k · velocity_k` instead of `q_k`, clipped
to the joint's limits, with `lead_k` = 0.14 s for the last joint and 0.01 s more for
each joint before it (a change at an early joint passes through more stages). At rest
the two are the same. The network is not changed by it.

### 3.2 From a bump to cos and sin

The cos / sin population reads its ring with the weights `cos q_j` and `sin q_j`,
where `q_j` is the joint angle ring neuron `j` stands for (the paper's Eq. 14). Its
120 neurons each respond to one direction of the pair `(cos q, sin q)` above their own
threshold. Its readout is fitted on a real ring whose bump is written at many
positions, so it gives a pair of length 1 for the bump as it really is. The synapse
from the ring is slower than the others (15 ms): a ring neuron fires only 20 to 60
times a second.

### 3.3 A product

A product population holds a pair `(a, b)`. Each of its 200 neurons responds to
`a + b` or to `a - b` (one of four diagonal directions) above its own threshold, up to
its own maximum rate. A weighted sum of the 200 rates gives `a · b`, because
`ab = ((a+b)² - (a-b)²) / 4`; the weights are a least-squares fit to the measured
rates. This is the population code of Eliasmith (2005), which the paper already cites
for its Fourier readout, and the "signed product-feature populations" of the paper in
a form that really multiplies. `a` is an entry of the orientation so far; `b` is the
cos or the sin of the next joint.

### 3.4 An entry of a link's orientation

A new matrix entry is a sum of two products. It gets a population of 60 neurons, half
of which respond to positive and half to negative values (the push-pull halves of the
paper), each above its own threshold. The next joint's products read the entry from
there, and so does the readout. Entries that are a single product, a single cos or
sin, or a constant need no population; entries that a joint does not change are read
where they already are.

Whether these populations should be in the path at all was decided by a rule written
down first: lower median error on held poses, the cheaper network on a tie. The two
versions tied ([`results.md`](results.md), section 9), and the version with link
populations has fewer synapses.

### 3.5 Readout cells

A readout cell is a neuron that never fires: its membrane potential is the weighted
sum of the spikes it receives, smoothed with 50 ms. Nine cells per link and nine for
the end effector hold the decoded matrix entries; they are read every 50 ms tick. The
matrix they give is "raw": close to a rotation, not exactly one. For an error in
degrees the nearest proper rotation is taken first (`kinematics.nearest_rotation`);
how far the raw matrix is from a rotation is reported separately.

### 3.6 The output rings

Each output ring has 100 neurons with preferred angles round the circle. A neuron of
the yaw ring is driven by `r11 · cos θ + r21 · sin θ`, which is largest for the neuron
whose preferred angle `θ` is the yaw; the lift ring by `r33 · cos θ + r32 · sin θ`.
The angle of a ring is read as the centre of its activity over the last four ticks.

The pitch ring needs `cos(pitch)`, which is not an entry of the matrix but the length
of the pair `(r11, r21)`. One more population holds that pair and its readout gives
the length; the pitch ring is driven by `length · cos θ - r31 · sin θ`.

A neuron of an output ring starts to fire when its drive exceeds 0.25. When pitch
nears ±90° the pair that drives the lift and yaw rings shrinks to nothing and those
two rings fall silent: that is the network's way of saying that lift and yaw have no
value there. The matrix is not affected.

## 4. Where the weights come from

No weight is set by hand and none is fitted on a test pose.

1. Each population's thresholds and maximum rates are drawn once from a seed
   (`ring_so3/neurons.py`, `design.py`).
2. `scripts/calibrate.py` measures each kind of population in NEST, in small rigs that
   run side by side, on a grid of inputs: a scalar population and a product population
   driven by the spikes of source populations; the cos / sin population driven by a
   real ring swept over its positions. The readout of each kind is the least-squares
   fit to those measured rates (`ring_so3/nest/calibrate.py`). It takes about a minute.
3. A connection from population A to population B is A's readout weights times B's
   input weights. All populations of one kind share one set of readout weights.

The result is stored in `artifacts/` with its settings and fit errors. A network loads
the file that matches its sizes, seed and encoder.

## 5. Running it

`FkNetwork` (`ring_so3/nest/network.py`) builds everything in one NEST kernel and
keeps it: nothing is rebuilt between readings. `step(q)` writes the joint angles on
the rings, simulates one 50 ms tick, and returns what the network holds: the angle
every joint ring holds, the raw matrix of every link and of the end effector, and the
angles and spike counts of the three output rings. `Session` (`ring_so3/session.py`)
steps a network together with the light robot of the simulator.

## 6. The copy without spikes

`ring_so3/rate/network.py` has the same populations, tuning and wiring, with every
neuron replaced by its firing rate for a constant input. It needs only NumPy and runs
in milliseconds. It was used to choose the population sizes before any test pose was
run (`scripts/rate_study.py`, [`results.md`](results.md), section 11), it tests the
wiring quickly, and it is a level the spiking network is compared with. It leaves out
spikes, delays, the ring's dynamics and the output rings.

## 7. What changed from the old two-joint decoder

| | Old decoder (SO-2 `legacy/compositional_fourier_decoder.py`) | This repo |
| --- | --- | --- |
| Joints | exactly 2 | any chain; built and tested for 2 to 7 |
| Product | a 32 x 32 grid of cells per feature whose two inputs are added; a ridge fit over all 16,384 cells turns the grid into a lookup table | a population per product that multiplies; the fit is per kind of population and does not grow with the chain |
| Weights | 3 matrices of 16,384 x 100, fitted on a 16 x 16 grid of poses of that one chain | one readout per kind of population, fewer than 700 numbers in all, the same for every chain |
| Cost of a further joint | 32 times more cells | 12 products |
| Per reading | kernel reset, network rebuilt, bumps injected | one persistent network, a reading every 50 ms |
| Output | lift, pitch, yaw rings | the matrix of every link, and lift, pitch, yaw rings |
| Angle on the ring | nearest index | between indices |
| Tested on | 16 poses, each angle on its own | 300 random poses per model and more ([`tests.md`](tests.md)) |

## 8. What changed from the plan, and why

Decided by measurement during the work; each is covered in [`results.md`](results.md).

- **Encoder.** The plan was v4's corrective encoder. It leaves the bump up to one
  index from the angle by design, and it rounds to an index. Writing every tick at
  the unrounded position holds the angle more exactly, so `fractional` is the default;
  the other two are measured as variants.
- **Pitch ring.** First driven through a readout of `sqrt(1 - r31²)`, which is steep
  near ±1 and gave the pitch ring the largest error of the three rings. Now driven by
  the length of `(r11, r21)`.
- **Writing ahead.** Not in the plan; added after the moving-arm test showed that the
  error while moving is a fixed delay and nothing else.
- **One-stage pass mark.** "Within 3% of its length" is missed (3.3% to 4.7% at worst,
  depending on the run); the direction mark is met. Reported in [`tests.md`](tests.md);
  the sizes were not changed for it.
- **Sizes.** The table of the plan was a starting point; the rate study and the first
  NEST runs confirmed it, so the sizes stayed.

## 9. Code taken from the SO-2 repo

Copied from branch `velocity-calibration` of the SO-2 repo, so that this repo runs on
its own. `ring_so3/nest/ring.py` and `ring_so3/mapping.py` say file by file what comes
from where.

| Here | From SO-2 | Changed |
| --- | --- | --- |
| ring connectivity, `ring_weight_matrix` | `math/ring.py` (`legacy` variant) | nothing; a test compares the two matrices when SO-2 is mounted |
| ring neuron parameters | `config/model_params/neuron_params.json`, entry `ring` | nothing |
| wall of the bounded ring | `nest/populations.py` (`wall_profile`), `graph/templates.py` (`BOUNDED`) | nothing |
| stimulus | `nest/populations.py` (`build_stimulus_generators`) | nothing |
| `continuous` and `corrective` encoders | `blocks/encoder.py` | one recorder per ring instead of one per neuron |
| angle to ring index | `control/mapping.py`, `math/circular.py` | unrounded position added |
| lift, pitch, yaw | `math/kinematics.py` | nothing |

## 10. Limits of the design

- Nothing pulls the decoded matrix back onto a proper rotation, so every joint adds a
  little error ([`results.md`](results.md), section 4).
- A neuron sends positive and negative weights, as in the old decoder's fitted
  matrices; a strictly excitatory / inhibitory version would need more populations.
- The readout follows a moving arm with a delay, because every population adds a few
  milliseconds ([`results.md`](results.md), section 6).
- A joint ring cannot follow a jump of the angle; the arm has to move.
- One joint angle per ring, known kinematics, fixed axes: the assumptions of the paper.

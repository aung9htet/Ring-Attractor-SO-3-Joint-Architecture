# Kangaroo migration of the SO(3) ring network: design

Date 2026-10-26, branch `kangaroo-migration` (from `origin/ik-cascade`, commit 9e3ed24).

## Intent

The SO(3) ring network of this repo (orientation readout + inverse-kinematics cascade, NEST) has the TIAGo
in Gazebo (ROS 1) as its only physical robot. The SO-2 repo (`../Ring-Attractor-SO-2-Joint-Architecture`,
branch `kangaroo-migration`) has already moved its joint-ring controller to PAL's Kangaroo in MuJoCo on
ROS 2 Humble. This work does the same for SO(3): the network stays as it is, the robot side changes to
Kangaroo, both arms are calibrated and measured, and the result is a control page that drives the
simulated Kangaroo arm with the ring IK, as the SO-2 repo has its ring GUI. The outcome is for a research
publication: every number and figure must trace to a recorded run.

## Goals

1. **Kangaroo arm chains.** Two entries in `ring_so3/chains.py`, `kangaroo_left4` and `kangaroo_right4`
   (PAL 4-DoF arm, `arm_<side>_1_joint` .. `arm_<side>_4_joint`): axes, fixed rotations, offsets, limits
   and velocity limits from the Kangaroo URDF generated in the image, base frame the arm's parent link,
   ending at the arm's last link (or tool frame if the 4-DoF arm has one). Shorter prefixes
   (`kangaroo_<side>2`, `3`) only if the existing experiments need them; otherwise not.
2. **`KangarooRobot`** in `ring_so3/robots.py` with the calls every robot has (`q`, `follow`, `reset`,
   `describe`, `difference`, `tolerance`, `place_speed`), and a ROS 2 link module beside
   `ring_so3/gazebo/link.py` (e.g. `ring_so3/kangaroo/link.py`) that reads `/joint_states`, sends
   `JointTrajectory` to `/arm_<side>_controller/joint_trajectory`, and advances physics.
3. **Container.** A way to run this repo in the Kangaroo image (ROS 2 Humble, MuJoCo, NEST at the
   repo's pinned commit `41892a5`), started the same way as `docker/run.bash gazebo up|exec|down`.
4. **Calibration.** Re-run `scripts/calibrate.py` and `scripts/calibrate_ik.py` in the Kangaroo image,
   to a new output, and compare with the checked-in `artifacts/` (both calibrations are chain
   independent: "fitted once per kind of population, the same for every chain"). The comparison is the
   finding; the checked-in files stay untouched.
5. **Measurements, both arms:**
   - held poses (`run_static.py`) and tracking (`run_tracking.py`) for `kangaroo_left4` / `kangaroo_right4`
     on the light robot;
   - closed-loop IK reaches (`reach_run`, test pairs) on the light robot and on the MuJoCo Kangaroo, ring
     IK and at least one standard method (e.g. `dls`) for comparison;
   - a FK check of the chain against the robot's own TF (base link to end link) at a few poses.
6. **Figures and record**, as in the SO-2 repo: figures (PNG + PDF) and their data in a new results
   directory, a `manifest.json` per figure set (inputs, commit, seeds, threads), and an experiment note
   `docs/experiments/2026-10-26_kangaroo_<name>.md` per experiment (hypothesis, config, seed, command,
   commit, results, interpretation, negative results included).
7. **Control system.** The existing IK simulator (`scripts/ik_simulator.py`, `ring_so3/sim/ik_server.py`,
   `ik_page.py`) offers "Kangaroo in MuJoCo" as the robot when run beside the Kangaroo sim; the chain
   selector switches between left and right arm. The page stays plain HTML/JS as it is.
8. README: a Kangaroo section with the commands, like section 11.2 for Gazebo.

## Non-goals

- No change to the network, its wiring, populations or calibration code.
- No SO-2 velocity-decoder calibration (k_pos, k_neg, τ) in this repo.
- No 5-DoF or 7-DoF Kangaroo arm (PAL ships only the 4-DoF variant ready to run; SO-2 decision Q1).
- No torso, legs or base motion; they stay where the sim puts them.
- No new UI framework (React etc.), no new dashboard.
- No re-run of the existing TIAGo / generic results.

## Anti-goals (a failure even if it runs)

- Kangaroo numbers in `chains.py` typed from memory or from the SO-2 knowledge base instead of read from
  the URDF in the image.
- A result or figure that no recorded run produced, or a number tuned to look better without saying so.
- Existing results, `artifacts/` or recorded data overwritten.
- Helpers, options or abstractions that no goal above needs.
- The TIAGo/Gazebo path broken.

## Constraints

- **Where:** local repo only, branch `kangaroo-migration`. Nothing pushed. No commit unless the owner asks.
- **Image:** none of the needed images is built on this machine. Build the Kangaroo image from the SO-2
  repo's `docker/ros2_kangaroo/Dockerfile` (+ its `kangaroo_sim.launch.py`), copied into this repo so it
  runs on its own, plus whatever this repo needs that it lacks (check: matplotlib, ffmpeg, opencv for the
  camera; add only what is missing). Python there is 3.10; this repo targets 3.8, so check it runs.
- **Hardware:** RTX 4070 laptop, 22 cores, 30 GB RAM.
- **Stepping:** `GazeboRobot` advances physics 50 ms per network tick in lock step, so a network slower than
  real time does not matter. `mujoco_ros2_control` documents pause / step services
  (`mujoco_ros2_control_msgs/srv/SetPause`, `StepSimulation`); PAL's Humble release pins 0.1.2, which
  may not have them. Verify in the image first. If present: lock step as in Gazebo. If absent (owner
  decision): build a newer `mujoco_ros2_control` from source that has them, into the image, and lock step.
  If that build fails: run against the sim clock in real time, stamp every tick with sim time, record the
  real-time factor and the lag, and report the MuJoCo reaches as real-time-degraded.
- **Known plant facts from SO-2** (to check, not to assume): the JTC is P-only and under gravity the
  measured position sits about 0.01 rad off the reference (SO-2 D13); `right j1` stops at about 1.46 rad
  (effort saturation, SO-2 F9); measured ranges are in
  `../Ring-Attractor-SO-2-Joint-Architecture/src/kangaroo_ring_controller/results/joint_limits/`; PAL home
  is left `[0.239, 1.321, 1.57, 0.8]`, right `[-0.239, 1.321, -1.57, 0.8]`; a single reset was sometimes
  ignored (verify and retry, SO-2 D18).
- **Seeds and threads** recorded in every output (results depend slightly on thread count).
- **Code shape:** match the style of `ring_so3/` (module docstrings that explain, short functions, numpy).
  Minimal: no future-proofing, no unused options.

## Process (decided with the owner)

- Skills: brainstorming (this), TDD (`mattpocock-skills:tdd`), understand (`understand-anything:understand`)
  for the codebase map, grill-with-docs for the plan, ponytail for minimal code.
- Separate agents, working independently:
  - **code writer**: writes the code from the plan; does not see the tests;
  - **test writer**: writes tests from the design and plan only (behaviour, independent oracles), for the
    main functionality only, not every line; runs them against the writer's code;
  - **semantic checker**: does the code do what this design says (same robot calls as `GazeboRobot`,
    angle conventions, frames, units, limits)?
  - **authenticator**: is the code true to its sources (Kangaroo numbers equal the URDF in the image, ROS 2
    topics and message types exist in the running sim, every figure and number traces to a real run)?
- Tests that need NEST or the sim run in the container; report results verbatim, including what could
  not run.

## Decided by the owner

- Calibration means re-running this repo's own calibration and measurements on the Kangaroo chains and on
  the simulated Kangaroo, not SO-2's decoder calibration.
- This local repo, branch `kangaroo-migration` from `ik-cascade`; no new GitHub repo.
- Both arms calibrated and measured.
- The control system is the existing IK simulator with Kangaroo as the robot; UI as minimal as needed.
- No pause/step in the image: build a newer `mujoco_ros2_control` from source; if that fails, real time,
  reported as degraded.

## My calls (not discussed; change them if wrong)

- Chain names `kangaroo_left4`, `kangaroo_right4`; joint limits from the URDF, narrowed to the SO-2
  measured range where that is smaller (the sim cannot reach beyond it), with the source of each limit
  noted in `chains.py`.
- Start pose: PAL home is the simulator's and each run's first pose only. IK reaches use the test pairs'
  own random starts (`ring_so3/ik/pairs.py`), on the light robot and MuJoCo alike, the protocol Gazebo used.
- Calibration files: experiments load the checked-in `artifacts/` (calibration code untouched). The re-run
  covers the default design (`calibration_N200_f120_p200_e60_seed1_fractional`) and its IK calibration
  only, written elsewhere; the finding is the largest absolute difference of every stored array, and
  whether the files are identical. If they differ beyond the fourth digit, report it and stop for the owner.
- Unreachable pairs: the pair set is not filtered. A trial that does not reach is counted a miss and
  classified (at a joint limit / blocked: joints stalled away from their targets / slow), so the
  success rate reads against the same pairs on both robots.
- Plant as PAL ships it (gains untouched). Like SO-2 D13, `KangarooRobot` takes its starting command from
  the controller's held reference; its tolerance is set from the measured gravity offset and reported.
- Results go under `docs/kangaroo/results/` (data + figures), notes under `docs/experiments/`.
- The Kangaroo container is a new mode of `docker/run.bash` (`kangaroo up|exec|down`), not a second script.
- The IK simulator's existing "Gazebo" camera panel is not ported; MuJoCo's own window shows the robot.

## Left to the builder

- Exact base and end links of the chain, after reading the URDF.
- Whether the closed-loop script is a new Kangaroo script or the Gazebo script with a robot
  choice, whichever is the smaller diff (built: `scripts/run_ik_gazebo.py` with a Kangaroo chain).
- Number of IK test pairs per arm on MuJoCo (enough for a median and p90; Gazebo used 30), within a
  run time of a few hours.
- Which existing report code (`scripts/report.py`, `scripts/ik_report.py`) to extend for the figures, or a
  small new script if extending is larger.

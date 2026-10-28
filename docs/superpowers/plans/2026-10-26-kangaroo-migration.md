# Kangaroo Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Put PAL's Kangaroo (4-DoF arms, MuJoCo, ROS 2 Humble) behind the SO(3) ring network as a lock-stepped robot, calibrate and measure both arms, and drive it from the existing IK simulator page.

**Architecture:** The network is untouched. Two chain entries (`kangaroo_left4`, `kangaroo_right4`) come from the URDF; `KangarooRobot` is `GazeboRobot` (same calls, same lock-step loop) over a new ROS 2 link that pauses MuJoCo and advances it with `StepSimulation`. A `kangaroo` mode of `docker/run.bash` runs everything in the image built from `docker/kangaroo/`. Measurements reuse the existing experiment scripts; one small report script turns their JSON into figures.

**Tech Stack:** Python 3.10 (image) / 3.8-compatible code, NumPy, NEST `HEAD@41892a5`, rclpy, `mujoco_ros2_control` 0.1.2, Matplotlib, unittest, Docker.

**Spec:** `docs/superpowers/specs/2026-10-26-kangaroo-migration-design.md`

## Global Constraints

- Branch `kangaroo-migration`; never push; commit only when the owner asks (commit steps below are "stage + show diff", the owner commits).
- No change to the network, its wiring, populations, or calibration code (`ring_so3/nest/*`, `ring_so3/wiring.py`, `ring_so3/ik/wiring.py`, `ring_so3/neurons.py`).
- Never overwrite `artifacts/` or anything under `docs/results/`, `docs/ik/results/`; new outputs go under `docs/kangaroo/results/`.
- Every number in a document traces to a JSON file under `docs/kangaroo/results/data/`; every output records seed, threads, commit.
- Kangaroo values come from the URDF in the image (`/opt/ros/humble/share/kangaroo_mujoco/models/mjcf_data_floating_4dof_no-end-effector_no-end-effector_fixed/robot_description_formatted.urdf`, verified equal to the live `/robot_description` for all 13 arm joints on 2026-10-26).
- Style of `ring_so3/`: explanatory module docstrings, short functions, numpy; no options nobody uses.
- Image tag `ring-so3-kangaroo:humble`, built from `docker/kangaroo/Dockerfile` (already copied from SO-2, + `python3-matplotlib`; built and verified 2026-10-26: NEST `HEAD@41892a50f`, Matplotlib 3.5.1).
- Tests: unittest, `python3 -B -m unittest`; behaviour only, independent oracles, main functionality only.

## Facts established during planning (2026-10-26, in the built image)

- `/mujoco_ros2_control_node/set_pause` (`SetPause`: `bool paused` → `success, message`) and `/mujoco_ros2_control_node/step_simulation` (`StepSimulation`: `uint32 steps` → `success, message`) exist; stepping needs a paused sim. While paused `/clock` is not published. 100 steps advanced `/clock` by exactly 0.0500 s (physics step 0.0005 s); one 100-step call took 0.12 s wall.
- `/joint_states` is published at 50 Hz (`joint_state_broadcaster` `update_rate: 50`), so after a 50 ms step the newest joint state can be up to 20 ms old.
- Controllers: `arm_left_controller`, `arm_right_controller` (JointTrajectoryController, topic `/arm_<side>_controller/joint_trajectory`), `joint_state_broadcaster`; arm joints `arm_<side>_{1..4}_joint`.
- MuJoCo's viewer needs X (GLFW): without `DISPLAY` `ros2_control_node` dies ("could not initialize GLFW"). With `DISPLAY` + xauth + `--gpus all` the launch comes up in about 45 s.
- Arm geometry (both sides, base `torso_link`, end `arm_<side>_tool_link`):

| joint | parent → child | xyz | rpy | axis | lower, upper | vel |
|---|---|---|---|---|---|---|
| arm_left_fixed_joint | torso_link → arm_left_base_link | 0 0.105954 0.384396 | 1.3963 3.14159 3.14159 | fixed | | |
| arm_right_fixed_joint | torso_link → arm_right_base_link | 0 -0.105954 0.384396 | 1.3963 3.14159 0 | fixed | | |
| arm_<s>_1_joint | base → 1 | 0 0 0 | 3.1416 0 3.1416 | 0 0 -1 | -2.6179938779914944, 2.6179938779914944 | 1.95 |
| arm_<s>_2_joint | 1 → 2 | 0.03711 0.02 -0.17011 | 0 1.5708 0 | 0 0 1 | -1.1344640137963142, 2.443460952792061 | 1.95 |
| arm_<s>_3_joint | 2 → 3 | 0.15989 -0.02 -0.03711 | 0 -1.5708 0 | 0 0 -1 | -2.6179938779914944, 2.6179938779914944 | 2.35 |
| arm_<s>_4_joint | 3 → 4 | -0.03711 -0.02 -0.17011 | 3.14159 1.5708 0 | 0 0 1 | -1.1345, 2.4435 | 2.35 |
| arm_<s>_tool_fixed_joint | 4 → tool | 0.35 -0.02 -0.0255 | 0 0 0 | fixed | | |

- SO-2 measured ranges (`../Ring-Attractor-SO-2-Joint-Architecture/src/kangaroo_ring_controller/results/joint_limits/`): only `arm_<s>_2_joint`'s upper limit is narrower than the URDF: left 1.4229904547193268 (`left_j1_20261013_180318`), right 1.4622630704048099 (`right_j1_20261013_125903`). All other measured ranges are as wide as or wider than the URDF.
- PAL home (SO-2 `joint_setup.py:16`): left `[0.239, 1.321, 1.57, 0.8]`, right `[-0.239, 1.321, -1.57, 0.8]`.

## Decisions made while planning (owner to check)

1. **Negative axes.** `Joint.axis` is `x|y|z` and the wiring depends on it, so joints 1 and 3 (axis −z) are folded: `Rot(−z, q) = Rx(π)·Rz(q)·Rx(π)`. The joint's fixed rotation gets `·Rx(π)` on the right, the next joint's fixed rotation and offset (or the tool's) get `Rx(π)·` on the left. Joint angles equal the robot's; links 2, 4 and the tool frame equal the URDF exactly; links 1 and 3 are the URDF frames turned half a turn about their x axis.
2. **Rounding.** The URDF writes 1.5708, 3.1416 and 3.14159 for π/2 and π (off by ≤ 3.7e-6 rad). The table keeps the URDF numbers; the chain builder snaps matrix entries within 1e-5 of 0 or ±1 (≤ 0.0006°), as `_matrix_tuple` does for TIAGo with 1e-12, so the wiring does not gain products with weight ~4e-6. The URDF test uses `atol=1e-5`.
3. **SO-2 D13 does not carry over.** SO-2 integrated velocities into a command state, so the gravity offset accumulated; here `follow` sends absolute targets (the goal-ring angle), so there is no command state to prime. Instead the robot's `tolerance` is set from the measured steady offset (Task 2).
4. **Stale joint states** are not hidden: the link waits for a joint state stamped after the step started, and records `sim time − stamp` every tick; the maximum goes into every result file.

## Review Focus

1. Sim not running or services missing: `Ros2Link` must raise a clear `RuntimeError` within ~30 s, not hang (Task 2, `test_no_sim_fails_fast`).
2. A run that ends (Ctrl-C, exception) leaves MuJoCo paused and frozen: `Ros2Link.close()` unpauses; scripts call it in `finally` (Task 2, `test_close_unpauses`).
3. A goal or start beyond the reachable range (shoulder `arm_<s>_2_joint` above ~1.42-1.46 rad): targets are clipped to the chain limits (Task 2, `test_targets_clip_to_the_measured_shoulder_limit`).
4. Wrong side: a left-arm chain on the right link must be refused (Task 2, `test_wrong_side_is_refused`).
5. Simulator offered Kangaroo while no sim runs, or a TIAGo chain with the Kangaroo robot: not offered / ignored (Task 4, `test_kangaroo_is_offered_only_for_its_chains`).

## Agents (owner's process)

Per task: the **test writer** gets the task's *Contract* and *Tests* sections only and writes the tests first (they must fail); the **code writer** gets the *Contract* section only (never the tests) and implements; the orchestrator runs the tests; then the **semantic checker** (does it do what the spec and Contract say: frames, angle conventions, units, limits, same robot calls as `GazeboRobot`) and the **authenticator** (are the numbers the URDF's, do topics/services/types exist in the running sim, does every reported number trace to a file) review the diff. Failures go back to the code writer as behaviour, never as test code.

---

### Task 1: Kangaroo arm chains

**Files:**
- Modify: `ring_so3/chains.py` (table, builder, registry, `KANGAROO_MODELS`, docstring)
- Test: `tests/test_kangaroo_chains.py`

**Contract:**
- `KANGAROO_ARM: Dict[str, Tuple]`: per side, rows `(name, xyz, rpy, axis_sign, lower, upper, velocity)` for joints 1-4 with the URDF numbers of the table above (axis_sign −1 for joints 1 and 3); `arm_<s>_2_joint` upper = the SO-2 measured value of that side, with a comment naming the SO-2 file; `KANGAROO_MOUNT[side] = (xyz, rpy)` (`arm_<s>_fixed_joint`); `KANGAROO_TOOL = ((0.35, -0.02, -0.0255), (0.0, 0.0, 0.0))`; `KANGAROO_BASE_LINK = "torso_link"`, `KANGAROO_TOOL_LINK = "arm_%s_tool_link"`.
- `kangaroo_chain(side: str) -> Chain`: name `"kangaroo_%s4" % side`, joints named `arm_<side>_<k>_joint`, axis `"z"`, `circular=False`, limits and `max_velocity` from the table. First joint: `fixed = R(mount_rpy)·R(j1_rpy)`, `offset = mount_xyz`. Fold negative axes as Decision 1:

```
carry = I
for each row: A = carry · rpy_matrix(*rpy) (joint 1: R(mount_rpy) · rpy_matrix(*rpy)); offset = carry · xyz (joint 1: mount_xyz)
              if axis_sign < 0: A = A · Rx(π); carry = Rx(π) else carry = I
tool_fixed = carry · rpy_matrix(*tool_rpy); tool_offset = carry · tool_xyz
```
  and snap entries as Decision 2 (a tolerance argument to the existing `_matrix_tuple`, default 1e-12 for TIAGo).
- Registered in `CHAINS`; `KANGAROO_MODELS = ["kangaroo_left4", "kangaroo_right4"]`. `MODEL_NAMES` unchanged (its 12-model test stays).

**Tests (test writer):** independent of `ring_so3.kinematics`, compose 4×4 transforms from the URDF file in the image as the existing `TiagoUrdfTests._urdf_frames` does (skip when the file is absent).
- `test_tool_orientation_equals_the_urdf`: 25 random poses inside the limits, both sides: `chain.rotation(q)` equals the URDF `torso_link → arm_<s>_tool_link` rotation, `atol=1e-5`.
- `test_link_frames_equal_the_urdf_up_to_the_folded_half_turn`: links 2, 4 equal the URDF link frames; links 1, 3 equal URDF frame · Rx(π); origins of every joint equal the URDF joint origins (`atol=1e-5`).
- `test_limits_and_speeds_are_the_urdf_values_except_the_measured_shoulder`: lower/upper/velocity equal the URDF for all joints except `arm_<s>_2_joint` upper = 1.4229904547193268 (left) / 1.4622630704048099 (right).

- [ ] **Step 1:** Test writer writes `tests/test_kangaroo_chains.py`.
- [ ] **Step 2:** Run `docker/run.bash kangaroo run -- 'python3 -B -m unittest tests.test_kangaroo_chains -v'` (Task 3's `run` action; until it exists: `docker run --rm -v "$PWD:/work" -w /work ring-so3-kangaroo:humble bash -c 'source /opt/ros/humble/setup.bash; python3 -B -m unittest tests.test_kangaroo_chains -v'`). Expected: FAIL (`unknown chain 'kangaroo_left4'`).
- [ ] **Step 3:** Code writer implements the Contract.
- [ ] **Step 4:** Same command: PASS; then the whole suite `python3 -B -m unittest discover -s tests` in the image — everything that passed before still passes (TIAGo URDF tests skip there). Report verbatim.
- [ ] **Step 5:** Semantic check + authenticator (URDF numbers, SO-2 limit values and files); stage and show the diff.

### Task 2: ROS 2 link and `KangarooRobot`

**Files:**
- Create: `ring_so3/kangaroo/__init__.py` (one-line docstring, as `ring_so3/gazebo/__init__.py`), `ring_so3/kangaroo/link.py`
- Modify: `ring_so3/robots.py` (`GazeboRobot` chain check via a class attribute; new `KangarooRobot`; module docstring)
- Test: `tests/test_kangaroo_robot.py` (stand-in link, host), `tests/test_kangaroo_sim.py` (live sim, skipped without it)

**Contract:**
- `ring_so3/kangaroo/link.py`, docstring in the style of `gazebo/link.py` (what is taken from where: SO-2 `kangaroo_ring_controller/arm.py` for topics; the facts above).
  - `ARM_JOINTS = {side: tuple("arm_%s_%d_joint" % (side, k) for k in 1..4)}`, `PAUSE_SERVICE = "/mujoco_ros2_control_node/set_pause"`, `STEP_SERVICE = "/mujoco_ros2_control_node/step_simulation"`, `PHYSICS_STEP_S = 0.0005`.
  - `class Ros2Link(side: str, node_name="ring_ik", timeout_s=30.0)`: `rclpy.init()` if not initialised; node with `use_sim_time`; a background executor thread for the subscriptions (`/joint_states`, `/clock`); command publisher on `/arm_<side>_controller/joint_trajectory`; waits for both services and a joint state with all four joints, else `RuntimeError` naming what is missing; then pauses.
  - `joint_state() -> (positions: list[4], velocities: list[4], stamp_s: float)`; `sim_time_s() -> float`.
  - `send(positions: Sequence[float], time_s: float)`: one-point `JointTrajectory`, stamp 0 (start on receipt, as SO-2), `time_from_start = time_s`.
  - `step(iterations: int) -> float`: calls `StepSimulation(steps=iterations)` (raises on `success=False`), then waits (≤ 1 s wall) for a joint state stamped after the sim time at call; sets `self.stale_s = sim_time − stamp` and `self.stale_max_s`; returns sim time.
  - `put_base_back() -> bool`: returns `False` (Kangaroo stands on fixed feet; nothing moved). `base_pose() -> None`. `frames() -> 0`.
  - `close()`: unpause, destroy node; idempotent.
- `robots.py`: `GazeboRobot.chain_prefix = "tiago"` used by the existing check (message unchanged in meaning). `class KangarooRobot(GazeboRobot)`: `name = "kangaroo"`, `place_speed = 1.0`, `tolerance = <measured, Step 3a>`, `__init__(chain, link)` sets `chain_prefix = "kangaroo_" + link.side` and `physics_step_s = link.PHYSICS_STEP_S`; `describe()` adds `joint_state_stale_max_s` and `tolerance`.

**Tests (test writer):**
- `tests/test_kangaroo_robot.py` (host, a stand-in link like `tests/test_robots.py::FakeLink` with `side`, `PHYSICS_STEP_S = 0.0005`, `stale_max_s`):
  - `test_one_follow_is_one_tick_of_physics`: `follow(target, 0.8, 0.05)` advances the link by exactly 100 steps (0.05 s) and sends once.
  - `test_targets_clip_to_the_measured_shoulder_limit`: target `arm_<s>_2_joint = 2.0` is sent as the chain's upper limit (1.4229904547193268 left).
  - `test_wrong_side_is_refused`: `KangarooRobot(get_chain("kangaroo_left4"), right_link)` and a TIAGo chain raise `ValueError`.
  - `test_the_closed_loop_runs_on_it`: `IkSession(StubIkNetwork(chain, IkDesign()), robot=...)` reaches a goal 0.25 rad away from mid-range within 5°.
- `tests/test_kangaroo_sim.py` (in `docker/run.bash kangaroo exec`, skipped unless `rclpy` imports and `/mujoco_ros2_control_node/step_simulation` is available within 5 s):
  - `test_follow_advances_the_clock_by_one_tick`: sim time after `follow` − before = 0.05 ± 1e-6; the commanded joint moved toward its target.
  - `test_tool_frame_matches_the_robot_tf`: at 3 poses (reset there), `chain.rotation(robot.q)` vs TF `torso_link → arm_<s>_tool_link` (tf2, from the sim's robot_state_publisher) within 1e-3 rad of angle.
  - `test_close_unpauses`: after `close()` a fresh node sees `/clock` advance within 2 s.
  - `test_no_sim_fails_fast` (in `kangaroo run`, no sim; skipped where the sim runs): `Ros2Link("left", timeout_s=5)` raises `RuntimeError` within 15 s.

- [ ] **Step 1:** Test writer writes both files. **Step 2:** run the host file in `kangaroo run`: FAIL (import error).
- [ ] **Step 3a (orchestrator, before code):** measure the steady offset: sim up, hold PAL home 2 s via the controller, read `|reference − measured|` from `/arm_<s>_controller/controller_state` for all 8 joints; tolerance = max offset × 2 rounded up to 0.005 rad. Record numbers in `docs/experiments/2026-10-26_kangaroo_plant_checks.md` and give the value to the code writer.
- [ ] **Step 3b:** Code writer implements the Contract.
- [ ] **Step 4:** host tests PASS in `kangaroo run`; sim tests PASS in `kangaroo exec` (Task 3); `tests/test_robots.py` still PASS. Verbatim.
- [ ] **Step 5:** Semantic check (same calls and loop as `GazeboRobot`; no `rospy.sleep`-style wall waits on sim time) + authenticator (topics, services, types against the running sim); stage, show diff.

### Task 3: Container mode

**Files:**
- Modify: `docker/run.bash` (usage text, `kangaroo` mode), `docker/image.env` (add `KANGAROO_IMAGE=ring-so3-kangaroo:humble` and a comment: built locally from `docker/kangaroo/`)
- Existing: `docker/kangaroo/Dockerfile`, `docker/kangaroo/kangaroo_sim.launch.py` (copied; launch docstring: "SO-2 repo's docs/kangaroo/DECISIONS.md D7")

**Contract:**
- `docker/run.bash kangaroo up NAME [--port 8765] [--out DIR]`: builds the image if missing; needs `DISPLAY` and `xauth` (MuJoCo's viewer) and uses `--gpus all` with SO-2's PRIME flags; starts `ros2 launch /opt/kangaroo_sim.launch.py` detached, repo at `/work`, `-p 127.0.0.1:PORT:PORT`, `SIM_PORT`; prints "Kangaroo ready after N s" once `ros2 control list_controllers` shows `arm_left_controller` and `arm_right_controller` active (timeout 300 s, then the container's last log lines and exit 1).
- `kangaroo exec NAME -- CMD`, `kangaroo down NAME`: as the gazebo actions.
- `kangaroo run [--out DIR] -- CMD`: one-shot in the image, no sim, `--network none` (calibration, light-robot runs, host tests).
- Setup string: `source /opt/ros/humble/setup.bash; source /usr/local/nest/bin/nest_vars.sh; export PYTHONPATH=/work:${PYTHONPATH:-}; cd /work`, same env as the other modes (`PYTHONDONTWRITEBYTECODE=1`, `MPLBACKEND=Agg`, user = host user).

**Tests:** no unit test (shell). Verification:
- [ ] **Step 1:** `docker/run.bash kangaroo run -- 'python3 -B -c "import nest, rclpy, ring_so3; print(nest.__version__)"'` prints `HEAD@41892a50f`.
- [ ] **Step 2:** `docker/run.bash kangaroo up k1` → "Kangaroo ready"; `docker/run.bash kangaroo exec k1 -- 'ros2 service list | grep step_simulation'` lists it; `down` stops it.
- [ ] **Step 3:** Full existing suite in the image: `docker/run.bash kangaroo run -- 'python3 -B -m unittest discover -s tests'`; report verbatim, separate Python-3.10 failures from TIAGo skips. Stage, show diff.

### Task 4: IK simulator drives Kangaroo

**Files:**
- Modify: `scripts/ik_simulator.py` (`--kangaroo`, chain choices `MODEL_NAMES + KANGAROO_MODELS`, `close()` links in `finally`), `ring_so3/sim/ik_server.py` (robot ↔ chain compatibility instead of `startswith("tiago")`), `ring_so3/sim/ik_page.py` (robot label, panel note), `README.md` (section 11.6 "With the Kangaroo in MuJoCo", three steps as 11.2)
- Test: `tests/test_ik_sim.py` (one new test)

**Contract:**
- `ik_server.ROBOT_CHAINS = {"gazebo": "tiago", "kangaroo": "kangaroo_"}`; a robot other than `light` is usable for a chain iff the chain name starts with its prefix; config sends `"robot_chains": {robot: [chain names]}` for the offered robots (replaces `gazebo_chains`); chains offered = `MODEL_NAMES + KANGAROO_MODELS`; the snapshot's lock-step block (today keyed `gazebo`) is sent for any non-light robot.
- `ik_simulator.py --kangaroo`: one `Ros2Link` per side on demand (the chain's side), `make_robot = lambda name, chain: KangarooRobot(chain, link_for(chain))`; default robot `kangaroo` when the chain is a Kangaroo chain. On exit every link is closed.
- Page: robot option labels `light` "Light robot (no dynamics)", `gazebo` "TIAGo in Gazebo", `kangaroo` "Kangaroo in MuJoCo"; option disabled unless the chain is in `robot_chains[robot]`; for `kangaroo` the panel shows "MuJoCo shows the robot in its own window" and the sim-time/lag line, no picture.

**Tests (test writer):** `test_kangaroo_is_offered_only_for_its_chains`: an `IkSimulator` with a stub network on `kangaroo_left4`, `robots=("light","kangaroo")` and a `make_robot` returning a stand-in robot: config lists `kangaroo_left4` under `robot_chains["kangaroo"]` and no TIAGo chain; after switching the chain to `tiago7`, `{"command":"robot","robot":"kangaroo"}` is ignored (robot stays `light`). Existing `test_gazebo_is_offered_only_when_it_is_there` still passes.

- [ ] Steps: test → FAIL → implement → PASS (host via `kangaroo run`) → manual: `kangaroo up`, `kangaroo exec ... ik_simulator.py --chain kangaroo_left4 --kangaroo`, open the page, give one goal, see MuJoCo's arm move; same for the right arm. Note what was seen in the plant-checks note. Semantic check + authenticator; stage, show diff.

### Task 5: Closed-loop script for MuJoCo

**Files:** Modify `scripts/run_ik_gazebo.py` (chain choices `tiago2..7 + KANGAROO_MODELS`; Kangaroo chain → `KangarooRobot(chain, Ros2Link(side))`, closed in `finally`; docstring and print say Gazebo or MuJoCo by chain).

**Contract:** same outputs and summary line as today; `document["robot"]` carries `KangarooRobot.describe()`.

- [ ] Verification (no new unit test; the reach code is `reach_run`, already tested): `kangaroo exec k1 -- 'python3 -B scripts/run_ik_gazebo.py --chain kangaroo_left4 --pairs 2 --out /out/try.json'` finishes with 2 trials; JSON has `robot.joint_state_stale_max_s`. Stage, show diff.

### Task 6: Calibration re-run (experiment)

- [ ] `docker/run.bash kangaroo run --out docs/kangaroo/results/data/calibration -- 'python3 -B scripts/calibrate.py --out /out/calibration_N200_f120_p200_e60_seed1_fractional.npz && python3 -B scripts/calibrate_ik.py --out /out/ik_calibration_N200_f120_p200_e60_seed1_fractional.npz'` (`calibrate_ik.py` reads the checked-in forward calibration, as it always does).
- [ ] Compare with `artifacts/` (inline `python3 -c` in the note, printed per array: shape, largest |difference|, identical?). Write the printout to `docs/kangaroo/results/data/calibration/comparison.json`.
- [ ] Note `docs/experiments/2026-10-26_kangaroo_calibration_rerun.md` (hypothesis: NEST at the same commit reproduces the files on Python 3.10; config, seed, threads, command, commit, result). **If any array differs beyond 1e-4 relative: stop and ask the owner** (spec).

### Task 7: Measurements (experiments; long runs in the background, each resumable)

All on both arms, outputs under `docs/kangaroo/results/data/`, seeds/threads recorded by the scripts:
- [ ] Held poses, light robot: `run_static.py --chain kangaroo_<s>4 --out .../static/kangaroo_<s>4.json` (300 poses + sweeps, `--threads 8`).
- [ ] Tracking, light robot: `run_tracking.py --chain kangaroo_<s>4 --speed {0.25,0.5,1.0} --out .../tracking/kangaroo_<s>4_<speed>.json`.
- [ ] IK reaches, light robot: `run_ik_reach.py --chain kangaroo_<s>4 --pairs 30 --out .../reach/light_ring_<s>.json` (test pairs, `TEST_SEED`).
- [ ] IK reaches, MuJoCo: `run_ik_gazebo.py --chain kangaroo_<s>4 --pairs 30 --out .../reach/mujoco_ring_<s>.json` and `--method dls --out .../reach/mujoco_dls_<s>.json` (same pairs). Before each arm, `kangaroo up` fresh; time guard `--max-minutes 110`, finish cut runs with `--only`.
- [ ] Note `docs/experiments/2026-10-26_kangaroo_measurements.md`: hypothesis (the 4-joint readout meets the marks set for the other chains: median ≤ 5°, p90 ≤ 10°; MuJoCo reaches close to light-robot reaches), every command, commit, results from the tables (Task 8), misses classified (at limit / blocked: joints stalled away from targets / slow), negative results.

### Task 8: Figures and tables

**Files:** Create `scripts/kangaroo_report.py` (reuse `scripts/report.py`'s `read`, `rows_of`, `stats`, `tracking_summary` by import; new code only for what they do not cover). Test: `tests/test_kangaroo_report.py`.

**Contract:** `python3 -B scripts/kangaroo_report.py [--data docs/kangaroo/results/data] [--out docs/kangaroo/results]` writes, for whatever data exists (a missing file is listed in the manifest, not an error):
- `figures/static_error.{png,pdf}`: orientation error distribution per arm, marks 5° and 10°;
- `figures/error_along_arm.{png,pdf}`: per-link median error per arm;
- `figures/tracking.{png,pdf}`: delay and error at the three speeds per arm;
- `figures/reach_error_over_time.{png,pdf}`: distance from goal over time, median and IQR, light-ring / MuJoCo-ring / MuJoCo-dls, per arm;
- `figures/mujoco_lag.{png,pdf}`: per-tick joint lag (target − measured) on MuJoCo, and joint-state staleness;
- `tables.md`: every number the notes quote (static median/p90/max, tracking delay, reach: reached %, final error, time to 5°, path ratio, misses by class, calibration comparison);
- `figures/manifest.json`: inputs (path, size, sha256), commit, date, and the seeds/threads read from the inputs.

**Tests (test writer):** `test_report_from_minimal_data`: synthetic data files in a temp dir (two static rows per arm with known errors, one reach row) → the files above exist, `tables.md` contains the medians computed by hand, manifest lists every input with its sha256 and lists the missing ones.

- [ ] test → FAIL → implement → PASS → run on the real data → semantic check + authenticator (every number in `tables.md` recomputed from the JSON by the authenticator) → stage, show diff.

### Task 9: Docs and hand-over

- [ ] `README.md`: section 11.6 (Task 4), a line in section 10's table for Kangaroo (code, docs, data), status line in section 9.
- [ ] `docs/kangaroo/README.md`: one page: what was migrated, decisions 1-4 above and the spec's "my calls", results summary pointing at `docs/kangaroo/results/tables.md` and the three experiment notes, open items.
- [ ] Whole-branch review (semantic checker + authenticator on the full diff), `ponytail-review` for over-engineering; final full test run verbatim.

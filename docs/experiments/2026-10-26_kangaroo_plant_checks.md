# 2026-10-26 Kangaroo plant checks

Facts about PAL's simulated Kangaroo (MuJoCo, `mujoco_ros2_control` 0.1.2, image `ring-so3-kangaroo:humble`) that the
robot code relies on. Commit: 9e3ed24 (branch `kangaroo-migration`, uncommitted work on top).

## Image
The Dockerfile installs PAL's packages unpinned, so a later build may pull another release. The image these runs
used: `ring-so3-kangaroo:humble`, id `sha256:d69ea6aa2d95…16595`, built 2026-10-26T06:14Z; Python 3.10.12, NumPy 1.21.5,
NEST `HEAD@41892a50f`. Packages (`dpkg-query -W 'ros-humble-kangaroo*' 'ros-humble-mujoco*' 'ros-humble-pal*'`):
kangaroo-description, -bringup, -controller-configuration 2.15.1; kangaroo-mujoco, -simulation 2.7.0;
kangaroo-moveit-config 2.2.2; mujoco-ros2-control (+ -msgs, -plugins) 0.1.2; mujoco-vendor 0.1.1;
pal-sea-arm-description, -controller-configuration 2.8.4; pal-sea-arm-moveit-config 1.0.7; pal-urdf-utils 2.11.1;
pal-pro-gripper-* 1.12.6 (Debian builds of 2026-09-07 to 2026-10-06).

## Forward kinematics against the robot's TF
`tests/test_kangaroo_sim.py` (`test_tool_frame_matches_the_robot_tf`, left arm, live sim) compares the
chain's tool frame with TF `torso_link` → `arm_left_tool_link`: rotation and position agree, largest position
difference 7.6e-6 m over three poses (tolerance 1e-3 m). Run of 2026-10-27, as printed by the test:
`docs/kangaroo/results/data/logs/fk_tf_test.log`.

## Lock step
- `/mujoco_ros2_control_node/set_pause` and `/step_simulation` exist. While paused `/clock` is not published.
- 100 steps advance `/clock` by exactly 0.0500 s (physics step 0.0005 s); one 100-step call took 0.12 s of wall time.
- `/joint_states` comes at 50 Hz (`joint_state_broadcaster` `update_rate: 50`): after a 50 ms step the newest joint
  state can be up to 20 ms old. The link records this per tick (`joint_state_stale_max_s` in every result file).

## Steady offset under gravity (tolerance of `KangarooRobot`)
- Hypothesis: PAL's arm controllers hold a pose with a small steady offset (SO-2 measured about 0.01 rad, SO-2 D13).
- Command: `docker/run.bash kangaroo up kprobe`, then
  `docker/run.bash kangaroo exec kprobe -- 'python3 -B docs/kangaroo/results/data/plant/offset_probe.py'`.
- Config: both arms sent to PAL home (left [0.239, 1.321, 1.57, 0.8], right [-0.239, 1.321, -1.57, 0.8]) in 2 s,
  3 s of sim time, then reference - measured averaged over 1 s (50 samples per arm) from
  `/arm_<side>_controller/controller_state`. No randomness.
- Result (`docs/kangaroo/results/data/plant/offset.json`): largest |offset| 0.00137 rad (right joint 2), left
  [0.00127, 0.00086, 0.000004, 0.00047], right [0.00097, 0.00137, 0.00013, 0.00047] rad.
- Interpretation: about 7x smaller than the 0.01 rad SO-2 reported (SO-2's number came from velocity-integrated commands,
  where the offset accumulated). Tolerance = 2 x max, rounded up to 0.005 rad: **0.005 rad**.

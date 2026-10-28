"""The connection to a running MuJoCo with PAL's Kangaroo: joint states in, arm targets out, physics in lock step.

Topics and messages taken from the SO-2 repo (``kangaroo_ring_controller/arm.py``: ``/joint_states``,
``/arm_<side>_controller/joint_trajectory``, a one-point ``JointTrajectory`` stamped 0, so the controller starts
it on receipt); lock step from ``mujoco_ros2_control`` (facts in ``docs/experiments/2026-10-26_kangaroo_plant_checks.md``):

* physics stays paused.  :meth:`Ros2Link.step` calls ``step_simulation``, which returns when the steps are done
  (100 steps = 0.05 s on ``/clock``), so there is no overshoot and no absolute schedule as for Gazebo;
* nothing here waits on the node's clock: it is sim time, and ``/clock`` is not published while paused.  Waits
  are on wall time, for messages, and every one is bounded (RuntimeError when it runs out);
* ``/clock`` lags at rest: right after a pause the clock of the last steps may still be on its way, and the last
  step of a ``step_simulation`` call is now and then not published.  So sim time is counted:
  from the clock of one single step at construction (taken once 50 ms passed without a clock after the pause), plus
  the steps asked for (shared by the links of a process: one simulation), or ``/clock`` where that is ahead (someone
  else ran the simulation);
* ``/joint_states`` comes at 50 Hz, so after a tick the newest joint state can be up to 20 ms old:
  :meth:`Ros2Link.step` waits for that newest one and records its age (``stale_s``, ``stale_max_s``); now and then
  one is lost (as the last ``/clock``): then the newest there is stays and ``missed_states`` counts it;
* subscriptions are served by an executor in a daemon thread; services are called with ``call_async`` and a wait
  on the future, so two links (left and right) can live in one process;
* Kangaroo stands on fixed feet: there is no base to put back and no camera.
"""

from __future__ import annotations

import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

ARM_JOINTS: Dict[str, Tuple[str, ...]] = {side: tuple("arm_%s_%d_joint" % (side, k) for k in range(1, 5))
                                          for side in ("left", "right")}
PAUSE_SERVICE = "/mujoco_ros2_control_node/set_pause"
STEP_SERVICE = "/mujoco_ros2_control_node/step_simulation"
JOINT_STATES_TOPIC = "/joint_states"
PHYSICS_STEP_S = 0.0005
JOINT_STATE_PERIOD_S = 0.02


class Ros2Link:
    PHYSICS_STEP_S = PHYSICS_STEP_S
    _counted_s = 0.0                          # sim time counted from the steps the links of this process asked for

    def __init__(self, side: str, timeout_s: float = 30.0) -> None:
        import rclpy
        from mujoco_ros2_control_msgs.srv import SetPause, StepSimulation
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        self._SetPause, self._StepSimulation = SetPause, StepSimulation
        self._JointTrajectory, self._JointTrajectoryPoint = JointTrajectory, JointTrajectoryPoint
        self.side = side
        self.joints = ARM_JOINTS[side]
        self._changed = threading.Condition()
        self._sim_time_s = 0.0
        self._state: Optional[Tuple[List[float], List[float], float]] = None
        self.stale_s = 0.0
        self.stale_max_s = 0.0
        self.missed_states = 0
        if not rclpy.ok():
            rclpy.init()
        # one node per side: two nodes of one name in a process clash on their parameter services
        self._node = rclpy.create_node("ring_ik_%s" % side, parameter_overrides=[Parameter("use_sim_time", value=True)])
        self._command = self._node.create_publisher(JointTrajectory, "/arm_%s_controller/joint_trajectory" % side, 1)
        self._node.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joints, 10)
        self._node.create_subscription(Clock, "/clock", self._on_clock, 10)
        self._pause = self._node.create_client(SetPause, PAUSE_SERVICE)
        self._step = self._node.create_client(StepSimulation, STEP_SERVICE)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spinner = threading.Thread(target=self._executor.spin, daemon=True)
        self._spinner.start()
        self._found = False
        try:
            self._connect(float(timeout_s))
        except BaseException:
            if self._found:                   # leave the simulation running, as found; the first error is the one raised
                try:
                    self._set_pause(False)
                except Exception:  # noqa: BLE001
                    pass
            self._shutdown()
            raise

    def _connect(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        missing = [name for name, client in ((PAUSE_SERVICE, self._pause), (STEP_SERVICE, self._step))
                   if not client.wait_for_service(timeout_sec=max(deadline - time.monotonic(), 0.0))]
        if not missing:
            self._found = True
            self._set_pause(False)            # joint states come only while physics runs (an earlier script may have left it paused)
            with self._changed:
                self._changed.wait_for(lambda: self._state is not None, timeout=max(deadline - time.monotonic(), 0.0))
            if self._state is None:
                missing.append("%s with %s" % (JOINT_STATES_TOPIC, ", ".join(self.joints)))
        if missing:
            raise RuntimeError("no Kangaroo simulation within %.0f s: missing %s" % (timeout_s, "; ".join(missing)))
        # a lost step cannot be asked again; so first ask for 0 steps until answered ("must be positive"; nothing steps)
        self._call(self._step, self._StepSimulation.Request(steps=0), check=False, attempts=3)
        self._set_pause(True)
        settle = time.monotonic() + 1.0       # the clock of the last steps is still on its way: wait for 50 ms without one
        with self._changed:
            seen = None
            while seen != self._sim_time_s:
                if time.monotonic() > settle:
                    raise RuntimeError("/clock still moves 1 s after the pause: something else runs the simulation")
                seen = self._sim_time_s
                self._changed.wait_for(lambda: self._sim_time_s != seen, timeout=0.05)
        for _ in range(3):                    # one more step publishes one clock message, the last: the true time
            self._call(self._step, self._StepSimulation.Request(steps=1), timeout_s=10.0)
            with self._changed:
                if self._changed.wait_for(lambda: self._sim_time_s != seen, timeout=0.2):
                    Ros2Link._counted_s = self._sim_time_s
                    return
        raise RuntimeError("no /clock after 3 single steps")

    # -- callbacks -----------------------------------------------------------
    def _on_joints(self, message) -> None:
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity)) if message.velocity else {}
        if any(name not in positions for name in self.joints):
            return
        with self._changed:
            self._state = ([float(positions[name]) for name in self.joints], [float(velocities.get(name, 0.0)) for name in self.joints],
                           message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec)
            self._changed.notify_all()

    def _on_clock(self, message) -> None:
        with self._changed:
            self._sim_time_s = message.clock.sec + 1e-9 * message.clock.nanosec
            self._changed.notify_all()

    # -- services ------------------------------------------------------------
    def _call(self, client, request, timeout_s: float = 2.0, check: bool = True, attempts: int = 1):
        """``attempts`` > 1 only for requests that may run twice: a new client's first answer is now and then lost
        (a new link in a process that had links before)."""

        for _ in range(attempts):
            future = client.call_async(request)
            deadline = time.monotonic() + timeout_s
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.001)
            if future.done():
                break
        else:
            raise RuntimeError("%s did not answer within %d x %.0f s" % (client.srv_name, attempts, timeout_s))
        response = future.result()
        if check and not response.success:
            raise RuntimeError("%s: %s" % (client.srv_name, response.message))
        return response

    def _set_pause(self, paused: bool) -> None:
        self._call(self._pause, self._SetPause.Request(paused=paused), attempts=3)

    # -- state ---------------------------------------------------------------
    def joint_state(self) -> Tuple[List[float], List[float], float]:
        with self._changed:
            return self._state

    def sim_time_s(self) -> float:
        with self._changed:
            return max(self._sim_time_s, Ros2Link._counted_s)

    def base_pose(self) -> None:
        return None

    def put_base_back(self) -> bool:
        return False                          # fixed feet: nothing moved

    def frames(self) -> int:
        return 0

    # -- acting --------------------------------------------------------------
    def send(self, positions: Sequence[float], time_s: float) -> None:
        """Targets for the four arm joints, to be reached ``time_s`` (sim time) after the controller receives them."""

        from rclpy.duration import Duration

        message = self._JointTrajectory(joint_names=list(self.joints))      # stamp 0: start on receipt
        message.points.append(self._JointTrajectoryPoint(positions=[float(v) for v in positions],
                                                         time_from_start=Duration(seconds=float(time_s)).to_msg()))
        self._command.publish(message)

    def step(self, iterations: int) -> float:
        """Let ``iterations`` physics steps pass; returns the sim time afterwards.

        A tick needs at least 20 ms (>= 41 physics steps): joint states come at 50 Hz.  If none stamped later than
        the end - 20.5 ms comes within 1 s (wall), the newest one stays and ``missed_states`` counts it."""

        end = self.sim_time_s() + int(iterations) * PHYSICS_STEP_S
        newest = end - JOINT_STATE_PERIOD_S - PHYSICS_STEP_S
        self._call(self._step, self._StepSimulation.Request(steps=int(iterations)), timeout_s=10.0)
        Ros2Link._counted_s = end
        with self._changed:                   # the newest joint state of the tick may still be on its way
            if not self._changed.wait_for(lambda: self._state[2] > newest, timeout=1.0):
                self.missed_states += 1       # now and then one is lost: go on with the newest there is (stale_s shows it)
            stamp = self._state[2]
        now = self.sim_time_s()
        self.stale_s = now - stamp
        self.stale_max_s = max(self.stale_max_s, self.stale_s)
        return now

    def close(self) -> None:
        """Let the simulation run again and leave."""

        if self._node is None:
            return
        try:
            paused_at = self._sim_time_s
            self._set_pause(False)
            with self._changed:
                if not self._changed.wait_for(lambda: self._sim_time_s > paused_at, timeout=1.0):
                    raise RuntimeError("/clock does not move 1 s after the unpause")
        finally:
            self._shutdown()

    def _shutdown(self) -> None:
        self._executor.shutdown(timeout_sec=5.0)
        self._spinner.join(timeout=5.0)
        if self._spinner.is_alive():
            raise RuntimeError("the executor thread of %s did not stop within 5 s; its node is left alive" % self._node.get_name())
        self._node.destroy_node()
        self._node = None


__all__ = ["ARM_JOINTS", "PAUSE_SERVICE", "PHYSICS_STEP_S", "Ros2Link", "STEP_SERVICE"]

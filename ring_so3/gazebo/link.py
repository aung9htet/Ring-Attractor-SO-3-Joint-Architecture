"""The connection to a running Gazebo with the TIAGo: joint states in, arm targets out, physics in lock step.

Taken from the SO-2 repo (``cosim/gazebo_ros_engine.py``: ``RospyTransport``; ``cosim/stepping.py``:
``ClockWaitStepper``) and cut down to what the IK loop needs:

* physics stays paused.  :meth:`RosLink.step` unpauses, waits until ``/clock`` has advanced by the
  tick, and pauses again.  Targets follow an absolute schedule, so the small overshoot of one
  step (one physics iteration at most at real-time factor 1) is not added to the next;
* nothing here calls ``rospy.sleep``: with ``/use_sim_time`` it would wait on a paused clock for ever;
* the mobile base is not anchored, so :meth:`put_base_back` returns it to the origin;
* :meth:`jpeg` gives the latest frame of the world's camera (the camera needs a GPU container).
"""

from __future__ import annotations

import math
import threading
import time
from typing import List, Optional, Sequence, Tuple

ARM_JOINTS: Tuple[str, ...] = tuple("arm_%d_joint" % k for k in range(1, 8))
ARM_COMMAND_TOPIC = "/arm_controller/command"
JOINT_STATES_TOPIC = "/joint_states"
CAMERA_TOPIC = "/ring_ik_camera/image_raw"
MODEL_NAME = "tiago"


class StepTimeout(RuntimeError):
    pass


class RosLink:
    def __init__(self, node_name: str = "ring_ik", physics_step_s: float = 0.001, step_timeout_s: float = 20.0,
                 camera_topic: Optional[str] = CAMERA_TOPIC) -> None:
        import rospy
        from gazebo_msgs.msg import ModelState
        from gazebo_msgs.srv import GetModelState, SetModelState
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, JointState
        from std_srvs.srv import Empty
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        self._rospy, self._JointTrajectory, self._JointTrajectoryPoint, self._ModelState = rospy, JointTrajectory, JointTrajectoryPoint, ModelState
        self.physics_step_s = float(physics_step_s)
        self.step_timeout_s = float(step_timeout_s)
        self._lock = threading.Lock()
        self._clock = threading.Condition()
        self._sim_time_s = 0.0
        self._state: Optional[Tuple[List[float], List[float], float]] = None
        self._frame = None
        self._frame_count = 0
        self._schedule_s: Optional[float] = None
        self.paused = False
        self.overshoot_s = 0.0
        rospy.init_node(node_name, anonymous=True, disable_signals=True)
        self._command = rospy.Publisher(ARM_COMMAND_TOPIC, JointTrajectory, queue_size=1)
        rospy.Subscriber(JOINT_STATES_TOPIC, JointState, self._on_joints, queue_size=1)
        rospy.Subscriber("/clock", Clock, self._on_clock, queue_size=1)
        if camera_topic:
            rospy.Subscriber(camera_topic, Image, self._on_image, queue_size=1, buff_size=2 ** 24)
        for name in ("/gazebo/pause_physics", "/gazebo/unpause_physics", "/gazebo/get_model_state", "/gazebo/set_model_state"):
            rospy.wait_for_service(name, timeout=30.0)
        self._pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self._unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self._get_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self._set_model = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self._unpause()                           # joint states come only while physics runs (an earlier script may have left it paused)
        deadline = time.monotonic() + 30.0
        while self._state is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if self._state is None:
            raise RuntimeError("no joint states from Gazebo within 30 s")
        self.pause()

    # -- callbacks -----------------------------------------------------------
    def _on_joints(self, message) -> None:
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity)) if message.velocity else {}
        if any(name not in positions for name in ARM_JOINTS):
            return
        with self._lock:
            self._state = ([float(positions[name]) for name in ARM_JOINTS], [float(velocities.get(name, 0.0)) for name in ARM_JOINTS],
                           message.header.stamp.to_sec())

    def _on_clock(self, message) -> None:
        with self._clock:
            self._sim_time_s = message.clock.to_sec()
            self._clock.notify_all()

    def _on_image(self, message) -> None:
        with self._lock:
            self._frame = message
            self._frame_count += 1

    # -- state ---------------------------------------------------------------
    def joint_state(self) -> Tuple[List[float], List[float], float]:
        with self._lock:
            return self._state

    def sim_time_s(self) -> float:
        with self._clock:
            return self._sim_time_s

    def base_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            state = self._get_model(model_name=MODEL_NAME, relative_entity_name="world")
        except Exception:  # noqa: BLE001 - no such model: unknown
            return None
        if not state.success:
            return None
        o = state.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        return float(state.pose.position.x), float(state.pose.position.y), float(yaw)

    def put_base_back(self) -> bool:
        try:
            current = self._get_model(model_name=MODEL_NAME, relative_entity_name="world")
            if not current.success:
                return False
            state = self._ModelState()
            state.model_name, state.reference_frame = MODEL_NAME, "world"
            state.pose.position.z = current.pose.position.z
            state.pose.orientation.w = 1.0
            return bool(self._set_model(model_state=state).success)
        except Exception:  # noqa: BLE001
            return False

    def jpeg(self, quality: int = 70) -> Optional[bytes]:
        """The latest camera frame as JPEG, or None when the camera gives no picture."""

        with self._lock:
            frame = self._frame
        if frame is None:
            return None
        import cv2
        import numpy as np

        channels = 3 if frame.encoding in ("rgb8", "bgr8") else 1
        image = np.frombuffer(frame.data, dtype=np.uint8).reshape(frame.height, frame.width, channels)
        if frame.encoding == "rgb8":
            image = image[:, :, ::-1]
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        return encoded.tobytes() if ok else None

    def frames(self) -> int:
        return self._frame_count

    # -- acting --------------------------------------------------------------
    def pause(self) -> None:
        self._pause()
        self.paused = True

    def unpause(self) -> None:
        self._unpause()
        self.paused = False
        self._schedule_s = None

    def send(self, positions: Sequence[float], time_s: float) -> None:
        """Targets for the seven arm joints, to be reached ``time_s`` from now (sim time)."""

        message = self._JointTrajectory()
        message.header.stamp = self._rospy.Time.from_sec(self.sim_time_s())
        message.joint_names = list(ARM_JOINTS)
        point = self._JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start = self._rospy.Duration.from_sec(float(time_s))
        message.points.append(point)
        self._command.publish(message)

    def step(self, iterations: int) -> float:
        """Let ``iterations`` physics steps pass; returns the sim time afterwards."""

        before = self.sim_time_s()
        span = int(iterations) * self.physics_step_s
        target = before + span
        if self._schedule_s is not None and self._schedule_s + span - before >= 0.5 * span:
            target = self._schedule_s + span
        self._unpause()
        deadline = time.monotonic() + self.step_timeout_s
        try:
            with self._clock:
                while self._sim_time_s < target - 1e-9:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        raise StepTimeout("Gazebo clock at %.3f s, target %.3f s, after %.0f s" % (self._sim_time_s, target, self.step_timeout_s))
                    self._clock.wait(min(remaining, 0.05))
        finally:
            self._pause()
        self.paused = True
        self._schedule_s = target
        after = self.sim_time_s()
        self.overshoot_s = after - target
        time.sleep(0.004)                      # the joint states of the last physics step are still on their way
        return after


__all__ = ["ARM_JOINTS", "CAMERA_TOPIC", "RosLink", "StepTimeout"]

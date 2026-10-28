"""The ROS 2 link and KangarooRobot against the live MuJoCo simulation (docker/run.bash kangaroo exec).

The live tests are skipped unless rclpy imports and the step_simulation service is there within 5 s;
test_no_sim_fails_fast is the opposite and runs only where rclpy imports and there is no simulation.
"""

import sys
import time
import unittest

import numpy as np

from ring_so3.chains import get_chain

try:
    import rclpy
    from rclpy.node import Node
except ImportError:                                           # not a ROS 2 environment
    rclpy = None

STEP_SERVICE = "/mujoco_ros2_control_node/step_simulation"
HOME = {"left": [0.239, 1.321, 1.57, 0.8], "right": [-0.239, 1.321, -1.57, 0.8]}


def sim_is_up() -> bool:
    if rclpy is None:
        return False
    if not rclpy.ok():
        rclpy.init()
    node = Node("kangaroo_test_probe")
    try:
        names = lambda: [n for n, _ in node.get_service_names_and_types()]
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if STEP_SERVICE in names():
                return True
            time.sleep(0.2)
        return False
    finally:
        node.destroy_node()
        rclpy.shutdown()


SIM = sim_is_up()


def rotation_from_quaternion(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def angle_between(a, b):
    return float(np.arccos(np.clip((np.trace(a.T @ b) - 1) / 2, -1.0, 1.0)))


@unittest.skipUnless(SIM, "needs rclpy and the Kangaroo simulation (docker/run.bash kangaroo exec)")
class KangarooSimTests(unittest.TestCase):
    side = "left"

    def setUp(self):
        from ring_so3.kangaroo.link import Ros2Link
        from ring_so3.robots import KangarooRobot
        self.chain = get_chain("kangaroo_%s4" % self.side)
        self.link = Ros2Link(self.side)
        self.addCleanup(self.link.close)
        self.robot = KangarooRobot(self.chain, self.link)

    def test_follow_advances_the_clock_by_one_tick(self):
        for _ in range(2):                                        # the first follow after construction, then a later one
            before, q0 = self.link.sim_time_s(), self.robot.q.copy()
            target = q0.copy()
            target[0] += 0.2
            self.robot.follow(target, 0.8, 0.05)
            self.assertAlmostEqual(self.link.sim_time_s() - before, 0.05, delta=1e-6)
            self.assertGreater(self.robot.q[0], q0[0])            # moved toward the target
            self.assertLess(self.robot.q[0], target[0] + 1e-9)
            # the measured joint-state stamp: within one broadcaster period (0.02 s) of the new sim time
            self.assertLess(before + 0.05 - 0.0205, self.robot.sim_time_s)
            self.assertLessEqual(self.robot.sim_time_s, before + 0.05 + 1e-6)

    def test_tool_frame_matches_the_robot_tf(self):
        import tf2_ros
        from rclpy.time import Time
        buffer = tf2_ros.Buffer()
        node = Node("kangaroo_test_tf", parameter_overrides=[rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        listener = tf2_ros.TransformListener(buffer, node, spin_thread=True)
        # LIFO: executor shutdown, then thread join, then node destroy (the listener thread is non-daemon)
        self.addCleanup(node.destroy_node)
        self.addCleanup(lambda: listener.dedicated_listener_thread.join(timeout=5))
        self.addCleanup(listener.executor.shutdown)
        home = np.array(HOME[self.side])
        poses = [home, home + [0.2, -0.2, 0.2, -0.2], home + [-0.2, -0.2, -0.2, 0.2]]
        for pose in poses:
            with self.subTest(pose=list(pose)):
                self.robot.reset(pose)
                for _ in range(5):                                # let the robot_state_publisher catch up
                    self.robot.follow(self.robot.q, 0.2, 0.05)
                time.sleep(0.5)                                   # wall time: only for message delivery
                tf = buffer.lookup_transform("torso_link", "arm_%s_tool_link" % self.side, Time()).transform
                t = tf.rotation
                tf_rotation = rotation_from_quaternion(t.x, t.y, t.z, t.w)
                self.assertLess(angle_between(self.chain.rotation(self.robot.q), tf_rotation), 1e-3)
                tf_position = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
                difference = float(np.linalg.norm(self.chain.frames(self.robot.q)[-1][0] - tf_position))
                print("tool translation difference to TF: %.3e m" % difference, file=sys.stderr)
                self.assertLess(difference, 1e-3)

    def test_close_unpauses(self):
        self.link.close()
        from rosgraph_msgs.msg import Clock
        node = Node("kangaroo_test_clock")
        self.addCleanup(node.destroy_node)
        stamps = []
        node.create_subscription(Clock, "/clock", lambda m: stamps.append(m.clock.sec + m.clock.nanosec * 1e-9), 10)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not (len(stamps) > 1 and stamps[-1] > stamps[0]):
            rclpy.spin_once(node, timeout_sec=0.1)
        self.assertTrue(len(stamps) > 1 and stamps[-1] > stamps[0], "/clock does not advance after close()")


@unittest.skipUnless(rclpy is not None and not SIM, "runs only with rclpy and no simulation (kangaroo run)")
class NoSimTests(unittest.TestCase):
    def test_no_sim_fails_fast(self):
        from ring_so3.kangaroo.link import Ros2Link
        start = time.monotonic()
        with self.assertRaises(RuntimeError):
            Ros2Link("left", timeout_s=5)
        self.assertLess(time.monotonic() - start, 15.0)


if __name__ == "__main__":
    unittest.main()

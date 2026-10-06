"""The robot interface: the light robot, and the Gazebo robot on a stand-in for ROS."""

import unittest

import numpy as np

from ring_so3.chains import get_chain
from ring_so3.ik.design import IkDesign
from ring_so3.ik.session import IkSession
from ring_so3.robots import GazeboRobot, LightRobot
from ring_so3.sim.server import StubIkNetwork


class FakeLink:
    """Seven joints that move toward what they were sent as a trajectory controller does; no ROS."""

    def __init__(self, q):
        self.q = np.array(q, dtype=float)
        self.goal, self.arrive_s = self.q.copy(), 0.05
        self.t, self.overshoot_s, self.sent, self.base_resets = 0.0, 0.0, 0, 0

    def joint_state(self):
        return list(self.q), [0.0] * 7, self.t

    def send(self, positions, time_s):
        self.goal, self.arrive_s, self.sent = np.array(positions, dtype=float), float(time_s), self.sent + 1

    def step(self, iterations):
        dt = iterations * 0.001
        self.q = self.q + (self.goal - self.q) * min(dt / self.arrive_s, 1.0)
        self.arrive_s = max(self.arrive_s - dt, 1e-6)
        self.t += dt
        return self.t

    def put_base_back(self):
        self.base_resets += 1
        return True

    def base_pose(self):
        return (0.0, 0.0, 0.0)

    def frames(self):
        return 0


class LightRobotTests(unittest.TestCase):
    def test_follow_is_a_proportional_step(self):
        chain = get_chain("generic3")
        robot = LightRobot(chain, [0.0, 0.0, 0.0])
        robot.follow([0.2, -0.1, np.nan], 0.8, 0.05)
        np.testing.assert_allclose(robot.q, [0.2 * 0.05 / 0.8, -0.1 * 0.05 / 0.8, 0.0])

    def test_limits_wrap_and_push(self):
        tiago = get_chain("tiago7")
        robot = LightRobot(tiago)
        robot.follow(tiago.limits()[:, 1] + 1.0, 0.05, 0.05)
        np.testing.assert_allclose(robot.q, tiago.limits()[:, 1])
        generic = LightRobot(get_chain("generic3"), [3.1, 0.0, 0.0])
        generic.follow([-3.1, 0.0, 0.0], 0.05, 0.05)                 # the short way round
        self.assertAlmostEqual(generic.q[0], -3.1, places=9)
        generic.displace(1, 0.3)
        self.assertAlmostEqual(generic.q[1], 0.3, places=9)


class GazeboRobotTests(unittest.TestCase):
    def setUp(self):
        self.chain = get_chain("tiago7")
        self.link = FakeLink(self.chain.limits().mean(axis=1))
        self.robot = GazeboRobot(self.chain, self.link)

    def test_one_follow_is_one_tick_of_physics(self):
        before = self.link.t
        target = self.robot.q + 0.1
        self.robot.follow(target, 0.8, 0.05)
        self.assertAlmostEqual(self.link.t - before, 0.05, places=9)
        self.assertEqual(self.link.sent, 1)
        np.testing.assert_allclose(self.robot.lag, target - self.robot.q)
        self.assertTrue(np.all(self.robot.q > self.link.q - 1e-12))

    def test_targets_stay_inside_the_limits_and_reset_moves_there(self):
        self.robot.follow(self.chain.limits()[:, 1] + 1.0, 0.05, 0.05)
        np.testing.assert_allclose(self.link.goal, self.chain.limits()[:, 1])
        goal = self.chain.limits().mean(axis=1) + 0.3
        self.robot.reset(goal)
        np.testing.assert_allclose(self.robot.q, goal, atol=0.01)
        self.assertGreaterEqual(self.link.base_resets, 1)

    def test_only_the_tiago_chains(self):
        with self.assertRaises(ValueError):
            GazeboRobot(get_chain("generic7"), self.link)

    def test_the_closed_loop_runs_on_it(self):
        network = StubIkNetwork(self.chain, IkDesign(), noise_deg=0.1)
        session = IkSession(network, robot=self.robot)
        session.warm_up(5)
        from ring_so3.kinematics import lift_pitch_yaw
        goal = lift_pitch_yaw(self.chain.rotation(self.chain.limits().mean(axis=1) + 0.25))
        trial = session.reach(goal)
        self.assertLess(trial.final_error_deg, 5.0)
        self.assertIsNotNone(trial.lag)


if __name__ == "__main__":
    unittest.main()

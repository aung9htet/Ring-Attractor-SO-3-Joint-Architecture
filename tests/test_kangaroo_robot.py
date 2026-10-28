"""The Kangaroo robot on a stand-in for the ROS 2 link (no ROS, no simulator)."""

import unittest

import numpy as np

from ring_so3.chains import get_chain
from ring_so3.ik.design import IkDesign
from ring_so3.ik.session import IkSession
from ring_so3.kinematics import lift_pitch_yaw
from ring_so3.robots import KangarooRobot
from ring_so3.sim.server import StubIkNetwork


class FakeKangarooLink:
    """Four joints that move toward what they were sent, 0.0005 s per physics step; no ROS."""

    PHYSICS_STEP_S = 0.0005

    def __init__(self, side, q):
        self.side, self.stale_max_s = side, 0.0
        self.q = np.array(q, dtype=float)
        self.goal, self.arrive_s = self.q.copy(), 0.05
        self.t, self.sent, self.steps = 0.0, 0, 0

    def joint_state(self):
        return list(self.q), [0.0] * 4, self.t

    def sim_time_s(self):
        return self.t

    def send(self, positions, time_s):
        self.goal, self.arrive_s, self.sent = np.array(positions, dtype=float), float(time_s), self.sent + 1

    def step(self, iterations):
        dt = iterations * self.PHYSICS_STEP_S
        self.q = self.q + (self.goal - self.q) * min(dt / self.arrive_s, 1.0)
        self.arrive_s = max(self.arrive_s - dt, 1e-6)
        self.t += dt
        self.steps += iterations
        return self.t

    def put_base_back(self):
        return False

    def base_pose(self):
        return None

    def frames(self):
        return 0


class KangarooRobotTests(unittest.TestCase):
    def setUp(self):
        self.chain = get_chain("kangaroo_left4")
        self.link = FakeKangarooLink("left", self.chain.limits().mean(axis=1))
        self.robot = KangarooRobot(self.chain, self.link)

    def test_one_follow_is_one_tick_of_physics(self):
        before = self.link.t
        self.robot.follow(self.robot.q + 0.1, 0.8, 0.05)
        self.assertEqual(self.link.steps, 100)
        self.assertAlmostEqual(self.link.t - before, 0.05, places=9)
        self.assertEqual(self.link.sent, 1)

    def test_targets_clip_to_the_measured_shoulder_limit(self):
        target = list(self.robot.q)
        target[1] = 2.0
        self.robot.follow(target, 0.8, 0.05)
        self.assertAlmostEqual(self.link.goal[1], 1.4229904547193268, places=12)
        self.assertLessEqual(self.link.goal[1], self.chain.limits()[1, 1])

    def test_wrong_side_is_refused(self):
        right_link = FakeKangarooLink("right", get_chain("kangaroo_right4").limits().mean(axis=1))
        with self.assertRaises(ValueError):
            KangarooRobot(get_chain("kangaroo_left4"), right_link)
        with self.assertRaises(ValueError):
            KangarooRobot(get_chain("tiago7"), self.link)

    def test_the_closed_loop_runs_on_it(self):
        network = StubIkNetwork(self.chain, IkDesign(), noise_deg=0.1)
        session = IkSession(network, robot=self.robot)
        session.warm_up(5)
        goal = lift_pitch_yaw(self.chain.rotation(self.chain.limits().mean(axis=1) + 0.25))
        trial = session.reach(goal)
        self.assertLess(trial.final_error_deg, 5.0)


if __name__ == "__main__":
    unittest.main()

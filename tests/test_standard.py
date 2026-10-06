"""The standard methods the ring IK is compared with."""

import unittest

import numpy as np

from ring_so3.chains import get_chain
from ring_so3.ik import rules
from ring_so3.ik.design import IkDesign
from ring_so3.ik.pairs import DEVELOPMENT_SEED, pairs
from ring_so3.kinematics import rot, rotation_angle
from ring_so3.standard import ik, integration


class RuleTests(unittest.TestCase):
    def test_best_angle_recovers_a_turn_about_the_axis(self):
        axis = np.array([0.0, 0.0, 1.0])
        for angle in (-2.0, -0.3, 0.7, 3.0):
            self.assertAlmostEqual(ik.best_angle(axis, np.eye(3), rot("z", angle)), angle, places=9)

    def test_small_turn_all_methods_make_the_turn_that_is_asked(self):
        chain = get_chain("generic7")
        rng = np.random.default_rng(1)
        caps = np.full(7, 10.0)
        for _ in range(10):
            q = chain.random_pose(rng)
            axes = rules.joint_axes(chain, q)
            e = rng.normal(size=3) * 0.01
            rotation = chain.rotation(q)
            goal = rules.small_rotation(e) @ rotation
            shares = ik.dls(axes, e, caps, None, rotation, goal)
            self.assertLess(np.linalg.norm(axes.T @ shares - e), 2e-3)
            swept = chain.rotation(q + ik.ccd(axes, e, caps, None, rotation, goal))          # one sweep of coordinate descent
            self.assertLess(rotation_angle(swept, goal), 0.5 * np.linalg.norm(e))
            self.assertGreater(float((axes.T @ ik.transpose(axes, e, caps)) @ e), 0.0)      # the transpose at least turns the right way

    def test_dls_is_the_smallest_motion(self):
        chain = get_chain("generic7")
        q = chain.random_pose(np.random.default_rng(2))
        axes, e, caps = rules.joint_axes(chain, q), np.array([0.01, -0.02, 0.015]), np.full(7, 10.0)
        cascade, _ = rules.cascade(axes, e)
        self.assertLessEqual(np.linalg.norm(ik.dls(axes, e, caps, damping=1e-6)), np.linalg.norm(cascade) + 1e-12)

    def test_caps_and_limits_are_kept(self):
        chain = get_chain("tiago7")
        design = IkDesign()
        caps = rules.share_caps(chain, design)
        q = chain.limits()[:, 1].copy()                       # every joint at its upper limit
        room = rules.limit_room(chain, q, design)
        axes, rotation = rules.joint_axes(chain, q), chain.rotation(q)
        for name, rule in ik.RULES.items():
            shares = rule(axes, np.array([0.4, 0.3, -0.5]), caps, room, rotation, rules.small_rotation([0.4, 0.3, -0.5]) @ rotation)
            self.assertTrue(np.all(np.abs(shares) <= caps + 1e-12), name)
            self.assertTrue(np.all(shares <= 1e-12), name)    # nothing moves beyond the limit

    def test_every_method_reaches_goals_on_seven_joints(self):
        chain = get_chain("generic7")
        for name, rule in ik.RULES.items():
            reached = 0
            for pair in pairs(chain, 6, DEVELOPMENT_SEED):
                trace = rules.simulate(chain, pair.start, pair.goal, IkDesign(kappa=ik.GAINS[name]), delay_s=0.0, rule=rule)
                reached += trace.final_error_deg <= 5.0
            self.assertGreaterEqual(reached, 5, name)

    def test_solver_finds_the_orientation(self):
        chain = get_chain("tiago7")
        for pair in pairs(chain, 4, DEVELOPMENT_SEED):
            goal = chain.rotation(pair.goal_pose)
            q, error, evaluations = ik.solve(chain, pair.goal_pose + 0.2, goal)
            self.assertLess(error, 0.5)
            self.assertTrue(np.all(q >= chain.limits()[:, 0] - 1e-9) and np.all(q <= chain.limits()[:, 1] + 1e-9))


class IntegrationTests(unittest.TestCase):
    def test_without_noise_the_integrated_orientation_stays_close(self):
        out = integration.run(get_chain("tiago7"), seconds=20.0, noise=0.0, bias=0.0)
        self.assertLess(out["error_final_deg"], 1.0)             # only the first-order step of the integration is left

    def test_a_bias_makes_it_drift_and_a_correction_bounds_it(self):
        chain = get_chain("tiago7")
        free = integration.run(chain, seconds=40.0, noise=0.02, bias=0.01)
        held = integration.run(chain, seconds=40.0, noise=0.02, bias=0.01, correct_s=2.0)
        self.assertGreater(free["error_final_deg"], 5.0)
        self.assertLess(held["error_median_deg"], free["error_median_deg"])


if __name__ == "__main__":
    unittest.main()

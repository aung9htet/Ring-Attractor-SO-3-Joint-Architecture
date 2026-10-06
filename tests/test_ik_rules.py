"""Test 1 of the inverse kinematics: the exact maths of the comparison, the cascade and the closed loop."""

import unittest

import numpy as np

from ring_so3.chains import MODEL_NAMES, get_chain
from ring_so3.ik import rules
from ring_so3.ik.design import IkDesign
from ring_so3.kinematics import from_lift_pitch_yaw, lift_pitch_yaw, rotation_angle


def random_lpy(rng):
    return np.array([rng.uniform(-np.pi, np.pi), rng.uniform(-1.4, 1.4), rng.uniform(-np.pi, np.pi)])


class ComparisonTests(unittest.TestCase):
    def test_goal_chain_gives_the_goal_matrix(self):
        rng = np.random.default_rng(1)
        for _ in range(20):
            lift, pitch, yaw = random_lpy(rng)
            np.testing.assert_allclose(rules.GOAL_CHAIN.rotation([yaw, pitch, lift]), from_lift_pitch_yaw(lift, pitch, yaw), atol=1e-12)
            np.testing.assert_allclose(lift_pitch_yaw(rules.goal_matrix(lift, pitch, yaw)), [lift, pitch, yaw], atol=1e-9)

    def test_matrix_comparison_is_sine_of_the_angle_times_the_axis(self):
        rng = np.random.default_rng(2)
        for _ in range(50):
            current = rules.small_rotation(rng.normal(size=3))
            axis = rng.normal(size=3)
            axis /= np.linalg.norm(axis)
            angle = rng.uniform(0.05, 3.0)
            goal = rules.small_rotation(angle * axis) @ current          # turn the hand by `angle` about `axis` (base frame)
            np.testing.assert_allclose(rules.turn_from_matrices(current, goal), np.sin(angle) * axis, atol=1e-9)
            self.assertAlmostEqual(rotation_angle(current, goal), angle, places=9)

    def test_angle_differences_are_the_sines_the_formulas_say(self):
        rng = np.random.default_rng(3)
        for _ in range(50):
            now, goal = random_lpy(rng), random_lpy(rng)
            r = from_lift_pitch_yaw(*now)
            expected = [np.cos(now[1]) * np.sin(goal[0] - now[0]), np.sin(goal[1] - now[1]), np.cos(now[1]) * np.sin(goal[2] - now[2])]
            np.testing.assert_allclose(rules.angle_differences(r, goal), expected, atol=1e-9)

    def test_both_comparisons_agree_for_a_small_turn(self):
        rng = np.random.default_rng(4)
        for _ in range(30):
            now = random_lpy(rng)
            goal = now + rng.normal(0.0, 1e-4, 3)
            r = from_lift_pitch_yaw(*now)
            by_matrix, by_angles = rules.turn(r, goal, "matrix"), rules.turn(r, goal, "angles")
            np.testing.assert_allclose(by_angles, np.cos(now[1]) * by_matrix, atol=2e-7)     # the angle form fades with cos(pitch)

    def test_no_turn_is_asked_for_at_the_goal(self):
        r = from_lift_pitch_yaw(0.3, -0.4, 1.2)
        for comparison in ("matrix", "angles"):
            np.testing.assert_allclose(rules.turn(r, [0.3, -0.4, 1.2], comparison), 0.0, atol=1e-12)


class CascadeTests(unittest.TestCase):
    def test_shares_add_up_to_the_turn_less_what_is_left(self):
        rng = np.random.default_rng(5)
        for name in MODEL_NAMES:
            chain = get_chain(name)
            for _ in range(10):
                axes = rules.joint_axes(chain, chain.random_pose(rng))
                e = rng.normal(0.0, 0.3, 3)
                caps = rng.uniform(0.05, 0.4, chain.n)
                shares, rests = rules.cascade(axes, e, caps)
                np.testing.assert_allclose(axes.T @ shares, e - rests[-1], atol=1e-9)
                np.testing.assert_allclose(rests[0], e, atol=0)
                self.assertTrue(np.all(np.abs(shares) <= caps + 1e-12))
                self.assertLessEqual(np.linalg.norm(rests[-1]), np.linalg.norm(e) + 1e-12)

    def test_axes_are_unit_vectors_and_the_first_one_is_fixed(self):
        rng = np.random.default_rng(6)
        for name in MODEL_NAMES:
            chain = get_chain(name)
            first = rules.joint_axes(chain, chain.random_pose(rng))
            second = rules.joint_axes(chain, chain.random_pose(rng))
            np.testing.assert_allclose(np.linalg.norm(first, axis=1), 1.0, atol=1e-12)
            np.testing.assert_allclose(first[0], second[0], atol=1e-12)

    def test_three_axes_at_right_angles_leave_nothing(self):
        shares, rests = rules.cascade(np.eye(3), [0.2, -0.1, 0.3])
        np.testing.assert_allclose(shares, [0.2, -0.1, 0.3])
        np.testing.assert_allclose(rests[-1], 0.0, atol=1e-12)

    def test_without_the_cascade_every_joint_gets_the_whole_turn(self):
        axes = np.array([[1.0, 0, 0], [1.0, 0, 0], [0, 1.0, 0]])
        shares, rests = rules.cascade(axes, [0.2, 0.1, 0.0], pass_on=False)
        np.testing.assert_allclose(shares, [0.2, 0.2, 0.1])
        with_cascade, _ = rules.cascade(axes, [0.2, 0.1, 0.0])
        np.testing.assert_allclose(with_cascade, [0.2, 0.0, 0.1])            # the second joint finds nothing left about its axis

    def test_a_joint_at_its_limit_passes_its_share_on(self):
        axes = np.array([[0, 0, 1.0], [0, 0, 1.0]])
        room = np.array([[1.0, 0.0], [1.0, 1.0]])                              # joint 1 has no room upward
        shares, rests = rules.cascade(axes, [0, 0, 0.2], room=room)
        np.testing.assert_allclose(shares, [0.0, 0.2])

    def test_a_small_step_along_the_shares_brings_the_hand_closer(self):
        rng = np.random.default_rng(7)
        for name in ("generic3", "generic7", "tiago7"):
            chain = get_chain(name)
            for comparison in ("matrix", "angles"):
                closer = 0
                for _ in range(40):
                    q = chain.limits().mean(axis=1) + rng.uniform(-0.3, 0.3, chain.n)
                    goal = np.array(lift_pitch_yaw(chain.rotation(q + rng.uniform(-0.15, 0.15, chain.n))))
                    if abs(goal[1]) > 1.2:
                        continue
                    e = rules.turn(chain.rotation(q), goal, comparison)
                    shares, _ = rules.cascade(rules.joint_axes(chain, q), e)
                    before = rotation_angle(chain.rotation(q), rules.goal_matrix(*goal))
                    after = rotation_angle(chain.rotation(q + 0.05 * shares), rules.goal_matrix(*goal))
                    closer += after < before
                self.assertGreaterEqual(closer, 36, (name, comparison))


class ClosedLoopTests(unittest.TestCase):
    def test_caps_and_room(self):
        design = IkDesign()
        generic = rules.share_caps(get_chain("generic3"), design)
        np.testing.assert_allclose(generic, design.lead_indices * 2 * np.pi / 200)
        chain = get_chain("tiago7")
        limits = chain.limits()
        room = rules.limit_room(chain, limits[:, 1], design)
        np.testing.assert_allclose(room[:, 1], 0.0, atol=1e-12)                # at the upper limits: no room upward
        np.testing.assert_allclose(room[:, 0], 1.0)
        np.testing.assert_allclose(rules.limit_room(chain, limits.mean(axis=1), design), 1.0)

    def test_the_exact_rule_reaches_goals_with_seven_joints(self):
        chain = get_chain("generic7")
        rng = np.random.default_rng(8)
        reached = 0
        for _ in range(12):
            q0, goal = chain.random_pose(rng), lift_pitch_yaw(chain.rotation(chain.random_pose(rng)))
            trace = rules.simulate(chain, q0, goal, IkDesign(), rng=rng)
            reached += trace.final_error_deg <= 5.0
            self.assertLess(trace.path_deg, 4.0 * trace.start_error_deg + 30.0)
        self.assertGreaterEqual(reached, 11)

    def test_a_disturbed_joint_is_corrected_and_a_new_goal_is_followed(self):
        chain = get_chain("generic7")
        rng = np.random.default_rng(9)
        q0 = chain.random_pose(rng)
        first, second = [lift_pitch_yaw(chain.rotation(chain.random_pose(rng))) for _ in range(2)]
        pushed = rules.simulate(chain, q0, first, IkDesign(), push=(1.0, 2, 0.3), rng=rng)
        self.assertLessEqual(pushed.final_error_deg, 5.0)
        changed = rules.simulate(chain, q0, first, IkDesign(), goal_change=(1.0, second), rng=rng)
        self.assertLessEqual(changed.final_error_deg, 5.0)
        weak = rules.simulate(chain, q0, first, IkDesign(), motor_scale=0.7, rng=rng)
        self.assertLessEqual(weak.final_error_deg, 5.0)

    def test_travel_takes_the_short_way_round(self):
        now = np.array([3.0, 0.0, -3.0])
        moved = rules.travel(now, [-3.0, 0.5, 3.0], 0.1)
        np.testing.assert_allclose(moved, [3.1, 0.1, -3.1], atol=1e-12)


if __name__ == "__main__":
    unittest.main()

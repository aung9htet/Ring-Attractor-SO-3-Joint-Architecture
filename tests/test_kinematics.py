"""Test 1 of docs/tests.md: the maths.  No neurons.

* the paper's readout ``vec(R) = T Φ`` holds exactly for every chain;
* computing joint by joint (and all at once) gives the same matrix as the direct product;
* the number of product features and of pairwise products is what the docs quote;
* the TIAGo table equals the robot's URDF (skipped where the robot description is not installed).
"""

import shutil
import subprocess
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from ring_so3 import kinematics as kin
from ring_so3.chains import CHAINS, MODEL_NAMES, TIAGO_ARM, TIAGO_BASE_LINK, TIAGO_TOOL_LINK, get_chain
from ring_so3.wiring import chain_wiring, flat_wiring


class RotationTests(unittest.TestCase):
    def test_axis_rotations_follow_the_right_hand_rule(self):
        np.testing.assert_allclose(kin.rot("z", np.pi / 2) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
        np.testing.assert_allclose(kin.rot("x", np.pi / 2) @ [0, 1, 0], [0, 0, 1], atol=1e-12)
        np.testing.assert_allclose(kin.rot("y", np.pi / 2) @ [0, 0, 1], [1, 0, 0], atol=1e-12)

    def test_a_joint_mixes_the_two_other_columns_as_the_wiring_assumes(self):
        rng = np.random.default_rng(0)
        m = rng.normal(size=(3, 3))
        for axis in kin.AXES:
            angle = 0.7
            a, (u, v) = kin.AXES.index(axis), kin.other_axes(axis)
            turned = m @ kin.rot(axis, angle)
            np.testing.assert_allclose(turned[:, a], m[:, a], atol=1e-12)
            np.testing.assert_allclose(turned[:, u], m[:, u] * np.cos(angle) + m[:, v] * np.sin(angle), atol=1e-12)
            np.testing.assert_allclose(turned[:, v], -m[:, u] * np.sin(angle) + m[:, v] * np.cos(angle), atol=1e-12)

    def test_lift_pitch_yaw_round_trip_and_convention(self):
        rng = np.random.default_rng(1)
        for _ in range(50):
            lift, yaw = rng.uniform(-np.pi, np.pi, 2)
            pitch = rng.uniform(-1.4, 1.4)
            r = kin.from_lift_pitch_yaw(lift, pitch, yaw)
            np.testing.assert_allclose(r, kin.rot("z", yaw) @ kin.rot("y", pitch) @ kin.rot("x", lift), atol=1e-12)
            np.testing.assert_allclose(kin.lift_pitch_yaw(r), (lift, pitch, yaw), atol=1e-9)

    def test_error_measure(self):
        r = kin.rot("x", 0.3) @ kin.rot("z", -1.0)
        self.assertAlmostEqual(kin.orientation_error_deg(r, r), 0.0, places=5)
        self.assertAlmostEqual(kin.orientation_error_deg(r @ kin.rot("y", np.radians(7.0)), r), 7.0, places=6)
        # a raw readout that is scaled and slightly sheared is first made a rotation
        self.assertLess(kin.orientation_error_deg(0.8 * r + 0.01, r), 1.0)
        self.assertAlmostEqual(kin.orthonormality_defect(r), 0.0, places=12)
        self.assertGreater(kin.orthonormality_defect(0.8 * r), 0.3)
        self.assertAlmostEqual(np.linalg.det(kin.nearest_rotation(-r)), 1.0, places=9)

    def test_chance_level_is_the_mean_angle_of_a_random_rotation(self):
        rng = np.random.default_rng(2)
        q = rng.normal(size=(20000, 4))
        w = np.abs(q[:, 0]) / np.linalg.norm(q, axis=1)                # random unit quaternions
        self.assertAlmostEqual(np.degrees(np.mean(2.0 * np.arccos(w))), kin.CHANCE_ERROR_DEG, delta=1.0)


class ProductFeatureTests(unittest.TestCase):
    def test_paper_readout_matrix_is_exact_for_every_chain(self):
        rng = np.random.default_rng(3)
        for name, chain in CHAINS.items():
            t = kin.readout_matrix(chain)
            self.assertEqual(t.shape, (9, 3 ** chain.n))
            for _ in range(10):
                q = chain.random_pose(rng)
                np.testing.assert_allclose(t @ kin.product_features(q), chain.rotation(q).reshape(9), atol=1e-9, err_msg=name)

    def test_number_of_product_features_that_occur(self):
        # 2 .. 7 joints: 8, 13, 21, 34, 55, 89 of the 3^n - 1 possible products, for both families
        for family in ("generic", "tiago"):
            counts = [len(kin.used_products(get_chain("%s%d" % (family, n)))) for n in range(2, 8)]
            self.assertEqual(counts, [8, 13, 21, 34, 55, 89], family)
        self.assertEqual(3 ** 7 - 1, 2186)
        self.assertEqual(kin.monomial_name((1, 0, 2)), "c1·s3")

    def test_the_paper_chain_is_rx_ry(self):
        chain = get_chain("generic2")
        q = [0.4, -1.1]
        np.testing.assert_allclose(chain.rotation(q), kin.rot("x", 0.4) @ kin.rot("y", -1.1), atol=1e-12)


class WiringTests(unittest.TestCase):
    def test_joint_by_joint_and_all_at_once_equal_the_direct_product(self):
        rng = np.random.default_rng(4)
        for name, chain in CHAINS.items():
            wirings = [chain_wiring(chain, relay=True), chain_wiring(chain, relay=False), flat_wiring(chain)]
            for _ in range(5):
                q = chain.random_pose(rng)
                true_links, true_tool = chain.link_rotations(q), chain.rotation(q)
                for wiring in wirings:
                    values = wiring.evaluate(q)
                    np.testing.assert_allclose(wiring.tool_matrix(values), true_tool, atol=1e-9, err_msg="%s %s" % (name, wiring.mode))
                    for decoded, true in zip(wiring.link_matrices(values), true_links):
                        np.testing.assert_allclose(decoded, true, atol=1e-9, err_msg=name)

    def test_pairwise_products_grow_by_twelve_per_joint(self):
        for family in ("generic", "tiago"):
            counts = [len(chain_wiring(get_chain("%s%d" % (family, n))).products) for n in range(1, 8)]
            self.assertEqual(counts, [0, 4, 14, 26, 38, 50, 62], family)
        self.assertEqual(len(chain_wiring(get_chain("generic7"), relay=False).products), 62)

    def test_all_at_once_builds_every_feature_pairwise(self):
        # one product per feature with two or more factors, plus the beginnings they share
        # (for 3 joints: 8 + 4 features and the two beginnings s1·s2 and c1·s2)
        for n, expected in ((2, 4), (3, 14)):
            chain = get_chain("generic%d" % n)
            wiring = flat_wiring(chain)
            self.assertEqual(len(wiring.products), expected)
            self.assertEqual(wiring.entries, [])
            self.assertEqual(wiring.links, [])

    def test_every_link_is_available_and_untouched_entries_are_not_copied(self):
        wiring = chain_wiring(get_chain("generic7"))
        self.assertEqual(len(wiring.links), 7)
        # a joint leaves three of the nine entries as they are: at most six new populations per joint
        per_link = {}
        for entry in wiring.entries:
            per_link[entry.link] = per_link.get(entry.link, 0) + 1
        self.assertTrue(all(count <= 6 for count in per_link.values()), per_link)
        self.assertNotIn(1, per_link)                 # link 1 is the first joint's cos and sin themselves
        summary = wiring.summary()
        self.assertEqual((summary["joints"], summary["products"]), (7, 62))

    def test_models_of_the_tests(self):
        self.assertEqual(len(MODEL_NAMES), 12)
        self.assertEqual([get_chain(name).n for name in MODEL_NAMES], [2, 3, 4, 5, 6, 7] * 2)


def _urdf_xml():
    if shutil.which("xacro") is None or shutil.which("rospack") is None:
        return None
    try:
        package = subprocess.run(["rospack", "find", "tiago_description"], capture_output=True, text=True, timeout=60).stdout.strip()
        if not package:
            return None
        result = subprocess.run(
            ["xacro", package + "/robots/tiago.urdf.xacro", "arm:=true", "end_effector:=pal-hey5", "ft_sensor:=schunk-ft",
             "laser_model:=sick-571", "camera_model:=orbbec-astra"], capture_output=True, text=True, timeout=120)
        return result.stdout if result.returncode == 0 and result.stdout.strip() else None
    except (OSError, subprocess.SubprocessError):
        return None


class TiagoUrdfTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        xml = _urdf_xml()
        if xml is None:
            raise unittest.SkipTest("the TIAGo robot description is not installed here")
        root = ET.fromstring(xml)
        by_child = {j.find("child").get("link"): j for j in root.findall("joint") if j.find("child") is not None}
        link, joints = TIAGO_TOOL_LINK, []
        while link != TIAGO_BASE_LINK:
            joint = by_child[link]
            joints.append(joint)
            link = joint.find("parent").get("link")
        cls.joints = list(reversed(joints))
        cls.xml = xml

    @staticmethod
    def _numbers(text):
        return [float(v) for v in text.split()]

    def _urdf_frames(self, q):
        """Independent of ring_so3.kinematics: 4 x 4 transforms composed as the URDF defines them."""

        def transform(xyz, rpy):
            roll, pitch, yaw = rpy
            cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
            t = np.eye(4)
            t[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp, cp * sr, cp * cr]]
            t[:3, 3] = xyz
            return t

        pose, frames, angles = np.eye(4), [], list(q)
        for joint in self.joints:
            origin = joint.find("origin")
            pose = pose @ transform(self._numbers(origin.get("xyz", "0 0 0")), self._numbers(origin.get("rpy", "0 0 0")))
            if joint.get("type") == "revolute":
                axis = np.array(self._numbers(joint.find("axis").get("xyz")))
                angle = angles.pop(0)
                k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
                turn = np.eye(4)
                turn[:3, :3] = np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)      # Rodrigues
                pose = pose @ turn
            frames.append(pose.copy())
        return frames

    def test_table_values_are_the_urdf_values(self):
        revolute = [j for j in self.joints if j.get("type") == "revolute"]
        self.assertEqual([j.get("name") for j in revolute], [row[0] for row in TIAGO_ARM])
        for joint, (name, xyz, rpy, lower, upper, velocity) in zip(revolute, TIAGO_ARM):
            np.testing.assert_allclose(self._numbers(joint.find("origin").get("xyz")), xyz, atol=1e-9, err_msg=name)
            np.testing.assert_allclose(self._numbers(joint.find("origin").get("rpy")), rpy, atol=1e-9, err_msg=name)
            self.assertEqual(self._numbers(joint.find("axis").get("xyz")), [0.0, 0.0, 1.0], name)
            limit = joint.find("limit")
            np.testing.assert_allclose([float(limit.get("lower")), float(limit.get("upper")), float(limit.get("velocity"))],
                                       [lower, upper, velocity], atol=1e-9, err_msg=name)

    def test_orientation_and_position_of_every_link_equal_the_urdf(self):
        chain = get_chain("tiago7")
        rng = np.random.default_rng(5)
        for _ in range(25):
            q = chain.random_pose(rng)
            urdf = self._urdf_frames(q)                     # arm_1 .. arm_7 links, then the tool link
            ours = chain.frames(q)[1:]                      # the same, without the base
            self.assertEqual(len(urdf), len(ours))
            for expected, (origin, rotation) in zip(urdf, ours):
                np.testing.assert_allclose(rotation, expected[:3, :3], atol=1e-6)
            # origins: ours are the joint origins (before the joint turns), which the turn does not move
            for expected, (origin, _) in zip(urdf, ours):
                np.testing.assert_allclose(origin, expected[:3, 3], atol=1e-6)
            np.testing.assert_allclose(chain.rotation(q), urdf[-1][:3, :3], atol=1e-6)
            for n in range(2, 7):
                np.testing.assert_allclose(get_chain("tiago%d" % n).rotation(q[:n]), urdf[n - 1][:3, :3], atol=1e-6)

    def test_against_the_kdl_solver_when_it_is_installed(self):
        try:
            import PyKDL
            from kdl_parser_py.urdf import treeFromString
        except ImportError:
            self.skipTest("PyKDL / kdl_parser_py are not installed")
        ok, tree = treeFromString(self.xml)
        self.assertTrue(ok)
        kdl_chain = tree.getChain(TIAGO_BASE_LINK, TIAGO_TOOL_LINK)
        solver = PyKDL.ChainFkSolverPos_recursive(kdl_chain)
        chain = get_chain("tiago7")
        rng = np.random.default_rng(6)
        for _ in range(10):
            q = chain.random_pose(rng)
            joints = PyKDL.JntArray(7)
            for i, value in enumerate(q):
                joints[i] = float(value)
            frame = PyKDL.Frame()
            solver.JntToCart(joints, frame)
            rotation = np.array([[frame.M[i, j] for j in range(3)] for i in range(3)])
            np.testing.assert_allclose(chain.rotation(q), rotation, atol=1e-6)
            np.testing.assert_allclose(chain.frames(q)[-1][0], [frame.p[0], frame.p[1], frame.p[2]], atol=1e-6)


if __name__ == "__main__":
    unittest.main()

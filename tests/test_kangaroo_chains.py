"""Task 1: the Kangaroo arm chains equal the robot's URDF (skipped where the robot description is not installed)."""

import os
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from ring_so3.chains import KANGAROO_BASE_LINK, KANGAROO_TOOL_LINK, get_chain

URDF = ("/opt/ros/humble/share/kangaroo_mujoco/models/mjcf_data_floating_4dof_no-end-effector_no-end-effector_fixed/"
        "robot_description_formatted.urdf")
SHOULDER_UPPER = {"left": 1.4229904547193268, "right": 1.4622630704048099}   # SO-2 measured values of arm_<s>_2_joint
RX_PI = np.diag([1.0, -1.0, -1.0])


class KangarooUrdfTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not os.path.exists(URDF):
            raise unittest.SkipTest("the Kangaroo robot description is not installed here")
        with open(URDF) as f:
            text = f.read()
        root = ET.fromstring(text[:text.index("</robot>") + len("</robot>")])
        by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        cls.joints = {}
        for side in ("left", "right"):
            link, joints = KANGAROO_TOOL_LINK % side, []
            while link != KANGAROO_BASE_LINK:
                joints.append(by_child[link])
                link = joints[-1].find("parent").get("link")
            cls.joints[side] = list(reversed(joints))

    @staticmethod
    def _numbers(text):
        return [float(v) for v in text.split()]

    def _urdf_frames(self, side, q):
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
        for joint in self.joints[side]:
            origin = joint.find("origin")
            pose = pose @ transform(self._numbers(origin.get("xyz", "0 0 0")), self._numbers(origin.get("rpy", "0 0 0")))
            origins = pose[:3, 3].copy()
            if joint.get("type") == "revolute":
                axis = np.array(self._numbers(joint.find("axis").get("xyz")))
                angle = angles.pop(0)
                k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
                turn = np.eye(4)
                turn[:3, :3] = np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)      # Rodrigues
                pose = pose @ turn
            frames.append((origins, pose[:3, :3].copy()))
        return frames           # fixed mount, joints 1-4, fixed tool

    def test_tool_orientation_equals_the_urdf(self):
        rng = np.random.default_rng(7)
        for side in ("left", "right"):
            chain = get_chain("kangaroo_%s4" % side)
            for _ in range(25):
                q = chain.random_pose(rng)
                np.testing.assert_allclose(chain.rotation(q), self._urdf_frames(side, q)[-1][1], atol=1e-4, err_msg=side)

    def test_link_frames_equal_the_urdf_up_to_the_folded_half_turn(self):
        rng = np.random.default_rng(11)
        for side in ("left", "right"):
            chain = get_chain("kangaroo_%s4" % side)
            for _ in range(25):
                q = chain.random_pose(rng)
                urdf = self._urdf_frames(side, q)[1:]           # joints 1-4, tool
                ours = chain.frames(q)[1:]
                self.assertEqual(len(urdf), len(ours))
                for k, ((u_origin, u_rot), (origin, rot)) in enumerate(zip(urdf, ours)):
                    expected = u_rot @ RX_PI if k in (0, 2) else u_rot
                    np.testing.assert_allclose(rot, expected, atol=1e-4, err_msg="%s frame %d" % (side, k + 1))
                    np.testing.assert_allclose(origin, u_origin, atol=1e-4, err_msg="%s origin %d" % (side, k + 1))

    def test_limits_and_speeds_are_the_urdf_values_except_the_measured_shoulder(self):
        for side in ("left", "right"):
            chain = get_chain("kangaroo_%s4" % side)
            revolute = [j for j in self.joints[side] if j.get("type") == "revolute"]
            self.assertEqual(chain.n, len(revolute))
            expected = []
            for joint in revolute:
                limit = joint.find("limit")
                expected.append([float(limit.get("lower")), float(limit.get("upper"))])
            expected[1][1] = SHOULDER_UPPER[side]
            np.testing.assert_allclose(chain.limits(), expected, atol=1e-12, err_msg=side)
            np.testing.assert_allclose([j.max_velocity for j in chain.joints],
                                       [float(j.find("limit").get("velocity")) for j in revolute], atol=1e-12, err_msg=side)


if __name__ == "__main__":
    unittest.main()

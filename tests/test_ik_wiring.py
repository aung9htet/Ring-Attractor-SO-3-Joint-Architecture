"""The comparison and the cascade as products and sums give what the exact rule gives."""

import unittest

import numpy as np

from ring_so3.chains import get_chain
from ring_so3.ik import rules
from ring_so3.ik.design import IkDesign
from ring_so3.ik.pairs import DEVELOPMENT_SEED, pairs
from ring_so3.ik.wiring import ik_wiring
from ring_so3.kinematics import lift_pitch_yaw


class IkWiringTest(unittest.TestCase):
    def check(self, name, comparison, cascade, near=None):
        chain = get_chain(name)
        design = IkDesign(comparison=comparison, cascade=cascade)
        wiring = ik_wiring(chain, design=design)
        caps = rules.share_caps(chain, design)
        compared = 0
        for pair in pairs(chain, 12, DEVELOPMENT_SEED):
            q = np.asarray(pair.start)
            goal = np.asarray(pair.goal if near is None else lift_pitch_yaw(chain.rotation(q + near * (pair.goal_pose - q))))
            room = rules.limit_room(chain, q, design)
            values = wiring.evaluate(q, goal, room=room)
            e = rules.turn(chain.rotation(q), goal, comparison)
            np.testing.assert_allclose(wiring.turn_vector(values), np.clip(e, -0.5, 0.5), atol=1e-9)
            if np.max(np.abs(e)) >= 0.5:
                continue                      # the comparison population is full; the exact rule has no such bound
            compared += 1
            shares, rests = rules.cascade(rules.joint_axes(chain, q), e, caps, room, pass_on=cascade)
            np.testing.assert_allclose(wiring.share_values(values), shares, atol=1e-9)
            np.testing.assert_allclose(wiring.left_vector(values), rests[-1], atol=1e-9)
        return compared

    def test_matrix_cascade(self):
        for name in ("generic3", "generic7", "tiago7"):
            self.check(name, "matrix", True)
            self.assertGreaterEqual(self.check(name, "matrix", True, near=0.03), 8)

    def test_angles_cascade(self):
        for name in ("generic3", "tiago7"):
            self.check(name, "angles", True)
            self.assertGreaterEqual(self.check(name, "angles", True, near=0.03), 8)

    def test_all_at_once(self):
        self.assertGreaterEqual(self.check("generic7", "matrix", False, near=0.03), 8)

    def test_counts(self):
        chain = get_chain("generic7")
        matrix = ik_wiring(chain, design=IkDesign(comparison="matrix")).summary()
        angles = ik_wiring(chain, design=IkDesign(comparison="angles")).summary()
        self.assertGreater(matrix["products"], angles["products"])
        self.assertEqual(matrix["share_populations"], 14)


class DrawingCountsTest(unittest.TestCase):
    """The architecture drawing counts its boxes from the wiring; the counts must add up to the wiring's own."""

    def test_parts_add_up(self):
        import os
        import sys

        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
        import draw_ik_architecture as drawing
        from ring_so3.design import Sizes

        for name, comparison, total in (("generic3", "matrix", 16900), ("tiago7", "matrix", 37620), ("generic3", "angles", 12640)):
            chain = get_chain(name)
            wiring, parts = drawing.count(chain, IkDesign(comparison=comparison))
            products = parts["goal"][0] + parts["comparison"][0] + sum(p[0] for p in parts["share"]) + sum(p[0] for p in parts["rest"])
            sums = parts["goal"][1] + parts["comparison"][1] + sum(p[1] for p in parts["rest"])
            halves = sum(p[1] for p in parts["share"])
            self.assertEqual((products, sums, halves), (len(wiring.muls), len(wiring.relays), len(wiring.halves)))
            self.assertEqual(parts["share"][0][0], 0)                      # the first joint's axis is a constant
            self.assertEqual(sum(drawing.neurons(chain, wiring, Sizes())), total)


if __name__ == "__main__":
    unittest.main()

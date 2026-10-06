"""The inverse-kinematics layer in NEST, with few points (the full measurements: scripts/run_ik_*.py)."""

import unittest

import numpy as np

try:
    import nest
except ImportError:          # pragma: no cover
    nest = None

from ring_so3.design import Design
from ring_so3.ik import experiments
from ring_so3.ik.design import IkDesign
from ring_so3.ik.pairs import DEVELOPMENT_SEED


@unittest.skipIf(nest is None, "NEST is not installed")
class GoalRingTest(unittest.TestCase):
    def test_written_ring_leads_with_the_share(self):
        from ring_so3.nest import calibrate, goal_rig, ik_calibrate
        from ring_so3.nest.goal_ring import WrittenRingSettings

        design = Design()
        rows = goal_rig.measure_lead(nest, calibrate.load(design), ik_calibrate.load(design), (-1.0, -0.2, 0.0, 0.2, 1.0),
                                     WrittenRingSettings(), design, seconds=3.0, threads=4)
        lead = [row["lead_mean"] for row in goal_rig.summarise(rows, settle_s=1.0)]
        self.assertLess(abs(lead[2]), 0.5)                       # no lead without a share
        self.assertLess(lead[0], -6.0)
        self.assertGreater(lead[4], 6.0)
        self.assertTrue(lead[0] < lead[1] < lead[2] < lead[3] < lead[4])
        self.assertTrue(all(row["silent_ticks"] == 0 for row in goal_rig.summarise(rows)))


@unittest.skipIf(nest is None, "NEST is not installed")
class IkNetworkTest(unittest.TestCase):
    def test_comparison_and_shares_open_loop(self):
        document = experiments.open_loop_run(nest, "generic3", 8, IkDesign(goal_ring="none"), DEVELOPMENT_SEED, log=None, threads=4)
        summary = document["summary"]
        self.assertLess(summary["turn_direction_error_median_deg"], 10.0)
        self.assertGreaterEqual(summary["share_right_sign"], 0.9)
        self.assertLess(summary["share_rms_rad"], 0.03)

    def test_three_joints_reach_a_goal(self):
        document = experiments.reach_run(nest, "generic3", 100, IkDesign(), DEVELOPMENT_SEED, shard=(0, 50), log=None, threads=4)
        rows = document["rows"]
        self.assertEqual(len(rows), 2)
        for row in rows:
            if row["exact_reached"]:
                self.assertLess(row["final_error_deg"], 8.0)
            self.assertLess(row["largest_lead"], 12.0)


if __name__ == "__main__":
    unittest.main()

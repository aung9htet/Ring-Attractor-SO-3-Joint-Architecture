"""The simulator's IK mode on the exact-rule stand-in network (no NEST)."""

import unittest

import numpy as np

from ring_so3.chains import get_chain
from ring_so3.ik.design import IkDesign
from ring_so3.kinematics import lift_pitch_yaw
from ring_so3.sim.page import PAGE_HTML
from ring_so3.sim.server import Simulator, StubIkNetwork, StubNetwork


def make(chain, cascade=True):
    return StubIkNetwork(chain, IkDesign(cascade=cascade), noise_deg=0.2)


class IkSimulatorTests(unittest.TestCase):
    def test_the_arm_reaches_a_goal_orientation(self):
        simulator = Simulator(make, "generic7", realtime=False, mode="manual")
        self.assertTrue(simulator.config()["ik"])
        chain = get_chain("generic7")
        goal = lift_pitch_yaw(chain.rotation(chain.random_pose(np.random.default_rng(5))))
        simulator.state.commands.put({"command": "goal", "lpy": [float(v) for v in goal]})
        for _ in range(300):
            snapshot = simulator.tick()
        ik = snapshot["ik"]
        self.assertEqual(snapshot["mode"], "ik")
        self.assertTrue(ik["on"])
        self.assertLess(ik["goal_error"], 5.0)
        self.assertEqual(len(ik["shares"]), 7)
        self.assertEqual(len(ik["joint_goal_rings"]), 7)
        self.assertEqual(len(ik["goal_rings"]), 3)
        self.assertEqual(len(ik["goal_matrix"]), 9)

    def test_sliders_take_over_and_the_goal_stays_on_the_hand(self):
        simulator = Simulator(make, "generic3", realtime=False, mode="manual")
        simulator.state.commands.put({"command": "random_goal"})
        for _ in range(20):
            simulator.tick()
        simulator.state.commands.put({"command": "target", "q": [0.3, -0.2, 0.5]})
        for _ in range(200):
            snapshot = simulator.tick()
        self.assertEqual(snapshot["mode"], "manual")
        np.testing.assert_allclose(snapshot["q"], [0.3, -0.2, 0.5], atol=5e-3)
        self.assertLess(snapshot["ik"]["goal_error"], 1.0)             # the goal has followed the hand
        self.assertLess(max(abs(v) for v in snapshot["ik"]["shares"]), 0.05)

    def test_the_cascade_switch_rebuilds(self):
        simulator = Simulator(make, "generic3", realtime=False, mode="manual")
        simulator.state.commands.put({"command": "cascade", "on": False})
        snapshot = simulator.tick()
        self.assertFalse(snapshot["ik"]["cascade"])

    def test_without_the_ik_layer_nothing_changes(self):
        simulator = Simulator(lambda chain: StubNetwork(chain), "generic3", realtime=False, mode="goals")
        simulator.state.commands.put({"command": "random_goal"})          # ignored
        snapshot = simulator.tick()
        self.assertIsNone(snapshot["ik"])
        self.assertFalse(simulator.config()["ik"])
        self.assertEqual(snapshot["mode"], "goals")

    def test_page_has_the_ik_section(self):
        for piece in ('id="iksec"', "/api/random_goal", "/api/cascade", "joint_goal_rings"):
            self.assertIn(piece, PAGE_HTML)


class IkSimulatorPageTests(unittest.TestCase):
    """The IK simulator (scripts/ik_simulator.py): methods, runs, replay, a goal given as a matrix."""

    def make(self, chain="generic7"):
        from ring_so3.sim.ik_server import IkSimulator

        return IkSimulator(make, chain, realtime=False)

    def test_every_method_reaches_the_same_goal_from_the_same_start(self):
        from ring_so3.sim.ik_server import METHODS

        simulator = self.make()
        simulator.handle({"command": "random_goal"})
        for _ in range(260):
            snapshot = simulator.tick()
        start = simulator.run["start"]
        self.assertLess(snapshot["ik"]["goal_error"], 5.0)
        for method in METHODS[1:]:
            simulator.handle({"command": "replay", "method": method})
            for _ in range(420):
                snapshot = simulator.tick()
            run = snapshot["runs"][-1]
            self.assertEqual((run["method"], snapshot["method"], snapshot["mode"]), (method, method, "ik"))
            np.testing.assert_allclose(simulator.run["start"], start, atol=1e-6)
            self.assertLess(run["final_error"], 5.0, method)
            self.assertIsNotNone(run["reached_s"], method)
        self.assertEqual(len(snapshot["runs"]), 6)

    def test_a_dragged_goal_is_a_matrix_and_a_run_starts_when_it_is_let_go(self):
        simulator = self.make("generic3")
        quarter = [0, -1, 0, 1, 0, 0, 0, 0, 1]                    # a quarter turn about z
        simulator.handle({"command": "goal", "matrix": quarter, "commit": False})
        first = simulator.run["id"]
        simulator.handle({"command": "goal", "matrix": quarter, "commit": False})
        self.assertEqual(simulator.run["id"], first)               # still the same run while dragging
        simulator.handle({"command": "goal", "matrix": quarter, "commit": True})
        for _ in range(200):
            snapshot = simulator.tick()
        np.testing.assert_allclose(snapshot["ik"]["goal"], [0.0, 0.0, np.pi / 2], atol=1e-3)
        self.assertLess(snapshot["ik"]["goal_error"], 5.0)

    def test_sliders_take_over_and_stop_holds(self):
        simulator = self.make("generic3")
        simulator.handle({"command": "random_goal"})
        simulator.tick()
        simulator.handle({"command": "target", "q": [0.2, 0.1, -0.3]})
        for _ in range(60):
            snapshot = simulator.tick()
        self.assertEqual(snapshot["mode"], "manual")
        np.testing.assert_allclose(snapshot["q"], [0.2, 0.1, -0.3], atol=1e-3)
        simulator.handle({"command": "stop"})
        self.assertEqual(simulator.tick()["mode"], "hold")

    def test_gazebo_is_offered_only_when_it_is_there(self):
        simulator = self.make("tiago7")
        self.assertEqual(simulator.config()["robots"], ["light"])
        simulator.handle({"command": "robot", "robot": "gazebo"})   # ignored
        self.assertEqual(simulator.tick()["robot"], "light")

    def test_kangaroo_is_offered_only_for_its_chains(self):
        from ring_so3.robots import LightRobot
        from ring_so3.sim.ik_server import IkSimulator

        class StandIn(LightRobot):                       # a light robot that also answers the lock-step block
            sim_time_s, lag = 0.0, 0.0
            link = type("Link", (), {"frames": lambda self: 0})()

        simulator = IkSimulator(make, "kangaroo_left4", realtime=False, robots=("light", "kangaroo"),
                                make_robot=lambda name, chain: StandIn(chain))
        offered = simulator.config()["robot_chains"]["kangaroo"]
        self.assertIn("kangaroo_left4", offered)
        self.assertFalse([name for name in offered if name.startswith("tiago")])
        simulator.handle({"command": "robot", "robot": "kangaroo"})
        self.assertEqual(simulator.tick()["robot"], "kangaroo")
        simulator.handle({"command": "chain", "chain": "tiago7"})
        simulator.tick()
        simulator.handle({"command": "robot", "robot": "kangaroo"})   # ignored
        self.assertEqual(simulator.tick()["robot"], "light")

    def test_page(self):
        from ring_so3.sim.ik_page import IK_PAGE_HTML

        for piece in ("/api/replay", "/api/method", "/gazebo.jpg", "nearestRing", "matrix: dragGoal.matrix"):
            self.assertIn(piece, IK_PAGE_HTML)


if __name__ == "__main__":
    unittest.main()

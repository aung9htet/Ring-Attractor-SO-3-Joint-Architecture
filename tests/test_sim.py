"""The simulator without NEST: the light robot, the main loop on a stand-in network, and the HTTP interface."""

import json
import threading
import unittest
import urllib.request

import numpy as np

from ring_so3.chains import get_chain
from ring_so3.kinematics import orientation_error_deg
from ring_so3.sim.robot import GoalSequence, SimRobot
from ring_so3.sim.server import SimServer, SimState, Simulator, StubNetwork


class RobotTests(unittest.TestCase):
    def test_moves_at_the_speed_limit_and_lands_on_the_target(self):
        chain = get_chain("tiago3")
        robot = SimRobot(chain, chain.limits().mean(axis=1), max_speed=0.5)
        target = chain.limits()[:, 1] - 0.1
        robot.set_target(target)
        previous, steps = robot.q.copy(), 0
        while not robot.arrived():
            q = robot.step(0.05)
            self.assertTrue(np.all(np.abs(q - previous) <= 0.5 * 0.05 + 1e-9))
            previous, steps = q.copy(), steps + 1
            self.assertLess(steps, 2000)
        np.testing.assert_allclose(robot.q, target, atol=2.5e-3)

    def test_stays_inside_the_limits_and_takes_the_short_way_round_a_circle(self):
        tiago = get_chain("tiago2")
        robot = SimRobot(tiago)
        robot.set_target([10.0, -10.0])
        np.testing.assert_allclose(robot.target, [tiago.joints[0].upper, tiago.joints[1].lower])
        generic = get_chain("generic2")
        robot = SimRobot(generic, [3.0, 0.0])
        robot.set_target([-3.0, 0.0])                                   # 0.28 rad through pi, not 6 rad back
        self.assertAlmostEqual(abs(robot.remaining()[0]), 2 * np.pi - 6.0, places=9)
        for _ in range(100):
            robot.step(0.05)
        self.assertTrue(robot.arrived())

    def test_goal_sequence_gives_a_new_goal_after_the_rest(self):
        chain = get_chain("generic3")
        robot, goals = SimRobot(chain, max_speed=2.0), GoalSequence(chain, seed=1, dwell_s=0.2)
        changes = 0
        for _ in range(600):
            changes += bool(goals.update(robot, 0.05))
            robot.step(0.05)
        self.assertGreaterEqual(changes, 3)
        self.assertEqual(len(goals.goals), changes)


class SimulatorTests(unittest.TestCase):
    def test_snapshot_has_true_and_decoded_frames_for_every_link(self):
        simulator = Simulator(lambda chain: StubNetwork(chain, noise_deg=0.2), "tiago7", realtime=False, mode="goals", speed=1.0)
        for _ in range(40):
            snapshot = simulator.tick()
        chain = get_chain("tiago7")
        self.assertEqual(len(snapshot["points"]), chain.n + 2)
        self.assertEqual((len(snapshot["true"]), len(snapshot["decoded"]), len(snapshot["link_error"])), (8, 8, 7))
        self.assertEqual((len(snapshot["rings"]), len(snapshot["rings"][0]), len(snapshot["out_rings"])), (7, 200, 3))
        true_tool = np.array(snapshot["true"][-1]).reshape(3, 3)
        np.testing.assert_allclose(true_tool, chain.rotation(snapshot["q"]), atol=2e-3)
        self.assertLess(orientation_error_deg(np.array(snapshot["decoded"][-1]).reshape(3, 3), true_tool), 2.0)
        self.assertLess(snapshot["error"], 2.0)
        self.assertTrue(snapshot["moving"] or snapshot["mode"] == "goals")
        json.dumps(snapshot)                                           # everything in it is plain JSON

    def test_commands_change_target_mode_speed_and_chain(self):
        simulator = Simulator(lambda chain: StubNetwork(chain), "generic3", realtime=False, mode="manual")
        simulator.state.commands.put({"command": "target", "q": [0.5, -0.5, 1.0]})
        simulator.state.commands.put({"command": "speed", "speed": 9.0})
        for _ in range(200):
            snapshot = simulator.tick()
        np.testing.assert_allclose(snapshot["q"], [0.5, -0.5, 1.0], atol=3e-3)
        self.assertEqual(simulator.speed, 2.0)                         # clipped
        simulator.state.commands.put({"command": "mode", "mode": "goals"})
        simulator.state.commands.put({"command": "chain", "chain": "tiago4"})
        snapshot = simulator.tick()
        self.assertEqual((simulator.chain.name, len(snapshot["q"]), snapshot["mode"]), ("tiago4", 4, "goals"))
        self.assertEqual(simulator.state.config["chain"], "tiago4")
        simulator.state.commands.put({"command": "target", "q": [1, 2]})        # wrong length: ignored
        simulator.tick()
        self.assertEqual(simulator.mode, "goals")


class WriteAheadTests(unittest.TestCase):
    def test_lead_writes_the_angle_the_joint_will_have_and_stays_inside_limits(self):
        from ring_so3.session import LEAD_BASE_S, LEAD_PER_JOINT_S, Session, lead_times

        np.testing.assert_allclose(lead_times(3), [LEAD_BASE_S + 2 * LEAD_PER_JOINT_S, LEAD_BASE_S + LEAD_PER_JOINT_S, LEAD_BASE_S])
        chain = get_chain("tiago3")
        robot = SimRobot(chain, chain.limits().mean(axis=1), max_speed=0.5)
        session = Session(StubNetwork(chain), robot, lead=True)
        robot.set_target(chain.limits()[:, 1])
        session.tick()
        np.testing.assert_allclose(session.written(), robot.q + lead_times(3) * 0.5, atol=1e-9)       # moving at the limit
        for _ in range(400):
            session.tick()
        self.assertTrue(robot.arrived())
        np.testing.assert_allclose(session.written(), chain.limits()[:, 1], atol=3e-3)                # at rest: the angle itself
        session.lead = False
        np.testing.assert_array_equal(session.written(), robot.q)

    def test_simulator_switch(self):
        simulator = Simulator(lambda chain: StubNetwork(chain), "generic3", realtime=False, mode="goals", speed=1.0)
        self.assertFalse(simulator.tick()["lead"])
        simulator.state.commands.put({"command": "lead", "on": True})
        self.assertTrue(simulator.tick()["lead"])
        self.assertTrue(simulator.session.lead)


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.state = SimState()
        self.server = SimServer(self.state, "127.0.0.1", 0)
        self.server.start()
        self.simulator = Simulator(lambda chain: StubNetwork(chain), "generic2", self.state, realtime=False, mode="manual")
        self.base = "http://127.0.0.1:%d" % self.server.port

    def tearDown(self):
        self.server.stop()

    def _get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=5) as response:
            return response.read().decode("utf-8")

    def _post(self, path, payload):
        request = urllib.request.Request(self.base + path, data=json.dumps(payload).encode("utf-8"),
                                         headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(request, timeout=5) as response:
            return json.loads(response.read().decode("utf-8"))

    def test_page_config_state_and_a_command(self):
        self.simulator.tick()
        page = self._get("/")
        self.assertIn("<canvas id=\"view\">", page)
        self.assertIn("EventSource(\"/events\")", page)
        config = json.loads(self._get("/config"))
        self.assertEqual((config["chain"], len(config["joints"]), config["stub"]), ("generic2", 2, True))
        self.assertTrue(self._post("/api/target", {"q": [0.3, 0.4]})["ok"])
        for _ in range(100):
            self.simulator.tick()
        state = json.loads(self._get("/state"))
        np.testing.assert_allclose(state["q"], [0.3, 0.4], atol=3e-3)
        history = json.loads(self._get("/history"))
        self.assertEqual((len(history), sorted(history[-1])), (101, ["error", "gerr", "goal", "ring", "t", "tr"]))
        self.assertEqual(history[-1]["t"], state["t"])
        with self.assertRaises(urllib.error.HTTPError):
            self._post("/api/nothing", {})

    def test_events_stream_starts_with_the_config_and_then_sends_ticks(self):
        received = []

        def read():
            with urllib.request.urlopen(self.base + "/events", timeout=5) as response:
                for raw in response:
                    line = raw.decode("utf-8").strip()
                    if line.startswith("event:"):
                        received.append(line.split(":", 1)[1].strip())
                    if len(received) >= 3:
                        return

        thread = threading.Thread(target=read, daemon=True)
        thread.start()
        for _ in range(200):
            self.simulator.tick()
            if len(received) >= 3:
                break
            thread.join(0.02)
        thread.join(2.0)
        self.assertEqual(received[0], "config")
        self.assertIn("tick", received[1:])


if __name__ == "__main__":
    unittest.main()

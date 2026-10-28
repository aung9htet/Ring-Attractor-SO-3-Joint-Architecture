"""The IK simulator's main loop: an orientation goal you set, a method that turns the arm to it, a robot that moves.

Served by the HTTP server of :mod:`ring_so3.sim.server` with the page of :mod:`ring_so3.sim.ik_page`.

* **Methods** (:data:`METHODS`): ``ring`` is the spiking network (each joint follows its goal ring);
  ``ring_all`` the same with every joint getting the whole comparison (the network is rebuilt);
  ``cascade`` the exact cascade; ``transpose``, ``dls``, ``ccd`` the standard methods of
  :mod:`ring_so3.standard.ik`.  With any method but the two ring ones the network still runs and
  shows its rings, but the joints follow the method, which reads the exact joint angles.
* **Robots**: ``light`` (no dynamics), and ``gazebo`` when the simulator was started beside a
  running Gazebo (the TIAGo chains only), ``kangaroo`` beside the MuJoCo Kangaroo (its chains only).
* **Runs**: a run starts when a goal is given (random goal, a drag that is let go, or "same
  goal, other method").  Its distance from the goal over time is kept, so that methods can be
  compared on the same start pose and goal.

Commands (``POST /api/<name>``): ``goal {lpy | matrix, commit}``, ``random_goal``,
``method {method}``, ``robot {robot}``, ``replay {method}``, ``target {q}``, ``stop``,
``speed {speed}``, ``chain {chain}``, ``quit``.
"""

from __future__ import annotations

import queue
import time
from typing import Any, Callable, Dict, List, Optional

import numpy as np

from ..chains import CHAINS, KANGAROO_MODELS, MODEL_NAMES, get_chain
from ..ik import rules
from ..ik.session import IkSession
from ..kinematics import Chain, lift_pitch_yaw, nearest_rotation, orientation_error_deg, rotation_angle
from ..robots import LightRobot, start_pose
from ..standard.targets import exact_targets
from .ik_page import IK_PAGE_HTML
from .server import SimState, StubNetwork

METHODS = ("ring", "ring_all", "cascade", "dls", "transpose", "ccd")
METHOD_LABELS = {"ring": "Ring IK (spiking network, cascade)", "ring_all": "Ring IK, every joint gets the whole turn",
                 "cascade": "Exact cascade (no neurons)", "dls": "Damped least squares", "transpose": "Jacobian transpose",
                 "ccd": "Cyclic coordinate descent"}
COMMANDS = ("goal", "random_goal", "method", "robot", "replay", "target", "stop", "speed", "chain", "quit")
ROBOT_CHAINS = {"gazebo": "tiago", "kangaroo": "kangaroo_"}     # robot -> prefix of the chains it can move
KEPT_RUNS = 6
REACHED_DEG = 5.0


class IkSimulator:
    def __init__(self, make_network: Callable[..., Any], chain_name: str = "tiago7", state: Optional[SimState] = None,
                 make_robot: Optional[Callable[[str, Chain], Any]] = None, robots: tuple = ("light",), robot: str = "light",
                 method: str = "ring", seed: int = 0, realtime: bool = True, speed: float = 1.0) -> None:
        """``make_network(chain, cascade=True)`` builds an IK network; ``make_robot(name, chain)`` a robot other than the light one."""

        self.make_network = make_network
        self.make_robot = make_robot
        self.robots = tuple(robots)
        self.state = state or SimState()
        self.state.page = IK_PAGE_HTML
        self.state.allowed = COMMANDS
        self.robot_name = robot if robot in self.robots else "light"
        self.method = method
        self.seed = int(seed)
        self.realtime = bool(realtime)
        self.speed = float(speed)
        self.running = True
        self._wall_per_tick = 0.0
        self.rng = np.random.default_rng(self.seed)
        self.load(chain_name)

    # -- building ------------------------------------------------------------
    def load(self, chain_name: str, keep_pose: Optional[np.ndarray] = None) -> None:
        self.chain = get_chain(chain_name)
        if not self._usable(self.robot_name):
            self.robot_name = "light"
        started = time.time()
        self.network = self.make_network(self.chain, cascade=self.method != "ring_all")
        self.build_s = time.time() - started
        self.dt_s = float(self.network.options.tick_ms) / 1000.0
        self.session = IkSession(self.network, robot=self._robot(self.robot_name, keep_pose))
        self._apply_method()
        self.mode = "hold"                       # "hold": nothing asked; "ik": reaching; "manual": sliders; "placing": back to a start pose
        self.manual_target = self.session.q.copy()
        self.runs: List[Dict[str, Any]] = []
        self.run: Optional[Dict[str, Any]] = None
        self.pending: Optional[Dict[str, Any]] = None
        self._previous_rotation = self.chain.rotation(self.session.q)
        self.state.publish("config", self.config())

    def _usable(self, robot: str) -> bool:
        return robot == "light" or self.chain.name.startswith(ROBOT_CHAINS.get(robot, "\0"))

    def _robot(self, name: str, q: Optional[np.ndarray] = None) -> Any:
        if name != "light" and self.make_robot is not None:
            return self.make_robot(name, self.chain)
        return LightRobot(self.chain, q if q is not None else start_pose(self.chain))

    def _apply_method(self) -> None:
        self.session.method = None if self.method in ("ring", "ring_all") else exact_targets(self.method)
        self.session.method_shares = None

    def config(self) -> Dict[str, Any]:
        return {
            "chain": self.chain.name, "description": self.chain.description, "chains": MODEL_NAMES + KANGAROO_MODELS,
            "joints": [{"name": j.name, "lower": j.lower, "upper": j.upper, "circular": j.circular, "axis": j.axis} for j in self.chain.joints],
            "neurons": int(self.network.neuron_count()), "ring_size": int(self.session.ring_size),
            "tick_ms": float(self.network.options.tick_ms), "build_s": round(self.build_s, 2),
            "reach": float(sum(np.linalg.norm(j.offset) for j in self.chain.joints) + np.linalg.norm(self.chain.tool_offset)),
            "stub": isinstance(self.network, StubNetwork), "ik": True,
            "caps": [round(float(c), 4) for c in rules.share_caps(self.chain, self.session.design, self.session.ring_size)],
            "methods": [{"name": m, "label": METHOD_LABELS[m]} for m in METHODS], "robots": list(self.robots),
            "robot_chains": {r: [n for n in MODEL_NAMES + KANGAROO_MODELS if n.startswith(ROBOT_CHAINS[r])] for r in self.robots if r in ROBOT_CHAINS},
        }

    # -- commands ------------------------------------------------------------
    def handle(self, command: Dict[str, Any]) -> None:
        name = command.get("command")
        if name == "goal":
            goal = None
            if isinstance(command.get("matrix"), list) and len(command["matrix"]) == 9:
                goal = np.array(lift_pitch_yaw(nearest_rotation(np.array(command["matrix"], dtype=float).reshape(3, 3))))
            elif isinstance(command.get("lpy"), list) and len(command["lpy"]) == 3:
                lpy = [float(v) for v in command["lpy"]]
                goal = np.array([float(rules.wrap(lpy[0])), min(max(lpy[1], -np.pi / 2), np.pi / 2), float(rules.wrap(lpy[2]))])
            if goal is not None:
                self._give(goal, new_run=bool(command.get("commit", True)))
        elif name == "random_goal":
            self._give(np.array(lift_pitch_yaw(self.chain.rotation(self.chain.random_pose(self.rng)))), new_run=True)
        elif name == "method" and command.get("method") in METHODS:
            self._set_method(command["method"])
        elif name == "robot" and command.get("robot") in self.robots:
            if command["robot"] != self.robot_name and self._usable(command["robot"]):
                self.robot_name = command["robot"]
                self._end_run()
                self.session.robot = self._robot(self.robot_name, self.session.q.copy())
                self.mode = "hold"
        elif name == "replay":
            source = self.run or (self.runs[-1] if self.runs else None)
            if source is not None:
                method = command.get("method") if command.get("method") in METHODS else METHODS[(METHODS.index(self.method) + 1) % len(METHODS)]
                self._end_run()
                self._set_method(method)
                self.pending = {"start": np.array(source["start"]), "goal": np.array(source["goal"])}
                self.mode = "placing"
        elif name == "target":
            q = command.get("q")
            if isinstance(q, list) and len(q) == self.chain.n:
                self._end_run()
                self.mode = "manual"
                self.manual_target = self.chain.clip(q) if not self.chain.joints[0].circular else np.array([float(v) for v in q])
        elif name == "stop":
            self._end_run()
            self.mode = "hold"
        elif name == "speed":
            self.speed = min(max(float(command.get("speed", self.speed)), 0.05), 3.0)
        elif name == "chain" and command.get("chain") in CHAINS:
            self.load(command["chain"])
        elif name == "quit":
            self.running = False

    def _set_method(self, method: str) -> None:
        rebuild = (method == "ring_all") != (self.method == "ring_all")
        self.method = method
        if rebuild:                               # the cascade is wired in: the other ring variant is another network
            runs, pose = self.runs, self.session.q.copy()
            self.load(self.chain.name, keep_pose=pose)
            self.runs = runs
        else:
            self._apply_method()

    def _give(self, goal: np.ndarray, new_run: bool) -> None:
        self.session.goal = goal
        if new_run or self.mode != "ik" or self.run is None:
            self._end_run()
            rotation = self.chain.rotation(self.session.q)
            self.run = {"id": (self.runs[-1]["id"] + 1) if self.runs else 1, "method": self.method, "robot": self.robot_name,
                        "start": self.session.q.copy().tolist(), "goal": goal.tolist(), "t": [], "error": [], "path": 0.0,
                        "start_error": float(np.degrees(rotation_angle(rotation, rules.goal_matrix(*goal)))), "reached_s": None}
            self._previous_rotation = rotation
        else:
            self.run["goal"] = goal.tolist()      # a drag in progress: the same run, the goal moves
        self.mode = "ik"

    def _end_run(self) -> None:
        if self.run is not None and self.run["t"]:
            self.runs = (self.runs + [self.run])[-KEPT_RUNS:]
        self.run = None

    # -- one tick ------------------------------------------------------------
    def _step(self) -> Any:
        session = self.session
        if self.mode == "ik":
            return session.tick()
        if self.mode == "placing" and self.pending is not None:
            difference = session.robot.difference(self.pending["start"])
            there = np.max(np.abs(difference)) < session.robot.tolerance
            if there and np.max(np.abs(rules.wrap(session.written - session.goal))) < 1e-6:
                goal, self.pending = self.pending["goal"], None
                self._give(goal, new_run=True)
                return session.tick()
            limit = 3.0 * self.dt_s
            session.robot.follow(session.q + np.clip(difference, -limit, limit) if not there else self.pending["start"], self.dt_s, self.dt_s, exact=True)
            return session.hold(move=False)
        if self.mode == "manual":
            difference = session.robot.difference(self.manual_target)
            session.robot.follow(session.q + np.clip(difference, -self.speed * self.dt_s, self.speed * self.dt_s), self.dt_s, self.dt_s, exact=True)
            return session.hold(move=False)
        return session.hold()

    def tick(self) -> Dict[str, Any]:
        started = time.time()
        while True:
            try:
                self.handle(self.state.commands.get_nowait())
            except queue.Empty:
                break
        result = self._step()
        snapshot = self.snapshot(result)
        self._wall_per_tick = 0.9 * self._wall_per_tick + 0.1 * (time.time() - started) if self._wall_per_tick else time.time() - started
        rate = self.dt_s / max(self._wall_per_tick, 1e-6)
        snapshot["rate"] = round(min(rate, 1.0) if self.realtime else rate, 2)
        self.state.publish("tick", snapshot)
        if self.realtime:
            spare = self.dt_s - (time.time() - started)
            if spare > 0:
                time.sleep(spare)
        return snapshot

    def _run_row(self, run: Dict[str, Any], live: bool) -> Dict[str, Any]:
        every = max(len(run["t"]) // 150, 1)
        final = run["error"][-1] if run["error"] else None
        return {"id": run["id"], "method": run["method"], "robot": run["robot"], "live": live,
                "start_error": round(run["start_error"], 1), "reached_s": run["reached_s"],
                "final_error": None if final is None else round(final, 2),
                "path_ratio": round(run["path"] / run["start_error"], 2) if run["start_error"] > 1.0 else None,
                "t": run["t"][::every], "error": run["error"][::every]}

    def snapshot(self, result: Any) -> Dict[str, Any]:
        session, chain, tick = self.session, self.chain, result.tick
        q = session.q
        frames = chain.frames(q)
        true = [rotation for _, rotation in frames[1:]]
        raw = list(tick.links) + [tick.tool]
        decoded = [nearest_rotation(m) for m in raw]
        goal_matrix = rules.goal_matrix(*session.goal)
        goal_error = float(np.degrees(rotation_angle(true[-1], goal_matrix)))
        if self.run is not None and self.mode == "ik":
            run = self.run
            run["path"] += float(np.degrees(rotation_angle(self._previous_rotation, true[-1])))
            run["t"].append(round(len(run["t"]) * self.dt_s + self.dt_s, 3))
            run["error"].append(round(goal_error, 2))
            if run["reached_s"] is None and goal_error <= REACHED_DEG:
                run["reached_s"] = run["t"][-1]
            elif goal_error > REACHED_DEG:
                run["reached_s"] = None           # reached means: within 5 deg and staying there
        self._previous_rotation = true[-1]
        number = lambda v: None if v is None or not np.isfinite(v) else round(float(v), 4)  # noqa: E731
        r3 = lambda values: [round(float(v), 4) for v in values]  # noqa: E731
        shares = result.shares if session.method is None or session.method_shares is None else session.method_shares
        robot = session.robot
        gazebo = None
        if self.robot_name != "light":
            link = robot.link
            gazebo = {"sim_time": round(float(robot.sim_time_s), 3), "frames": int(link.frames()), "lag": r3(np.ravel(robot.lag)),
                      "overshoot_ms": round(1000.0 * float(getattr(link, "overshoot_s", 0.0)), 2)}
        return {
            "t": round(tick.t_ms / 1000.0, 3), "chain": chain.name, "mode": self.mode, "method": self.method, "robot": self.robot_name,
            "speed": self.speed, "q": r3(q), "target": r3(self.manual_target if self.mode == "manual" else robot.target),
            "points": [r3(origin) for origin, _ in frames],
            "true": [r3(m.reshape(9)) for m in true], "decoded": [r3(m.reshape(9)) for m in decoded],
            "error": round(orientation_error_deg(raw[-1], true[-1]), 3),
            "lpy_true": r3(lift_pitch_yaw(true[-1])),
            "lpy_ring": [None if not np.isfinite(v) else round(float(v), 4) for v in tick.angles],
            "rings": [[int(c) for c in row] for row in self.network.ring_counts()],
            "written": [round(float(ring.target_index), 2) if ring.target_index is not None else None for ring in self.network.rings],
            "out_rings": [[int(c) for c in row] for row in self.network.output_counts()],
            "ik": {
                "on": self.mode == "ik", "goal": [number(v) for v in session.goal], "written": [number(v) for v in session.written],
                "goal_matrix": r3(goal_matrix.reshape(9)), "goal_error": round(goal_error, 3),
                "turn": [number(v) for v in result.turn], "shares": [number(v) for v in shares],
                "network_shares": [number(v) for v in result.shares],
                "left": number(float(np.linalg.norm(result.left))), "lead": [number(v) for v in result.lead],
                "goal_rings": [[int(c) for c in row] for row in self.network.goal_counts()],
                "joint_goal_rings": [[int(c) for c in row] for row in self.network.joint_goal_counts()],
            },
            "runs": [self._run_row(run, False) for run in self.runs] + ([self._run_row(self.run, True)] if self.run is not None and self.run["t"] else []),
            "gazebo": gazebo,
        }

    def run_loop(self, max_ticks: Optional[int] = None) -> None:
        done = 0
        while self.running and (max_ticks is None or done < max_ticks):
            self.tick()
            done += 1


__all__ = ["COMMANDS", "IkSimulator", "METHODS", "METHOD_LABELS", "exact_targets"]

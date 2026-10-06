"""The simulator: the spiking network live behind one browser page.

A standard-library HTTP server (daemon threads) serves the page and a few JSON
endpoints; the network and the light robot run in the main thread, tick by tick:

* ``GET /``            the page (:mod:`ring_so3.sim.page`)
* ``GET /config``      chain, joints and limits, sizes
* ``GET /state``       the latest tick
* ``GET /history``     time, error and angles of the last 30 s (a page that is opened or reloaded fills its charts from it)
* ``GET /events``      server-sent events: one ``tick`` message per tick, ``config`` after a rebuild
* ``POST /api/target`` ``{"q": [...]}``          joint targets (the sliders); switches to manual mode
* ``POST /api/mode``   ``{"mode": "manual"|"goals"}``   goals: the arm keeps moving from goal to goal
* ``POST /api/speed``  ``{"speed": rad_per_s}``  joint speed limit
* ``POST /api/chain``  ``{"chain": "tiago7"}``   rebuild the network for another chain
* ``POST /api/random`` one random target pose (manual mode)
* ``POST /api/lead``   ``{"on": true|false}``    write each joint's angle ahead by the readout's delay
* ``POST /api/goal``   ``{"lpy": [lift, pitch, yaw]}``  an orientation goal: the arm reaches it by itself (IK network only)
* ``POST /api/random_goal``  the orientation of a random pose as the goal (IK network only)
* ``POST /api/cascade`` ``{"on": true|false}``  shares passed down the arm, or every joint gets the whole comparison (rebuilds)
* ``POST /api/quit``

The network is anything with the interface of :class:`ring_so3.nest.network.FkNetwork`;
:class:`StubNetwork` is an exact-kinematics stand-in used by the tests and by
``scripts/simulator.py --stub`` to try the page without NEST.
"""

from __future__ import annotations

import json
import queue
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, List, Optional

import numpy as np

from ..chains import CHAINS, MODEL_NAMES, get_chain
from ..ik import rules
from ..ik.design import IkDesign
from ..ik.session import IkSession
from ..kinematics import (Chain, lift_pitch_yaw, nearest_rotation, orientation_error_deg, rotation_angle)
from ..mapping import RingMap
from ..session import Session
from .page import PAGE_HTML
from .robot import GoalSequence, SimRobot

TICK_MS = 50.0


class _StubRing:
    def __init__(self, mapping: RingMap) -> None:
        self.mapping = mapping
        self.counts = np.zeros(mapping.size)
        self.target_index = 0.0

    def write(self, angle: float) -> None:
        self.target_index = self.mapping.position(angle)
        offsets = (np.arange(self.mapping.size) - self.target_index + self.mapping.size / 2.0) % self.mapping.size - self.mapping.size / 2.0
        self.counts = np.round(3.0 * np.exp(-0.5 * (offsets / 4.0) ** 2))


class _StubOptions:
    tick_ms = TICK_MS
    encoder = "fractional"


class StubNetwork:
    """Exact kinematics with a little noise, in the shape of the spiking network (no NEST)."""

    def __init__(self, chain: Chain, noise_deg: float = 0.5, seed: int = 0, ring_size: int = 200) -> None:
        from ..nest.network import Tick        # the record the spiking network returns (needs no NEST to import)

        self._tick_type = Tick
        self.chain = chain
        self.options = _StubOptions()
        self.rings = [_StubRing(RingMap.for_joint(joint, ring_size)) for joint in chain.joints]
        self.rng = np.random.default_rng(seed)
        self.noise = np.radians(noise_deg)
        self.t_ms = 0.0
        self._outputs = np.zeros((3, 100))

    def step(self, q):
        angles = np.asarray(q, dtype=float)
        for ring, angle in zip(self.rings, angles):
            ring.write(float(angle))
        self.t_ms += TICK_MS
        links = np.array(self.chain.link_rotations(angles)) + self.rng.normal(0.0, self.noise / 2.0, (self.chain.n, 3, 3))
        tool = self.chain.rotation(angles) + self.rng.normal(0.0, self.noise / 2.0, (3, 3))
        lpy = np.array(lift_pitch_yaw(nearest_rotation(tool)))
        theta = 2.0 * np.pi * np.arange(100) / 100.0
        self._outputs = np.round(4.0 * np.maximum(np.cos(theta[None, :] - lpy[:, None]) - 0.25, 0.0))
        return self._tick_type(self.t_ms, angles.copy(), np.zeros(self.chain.n), links, tool, lpy, self._outputs.sum(axis=1))

    def output_counts(self) -> np.ndarray:
        return self._outputs

    def ring_counts(self) -> np.ndarray:
        return np.array([ring.counts for ring in self.rings])

    def neuron_count(self) -> int:
        return 0


class StubIkNetwork(StubNetwork):
    """The exact rule in the shape of the spiking IK network (no NEST)."""

    def __init__(self, chain: Chain, ik_design: Optional[IkDesign] = None, noise_deg: float = 0.5, seed: int = 0, ring_size: int = 200) -> None:
        from ..nest.ik_network import IkTick

        super().__init__(chain, noise_deg, seed, ring_size)
        self._ik_tick_type = IkTick
        self.ik_design = ik_design or IkDesign()
        self.calibration = type("StubCalibration", (), {"settings": {"ring_size": int(ring_size)}})()
        self.caps = rules.share_caps(chain, self.ik_design, ring_size)
        self.goal_rings = [_StubRing(RingMap(ring_size, -np.pi, np.pi, True)) for _ in range(3)]
        self.joint_goal_rings = [_StubRing(RingMap.for_joint(joint, ring_size)) for joint in chain.joints]
        self.ik_wiring = type("StubWiring", (), {"caps": self.caps})()

    def step_ik(self, q, goal_lpy):
        tick = self.step(q)
        angles = np.asarray(q, dtype=float)
        goal = np.asarray(goal_lpy, dtype=float)
        e = np.clip(rules.turn(nearest_rotation(tick.tool), goal, self.ik_design.comparison), -0.5, 0.5)
        room = rules.limit_room(self.chain, angles, self.ik_design, int(self.calibration.settings["ring_size"]))
        shares, rests = rules.cascade(rules.joint_axes(self.chain, angles), e, self.caps, room, self.ik_design.cascade)
        shares = shares + self.rng.normal(0.0, 0.003, self.chain.n)
        for ring, angle in zip(self.goal_rings, goal):
            ring.write(float(angle))
        joint_goal = angles + shares
        lead = np.zeros(self.chain.n)
        for k, ring in enumerate(self.joint_goal_rings):
            ring.write(float(joint_goal[k]))
            lead[k] = shares[k] / ring.mapping.radians_per_index
        return self._ik_tick_type(tick, e, shares, rests[1:], goal.copy(), joint_goal, lead)

    def goal_counts(self) -> np.ndarray:
        return np.array([ring.counts for ring in self.goal_rings])

    def joint_goal_counts(self) -> np.ndarray:
        return np.array([ring.counts for ring in self.joint_goal_rings])


class SimState:
    """What the HTTP threads and the main loop share."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._subscribers: List["queue.Queue[str]"] = []
        self.commands: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.config: Dict[str, Any] = {}
        self.snapshot: Dict[str, Any] = {}
        #: the page that is served, the commands it may post, and where a camera picture comes from (the IK simulator sets these)
        self.page: str = PAGE_HTML
        self.allowed = ("target", "mode", "speed", "chain", "random", "lead", "goal", "random_goal", "cascade", "quit")
        self.image: Optional[Callable[[], Optional[bytes]]] = None
        self.history: "deque[Dict[str, Any]]" = deque(maxlen=600)          # 30 s of 50 ms ticks

    def subscribe(self) -> "queue.Queue[str]":
        subscriber: "queue.Queue[str]" = queue.Queue(maxsize=100)
        with self._lock:
            self._subscribers.append(subscriber)
        return subscriber

    def unsubscribe(self, subscriber: "queue.Queue[str]") -> None:
        with self._lock:
            if subscriber in self._subscribers:
                self._subscribers.remove(subscriber)

    def publish(self, kind: str, payload: Dict[str, Any]) -> None:
        text = "event: %s\ndata: %s\n\n" % (kind, json.dumps(payload, separators=(",", ":")))
        with self._lock:
            if kind == "config":
                self.config = payload
                self.history.clear()
            else:
                self.snapshot = payload
                self.history.append({"t": payload.get("t"), "error": payload.get("error"),
                                     "tr": payload.get("lpy_true"), "ring": payload.get("lpy_ring"),
                                     "gerr": (payload.get("ik") or {}).get("goal_error"),
                                     "goal": (payload.get("ik") or {}).get("written")})
            for subscriber in list(self._subscribers):
                try:
                    subscriber.put_nowait(text)
                except queue.Full:                      # a page that stopped reading: drop its oldest message
                    try:
                        subscriber.get_nowait()
                        subscriber.put_nowait(text)
                    except (queue.Empty, queue.Full):
                        pass


class _Handler(BaseHTTPRequestHandler):
    server_version = "RingSO3Sim/1"

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A002 - stdlib signature
        return

    def _send(self, body: bytes, content_type: str, code: int = 200) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, payload: Any, code: int = 200) -> None:
        self._send(json.dumps(payload).encode("utf-8"), "application/json", code)

    def do_GET(self) -> None:  # noqa: N802 - stdlib naming
        state: SimState = self.server.state          # type: ignore[attr-defined]
        path = self.path.split("?")[0]
        if path == "/":
            self._send(state.page.encode("utf-8"), "text/html; charset=utf-8")
        elif path == "/gazebo.jpg":
            picture = state.image() if state.image is not None else None
            if picture is None:
                self._json({"error": "no picture"}, 404)
            else:
                self._send(picture, "image/jpeg")
        elif path == "/config":
            self._json(state.config)
        elif path == "/state":
            self._json(state.snapshot)
        elif path == "/history":
            self._json(list(state.history))
        elif path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            subscriber = state.subscribe()
            try:
                if state.config:
                    self.wfile.write(("event: config\ndata: %s\n\n" % json.dumps(state.config)).encode("utf-8"))
                    self.wfile.flush()
                while True:
                    try:
                        self.wfile.write(subscriber.get(timeout=10.0).encode("utf-8"))
                    except queue.Empty:
                        self.wfile.write(b": keep-alive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                state.unsubscribe(subscriber)
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self) -> None:  # noqa: N802 - stdlib naming
        state: SimState = self.server.state          # type: ignore[attr-defined]
        length = int(self.headers.get("Content-Length") or 0)
        try:
            payload = json.loads(self.rfile.read(length).decode("utf-8") or "{}") if length else {}
        except ValueError:
            self._json({"error": "the body is not JSON"}, 400)
            return
        name = self.path.split("?")[0]
        if not name.startswith("/api/"):
            self._json({"error": "not found"}, 404)
            return
        command = dict(payload if isinstance(payload, dict) else {}, command=name[len("/api/"):])
        if command["command"] not in state.allowed:
            self._json({"error": "unknown command"}, 404)
            return
        state.commands.put(command)
        self._json({"ok": True})


class SimServer:
    def __init__(self, state: SimState, host: str = "0.0.0.0", port: int = 8765) -> None:
        self.httpd = ThreadingHTTPServer((host, int(port)), _Handler)
        self.httpd.daemon_threads = True
        self.httpd.state = state                         # type: ignore[attr-defined]
        self.port = self.httpd.server_address[1]
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.httpd.shutdown()
        self.httpd.server_close()


class Simulator:
    """The main loop: robot + network + what the page asks for."""

    def __init__(self, make_network: Callable[[Chain], Any], chain_name: str = "tiago7", state: Optional[SimState] = None,
                 speed: float = 0.5, mode: str = "manual", seed: int = 0, realtime: bool = True, lead: bool = False) -> None:
        self.make_network = make_network
        self.state = state or SimState()
        self.speed = float(speed)
        self.mode = mode
        self.lead = bool(lead)
        self.seed = int(seed)
        self.realtime = bool(realtime)
        self.running = True
        self.cascade = True
        self._wall_per_tick = 0.0
        self.load(chain_name)

    def load(self, chain_name: str) -> None:
        self.chain = get_chain(chain_name)
        started = time.time()
        try:
            self.network = self.make_network(self.chain, cascade=self.cascade)
        except TypeError:                                 # a network without the IK layer takes the chain only
            self.network = self.make_network(self.chain)
        self.build_s = time.time() - started
        start = np.zeros(self.chain.n) if self.chain.joints[0].circular else self.chain.limits().mean(axis=1)
        self.robot = SimRobot(self.chain, start, max_speed=self.speed)
        self.session = Session(self.network, self.robot, lead=self.lead)
        self.ik = IkSession(self.network, start) if hasattr(self.network, "step_ik") else None
        if self.ik is None and self.mode == "ik":
            self.mode = "manual"
        self.goals = GoalSequence(self.chain, self.seed, dwell_s=1.0)
        self.dt_s = self.session.dt_s
        self.state.publish("config", self.config())

    def config(self) -> Dict[str, Any]:
        return {
            "chain": self.chain.name, "description": self.chain.description, "chains": MODEL_NAMES,
            "joints": [{"name": j.name, "lower": j.lower, "upper": j.upper, "circular": j.circular, "axis": j.axis}
                       for j in self.chain.joints],
            "neurons": int(self.network.neuron_count()), "ring_size": int(self.network.rings[0].mapping.size),
            "tick_ms": float(self.network.options.tick_ms), "build_s": round(self.build_s, 2),
            "reach": float(sum(np.linalg.norm(j.offset) for j in self.chain.joints) + np.linalg.norm(self.chain.tool_offset)),
            "stub": isinstance(self.network, StubNetwork), "ik": self.ik is not None,
            "caps": [round(float(c), 4) for c in self.network.ik_wiring.caps] if self.ik is not None else None,
        }

    def handle(self, command: Dict[str, Any]) -> None:
        name = command.get("command")
        if name == "target":
            q = command.get("q")
            if isinstance(q, list) and len(q) == self.chain.n:
                self.mode = "manual"
                self.robot.set_target([float(v) for v in q])
        elif name == "goal" and self.ik is not None:
            lpy = command.get("lpy")
            if isinstance(lpy, list) and len(lpy) == 3:
                self._to_ik()
                self.ik.goal = np.array([float(rules.wrap(lpy[0])), min(max(float(lpy[1]), -np.pi / 2), np.pi / 2), float(rules.wrap(lpy[2]))])
        elif name == "random_goal" and self.ik is not None:
            self._to_ik()
            self.ik.goal = np.array(lift_pitch_yaw(self.chain.rotation(self.goals.next_goal())))
        elif name == "cascade" and self.ik is not None:
            self.cascade = bool(command.get("on"))
            self.load(self.chain.name)
        elif name == "mode" and command.get("mode") in ("manual", "goals"):
            self.mode = command["mode"]
            if self.mode == "manual":
                self.robot.set_target(self.robot.q)
        elif name == "speed":
            self.speed = min(max(float(command.get("speed", self.speed)), 0.05), 2.0)
            self.robot.max_speed = self.speed
        elif name == "random":
            self.mode = "manual"
            self.robot.set_target(self.goals.next_goal())
        elif name == "lead":
            self.lead = bool(command.get("on"))
            self.session.lead = self.lead
        elif name == "chain" and command.get("chain") in CHAINS:
            self.load(command["chain"])
        elif name == "quit":
            self.running = False

    def _to_ik(self) -> None:
        if self.mode != "ik":
            self.mode = "ik"
            self.ik.q = self.robot.q.copy()

    def _tick_network(self) -> Any:
        """One tick of robot and network.  With the IK layer: in mode ``ik`` the arm follows its goal rings;
        otherwise it is moved from outside and the goal on the goal rings stays on the hand."""

        if self.ik is None:
            return self.session.tick()
        if self.mode == "ik":
            result = self.ik.tick()
            self.robot.q = self.ik.q.copy()
            self.robot.set_target(self.robot.q)
            self.robot.velocity = np.zeros(self.chain.n)
        else:
            self.ik.q = self.robot.step(self.dt_s).copy()
            result = self.ik.hold()
        self._ik_tick = result
        return result.tick

    def tick(self) -> Dict[str, Any]:
        started = time.time()
        while True:
            try:
                self.handle(self.state.commands.get_nowait())
            except queue.Empty:
                break
        if self.mode == "goals":
            self.goals.update(self.robot, self.dt_s)
        tick = self._tick_network()
        snapshot = self.snapshot(tick)
        self._wall_per_tick = 0.9 * self._wall_per_tick + 0.1 * (time.time() - started) if self._wall_per_tick else time.time() - started
        rate = self.dt_s / max(self._wall_per_tick, 1e-6)                 # how fast the network could run
        snapshot["rate"] = round(min(rate, 1.0) if self.realtime else rate, 2)   # how fast it does: held at real time
        self.state.publish("tick", snapshot)
        if self.realtime:
            spare = self.dt_s - (time.time() - started)
            if spare > 0:
                time.sleep(spare)
        return snapshot

    def snapshot(self, tick: Any) -> Dict[str, Any]:
        q = self.robot.q
        frames = self.chain.frames(q)                       # base, links 1..n, tool
        true = [rotation for _, rotation in frames[1:]]
        raw = list(tick.links) + [tick.tool]
        decoded = [nearest_rotation(m) for m in raw]
        lpy_true = lift_pitch_yaw(true[-1])
        r3 = lambda values: [round(float(v), 4) for v in values]  # noqa: E731
        return {
            "t": round(tick.t_ms / 1000.0, 3), "chain": self.chain.name, "mode": self.mode, "speed": self.speed, "lead": self.lead,
            "q": r3(q), "target": r3(self.robot.target), "moving": bool(not self.robot.arrived()),
            "points": [r3(origin) for origin, _ in frames],
            "true": [r3(m.reshape(9)) for m in true], "decoded": [r3(m.reshape(9)) for m in decoded],
            "error": round(orientation_error_deg(raw[-1], true[-1]), 3),
            "link_error": [round(orientation_error_deg(raw[k], true[k]), 3) for k in range(self.chain.n)],
            "lpy_true": r3(lpy_true),
            "lpy_ring": [None if not np.isfinite(v) else round(float(v), 4) for v in tick.angles],
            "lpy_matrix": r3(lift_pitch_yaw(decoded[-1])),
            "rings": [[int(c) for c in row] for row in self.network.ring_counts()],
            "written": [round(float(ring.target_index), 2) if ring.target_index is not None else None for ring in self.network.rings],
            "out_rings": [[int(c) for c in row] for row in self.network.output_counts()],
            "ik": self.ik_snapshot(true[-1]) if self.ik is not None else None,
        }

    def ik_snapshot(self, true_tool: np.ndarray) -> Dict[str, Any]:
        tick, ik = self._ik_tick, self.ik
        number = lambda v: None if not np.isfinite(v) else round(float(v), 4)  # noqa: E731
        goal = rules.goal_matrix(*ik.goal)
        return {
            "on": self.mode == "ik", "cascade": bool(self.network.ik_design.cascade),
            "goal": [number(v) for v in ik.goal], "written": [number(v) for v in ik.written],
            "goal_matrix": [round(float(v), 4) for v in goal.reshape(9)],
            "goal_error": round(float(np.degrees(rotation_angle(true_tool, goal))), 3),
            "turn": [number(v) for v in tick.turn], "shares": [number(v) for v in tick.shares],
            "left": number(float(np.linalg.norm(tick.left))), "lead": [number(v) for v in tick.lead],
            "goal_rings": [[int(c) for c in row] for row in self.network.goal_counts()],
            "joint_goal_rings": [[int(c) for c in row] for row in self.network.joint_goal_counts()],
        }

    def run(self, max_ticks: Optional[int] = None) -> None:
        done = 0
        while self.running and (max_ticks is None or done < max_ticks):
            self.tick()
            done += 1


__all__ = ["SimServer", "SimState", "Simulator", "StubIkNetwork", "StubNetwork"]

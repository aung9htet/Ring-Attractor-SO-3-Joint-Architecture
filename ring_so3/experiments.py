"""The measurements behind docs/results.md: one function per kind of run, each returning a JSON-ready dict.

``scripts/run_static.py`` and ``scripts/run_tracking.py`` are thin command lines over
:func:`static_run` and :func:`tracking_run`; ``scripts/run_experiments.py`` lists every
run of the results page and starts them side by side.
"""

from __future__ import annotations

import json
import os
import time
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from .chains import get_chain
from .design import Design, Sizes
from .kinematics import Chain, circular_difference, lift_pitch_yaw, orientation_error_deg
from .session import Session, pose_row
from .sim.robot import GoalSequence, SimRobot

POSE_SEED = 20261005          # the test poses; calibration and size selection never use it


def make_design(product: Optional[int] = None, entry: Optional[int] = None, feature: Optional[int] = None,
                ring: Optional[int] = None, seed: int = 1) -> Design:
    d = Sizes()
    return Design(sizes=Sizes(ring or d.ring, feature or d.feature, product or d.product, entry or d.entry, d.output_ring), seed=int(seed))


def build(backend: Any, chain_name: str, design: Optional[Design] = None, max_speed: float = 1.0, lead: bool = False,
          **options: Any):
    """A network for ``chain_name`` with the stored calibration of its design and encoder, and a session on it."""

    from .nest import calibrate
    from .nest.network import FkNetwork, NetworkOptions

    chain = get_chain(chain_name)
    design = design or Design()
    network_options = NetworkOptions(**options)
    calibration = calibrate.load(design, design.sizes.ring, network_options.encoder)
    network = FkNetwork(backend, chain, calibration, design, network_options)
    session = Session(network, SimRobot(chain, np.zeros(chain.n) if chain.joints[0].circular else chain.limits().mean(axis=1),
                                        max_speed=max_speed), lead=lead)
    return chain, design, network, session


def test_poses(chain: Chain, count: int, seed: int = POSE_SEED) -> List[np.ndarray]:
    rng = np.random.default_rng([int(seed), chain.n, 0 if chain.joints[0].circular else 1])
    return [chain.random_pose(rng) for _ in range(int(count))]


def sweep_poses(chain: Chain, points: int, seed: int = POSE_SEED) -> List[Dict[str, Any]]:
    """Every joint in turn over its whole range, the others at one fixed random pose."""

    rng = np.random.default_rng([int(seed), chain.n, 7])
    base = chain.random_pose(rng)
    out = []
    for j, joint in enumerate(chain.joints):
        values = np.linspace(joint.lower, joint.upper, int(points), endpoint=not joint.circular)
        for value in values:
            q = base.copy()
            q[j] = value
            out.append({"joint": j, "q": q})
    return out


def legacy_sweep_poses(points: int) -> List[Dict[str, Any]]:
    """The two sweeps the old two-joint decoder was tested on: one joint over the circle, the other at 0."""

    values = np.linspace(-np.pi, np.pi, int(points), endpoint=False)
    out = [{"joint": 1, "q": np.array([0.0, v])} for v in values]            # "q1 fixed"
    out += [{"joint": 0, "q": np.array([v, 0.0])} for v in values]           # "q2 fixed"
    return out


def _round(value: Any, digits: int = 5) -> Any:
    if isinstance(value, float):
        return round(value, digits) if np.isfinite(value) else None
    if isinstance(value, (list, tuple)):
        return [_round(v, digits) for v in value]
    if isinstance(value, dict):
        return {k: _round(v, digits) for k, v in value.items()}
    if isinstance(value, (np.floating, np.integer)):
        return _round(value.item(), digits)
    if isinstance(value, np.ndarray):
        return _round(value.tolist(), digits)
    return value


def write_json(path: str, document: Dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(_round(document), stream, indent=None, separators=(",", ":"))


def _meta(chain: Chain, design: Design, network: Any, started: float, sim_s: float) -> Dict[str, Any]:
    o = network.options
    wall = time.time() - started
    return {
        "chain": chain.name, "joints": chain.n, "design": design.describe(),
        "options": {"mode": o.mode, "relay": o.relay, "encoder": o.encoder, "threads": o.threads, "seed": o.seed,
                    "tick_ms": o.tick_ms, "readout_tau_ms": o.readout_tau_ms, "noise_pa": o.noise_pa},
        "neurons": network.neuron_count(), "products": len(network.products), "link_populations": len(network.entries),
        "synapses": int(network.backend.GetKernelStatus("num_connections")),
        "simulated_s": round(sim_s, 2), "wall_s": round(wall, 1), "speed": round(sim_s / wall, 3) if wall > 0 else None,
        "calibration_fit": network.calibration.fit,
    }


def static_run(backend: Any, chain_name: str, poses: int = 300, sweep_points: int = 24, design: Optional[Design] = None,
               silence: float = 0.0, pose_seed: int = POSE_SEED, max_speed: float = 1.0, legacy_sweep_points: int = 0,
               log=print, **options: Any) -> Dict[str, Any]:
    """Random held poses and a sweep of every joint.  ``silence``: share of product neurons switched off.
    ``legacy_sweep_points``: also the old decoder's two sweeps (two-joint chains only)."""

    started = time.time()
    chain, design, network, session = build(backend, chain_name, design, max_speed, **options)
    dead = network.silence(silence, seed=3) if silence > 0.0 else 0
    session.warm_up()
    rows: List[Dict[str, Any]] = []
    for i, q in enumerate(test_poses(chain, poses, pose_seed)):
        row = pose_row(chain, network, session.go_to(q))
        row.update(kind="random", index=i)
        rows.append(row)
        if log is not None and (i + 1) % 50 == 0:
            errors = [r["error_deg"] for r in rows]
            log("%s: %d poses, error median %.2f p90 %.2f deg, %.0f s" % (chain_name, i + 1, np.median(errors), np.percentile(errors, 90), time.time() - started))
    for i, item in enumerate(sweep_poses(chain, sweep_points, pose_seed) if sweep_points > 0 else []):
        row = pose_row(chain, network, session.go_to(item["q"]))
        row.update(kind="sweep", index=i, sweep_joint=item["joint"])
        rows.append(row)
    if legacy_sweep_points > 0:
        if chain.n != 2:
            raise ValueError("the old decoder's sweeps are for two joints")
        for i, item in enumerate(legacy_sweep_poses(legacy_sweep_points)):
            row = pose_row(chain, network, session.go_to(item["q"]))
            row.update(kind="legacy_sweep", index=i, sweep_joint=item["joint"])
            rows.append(row)
    document = _meta(chain, design, network, started, network.t_ms / 1000.0)
    document.update(experiment="static", pose_seed=int(pose_seed), max_speed=float(max_speed), silenced_neurons=int(dead),
                    silence=float(silence), rows=rows)
    return document


def tracking_run(backend: Any, chain_name: str, speed: float = 0.5, goals: int = 20, dwell_s: float = 1.0,
                 design: Optional[Design] = None, goal_seed: int = POSE_SEED + 1, lead: bool = False, log=print,
                 **options: Any) -> Dict[str, Any]:
    """The arm keeps moving from goal to goal; every tick is logged.  ``lead``: the encoders write ahead (Session)."""

    started = time.time()
    chain, design, network, session = build(backend, chain_name, design, max_speed=speed, lead=lead, **options)
    session.warm_up()
    sequence = GoalSequence(chain, goal_seed, dwell_s)
    ticks: List[Dict[str, Any]] = []
    reached = 0
    while reached <= int(goals):
        if sequence.update(session.robot, session.dt_s):
            reached += 1
            if reached > int(goals):
                break
        tick = session.tick()
        q = session.robot.q.copy()
        true = chain.rotation(q)
        lpy = lift_pitch_yaw(true)
        ticks.append({
            "t": tick.t_ms / 1000.0, "q": q.tolist(), "moving": bool(not session.robot.arrived()), "goal": reached,
            "joint_speed": float(np.max(np.abs(session.robot.velocity))),
            "error_deg": orientation_error_deg(tick.tool, true),
            "true_lift_pitch_yaw": list(lpy), "ring_lift_pitch_yaw": tick.angles.tolist(),
            "matrix_lift_pitch_yaw": list(lift_pitch_yaw(tick.tool_rotation())),
            "tool": tick.tool.tolist(), "cos_pitch": float(np.cos(lpy[1])),
            "joint_ring_error_index": tick.ring_error.tolist(),
        })
    document = _meta(chain, design, network, started, network.t_ms / 1000.0)
    document.update(experiment="tracking", speed=float(speed), goals=int(goals), dwell_s=float(dwell_s),
                    goal_seed=int(goal_seed), lead=bool(lead), lead_s=session.lead_s.tolist() if lead else None, ticks=ticks)
    if log is not None:
        errors = np.array([t["error_deg"] for t in ticks])
        moving = np.array([t["moving"] for t in ticks])
        log("%s at %.2f rad/s: %d ticks, error median %.2f deg while moving, %.2f deg at rest" % (
            chain_name, speed, len(ticks), np.median(errors[moving]) if moving.any() else float("nan"),
            np.median(errors[~moving]) if (~moving).any() else float("nan")))
    return document


__all__ = ["POSE_SEED", "build", "legacy_sweep_poses", "make_design", "static_run", "sweep_poses", "test_poses", "tracking_run", "write_json"]

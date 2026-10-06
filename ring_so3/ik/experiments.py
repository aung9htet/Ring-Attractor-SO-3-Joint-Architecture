"""The measurements of the inverse-kinematics layer in NEST, each returning a JSON-ready dict."""

from __future__ import annotations

import os
import time
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from ..chains import get_chain
from ..design import Design
from ..kinematics import Chain, lift_pitch_yaw
from . import rules
from .design import IkDesign
from .pairs import TEST_SEED, pairs

MOVE_SPEED = 3.0          # rad/s: how fast arm and goal are moved between two open-loop measurements
HOLD_TICKS = 20
MEASURE_TICKS = 8


def build_ik(backend: Any, chain_name: str, ik_design: Optional[IkDesign] = None, design: Optional[Design] = None,
             goal_ring_settings: Any = None, **options: Any):
    from ..nest import calibrate, ik_calibrate
    from ..nest.ik_network import IkNetwork
    from ..nest.network import NetworkOptions

    chain = get_chain(chain_name)
    design = design or Design()
    network_options = NetworkOptions(**options)
    calibration = calibrate.load(design, design.sizes.ring, network_options.encoder)
    ik_cal = ik_calibrate.load(design, design.sizes.ring, network_options.encoder)
    return chain, IkNetwork(backend, chain, calibration, ik_cal, design, network_options, ik_design or IkDesign(), goal_ring_settings)


def meta(chain: Chain, network: Any, started: float) -> Dict[str, Any]:
    o = network.options
    wall, sim_s = time.time() - started, network.t_ms / 1000.0
    return {"chain": chain.name, "joints": chain.n, "design": network.design.describe(), "ik_design": network.ik_design.describe(),
            "options": {"encoder": o.encoder, "threads": o.threads, "seed": o.seed, "tick_ms": o.tick_ms,
                        "readout_tau_ms": o.readout_tau_ms, "noise_pa": o.noise_pa},
            "neurons": network.neuron_count(), "ik_wiring": network.ik_wiring.summary(),
            "synapses": int(network.backend.GetKernelStatus("num_connections")),
            "simulated_s": round(sim_s, 2), "wall_s": round(wall, 1), "speed": round(sim_s / wall, 3) if wall > 0 else None,
            "ik_calibration_fit": network.ik_calibration.fit}


def _toward(current: np.ndarray, target: np.ndarray, step: float, circular: Optional[np.ndarray] = None) -> np.ndarray:
    difference = target - current
    if circular is not None:
        difference = np.where(circular, rules.wrap(difference), difference)
    return current + np.clip(difference, -step, step)


def angle_between_deg(a: Sequence[float], b: Sequence[float]) -> float:
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    norm = float(np.linalg.norm(a) * np.linalg.norm(b))
    return float(np.degrees(np.arccos(np.clip(a @ b / norm, -1.0, 1.0)))) if norm > 0 else float("nan")


def open_loop_run(backend: Any, chain_name: str, count: int = 100, ik_design: Optional[IkDesign] = None,
                  pair_seed: int = TEST_SEED, fraction: Sequence[float] = (0.02, 0.25), log=print, **options: Any) -> Dict[str, Any]:
    """Tests 3a and 3b: the arm is held at a pose, a goal part of the way to another pose is written,
    and comparison and shares are read and compared with the exact ones."""

    started = time.time()
    chain, network = build_ik(backend, chain_name, ik_design, **options)
    design = network.ik_design
    dt = network.options.tick_ms / 1000.0
    circular = np.array([j.circular for j in chain.joints])
    rng = np.random.default_rng([int(pair_seed), 99])
    q = np.zeros(chain.n) if chain.joints[0].circular else chain.limits().mean(axis=1)
    goal = np.array(lift_pitch_yaw(chain.rotation(q)))
    for _ in range(20):
        network.step_ik(q, goal)
    rows: List[Dict[str, Any]] = []
    for pair in pairs(chain, count, pair_seed):
        f = float(rng.uniform(*fraction))
        step_to = np.where(circular, rules.wrap(pair.goal_pose - pair.start), pair.goal_pose - pair.start)
        target_goal = np.array(lift_pitch_yaw(chain.rotation(pair.start + f * step_to)))
        moved = 0
        while moved < 400 and (np.max(np.abs(np.where(circular, rules.wrap(pair.start - q), pair.start - q))) > 1e-9
                               or np.max(np.abs(rules.wrap(target_goal - goal))) > 1e-9):
            q = _toward(q, pair.start, MOVE_SPEED * dt, circular)
            goal = rules.travel(goal, target_goal, MOVE_SPEED * dt)
            network.step_ik(q, goal)
            moved += 1
        ticks = [network.step_ik(q, goal) for _ in range(HOLD_TICKS)][-MEASURE_TICKS:]
        turn = np.mean([t.turn for t in ticks], axis=0)
        shares = np.mean([t.shares for t in ticks], axis=0)
        rests = np.mean([t.rests for t in ticks], axis=0)
        true_turn = rules.turn(chain.rotation(q), goal, design.comparison)
        wired_turn, wired_shares, wired_left = network.exact(q, goal)
        rows.append({
            "index": pair.index, "q": q.tolist(), "goal": goal.tolist(), "fraction": f,
            "turn": turn.tolist(), "true_turn": true_turn.tolist(), "wired_turn": wired_turn.tolist(),
            "turn_direction_error_deg": angle_between_deg(turn, true_turn),
            "turn_sd": np.std([t.turn for t in ticks], axis=0).tolist(),
            "shares": shares.tolist(), "wired_shares": wired_shares.tolist(),
            "share_sd": np.std([t.shares for t in ticks], axis=0).tolist(),
            "caps": network.ik_wiring.caps.tolist(),
            "rests": rests.tolist(), "wired_left": wired_left.tolist(),
            "room": rules.limit_room(chain, q, design, int(network.calibration.settings["ring_size"])).tolist(),
        })
        if log is not None and len(rows) % 20 == 0:
            log("%s: %d pairs, comparison direction error median %.1f deg, %.0f s" % (
                chain_name, len(rows), np.median([r["turn_direction_error_deg"] for r in rows]), time.time() - started))
    document = meta(chain, network, started)
    document.update(experiment="ik_open_loop", pair_seed=int(pair_seed), fraction=list(fraction), rows=rows,
                    summary=open_loop_summary(rows))
    return document


def exact_trace(chain: Chain, pair: Any, design: IkDesign, **options: Any) -> rules.Trace:
    """The exact rule on the same pair, with the delay, noise and dead zone the spiking network has about."""

    settings = dict(delay_s=0.2, noise=0.005, dead_zone=0.01, turn_clip=0.5, rng=np.random.default_rng(int(pair.index)))
    settings.update(options)
    return rules.simulate(chain, pair.start, pair.goal, design, **settings)


def reach_run(backend: Any, chain_name: str, count: int = 100, ik_design: Optional[IkDesign] = None, pair_seed: int = TEST_SEED,
              shard: Sequence[int] = (0, 1), max_minutes: Optional[float] = None, motor_scale: float = 1.0,
              push: Optional[Sequence[float]] = None, goal_change_s: Optional[float] = None, goal_ring_settings: Any = None,
              silence: float = 0.0, steep_deg: float = 0.0, only: Optional[Sequence[int]] = None, log=print,
              make_robot: Any = None, method: str = "ring", checkpoint: Optional[str] = None,
              **options: Any) -> Dict[str, Any]:
    """Tests 4 to 7: closed-loop trials on start and goal pairs.  ``shard``: ``(i, n)`` runs every n-th pair
    from the i-th on.  ``max_minutes``: stop before the next trial when this much time has passed (the
    pairs that were not run are listed).  ``push``: ``(time, angle)``: one joint, chosen per pair, is
    displaced.  ``goal_change_s``: at that time the goal becomes the next pair's goal.
    ``make_robot(chain)``: another robot than the light one (Gazebo).  ``method``: ``ring`` (each joint follows
    its goal ring) or one of :data:`ring_so3.standard.targets.EXACT_METHODS` (the network runs, the joints follow the method).
    ``checkpoint``: a file that is written again after every trial, with the pairs still to run listed as not
    run, so that a run that is stopped from outside loses at most the trial it was in."""

    from .session import IkSession, trial_row

    started = time.time()
    chain, network = build_ik(backend, chain_name, ik_design, goal_ring_settings=goal_ring_settings, **options)
    design = network.ik_design
    dead = network.silence_ik(silence, seed=3) if silence > 0.0 else 0
    session = IkSession(network, motor_scale=motor_scale, robot=make_robot(chain) if make_robot is not None else None)
    if method != "ring":
        from ..standard.targets import exact_targets

        session.method = exact_targets(method)
        session.method_shares = None
    session.warm_up()
    start_base = session.robot.link.base_pose() if hasattr(session.robot, "link") else None
    every = pairs(chain, count, pair_seed, steep_deg)
    chosen = [pair for pair in every if pair.index % int(shard[1]) == int(shard[0])]
    if only is not None:                      # the pairs an earlier run did not get to
        chosen = [pair for pair in every if pair.index in set(int(i) for i in only)]
    rng = np.random.default_rng([int(pair_seed), 77])
    pushed = {pair.index: int(rng.integers(chain.n)) for pair in every}
    rows, skipped = [], []

    def document_now(not_run: Sequence[int]) -> Dict[str, Any]:
        out = meta(chain, network, started)
        out.update(experiment="ik_reach", pair_seed=int(pair_seed), pairs=int(count), shard=[int(shard[0]), int(shard[1])],
                   motor_scale=float(motor_scale), push=list(push) if push is not None else None, goal_change_s=goal_change_s,
                   goal_ring_settings=network.joint_goal_rings[0].settings.describe() if network.joint_goal_rings else None,
                   silence=float(silence), silenced_neurons=int(dead), steep_deg=float(steep_deg),
                   method=method, robot=session.robot.describe(), base_at_start=list(start_base) if start_base else None,
                   skipped=list(not_run), rows=rows, summary=reach_summary(rows))
        return out

    for position, pair in enumerate(chosen):
        if max_minutes is not None and (time.time() - started) / 60.0 > float(max_minutes):
            skipped.append(pair.index)
            continue
        if hasattr(session.robot, "link"):
            session.robot.link.put_base_back()
        session.place(pair.start)
        push_now = (float(push[0]), pushed[pair.index], float(push[1])) if push is not None else None
        change = (float(goal_change_s), every[(pair.index + 1) % len(every)].goal) if goal_change_s is not None else None
        trial = session.reach(pair.goal, push=push_now, goal_change=change)
        row = trial_row(trial, design.reach_deg)
        exact = exact_trace(chain, pair, design, motor_scale=motor_scale, push=push_now, goal_change=change)
        row.update(index=pair.index, start=pair.start.tolist(), goal=(change[1] if change else pair.goal).tolist()
                   if hasattr(change[1] if change else pair.goal, "tolist") else list(change[1] if change else pair.goal),
                   exact_final_error_deg=exact.final_error_deg, exact_reached=bool(exact.final_error_deg <= design.reach_deg),
                   exact_time_to_5_deg=exact.time_to(5.0),
                   exact_path_ratio=exact.path_deg / exact.start_error_deg if exact.start_error_deg > 1e-9 else float("nan"),
                   exact_at_limit=bool(exact.at_limit), pushed_joint=pushed[pair.index] if push is not None else None,
                   start_off_rad=float(np.max(np.abs(rules.wrap(trial.q[0] - pair.start)))),     # how far from the start pose the trial began
                   base=list(session.robot.link.base_pose() or ()) if hasattr(session.robot, "link") else None)
        rows.append(row)
        if log is not None:
            log("%s pair %d: from %.0f deg to %.1f deg (%s), 5 deg after %.1f s, path %.2f, %.1f s; exact rule %.1f deg | %d done, %.0f%% reached, %.0f s" % (
                chain_name, pair.index, row["start_error_deg"], row["final_error_deg"], "at a limit" if row["at_limit"] else "free",
                row["time_to_5_deg"], row["path_ratio"], row["duration_s"], row["exact_final_error_deg"], len(rows),
                100 * np.mean([r["reached"] for r in rows]), time.time() - started))
        if checkpoint:
            from ..experiments import write_json

            write_json(checkpoint + ".part", document_now(skipped + [later.index for later in chosen[position + 1:]]))
            os.replace(checkpoint + ".part", checkpoint)
    return document_now(skipped)


def _median(values: Sequence[float]) -> Optional[float]:
    finite = [float(v) for v in values if v is not None and np.isfinite(v)]
    return float(np.median(finite)) if finite else None


def reach_summary(rows: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    if not rows:
        return {"trials": 0}
    reached = [r for r in rows if r["reached"]]
    exact = [r for r in rows if r["exact_reached"]]
    return {
        "trials": len(rows),
        "reached": float(np.mean([r["reached"] for r in rows])),
        "exact_reached": float(np.mean([r["exact_reached"] for r in rows])),
        "reached_of_exact_reached": float(np.mean([r["reached"] for r in exact])) if exact else None,
        "final_error_reached_median_deg": _median([r["final_error_deg"] for r in reached]),
        "final_error_median_deg": _median([r["final_error_deg"] for r in rows]),
        "time_to_5_deg_median_s": _median([r["time_to_5_deg"] for r in rows]),
        "path_ratio_median": _median([r["path_ratio"] for r in reached]),
        "exact_time_to_5_deg_median_s": _median([r["exact_time_to_5_deg"] for r in rows]),
        "exact_path_ratio_median": _median([r["exact_path_ratio"] for r in exact]),
        "missed": int(len(rows) - len(reached)),
        "missed_at_limit": int(sum(1 for r in rows if not r["reached"] and r["at_limit"])),
        "settled": float(np.mean([r["settled_s"] is not None for r in rows])),
        "duration_median_s": _median([r["duration_s"] for r in rows]),
        "decoded_error_median_deg": _median([r["decoded_error_median_deg"] for r in rows]),
    }


def open_loop_summary(rows: Sequence[Dict[str, Any]], share_floor: float = 0.05) -> Dict[str, Any]:
    turn, true, wired = [np.array([r[key] for r in rows]) for key in ("turn", "true_turn", "wired_turn")]
    shares, wired_shares = np.array([r["shares"] for r in rows]), np.array([r["wired_shares"] for r in rows])
    direction = np.array([r["turn_direction_error_deg"] for r in rows])
    large = np.abs(wired_shares) > share_floor
    n = shares.shape[1]
    per_joint = []
    for k in range(n):
        mask = large[:, k]
        per_joint.append({
            "joint": k + 1, "large": int(mask.sum()),
            "right_sign": float(np.mean(np.sign(shares[mask, k]) == np.sign(wired_shares[mask, k]))) if mask.any() else None,
            "rms_rad": float(np.sqrt(np.mean((shares[:, k] - wired_shares[:, k]) ** 2))),
            "sd_rad": float(np.mean([r["share_sd"][k] for r in rows])),
        })
    quiet = np.abs(wired_shares) < 0.01
    return {
        "pairs": len(rows),
        "turn_size_median_rad": float(np.median(np.linalg.norm(true, axis=1))),
        "turn_direction_error_median_deg": float(np.median(direction)),
        "turn_direction_error_p90_deg": float(np.percentile(direction, 90)),
        "turn_rms_rad": float(np.sqrt(np.mean((turn - wired) ** 2))),
        "turn_sd_rad": float(np.mean([r["turn_sd"] for r in rows])),
        "share_floor_rad": share_floor,
        "shares_large": int(large.sum()),
        "share_right_sign": float(np.mean(np.sign(shares[large]) == np.sign(wired_shares[large]))) if large.any() else None,
        "share_rms_rad": float(np.sqrt(np.mean((shares - wired_shares) ** 2))),
        "share_when_none_asked_rms_rad": float(np.sqrt(np.mean(shares[quiet] ** 2))) if quiet.any() else None,
        "left_rms_rad": float(np.sqrt(np.mean((np.array([r["rests"][-1] for r in rows]) - np.array([r["wired_left"] for r in rows])) ** 2))),
        "per_joint": per_joint,
    }


__all__ = ["angle_between_deg", "build_ik", "exact_trace", "meta", "open_loop_run", "open_loop_summary", "reach_run", "reach_summary"]

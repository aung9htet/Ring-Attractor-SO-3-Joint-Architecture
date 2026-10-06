"""Test 3c: one joint's goal ring on its own.  Several rigs side by side in one kernel, one per share."""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from ..design import Design
from ..mapping import RingMap
from .calibrate import Calibration, _kernel
from .goal_ring import GoalRingSettings, WrittenRingSettings, bump_spread, build_goal_ring, build_written_ring
from .ik_calibrate import IkCalibration
from .populations import connect_weights, create_population, decoded_weights, set_value
from .ring import build_joint_ring

TICK_MS = 50.0


def measure_lead(backend: Any, base: Calibration, ik: IkCalibration, shares: Sequence[float], settings: Any = None,
                 design: Optional[Design] = None, position: float = 100.37, seconds: float = 4.0, threads: int = 8, seed: int = 1,
                 mapping: Optional[RingMap] = None) -> List[Dict[str, Any]]:
    """Hold the state bump at ``position`` and a share (in caps, -1 .. 1) on the halves; follow the goal bump.

    Returns one dict per share: the lead per tick (goal index minus state index), gain spikes, spread.
    """

    design = design or Design()
    _kernel(backend, threads, seed)
    mapping = mapping or RingMap(int(base.settings["ring_size"]), -np.pi, np.pi, True)
    one = np.array([[1.0]])
    rigs = []
    for i, share in enumerate(shares):
        state = build_joint_ring(backend, "state%d" % i, mapping, mode="fractional")
        source = create_population(backend, design.entry, "source%d" % i, seed=300 + i)
        set_value(backend, source, [abs(float(share))])
        zero = create_population(backend, design.entry, "zero%d" % i, seed=400 + i)
        set_value(backend, zero, [0.0])
        halves = []
        for h, feeder in enumerate((source, zero) if share >= 0 else (zero, source)):
            half = create_population(backend, design.entry, "half%d_%d" % (i, h), seed=500 + 2 * i + h)
            connect_weights(backend, feeder.neurons, half.neurons, decoded_weights(base.decoders["source"], one, half))
            halves.append((half, ik.decoders["half"]))
        build = build_written_ring if isinstance(settings, WrittenRingSettings) else build_goal_ring
        goal = build(backend, "goal%d" % i, mapping, state, halves[0], halves[1], settings)
        rigs.append((state, goal))
    angle = mapping.angle(position)
    rows = [{"share": float(share), "lead": [], "gain": [], "spread": [], "rewritten": 0} for share in shares]
    for _ in range(int(round(seconds * 1000.0 / TICK_MS))):
        for (state, goal), row in zip(rigs, rows):
            state.drive(backend, angle)
            if goal.ring.generators is not None:           # an attractor ring is written again when it has lost the state bump
                row["rewritten"] += int(goal.ring.drive(backend, angle) and np.isfinite(goal.ring.index))
        backend.Simulate(TICK_MS)
        for (state, goal), row in zip(rigs, rows):
            state.read(backend)
            counts = goal.ring.read(backend)
            both = np.isfinite(goal.ring.index) and np.isfinite(state.index)
            row["lead"].append(mapping.index_error(goal.ring.index, state.index) if both else float("nan"))
            row["gain"].append(goal.gain_counts(backend).tolist())
            row["spread"].append(bump_spread(counts, goal.ring.index))
    return rows


def summarise(rows: Sequence[Dict[str, Any]], settle_s: float = 2.0) -> List[Dict[str, Any]]:
    skip = int(round(settle_s * 1000.0 / TICK_MS))
    out = []
    for row in rows:
        lead = np.array(row["lead"][skip:], dtype=float)
        gain = np.array(row["gain"][skip:], dtype=float)
        spread = np.array(row["spread"], dtype=float)
        out.append({"share": row["share"], "lead_mean": float(np.nanmean(lead)), "lead_sd": float(np.nanstd(lead)),
                    "silent_ticks": int(np.sum(~np.isfinite(lead))), "gain_hz": (gain.mean(axis=0) / (TICK_MS / 1000.0)).tolist(),
                    "spread_max": float(np.nanmax(spread[4:])) if np.any(np.isfinite(spread[4:])) else float("nan"),
                    "rewritten": int(row["rewritten"])})
    return out


#: the shares of test 3c, in caps; 0.08 caps is 0.02 rad on a ring that covers the whole circle
TEST_SHARES = (-1.0, -0.5, -0.2, -0.08, 0.0, 0.08, 0.2, 0.5, 1.0)
SMALL_SHARE = 0.08


def judge(rows: Sequence[Dict[str, Any]], settle_s: float = 1.0, window_s: float = 0.5) -> Dict[str, Any]:
    """The marks of test 3c from one run of :func:`measure_lead`.  A trial is the lead averaged over
    ``window_s`` (the joint follows its goal ring slowly, so it sees the lead averaged over about that long)."""

    skip, width = int(round(settle_s * 1000.0 / TICK_MS)), int(round(window_s * 1000.0 / TICK_MS))
    by_share, windows = {}, {}
    for row in rows:
        lead = np.array(row["lead"][skip:], dtype=float)
        count = lead.size // width
        windows[row["share"]] = np.array([np.nanmean(lead[i * width:(i + 1) * width]) for i in range(count)])
        by_share[row["share"]] = float(np.nanmean(lead))
    shares = sorted(by_share)
    means = np.array([by_share[s] for s in shares])
    small = [s for s in shares if abs(abs(s) - SMALL_SHARE) < 1e-9]
    right = np.concatenate([np.sign(windows[s]) == np.sign(s) for s in small]) if small else np.array([])
    spread = max(float(np.nanmax(np.array(row["spread"][4:], dtype=float))) for row in rows)
    silent = int(sum(np.sum(~np.isfinite(np.array(row["lead"][4:], dtype=float))) for row in rows))
    rewritten = int(sum(row["rewritten"] for row in rows))
    out = {
        "lead_without_share": by_share.get(0.0, float("nan")),
        "lead_without_share_sd": float(np.nanstd(windows[0.0])) if 0.0 in windows else float("nan"),
        "lead_at_full_share": float(min(abs(by_share[shares[0]]), abs(by_share[shares[-1]]))),
        "full_share_right_sign": bool(by_share[shares[0]] < 0 < by_share[shares[-1]]),
        "rising": bool(np.all(np.diff(means) > -0.5)),
        "small_share_right_sign": float(np.mean(right)) if right.size else float("nan"),
        "spread_max": spread, "silent_ticks": silent, "rewritten": rewritten,
        "lead_by_share": {("%+.2f" % s): by_share[s] for s in shares},
    }
    out["passed"] = bool(abs(out["lead_without_share"]) < 1.0 and out["lead_at_full_share"] >= 6.0 and out["full_share_right_sign"]
                         and out["rising"] and out["small_share_right_sign"] >= 0.9 and spread < 0.2 and silent == 0 and rewritten == 0)
    return out


__all__ = ["SMALL_SHARE", "TEST_SHARES", "judge", "measure_lead", "summarise"]

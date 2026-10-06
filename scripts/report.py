#!/usr/bin/env python3
"""Tables and figures of docs/results.md from the data in docs/results/data/.

    python3 -B scripts/report.py                       # writes docs/results/tables.md and docs/results/figures/
    python3 -B scripts/report.py --no-figures

Nothing here simulates; it only reads what scripts/run_experiments.py wrote.
Every number quoted in README.md and docs/results.md comes from the tables
written here (scripts/check_doc_numbers.py checks that).
"""

import argparse
import glob
import json
import os
import sys
from fractions import Fraction

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.chains import MODEL_NAMES, get_chain  # noqa: E402
from ring_so3.kinematics import CHANCE_ERROR_DEG, nearest_rotation, rotation_angle  # noqa: E402
from ring_so3.wiring import chain_wiring, flat_wiring  # noqa: E402

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
PASS_MEDIAN, PASS_P90 = 5.0, 10.0
INK, GREY, LIGHT = "#1c2330", "#8a94a3", "#d9dee6"
FAMILY_COLORS = {"generic": "#2a78d6", "tiago": "#1baf7a"}
NEW, OLD, ERROR = "#2a78d6", "#eb6834", "#eb6834"
VARIANT_LABELS = {
    "default": "as built", "silence05": "5% of product neurons off", "silence10": "10% of product neurons off",
    "silence20": "20% of product neurons off", "noise050": "noise current 50 pA", "noise150": "noise current 150 pA",
    "ring100": "joint rings of 100 neurons", "seed2": "other neurons (seed 2)", "seed3": "other neurons (seed 3)",
    "nearest": "encoder writes on the nearest index", "corrective": "v4's corrective encoder",
    "direct": "no link populations in the path", "product400": "product 400, link 100 neurons",
}


# -- reading -------------------------------------------------------------------
def read(path):
    with open(path, encoding="utf-8") as stream:
        return json.load(stream)


def load(data):
    out = {"static": {}, "cross": {}, "tracking": {}, "robust": {}, "old": None, "rate": None}
    for path in sorted(glob.glob(os.path.join(data, "static", "*.json"))):
        out["static"][os.path.basename(path)[:-5]] = read(path)
    for path in sorted(glob.glob(os.path.join(data, "cross", "*_flat.json"))):
        out["cross"][os.path.basename(path)[:-10]] = read(path)
    for path in sorted(glob.glob(os.path.join(data, "tracking", "*.json"))):
        model, speed = os.path.basename(path)[:-5].rsplit("_", 1)
        out["tracking"][(model, float(speed))] = read(path)
    out["tracking_variants"] = {}
    for path in sorted(glob.glob(os.path.join(data, "tracking_variants", "*.json"))):
        out["tracking_variants"][os.path.basename(path)[:-5].split("_", 1)[1]] = read(path)
    for path in sorted(glob.glob(os.path.join(data, "robust", "*.json"))):
        model, variant = os.path.basename(path)[:-5].split("_", 1)
        out["robust"][(model, variant)] = read(path)
    old_rows = []
    for path in sorted(glob.glob(os.path.join(data, "old_decoder", "*.json"))):
        old_rows.extend(read(path)["rows"])
    out["old"] = old_rows or None
    rate = os.path.join(data, "rate_study.json")
    out["rate"] = read(rate) if os.path.exists(rate) else None
    speed = os.path.join(data, "speed.json")
    out["speed"] = read(speed) if os.path.exists(speed) else None
    parts = os.path.join(data, "parts.json")
    out["parts"] = read(parts) if os.path.exists(parts) else None
    return out


def rows_of(document, kind="random"):
    return [r for r in document["rows"] if r.get("kind") == kind]


# -- statistics ----------------------------------------------------------------
def stats(values):
    a = np.asarray([v for v in values if v is not None and np.isfinite(v)], dtype=float)
    if a.size == 0:
        return float("nan"), float("nan"), float("nan")
    return float(np.median(a)), float(np.percentile(a, 90)), float(np.max(a))


def median_interval(values, seed=0, draws=2000):
    a = np.asarray([v for v in values if v is not None and np.isfinite(v)], dtype=float)
    if a.size < 2:
        return float("nan"), float("nan")
    rng = np.random.default_rng(seed)
    medians = np.median(a[rng.integers(0, a.size, (draws, a.size))], axis=1)
    return float(np.percentile(medians, 2.5)), float(np.percentile(medians, 97.5))


def sign_test(differences):
    """Two-sided exact sign test on paired differences (zeros dropped)."""

    d = [v for v in differences if v != 0 and np.isfinite(v)]
    n, k = len(d), sum(1 for v in d if v < 0)
    if n == 0:
        return 1.0
    tail = sum(Fraction(_binomial(n, i), 2 ** n) for i in range(0, min(k, n - k) + 1))
    return float(min(Fraction(1), 2 * tail))


def _binomial(n, k):
    out = 1
    for i in range(1, k + 1):
        out = out * (n - k + i) // i
    return out


def fmt(value, digits=2):
    return "-" if value is None or not np.isfinite(value) else "%.*f" % (digits, value)


def triple(values, digits=2):
    return " / ".join(fmt(v, digits) for v in stats(values))


def p_text(p):
    return "under 0.001" if p < 0.001 else "%.3f" % p


def paired(a_rows, b_rows, key="error_deg"):
    """Rows of two runs matched by pose index: (a, b) arrays."""

    b_by_index = {(r.get("kind"), r["index"]): r for r in b_rows}
    pairs = [(r[key], b_by_index[(r.get("kind"), r["index"])][key]) for r in a_rows if (r.get("kind"), r["index"]) in b_by_index]
    pairs = [(x, y) for x, y in pairs if x is not None and y is not None]
    return np.array([p[0] for p in pairs]), np.array([p[1] for p in pairs])


def paired_line(label, a, b):
    """``a``: this repo's readout, ``b``: the other; negative difference = ``a`` is closer."""

    d = a - b
    return "| %s | %d | %s | %s | %+.2f | %.0f%% | %.0f%% | %s |" % (
        label, d.size, triple(a), triple(b), float(np.median(d)), 100.0 * np.mean(d < 0), 100.0 * np.mean(d > 0), p_text(sign_test(d)))


# -- tracking --------------------------------------------------------------------
def tracking_summary(document):
    """Delay and error of one tracking run."""

    chain = get_chain(document["chain"])
    ticks = document["ticks"]
    dt = 0.05
    true = np.array([chain.rotation(t["q"]) for t in ticks])
    decoded = np.array([nearest_rotation(np.array(t["tool"])) for t in ticks])
    moving = np.array([t["moving"] for t in ticks])
    error = np.array([t["error_deg"] for t in ticks])

    def lagged(lag):
        index = np.flatnonzero(moving)
        index = index[index >= lag]
        return np.median([np.degrees(rotation_angle(decoded[i], true[i - lag])) for i in index]) if index.size else float("nan")

    lags = list(range(0, 21))
    medians = [lagged(lag) for lag in lags]
    best = int(np.nanargmin(medians))
    delay = best * dt
    if 0 < best < len(lags) - 1:                          # refine between ticks with a parabola through the three lowest
        y0, y1, y2 = medians[best - 1], medians[best], medians[best + 1]
        denominator = y0 - 2 * y1 + y2
        if denominator > 1e-9:
            delay = (best + 0.5 * (y0 - y2) / denominator) * dt
    rest = error[~moving]
    # the arm rests 1 s (20 ticks) at every goal: the reading at the end of that rest is "1 s after the stop"
    stops = [i for i in range(1, len(ticks)) if moving[i - 1] and not moving[i]]
    after = [error[i + 19] for i in stops if i + 19 < len(ticks) and not moving[i:i + 20].any()]
    return {
        "ticks": len(ticks), "moving_share": float(np.mean(moving)), "delay_s": float(delay),
        "error_moving": stats(error[moving]), "error_rest": stats(rest),
        "error_after_delay": float(medians[best]),
        "error_1s_after_stop": stats(after), "stops": len(after),
    }


# -- tables ----------------------------------------------------------------------
def table_parts(data):
    p = data.get("parts")
    if not p:
        return ["No data of the parts."]
    fit, f, st = p["calibration_fit"], p["feature"], p["stage"]
    out = ["| Part | Measured on | Result | Pass mark | Met |", "| --- | --- | --- | --- | --- |"]

    def row(part, on, result, mark, met):
        out.append("| %s | %s | %s | %s | %s |" % (part, on, result, mark, "yes" if met else "NO"))

    row("cos / sin population", "%d ring positions between indices" % f["positions"],
        "largest error of cos or sin %.3f; angle of the pair off by %.2f deg (median) / %.2f deg (largest); length off by at most %.3f"
        % (f["max"], f["angle_error_deg_median"], f["angle_error_deg_max"], f["length_error_max"]), "within 0.03", f["max"] <= 0.03)
    row("product population", "%d random pairs" % p["product"]["points"],
        "root mean square error %.4f, largest %.3f" % (p["product"]["rms"], p["product"]["max"]), "rms at most 0.02", p["product"]["rms"] <= 0.02)
    row("link-orientation population", "%d random values" % p["scalar"]["points"],
        "root mean square error %.4f, largest %.3f" % (p["scalar"]["rms"], p["scalar"]["max"]), "(none set)", True)
    row("length population (pitch ring)", "%d random pairs in the unit disc" % p["norm"]["points"],
        "root mean square error %.4f, largest %.3f" % (p["norm"]["rms"], p["norm"]["max"]), "(none set)", True)
    row("one stage: direction", "%d vectors longer than 0.3, two-joint chain" % st["vectors_longer_than_0.3"],
        "direction off by %.2f deg (median) / %.2f deg (largest)" % (st["direction_error_deg_median"], st["direction_error_deg_max"]),
        "within 2 deg", st["direction_error_deg_max"] <= 2.0)
    row("one stage: length", "the same vectors",
        "length off by at most %.3f; as a share of the vector's own length at most %.1f%%" % (st["length_error_abs_max"], st["length_error_relative_max_percent"]),
        "within 3% of its length", st["length_error_relative_max_percent"] <= 3.0)
    out += ["", "The joint ring itself (from the calibration run): a held bump has %d active neurons and %.0f spikes per second in all; "
            "the centre of its activity is %+.2f ring indices from the written position on average, %.2f from tick to tick (standard deviation)."
            % (fit["ring_active_neurons"], fit["ring_total_rate_hz"], fit["ring_index_error_mean"], fit["ring_index_error_sd"]),
            "", "Fit error of the readouts on their own calibration points (root mean square): link population %.4f, product %.4f, length %.4f, "
            "cos / sin %.4f per 50 ms tick." % (fit["scalar_rms"], fit["product_rms"], fit["norm_rms"], fit["feature_rms"])]
    if p.get("ring_lag"):
        out += ["", "A joint ring's bump behind an angle that moves at a steady speed (one ring, encoder at its usual rate and four times as strong):", "",
                "| Encoder rate (Hz) | Speed (ring indices per tick) | Bump behind by (indices) | The same in time (s) |", "| ---: | ---: | ---: | ---: |"]
        for row in p["ring_lag"]:
            out.append("| %.0f | %.1f | %.2f | %.3f |" % (row["rate_hz"], row["speed_index_per_tick"], row["lag_index"], row["lag_s"]))
    return out


def table_models(data):
    speed = {(r["model"], r["threads"]): r for r in (data.get("speed") or {}).get("rows", [])}
    out = ["| Model | Joints | Products | Link populations | Neurons | Synapses | Speed with 4 threads | Speed with 8 threads |",
           "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for name in MODEL_NAMES:
        d = data["static"].get(name)
        if d:
            cells = ["%.2f" % speed[(name, t)]["speed"] if (name, t) in speed else "-" for t in (4, 8)]
            out.append("| %s | %d | %d | %d | %s | %s | %s | %s |" % (name, d["joints"], d["products"], d["link_populations"],
                                                                    format(d["neurons"], ","), format(d["synapses"], ","), cells[0], cells[1]))
    if data.get("speed"):
        out += ["", "Speed: simulated time per wall-clock time with the arm moving, measured alone on the machine (%s, %d logical cores; "
                "`scripts/benchmark.py`). 1.00 means the network keeps up with real time." % (data["speed"]["cpu"], data["speed"]["cores"])]
    return out


def table_cost():
    out = ["| Joints | Possible product features (3^n - 1) | Features that occur | Pairwise products, all at once | Pairwise products, joint by joint | Cells of the old decoder's design (16 x 32^n) |",
           "| ---: | ---: | ---: | ---: | ---: | ---: |"]
    from ring_so3.kinematics import used_products

    for n in range(2, 8):
        chain = get_chain("generic%d" % n)
        out.append("| %d | %s | %d | %d | %d | %s |" % (n, format(3 ** n - 1, ","), len(used_products(chain)), len(flat_wiring(chain).products),
                                                        len(chain_wiring(chain).products), format(16 * 32 ** n, ",") if n <= 4 else "%.1e" % (16 * 32.0 ** n)))
    return out


def table_static(data):
    out = ["| Model | Poses | Error, median / p90 / max (deg) | 95% interval of the median | Within 5 deg | Within 10 deg | Similar (median <= 5, p90 <= 10) | Rings alone, median / p90 (deg) | Not arrived |",
           "| --- | ---: | --- | --- | ---: | ---: | --- | --- | ---: |"]
    for name in MODEL_NAMES:
        d = data["static"].get(name)
        if not d:
            continue
        rows = rows_of(d)
        e = np.array([r["error_deg"] for r in rows])
        lo, hi = median_interval(e)
        med, p90, _ = stats(e)
        held = [r["held_error_deg"] for r in rows]
        out.append("| %s | %d | %s | %s - %s | %.0f%% | %.0f%% | %s | %s / %s | %d |" % (
            name, e.size, triple(e), fmt(lo), fmt(hi), 100 * np.mean(e <= 5.0), 100 * np.mean(e <= 10.0),
            "yes" if med <= PASS_MEDIAN and p90 <= PASS_P90 else "NO", fmt(stats(held)[0]), fmt(stats(held)[1]),
            sum(1 for r in rows if not r["arrived"])))
    return out


def table_links(data):
    out = ["| Model | Link 1 | 2 | 3 | 4 | 5 | 6 | 7 | End effector |", "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for name in MODEL_NAMES:
        d = data["static"].get(name)
        if not d:
            continue
        rows = rows_of(d)
        per_link = np.array([r["link_error_deg"] for r in rows])
        cells = [fmt(v) for v in np.median(per_link, axis=0)] + ["-"] * (7 - per_link.shape[1])
        out.append("| %s | %s | %s |" % (name, " | ".join(cells), fmt(np.median([r["error_deg"] for r in rows]))))
    growth = []
    for name in ("generic7", "tiago7"):
        d = data["static"].get(name)
        if d:
            med = np.median(np.array([r["link_error_deg"] for r in rows_of(d)]), axis=0)
            growth.append("%s %.2f deg" % (name, (med[-1] - med[1]) / (len(med) - 2)))
    if growth:
        out += ["", "Growth per joint from link 2 to link 7 (difference of the medians divided by 5): %s." % ", ".join(growth)]
    return out


def ring_errors(rows, minimum_cos=0.3):
    usable = [r for r in rows if r["cos_pitch"] > minimum_cos]
    return np.array([[abs(v) if v is not None else np.nan for v in r["ring_angle_error_deg"]] for r in usable]), len(usable)


def table_rings(data):
    out = ["| Model | Poses with cos(pitch) > 0.3 | Lift ring error, median / p90 / max (deg) | Pitch ring | Yaw ring | p90 of the three <= 10 deg |",
           "| --- | ---: | --- | --- | --- | --- |"]
    for name in MODEL_NAMES:
        d = data["static"].get(name)
        if not d:
            continue
        errors, count = ring_errors(rows_of(d))
        if not count:
            continue
        p90 = [stats(errors[:, a])[1] for a in range(3)]
        out.append("| %s | %d of %d | %s | %s | %s | %s |" % (name, count, len(rows_of(d)), triple(errors[:, 0]), triple(errors[:, 1]),
                                                           triple(errors[:, 2]), "yes" if max(p90) <= 10.0 else "NO"))
    out += ["", "tiago2: the first two joints of the TIAGo arm cannot change lift, so its lift ring is driven by a constant and shows no error."]
    return out


def table_singularity(data):
    """Output rings against cos(pitch), all models pooled: error of lift and yaw and how many spikes their rings give."""

    rows = [r for name in MODEL_NAMES if name in data["static"] for r in data["static"][name]["rows"]]
    out = ["| cos(pitch) | Poses | Lift ring error, median (deg) | Yaw ring error, median (deg) | Lift ring spikes, median | Pitch ring error, median (deg) | Matrix error, median (deg) |",
           "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for lo, hi in ((0.0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.5), (0.5, 0.8), (0.8, 1.01)):
        chosen = [r for r in rows if lo <= r["cos_pitch"] < hi]
        if not chosen:
            continue
        e = np.array([[abs(v) if v is not None else np.nan for v in r["ring_angle_error_deg"]] for r in chosen])
        silent = np.mean([r["ring_lift_pitch_yaw"][0] is None for r in chosen])
        lift = fmt(np.nanmedian(e[:, 0])) if not np.all(np.isnan(e[:, 0])) else "silent"
        yaw = fmt(np.nanmedian(e[:, 2])) if not np.all(np.isnan(e[:, 2])) else "silent"
        out.append("| %.1f - %.1f | %d | %s%s | %s | %.0f | %s | %s |" % (
            lo, min(hi, 1.0), len(chosen), lift, " (silent in %.0f%%)" % (100 * silent) if silent > 0 else "", yaw,
            np.median([r["ring_spikes"][0] for r in chosen]), fmt(np.nanmedian(e[:, 1])), fmt(np.median([r["error_deg"] for r in chosen]))))
    return out


def table_sweeps(data):
    out = ["| Model | Swept joint | Poses | Error, median / max (deg) |", "| --- | ---: | ---: | --- |"]
    for name in MODEL_NAMES:
        d = data["static"].get(name)
        if not d:
            continue
        sweeps = rows_of(d, "sweep")
        for joint in sorted({r["sweep_joint"] for r in sweeps}):
            e = [r["error_deg"] for r in sweeps if r["sweep_joint"] == joint]
            out.append("| %s | %d | %d | %s / %s |" % (name, joint + 1, len(e), fmt(np.median(e)), fmt(np.max(e))))
    return out


def table_tracking(data):
    out = ["| Model | Joint speed limit (rad/s) | Ticks | Delay (s) | Error while moving, median / p90 (deg) | Error at rest, median / p90 (deg) | Error 1 s after a stop, median / max (deg) | Held poses of the same model, p90 (deg) | Delay <= 0.5 s | Back at the held-pose level 1 s after a stop |",
           "| --- | ---: | ---: | ---: | --- | --- | --- | ---: | --- | --- |"]
    summaries = {}
    for (model, speed), d in sorted(data["tracking"].items()):
        s = tracking_summary(d)
        summaries[(model, speed)] = s
        held = stats([r["error_deg"] for r in rows_of(data["static"][model])])[1] if model in data["static"] else float("nan")
        back = s["error_1s_after_stop"][0] <= held if np.isfinite(held) and np.isfinite(s["error_1s_after_stop"][0]) else None
        out.append("| %s | %.2f | %d | %.2f | %s / %s | %s / %s | %s / %s | %s | %s | %s |" % (
            model, speed, s["ticks"], s["delay_s"], fmt(s["error_moving"][0]), fmt(s["error_moving"][1]),
            fmt(s["error_rest"][0]), fmt(s["error_rest"][1]), fmt(s["error_1s_after_stop"][0]), fmt(s["error_1s_after_stop"][2]),
            fmt(held), "yes" if s["delay_s"] <= 0.5 else "NO", "-" if back is None else ("yes" if back else "NO")))
    return out, summaries


def table_delay(data, summaries):
    labels = (("default", "as built (link populations in the path, readout smoothed with 50 ms)"),
              ("direct", "no link populations in the path"), ("tau20", "readout smoothed with 20 ms"),
              ("direct_tau20", "both"),
              ("lead", "as built, but the encoders write the angle each joint will have one delay later"),
              ("othergoals", "as built, other goals"),
              ("othergoals_lead", "writing ahead, other goals"))
    out = ["| Variant | Delay (s) | Error while moving, median (deg) | The same against the true orientation one delay earlier (deg) | Error at rest, median / p90 (deg) |",
           "| --- | ---: | ---: | ---: | --- |"]
    for key, label in labels:
        if key == "default":
            s = summaries.get(("tiago7", 0.5))
        else:
            d = data.get("tracking_variants", {}).get(key)
            s = tracking_summary(d) if d else None
        if s:
            out.append("| %s | %.2f | %s | %s | %s / %s |" % (label, s["delay_s"], fmt(s["error_moving"][0]), fmt(s["error_after_delay"]),
                                                            fmt(s["error_rest"][0]), fmt(s["error_rest"][1])))
    return out


def table_cross(data):
    head = ["| Model | Poses | Joint by joint, median / p90 / max (deg) | All at once | Median of (joint by joint - all at once) (deg) | Joint by joint closer | All at once closer | Sign test p |",
            "| --- | ---: | --- | --- | ---: | ---: | ---: | --- |"]
    out = list(head)
    for name, flat in sorted(data["cross"].items()):
        if name in data["static"]:
            a, b = paired(rows_of(data["static"][name]), rows_of(flat))
            out.append(paired_line(name, a, b))
    return out


def table_old(data):
    if not data["old"] or "generic2" not in data["static"]:
        return ["No data of the old decoder."], None
    new = data["static"]["generic2"]["rows"]
    old = data["old"]
    out = ["| Poses | Number | This decoder, median / p90 / max (deg) | Old decoder | Median of (this - old) (deg) | This closer | Old closer | Sign test p |",
           "| --- | ---: | --- | --- | ---: | ---: | ---: | --- |"]
    result = {}
    for kind, label in (("random", "300 random poses"), ("legacy_sweep", "the old decoder's two sweeps")):
        a, b = paired([r for r in new if r.get("kind") == kind], [r for r in old if r.get("kind") == kind])
        if a.size:
            out.append(paired_line(label, a, b))
            result[kind] = (a, b)
    out += ["", "Error of each of the three angles on the random poses with cos(pitch) > 0.3, median / p90 / max (deg):", "",
            "| Decoder | Poses | Lift | Pitch | Yaw |", "| --- | ---: | --- | --- | --- |"]
    for label, rows in (("this decoder (output rings)", [r for r in new if r.get("kind") == "random"]),
                        ("old decoder (output rings)", [r for r in old if r.get("kind") == "random"])):
        errors, count = ring_errors(rows)
        out.append("| %s | %d | %s | %s | %s |" % (label, count, triple(errors[:, 0]), triple(errors[:, 1]), triple(errors[:, 2])))
    return out, result


def table_robust(data):
    out = ["| Model | Variant | Poses | Error, median / p90 / max (deg) | Median of (variant - as built) on the same poses (deg) | Similar |",
           "| --- | --- | ---: | --- | ---: | --- |"]
    for model in sorted({m for m, _ in data["robust"]}):
        base = rows_of(data["static"][model]) if model in data["static"] else []
        variants = [v for m, v in data["robust"] if m == model]
        count = min([len(rows_of(data["robust"][(model, v)])) for v in variants] + [len(base)]) if base else 0
        if base:
            e = [r["error_deg"] for r in base[:count]]
            out.append("| %s | %s | %d | %s | 0 | %s |" % (model, VARIANT_LABELS["default"], len(e), triple(e), "yes"))
        for variant in [v for v in VARIANT_LABELS if v in variants]:
            rows = rows_of(data["robust"][(model, variant)])
            e = np.array([r["error_deg"] for r in rows])
            med, p90, _ = stats(e)
            difference = "-"
            if base:
                a, b = paired(rows, base)
                difference = "%+.2f" % float(np.median(a - b))
            out.append("| %s | %s | %d | %s | %s | %s |" % (model, VARIANT_LABELS[variant], e.size, triple(e), difference,
                                                         "yes" if med <= PASS_MEDIAN and p90 <= PASS_P90 else "NO"))
    return out


def table_marks(data, summaries):
    out = ["| Test | Pass mark | Outcome | Met |", "| --- | --- | --- | --- |"]

    def row(test, mark, outcome, met):
        out.append("| %s | %s | %s | %s |" % (test, mark, outcome, "-" if met is None else ("yes" if met else "NO")))

    row("1. Maths", "equal to 1e-9; URDF to 1e-6", "asserted by `tests/test_kinematics.py`", True)
    p = data.get("parts")
    if p:
        f, st = p["feature"], p["stage"]
        row("3. Parts: cos / sin", "within 0.03", "largest error %.3f" % f["max"], f["max"] <= 0.03)
        row("3. Parts: product", "rms at most 0.02", "rms %.4f" % p["product"]["rms"], p["product"]["rms"] <= 0.02)
        row("3. Parts: one stage, direction", "within 2 deg", "largest %.2f deg" % st["direction_error_deg_max"], st["direction_error_deg_max"] <= 2.0)
        row("3. Parts: one stage, length", "within 3% of the vector's length", "largest %.1f%%" % st["length_error_relative_max_percent"],
            st["length_error_relative_max_percent"] <= 3.0)
    static = [(name, stats([r["error_deg"] for r in rows_of(data["static"][name])])) for name in MODEL_NAMES if name in data["static"]]
    if static:
        ok = [name for name, (med, p90, _) in static if med <= PASS_MEDIAN and p90 <= PASS_P90]
        row("4. Held poses", "median at most 5 deg and p90 at most 10 deg, every model",
            "%d of %d models; medians %.2f to %.2f deg, largest p90 %.2f deg, largest error %.2f deg" % (
                len(ok), len(static), min(m for _, (m, _, _) in static), max(m for _, (m, _, _) in static),
                max(p for _, (_, p, _) in static), max(t for _, (_, _, t) in static)), len(ok) == len(static))
        worst = 0.0
        for name, _ in static:
            errors, count = ring_errors(rows_of(data["static"][name]))
            if count:
                worst = max(worst, max(stats(errors[:, a])[1] for a in range(3)))
        row("6. Output rings", "p90 at most 10 deg where cos(pitch) > 0.3", "largest p90 of any ring and model %.2f deg" % worst, worst <= 10.0)
    half = [(key, s) for key, s in summaries.items() if abs(key[1] - 0.5) < 1e-9]
    if half:
        delays = [s["delay_s"] for _, s in half]
        row("7. Moving arm: delay", "at most 0.5 s at 0.5 rad/s", "%.2f to %.2f s (%d models)" % (min(delays), max(delays), len(half)), max(delays) <= 0.5)
        back = []
        for (model, _), s in summaries.items():
            if model in data["static"]:
                back.append(s["error_1s_after_stop"][0] <= stats([r["error_deg"] for r in rows_of(data["static"][model])])[1])
        row("7. Moving arm: after a stop", "back at the held-pose level 1 s after a stop", "%d of %d sessions" % (sum(back), len(back)), all(back))
    return out


def table_encoders(data):
    """The three ways of writing an angle on a ring, on the same 100 poses."""

    out = ["| Model | Encoder | Error against the true pose, median / p90 / max (deg) | Rounding to a ring index alone, median (deg) | Rings read back, median (deg) | Readout against the pose that was written, median (deg) |",
           "| --- | --- | --- | ---: | ---: | ---: |"]
    labels = (("default", "between indices, every tick (used)"), ("nearest", "nearest index, every tick"),
              ("corrective", "nearest index, only when the bump is off (v4)"))
    for model in sorted({m for m, _ in data["robust"]}):
        for key, label in labels:
            if key == "default":
                rows = rows_of(data["static"][model])[:100] if model in data["static"] else []
            else:
                rows = rows_of(data["robust"][(model, key)]) if (model, key) in data["robust"] else []
            if rows:
                out.append("| %s | %s | %s | %s | %s | %s |" % (
                    model, label, triple([r["error_deg"] for r in rows]), fmt(np.median([r["written_error_deg"] for r in rows])),
                    fmt(np.median([r["held_error_deg"] for r in rows])), fmt(np.median([r["decoder_error_deg"] for r in rows]))))
    return out


def table_rate(data):
    if not data["rate"]:
        return ["No rate study."]
    out = ["| Model | Rate copy, median / p90 (deg) | Spiking network, median / p90 (deg) |", "| --- | --- | --- |"]
    chosen = {r["model"]: r for r in data["rate"]["rows"]
              if r["product"] == 200 and r["relay"] and r["noise"] == 0.0 and r.get("fractional", True)}
    for name in MODEL_NAMES:
        if name in chosen and name in data["static"]:
            e = [r["error_deg"] for r in rows_of(data["static"][name])]
            out.append("| %s | %s / %s | %s / %s |" % (name, fmt(chosen[name]["median"]), fmt(chosen[name]["p90"]), fmt(stats(e)[0]), fmt(stats(e)[1])))
    return out


def table_sizes(data):
    if not data["rate"]:
        return ["No rate study."]
    out = ["| Model | Product 100, link 40 | Product 200, link 60 (chosen) | Product 400, link 100 |", "| --- | --- | --- | --- |"]
    for name in ("generic2", "generic4", "generic7", "tiago7"):
        cells = []
        for product in (100, 200, 400):
            match = [r for r in data["rate"]["rows"] if r["model"] == name and r["product"] == product and r["relay"]
                     and r["noise"] == 0.0 and r.get("fractional", True)]
            cells.append("%s / %s" % (fmt(match[0]["median"]), fmt(match[0]["p90"])) if match else "-")
        out.append("| %s | %s |" % (name, " | ".join(cells)))
    return out


# -- figures ---------------------------------------------------------------------
def _style(plt):
    plt.rcParams.update({
        "font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "axes.edgecolor": GREY, "axes.labelcolor": INK,
        "xtick.color": GREY, "ytick.color": GREY, "text.color": INK, "axes.grid": True, "grid.color": LIGHT, "grid.linewidth": 0.8,
        "axes.axisbelow": True, "legend.frameon": False, "figure.dpi": 150, "savefig.bbox": "tight", "axes.titlesize": 10.5,
        "axes.titleweight": "bold", "axes.titlelocation": "left",
    })


def figure_error_by_joints(plt, data, path):
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    for family, color in FAMILY_COLORS.items():
        xs, med, p90 = [], [], []
        for n in range(2, 8):
            d = data["static"].get("%s%d" % (family, n))
            if d:
                e = [r["error_deg"] for r in rows_of(d)]
                xs.append(n)
                med.append(np.median(e))
                p90.append(np.percentile(e, 90))
        if not xs:
            continue
        ax.plot(xs, med, "-o", color=color, lw=2, ms=6, mec="white", mew=1.2)
        ax.plot(xs, p90, "--o", color=color, lw=1.5, ms=4.5, mec="white", mew=1)
        label = {"generic": "generic chains", "tiago": "TIAGo arm"}[family]
        ax.annotate(label + ", median", (xs[-1], med[-1]), xytext=(8, -3 if family == "generic" else 3), textcoords="offset points", color=INK, fontsize=9, va="center")
        ax.annotate("90th percentile", (xs[-1], p90[-1]), xytext=(8, 3 if family == "generic" else -3), textcoords="offset points", color=GREY, fontsize=9, va="center")
    ax.axhline(PASS_MEDIAN, color=GREY, lw=1, ls=":")
    ax.annotate("pass mark for the median: 5 deg", (2, PASS_MEDIAN), xytext=(0, 4), textcoords="offset points", color=GREY, fontsize=9)
    ax.set_xlim(1.8, 9.2)
    ax.set_ylim(0, 5.8)
    ax.set_xticks(range(2, 8))
    ax.set_xlabel("joints in the chain")
    ax.set_ylabel("orientation error (deg)")
    ax.set_title("Orientation error of the end effector, 300 held poses per model")
    fig.savefig(path)
    plt.close(fig)


def figure_along_arm(plt, data, path):
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    for family, color in FAMILY_COLORS.items():
        d = data["static"].get(family + "7")
        if not d:
            continue
        per_link = np.array([r["link_error_deg"] for r in rows_of(d)])
        med = np.median(per_link, axis=0)
        p90 = np.percentile(per_link, 90, axis=0)
        ax.fill_between(range(1, 8), med, p90, color=color, alpha=0.12, lw=0)
        ax.plot(range(1, 8), med, "-o", color=color, lw=2, ms=6, mec="white", mew=1.2)
        ax.annotate({"generic": "generic7", "tiago": "tiago7"}[family], (7, med[-1]), xytext=(8, 0), textcoords="offset points", color=INK, fontsize=9, va="center")
    ax.set_xlim(0.8, 7.9)
    ax.set_ylim(0, None)
    ax.set_xlabel("link (number of joints passed)")
    ax.set_ylabel("orientation error of the link (deg)")
    ax.set_title("Error along the arm: median (line) up to the 90th percentile (band)")
    fig.savefig(path)
    plt.close(fig)


def figure_old_new(plt, old_pairs, path):
    if not old_pairs or "random" not in old_pairs:
        return False
    new, old = old_pairs["random"]
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    for values, color, label, level in ((new, NEW, "this decoder", 88), (old, OLD, "old decoder", 55)):
        v = np.sort(values)
        ax.plot(v, np.arange(1, v.size + 1) / v.size * 100.0, color=color, lw=2)
        ax.annotate("%s: median %.1f deg" % (label, np.median(v)), (np.percentile(v, level), level), xytext=(10, -4), textcoords="offset points", color=INK, fontsize=9)
    ax.set_xlim(0, max(12.0, float(np.percentile(old, 99))))
    ax.set_ylim(0, 100)
    ax.set_xlabel("orientation error (deg)")
    ax.set_ylabel("poses with at most this error (%)")
    ax.set_title("Two joints, the same %d random poses" % new.size)
    fig.savefig(path)
    plt.close(fig)
    return True


def figure_legacy_sweeps(plt, data, path):
    if not data["old"] or "generic2" not in data["static"]:
        return False
    new = [r for r in data["static"]["generic2"]["rows"] if r.get("kind") == "legacy_sweep"]
    old = [r for r in data["old"] if r.get("kind") == "legacy_sweep"]
    if not new or not old:
        return False
    fig, axes = plt.subplots(2, 3, figsize=(10.5, 5.2), sharex=True, sharey=True)
    for row, joint in enumerate((1, 0)):
        for col, name in enumerate(("lift", "pitch", "yaw")):
            ax = axes[row][col]
            for rows, color, lw in ((old, OLD, 1.8), (new, NEW, 1.8)):
                chosen = sorted([r for r in rows if r["sweep_joint"] == joint], key=lambda r: r["q"][joint])
                x = [np.degrees(r["q"][joint]) for r in chosen]
                y = [np.nan if r["ring_lift_pitch_yaw"][col] is None else np.degrees(r["ring_lift_pitch_yaw"][col]) for r in chosen]
                ax.plot(x, y, "o", color=color, ms=4, mec="white", mew=0.6)
            chosen = sorted([r for r in new if r["sweep_joint"] == joint], key=lambda r: r["q"][joint])
            ax.plot([np.degrees(r["q"][joint]) for r in chosen], [np.degrees(r["true_lift_pitch_yaw"][col]) for r in chosen],
                    "-", color=INK, lw=1, zorder=0)
            ax.set_title("%s, joint %d swept" % (name, joint + 1))
            ax.set_yticks([-180, -90, 0, 90, 180])
            ax.set_xticks([-180, -90, 0, 90, 180])
            if row == 1:
                ax.set_xlabel("swept joint angle (deg)")
            if col == 0:
                ax.set_ylabel("angle (deg)")
    axes[0][0].plot([], [], "-", color=INK, lw=1, label="true")
    axes[0][0].plot([], [], "o", color=NEW, ms=4, label="this decoder's output ring")
    axes[0][0].plot([], [], "o", color=OLD, ms=4, label="old decoder's output ring")
    fig.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.02))
    fig.savefig(path)
    plt.close(fig)
    return True


def figure_rings(plt, data, path):
    rows = [r for name in MODEL_NAMES if name in data["static"] for r in data["static"][name]["rows"]]
    if not rows:
        return False
    fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.3), sharey=True)
    for a, name in enumerate(("lift", "pitch", "yaw")):
        ax = axes[a]
        x = [r["cos_pitch"] for r in rows if r["ring_angle_error_deg"][a] is not None]
        y = [abs(r["ring_angle_error_deg"][a]) for r in rows if r["ring_angle_error_deg"][a] is not None]
        ax.plot(x, y, ".", color=NEW, ms=3, alpha=0.5)
        ax.axvline(0.3, color=GREY, lw=1, ls=":")
        ax.set_yscale("symlog", linthresh=10)
        ax.set_yticks([0, 5, 10, 30, 90, 180])
        ax.set_yticklabels(["0", "5", "10", "30", "90", "180"])
        ax.set_xlim(0, 1.02)
        ax.set_title(name + " ring")
        ax.set_xlabel("cos(pitch) of the true orientation")
        if a == 0:
            ax.set_ylabel("error of the ring's angle (deg)")
    fig.savefig(path)
    plt.close(fig)
    return True


def figure_tracking(plt, data, path, model="tiago7", speed=0.5):
    d = data["tracking"].get((model, speed))
    if not d:
        return False
    ticks = [t for t in d["ticks"] if t["t"] <= d["ticks"][0]["t"] + 40.0]
    t = np.array([x["t"] for x in ticks]) - ticks[0]["t"]
    fig, axes = plt.subplots(4, 1, figsize=(8.5, 7.0), sharex=True)
    for a, name in enumerate(("lift", "pitch", "yaw")):
        ax = axes[a]
        true = np.degrees([x["true_lift_pitch_yaw"][a] for x in ticks])
        ring = np.array([np.nan if x["ring_lift_pitch_yaw"][a] is None else np.degrees(x["ring_lift_pitch_yaw"][a]) for x in ticks])
        for values, color, lw in ((true, INK, 1.6), (ring, NEW, 1.6)):
            v = values.copy()
            v[1:][np.abs(np.diff(v)) > 180] = np.nan            # do not draw the jump from +180 to -180
            ax.plot(t, v, color=color, lw=lw)
        half = 90 if a == 1 else 180
        ax.set_ylim(-half - 10, half + 10)
        ax.set_yticks([-half, 0, half])
        ax.set_ylabel(name + " (deg)")
    handles = [axes[0].plot([], [], color=INK, label="true")[0], axes[0].plot([], [], color=NEW, label="output ring")[0]]
    fig.suptitle("%s keeps moving from goal to goal, joint speed limit %.2f rad/s" % (model, speed), x=0.01, y=0.995, ha="left",
                 fontsize=10.5, fontweight="bold")
    fig.legend(handles=handles, loc="upper right", ncol=2, bbox_to_anchor=(0.99, 0.965))
    fig.subplots_adjust(top=0.9)
    axes[3].plot(t, [x["error_deg"] for x in ticks], color=ERROR, lw=1.4)
    axes[3].set_ylabel("orientation error (deg)")
    axes[3].set_ylim(0, None)
    axes[3].set_xlabel("time (s)")
    fig.savefig(path)
    plt.close(fig)
    return True


def figure_robust(plt, data, path):
    models = sorted({m for m, _ in data["robust"]})
    if not models:
        return False
    fig, axes = plt.subplots(1, len(models), figsize=(5.2 * len(models), 4.2), sharey=True, squeeze=False)
    names = ["default"] + [v for v in VARIANT_LABELS if v != "default"]
    for ax, model in zip(axes[0], models):
        ys, labels = [], []
        for y, variant in enumerate(names):
            if variant == "default":
                rows = rows_of(data["static"][model])[:100] if model in data["static"] else []
            else:
                rows = rows_of(data["robust"][(model, variant)]) if (model, variant) in data["robust"] else []
            if not rows:
                continue
            e = [r["error_deg"] for r in rows]
            med, p90, _ = stats(e)
            ax.plot([med, p90], [y, y], color=LIGHT, lw=4, solid_capstyle="round")
            ax.plot(med, y, "o", color=NEW if variant != "default" else INK, ms=7, mec="white", mew=1.2)
            ax.plot(p90, y, "|", color=GREY, ms=9, mew=1.5)
            ys.append(y)
            labels.append(VARIANT_LABELS[variant])
        ax.set_yticks(ys)
        ax.set_yticklabels(labels)
        ax.invert_yaxis()
        ax.set_xlim(0, None)
        ax.set_xlabel("orientation error (deg): median (dot) to 90th percentile (bar end)")
        ax.set_title(model)
        ax.grid(axis="y", visible=False)
    fig.savefig(path)
    plt.close(fig)
    return True


def figure_cost(plt, path):
    from ring_so3.kinematics import used_products

    ns = list(range(2, 8))
    flat = [len(flat_wiring(get_chain("generic%d" % n)).products) for n in ns]
    chain = [len(chain_wiring(get_chain("generic%d" % n)).products) for n in ns]
    features = [len(used_products(get_chain("generic%d" % n))) for n in ns]
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    for values, color, label, ls in ((flat, GREY, "all at once: pairwise products", "-"), (features, GREY, "product features that occur", ":"),
                                     (chain, NEW, "joint by joint: pairwise products", "-")):
        ax.plot(ns, values, ls, color=color, lw=2, marker="o", ms=5, mec="white", mew=1)
        ax.annotate(label, (ns[-1], values[-1]), xytext=(8, 0), textcoords="offset points", color=INK, fontsize=9, va="center")
    ax.set_xlim(1.8, 10.8)
    ax.set_xticks(ns)
    ax.set_ylim(0, None)
    ax.set_xlabel("joints in the chain")
    ax.set_ylabel("count")
    ax.set_title("What the readout costs as the chain grows")
    fig.savefig(path)
    plt.close(fig)


# -- main ------------------------------------------------------------------------
def build(data, out_dir, figures=True):
    lines = ["# Tables of the results", "",
             "Written by `scripts/report.py` from `docs/results/data/`; do not edit by hand. Orientation error is the angle of",
             "the single rotation that turns the decoded orientation into the true one, in degrees. A readout that knows nothing",
             "would be %.1f deg off on average. Median / p90 / max: half of the poses are at or below the median, 90%% at or below" % CHANCE_ERROR_DEG,
             "p90. Reading and figures: [`../results.md`](../results.md).", ""]

    def section(title, body, note=None):
        lines.extend(["## " + title, ""])
        if note:
            lines.extend([note, ""])
        lines.extend(body + [""])

    section("0. The parts on their own (test 3)", table_parts(data),
            "Each part measured in NEST on inputs its readout was not fitted on (`scripts/run_parts.py`).")
    section("1. The models", table_models(data),
            "Sizes of the spiking network and how fast it runs.")
    section("2. What the readout costs", table_cost(),
            "Counted from the wiring, no simulation. The old decoder used a 32 x 32 grid of cells for each of 16 signed features of two joints.")
    section("3. Held poses (test 4)", table_static(data),
            "300 random poses per model; each is reached by moving the arm, held for 1 s, and the readout of the last 0.5 s is compared "
            "with the exact kinematics. *Rings alone*: the exact orientation of the angles read back from the joint rings' activity, "
            "against the true pose: what the rings themselves lose.")
    section("4. Error of every link (test 5)", table_links(data), "Median orientation error of each link's frame over the same poses (deg).")
    section("5. Output rings (test 6)", table_rings(data),
            "Angle of each output ring (centre of its activity) against the true lift, pitch and yaw, for the poses where lift and yaw are defined well.")
    section("6. Output rings near pitch = 90 deg", table_singularity(data),
            "All models and all poses (random and sweeps) pooled, by cos(pitch). Where cos(pitch) is small, lift and yaw are not defined; "
            "the matrix stays right.")
    section("7. Sweeps of single joints", table_sweeps(data),
            "One joint over its whole range, the others at one fixed pose (24 values per joint; 72 for generic2).")
    tracking, summaries = table_tracking(data)
    section("8. A moving arm (test 7)", tracking,
            "The arm goes from one random goal to the next and rests 1 s at each. *Delay*: the shift in time for which the readout matches "
            "the true orientation best while the arm moves. *Error at rest* and *1 s after a stop* are single 50 ms readings (a held pose in "
            "section 3 is the mean of ten). Pass marks: delay at most 0.5 s at 0.5 rad/s; 1 s after a stop the error is back at the level of "
            "held poses, taken here as: its median is at most the 90th percentile of the same model's held poses.")
    section("8b. Where the delay comes from", table_delay(data, summaries),
            "tiago7 at 0.5 rad/s with a shorter path through the network. What remains with both changes is mostly the joint rings' own "
            "lag behind a moving angle and the cos / sin populations.")
    section("9. All at once against joint by joint (test 8)", table_cross(data), "The same poses for both versions.")
    old, old_pairs = table_old(data)
    section("10. Against the old two-joint decoder (test 9)", old,
            "The old decoder is the frozen legacy code of the SO-2 repo with its stored weights (16,884 neurons); this decoder is `generic2` "
            "(%s neurons). The old decoder gives three angles; its orientation is the rotation built from them."
            % (format(data["static"]["generic2"]["neurons"], ",") if "generic2" in data["static"] else "?"))
    section("11. Variants and robustness (test 10)", table_robust(data), "The first 100 random poses of the static test.")
    section("11b. The three encoders", table_encoders(data),
            "*Rounding alone*: the exact orientation of the written ring indices against the true pose. *Rings read back*: the exact orientation "
            "of the angles read from the joint rings' activity against the true pose. Each encoder has its own calibration.")
    section("12. Rate copy against the spiking network", table_rate(data),
            "The rate copy has the same populations as constant firing rates, with readouts fitted to the closed-form rates.")
    section("13. Rate copy: the sizes considered", table_sizes(data),
            "Error median / p90 (deg) predicted by the rate copy on 200 poses (not the test poses). The middle column was chosen before the test poses were run.")

    section("14. The pass marks", table_marks(data, summaries),
            "Set before the tests were run (docs/tests.md). Tests without a mark (2, 5, 8, 9, 10) are reported in their sections.")

    written = []
    if figures:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        _style(plt)
        fig_dir = os.path.join(out_dir, "figures")
        os.makedirs(fig_dir, exist_ok=True)
        figure_cost(plt, os.path.join(fig_dir, "cost.png"))
        written.append("cost.png")
        if data["static"]:
            figure_error_by_joints(plt, data, os.path.join(fig_dir, "error_by_joints.png"))
            figure_along_arm(plt, data, os.path.join(fig_dir, "error_along_arm.png"))
            written += ["error_by_joints.png", "error_along_arm.png"]
        for name, made in (("old_vs_new.png", figure_old_new(plt, old_pairs, os.path.join(fig_dir, "old_vs_new.png"))),
                           ("legacy_sweeps.png", figure_legacy_sweeps(plt, data, os.path.join(fig_dir, "legacy_sweeps.png"))),
                           ("output_rings.png", figure_rings(plt, data, os.path.join(fig_dir, "output_rings.png"))),
                           ("tracking_tiago7.png", figure_tracking(plt, data, os.path.join(fig_dir, "tracking_tiago7.png"))),
                           ("robustness.png", figure_robust(plt, data, os.path.join(fig_dir, "robustness.png")))):
            if made:
                written.append(name)
    path = os.path.join(out_dir, "tables.md")
    with open(path, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines).rstrip() + "\n")
    return path, written


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--data", default=os.path.join(ROOT, "docs", "results", "data"))
    parser.add_argument("--out", default=os.path.join(ROOT, "docs", "results"))
    parser.add_argument("--no-figures", action="store_true")
    args = parser.parse_args(argv)
    path, written = build(load(args.data), args.out, figures=not args.no_figures)
    print("written: %s (%d lines), %d figures" % (os.path.relpath(path), sum(1 for _ in open(path, encoding="utf-8")), len(written)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

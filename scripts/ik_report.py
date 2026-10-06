#!/usr/bin/env python3
"""Tables and figures of docs/ik/results.md from the data in docs/ik/results/data/.

    python3 -B scripts/ik_report.py                    # writes docs/ik/results/tables.md and docs/ik/results/figures/
    python3 -B scripts/ik_report.py --no-figures

Nothing here simulates.  The shards of a run (``<run>_<i>of<n>.json``) are put together; pairs
that a shard did not get to are counted and named.  Every number quoted in docs/ik/results.md
and in the IK section of README.md comes from the tables written here.
"""

import argparse
import glob
import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.ik.experiments import reach_summary  # noqa: E402

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
INK, GREY, LIGHT = "#1c2330", "#8a94a3", "#d9dee6"
BLUE, GREEN, ORANGE, PURPLE = "#2a78d6", "#1baf7a", "#eb6834", "#8a56c2"
MARKS = {"reached_generic7": 0.95, "reached_tiago7": 0.85, "of_exact": 0.90, "final_deg": 3.0, "path": 1.3, "time_s": 6.0,
         "direction_deg": 10.0, "sign": 0.95, "disturbed": 0.90}
ARMS = ("generic3", "generic7", "tiago7")


def read(path):
    with open(path, encoding="utf-8") as stream:
        return json.load(stream)


def load(data):
    out = {"study": None, "parts": {}, "scans": {}, "runs": {}, "standard": None, "forward": {}}
    if os.path.exists(os.path.join(data, "standard.json")):
        out["standard"] = read(os.path.join(data, "standard.json"))
    forward = os.path.normpath(os.path.join(data, "..", "..", "..", "results", "data"))     # the forward network's own results
    for arm in ARMS:
        static, tracking = os.path.join(forward, "static", arm + ".json"), os.path.join(forward, "tracking", arm + "_0.50.json")
        if os.path.exists(static) and os.path.exists(tracking):
            out["forward"][arm] = {"static": read(static), "tracking": read(tracking)}
    if os.path.exists(os.path.join(data, "study.json")):
        out["study"] = read(os.path.join(data, "study.json"))
    for path in sorted(glob.glob(os.path.join(data, "parts_*.json"))):
        out["parts"][os.path.basename(path)[len("parts_"):-5]] = read(path)
    for path in sorted(glob.glob(os.path.join(data, "goal_ring_scan_*.json"))):
        out["scans"][os.path.basename(path)[len("goal_ring_scan_"):-5]] = read(path)
    for path in sorted(glob.glob(os.path.join(data, "*_*of*.json"))):
        match = re.match(r"(.+)_(\d+)of(\d+)(_rest\d+)?\.json$", os.path.basename(path))
        if not match:
            continue
        document = read(path)
        run = out["runs"].setdefault(match.group(1), {"rows": [], "skipped": {}, "shards": 0, "of": int(match.group(3)), "meta": document})
        run["rows"] += document["rows"]
        run["skipped"][match.group(2)] = document.get("skipped", [])          # files sort so that a later completion comes last
        run["shards"] += match.group(4) is None
    for run in out["runs"].values():
        run["skipped"] = sorted(index for left in run["skipped"].values() for index in left)
        run["rows"].sort(key=lambda row: row["index"])
        run["summary"] = reach_summary(run["rows"])
        run["speed"] = run["meta"].get("speed")
    return out


def fmt(value, digits=2, unit=""):
    if value is None or (isinstance(value, float) and not np.isfinite(value)):
        return "–"
    text = ("%." + str(digits) + "f") % value
    return (text[1:] if text.startswith("-") and float(text) == 0.0 else text) + unit


def pct(value):
    return "–" if value is None else "%.0f%%" % (100.0 * value)


def met(ok):
    return "–" if ok is None else ("met" if ok else "**not met**")


def by_index(run):
    return {row["index"]: row for row in run["rows"]}


def incomplete(run):
    missing = run["of"] - run["shards"]
    notes = []
    if missing:
        notes.append("%d of %d shards missing" % (missing, run["of"]))
    if run["skipped"]:
        notes.append("%d pairs not run (time guard)" % len(run["skipped"]))
    return "; ".join(notes)


# -- tables --------------------------------------------------------------------
def table_study(data):
    study = data["study"]
    if not study:
        return []
    lines = ["## 1. The rule itself (exact kinematics, no neurons)", "",
             "100 start and goal pairs per arm (seed %d, not the test pairs). Delay %.1f s, share noise %.3f rad, dead zone %.2f rad, "
             "readout off by %.0f°. Loop gain %.2f per second (all joints at once: 1.00)." % (
                 study["seed"], study["base"]["delay_s"], study["base"]["noise"], study["base"]["dead_zone"], study["base"]["bias_deg"],
                 study["design"]["kappa"]), "",
             "| Arm | Rule | Reached | Final error, median | 5° after | Path / shortest | Missed (at a joint limit) |",
             "| --- | --- | ---: | ---: | ---: | ---: | ---: |"]
    main = ("cascade", "all joints at once", "one joint at a time", "cascade, angle comparison")
    for row in study["rows"]:
        if row["label"] in main:
            lines.append("| %s | %s | %.0f%% | %s | %s | %s | %d (%d) |" % (
                row["model"], row["label"], row["reached_pct"], fmt(row["final_median_deg"], 2, "°"), fmt(row["to_5_deg_s"], 1, " s"),
                fmt(row["path_over_shortest"]), row["missed"], row["missed_at_limit"]))
    lines += ["", "What the rule needs (7 joints, cascade):", "",
              "| Arm | Condition | Reached | Final error, median | 5° after | Path / shortest |", "| --- | --- | ---: | ---: | ---: | ---: |"]
    for row in study["rows"]:
        if row["label"] not in main:
            lines.append("| %s | %s | %.0f%% | %s | %s | %s |" % (
                row["model"], row["label"].replace("cascade, ", ""), row["reached_pct"], fmt(row["final_median_deg"], 2, "°"),
                fmt(row["to_5_deg_s"], 1, " s"), fmt(row["path_over_shortest"])))
    return lines + [""]


def table_parts(data):
    if not data["parts"]:
        return []
    lines = ["## 2. Comparison and shares in spikes, arm held by hand (tests 3a, 3b)", "",
             "100 poses each; the goal is part of the way to another pose. Marks: direction of the comparison within 10° (median); "
             "right sign in 95% of the shares above 0.05 rad.", "",
             "| Arm | Comparison | Neurons | Direction error, median | p90 | Comparison error, rms | Shares above 0.05 rad | Right sign | Share error, rms | Share where none is asked, rms | 3a | 3b |",
             "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |"]
    for name, document in data["parts"].items():
        s = document["summary"]
        design = document["ik_design"]
        label = design["comparison"] + ("" if design["cascade"] else ", all at once")
        lines.append("| %s | %s | %s | %s | %s | %s | %d | %s | %s | %s | %s | %s |" % (
            document["chain"], label, "{:,}".format(document["neurons"]), fmt(s["turn_direction_error_median_deg"], 1, "°"),
            fmt(s["turn_direction_error_p90_deg"], 1, "°"), fmt(s["turn_rms_rad"], 3, " rad"), s["shares_large"],
            pct(s["share_right_sign"]), fmt(s["share_rms_rad"], 3, " rad"), fmt(s["share_when_none_asked_rms_rad"], 3, " rad"),
            met(s["turn_direction_error_median_deg"] <= MARKS["direction_deg"]), met(s["share_right_sign"] >= MARKS["sign"])))
    lines += ["", "Share error along the arm (rms, rad): does the cascade get worse from joint to joint?", ""]
    for name, document in data["parts"].items():
        if document["ik_design"]["comparison"] == "matrix" and document["ik_design"]["cascade"]:
            lines.append("- %s: %s" % (document["chain"], ", ".join("joint %d %.3f" % (r["joint"], r["rms_rad"]) for r in document["summary"]["per_joint"])))
    return lines + [""]


SCAN_NAMES = {"a": "(a) attractor ring, share pair silent without a share", "b": "(b) attractor ring, share pair in push-pull",
              "c": "(c) no attractor: a gain pair gated by the state bump writes beside it"}


def table_goal_ring(data):
    if not data["scans"]:
        return []
    lines = ["## 3. One joint's goal ring (test 3c)", "",
             "Marks: lead without a share below 1 index; lead at a full share 6 indices or more, with the right sign; rising with the "
             "share; right sign for a share of 0.02 rad in 90% of the trials (0.5 s each); never two bumps; never silent; never written again.", "",
             "| Version | Settings tried | Meet all marks | Best: lead without a share | at a full share | 0.02 rad share, right sign | Bump lost and written again |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for key in sorted(data["scans"]):
        runs = data["scans"][key]["runs"]

        def badness(run):
            m = run["marks"]
            return (not m["passed"], m["rewritten"] + m["silent_ticks"], -m["small_share_right_sign"], abs(m["lead_without_share"]))
        best = min(runs, key=badness)["marks"]
        lines.append("| %s | %d | %d | %s | %s | %s | %d |" % (
            SCAN_NAMES.get(key, key), len(runs), sum(run["marks"]["passed"] for run in runs), fmt(best["lead_without_share"], 2),
            fmt(best["lead_at_full_share"], 1), pct(best["small_share_right_sign"]), best["rewritten"]))
    if "c" in data["scans"]:
        chosen = [run for run in data["scans"]["c"]["runs"] if run["settings"]["shift"] == 8 and abs(run["settings"]["rest_pa"] - 304000.0) < 1
                  and abs(run["settings"]["share_drive_pa"] - 228000.0) < 1]
        if chosen:
            lines += ["", "The setting that is used (shift 8, rest current 304,000 pA, share drive 228,000 pA): lead in ring indices against the share "
                      "(in units of the largest share a joint takes).", "", "| Share | " + " | ".join(sorted(chosen[0]["marks"]["lead_by_share"], key=float)) + " |",
                      "| --- |" + " ---: |" * len(chosen[0]["marks"]["lead_by_share"]),
                      "| Lead | " + " | ".join(fmt(chosen[0]["marks"]["lead_by_share"][k], 2) for k in sorted(chosen[0]["marks"]["lead_by_share"], key=float)) + " |"]
    return lines + [""]


def reach_line(name, run):
    s = run["summary"]
    return "| %s | %d | %s | %s | %s | %s | %s | %s | %d (%d) | %s |" % (
        name, s["trials"], pct(s["reached"]), pct(s["exact_reached"]), pct(s["reached_of_exact_reached"]),
        fmt(s["final_error_reached_median_deg"], 2, "°"), fmt(s["time_to_5_deg_median_s"], 1, " s"), fmt(s["path_ratio_median"]),
        s["missed"], s["missed_at_limit"], incomplete(run) or "complete")


REACH_HEAD = ["| Run | Trials | Reached | Exact rule reaches | Reached, of those the exact rule reaches | Final error of reached, median | 5° after, median | Path / shortest, median | Missed (with a joint at a limit) | Data |",
              "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |"]


def table_reach(data):
    runs = data["runs"]
    names = [n for n in ("reach_generic3", "reach_generic7", "reach_tiago7") if n in runs]
    if not names:
        return []
    lines = ["## 4. Reaching and trajectory in closed loop (tests 4, 5)", "",
             "Spiking network, light robot, 100 start and goal pairs per arm (test seed). Reached: true error at most 5° at the end of the trial.", ""]
    lines += REACH_HEAD + [reach_line(n.replace("reach_", ""), runs[n]) for n in names]
    lines += ["", "| Arm | Neurons | Speed (of real time, shards side by side) | Trial length, median | Decoded orientation off by, median | Exact rule: 5° after | Exact rule: path / shortest |",
              "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for n in names:
        s, meta = runs[n]["summary"], runs[n]["meta"]
        lines.append("| %s | %s | %s | %s | %s | %s | %s |" % (
            n.replace("reach_", ""), "{:,}".format(meta["neurons"]), fmt(meta["speed"]), fmt(s["duration_median_s"], 1, " s"),
            fmt(s["decoded_error_median_deg"], 2, "°"), fmt(s["exact_time_to_5_deg_median_s"], 1, " s"), fmt(s["exact_path_ratio_median"])))
    return lines + [""]


def paired_reach(a, b):
    """Of the pairs both runs have: how many each reached, and how many of those ``a`` reached ``b`` reached too."""

    rows_a, rows_b = by_index(a), by_index(b)
    common = sorted(set(rows_a) & set(rows_b))
    reached_a = [i for i in common if rows_a[i]["reached"]]
    return {"common": len(common), "a": sum(rows_a[i]["reached"] for i in common), "b": sum(rows_b[i]["reached"] for i in common),
            "kept": sum(rows_b[i]["reached"] for i in reached_a), "of": len(reached_a)}


def table_comparisons(data):
    runs = data["runs"]
    lines = []
    for arm in ARMS:
        if "reach_" + arm in runs and "angles_" + arm in runs:
            if not lines:
                lines = ["## 5. The two comparisons (test 6)", "", "The first 50 test pairs for the angle comparison; all 100 for the matrices. Rule for the default, fixed before the runs: more goals reached; within 2 points, the straighter path; then fewer neurons.", ""]
                lines += REACH_HEAD
            lines.append(reach_line(arm + ", matrices", runs["reach_" + arm]))
            lines.append(reach_line(arm + ", angles", runs["angles_" + arm]))
    if lines:
        lines += ["", "| Arm | Neurons, matrices | Neurons, angles |", "| --- | ---: | ---: |"]
        for arm in ARMS:
            if "reach_" + arm in runs and "angles_" + arm in runs:
                lines.append("| %s | %s | %s |" % (arm, "{:,}".format(runs["reach_" + arm]["meta"]["neurons"]), "{:,}".format(runs["angles_" + arm]["meta"]["neurons"])))
        lines.append("")
    return lines


DISTURBANCES = (("weak", "robot moves 30% less than told"), ("strong", "robot moves 30% more than told"),
                ("push", "one joint pushed 0.3 rad after 2 s"), ("newgoal", "goal changed after 2 s"))


def table_disturbances(data):
    runs = data["runs"]
    lines = []
    for arm in ("generic7", "tiago7"):
        base = runs.get("reach_" + arm)
        for key, label in DISTURBANCES:
            run = runs.get("%s_%s" % (key, arm))
            if base is None or run is None:
                continue
            if not lines:
                lines = ["## 6. Disturbances (test 7)", "",
                         "On the first 25 test pairs. Mark: reached in 90% of the trials that are reached without the disturbance. "
                         "For a changed goal the trial is judged against the new goal, and the mark is taken against all trials.", "",
                         "| Arm | Disturbance | Trials | Reached | Reached without it (same pairs) | Of those reached without: reached with | Final error of reached, median | 5° after, median | Mark | Data |",
                         "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |"]
            p, s = paired_reach(base, run), run["summary"]
            share = (s["reached"] / (p["a"] / p["common"])) if key == "newgoal" and p["a"] else (p["kept"] / p["of"] if p["of"] else None)
            lines.append("| %s | %s | %d | %s | %d of %d | %s | %s | %s | %s | %s |" % (
                arm, label, s["trials"], pct(s["reached"]), p["a"], p["common"], "–" if key == "newgoal" else "%d of %d" % (p["kept"], p["of"]),
                fmt(s["final_error_reached_median_deg"], 2, "°"), fmt(s["time_to_5_deg_median_s"], 1, " s"),
                met(share is not None and share >= MARKS["disturbed"]), incomplete(run) or "complete"))
    return lines + ([""] if lines else [])


def table_hard(data):
    runs = data["runs"]
    lines = []
    steep = [(n, label) for n, label in (("steep_generic7", "generic7, matrices"), ("steep_angles_generic7", "generic7, angles")) if n in runs]
    if steep:
        lines += ["## 7. Hard cases (test 8)", "", "Goals whose pitch is 75° or more from level (20 pairs of their own):", ""] + REACH_HEAD
        lines += [reach_line(label, runs[n]) for n, label in steep] + [""]
    if "reach_tiago7" in runs:
        rows = runs["reach_tiago7"]["rows"]
        at_limit = [r for r in rows if r["at_limit"]]
        free = [r for r in rows if not r["at_limit"]]
        if not steep:
            lines += ["## 7. Hard cases (test 8)", ""]
        lines += ["TIAGo, 7 joints, by how the trial ended:", "", "| Ended | Trials | Reached | Exact rule reaches | Final error, median |", "| --- | ---: | ---: | ---: | ---: |"]
        for label, group in (("with a joint at a limit", at_limit), ("with no joint at a limit", free)):
            if group:
                lines.append("| %s | %d | %s | %s | %s |" % (label, len(group), pct(np.mean([r["reached"] for r in group])),
                                                           pct(np.mean([r["exact_reached"] for r in group])), fmt(float(np.median([r["final_error_deg"] for r in group])), 2, "°")))
        lines.append("")
    return lines


VARIANTS = (("allatonce", "every joint gets the whole comparison (gain 1.0)"), ("silence10", "10% of the neurons of comparison and cascade off"),
            ("noise", "noise current of 50 pA into every population"), ("seed2", "other random spikes (kernel seed 2)"), ("seed3", "other random spikes (kernel seed 3)"))


def table_variants(data):
    runs = data["runs"]
    base = runs.get("reach_generic7")
    present = [(key, label) for key, label in VARIANTS if key + "_generic7" in runs]
    if base is None or not present:
        return []
    lines = ["## 8. Variants (test 9)", "", "generic7, the first 25 test pairs.", "",
             "| Variant | Trials | Reached | As built, same pairs | Final error of reached, median | 5° after, median | Path / shortest, median | Data |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |"]
    for key, label in present:
        run = runs[key + "_generic7"]
        p, s = paired_reach(base, run), run["summary"]
        lines.append("| %s | %d | %s | %d of %d | %s | %s | %s | %s |" % (
            label, s["trials"], pct(s["reached"]), p["a"], p["common"], fmt(s["final_error_reached_median_deg"], 2, "°"),
            fmt(s["time_to_5_deg_median_s"], 1, " s"), fmt(s["path_ratio_median"]), incomplete(run) or "complete"))
    return lines + [""]


STANDARD_ORDER = ("dls", "transpose", "ccd", "cascade")
STANDARD_LABELS = {"dls": "damped least squares", "transpose": "Jacobian transpose", "ccd": "cyclic coordinate descent",
                   "cascade": "exact cascade (the ring IK's rule without neurons)"}


def standard_row(data, arm, method, condition):
    for row in (data["standard"] or {}).get("reach", []):
        if (row["chain"], row["method"], row["condition"]) == (arm, method, condition):
            return row
    return None


def table_standard(data):
    standard, runs = data["standard"], data["runs"]
    if not standard:
        return []
    lines = ["## 9. Ring IK beside the standard methods", "",
             "The same 100 test pairs per arm, the same joint limits and the same largest step per joint, on the light robot. "
             "\"As the network\": the method sees the arm 0.2 s late, with share noise 0.005 rad, a dead zone of 0.01 rad, the turn bounded at "
             "0.5 rad per component and the orientation off by 1°. \"Perfect\": none of that. Loop gain 1.25 per second (transpose: 1.00). "
             "Last column: of the goals this method reaches, how many the ring IK reaches too (same pairs).", "",
             "| Arm | Method | Sensing | Reached | Final error of reached, median | 5° after, median | Path / shortest, median | Missed (at a limit) | Ring IK reaches of these |",
             "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for arm in ARMS:
        ring = runs.get("reach_" + arm)
        if ring:
            s = ring["summary"]
            lines.append("| %s | **ring IK (spikes)** | its own | %s of %d | %s | %s | %s | %d (%d) | – |" % (
                arm, pct(s["reached"]), s["trials"], fmt(s["final_error_reached_median_deg"], 2, "°"), fmt(s["time_to_5_deg_median_s"], 1, " s"),
                fmt(s["path_ratio_median"]), s["missed"], s["missed_at_limit"]))
        ring_rows = by_index(ring) if ring else {}
        for condition in ("as the network", "perfect"):
            for method in STANDARD_ORDER:
                row = standard_row(data, arm, method, condition)
                if row is None:
                    continue
                both = [r for r in row["rows"] if r["reached"] and r["index"] in ring_rows]
                kept = "%d of %d" % (sum(ring_rows[r["index"]]["reached"] for r in both), len(both)) if both else "–"
                lines.append("| %s | %s | %s | %s | %s | %s | %s | %d (%d) | %s |" % (
                    arm, STANDARD_LABELS[method], condition, pct(row["reached"]), fmt(row["final_error_reached_median_deg"], 2, "°"),
                    fmt(row["time_to_5_deg_median_s"], 1, " s"), fmt(row["path_ratio_median"]), row["missed"], row["missed_at_limit"], kept))
    lines += ["", "Solving for the joint angles in one go (bounded least squares from the start pose; no trajectory):", "",
              "| Arm | Solved within 0.1° | Within 5° | Evaluations of the kinematics, median |", "| --- | ---: | ---: | ---: |"]
    for row in standard["solve"]:
        lines.append("| %s | %s | %s | %.0f |" % (row["chain"], pct(row["exact"]), pct(row["solved"]), row["evaluations_median"]))
    return lines + [""]


def table_forward(data):
    standard = data["standard"]
    if not standard or not data["forward"]:
        return []
    lines = ["## 10. Forward kinematics: the ring network beside the exact computation", "",
             "The exact orientation is a product of 3 x 3 matrices in double precision: no error, no delay. The ring network's numbers are those of "
             "the forward results (`docs/results/`), 300 held poses and a moving arm at 0.5 rad/s.", "",
             "| Arm | Exact: sines and cosines | Exact: multiplications | Network: neurons | Network: product populations | Network: error at a held pose, median | p90 | Network: error while moving at 0.5 rad/s, median |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for cost in standard["fk_cost"]:
        forward = data["forward"].get(cost["chain"])
        if not forward:
            continue
        errors = [r["error_deg"] for r in forward["static"]["rows"] if r.get("kind") == "random"]
        moving = [t["error_deg"] for t in forward["tracking"]["ticks"] if t["moving"]]
        lines.append("| %s | %d | %d | %s | %d | %s | %s | %s |" % (
            cost["chain"], cost["trigonometric"], cost["multiplications_exact"], "{:,}".format(forward["static"]["neurons"]), cost["products_network"],
            fmt(float(np.median(errors)), 2, "°"), fmt(float(np.percentile(errors, 90)), 2, "°"), fmt(float(np.median(moving)), 2, "°")))
    return lines + [""]


def table_integration(data):
    standard = data["standard"]
    if not standard:
        return []
    lines = ["## 11. Path integration: orientation from joint velocities", "",
             "The standard way to follow an orientation without position sensing: integrate the joint velocities. Arm moving from goal to goal "
             "at up to 0.5 rad/s for 60 s; every velocity reading has noise and a constant offset; median over 8 runs. The ring network reads "
             "joint positions, so its error does not grow; its row is the forward network's tracking run at the same speed.", "",
             "| Arm | Method | Velocity noise, offset (rad/s) | Corrected | Error after 10 s | 30 s | 60 s | Median over the run | Largest |",
             "| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: |"]
    for row in standard["integration"]:
        curves = np.array(row["error_deg"], dtype=float)
        largest = float(np.median(curves.max(axis=1)))
        free = row["correct_s"] is None
        lines.append("| %s | integration | %.3f, %.3f | %s | %s | %s | %s | %s | %s |" % (
            row["chain"], row["noise"], row["bias"], "never" if free else "every %.0f s" % row["correct_s"],
            fmt(row["error_at_10s_deg"], 1, "°") if free else "–", fmt(row["error_at_30s_deg"], 1, "°") if free else "–",
            fmt(row["error_final_deg"], 1, "°") if free else "–", fmt(row["error_median_deg"], 1, "°"), fmt(largest, 1, "°")))
    for arm in ("generic7", "tiago7"):
        forward = data["forward"].get(arm)
        if forward:
            ticks = forward["tracking"]["ticks"]
            errors = np.array([t["error_deg"] for t in ticks])
            third = len(errors) // 3
            lines.append("| %s | **ring network** (positions) | – | not needed | %s | %s | %s | %s | %s |" % (
                arm, fmt(float(np.median(errors[:third])), 1, "°") + " (first third)", fmt(float(np.median(errors[third:2 * third])), 1, "°") + " (second)",
                fmt(float(np.median(errors[2 * third:])), 1, "°") + " (last)", fmt(float(np.median(errors)), 1, "°"), fmt(float(errors.max()), 1, "°")))
    return lines + [""]


GAZEBO_RUNS = (("gazebo_ring", "ring IK (spikes)"), ("gazebo_dls", "damped least squares"))


def table_gazebo(data):
    runs = data["runs"]
    present = [(name, label) for name, label in GAZEBO_RUNS if name in runs]
    if not present:
        return []
    base = runs.get("reach_tiago7")
    lines = ["## 12. The TIAGo in Gazebo as the robot", "",
             "The first 30 test pairs of tiago7. Gazebo's physics advances 50 ms per tick of the network; each joint's target goes to the arm's "
             "trajectory controller. Mark, fixed before the run: ring IK in Gazebo reaches at least 80% of the goals it reaches on the light robot.", ""]
    lines += REACH_HEAD + [reach_line(label + ", Gazebo", runs[name]) for name, label in present]
    lines += ["", "| Method | On the light robot, same pairs | Of those: reached in Gazebo | Joint behind its target, median of the largest per tick | Base moved by (largest) | Mark |",
              "| --- | ---: | ---: | ---: | ---: | --- |"]
    for name, label in present:
        run = runs[name]
        lag = [r["lag_median_rad"] for r in run["rows"] if r.get("lag_median_rad") is not None]
        base_shift = [float(np.hypot(r["base"][0], r["base"][1])) for r in run["rows"] if r.get("base")]
        if name == "gazebo_ring" and base:
            p = paired_reach(base, run)
            light, kept, mark = "%d of %d" % (p["a"], p["common"]), "%d of %d" % (p["kept"], p["of"]), met(p["of"] > 0 and p["kept"] / p["of"] >= 0.8)
        else:
            light, kept, mark = "–", "–", "–"
            row = standard_row(data, "tiago7", name.replace("gazebo_", ""), "perfect")
            if row:
                mine = by_index(run)
                common = [r for r in row["rows"] if r["index"] in mine]
                reached = [r for r in common if r["reached"]]
                light = "%d of %d" % (len(reached), len(common))
                kept = "%d of %d" % (sum(mine[r["index"]]["reached"] for r in reached), len(reached))
        lines.append("| %s | %s | %s | %s | %s | %s |" % (
            label, light, kept, fmt(float(np.degrees(np.median(lag))), 2, "°") if lag else "–",
            fmt(1000.0 * max(base_shift), 1, " mm") if base_shift else "–", mark))
    return lines + [""]


def table_marks(data):
    runs, parts = data["runs"], data["parts"]
    lines = ["## 0. The marks, set before the runs", "", "| Test | Mark | Result | |", "| --- | --- | --- | --- |"]
    if parts:
        worst = max(d["summary"]["turn_direction_error_median_deg"] for d in parts.values())
        sign = min(d["summary"]["share_right_sign"] for d in parts.values())
        lines.append("| 3a | direction of the comparison within 10° (median) | %s on the worst of %d runs | %s |" % (fmt(worst, 1, "°"), len(parts), met(worst <= 10.0)))
        lines.append("| 3b | right sign in 95%% of the shares above 0.05 rad | %s on the worst run | %s |" % (pct(sign), met(sign >= 0.95)))
    if "c" in data["scans"] or data["scans"]:
        passed = {key: sum(run["marks"]["passed"] for run in scan["runs"]) for key, scan in data["scans"].items()}
        lines.append("| 3c | one joint's goal ring: no lead without a share, rising to 6 indices, right sign at 0.02 rad, one bump | settings that meet it: %s | %s |" % (
            ", ".join("%s %d of %d" % (key, passed[key], len(data["scans"][key]["runs"])) for key in sorted(passed)), met(any(passed.values()))))
    for arm, mark in (("generic7", MARKS["reached_generic7"]), ("tiago7", MARKS["reached_tiago7"])):
        run = runs.get("reach_" + arm)
        if run:
            s = run["summary"]
            lines.append("| 4 | %s: %d%% of the goals reached | %s of %d | %s |" % (arm, round(100 * mark), pct(s["reached"]), s["trials"], met(s["reached"] >= mark)))
            lines.append("| 4 | %s: 90%% of the goals the exact rule reaches | %s | %s |" % (arm, pct(s["reached_of_exact_reached"]), met(s["reached_of_exact_reached"] is not None and s["reached_of_exact_reached"] >= MARKS["of_exact"])))
            lines.append("| 4 | %s: final error of reached goals 3° (median) | %s | %s |" % (arm, fmt(s["final_error_reached_median_deg"], 2, "°"), met(s["final_error_reached_median_deg"] is not None and s["final_error_reached_median_deg"] <= MARKS["final_deg"])))
            lines.append("| 5 | %s: path at most 1.3 times the shortest (median) | %s | %s |" % (arm, fmt(s["path_ratio_median"]), met(s["path_ratio_median"] is not None and s["path_ratio_median"] <= MARKS["path"])))
            lines.append("| 5 | %s: 5° within 6 s (median) | %s | %s |" % (arm, fmt(s["time_to_5_deg_median_s"], 1, " s"), met(s["time_to_5_deg_median_s"] is not None and s["time_to_5_deg_median_s"] <= MARKS["time_s"])))
    if "reach_generic3" in runs:
        s = runs["reach_generic3"]["summary"]
        lines.append("| 4 | generic3: reported, no mark | %s of %d reached (exact rule %s) | – |" % (pct(s["reached"]), s["trials"], pct(s["exact_reached"])))
    for arm in ("generic7", "tiago7"):
        base = runs.get("reach_" + arm)
        for key, label in DISTURBANCES:
            run = runs.get("%s_%s" % (key, arm))
            if base and run:
                p, s = paired_reach(base, run), run["summary"]
                if key == "newgoal":
                    share = s["reached"] / (p["a"] / p["common"]) if p["a"] else None
                    text = "%s reached, %s without" % (pct(s["reached"]), pct(p["a"] / p["common"]))
                else:
                    share = p["kept"] / p["of"] if p["of"] else None
                    text = "%d of %d" % (p["kept"], p["of"])
                lines.append("| 7 | %s, %s: 90%% of the trials reached without | %s | %s |" % (arm, label, text, met(share is not None and share >= MARKS["disturbed"])))
    ring, base = runs.get("gazebo_ring"), runs.get("reach_tiago7")
    if ring and base:
        p = paired_reach(base, ring)
        lines.append("| Gazebo | ring IK on the Gazebo TIAGo: 80%% of the goals it reaches on the light robot | %d of %d | %s |" % (
            p["kept"], p["of"], met(p["of"] > 0 and p["kept"] / p["of"] >= 0.8)))
    return lines + [""]


# -- figures -------------------------------------------------------------------
def _style(plt):
    plt.rcParams.update({
        "font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "axes.edgecolor": GREY, "axes.labelcolor": INK,
        "xtick.color": GREY, "ytick.color": GREY, "text.color": INK, "axes.grid": True, "grid.color": LIGHT, "grid.linewidth": 0.8,
        "axes.axisbelow": True, "legend.frameon": False, "figure.dpi": 150, "savefig.bbox": "tight", "axes.titlesize": 10.5,
        "axes.titleweight": "bold", "axes.titlelocation": "left",
    })


def figure_error_over_time(plt, data, path):
    names = [n for n in ("reach_generic3", "reach_generic7", "reach_tiago7") if n in data["runs"]]
    if not names:
        return False
    figure, axes = plt.subplots(1, len(names), figsize=(4.2 * len(names), 3.3), sharey=True, squeeze=False)
    grid = np.arange(0.0, 20.01, 0.2)
    for axis, name in zip(axes[0], names):
        curves = []
        for row in data["runs"][name]["rows"]:
            t, e = np.array(row["t"]), np.array(row["error_deg"])
            curve = np.interp(grid, t, e, right=e[-1])                    # after its end a trial stays where it stopped
            curves.append(curve)
            axis.plot(grid, curve, color=GREEN if row["reached"] else ORANGE, linewidth=0.6, alpha=0.35)
        axis.plot(grid, np.median(curves, axis=0), color=INK, linewidth=2)
        axis.axhline(5.0, color=GREY, linewidth=1, linestyle="--")
        axis.set_yscale("symlog", linthresh=5.0)
        axis.set_yticks([0, 5, 20, 60, 180])
        axis.set_yticklabels(["0", "5", "20", "60", "180"])
        axis.set_xlabel("time since the goal was given (s)")
        axis.set_title("%s: %d of %d reached" % (name.replace("reach_", ""), sum(r["reached"] for r in data["runs"][name]["rows"]), len(curves)))
    axes[0][0].set_ylabel("distance from the goal (deg)")
    axes[0][0].plot([], [], color=GREEN, linewidth=1, label="reached")
    axes[0][0].plot([], [], color=ORANGE, linewidth=1, label="missed")
    axes[0][0].plot([], [], color=INK, linewidth=2, label="median")
    axes[0][0].legend(loc="upper right", fontsize=8)
    figure.savefig(path)
    plt.close(figure)
    return True


def figure_goal_ring(plt, data, path):
    scans = data["scans"]
    if "c" not in scans:
        return False
    figure, axis = plt.subplots(figsize=(5.4, 3.5))
    shares = np.array(scans["c"]["shares"])

    def best(scan):
        return min(scan["runs"], key=lambda run: (not run["marks"]["passed"], run["marks"]["rewritten"], abs(run["marks"]["lead_without_share"])))
    chosen = [run for run in scans["c"]["runs"] if run["settings"]["shift"] == 8 and abs(run["settings"]["rest_pa"] - 304000.0) < 1
              and abs(run["settings"]["share_drive_pa"] - 228000.0) < 1]
    series = [("no attractor (used)", (chosen or [best(scans["c"])])[0], BLUE)]
    for key, label, color in (("a", "attractor ring, best of (a)", ORANGE), ("b", "attractor ring, best of (b)", PURPLE)):
        if key in scans:
            series.append((label, best(scans[key]), color))
    axis.plot([-1, 1], [-8, 8], color=GREY, linewidth=1, linestyle="--", label="lead = share")
    for label, run, color in series:
        mean = [row["lead_mean"] for row in run["summary"]]
        sd = [row["lead_sd"] for row in run["summary"]]
        axis.errorbar(shares, mean, yerr=sd, color=color, linewidth=2, marker="o", markersize=4, capsize=2, label=label)
    axis.set_xlabel("share (in units of the largest share a joint takes)")
    axis.set_ylabel("lead of the goal bump (ring indices)")
    axis.set_title("One joint's goal ring: lead against share")
    axis.legend(fontsize=8, loc="upper left")
    figure.savefig(path)
    plt.close(figure)
    return True


def figure_example(plt, data, path, name="reach_tiago7"):
    run = data["runs"].get(name)
    if not run:
        return False
    reached = [row for row in run["rows"] if row["reached"] and row["start_error_deg"] > 90]
    if not reached:
        return False
    row = sorted(reached, key=lambda r: r["time_to_5_deg"])[len(reached) // 2]            # the median trial among the long turns
    t = np.array(row["t"])
    figure, axes = plt.subplots(3, 1, figsize=(6.2, 6.6), sharex=True)
    axes[0].plot(t, row["error_deg"], color=INK, linewidth=2)
    axes[0].axhline(5.0, color=GREY, linewidth=1, linestyle="--")
    axes[0].set_ylabel("from the goal (deg)")
    axes[0].set_title("%s, pair %d: a turn of %.0f°" % (name.replace("reach_", ""), row["index"], row["start_error_deg"]))
    shares = np.degrees(np.array(row["shares"]))
    q = np.degrees(np.array(row["q"]))
    colors = plt.cm.viridis(np.linspace(0.05, 0.9, shares.shape[1]))
    for k in range(shares.shape[1]):
        axes[1].plot(t, shares[:, k], color=colors[k], linewidth=1.4, label="joint %d" % (k + 1))
        axes[2].plot(t, q[:, k] - q[0, k], color=colors[k], linewidth=1.4)
    axes[1].set_ylabel("share of the turn (deg)")
    axes[1].legend(fontsize=7, ncol=4, loc="upper right")
    axes[2].set_ylabel("joint angle, from its start (deg)")
    axes[2].set_xlabel("time since the goal was given (s)")
    figure.savefig(path)
    plt.close(figure)
    return True


def figure_reached(plt, data, path):
    runs = data["runs"]
    items = []
    for arm in ARMS:
        for prefix, label in (("reach", "matrices"), ("angles", "angles")):
            run = runs.get("%s_%s" % (prefix, arm))
            if run:
                items.append(("%s, %s" % (arm, label), run["summary"]["reached"], run["summary"]["exact_reached"]))
    if not items:
        return False
    figure, axis = plt.subplots(figsize=(6.0, 0.42 * len(items) + 1.3))
    y = np.arange(len(items))[::-1]
    axis.barh(y, [100 * item[1] for item in items], color=BLUE, height=0.55, label="spiking network")
    axis.scatter([100 * item[2] for item in items], y, color=INK, marker="|", s=220, linewidths=2, label="exact rule, same pairs", zorder=3)
    for yi, item in zip(y, items):
        axis.text(100 * item[1] - 1.5, yi, "%.0f%%" % (100 * item[1]), va="center", ha="right", color="white", fontsize=9)
    axis.set_yticks(y)
    axis.set_yticklabels([item[0] for item in items])
    axis.set_xlim(0, 100)
    axis.set_xlabel("goals reached within 5° (%)")
    axis.grid(axis="y", visible=False)
    axis.legend(fontsize=8, loc="lower left")
    figure.savefig(path)
    plt.close(figure)
    return True


METHOD_COLORS = {"ring": BLUE, "cascade": INK, "dls": ORANGE, "transpose": GREEN, "ccd": "#b5892a"}


def figure_methods(plt, data, path):
    if not data["standard"]:
        return False
    arms = [arm for arm in ARMS if "reach_" + arm in data["runs"]]
    if not arms:
        return False
    names = ("ring",) + STANDARD_ORDER
    labels = {"ring": "ring IK (spikes)", "dls": "damped least squares", "transpose": "Jacobian transpose", "ccd": "coordinate descent", "cascade": "exact cascade"}
    figure, axes = plt.subplots(1, 3, figsize=(11.5, 3.4))
    width = 0.8 / len(names)
    for column, (key, title, top) in enumerate((("reached", "Goals reached (%)", 100.0), ("time", "Time to 5° (s, median)", None), ("path", "Path / shortest turn (median)", None))):
        axis = axes[column]
        for m, name in enumerate(names):
            values = []
            for arm in arms:
                if name == "ring":
                    s = data["runs"]["reach_" + arm]["summary"]
                    value = {"reached": 100 * s["reached"], "time": s["time_to_5_deg_median_s"], "path": s["path_ratio_median"]}[key]
                else:
                    row = standard_row(data, arm, name, "as the network")
                    value = {"reached": 100 * row["reached"], "time": row["time_to_5_deg_median_s"], "path": row["path_ratio_median"]}[key]
                values.append(value if value is not None else 0.0)
            x = np.arange(len(arms)) + (m - (len(names) - 1) / 2.0) * width
            axis.bar(x, values, width * 0.9, color=METHOD_COLORS[name], label=labels[name] if column == 0 else None)
        axis.set_xticks(np.arange(len(arms)))
        axis.set_xticklabels(arms)
        axis.set_title(title)
        axis.grid(axis="x", visible=False)
        if top:
            axis.set_ylim(0, top)
        if key == "path":
            axis.set_ylim(1.0, None)
    figure.legend(loc="lower center", ncol=5, fontsize=8, bbox_to_anchor=(0.5, -0.06))
    figure.savefig(path)
    plt.close(figure)
    return True


def figure_integration(plt, data, path, arm="tiago7"):
    standard = data["standard"]
    if not standard:
        return False
    rows = [row for row in standard["integration"] if row["chain"] == arm and row["correct_s"] is None and row["noise"] > 0]
    if not rows:
        return False
    figure, axis = plt.subplots(figsize=(6.2, 3.6))
    shades = ["#f5b08f", ORANGE, "#a8401a"]
    for row, color in zip(rows, shades):
        axis.plot(row["t"], np.median(np.array(row["error_deg"], dtype=float), axis=0), color=color, linewidth=2,
                  label="integrated, noise %.2f, offset %.3f rad/s" % (row["noise"], row["bias"]))
    forward = data["forward"].get(arm)
    if forward:
        ticks = forward["tracking"]["ticks"]
        t = np.array([tick["t"] for tick in ticks]) - ticks[0]["t"]
        keep = t <= 60.0
        axis.plot(t[keep], np.array([tick["error_deg"] for tick in ticks])[keep], color=BLUE, linewidth=1.2, label="ring network, from joint positions")
    axis.set_xlabel("time the arm has been moving (s)")
    axis.set_ylabel("orientation error (deg)")
    axis.set_title("%s: an integrated orientation drifts, a read one does not" % arm)
    axis.legend(fontsize=8, loc="upper left")
    figure.savefig(path)
    plt.close(figure)
    return True


BLOCK_START = "<!-- ik-comparison:start (this block is written by scripts/ik_report.py; do not edit it by hand) -->"
BLOCK_END = "<!-- ik-comparison:end -->"


def technical_block(data):
    """The comparison with the standard methods, short, for the appendix of TECHNICAL.md."""

    standard, runs = data["standard"], data["runs"]
    if not standard:
        return []
    lines = ["*From `docs/ik/results/data/`; the full tables are sections 9 to 12 of [`docs/ik/results/tables.md`](docs/ik/results/tables.md).*", "",
             "**Inverse kinematics.** The test pairs of each arm, light robot, loop gain 1.25 per second (transpose: 1.00). "
             "\"As the network\": 0.2 s delay, share noise 0.005 rad, dead zone 0.01 rad, turn bounded at 0.5 rad, orientation off by 1°.", "",
             "| Arm | Method | Sensing | Trials | Reached | Final error of reached, median | 5° after, median | Path / shortest, median |",
             "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |"]
    for arm in ARMS:
        ring = runs.get("reach_" + arm)
        if ring:
            s = ring["summary"]
            lines.append("| %s | **ring IK (spikes)** | its own | %d | %s | %s | %s | %s |" % (
                arm, s["trials"], pct(s["reached"]), fmt(s["final_error_reached_median_deg"], 2, "°"),
                fmt(s["time_to_5_deg_median_s"], 1, " s"), fmt(s["path_ratio_median"])))
        for condition in ("as the network", "perfect"):
            for method in STANDARD_ORDER:
                row = standard_row(data, arm, method, condition)
                if row:
                    lines.append("| %s | %s | %s | %d | %s | %s | %s | %s |" % (
                        arm, STANDARD_LABELS[method].split(" (")[0], condition, row["trials"], pct(row["reached"]),
                        fmt(row["final_error_reached_median_deg"], 2, "°"), fmt(row["time_to_5_deg_median_s"], 1, " s"), fmt(row["path_ratio_median"])))
    lines += ["", "**In one go** (bounded least squares from the start pose): " + "; ".join(
        "%s %s solved to 0.1° with %.0f evaluations of the kinematics" % (row["chain"], pct(row["exact"]), row["evaluations_median"]) for row in standard["solve"]) + ".", ""]
    if data["forward"]:
        lines += ["**Forward kinematics.**", "",
                  "| Arm | Exact: multiplications | Network: neurons | Network: error at a held pose, median | While moving at 0.5 rad/s, median |",
                  "| --- | ---: | ---: | ---: | ---: |"]
        for cost in standard["fk_cost"]:
            forward = data["forward"].get(cost["chain"])
            if forward:
                errors = [r["error_deg"] for r in forward["static"]["rows"] if r.get("kind") == "random"]
                moving = [t["error_deg"] for t in forward["tracking"]["ticks"] if t["moving"]]
                lines.append("| %s | %d | %s | %s | %s |" % (cost["chain"], cost["multiplications_exact"], "{:,}".format(forward["static"]["neurons"]),
                                                          fmt(float(np.median(errors)), 2, "°"), fmt(float(np.median(moving)), 2, "°")))
        lines.append("")
    lines += ["**Path integration** of the orientation from joint velocities, tiago7, 60 s of motion, never corrected (median of 8 runs), "
              "beside the ring network, which reads joint positions.", "",
              "| Velocity noise, offset (rad/s) | Error after 10 s | After 30 s | After 60 s |", "| --- | ---: | ---: | ---: |"]
    for row in standard["integration"]:
        if row["chain"] == "tiago7" and row["correct_s"] is None:
            lines.append("| %.3f, %.3f | %s | %s | %s |" % (row["noise"], row["bias"], fmt(row["error_at_10s_deg"], 1, "°"),
                                                         fmt(row["error_at_30s_deg"], 1, "°"), fmt(row["error_final_deg"], 1, "°")))
    forward = data["forward"].get("tiago7")
    if forward:
        errors = np.array([t["error_deg"] for t in forward["tracking"]["ticks"]])
        third = len(errors) // 3
        lines.append("| ring network | %s | %s | %s |" % (fmt(float(np.median(errors[:third])), 1, "°") + " (first third of its run)",
                                                        fmt(float(np.median(errors[third:2 * third])), 1, "°") + " (second)",
                                                        fmt(float(np.median(errors[2 * third:])), 1, "°") + " (last)"))
    return lines


def update_block(path, lines):
    """Replace what stands between the two markers of ``path``; True when the file has them."""

    if not lines or not os.path.exists(path):
        return False
    with open(path, encoding="utf-8") as stream:
        text = stream.read()
    if BLOCK_START not in text or BLOCK_END not in text:
        return False
    head, rest = text.split(BLOCK_START, 1)
    _, tail = rest.split(BLOCK_END, 1)
    with open(path, "w", encoding="utf-8") as stream:
        stream.write(head + BLOCK_START + "\n" + "\n".join(lines).rstrip() + "\n" + BLOCK_END + tail)
    return True


def build(data, out_dir, figures=True):
    os.makedirs(out_dir, exist_ok=True)
    lines = ["# Inverse kinematics: tables", "", "Written by `scripts/ik_report.py` from `docs/ik/results/data/`. Do not edit by hand.", ""]
    for table in (table_marks, table_study, table_parts, table_goal_ring, table_reach, table_comparisons, table_disturbances, table_hard, table_variants,
                  table_standard, table_forward, table_integration, table_gazebo):
        lines += table(data)
    with open(os.path.join(out_dir, "tables.md"), "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines).rstrip() + "\n")
    written = ["tables.md"]
    if update_block(os.path.join(ROOT, "TECHNICAL.md"), technical_block(data)):
        written.append(os.path.join("..", "..", "..", "TECHNICAL.md") + " (the comparison block of appendix A)")
    if figures:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        _style(plt)
        folder = os.path.join(out_dir, "figures")
        os.makedirs(folder, exist_ok=True)
        for name, function in (("error_over_time.png", figure_error_over_time), ("goal_ring.png", figure_goal_ring),
                               ("example_trial.png", figure_example), ("reached.png", figure_reached),
                               ("methods.png", figure_methods), ("integration.png", figure_integration)):
            if function(plt, data, os.path.join(folder, name)):
                written.append("figures/" + name)
    return written


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--data", default=os.path.join(ROOT, "docs", "ik", "results", "data"))
    parser.add_argument("--out", default=os.path.join(ROOT, "docs", "ik", "results"))
    parser.add_argument("--no-figures", action="store_true")
    args = parser.parse_args(argv)
    for name in build(load(args.data), args.out, not args.no_figures):
        print("written:", os.path.relpath(os.path.join(args.out, name), ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

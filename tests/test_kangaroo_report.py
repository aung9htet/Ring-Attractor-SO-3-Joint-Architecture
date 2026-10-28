import hashlib
import json
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def static_row(i, err):
    # same row keys as docs/results/data/static/tiago4.json (kind "random")
    return {"q": [0.0] * 4, "arrived": True, "move_ticks": 36, "error_deg": err,
            "decoder_error_deg": err, "written_error_deg": 0.0, "held_error_deg": err,
            "link_error_deg": [err / 4, err / 3, err / 2, err], "defect": 0.1,
            "kind": "random", "index": i}


def static_file(errs):
    return {"chain": "kangaroo", "joints": 4, "design": {}, "options": {}, "neurons": 1,
            "products": 1, "link_populations": 4, "synapses": 1, "simulated_s": 1.0,
            "wall_s": 1.0, "speed": 1.0, "calibration_fit": {}, "experiment": "static",
            "pose_seed": 7, "max_speed": 1.0, "silenced_neurons": 0, "silence": 0.0,
            "rows": [static_row(i, e) for i, e in enumerate(errs)]}


def reach_file():
    # one real row (full key set) from a real reach file
    real = json.loads((ROOT / "docs/ik/results/data/reach_tiago7_0of3.json").read_text())
    row = dict(real["rows"][0], lag_median_rad=0.001, lag_final_rad=[0.0, 0.001, 0.0, 0.002])
    # missed, not at a limit, max |lag_final_rad| 0.01 > 0.005 -> "Missed, joints far from target"
    miss = dict(row, reached=False, exact_reached=False, at_limit=False, exact_at_limit=False,
                lag_final_rad=[0.0, 0.0, 0.01, 0.0], index=row["index"] + 1)
    return {"chain": "kangaroo", "joints": 4, "design": {}, "ik_design": {}, "options": {},
            "neurons": 1, "ik_wiring": {}, "synapses": 1, "simulated_s": 1.0, "wall_s": 1.0,
            "speed": 1.0, "ik_calibration_fit": {}, "experiment": "reach", "pair_seed": 3,
            "pairs": 1, "shard": [0, 1], "motor_scale": 1.0, "push": None,
            "goal_change_s": None, "goal_ring_settings": {}, "silence": 0.0,
            "silenced_neurons": 0, "steep_deg": 5.0, "skipped": [], "rows": [row, miss],
            "summary": {}}


class KangarooReport(unittest.TestCase):
    def test_report_from_minimal_data(self):
        with tempfile.TemporaryDirectory() as tmp:
            data, out = Path(tmp, "data"), Path(tmp, "out")
            written = {
                "static/kangaroo_left4.json": static_file([1.0, 3.0]),   # median 2.0
                "static/kangaroo_right4.json": static_file([2.0, 4.0]),  # median 3.0
                "reach/light_ring_left.json": reach_file(),
            }
            digests = {}
            for rel, obj in written.items():
                p = data / rel
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_text(json.dumps(obj))
                digests[rel] = hashlib.sha256(p.read_bytes()).hexdigest()
            missing = ["tracking/kangaroo_left4_0.25.json", "reach/mujoco_ring_left.json",
                       "reach/mujoco_dls_left.json", "calibration/comparison.json"]

            r = subprocess.run(["python3", "-B", "scripts/kangaroo_report.py",
                                "--data", str(data), "--out", str(out)],
                               cwd=ROOT, capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)

            for name in ("static_error", "error_along_arm"):
                for ext in ("png", "pdf"):
                    self.assertTrue((out / "figures" / f"{name}.{ext}").is_file(), name + ext)

            tables = (out / "tables.md").read_text()
            lines = tables.splitlines()
            for arm, vals in (("left", ("2.00", "2.80", "3.00")), ("right", ("3.00", "3.80", "4.00"))):
                self.assertTrue(any(arm in l.lower() and all(v in l for v in vals) for l in lines),
                                f"no {arm} static line with median/p90/max {vals}")
            hi = next(i for i, l in enumerate(lines) if "joints far from target" in l.lower())
            col = [c.strip().lower() for c in lines[hi].strip("|").split("|")]
            col = next(i for i, c in enumerate(col) if "joints far from target" in c)
            rows = [l for l in lines[hi + 2:] if l.startswith("|")]
            rows = [r for r in rows[:next((i for i, r in enumerate(rows) if "left" not in r.lower()), len(rows))]]
            cells = [[c.strip() for c in r.strip("|").split("|")] for r in rows if "left" in r.lower()]
            self.assertTrue(cells and cells[0][col] == "1", (lines[hi], rows))

            manifest = (out / "figures" / "manifest.json").read_text()
            json.loads(manifest)
            for rel, digest in digests.items():
                self.assertIn(rel, manifest)
                self.assertIn(digest, manifest)
            for rel in missing:
                self.assertIn(rel, manifest)

    def test_reach_trials_cell_counts_skipped(self):
        obj = reach_file()
        obj["skipped"] = [{"index": 5}, {"index": 6}, {"index": 7}]      # a run cut short
        for row in obj["rows"]:
            row["lag_median_rad"] = 0.001                                # a scalar, as ring_so3/ik/session.py writes it
        with tempfile.TemporaryDirectory() as tmp:
            data, out = Path(tmp, "data"), Path(tmp, "out")
            for rel, o in (("reach/mujoco_ring_left.json", obj), ("static/kangaroo_left4.json", static_file([1.0, 3.0])),
                           ("static/kangaroo_right4.json", static_file([2.0, 4.0]))):
                p = data / rel
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_text(json.dumps(o))
            r = subprocess.run(["python3", "-B", "scripts/kangaroo_report.py", "--data", str(data), "--out", str(out)],
                               cwd=ROOT, capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
            lines = (out / "tables.md").read_text().splitlines()
            self.assertTrue(any("Missed, joints on target" in l and "Missed, joints far from target" in l for l in lines))
            hi = next(i for i, l in enumerate(lines) if "Trials" in l)
            cells = [c.strip() for c in lines[hi + 2].strip("|").split("|")]
            self.assertEqual(cells[2], "2 (3 not run)", lines[hi + 2])


if __name__ == "__main__":
    unittest.main()

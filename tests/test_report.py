"""The report's statistics and the check that documents quote only numbers from the generated tables."""

import json
import os
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import check_doc_numbers  # noqa: E402
import report  # noqa: E402
from ring_so3.chains import get_chain  # noqa: E402
from ring_so3.kinematics import rot  # noqa: E402


class StatisticsTests(unittest.TestCase):
    def test_sign_test_is_exact(self):
        self.assertAlmostEqual(report.sign_test([-1] * 10), 2 / 2 ** 10)
        self.assertAlmostEqual(report.sign_test([-1, 1]), 1.0)
        self.assertAlmostEqual(report.sign_test([-1] * 5 + [1]), 2 * (1 + 6) / 64)
        self.assertEqual(report.sign_test([0, 0]), 1.0)
        self.assertLess(report.sign_test([-1.0] * 300), 1e-80)             # no overflow for many pairs

    def test_median_interval_and_stats(self):
        values = np.random.default_rng(0).normal(3.0, 1.0, 400)
        low, high = report.median_interval(values)
        self.assertTrue(low < np.median(values) < high)
        self.assertLess(high - low, 0.5)
        self.assertEqual(report.stats([1.0, 2.0, 3.0, None, float("nan")]), (2.0, 2.8, 3.0))
        self.assertEqual(report.triple([1.0, 2.0, 3.0]), "2.00 / 2.80 / 3.00")

    def test_paired_matches_poses_by_kind_and_index(self):
        a = [{"kind": "random", "index": i, "error_deg": 1.0 + i} for i in range(4)]
        b = [{"kind": "random", "index": i, "error_deg": 5.0} for i in (0, 1, 3)] + [{"kind": "sweep", "index": 2, "error_deg": 0.0}]
        x, y = report.paired(a, b)
        np.testing.assert_array_equal(x, [1.0, 2.0, 4.0])
        np.testing.assert_array_equal(y, [5.0, 5.0, 5.0])
        self.assertIn("| label | 3 | 2.00 / 3.60 / 4.00 | 5.00 / 5.00 / 5.00 | -3.00 | 100% | 0% |", report.paired_line("label", x, y))


class TrackingTests(unittest.TestCase):
    def test_delay_is_found_from_a_log_with_a_known_delay(self):
        chain = get_chain("generic2")
        lag = 4                                                             # ticks: 0.2 s
        angles = [np.array([0.02 * i, 0.5]) for i in range(140)] + [np.array([0.02 * 139, 0.5])] * 40
        ticks = []
        for i, q in enumerate(angles):
            late = chain.rotation(angles[max(i - lag, 0)]) @ rot("z", 0.01)
            ticks.append({"t": 0.05 * i, "q": q.tolist(), "moving": i < 140, "tool": late.tolist(),
                          "error_deg": float(np.degrees(np.arccos(np.clip((np.trace(late.T @ chain.rotation(q)) - 1) / 2, -1, 1))))})
        summary = report.tracking_summary({"chain": "generic2", "ticks": ticks})
        self.assertAlmostEqual(summary["delay_s"], 0.2, delta=0.03)
        self.assertGreater(summary["error_moving"][0], summary["error_rest"][0])
        self.assertAlmostEqual(summary["error_1s_after_stop"][0], np.degrees(0.01), places=2)


class BuildTests(unittest.TestCase):
    def _static(self, name, errors):
        chain = get_chain(name)
        rows = [{"kind": "random", "index": i, "error_deg": e, "decoder_error_deg": e, "written_error_deg": 0.0, "held_error_deg": 0.2,
                 "link_error_deg": [e * (k + 1) / chain.n for k in range(chain.n)], "arrived": True, "cos_pitch": 0.9,
                 "ring_angle_error_deg": [0.5, -1.0, 0.7], "ring_lift_pitch_yaw": [0.1, 0.2, 0.3], "ring_spikes": [500, 500, 500],
                 "true_lift_pitch_yaw": [0.1, 0.2, 0.3], "q": [0.0] * chain.n} for i, e in enumerate(errors)]
        return {"chain": name, "joints": chain.n, "products": 4, "link_populations": 0, "neurons": 1920, "synapses": 400000,
                "speed": 3.0, "rows": rows}

    def test_tables_are_written_from_the_data_and_the_number_check_reads_them(self):
        with tempfile.TemporaryDirectory() as root:
            data = os.path.join(root, "docs", "results", "data")
            os.makedirs(os.path.join(data, "static"))
            with open(os.path.join(data, "static", "generic2.json"), "w", encoding="utf-8") as stream:
                json.dump(self._static("generic2", [0.5, 1.5, 2.5, 3.5]), stream)
            path, written = report.build(report.load(data), os.path.join(root, "docs", "results"), figures=False)
            with open(path, encoding="utf-8") as stream:
                text = stream.read()
            self.assertIn("| generic2 | 4 | 2.00 / 3.20 / 3.50 |", text)
            self.assertIn("| generic2 | 2 | 4 | 0 | 1,920 | 400,000 | - | - |", text)
            self.assertIn("| 7 | 2,186 | 89 | 166 | 62 |", text)                     # counted from the wiring
            self.assertIn("No data of the old decoder.", text)
            self.assertEqual(written, [])
            with open(os.path.join(root, "README.md"), "w", encoding="utf-8") as stream:
                stream.write("# x\n\n## 1. Result\n\nThe median is 2.00 deg and p90 3.20 deg; `9.99` in code is ignored.\n\n## 2. Setup\n\n7.77 is outside.\n")
            self.assertEqual(check_doc_numbers.check(root), (2, []))
            with open(os.path.join(root, "README.md"), "w", encoding="utf-8") as stream:
                stream.write("## 1. Result\n\nThe median is 2.01 deg, 55% of the poses.\n\n## 2. Setup\n")
            self.assertEqual(check_doc_numbers.check(root), (2, [("README.md", "2.01"), ("README.md", "55%")]))

    def test_documents_of_this_repo_quote_only_numbers_from_the_tables(self):
        if not os.path.exists(os.path.join(ROOT, check_doc_numbers.TABLES)):
            self.skipTest("docs/results/tables.md is not generated yet")
        checked, missing = check_doc_numbers.check(ROOT)
        self.assertEqual(missing, [], "run scripts/report.py, then correct the documents")


class DocumentTests(unittest.TestCase):
    def test_links_and_named_files_in_the_documents_exist(self):
        import glob
        import re

        documents = [os.path.join(ROOT, "README.md"), os.path.join(ROOT, "TECHNICAL.md")]
        documents += sorted(glob.glob(os.path.join(ROOT, "docs", "**", "*.md"), recursive=True))
        problems = []
        for path in documents:
            with open(path, encoding="utf-8") as stream:
                text = stream.read()
            folder = os.path.dirname(path)
            for target in re.findall(r"\]\(([^)#\s]+)(?:#[^)]*)?\)", text):
                if target.startswith(("http://", "https://")):
                    continue
                if not os.path.exists(os.path.normpath(os.path.join(folder, target))):
                    problems.append("%s -> %s" % (os.path.relpath(path, ROOT), target))
            for name in re.findall(r"`((?:ring_so3|scripts|tests|docker|docs|artifacts)/[\w./-]+\.(?:py|md|bash|env|json|png|mp4))`", text):
                if "<" in name or "*" in name:
                    continue
                if not os.path.exists(os.path.join(ROOT, name)):
                    problems.append("%s names %s" % (os.path.relpath(path, ROOT), name))
        self.assertEqual(problems, [])


if __name__ == "__main__":
    unittest.main()

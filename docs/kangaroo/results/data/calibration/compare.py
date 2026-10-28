"""Compare the re-run calibration with the checked-in artifacts/, per array; writes comparison.json.

Run from the repo root in the image (optional argument: output directory):
docker/run.bash kangaroo run -- 'python3 -B docs/kangaroo/results/data/calibration/compare.py'
"""
import json, sys
import numpy as np

D = 'docs/kangaroo/results/data/calibration'
OUT = sys.argv[1] if len(sys.argv) > 1 else D
out = []
for name in ('calibration_N200_f120_p200_e60_seed1_fractional', 'ik_calibration_N200_f120_p200_e60_seed1_fractional'):
    a, b = np.load('artifacts/%s.npz' % name), np.load('%s/%s.npz' % (D, name))
    for k in sorted(set(a.files) | set(b.files)):
        if k not in a.files or k not in b.files:
            out.append({'file': name, 'array': k, 'missing_in': 'artifacts' if k not in a.files else 'rerun'}); continue
        x, y = a[k], b[k]
        same_shape = x.shape == y.shape
        d = float(np.max(np.abs(x.astype(float) - y.astype(float)))) if same_shape and x.size and x.dtype.kind in 'fiub' else None
        scale = float(np.max(np.abs(x.astype(float)))) if same_shape and x.size and x.dtype.kind in 'fiub' else None
        out.append({'file': name, 'array': k, 'shape': list(x.shape), 'same_shape': same_shape, 'max_abs_difference': d,
                    'max_abs_value': scale, 'relative': (d / scale if d is not None and scale else None), 'identical': bool(same_shape and np.array_equal(x, y))})
json.dump(out, open('%s/comparison.json' % OUT, 'w'), indent=1)
for r in out: print(r)

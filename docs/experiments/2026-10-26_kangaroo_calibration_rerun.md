# 2026-10-26 Calibration re-run in the Kangaroo image

**Hypothesis.** NEST at the repo's pinned commit (`HEAD@41892a5`) reproduces the checked-in calibration of the default
design on Python 3.10 in the Kangaroo image (`ring-so3-kangaroo:humble`), since the calibration does not depend on the
chain (populations are fitted once per kind, the same for every chain).

**Config.** Default design `calibration_N200_f120_p200_e60_seed1_fractional` and its IK calibration; `scripts/calibrate.py`
and `scripts/calibrate_ik.py` with their defaults (seed 1, 8 threads); `calibrate_ik.py` reads the checked-in forward
calibration, as it always does. The checked-in `artifacts/` are not touched; experiments keep loading them.

**Commands** (branch `kangaroo-migration`, commit 9e3ed24 plus the staged Kangaroo work, which does not touch calibration code):

```bash
docker/run.bash kangaroo run --out docs/kangaroo/results/data/calibration -- 'python3 -B scripts/calibrate.py --out /out/calibration_N200_f120_p200_e60_seed1_fractional.npz && python3 -B scripts/calibrate_ik.py --out /out/ik_calibration_N200_f120_p200_e60_seed1_fractional.npz'
docker/run.bash kangaroo run --out docs/kangaroo/results/data/calibration/repeat -- 'python3 -B scripts/calibrate.py --out /out/calibration_N200_f120_p200_e60_seed1_fractional.npz'
```

The comparison with `artifacts/` (per array: shape, largest absolute and relative difference, identical) is in
`docs/kangaroo/results/data/calibration/comparison.json` and in the calibration table of `docs/kangaroo/results/tables.md`.
It is written by `docker/run.bash kangaroo run -- 'python3 -B docs/kangaroo/results/data/calibration/compare.py'`;
`docker/run.bash kangaroo run --out DIR -- 'python3 -B docs/kangaroo/results/data/calibration/compare.py /out'` writes
it into DIR instead, and that copy is byte for byte the checked-in file.

The calibration code is that of commit 9e3ed24: the staged Kangaroo work (saved as
`docs/kangaroo/results/data/code_at_runs.patch`, `git apply` on 9e3ed24) touches none of `scripts/calibrate*.py` or the
network modules they use.

**Results.**
- Settings recorded in the two `.json` files are identical (threads, seeds, sizes); only the fit statistics and the run
  time differ.
- Four of the six forward arrays (`decoders_norm`, `_product`, `_scalar`, `_source`) and both IK arrays agree with `artifacts/` to about 1e-12 relative (floating point).
- `ring_profile` differs: largest difference 0.6 Hz in one of 200 bins (relative 0.0085); total ring rate 1004.4 Hz
  (artifacts) against 1005.0 Hz (re-run).
- `decoders_feature` differs by up to 0.000161, 28 % of its largest weight; its fit error is essentially the same
  (`feature_rms` 0.01791 artifacts, 0.01784 re-run).
- The two re-runs in the image are bit-identical to each other in every array.

**Interpretation.** The calibration is deterministic in this image; the difference from `artifacts/` most likely comes
from the environment the artifacts were made in (the SO-2 Noetic image, Python 3.8), not from run-to-run noise; that
environment was not rebuilt to test it. A 0.06 % change
in the measured ring spike counts moves the feature decoders, a regularised least-squares fit, a long way in weight
space while their fit error stays the same; the repo's README already notes that the default weights file "differs in
the fourth digit" between writes. This exceeds the spec's 1e-4 relative mark, so it was put to the owner; all Kangaroo
measurements use the checked-in `artifacts/`, unchanged.

**Negative result.** The checked-in calibration is not reproducible bit for bit in the Kangaroo image.

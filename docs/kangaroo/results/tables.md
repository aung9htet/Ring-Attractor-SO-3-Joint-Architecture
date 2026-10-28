# Tables of the Kangaroo results

Written by `scripts/kangaroo_report.py`; do not edit by hand. Errors in degrees.

## Held poses

| Arm | Poses | Error, median / p90 / max (deg) |
| --- | ---: | --- |
| left | 300 | 0.79 / 1.31 / 1.91 |
| right | 300 | 0.78 / 1.32 / 1.91 |

## Tracking

| Arm | Joint speed limit (rad/s) | Delay (s) | Error while moving, median / p90 (deg) | Error at rest, median / p90 (deg) | Error 1 s after a stop, median / p90 (deg) |
| --- | ---: | ---: | --- | --- | --- |
| left | 0.25 | 0.18 | 3.14 / 5.84 | 0.89 / 1.51 | 0.95 / 1.29 |
| left | 0.50 | 0.18 | 5.77 / 11.21 | 0.82 / 1.50 | 0.90 / 1.25 |
| left | 1.00 | 0.18 | 10.92 / 21.33 | 0.89 / 1.62 | 0.87 / 1.62 |
| right | 0.25 | 0.18 | 3.14 / 5.78 | 0.89 / 1.44 | 0.74 / 1.42 |
| right | 0.50 | 0.18 | 5.76 / 11.10 | 0.90 / 1.42 | 0.78 / 1.32 |
| right | 1.00 | 0.18 | 10.94 / 21.32 | 0.87 / 1.43 | 0.85 / 1.47 |

## Reaching

| Arm | Method | Trials | Reached (%) | Final error of reached, median (deg) | Time to 5 deg, median (s) | Path ratio, median | Missed at a limit | Missed, joints far from target | Missed, joints on target |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| left | light ring | 30 (0 not run) | 76.67 | 1.29 | 7.70 | 1.31 | 7 | 0 | 0 |
| left | MuJoCo ring | 30 (0 not run) | 76.67 | 1.10 | 7.60 | 1.29 | 7 | 0 | 0 |
| left | MuJoCo dls | 30 (0 not run) | 76.67 | 0.39 | 7.60 | 1.02 | 7 | 0 | 0 |
| right | light ring | 30 (0 not run) | 76.67 | 1.48 | 7.85 | 1.32 | 7 | 0 | 0 |
| right | MuJoCo ring | 30 (0 not run) | 76.67 | 1.19 | 7.60 | 1.29 | 7 | 0 | 0 |
| right | MuJoCo dls | 30 (0 not run) | 76.67 | 0.39 | 7.70 | 1.02 | 7 | 0 | 0 |

Missed, joints far from target: max |lag_final_rad| over joints > 0.005 rad; a miss without lag data (the light robot has none) is counted under on target. Time to 5 deg median over all trials that got there, final error and path ratio over reached trials (as reach_summary).

## Calibration comparison

| Array | Shape | Largest absolute difference | Relative difference | Identical |
| --- | --- | ---: | ---: | --- |
| decoders_feature | 120x2 | 0.000161 | 0.28 | no |
| decoders_norm | 120x1 | 3.57e-15 | 2.96e-12 | no |
| decoders_product | 200x1 | 1.19e-15 | 2.06e-12 | no |
| decoders_scalar | 60x1 | 1.49e-15 | 3.26e-12 | no |
| decoders_source | 60x1 | 1.18e-15 | 2.47e-12 | no |
| ring_profile | 200 | 0.6 | 0.00852 | no |
| decoders_bounded | 60x1 | 8.7e-16 | 4.24e-13 | no |
| decoders_half | 60x1 | 6.75e-16 | 4.95e-13 | no |

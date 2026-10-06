#!/usr/bin/env python3
"""Test 3: every part of the network measured on its own in NEST, on inputs its readout was not fitted on.

    python3 -B scripts/run_parts.py --out docs/results/data/parts.json

* scalar, product and length populations driven by source populations at random values;
* the cos / sin population driven by a real joint ring written at random positions between indices;
* one multiplying stage: the two-joint chain, where the second joint turns the pair
  ``(s1·s2, -s1·c2)`` of length ``|s1|``;
* how far a joint ring's bump trails an angle that moves at a steady speed.

The same measurements, with fewer points, are asserted in tests/test_nest_parts.py.
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3 import experiments  # noqa: E402
from ring_so3.design import Design  # noqa: E402
from ring_so3.kinematics import circular_difference, orientation_error_deg  # noqa: E402
from ring_so3.mapping import RingMap  # noqa: E402

SEED = 31415            # not the seed of the test poses, and not a calibration grid


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)

    import nest
    from ring_so3.nest import calibrate

    design = Design()
    cal = calibrate.load(design)
    rng = np.random.default_rng(SEED)
    # the encoders' random spikes depend on the number of NEST threads, so the thread count is part of the record
    out = {"experiment": "parts", "threads": args.threads, "seed": SEED, "design": design.describe(),
           "calibration_fit": cal.fit, "calibration_settings": cal.settings}

    pairs, values = rng.uniform(-1.0, 1.0, (300, 2)), rng.uniform(-1.0, 1.0, (150, 1))
    source = cal.decoders["source"]
    product = calibrate.measure_driven(nest, design, "product", pairs, source, threads=args.threads) @ cal.decoders["product"]
    error = product[:, 0] - pairs[:, 0] * pairs[:, 1]
    out["product"] = {"points": len(pairs), "rms": float(np.sqrt(np.mean(error ** 2))), "max": float(np.max(np.abs(error)))}
    scalar = calibrate.measure_driven(nest, design, "scalar", values, source, threads=args.threads) @ cal.decoders["scalar"]
    error = (scalar - values)[:, 0]
    out["scalar"] = {"points": len(values), "rms": float(np.sqrt(np.mean(error ** 2))), "max": float(np.max(np.abs(error)))}
    radius, angle = np.sqrt(rng.uniform(0.0, 1.0, 200)), rng.uniform(0.0, 2 * np.pi, 200)
    disc = np.stack([radius * np.cos(angle), radius * np.sin(angle)], axis=1)
    norm = calibrate.measure_driven(nest, design, "norm", disc, source, threads=args.threads) @ cal.decoders["norm"]
    error = norm[:, 0] - radius
    out["norm"] = {"points": len(disc), "rms": float(np.sqrt(np.mean(error ** 2))), "max": float(np.max(np.abs(error)))}

    size = int(cal.settings["ring_size"])
    positions = np.sort(rng.uniform(0.0, size, 80))
    tuning = calibrate.feature_tuning(design, cal.settings["feature_tau"])
    scale = cal.feature_scale(RingMap(size, -np.pi, np.pi, True))
    hold = 14
    angles, rates, index_error = calibrate.measure_feature(nest, design, tuning, scale, positions, "fractional", size,
                                                           threads=args.threads, seed=SEED, hold_ticks=hold)
    decoded = (rates @ cal.decoders["feature"]).reshape(len(positions), hold, 2).mean(axis=1)
    written = angles.reshape(len(positions), hold)[:, 0]
    error = decoded - np.stack([np.cos(written), np.sin(written)], axis=1)
    angle_error = np.degrees(np.angle(np.exp(1j * (np.arctan2(decoded[:, 1], decoded[:, 0]) - written))))
    out["feature"] = {
        "positions": len(positions), "max": float(np.max(np.abs(error))), "rms": float(np.sqrt(np.mean(error ** 2))),
        "angle_error_deg_max": float(np.max(np.abs(angle_error))), "angle_error_deg_median": float(np.median(np.abs(angle_error))),
        "length_error_max": float(np.max(np.abs(np.linalg.norm(decoded, axis=1) - 1.0))),
        "ring_index_error_mean": float(np.mean(index_error)), "ring_index_error_sd_per_tick": float(np.std(index_error)),
    }

    out["ring_lag"] = []
    for rate_hz in (200.0, 800.0):                       # the encoder's rate, and four times as strong
        for speed in (0.5, 1.5, 3.0):                    # ring indices per 50 ms tick
            nest.ResetKernel()
            nest.set_verbosity("M_ERROR")
            nest.SetKernelStatus({"local_num_threads": 2, "rng_seed": SEED})
            from ring_so3.nest.ring import build_joint_ring

            mapping = RingMap(size, -np.pi, np.pi, True)
            ring = build_joint_ring(nest, "ring", mapping, mode="fractional")
            ring.rate_hz = rate_hz
            errors = []
            for t in range(140):
                target = 60.0 + max(t - 20, 0) * speed
                ring.drive(nest, mapping.angle(target))
                nest.Simulate(50.0)
                ring.read(nest)
                if t >= 60:
                    errors.append(mapping.index_error(ring.index, target % size))
            out["ring_lag"].append({"rate_hz": rate_hz, "speed_index_per_tick": speed, "lag_index": float(-np.mean(errors)),
                                    "lag_ticks": float(-np.mean(errors) / speed), "lag_s": float(-np.mean(errors) / speed * 0.05)})

    chain, _, network, session = experiments.build(nest, "generic2", design, threads=args.threads)
    session.warm_up()
    rows = []
    for _ in range(80):
        m = session.go_to(chain.random_pose(rng))
        true = chain.rotation(m.q)
        pair, true_pair = np.array([m.tool[1, 0], m.tool[1, 2]]), np.array([true[1, 0], true[1, 2]])
        rows.append((np.linalg.norm(true_pair), np.linalg.norm(pair) - np.linalg.norm(true_pair),
                     abs(np.degrees(circular_difference(np.arctan2(pair[1], pair[0]), np.arctan2(true_pair[1], true_pair[0])))),
                     float(np.max(np.abs(m.tool - true))), orientation_error_deg(m.tool, true)))
    a = np.array(rows)
    long_enough = a[:, 0] >= 0.3
    out["stage"] = {
        "poses": len(rows), "vectors_longer_than_0.3": int(long_enough.sum()),
        "direction_error_deg_max": float(np.max(a[long_enough, 2])), "direction_error_deg_median": float(np.median(a[long_enough, 2])),
        "length_error_abs_max": float(np.max(np.abs(a[:, 1]))),
        "length_error_relative_max_percent": float(100.0 * np.max(np.abs(a[long_enough, 1]) / a[long_enough, 0])),
        "entry_error_max": float(np.max(a[:, 3])), "entry_error_rms_of_max": float(np.sqrt(np.mean(a[:, 3] ** 2))),
        "orientation_error_deg_median": float(np.median(a[:, 4])),
    }
    experiments.write_json(args.out, out)
    for key in ("scalar", "product", "norm", "feature", "stage"):
        print(key, {k: round(v, 4) if isinstance(v, float) else v for k, v in out[key].items()})
    for row in out["ring_lag"]:
        print("ring lag", {k: round(v, 3) for k, v in row.items()})
    print("written:", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

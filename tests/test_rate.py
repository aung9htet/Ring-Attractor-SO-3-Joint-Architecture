"""The pieces that need no NEST: neuron tuning and readouts, the ring mapping, the ring's weights and stimulus, the rate copy."""

import unittest

import numpy as np

from ring_so3.chains import get_chain
from ring_so3.design import Design, Sizes
from ring_so3.kinematics import orientation_error_deg
from ring_so3.mapping import RingMap
from ring_so3.neurons import NeuronModel, SAMPLES, analytic_decoders, make_tuning, solve_decoders
from ring_so3.nest.ring import (bump_rates, fractional_bump_rates, ring_distances, ring_weight_matrix, wall_profile)
from ring_so3.rate.network import RateNetwork
from ring_so3.wiring import chain_wiring, flat_wiring


class NeuronTests(unittest.TestCase):
    def test_rate_and_current_are_inverse_and_zero_below_rheobase(self):
        model = NeuronModel()
        self.assertAlmostEqual(model.rheobase, 187.5)
        rates = np.array([5.0, 50.0, 150.0, 300.0])
        np.testing.assert_allclose(model.rate(model.current(rates)), rates, rtol=1e-9)
        np.testing.assert_array_equal(model.rate(np.array([0.0, 100.0, model.rheobase])), [0.0, 0.0, 0.0])
        self.assertLess(model.rate(np.array([1e9]))[0], 1000.0 / model.t_ref + 1e-6)          # the refractory time caps the rate

    def test_tuning_puts_threshold_and_maximum_rate_where_they_were_drawn(self):
        tuning = make_tuning("scalar", 40, seed=3)
        self.assertEqual((tuning.dims, tuning.n, set(tuning.encoders[:, 0])), (1, 40, {1.0, -1.0}))
        at_edge = tuning.rates(np.array([[1.0]]))[0][tuning.encoders[:, 0] > 0]
        self.assertTrue(np.all((at_edge >= 100.0 - 1e-6) & (at_edge <= 200.0 + 1e-6)))
        self.assertTrue(np.all(tuning.rates(np.array([[-1.0]]))[0][tuning.encoders[:, 0] > 0] == 0.0))
        again = make_tuning("scalar", 40, seed=3)
        np.testing.assert_array_equal(tuning.gain, again.gain)                                 # the seed fixes the population
        product = make_tuning("product", 8, seed=1)
        self.assertTrue(np.all(np.abs(product.encoders @ np.array([1.0, 1.0])) <= 1.0 + 1e-12))

    def test_readouts_of_each_kind_reach_their_target(self):
        design = Design()
        limits = {"scalar": 0.02, "product": 0.02, "phasor": 0.02}
        for kind, limit in limits.items():
            tuning = design.tuning(kind)
            x, y = SAMPLES[kind]()
            error = tuning.rates(x) @ analytic_decoders(tuning, design.regularisation) - y
            self.assertLess(np.sqrt(np.mean(error ** 2)), limit, kind)

    def test_more_regularisation_gives_smaller_weights(self):
        tuning = make_tuning("scalar", 60, seed=2)
        x, y = SAMPLES["scalar"]()
        small, large = solve_decoders(tuning.rates(x), y, 0.01), solve_decoders(tuning.rates(x), y, 0.3)
        self.assertGreater(np.linalg.norm(small), np.linalg.norm(large))


class MappingTests(unittest.TestCase):
    def test_circular_mapping_is_the_ring_angle(self):
        m = RingMap(200, -np.pi, np.pi, True)
        self.assertEqual((m.index(0.0), m.index(np.pi / 2), m.index(-np.pi / 2), m.index(2 * np.pi)), (0, 50, 150, 0))
        self.assertAlmostEqual(m.radians_per_index, 2 * np.pi / 200)
        for index in (0, 37, 150, 199):
            self.assertEqual(m.index(m.angle(index)), index)
        self.assertAlmostEqual(m.position(0.01), 0.01 / (2 * np.pi) * 200)
        self.assertAlmostEqual(m.held_angle(0.013, fractional=True), 0.013)
        self.assertAlmostEqual(m.held_angle(0.013, fractional=False), 0.0)
        self.assertAlmostEqual(m.index_error(1.0, 199.0), 2.0)

    def test_bounded_mapping_puts_the_limits_on_70_percent_of_the_ring(self):
        joint = get_chain("tiago7").joints[0]                       # arm_1: 0 .. 2.749 rad
        m = RingMap.for_joint(joint, 200)
        self.assertEqual((m.arc, m.wall()), ((30.0, 170.0), (15, 185)))
        self.assertEqual((m.index(joint.lower), m.index(joint.upper), m.index(99.0), m.index(-99.0)), (30, 170, 170, 30))
        self.assertAlmostEqual(m.angle(100.0), (joint.lower + joint.upper) / 2.0)
        angles = m.neuron_angles()
        self.assertAlmostEqual(angles[30], joint.lower)
        self.assertAlmostEqual(angles[25] - angles[30], -5 * m.radians_per_index)            # the line continues past the limit
        np.testing.assert_array_equal(wall_profile(200, 15, 185)[[0, 14, 15, 100, 185, 186]], [0.9, 0.9, 1.0, 1.0, 1.0, 0.9])

    def test_centre_of_activity(self):
        m = RingMap(200, -np.pi, np.pi, True)
        counts = np.zeros(200)
        counts[[198, 199, 0, 1, 2]] = [1, 2, 3, 2, 1]
        self.assertAlmostEqual(m.population_vector_index(counts) % 200, 0.0, places=6)
        self.assertTrue(np.isnan(m.population_vector_index(np.zeros(200))))
        bounded = RingMap(200, 0.0, 2.0, False)
        counts = np.zeros(200)
        counts[100:105] = 1
        self.assertAlmostEqual(bounded.population_vector_index(counts), 102.0, places=6)


class RingPortTests(unittest.TestCase):
    def test_weights_depend_only_on_the_distance_round_the_ring(self):
        w = ring_weight_matrix(200)
        self.assertEqual(w.shape, (200, 200))
        np.testing.assert_array_equal(w[0], np.roll(w[7], -7))                               # every neuron has the same profile
        np.testing.assert_array_equal(w, w.T)
        self.assertGreater(w[0, 1], 0.0)                                                     # neighbours excite
        self.assertLess(w[0, 20], -0.9)                                                      # the surround inhibits
        self.assertAlmostEqual(w[0, 100], 0.0, places=3)                                     # the far side is left alone
        distances = ring_distances(200)
        self.assertEqual((distances[0], distances[100], distances[1], distances[199]), (0.0, 50.0, 0.5, 0.5))

    def test_weights_and_neuron_equal_the_so2_repo_when_it_is_mounted(self):
        import json
        import os
        import sys

        package = "/so2/src/tiago_ring_controller/src"
        if not os.path.isdir(package):
            self.skipTest("the SO-2 repo is not mounted (docker/run.bash nest --so2 DIR)")
        sys.path.insert(0, package)
        try:
            for name in [m for m in sys.modules if m.startswith("tiago_ring_controller")]:
                del sys.modules[name]                     # the image carries an older copy of the package
            from tiago_ring_controller.math.ring import ring_weight_matrix as so2_matrix
        finally:
            sys.path.remove(package)
        for size in (100, 200):
            np.testing.assert_array_equal(ring_weight_matrix(size), so2_matrix(size, "legacy"))
        from ring_so3.nest.ring import RING_NEURON

        with open(os.path.join(package, "config", "model_params", "neuron_params.json"), encoding="utf-8") as stream:
            self.assertEqual(json.load(stream)["ring"], RING_NEURON)

    def test_stimulus_window_and_its_fractional_blend(self):
        whole = bump_rates(200, 3)
        self.assertEqual((int(np.count_nonzero(whole)), whole[198], whole[8], whole[9]), (11, 200.0, 200.0, 0.0))
        blend = fractional_bump_rates(200, 50.25)
        self.assertAlmostEqual(np.sum(blend * np.arange(200)) / np.sum(blend), 50.25)        # centred on the position itself
        self.assertAlmostEqual(blend[45], 150.0)
        self.assertAlmostEqual(blend[56], 50.0)
        np.testing.assert_array_equal(fractional_bump_rates(200, 50.0), bump_rates(200, 50))


class RateCopyTests(unittest.TestCase):
    def test_rate_copy_decodes_the_orientation_of_every_link(self):
        chain = get_chain("generic4")
        network = RateNetwork(chain_wiring(chain))
        rng = np.random.default_rng(8)
        errors = []
        for _ in range(25):
            q = chain.random_pose(rng)
            readout = network.decode(q)
            errors.append(orientation_error_deg(readout.tool, chain.rotation(q)))
            self.assertEqual(len(readout.links), 4)
            np.testing.assert_allclose(readout.held, q)
        self.assertLess(np.median(errors), 2.5)
        self.assertLess(np.max(errors), 5.0)

    def test_nearest_index_encoder_noise_and_small_populations_cost_accuracy(self):
        chain = get_chain("generic3")
        rng = np.random.default_rng(9)
        poses = [chain.random_pose(rng) for _ in range(40)]

        def median_error(network, **kwargs):
            return np.median([orientation_error_deg(network.decode(q, **kwargs).tool, chain.rotation(q)) for q in poses])

        network = RateNetwork(chain_wiring(chain))
        between = median_error(network)
        self.assertGreater(median_error(network, fractional=False), between)
        self.assertGreater(median_error(network, noise=0.05, rng=np.random.default_rng(1)), between)
        small = RateNetwork(chain_wiring(chain), Design(sizes=Sizes(product=50, entry=20, feature=40)))
        self.assertGreater(median_error(small), between)
        held = network.decode(poses[0], fractional=False).held
        np.testing.assert_allclose(held, [m.held_angle(a) for m, a in zip(network.maps, poses[0])])

    def test_all_at_once_wiring_runs_on_the_rate_copy_too(self):
        chain = get_chain("generic3")
        network = RateNetwork(flat_wiring(chain))
        q = [0.5, -1.0, 2.0]
        self.assertLess(orientation_error_deg(network.decode(q).tool, chain.rotation(q)), 4.0)


if __name__ == "__main__":
    unittest.main()

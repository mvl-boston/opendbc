import unittest

from opendbc.car.honda.nidec_long_helpers import NIDEC_BRAKE_BANDS, NIDEC_BRAKE_COAST_KI, NIDEC_BRAKE_COAST_LEARN_MAX_BRAKE, \
                                                NIDEC_BRAKE_COAST_MAX, NIDEC_BRAKE_COAST_MIN, NIDEC_BRAKE_GAIN_KI, NIDEC_BRAKE_GAIN_MAX, \
                                                NIDEC_BRAKE_GAIN_MIN, NIDEC_BRAKE_REDUCTION_FADE_START, NidecBrakeCoast, NidecBrakeGain, \
                                                nidec_band_weights, nidec_brake_coast_error, nidec_brake_scale

DT = 0.02


def uniform(value):
  return NidecBrakeGain({band: value for band, _ in NIDEC_BRAKE_BANDS})


class TestBandWeights(unittest.TestCase):
  def test_weights_sum_to_one_on_two_bracketing_nodes(self):
    for v in (0.0, 0.5, 0.9, 2.2, 6.0, 15.0, 24.9, 40.0):
      weights = nidec_band_weights(NIDEC_BRAKE_BANDS, v)
      self.assertAlmostEqual(sum(weights.values()), 1.0)
      self.assertLessEqual(sum(1 for w in weights.values() if w > 0.0), 2)

  def test_edge_nodes_own_everything_outside_the_grid(self):
    self.assertEqual(nidec_band_weights(NIDEC_BRAKE_BANDS, 0.1)[NIDEC_BRAKE_BANDS[0][0]], 1.0)
    self.assertEqual(nidec_band_weights(NIDEC_BRAKE_BANDS, 35.0)[NIDEC_BRAKE_BANDS[-1][0]], 1.0)

  def test_midpoint_splits_evenly(self):
    weights = nidec_band_weights(NIDEC_BRAKE_BANDS, 1.0)
    self.assertAlmostEqual(weights["005"], 0.5)
    self.assertAlmostEqual(weights["015"], 0.5)


class TestNidecBrakeGain(unittest.TestCase):
  def test_uniform_seed_is_the_old_single_gain_at_every_speed(self):
    gain = uniform(0.3)
    for v in (0.0, 1.0, 4.0, 10.0, 30.0):
      self.assertAlmostEqual(gain.at(v), 0.3)

  def test_error_at_a_node_only_moves_that_node(self):
    gain = uniform(0.3)
    gain.update(0.5, 0.5, DT)
    self.assertAlmostEqual(gain.values["005"], 0.3 + NIDEC_BRAKE_GAIN_KI * 0.5 * DT)
    for band, _ in NIDEC_BRAKE_BANDS[1:]:
      self.assertEqual(gain.values[band], 0.3)

  def test_low_speed_learning_leaves_highway_gain_alone(self):
    gain = uniform(0.3)
    for _ in range(500):
      gain.update(-0.2, 1.0, DT)
    self.assertLess(gain.at(1.0), 0.0)
    self.assertAlmostEqual(gain.at(20.0), 0.3)

  def test_total_learning_rate_matches_the_single_integrator(self):
    gain = uniform(0.0)
    gain.update(1.0, 6.5, DT)
    self.assertAlmostEqual(sum(gain.values.values()), NIDEC_BRAKE_GAIN_KI * DT)

  def test_factors_stay_inside_the_limits(self):
    gain = uniform(0.0)
    for _ in range(20000):
      gain.update(1.0, 3.0, DT)
    self.assertEqual(gain.values["030"], NIDEC_BRAKE_GAIN_MAX)
    for _ in range(40000):
      gain.update(-1.0, 3.0, DT)
    self.assertEqual(gain.values["030"], NIDEC_BRAKE_GAIN_MIN)

  def test_stored_values_outside_the_limits_are_clamped(self):
    self.assertEqual(uniform(9.0).at(5.0), NIDEC_BRAKE_GAIN_MAX)
    self.assertEqual(uniform(-3.0).at(5.0), NIDEC_BRAKE_GAIN_MIN)


class TestNidecBrakeCoast(unittest.TestCase):
  def coast(self, value=0.3):
    return NidecBrakeCoast({band: value for band, _ in NIDEC_BRAKE_BANDS})

  def test_more_decel_than_commanded_raises_the_coast_decel(self):
    coast = self.coast()
    coast.update(0.4, 5.0, DT)
    self.assertAlmostEqual(coast.at(5.0), 0.3 + NIDEC_BRAKE_COAST_KI * 0.4 * DT)

  def test_less_decel_than_commanded_lowers_it(self):
    coast = self.coast()
    coast.update(-0.4, 5.0, DT)
    self.assertLess(coast.at(5.0), 0.3)

  def test_other_speeds_are_untouched(self):
    coast = self.coast()
    for _ in range(200):
      coast.update(0.4, 5.0, DT)
    self.assertAlmostEqual(coast.at(1.5), 0.3)
    self.assertAlmostEqual(coast.at(17.0), 0.3)

  def test_never_negative_and_bounded(self):
    coast = self.coast()
    for _ in range(20000):
      coast.update(-1.0, 0.5, DT)
    self.assertEqual(coast.at(0.5), NIDEC_BRAKE_COAST_MIN)
    self.assertGreaterEqual(NIDEC_BRAKE_COAST_MIN, 0.0)
    for _ in range(40000):
      coast.update(1.0, 0.5, DT)
    self.assertEqual(coast.at(0.5), NIDEC_BRAKE_COAST_MAX)


class TestBrakeCoastError(unittest.TestCase):
  def test_nothing_without_a_brake_request(self):
    self.assertEqual(nidec_brake_coast_error(0.4, 0.0, False, True), 0.0)
    self.assertEqual(nidec_brake_coast_error(-0.4, 0.2, False, True), 0.0)

  def test_light_braking_counts_and_fades_to_nothing_at_heavy_braking(self):
    self.assertAlmostEqual(nidec_brake_coast_error(0.4, 0.01, True, True), 0.4 * 0.98)
    self.assertAlmostEqual(nidec_brake_coast_error(-0.4, NIDEC_BRAKE_COAST_LEARN_MAX_BRAKE / 2, True, True), -0.2)
    self.assertEqual(nidec_brake_coast_error(0.4, NIDEC_BRAKE_COAST_LEARN_MAX_BRAKE, True, True), 0.0)
    self.assertEqual(nidec_brake_coast_error(0.4, 1.0, True, True), 0.0)

  def test_absorbed_request_only_lowers_the_coast_decel(self):
    self.assertEqual(nidec_brake_coast_error(-0.4, 0.0, True, True), -0.4)
    self.assertEqual(nidec_brake_coast_error(0.4, 0.0, True, True), 0.0)

  def test_absorbed_request_needs_the_gas_wire_at_zero(self):
    self.assertEqual(nidec_brake_coast_error(-0.4, 0.0, True, False), 0.0)


class TestBrakeScale(unittest.TestCase):
  def test_gain_at_or_above_one_is_applied_as_is(self):
    self.assertAlmostEqual(nidec_brake_scale(0.0, 0.2), 1.0)
    self.assertAlmostEqual(nidec_brake_scale(0.6, 0.2), 1.6)
    self.assertAlmostEqual(nidec_brake_scale(0.6, 1.0), 1.6)

  def test_reduction_applies_in_full_to_light_braking(self):
    self.assertAlmostEqual(nidec_brake_scale(-0.3, 0.1), 0.7)
    self.assertAlmostEqual(nidec_brake_scale(-0.3, NIDEC_BRAKE_REDUCTION_FADE_START), 0.7)

  def test_reduction_fades_out_and_is_gone_at_a_full_request(self):
    self.assertAlmostEqual(nidec_brake_scale(-0.3, 0.75), 0.85)
    self.assertAlmostEqual(nidec_brake_scale(-0.3, 1.0), 1.0)
    self.assertAlmostEqual(nidec_brake_scale(NIDEC_BRAKE_GAIN_MIN, 1.0), 1.0)

  def test_applied_brake_never_decreases_as_the_request_grows(self):
    last = 0.0
    for i in range(101):
      brake = i / 100
      applied = brake * nidec_brake_scale(NIDEC_BRAKE_GAIN_MIN, brake)
      self.assertGreaterEqual(applied, last)
      last = applied


if __name__ == "__main__":
  unittest.main()

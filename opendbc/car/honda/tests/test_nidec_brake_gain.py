import unittest

from opendbc.car.honda.nidec_long_helpers import NIDEC_ACCEL_SETTLE_TIME, NIDEC_BRAKE_ACCEL_SCALE, NIDEC_BRAKE_BANDS, NIDEC_BRAKE_COAST_GAS_OFF_MIN, \
                                                NIDEC_BRAKE_COAST_KI, NIDEC_BRAKE_COAST_LEARN_MAX_BRAKE, NIDEC_BRAKE_COAST_MAX, NIDEC_BRAKE_COAST_MIN, \
                                                NIDEC_BRAKE_CREEP_MAX, NIDEC_BRAKE_CREEP_SPEED, NIDEC_BRAKE_GAIN_KI, NIDEC_BRAKE_GAIN_MAX, \
                                                NIDEC_BRAKE_GAIN_MIN, NIDEC_STOPPING_BRAKE_RATE, NIDEC_STOPPING_HOLD_RATE, NidecAccelValid, \
                                                NidecBrakeCoast, NidecBrakeGain, nidec_band_weights, nidec_brake_after_coast, \
                                                nidec_brake_coast_error, nidec_brake_scale, nidec_stopping_brake, nidec_stopping_hold

DT = 0.02
GAS_OFF = NIDEC_BRAKE_COAST_GAS_OFF_MIN


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
    self.assertEqual(gain.at(1.0), 0.0)
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
    self.assertEqual(nidec_brake_scale(-0.5, 0.1), 1.0)


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

  def test_bounded_and_negative_only_at_creep_speeds(self):
    for band, speed in NIDEC_BRAKE_BANDS:
      coast = self.coast()
      for _ in range(20000):
        coast.update(-1.0, speed, DT)
      self.assertEqual(coast.values[band], -NIDEC_BRAKE_CREEP_MAX if speed < NIDEC_BRAKE_CREEP_SPEED else NIDEC_BRAKE_COAST_MIN)
      for _ in range(40000):
        coast.update(1.0, speed, DT)
      self.assertEqual(coast.values[band], NIDEC_BRAKE_COAST_MAX)
    self.assertEqual(NIDEC_BRAKE_COAST_MIN, 0.0)
    self.assertEqual([band for band, speed in NIDEC_BRAKE_BANDS if speed < NIDEC_BRAKE_CREEP_SPEED], ["005", "015"])

  def test_stored_values_load_inside_their_bounds(self):
    coast = NidecBrakeCoast({band: -2.0 for band, _ in NIDEC_BRAKE_BANDS})
    self.assertEqual(coast.values["005"], -NIDEC_BRAKE_CREEP_MAX)
    self.assertEqual(coast.values["030"], 0.0)

  def test_creep_is_learned_from_a_stop_that_brakes_too_little(self):
    # 0.3 m/s2 asked for at 0.7 m/s, the car is pushed 0.35 m/s2 by creep: the brake has to add it
    coast = self.coast(0.0)
    for _ in range(3000):
      brake = nidec_brake_after_coast(0.3 / NIDEC_BRAKE_ACCEL_SCALE, coast.at(0.7))
      a_ego = 0.35 - brake * NIDEC_BRAKE_ACCEL_SCALE
      coast.update(nidec_brake_coast_error(-0.3 - a_ego, brake, True, GAS_OFF), 0.7, DT)
    self.assertAlmostEqual(coast.at(0.7), -0.35, places=2)
    self.assertAlmostEqual(nidec_brake_after_coast(0.3 / NIDEC_BRAKE_ACCEL_SCALE, coast.at(0.7)) * NIDEC_BRAKE_ACCEL_SCALE, 0.65, places=2)


class TestBrakeAfterCoast(unittest.TestCase):
  def test_coast_decel_comes_off_the_request(self):
    self.assertAlmostEqual(nidec_brake_after_coast(0.2, 0.48), 0.1)
    self.assertEqual(nidec_brake_after_coast(0.05, 0.48), 0.0)

  def test_creep_adds_to_a_request(self):
    self.assertAlmostEqual(nidec_brake_after_coast(0.05, -0.48), 0.15)
    self.assertEqual(nidec_brake_after_coast(0.95, -0.48), 1.0)

  def test_creep_never_brakes_without_a_request(self):
    self.assertEqual(nidec_brake_after_coast(0.0, -NIDEC_BRAKE_CREEP_MAX), 0.0)
    self.assertEqual(nidec_brake_after_coast(0.0, 0.5), 0.0)


class TestStoppingBrake(unittest.TestCase):
  STEP = DT / NIDEC_BRAKE_ACCEL_SCALE

  def test_hold_is_creep_plus_a_downhill_grade(self):
    self.assertAlmostEqual(nidec_stopping_hold(0.0), 0.15)
    self.assertAlmostEqual(nidec_stopping_hold(-0.48), 0.25)
    self.assertAlmostEqual(nidec_stopping_hold(0.48), 0.15)

  def test_rises_quickly_up_to_the_hold(self):
    self.assertAlmostEqual(nidec_stopping_brake(0.4, 0.05, 0.15, DT), 0.05 + NIDEC_STOPPING_HOLD_RATE * self.STEP)
    self.assertAlmostEqual(nidec_stopping_brake(0.4, 0.149, 0.15, DT), 0.15)

  def test_rises_slowly_above_the_hold(self):
    self.assertAlmostEqual(nidec_stopping_brake(0.4, 0.15, 0.15, DT), 0.15 + NIDEC_STOPPING_BRAKE_RATE * self.STEP)
    self.assertAlmostEqual(nidec_stopping_brake(0.4, 0.30, 0.15, DT), 0.30 + NIDEC_STOPPING_BRAKE_RATE * self.STEP)

  def test_never_above_the_request(self):
    self.assertEqual(nidec_stopping_brake(0.052, 0.05, 0.15, DT), 0.052)
    self.assertEqual(nidec_stopping_brake(0.0, 0.3, 0.15, DT), 0.0)

  def test_weak_brake_at_the_stop_reaches_the_hold_within_a_second(self):
    # route 2b4 t=859: the stopping state began with 17 counts of 256 on the wire
    brake, frames = 17 / 256, 0
    while brake < nidec_stopping_hold(0.0):
      brake = nidec_stopping_brake(1.0, brake, nidec_stopping_hold(0.0), DT)
      frames += 1
    self.assertLess(frames * DT, 0.5)
    self.assertLess(nidec_stopping_brake(1.0, brake, nidec_stopping_hold(0.0), DT) - brake, 2 * NIDEC_STOPPING_BRAKE_RATE * self.STEP)


class TestAccelValid(unittest.TestCase):
  def run_speeds(self, valid, speeds):
    return [valid.update(v) for v in speeds]

  def test_not_valid_until_the_speed_has_read_for_the_settle_time(self):
    valid = NidecAccelValid(0.01)
    frames = round(NIDEC_ACCEL_SETTLE_TIME / 0.01)
    out = self.run_speeds(valid, [0.36] * (frames + 5))
    self.assertFalse(any(out[:frames - 1]))
    self.assertTrue(all(out[frames:]))

  def test_drop_out_is_invalid_at_once(self):
    valid = NidecAccelValid(0.01)
    self.run_speeds(valid, [1.0] * 100)
    self.assertTrue(valid.update(0.34))
    self.assertFalse(valid.update(0.0))

  def test_sensor_chatter_at_the_floor_never_becomes_valid(self):
    # route 2b4 t=470.9-471.4: 0.36 -> 0 -> 0.36 -> 0 inside half a second
    valid = NidecAccelValid(0.01)
    self.run_speeds(valid, [1.0] * 100)
    out = self.run_speeds(valid, [0.0] * 40 + [0.36] * 10 + [0.0] * 30)
    self.assertFalse(any(out))


class TestBrakeCoastError(unittest.TestCase):
  def test_nothing_without_a_brake_request(self):
    self.assertEqual(nidec_brake_coast_error(0.4, 0.0, False, GAS_OFF), 0.0)
    self.assertEqual(nidec_brake_coast_error(-0.4, 0.2, False, GAS_OFF), 0.0)

  def test_light_braking_counts_and_fades_to_nothing_at_heavy_braking(self):
    self.assertAlmostEqual(nidec_brake_coast_error(0.4, 0.01, True, GAS_OFF), 0.4 * 0.98)
    self.assertAlmostEqual(nidec_brake_coast_error(-0.4, NIDEC_BRAKE_COAST_LEARN_MAX_BRAKE / 2, True, GAS_OFF), -0.2)
    self.assertEqual(nidec_brake_coast_error(0.4, NIDEC_BRAKE_COAST_LEARN_MAX_BRAKE, True, GAS_OFF), 0.0)
    self.assertEqual(nidec_brake_coast_error(0.4, 1.0, True, GAS_OFF), 0.0)

  def test_absorbed_request_only_lowers_the_coast_decel(self):
    self.assertEqual(nidec_brake_coast_error(-0.4, 0.0, True, GAS_OFF), -0.4)
    self.assertEqual(nidec_brake_coast_error(0.4, 0.0, True, GAS_OFF), 0.0)

  def test_nothing_while_the_engine_is_still_running_down(self):
    for gas_off_time in (0.0, 0.5, NIDEC_BRAKE_COAST_GAS_OFF_MIN - DT):
      self.assertEqual(nidec_brake_coast_error(-0.4, 0.0, True, gas_off_time), 0.0)
      self.assertEqual(nidec_brake_coast_error(-0.4, 0.1, True, gas_off_time), 0.0)
      self.assertEqual(nidec_brake_coast_error(0.4, 0.1, True, gas_off_time), 0.0)

  def test_gas_cut_run_down_does_not_lower_the_coast_decel(self):
    # gas cut at t=0 with a -0.3 m/s2 command: the car goes from +0.4 to its 0.6 m/s2 coast decel
    # over about a second, the request is swallowed whole the entire time
    coast = NidecBrakeCoast({band: 0.6 for band, _ in NIDEC_BRAKE_BANDS})
    for i in range(150):
      t = i * DT
      a_ego = -0.6 + 1.0 * max(1.0 - t / 1.2, 0.0)
      coast.update(nidec_brake_coast_error(-0.3 - a_ego, 0.0, True, t), 15.0, DT)
    self.assertAlmostEqual(coast.at(15.0), 0.6)


class TestBrakeScale(unittest.TestCase):
  def test_gain_is_one_plus_factor_and_never_below_one(self):
    self.assertAlmostEqual(nidec_brake_scale(0.0, 0.2), 1.0)
    self.assertAlmostEqual(nidec_brake_scale(0.6, 0.2), 1.6)
    self.assertAlmostEqual(nidec_brake_scale(0.6, 1.0), 1.6)
    self.assertAlmostEqual(nidec_brake_scale(-0.3, 0.1), 1.0)
    self.assertGreaterEqual(NIDEC_BRAKE_GAIN_MIN, 0.0)


if __name__ == "__main__":
  unittest.main()

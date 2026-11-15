import unittest

from opendbc.car.honda.nidec_long_helpers import DelayLine, NIDEC_GAS_RECOVERY_TICKS, NIDEC_GAS_STAGE, NIDEC_GAS_STAGE_MIN_ACCEL, \
                                                NIDEC_GAS_UNWOUND_TICKS, nidec_gas_staged, nidec_gas_unwound, \
                                                nidec_speed_lead_mps, nidec_speed_channel_pinned, nidec_set_speed_governed


class TestNidecSpeedChannelPinned(unittest.TestCase):
  def test_positive_knee(self):
    self.assertTrue(nidec_speed_channel_pinned(11.0, 10.0))
    self.assertFalse(nidec_speed_channel_pinned(5.0, 7.0))

  def test_coast_floor_mirror(self):
    self.assertTrue(nidec_speed_channel_pinned(-3.0, 10.0))
    self.assertFalse(nidec_speed_channel_pinned(-2.0, 10.0))
    self.assertFalse(nidec_speed_channel_pinned(0.0, 10.0))

  def test_crv_coast_growth_case_from_route_25(self):
    # CR-V 2015 route a6354c3dcf16e7cf/25: plan -0.5 m/s2 on a -0.45 coast floor. The undershoot is
    # permanent and err*accel is positive growth; it must be read as pinned once the lead is past
    # the coast knee, which with the default factor (4) it is not and with a grown one (6) it is.
    accel_lag = -0.5
    self.assertFalse(nidec_speed_channel_pinned(4.0 * accel_lag, 10.0))
    self.assertTrue(nidec_speed_channel_pinned(6.0 * accel_lag, 10.0))


class TestGasStaging(unittest.TestCase):
  def staged(self, wire=0, recovery_ticks=NIDEC_GAS_RECOVERY_TICKS, accel=0.3, v_ego=5.0, braking=False, gas_pressed=False,
             launch_active=False):
    return nidec_gas_staged(wire, recovery_ticks, accel, v_ego, braking, gas_pressed, launch_active, 198)

  def unwound(self, zero_ticks=NIDEC_GAS_UNWOUND_TICKS, wire_target=40, accel=0.3, v_ego=5.0, braking=False, gas_pressed=False,
              launch_active=False):
    return nidec_gas_unwound(zero_ticks, wire_target, accel, v_ego, braking, gas_pressed, launch_active)

  def test_window_holds_the_wire_at_the_stage(self):
    self.assertEqual(self.staged(wire=0), NIDEC_GAS_STAGE)
    self.assertEqual(self.staged(wire=35), NIDEC_GAS_STAGE)

  def test_feedforward_above_the_stage_is_untouched(self):
    self.assertEqual(self.staged(wire=150), 150)
    self.assertEqual(self.staged(wire=198), 198)

  def test_no_stage_outside_a_window(self):
    self.assertEqual(self.staged(wire=35, recovery_ticks=0), 35)

  def test_no_stage_without_a_positive_plan(self):
    self.assertEqual(self.staged(accel=NIDEC_GAS_STAGE_MIN_ACCEL), 0)
    self.assertEqual(self.staged(accel=-0.4), 0)

  def test_no_stage_with_brake_driver_gas_launch_or_standstill(self):
    self.assertEqual(self.staged(braking=True), 0)
    self.assertEqual(self.staged(gas_pressed=True), 0)
    self.assertEqual(self.staged(launch_active=True), 0)
    self.assertEqual(self.staged(v_ego=0.0), 0)

  def test_stage_never_exceeds_the_gas_limit(self):
    self.assertEqual(nidec_gas_staged(0, 1, 0.3, 5.0, False, False, False, 90), 90)

  def test_coast_restart_opens_a_window(self):
    self.assertTrue(self.unwound())
    self.assertFalse(self.unwound(zero_ticks=NIDEC_GAS_UNWOUND_TICKS - 1))

  def test_no_window_while_the_plan_is_not_asking_for_accel(self):
    self.assertFalse(self.unwound(accel=0.0))
    self.assertFalse(self.unwound(wire_target=0))
    self.assertFalse(self.unwound(accel=-0.5))

  def test_no_window_with_brake_driver_gas_launch_or_standstill(self):
    self.assertFalse(self.unwound(braking=True))
    self.assertFalse(self.unwound(gas_pressed=True))
    self.assertFalse(self.unwound(launch_active=True))
    self.assertFalse(self.unwound(v_ego=0.05))


class TestDelayLine(unittest.TestCase):
  def test_returns_value_from_delay_ago(self):
    line = DelayLine(0.25, 0.01)
    self.assertEqual(line.frames, 25)
    out = [line.push(float(i)) for i in range(1, 60)]
    self.assertEqual(out[:25], [0.0] * 25)
    self.assertEqual(out[25], 1.0)
    self.assertEqual(out[58], 34.0)

  def test_half_rate_line_spans_the_same_time(self):
    self.assertEqual(DelayLine(0.25, 0.02).frames, 12)
    self.assertEqual(DelayLine(0.15, 0.02).frames, 8)

  def test_zero_delay_is_passthrough(self):
    line = DelayLine(0.0, 0.01)
    self.assertEqual(line.push(3.0), 3.0)
    self.assertEqual(line.push(-1.0), -1.0)

  def test_reset_fills_history(self):
    line = DelayLine(0.05, 0.01)
    for i in range(10):
      line.push(float(i))
    line.reset(-0.7)
    self.assertEqual([line.push(1.0) for _ in range(5)], [-0.7] * 5)
    self.assertEqual(line.push(1.0), 1.0)

  def test_tuple_values(self):
    line = DelayLine(0.04, 0.02, (0.0, 0.0))
    self.assertEqual(line.push((-1.0, 0.2)), (0.0, 0.0))
    self.assertEqual(line.push((-2.0, 0.4)), (0.0, 0.0))
    self.assertEqual(line.push((-2.0, 0.4)), (-1.0, 0.2))

  def test_brake_step_does_not_wind_the_integrator(self):
    # route 00000250 13:03:22 ET shape: plan steps to -2.2, the car follows 0.25 s later. Integrating
    # plan - aEgo winds ~ -0.4 during the dead time; against the delayed plan there is nothing to wind.
    dt, delay, ki = 0.01, 0.25, 0.76
    line = DelayLine(delay, dt)
    plan = [0.0] * 50 + [-2.2] * 100
    a_ego = [0.0] * 75 + [-2.2] * 75
    i_now = i_delayed = 0.0
    for p, a in zip(plan, a_ego, strict=True):
      i_now += ki * dt * (p - a)
      i_delayed += ki * dt * (line.push(p) - a)
    self.assertLess(i_now, -0.4)
    self.assertAlmostEqual(i_delayed, 0.0, places=6)


class TestNidecSetSpeedGoverned(unittest.TestCase):
  def test_crv_route_25_hold_zone(self):
    # 116.5 kph actual against a 120 kph display: the PCM, not the wire, held the car
    self.assertTrue(nidec_set_speed_governed(32.4, 33.3))
    self.assertTrue(nidec_set_speed_governed(33.3, 33.3))

  def test_below_margin_is_servo_territory(self):
    self.assertFalse(nidec_set_speed_governed(30.0, 33.3))
    self.assertFalse(nidec_set_speed_governed(31.2, 33.3))


class TestNidecSpeedLead(unittest.TestCase):
  def test_poisoned_alpha_floor_from_route_21_seg62(self):
    # Ridgeline route 7862788bd18c1a53/21 seg 62: cmd ~0.17, sf_low ~2.18, alpha ~-0.66 -> lead -0.29
    sf = 2.177781581878662
    accel = 0.17166689038276672
    alpha = -0.6604037894504735
    lead = nidec_speed_lead_mps(sf, accel, alpha)
    self.assertGreaterEqual(lead, 0.35)
    self.assertAlmostEqual(lead, 0.35, places=2)

  def test_coasting_plan_unchanged(self):
    self.assertAlmostEqual(nidec_speed_lead_mps(2.0, 0.0, -0.5), -0.5)

  def test_healthy_alpha_unchanged(self):
    self.assertAlmostEqual(nidec_speed_lead_mps(2.5, 0.4, 0.2), 1.2)


if __name__ == "__main__":
  unittest.main()

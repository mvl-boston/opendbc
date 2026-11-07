import unittest

from opendbc.car.honda.nidec_long_helpers import nidec_speed_lead_mps, nidec_speed_channel_pinned, nidec_set_speed_governed


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

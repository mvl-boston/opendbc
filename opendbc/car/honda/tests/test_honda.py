import unittest

from opendbc.car.honda.values import CAR, CarControllerParams, HondaFlags, HONDA_BOSCH, HONDA_BOSCH_ACCEL_ENVELOPE, bosch_accel_max


class TestHondaFingerprint(unittest.TestCase):
  def test_tja_bosch_only(self):
    for car_model in CAR:
      if car_model.config.flags & HondaFlags.BOSCH_TJA_CONTROL:
        assert car_model.config.flags & HondaFlags.BOSCH, "Nidec car found with TJA control"


class TestBoschAccelMax(unittest.TestCase):
  def test_envelope_platforms_are_bosch(self):
    assert HONDA_BOSCH_ACCEL_ENVELOPE <= HONDA_BOSCH

  def test_prelude_below_pcm_dropout_requests(self):
    # route acd27eb0b6ecb6dd/00000197--22b2397846: the PCM stopped honoring ACCEL_COMMAND after the
    # request sat at 2.0 m/s2 through 16-22.4 m/s (t=389.6) and 14-19.2 m/s (t=1714.1)
    for v_ego in (14.0, 16.0, 19.2, 22.4):
      assert bosch_accel_max(CAR.HONDA_PRELUDE_6G, v_ego) <= 1.1

  def test_prelude_envelope_shape(self):
    limits = [bosch_accel_max(CAR.HONDA_PRELUDE_6G, v) for v in (0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 60.0)]
    assert limits[0] == 1.6
    assert limits[-1] == 0.6
    assert limits == sorted(limits, reverse=True)
    assert all(0.0 < limit <= CarControllerParams.BOSCH_ACCEL_MAX for limit in limits)

  def test_other_bosch_unchanged(self):
    for car_model in HONDA_BOSCH - HONDA_BOSCH_ACCEL_ENVELOPE:
      for v_ego in (0.0, 20.0, 40.0):
        assert bosch_accel_max(car_model, v_ego) == CarControllerParams.BOSCH_ACCEL_MAX

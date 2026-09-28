import unittest

import numpy as np

from opendbc.car import DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.honda.lateral_model import (FF_CORRECTION_MAX, FILTER_TAU, GAIN_BINS_MPH, GAIN_KEY_FMT, GAIN_MAX, GAIN_MIN,
                                             GAIN_PRIOR, PRESS_HOLDOFF, HondaLateralModel)

LAF = 1.8


def make_model(params=None):
  store = dict(params or {})
  return HondaLateralModel(LAF, store.get)


def step(model, request, wire, v_ego, desired_la, actual_la, lat_active=True, steer_control_active=True, pressed=False):
  v_sq = v_ego * v_ego
  return model.update(request, wire, lat_active, steer_control_active, pressed, v_ego, desired_la / v_sq, actual_la / v_sq)


def drive(model, v_ego, gain_true, seconds, wire_fn, pressed=False, lat_active=True):
  """Constant-speed drive on a first-order plant lat_accel = gain_true * wire (0.5 s lag). Torque sign
  convention: right positive; curvature is left positive, so measured curvature = -lat_accel / v^2."""
  la = 0.0
  a = np.exp(-DT_CTRL / 0.5)
  for k in range(int(seconds / DT_CTRL)):
    wire = wire_fn(k * DT_CTRL)
    la = a * la + (1 - a) * gain_true * wire
    step(model, wire, wire, v_ego, -la, -la, pressed=pressed, lat_active=lat_active)
  return la


class TestHondaLateralModel(unittest.TestCase):
  def test_prior_and_interpolation(self):
    model = make_model()
    for mph, prior in zip(GAIN_BINS_MPH, GAIN_PRIOR, strict=True):
      self.assertAlmostEqual(model.gain(mph * CV.MPH_TO_MS), prior)
    # linear between bins, clamped outside
    mid = 12.5 * CV.MPH_TO_MS
    self.assertAlmostEqual(model.gain(mid), 0.5 * (GAIN_PRIOR[1] + GAIN_PRIOR[2]))
    self.assertAlmostEqual(model.gain(0.0), GAIN_PRIOR[0])
    self.assertAlmostEqual(model.gain(60.0), GAIN_PRIOR[-1])

  def test_persisted_values_load_and_clip(self):
    params = {GAIN_KEY_FMT.format(slot=30): 1.23, GAIN_KEY_FMT.format(slot=70): 9.0, GAIN_KEY_FMT.format(slot=5): -1.0}
    model = make_model(params)
    self.assertAlmostEqual(model.gain(30 * CV.MPH_TO_MS), 1.23)
    self.assertAlmostEqual(model.gain(70 * CV.MPH_TO_MS), GAIN_MAX)
    self.assertAlmostEqual(model.gain(5 * CV.MPH_TO_MS), GAIN_MIN)
    self.assertAlmostEqual(model.gain(40 * CV.MPH_TO_MS), GAIN_PRIOR[GAIN_BINS_MPH.index(40)])
    self.assertEqual(set(model.learned_values()), set(HondaLateralModel.param_keys()))
    self.assertEqual(len(model.param_keys()), len(GAIN_BINS_MPH))

  def test_feedforward_correction_low_speed_adds_torque_into_the_turn(self):
    model = make_model()
    v = 8.0
    desired_la = 0.5   # left turn: positive lateral accel, positive curvature, negative torque
    out = step(model, -desired_la / LAF, 0.0, v, desired_la, 0.0)
    expected = -desired_la / model.gain(v) + desired_la / LAF
    self.assertLess(expected, -0.3)   # the car needs a lot more than latAccelFactor says at 8 m/s
    self.assertAlmostEqual(model.ff_correction, expected)
    self.assertAlmostEqual(out, -desired_la / LAF + expected)
    # mirror image to the right
    out_r = step(model, desired_la / LAF, 0.0, v, -desired_la, 0.0)
    self.assertAlmostEqual(out_r, -out)

  def test_feedforward_correction_highway_is_small(self):
    model = make_model()
    v = 28.0
    desired_la = 1.0
    step(model, -desired_la / LAF, 0.0, v, desired_la, 0.0)
    self.assertLess(abs(model.ff_correction), 0.15)

  def test_feedforward_correction_is_bounded(self):
    model = make_model()
    step(model, -1.0, 0.0, 6.0, 2.5, 0.0)
    self.assertAlmostEqual(model.ff_correction, -FF_CORRECTION_MAX)

  def test_feedback_passes_through_with_unity_gain(self):
    # the correction depends on the desired curvature only: two different controller outputs at the same
    # operating point differ at the wire by exactly their difference (no loop-gain change)
    model = make_model()
    v = 10.0
    out_a = step(model, -0.20, 0.0, v, 0.4, 0.3)
    out_b = step(model, -0.35, 0.0, v, 0.4, 0.3)
    self.assertAlmostEqual(out_b - out_a, -0.15)

  def test_inactive_passes_request_through(self):
    model = make_model()
    out = step(model, 0.3, 0.0, 10.0, 0.5, 0.0, lat_active=False)
    self.assertEqual(out, 0.3)
    self.assertEqual(model.ff_correction, 0.0)
    self.assertFalse(model.learning)

  def test_output_is_clipped_to_unit_torque(self):
    model = make_model()
    out = step(model, -0.9, 0.0, 8.0, 1.0, 0.0)
    self.assertEqual(out, -1.0)
    # applied_correction is what the clip let through, so request + applied_correction == output always
    self.assertAlmostEqual(model.applied_correction, -0.1)
    self.assertLess(model.ff_correction, model.applied_correction)

  def test_applied_correction_matches_output_minus_request(self):
    model = make_model()
    for request, v, desired_la in ((-0.2, 8.0, 0.5), (0.3, 20.0, -0.8), (0.0, 30.0, 0.2)):
      out = step(model, request, 0.0, v, desired_la, 0.0)
      self.assertAlmostEqual(out - request, model.applied_correction)
      self.assertAlmostEqual(model.applied_correction, model.ff_correction)
    step(model, 0.3, 0.0, 10.0, 0.5, 0.0, lat_active=False)
    self.assertEqual(model.applied_correction, 0.0)

  def test_identifies_plant_gain(self):
    model = make_model()
    v = 12.0
    g_true = 2.2
    self.assertNotAlmostEqual(model.gain(v), g_true, delta=0.5)
    # alternate a held torque left/right so both signs and the filters settle well inside each dwell
    drive(model, v, g_true, 240.0, lambda t: 0.5 if (t // 15.0) % 2 == 0 else -0.5)
    self.assertAlmostEqual(model.gain(v), g_true, delta=0.1)
    self.assertTrue(model.learning)

  def test_identification_ignores_saturation_and_rate_limits(self):
    # regressing on the actual wire means a pinned wire is still a valid sample
    model = make_model()
    v = 12.0
    g_true = 0.9
    drive(model, v, g_true, 200.0, lambda t: 1.0 if (t // 20.0) % 2 == 0 else -1.0)
    self.assertAlmostEqual(model.gain(v), g_true, delta=0.1)

  def test_identification_pauses_while_pressed_and_for_holdoff(self):
    model = make_model()
    v = 12.0
    before = list(model.gains)
    drive(model, v, 2.5, 30.0, lambda t: 0.6, pressed=True)
    self.assertEqual(model.gains, before)
    # release: stays paused for the hold-off
    la = 2.5 * 0.6
    for _ in range(int(PRESS_HOLDOFF / DT_CTRL) - 2):
      step(model, 0.6, 0.6, v, -la, -la)
      self.assertFalse(model.learning)
    for _ in range(int(0.2 / DT_CTRL)):
      step(model, 0.6, 0.6, v, -la, -la)
    self.assertTrue(model.learning)

  def test_identification_needs_excitation_and_sign_agreement(self):
    model = make_model()
    v = 12.0
    before = list(model.gains)
    drive(model, v, 1.5, 20.0, lambda t: 0.05)                      # too little torque
    self.assertEqual(model.gains, before)
    la = 0.5
    for _ in range(int(5 * FILTER_TAU / DT_CTRL)):
      step(model, 0.5, 0.5, v, la, la)                              # car turning the wrong way: driver or slip
    self.assertEqual(model.gains, before)
    self.assertFalse(model.learning)

  def test_gains_stay_bounded(self):
    model = make_model()
    v = 12.0
    drive(model, v, 20.0, 120.0, lambda t: 0.5 if (t // 15.0) % 2 == 0 else -0.5)
    self.assertLessEqual(max(model.gains), GAIN_MAX)
    drive(model, v, 0.01, 120.0, lambda t: 0.5 if (t // 15.0) % 2 == 0 else -0.5)
    self.assertGreaterEqual(min(model.gains), GAIN_MIN)

  def test_learning_is_local_in_speed(self):
    model = make_model()
    drive(model, 12.0, 2.2, 200.0, lambda t: 0.5 if (t // 15.0) % 2 == 0 else -0.5)
    for mph, prior in zip(GAIN_BINS_MPH, GAIN_PRIOR, strict=True):
      if mph in (20, 30):
        continue
      self.assertAlmostEqual(model.gain(mph * CV.MPH_TO_MS), prior)


if __name__ == "__main__":
  unittest.main()

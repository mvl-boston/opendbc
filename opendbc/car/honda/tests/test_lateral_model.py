import unittest

import numpy as np

from opendbc.car import DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.honda.lateral_model import (DEFAULT_LAT_ACCEL_FACTOR, FF_CORRECTION_MAX, FF_SATURATION_FADE, FILTER_TAU,
                                             GAIN_BINS_MPH, GAIN_KEY_FMT, GAIN_MAX, GAIN_MIN, GAIN_PRIOR, PRESS_HOLDOFF,
                                             SHAPE_ANCHOR_LAT_ACCEL, SHAPE_BINS_LAT_ACCEL, SHAPE_KEY_FMT, SHAPE_MAX,
                                             SHAPE_MIN, HondaLateralModel)


def make_model(params=None):
  store = dict(params or {})
  return HondaLateralModel(DEFAULT_LAT_ACCEL_FACTOR, store.get)


def step(model, request, wire, v_ego, desired_la, actual_la, lat_active=True, steer_control_active=True, pressed=False):
  v_sq = v_ego * v_ego
  return model.update(request, wire, lat_active, steer_control_active, pressed, v_ego, desired_la / v_sq, actual_la / v_sq)


PLANT_LAG_SIM = 0.8   # s; the routes fit 1.0-1.3 s, deliberately not the model's PLANT_TAU so the tests see a mismatch


def drive(model, v_ego, gain_true, seconds, wire_fn, pressed=False, lat_active=True, shape_true=None):
  """Constant-speed drive on a first-order plant lat_accel = gain_true * shape_true(|lat_accel|) * wire
  (PLANT_LAG_SIM lag). Torque sign convention: right positive; curvature is left positive, so measured
  curvature = -lat_accel / v^2."""
  la = 0.0
  a = np.exp(-DT_CTRL / PLANT_LAG_SIM)
  for k in range(int(seconds / DT_CTRL)):
    wire = wire_fn(k * DT_CTRL)
    s = 1.0 if shape_true is None else shape_true(abs(la))
    la = a * la + (1 - a) * gain_true * s * wire
    step(model, wire, wire, v_ego, -la, -la, pressed=pressed, lat_active=lat_active)
  return la


def centering_shape(lat_accel):
  """A car whose EPS fights back harder in hard turns: unity through the anchor band, 40% less lateral
  accel per unit torque by 2 m/s^2, flat beyond."""
  return float(np.interp(lat_accel, [SHAPE_ANCHOR_LAT_ACCEL, 2.0], [1.0, 0.6]))


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
    self.assertEqual(len(model.param_keys()), len(GAIN_BINS_MPH) + len(SHAPE_BINS_LAT_ACCEL))

  def test_shape_prior_is_neutral(self):
    model = make_model()
    for la in (0.0, 0.5, SHAPE_ANCHOR_LAT_ACCEL, 1.2, 2.5, 4.0):
      self.assertEqual(model.shape(la), 1.0)
      self.assertEqual(model.shape(-la), 1.0)
    # with a neutral shape the correction is the plain per-speed one
    v = 8.0
    desired_la = 2.0
    step(model, 0.0, 0.0, v, desired_la, 0.0)
    self.assertAlmostEqual(model.ff_correction, max(-desired_la / model.gain(v) + desired_la / DEFAULT_LAT_ACCEL_FACTOR,
                                                    -FF_CORRECTION_MAX))
    self.assertEqual(model.shape_now, 1.0)
    self.assertAlmostEqual(model.gain_now, model.gain(v))

  def test_persisted_shape_loads_clips_and_shapes_the_feedforward(self):
    params = {SHAPE_KEY_FMT.format(slot=15): 0.8, SHAPE_KEY_FMT.format(slot=20): 0.1, SHAPE_KEY_FMT.format(slot=30): 9.0}
    model = make_model(params)
    self.assertAlmostEqual(model.shape(1.5), 0.8)
    self.assertAlmostEqual(model.shape(2.0), SHAPE_MIN)
    self.assertAlmostEqual(model.shape(3.0), SHAPE_MAX)
    self.assertAlmostEqual(model.shape(5.0), SHAPE_MAX)          # held beyond the last bin
    self.assertEqual(model.shape(SHAPE_ANCHOR_LAT_ACCEL), 1.0)   # the anchor cannot be persisted away
    self.assertAlmostEqual(model.shape(1.25), 0.9)               # linear between bins
    self.assertAlmostEqual(model.shape(1.0), 1.0)
    # a shape below 1 means the car needs more torque per m/s^2 there: the correction grows into the turn
    v = 25.0
    desired_la = 1.5
    step(model, 0.0, 0.0, v, desired_la, 0.0)
    expected = -desired_la / (model.gain(v) * 0.8) + desired_la / DEFAULT_LAT_ACCEL_FACTOR
    self.assertAlmostEqual(model.ff_correction, expected)
    self.assertAlmostEqual(model.shape_now, 0.8)
    self.assertAlmostEqual(model.gain_now, model.gain(v) * 0.8)
    self.assertLess(model.ff_correction, -desired_la / model.gain(v) + desired_la / DEFAULT_LAT_ACCEL_FACTOR)

  def test_feedforward_correction_low_speed_adds_torque_into_the_turn(self):
    model = make_model()
    v = 8.0
    desired_la = 0.5   # left turn: positive lateral accel, positive curvature, negative torque
    out = step(model, -desired_la / DEFAULT_LAT_ACCEL_FACTOR, 0.0, v, desired_la, 0.0)
    expected = -desired_la / model.gain(v) + desired_la / DEFAULT_LAT_ACCEL_FACTOR
    self.assertLess(expected, -0.3)   # the car needs a lot more than latAccelFactor says at 8 m/s
    self.assertAlmostEqual(model.ff_correction, expected)
    self.assertAlmostEqual(out, -desired_la / DEFAULT_LAT_ACCEL_FACTOR + expected)
    # mirror image to the right
    out_r = step(model, desired_la / DEFAULT_LAT_ACCEL_FACTOR, 0.0, v, -desired_la, 0.0)
    self.assertAlmostEqual(out_r, -out)

  def test_feedforward_correction_highway_is_small(self):
    model = make_model()
    v = 28.0
    desired_la = 1.0
    step(model, -desired_la / DEFAULT_LAT_ACCEL_FACTOR, 0.0, v, desired_la, 0.0)
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
    # alternate a held torque left/right so both signs and the filters settle well inside each dwell;
    # gentle enough (|la| ~0.66) to sit in the anchor band, where the speed table is defined
    drive(model, v, g_true, 240.0, lambda t: 0.3 if (t // 15.0) % 2 == 0 else -0.3)
    self.assertAlmostEqual(model.gain(v), g_true, delta=0.1)
    self.assertTrue(model.learning)

  def test_identification_ignores_saturation_and_rate_limits(self):
    # regressing on the actual wire means a pinned wire is still a valid sample for the speed table
    model = make_model()
    v = 12.0
    g_true = 0.6
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

  def test_gentle_curves_train_the_gain_table_only(self):
    # inside the anchor band the shape is 1.0 by definition, so a wrong gain is corrected in the speed
    # table and the shape table never moves
    model = make_model()
    v = 12.0
    g_true = 2.2
    drive(model, v, g_true, 200.0, lambda t: 0.25 if (t // 15.0) % 2 == 0 else -0.25)   # |la| ~0.55
    self.assertAlmostEqual(model.gain(v), g_true, delta=0.1)
    self.assertEqual(list(model.shapes), [1.0] * len(SHAPE_BINS_LAT_ACCEL))
    self.assertFalse(model.learning_shape)

  def test_identifies_centering_shape(self):
    model = make_model()
    v = 20.0
    g_true = model.gain(v)   # speed table already right: hard turns must move the shape, not the gain

    # dwell in a gentle (anchor band), a moderate and a hard turn in turn, both signs, like a drive that
    # is mostly gentle curves with the odd hard corner
    def wire_fn(t):
      mag = (0.3, 0.45, 0.85)[int(t // 15.0) % 3]
      return mag if (t // 90.0) % 2 == 0 else -mag
    # steady-state lateral accel of the hard dwell on this plant
    la_hard = 1.0
    for _ in range(100):
      la_hard = g_true * 0.85 * centering_shape(la_hard)
    self.assertGreater(la_hard, 1.25)
    self.assertLess(g_true * 0.3, SHAPE_ANCHOR_LAT_ACCEL)
    drive(model, v, g_true, 900.0, wire_fn, shape_true=centering_shape)
    self.assertTrue(model.learning_shape)
    self.assertLess(model.shape(la_hard), 0.9)
    self.assertAlmostEqual(model.shape(la_hard), centering_shape(la_hard), delta=0.1)
    self.assertAlmostEqual(model.gain(v), g_true, delta=0.1)
    # bins above the excitation keep their prior
    self.assertEqual(model.shapes[-1], 1.0)

  def test_shape_learns_from_a_pinned_wire(self):
    # route 00000114: the wire sat at 433 for 81% of the hard-turn ticks, delivering 1.27 m/s^2 where the
    # speed table promised 1.7. That sample is the shape, and it must not touch the speed table.
    model = make_model()
    v = 13.4
    g_true = model.gain(v)
    la_pinned = 1.0
    for _ in range(100):
      la_pinned = g_true * 1.0 * centering_shape(la_pinned)
    drive(model, v, g_true, 300.0, lambda t: 1.0 if (t // 20.0) % 2 == 0 else -1.0, shape_true=centering_shape)
    self.assertTrue(model.learning_shape)
    self.assertLess(model.shape(la_pinned), 0.9)
    self.assertAlmostEqual(model.shape(la_pinned), centering_shape(la_pinned), delta=0.1)
    self.assertAlmostEqual(model.gain(v), g_true)

  def test_correction_does_not_oppose_a_saturated_request(self):
    # model gain above latAccelFactor: the correction wants to take torque away
    params = {GAIN_KEY_FMT.format(slot=30): 2.6, GAIN_KEY_FMT.format(slot=40): 2.6}
    model = make_model(params)
    v = 30 * CV.MPH_TO_MS
    desired_la = 1.8
    out = step(model, -0.5, 0.0, v, desired_la, 0.0)
    self.assertGreater(model.ff_correction, 0.05)
    self.assertAlmostEqual(out, -0.5 + model.ff_correction)        # unsaturated: full correction
    out = step(model, -1.0, 0.0, v, desired_la, 0.0)
    self.assertEqual(out, -1.0)                                      # saturated: none of it
    self.assertEqual(model.applied_correction, 0.0)
    out = step(model, -(1.0 - FF_SATURATION_FADE / 2), 0.0, v, desired_la, 0.0)
    self.assertAlmostEqual(out, -(1.0 - FF_SATURATION_FADE / 2) + 0.5 * model.ff_correction)   # half way: half of it
    # a correction into the turn is never faded
    out = step(model, -1.0, 0.0, 8.0, 1.0, 0.0)
    self.assertLess(model.ff_correction, 0.0)
    self.assertEqual(out, -1.0)
    out = step(model, -0.5, 0.0, 8.0, 1.0, 0.0)
    self.assertAlmostEqual(out, max(-0.5 + model.ff_correction, -1.0))

  def test_shapes_stay_bounded(self):
    model = make_model()
    v = 20.0
    g_true = model.gain(v)
    drive(model, v, g_true, 300.0, lambda t: 0.8 if (t // 15.0) % 2 == 0 else -0.8, shape_true=lambda la: 0.05)
    self.assertGreaterEqual(min(model.shapes), SHAPE_MIN)
    model = make_model()
    drive(model, v, g_true, 300.0, lambda t: 0.6 if (t // 15.0) % 2 == 0 else -0.6, shape_true=lambda la: 5.0)
    self.assertLessEqual(max(model.shapes), SHAPE_MAX)


if __name__ == "__main__":
  unittest.main()

import unittest

import numpy as np

from opendbc.car import DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.honda.lateral_model import (CEILING_KEY, CEILING_MAX, CEILING_MIN, CEILING_PROBE, DEFAULT_LAT_ACCEL_FACTOR,
                                             FF_CORRECTION_MAX, FF_SATURATION_FADE, FILTER_TAU, GAIN_BINS_MPH, GAIN_BINS_MS,
                                             GAIN_KEY_FMT, GAIN_MAX, GAIN_MIN, GAIN_PRIOR, MAX_LAT_ACCEL, MAX_LAT_JERK_DOWN,
                                             MAX_LAT_JERK_UP, PRESS_HOLDOFF, SHAPE_ANCHOR_LAT_ACCEL, SHAPE_BINS_LAT_ACCEL,
                                             SHAPE_KEY_FMT, SHAPE_MAX, SHAPE_MIN, WIRE_RATE_MAX, HondaLateralModel)
from opendbc.car.lateral import ISO_LATERAL_ACCEL, ISO_LATERAL_JERK


def make_model(params=None):
  store = dict(params or {})
  return HondaLateralModel(DEFAULT_LAT_ACCEL_FACTOR, store.get)


def step(model, request, wire, v_ego, desired_la, actual_la, lat_active=True, steer_control_active=True, pressed=False,
         angle=0.0, rate=0.0):
  v_sq = v_ego * v_ego
  return model.update(request, wire, lat_active, steer_control_active, pressed, v_ego, desired_la / v_sq, actual_la / v_sq,
                      angle, rate)


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
    self.assertEqual(len(model.param_keys()), len(GAIN_BINS_MPH) + len(SHAPE_BINS_LAT_ACCEL) + 1)
    self.assertTrue(CEILING_KEY in model.learned_values())

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
    # non-increasing: the bins above the last measured one inherit its saturation, a persisted value
    # above 1.0 (the tables written before the bound) or above its lower neighbor is projected down
    self.assertAlmostEqual(model.shape(2.5), SHAPE_MIN)
    self.assertAlmostEqual(model.shape(3.0), SHAPE_MIN)
    self.assertAlmostEqual(model.shape(5.0), SHAPE_MIN)          # held beyond the last bin
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
    # the tables route 0000011f drove with (1.5-1.6, i.e. a feedforward cut where the car was undershooting)
    model = make_model({SHAPE_KEY_FMT.format(slot=10): 1.6, SHAPE_KEY_FMT.format(slot=15): 1.5})
    self.assertEqual(list(model.shapes), [SHAPE_MAX] * len(SHAPE_BINS_LAT_ACCEL))
    self.assertEqual(SHAPE_MAX, 1.0)

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
    # bins above the excitation are held at the saturation the last measured bin showed, never above it
    self.assertLessEqual(model.shapes[-1], model.shape(la_hard) + 1e-9)
    self.assertEqual(model.shapes, sorted(model.shapes, reverse=True))

  def test_shape_learns_from_short_pinned_dwells_between_corrections(self):
    # route 0000011f: 136 pinned, unpressed dwells, median 0.06 s, the longest 1.45 s, the driver pressing
    # in between. The shape must learn from those; the speed table's 2 s filter never admitted one.
    model = make_model()
    v = 14.5
    g_true = model.gain(v)
    la_pinned = 1.0
    for _ in range(100):
      la_pinned = g_true * 1.0 * centering_shape(la_pinned)
    self.assertGreater(la_pinned, 1.0)
    la = 0.0
    a = np.exp(-DT_CTRL / PLANT_LAG_SIM)
    n_shape = 0
    n_gain = 0
    for k in range(int(240.0 / DT_CTRL)):
      t = k * DT_CTRL
      cycle = t % 6.0                                   # 6 s turn cycles: pinned right, pressed 0.4 s every 1.5 s
      wire = 1.0 if (t // 6.0) % 2 == 0 else -1.0
      if cycle > 4.0:                                   # 2 s straight between turns
        wire = 0.0
      pressed = (cycle % 1.5) < 0.4 and cycle <= 4.0
      la = a * la + (1 - a) * g_true * centering_shape(abs(la)) * wire
      step(model, wire, wire, v, -la, -la, pressed=pressed)
      n_shape += model.learning_shape
      n_gain += model.learning_gain
    self.assertGreater(n_shape, 200)
    self.assertEqual(n_gain, 0)                          # a pinned hard turn is never a speed-table sample
    self.assertAlmostEqual(model.shape(la_pinned), centering_shape(la_pinned), delta=0.12)
    self.assertAlmostEqual(model.gain(v), g_true)

  def test_shape_never_learns_above_one(self):
    # a car that delivered *more* per unit torque in hard turns than in gentle ones would be a sign error
    # somewhere else (driver help, lag mismatch); the bound keeps it from cutting the feedforward
    model = make_model()
    v = 20.0
    drive(model, v, model.gain(v), 300.0, lambda t: 0.6 if (t // 15.0) % 2 == 0 else -0.6, shape_true=lambda la: 1.6)
    self.assertTrue(model.learning_shape)
    self.assertEqual(max(model.shapes), 1.0)

  def test_shape_feedforward_carries_the_return_without_an_integrator(self):
    # 33 mph, the plant of route 0000011f: shape 0.6 at 1.0 m/s^2 and 0.5 from 1.5 up. The driver has been
    # overriding in a pinned turn, so the torque controller's integrator holds nothing; the request into
    # the model is the controller's linear feedforward alone. As the planner's desired lateral accel comes
    # back down through the knee, the model's feedforward must leave the torque limit on its own and
    # reach the anchor-band value, with nothing accumulated during the press.
    params = {SHAPE_KEY_FMT.format(slot=10): 0.6, SHAPE_KEY_FMT.format(slot=15): 0.5, SHAPE_KEY_FMT.format(slot=20): 0.5,
              GAIN_KEY_FMT.format(slot=30): 2.5, GAIN_KEY_FMT.format(slot=40): 2.5}
    model = make_model(params)
    v = 33 * CV.MPH_TO_MS
    lin_factor = DEFAULT_LAT_ACCEL_FACTOR
    outs = []
    for des in (1.6, 1.4, 1.2, 1.1, 1.0, 0.9, 0.8, 0.75, 0.5):
      outs.append(step(model, des / lin_factor, 1.0, v, -des, -1.1, pressed=False))
    self.assertEqual(outs[0], 1.0)                                                  # beyond the car: pinned
    self.assertLess(outs[3], 1.0)                                                   # 1.1 m/s^2: already off the limit
    self.assertEqual(outs, sorted(outs, reverse=True))                              # monotone unwind of the request
    self.assertAlmostEqual(outs[7], 0.75 / model.gain(v))                           # anchor band: the model's own feedforward
    self.assertAlmostEqual(outs[-1], 0.5 / model.gain(v))
    self.assertAlmostEqual(model.ff_correction, 0.5 / model.gain(v) - 0.5 / lin_factor)   # i.e. the plain speed correction
    # the correction has no state: the same desired lateral accel gives the same torque whether or not
    # the driver was pressing for the last minute
    out_a = step(model, 1.0 / lin_factor, 1.0, v, -1.0, -1.1)
    for _ in range(int(60.0 / DT_CTRL)):
      step(model, 1.0, 1.0, v, -1.6, -1.1, pressed=True)
    out_b = step(model, 1.0 / lin_factor, 1.0, v, -1.0, -1.1)
    self.assertEqual(out_a, out_b)

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


def drive_through_limit(model, v_ego, g_true, seconds, wire_fn, plant):
  """Like drive(), but the request goes through limit() first the way the car controller sends it, so the
  EPS ceiling clip acts on the wire the plant sees. plant(g, wire, la) is the steady-state lateral accel."""
  la = 0.0
  last = 0.0
  a = np.exp(-DT_CTRL / PLANT_LAG_SIM)
  for k in range(int(seconds / DT_CTRL)):
    req = wire_fn(k * DT_CTRL)
    last = model.limit(req, last, v_ego)
    la = a * la + (1 - a) * plant(g_true, last, la)
    step(model, req, last, v_ego, -la, -la)
  return la


def levels_fn(levels, dwell, half_period):
  return lambda t: levels[int(t // dwell) % len(levels)] * (1.0 if (t // half_period) % 2 == 0 else -1.0)


# wire levels for the ceiling tests: a real wire holds many different levels on both sides of the ceiling. With
# only two levels above the lowest candidate every candidate fits them exactly (a line through two points)
CEILING_LEVELS = (0.3, 0.5, 0.6, 0.7, 0.85, 1.0, 0.45, 0.8, 0.55, 0.95)
CEILING_DWELL = 10.0
CEILING_HALF_PERIOD = 100.0


def ceiling_levels():
  return levels_fn(CEILING_LEVELS, CEILING_DWELL, CEILING_HALF_PERIOD)


def clamp_plant(ceiling):
  return lambda g, w, la: g * np.clip(w, -ceiling, ceiling)


def linear_plant(g, w, la):
  return g * w


class TestHondaLateralEpsCeiling(unittest.TestCase):
  def test_prior_is_no_ceiling_and_persisted_value_loads_and_clips(self):
    model = make_model()
    self.assertEqual(model.ceiling, CEILING_MAX)
    self.assertEqual(model.wire_limit, 1.0)
    self.assertEqual(model.effective_wire(0.9), 0.9)
    model = make_model({CEILING_KEY: 0.55})
    self.assertAlmostEqual(model.ceiling, 0.55)
    self.assertAlmostEqual(model.wire_limit, 0.55 + CEILING_PROBE)
    self.assertAlmostEqual(model.effective_wire(-0.9), -0.55)
    self.assertEqual(model.effective_wire(0.3), 0.3)
    self.assertEqual(make_model({CEILING_KEY: 0.1}).ceiling, CEILING_MIN)
    self.assertEqual(make_model({CEILING_KEY: 7.0}).ceiling, CEILING_MAX)
    # a car-specific seed is used only when nothing is persisted
    self.assertAlmostEqual(HondaLateralModel(DEFAULT_LAT_ACCEL_FACTOR, {}.get, ceiling_prior=0.6).ceiling, 0.6)
    self.assertAlmostEqual(HondaLateralModel(DEFAULT_LAT_ACCEL_FACTOR, {CEILING_KEY: 0.8}.get, ceiling_prior=0.6).ceiling, 0.8)

  def test_limit_holds_the_wire_to_the_ceiling_plus_the_probe_band(self):
    model = make_model({CEILING_KEY: 0.55})
    v = 15.0
    wire = run_limit(model, v, lambda k: 1.0, 400)
    self.assertAlmostEqual(wire[-1], 0.55 + CEILING_PROBE)
    self.assertTrue(model.ceiling_limited)
    wire = run_limit(model, v, lambda k: -1.0, 400)
    self.assertAlmostEqual(wire[-1], -(0.55 + CEILING_PROBE))
    # inside the band nothing changes
    model = make_model({CEILING_KEY: 0.55})
    wire = run_limit(model, v, lambda k: 0.5, 400)
    self.assertAlmostEqual(wire[-1], 0.5)
    self.assertFalse(model.ceiling_limited)
    # and with no ceiling the unit clip is all there is
    model = make_model()
    self.assertEqual(model.limit(5.0, 0.999, v), 1.0)
    self.assertFalse(model.ceiling_limited)

  def test_shape_regresses_on_the_wire_the_eps_acted_on_and_the_speed_table_skips_it(self):
    # a car that clamps at 0.5: pinned at 1.0 it delivers g_true * 0.5. With the ceiling known, the speed
    # table must read g_true from the dwells the clip did not shape and leave the pinned ones alone
    v = 12.0
    g_true = 1.2
    wire_fn = levels_fn((0.3, 0.5, 1.0), 15.0, 45.0)
    model = make_model({CEILING_KEY: 0.5})
    drive_through_limit(model, v, g_true, 270.0, wire_fn, clamp_plant(0.5))
    self.assertAlmostEqual(model.ceiling, 0.5, delta=0.06)
    self.assertAlmostEqual(model.gain(v), g_true, delta=0.1)
    # without it the pinned dwells read the gain low (and the shape table then has to make up the difference)
    model = make_model()
    model.ceiling_samples = -10**9   # hold the bank out of it
    drive_through_limit(model, v, g_true, 270.0, wire_fn, clamp_plant(0.5))
    self.assertLess(model.gain(v), g_true - 0.2)

  def test_speed_table_does_not_learn_from_a_clipped_sample(self):
    # a persisted ceiling below the car's: the gain must not be regressed on the wire the clip left, which
    # would read the car's response per unit of a wire it never got (route 00000129: 2-3x the measured gain)
    v = 12.0
    g_true = 1.2
    model = make_model({CEILING_KEY: 0.4})
    drive(model, v, g_true, 200.0, levels_fn((1.0,), 20.0, 20.0))
    self.assertEqual(model.gain(v), make_model().gain(v))
    # a wire inside the ceiling trains it as before
    model = make_model({CEILING_KEY: 0.4})
    drive(model, v, g_true, 200.0, levels_fn((0.3,), 20.0, 20.0))
    self.assertAlmostEqual(model.gain(v), g_true, delta=0.1)

  def test_learns_a_torque_clamp(self):
    # a plant that ignores everything above 0.55 of full torque, driven at many levels on both sides of it
    v = 15.0
    model = make_model()
    g_true = model.gain(v)
    drive_through_limit(model, v, g_true, 400.0, ceiling_levels(), clamp_plant(0.55))
    self.assertTrue(model.learning_ceiling)
    self.assertAlmostEqual(model.ceiling, 0.55, delta=0.06)
    self.assertAlmostEqual(model.wire_limit, 0.55 + CEILING_PROBE, delta=0.06)
    # the shape table has little left to explain at the lateral accel the car reaches once the ceiling is
    # in (it learned from the pinned dwells before the ceiling came down, and recovers afterwards)
    self.assertGreater(model.shape(g_true * 0.55), 0.85)
    # and the learned value is persisted
    self.assertAlmostEqual(model.learned_values()[CEILING_KEY], model.ceiling)
    # a higher clamp is found where it is
    model = make_model()
    drive_through_limit(model, v, g_true, 400.0, ceiling_levels(), clamp_plant(0.7))
    self.assertAlmostEqual(model.ceiling, 0.7, delta=0.06)

  def test_no_clamp_is_learned_on_a_linear_plant(self):
    v = 15.0
    model = make_model()
    drive_through_limit(model, v, model.gain(v), 400.0, ceiling_levels(), linear_plant)
    self.assertAlmostEqual(model.ceiling, CEILING_MAX, places=2)
    # nor on a pinned one, where a clamp and a shape cannot be told apart
    model = make_model()
    drive(model, 12.0, 0.6, 200.0, levels_fn((1.0,), 20.0, 20.0))
    self.assertAlmostEqual(model.ceiling, CEILING_MAX, places=2)

  def test_centering_shape_is_not_mistaken_for_a_clamp(self):
    # the plant of test_identifies_centering_shape: lateral accel saturates with lateral accel, not torque.
    # The data cannot separate the two perfectly (both say the top of the wire buys little), but the ceiling
    # may not collapse: whatever it settles on costs at most a few percent of lateral accel on such a car
    v = 20.0
    model = make_model()
    g_true = model.gain(v)
    la_full = 0.0
    for _ in range(100):
      la_full = g_true * 0.85 * centering_shape(la_full)
    plant = lambda g, w, la: g * centering_shape(abs(la)) * w  # noqa: E731
    drive_through_limit(model, v, g_true, 600.0, levels_fn((0.3, 0.45, 0.85), 15.0, 90.0), plant)
    self.assertGreaterEqual(model.ceiling, 0.6)
    la_limited = 0.0
    for _ in range(100):
      la_limited = g_true * min(0.85, model.wire_limit) * centering_shape(la_limited)
    self.assertGreater(la_limited, 0.95 * la_full)
    model = make_model()
    drive_through_limit(model, v, g_true, 600.0, ceiling_levels(), plant)
    self.assertGreaterEqual(model.ceiling, 0.6)

  def test_recovers_from_a_persisted_ceiling_that_is_too_low(self):
    # the probe band lets the bank see torque above the ceiling; on a car that answers to it the ceiling climbs.
    # On a linear plant the last step is slow: the band above the ceiling carries little of the variance
    v = 15.0
    model = make_model({CEILING_KEY: 0.45})
    self.assertAlmostEqual(model.wire_limit, 0.55)
    drive_through_limit(model, v, model.gain(v), 600.0, ceiling_levels(), linear_plant)
    self.assertGreaterEqual(model.ceiling, 0.85)
    # and on a car with a clamp above the persisted value it climbs to the clamp (route 00000129 left the
    # MDX 3G at 0.416 against a measured 0.50-0.58)
    model = make_model({CEILING_KEY: 0.416})
    drive_through_limit(model, v, model.gain(v), 400.0, ceiling_levels(), clamp_plant(0.55))
    self.assertAlmostEqual(model.ceiling, 0.55, delta=0.06)

  def test_inflated_gain_table_does_not_pull_the_ceiling_down(self):
    # route 00000129's tables: 2-3x the measured gain at 10-20 mph. Whatever the table says, the bank's score is
    # about how |lat accel| follows |wire| and the ceiling stays where the car's clamp is while the table corrects
    gains = (0.39, 0.77, 1.12, 1.57, 2.63, 2.86, 2.1, 2.3, 2.6)
    params = {GAIN_KEY_FMT.format(slot=mph): g for mph, g in zip(GAIN_BINS_MPH, gains, strict=True)}
    params[CEILING_KEY] = 0.538
    for v, g_true in ((10.0, 0.4), (15.0, 1.2), (20.0, 2.0), (25.0, 2.4)):
      model = make_model(params)
      drive_through_limit(model, v, g_true, 400.0, ceiling_levels(), clamp_plant(0.55))
      self.assertGreaterEqual(model.ceiling, 0.53, msg=f"v={v}")
      self.assertLessEqual(model.ceiling, 0.61, msg=f"v={v}")
      self.assertAlmostEqual(model.gain(v), g_true, delta=0.15 * g_true, msg=f"v={v}")

  def test_ceiling_holds_without_settled_dwells(self):
    # a wire that never holds a level above the lowest candidate (a town drive: routes 00000127 and 00000129
    # had no settled, unpressed dwell above 0.42) is no evidence either way for the dwell bank, whatever the
    # plant; without the steering sensor (not fed here) the steer-rate bank has nothing to say either
    v = 15.0
    sweep = lambda t: 0.75 * np.sin(2 * np.pi * t / 6.0) + 0.25 * np.sin(2 * np.pi * t / 2.3)  # noqa: E731
    for prior, plant in ((0.538, clamp_plant(0.55)), (0.538, linear_plant), (1.0, clamp_plant(0.55))):
      model = make_model({CEILING_KEY: prior})
      drive_through_limit(model, v, model.gain(v), 300.0, sweep, plant)
      self.assertAlmostEqual(model.ceiling, prior, places=3)
      self.assertEqual(model.ceiling_samples, 0)

  def test_ceiling_does_not_learn_while_pressed(self):
    v = 15.0
    model = make_model()
    g_true = model.gain(v)
    la = 0.0
    a = np.exp(-DT_CTRL / PLANT_LAG_SIM)
    wire_fn = ceiling_levels()
    for k in range(int(200.0 / DT_CTRL)):
      wire = wire_fn(k * DT_CTRL)
      la = a * la + (1 - a) * g_true * float(np.clip(wire, -0.55, 0.55))
      step(model, wire, wire, v, -la, -la, pressed=True)
    self.assertEqual(model.ceiling, CEILING_MAX)
    self.assertEqual(model.ceiling_samples, 0)


STEER_RATIO_SIM = 15.5
WHEELBASE_SIM = 2.7
RATE_GAIN_SIM = 18.0        # deg/s per unit wire the EPS acts on (route 00000139 fits 17-19 at the knee)
CENTERING_SIM = 1.0         # 1/s, self-aligning return: 0.55 of wire holds ~10 deg
RATE_DELAY_SIM = 0.12       # s, deliberately not the model's RATE_DELAY


def steer_plant(clamp=None):
  """Steer-rate plant: rate = RATE_GAIN_SIM * clip(wire, +-clamp) - CENTERING_SIM * angle (deg/s). clamp None is linear."""
  def plant(wire, angle):
    w = wire if clamp is None else float(np.clip(wire, -clamp, clamp))
    return RATE_GAIN_SIM * w - CENTERING_SIM * angle
  return plant


def sweep_wire(t):
  """A town drive's wire: never holds a level, runs to full scale on both sides (the dwell bank gets nothing from it)."""
  return 0.75 * np.sin(2 * np.pi * t / 6.0) + 0.25 * np.sin(2 * np.pi * t / 2.3)


def drive_steer(model, v_ego, seconds, wire_fn, plant, pressed=False, noise=0.0, seed=0):
  """Drive the steer-rate plant with the request through limit() the way the car controller sends it, feeding the
  model the steering sensor (angle, rate) and the lateral accel the angle implies. Returns the angle trace."""
  rng = np.random.default_rng(seed)
  angle = 0.0
  last = 0.0
  from collections import deque
  delay_line = deque([0.0] * int(round(RATE_DELAY_SIM / DT_CTRL)), maxlen=int(round(RATE_DELAY_SIM / DT_CTRL)))
  angles = []
  for k in range(int(seconds / DT_CTRL)):
    last = model.limit(wire_fn(k * DT_CTRL), last, v_ego)
    delayed = delay_line[0]
    delay_line.append(last)
    rate = plant(delayed, angle)
    angle += rate * DT_CTRL
    la = np.radians(angle) * v_ego * v_ego / (STEER_RATIO_SIM * WHEELBASE_SIM)
    step(model, last, last, v_ego, -la, -la, pressed=pressed, angle=angle, rate=rate + noise * rng.standard_normal())
    angles.append(angle)
  return np.array(angles)


class TestHondaLateralEpsCeilingSteerRate(unittest.TestCase):
  def test_learns_a_clamp_from_a_wire_that_never_dwells(self):
    # the Integra case: the dwell bank counts nothing on this wire, the steer-rate bank finds the clamp
    v = 15.0
    model = make_model()
    drive_steer(model, v, 300.0, sweep_wire, steer_plant(0.55))
    self.assertEqual(model.ceiling_samples, 0)
    self.assertGreater(model.rate_samples, 0)
    self.assertLess(min(model.rate_resid), 1.0)   # the bank scored
    self.assertAlmostEqual(model.rate_knee, 0.55, delta=0.06)
    self.assertLess(model.ceiling, 0.75)
    # a longer stretch settles it
    drive_steer(model, v, 600.0, sweep_wire, steer_plant(0.55))
    self.assertAlmostEqual(model.ceiling, 0.55, delta=0.06)
    self.assertAlmostEqual(model.learned_values()[CEILING_KEY], model.ceiling)
    # a higher clamp is found where it is
    model = make_model()
    drive_steer(model, v, 900.0, sweep_wire, steer_plant(0.75))
    self.assertAlmostEqual(model.ceiling, 0.75, delta=0.06)

  def test_holds_the_seed_on_the_car_it_was_measured_on(self):
    v = 15.0
    model = make_model({CEILING_KEY: 0.55})
    drive_steer(model, v, 300.0, sweep_wire, steer_plant(0.55), noise=3.0)
    self.assertAlmostEqual(model.ceiling, 0.55, delta=0.03)

  def test_no_clamp_is_learned_on_a_linear_plant(self):
    v = 15.0
    for noise in (0.0, 3.0):
      model = make_model()
      drive_steer(model, v, 300.0, sweep_wire, steer_plant(None), noise=noise)
      self.assertGreater(model.rate_samples, 0)
      self.assertAlmostEqual(model.ceiling, CEILING_MAX, places=2, msg=f"noise={noise}")

  def test_recovers_from_a_persisted_ceiling_that_is_too_low(self):
    # the probe band shows the bank torque above the ceiling; a car that answers to it has no clamp there
    v = 15.0
    model = make_model({CEILING_KEY: 0.45})
    drive_steer(model, v, 600.0, sweep_wire, steer_plant(None))
    self.assertGreaterEqual(model.ceiling, 0.85)
    model = make_model({CEILING_KEY: 0.416})
    drive_steer(model, v, 900.0, sweep_wire, steer_plant(0.55))
    self.assertAlmostEqual(model.ceiling, 0.55, delta=0.06)

  def test_centering_is_not_mistaken_for_a_clamp(self):
    # a car with a strong self-aligning torque and a linear EPS: the angle it reaches saturates, the rate it
    # buys per unit wire does not. The centering terms carry that, the clip must not
    v = 15.0
    model = make_model()
    strong = lambda wire, angle: RATE_GAIN_SIM * wire - 3.0 * CENTERING_SIM * angle  # noqa: E731
    drive_steer(model, v, 300.0, sweep_wire, strong)
    self.assertGreater(model.rate_samples, 0)
    self.assertAlmostEqual(model.ceiling, CEILING_MAX, places=2)

  def test_needs_speed_wire_and_an_unpressed_wheel(self):
    model = make_model()
    drive_steer(model, 8.0, 60.0, sweep_wire, steer_plant(0.55))            # below RATE_MIN_SPEED
    self.assertEqual(model.rate_samples, 0)
    drive_steer(model, 15.0, 60.0, lambda t: 0.2 * np.sin(t), steer_plant(0.55))   # never above RATE_MIN_WIRE
    self.assertEqual(model.rate_samples, 0)
    drive_steer(model, 15.0, 120.0, sweep_wire, steer_plant(0.55), pressed=True)
    self.assertEqual(model.rate_samples, 0)
    self.assertEqual(model.ceiling, CEILING_MAX)

  def test_a_poor_fit_is_not_evidence(self):
    # a steer rate the wire does not explain (noise dominates): the bank counts samples but moves nothing
    v = 15.0
    model = make_model({CEILING_KEY: 0.55})
    drive_steer(model, v, 300.0, sweep_wire, steer_plant(None), noise=60.0)
    self.assertGreater(model.rate_samples, 0)
    self.assertAlmostEqual(model.ceiling, 0.55, places=3)

  def test_grey_lines_bound_follows_the_learned_ceiling(self):
    v = 15.0
    model = make_model()
    drive_steer(model, v, 900.0, sweep_wire, steer_plant(0.55))
    self.assertAlmostEqual(model.wire_limit, model.ceiling + CEILING_PROBE)
    self.assertLess(model.wire_limit, 0.75)


def run_limit(model, v_ego, request_fn, ticks, last=0.0):
  """Feed a request sequence through limit() the way the car controller does; returns the wire trace."""
  wire = []
  for k in range(ticks):
    last = model.limit(request_fn(k), last, v_ego)
    wire.append(last)
  return np.array(wire)


class TestHondaLateralWireLimits(unittest.TestCase):
  def test_targets_are_iso_11270(self):
    self.assertEqual(MAX_LAT_JERK_DOWN, ISO_LATERAL_JERK)
    self.assertEqual(MAX_LAT_JERK_UP, ISO_LATERAL_JERK)
    self.assertEqual(MAX_LAT_ACCEL, ISO_LATERAL_ACCEL)

  def test_0_5s_jerk_and_max_lateral_accel_match_test_lateral_limits(self):
    # opendbc/car/tests/test_lateral_limits.py measures a torque car's jerk as the lateral accel its rate
    # limiter lets it reach in 0.5 s from center (and give back in 0.5 s from full lock), through one
    # linear gain, and caps the plant gain at ISO_LATERAL_ACCEL. It has no STEER_DELTA to evaluate for
    # Honda, so the same measurement is made here against limit() itself: at every speed bin and between
    # bins, with the priors and with the tables saturated at GAIN_MAX, using the model's own gain as the
    # linear plant. Symmetric 5 m/s^3 is the planner's own bound (drive_helpers.clip_curvature).
    ticks = int(round(0.5 / DT_CTRL))
    for params in (None, {GAIN_KEY_FMT.format(slot=mph): GAIN_MAX for mph in GAIN_BINS_MPH}):
      speeds = list(GAIN_BINS_MS) + [0.5 * (a + b) for a, b in zip(GAIN_BINS_MS[:-1], GAIN_BINS_MS[1:], strict=True)]
      for v in speeds + [0.0, 45.0]:
        model = make_model(params)
        g = model.gain(v)
        self.assertLessEqual(g * 1.0, ISO_LATERAL_ACCEL + 1e-9)               # test_max_lateral_accel

        up = run_limit(model, v, lambda k: 1.0, ticks)
        up_jerk = g * abs(up[-1]) / 0.5
        self.assertLessEqual(up_jerk, ISO_LATERAL_JERK + 1e-9, msg=f"v={v:.1f} up {up_jerk:.3f}")

        down = run_limit(model, v, lambda k: 0.0, ticks, last=1.0)
        down_jerk = g * (1.0 - down[-1]) / 0.5
        self.assertLessEqual(down_jerk, ISO_LATERAL_JERK + 1e-9, msg=f"v={v:.1f} down {down_jerk:.3f}")

        # a full reversal from lock to lock is a down leg then an up leg and may not go faster than either
        rev = run_limit(model, v, lambda k: -1.0, ticks, last=1.0)
        rev_jerk = g * (1.0 - rev[-1]) / 0.5
        self.assertLessEqual(rev_jerk, ISO_LATERAL_JERK + 1e-9, msg=f"v={v:.1f} reversal {rev_jerk:.3f}")

  def test_any_request_sequence_stays_inside_iso_per_tick(self):
    # per tick, whatever the controller asks for: |gain * wire| <= 3 m/s^2 and |d(gain * wire)| <= 5 m/s^3 * DT
    rng = np.random.default_rng(0)
    for params in (None, {GAIN_KEY_FMT.format(slot=mph): GAIN_MAX for mph in GAIN_BINS_MPH}):
      for v in (3.0, 8.0, 13.0, 20.0, 30.0):
        model = make_model(params)
        g = model.gain(v)
        req = np.concatenate((rng.uniform(-3.0, 3.0, 400), np.tile([1.0, -1.0], 100), rng.normal(0.0, 0.3, 400)))
        wire = run_limit(model, v, req.__getitem__, len(req))
        la = g * wire
        self.assertLessEqual(np.max(np.abs(la)), ISO_LATERAL_ACCEL + 1e-9)
        self.assertLessEqual(np.max(np.abs(np.diff(np.concatenate(([0.0], la))))), ISO_LATERAL_JERK * DT_CTRL + 1e-9)
        self.assertLessEqual(np.max(np.abs(wire)), 1.0)

  def test_jerk_bound_is_constant_in_lateral_accel_across_speeds(self):
    # a step request from 0 to full torque: the wire may only climb at MAX_LAT_JERK_UP in lateral accel,
    # i.e. by MAX_LAT_JERK_UP * DT_CTRL / gain(v) in torque per tick, at every speed
    for v in (10.0, 15.0, 25.0, 30.0):
      model = make_model()
      g = model.gain(v)
      wire = run_limit(model, v, lambda k: 1.0, 5)
      la = g * wire
      steps = np.diff(np.concatenate(([0.0], la)))
      for s in steps:
        self.assertAlmostEqual(s, MAX_LAT_JERK_UP * DT_CTRL, places=9)
      self.assertTrue(model.jerk_limited)
      self.assertFalse(model.accel_limited)

  def test_return_to_center_runs_at_the_down_rate(self):
    v = 15.0
    model = make_model()
    g = model.gain(v)
    # hold a right turn, then request zero
    wire = run_limit(model, v, lambda k: 0.8, 300)
    self.assertAlmostEqual(wire[-1], 0.8)
    unwind = run_limit(model, v, lambda k: 0.0, 200, last=wire[-1])
    la_steps = -np.diff(np.concatenate(([g * 0.8], g * unwind)))
    binding = la_steps > 1e-9
    self.assertTrue(binding[0])
    for s in la_steps[binding][:-1]:         # every full step runs at the down rate; the last one is the remainder
      self.assertAlmostEqual(s, MAX_LAT_JERK_DOWN * DT_CTRL, places=9)
    self.assertLessEqual(la_steps[binding][-1], MAX_LAT_JERK_DOWN * DT_CTRL + 1e-9)
    # and it takes gain * 0.8 / MAX_LAT_JERK_DOWN seconds to get there
    ticks_needed = int(np.ceil(g * 0.8 / (MAX_LAT_JERK_DOWN * DT_CTRL)))
    self.assertEqual(int(np.argmax(unwind <= 1e-9)), ticks_needed - 1)

  def test_crossing_zero_is_down_then_up(self):
    v = 15.0
    model = make_model()
    g = model.gain(v)
    la_last = 0.03            # small right lateral accel commanded
    last = la_last / g
    out = model.limit(-1.0, last, v)
    # one down-rate step carries it through zero (0.03 -> -0.02); the up bound caps how far past zero it may land
    self.assertAlmostEqual(g * out, la_last - MAX_LAT_JERK_DOWN * DT_CTRL)
    self.assertGreaterEqual(g * out, -MAX_LAT_JERK_UP * DT_CTRL)
    out = model.limit(-1.0, 0.2 / g, v)     # from further right, the same step stops on the up bound instead
    self.assertAlmostEqual(g * out, 0.2 - MAX_LAT_JERK_DOWN * DT_CTRL)
    out = model.limit(-1.0, 0.04 / g, v)
    self.assertAlmostEqual(g * out, -0.01)
    out = model.limit(-1.0, 0.001 / g, v)   # almost centered: one step, and never further past zero than the up rate
    self.assertAlmostEqual(g * out, max(0.001 - MAX_LAT_JERK_DOWN * DT_CTRL, -MAX_LAT_JERK_UP * DT_CTRL))

  def test_low_speed_return_is_no_longer_held_to_the_old_torque_rate(self):
    # the old limiter took 0.33 s for a full swing at every speed; at town speeds a unit of torque buys so
    # little lateral accel that the ISO jerk bound is much looser than that, so only the wire-rate
    # backstop remains, at 0.1 s
    v = 10.0 * CV.MPH_TO_MS
    model = make_model()
    self.assertLess(model.gain(v), MAX_LAT_JERK_DOWN / WIRE_RATE_MAX)     # jerk bound looser than backstop here
    first = model.limit(0.0, -1.0, v)
    self.assertTrue(model.rate_limited)
    self.assertAlmostEqual(first, -1.0 + WIRE_RATE_MAX * DT_CTRL)
    self.assertGreater(MAX_LAT_JERK_DOWN * DT_CTRL / model.gain(v), WIRE_RATE_MAX * DT_CTRL)   # jerk step alone was larger
    unwind = run_limit(model, v, lambda k: 0.0, 50, last=-1.0)
    self.assertLessEqual(int(np.argmax(unwind >= -1e-9)), int(1.0 / (WIRE_RATE_MAX * DT_CTRL)))
    # while on the highway the ISO bound is the tighter one and the backstop never binds
    v = 30.0
    model = make_model()
    self.assertGreater(model.gain(v), MAX_LAT_JERK_DOWN / WIRE_RATE_MAX)
    run_limit(model, v, lambda k: 0.0, 5, last=-1.0)
    self.assertTrue(model.jerk_limited)
    self.assertFalse(model.rate_limited)

  def test_highway_rate_is_tighter_than_the_old_limiter(self):
    # the old fixed rate was 0.03/tick regardless of speed; through gain(v) at 30 m/s the ISO rate is lower
    v = 30.0
    model = make_model()
    per_tick = MAX_LAT_JERK_UP * DT_CTRL / model.gain(v)
    self.assertLess(per_tick, 0.03)
    wire = run_limit(model, v, lambda k: 1.0, 1)
    self.assertAlmostEqual(wire[0], per_tick)

  def test_lateral_accel_bound_holds_for_any_learned_gain(self):
    # the accel bound never lets the wire command more than MAX_LAT_ACCEL, and with the gain table clipped at
    # GAIN_MAX a unit of torque cannot exceed it anyway
    self.assertLessEqual(GAIN_MAX * 1.0, MAX_LAT_ACCEL)
    params = {GAIN_KEY_FMT.format(slot=mph): 9.0 for mph in GAIN_BINS_MPH}
    model = make_model(params)
    v = 30.0
    wire = run_limit(model, v, lambda k: 1.0, 2000)
    self.assertLessEqual(model.gain(v) * abs(wire[-1]), MAX_LAT_ACCEL + 1e-9)
    self.assertAlmostEqual(wire[-1], 1.0)

  def test_unbound_request_passes_through_bit_exact(self):
    v = 15.0
    model = make_model()
    g = model.gain(v)
    last = 0.31
    small = last + 0.5 * MAX_LAT_JERK_UP * DT_CTRL / g
    self.assertEqual(model.limit(small, last, v), small)
    self.assertFalse(model.jerk_limited or model.accel_limited or model.rate_limited)

  def test_output_is_unit_torque_bounded(self):
    v = 15.0
    model = make_model()
    self.assertEqual(model.limit(5.0, 0.999, v), 1.0)
    self.assertEqual(model.limit(-5.0, -0.999, v), -1.0)


if __name__ == "__main__":
  unittest.main()

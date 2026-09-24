import math
import unittest

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.honda.steer_torque_learner import (ALPHA_MAX, FACTOR_MAX, FACTOR_MIN, LAT_ALPHA_MAX, LAT_AXIS_FRAME_KEY,
                                                    LAT_SLOTS, SPEED_SLOTS, TORQUE_SLOTS, SteerTorqueLearner,
                                                    lat_pct_depart_frame, path_learning_curv_err)

MAX_LAT_ACCEL = 1.8


def make_learner(params=None):
  store = dict(params or {})
  return SteerTorqueLearner(MAX_LAT_ACCEL, store.get)


def step(learner, torque, v_ego, desired_la, actual_la, last_torque=None, lat_active=True, steer_control_active=True,
         steering_pressed=False, steering_angle_deg=15.0, steering_rate_deg=5.0):
  # feed lateral accelerations directly; the learner sees them as curvature * v^2
  v_sq = v_ego * v_ego
  if last_torque is None:
    last_torque = learner.prev_output  # unconstrained: applied exactly what was asked
  return learner.update(torque, last_torque, lat_active, steer_control_active, steering_pressed,
                        v_ego, desired_la / v_sq, actual_la / v_sq, steering_angle_deg, steering_rate_deg)


class TestSteerTorqueLearner(unittest.TestCase):
  def test_param_key_layout(self):
    keys = SteerTorqueLearner.param_keys()
    # 20 lat slots (0% frozen) + 10 torque slots (0% frozen) + 7 speed slots (30 mph frozen), factor + alpha each
    assert len(keys) == 2 * (20 + 10 + 7)
    assert len(set(keys)) == len(keys)
    assert "HondaSteerLatFactorP000Params" not in keys
    assert "HondaSteerTorqueFactor000Params" not in keys
    assert "HondaSteerSpeedFactor30Params" not in keys
    for expected in ("HondaSteerLatFactorN100Params", "HondaSteerLatAlphaP010Params", "HondaSteerTorqueFactor100Params",
                     "HondaSteerTorqueAlpha010Params", "HondaSteerSpeedFactor00Params", "HondaSteerSpeedAlpha70Params"):
      assert expected in keys
    learner = make_learner()
    assert set(learner.learned_values()) - {LAT_AXIS_FRAME_KEY} == set(keys)
    assert all(pos in dict(LAT_SLOTS) for pos in range(-100, 101, 10))
    assert all(pos in dict(TORQUE_SLOTS) for pos in range(0, 101, 10))
    assert all(pos in dict(SPEED_SLOTS) for pos in range(0, 71, 10))

  def test_identity_before_learning(self):
    learner = make_learner()
    v = 30 * CV.MPH_TO_MS
    for torque in (-1.0, -0.5, -0.1, 0.1, 0.5, 1.0):
      out = step(learner, torque, v, desired_la=0.0, actual_la=0.0)
      assert math.isclose(out, torque, abs_tol=1e-9)
    assert step(learner, 0.0, v, 0.0, 0.0) == 0.0

  def test_hat_weights(self):
    learner = make_learner()
    w = learner.lat.weights(-95.0)
    assert math.isclose(w[-100], 0.5) and math.isclose(w[-90], 0.5)
    assert math.isclose(sum(w.values()), 1.0)
    w = learner.speed.weights(500.0)
    assert w[70] == 1.0
    w = learner.torque.weights(-5.0)
    assert w[0] == 1.0

  def test_loads_and_clips_persisted_values(self):
    learner = make_learner({"HondaSteerSpeedFactor50Params": 1.5, "HondaSteerSpeedAlpha50Params": 9.0,
                            "HondaSteerLatFactorP100Params": "0.7", "HondaSteerLatAlphaP100Params": 9.0,
                            "HondaSteerTorqueFactor100Params": None, LAT_AXIS_FRAME_KEY: 2})
    assert learner.speed.factors[50] == 1.5
    assert learner.speed.alphas[50] == ALPHA_MAX
    assert learner.lat.factors[100] == 0.7
    assert learner.lat.alphas[100] == LAT_ALPHA_MAX
    assert learner.torque.factors[100] == 1.0
    # a param store that raises (unknown key) falls back to defaults instead of crashing

    def raising_get(key):
      raise KeyError(key)
    learner = SteerTorqueLearner(MAX_LAT_ACCEL, raising_get)
    assert learner.speed.factors[50] == 1.0

  def test_shaped_output_uses_all_three_axes(self):
    learner = make_learner()
    v = 50 * CV.MPH_TO_MS
    learner.speed.factors[50] = 1.2
    learner.torque.factors[50] = 1.1
    learner.lat.factors[50] = 1.05
    learner.lat.alphas[50] = 0.02
    # away from center at +50% |lat g| uses the +50% lat slot (angle/rate same sign)
    out = step(learner, 0.5, v, desired_la=0.9, actual_la=0.9, steering_angle_deg=15.0, steering_rate_deg=5.0)
    assert math.isclose(out, 0.5 * 1.2 * 1.1 * 1.05 + 0.02, rel_tol=1e-9)
    assert math.isclose(learner.lat_pct, 50.0, abs_tol=1e-3)
    # toward center at the same |lat g| uses the -50% slot (rate opposes angle), not the +50 factors
    out = step(learner, -0.5, v, desired_la=-0.9, actual_la=0.9, steering_angle_deg=-15.0, steering_rate_deg=5.0)
    assert math.isclose(out, -(0.5 * 1.2 * 1.1), abs_tol=1e-3)
    assert math.isclose(learner.lat_pct, -50.0, abs_tol=1e-3)

  def test_blended_factors_telemetry(self):
    learner = make_learner()
    v = 50 * CV.MPH_TO_MS
    learner.lat.factors[50] = 1.1
    learner.torque.factors[50] = 1.2
    learner.speed.factors[50] = 0.9
    step(learner, 0.5, v, desired_la=0.9, actual_la=0.9, steering_angle_deg=12.0, steering_rate_deg=4.0)
    assert math.isclose(learner.blended_lat_factor, 1.1, abs_tol=1e-3)
    assert math.isclose(learner.blended_torque_factor, 1.2, abs_tol=1e-3)
    assert math.isclose(learner.blended_speed_factor, 0.9, abs_tol=1e-3)

  def test_frozen_slots_never_move(self):
    learner = make_learner()
    v = 30 * CV.MPH_TO_MS  # exactly the frozen speed slot
    for _ in range(500):
      # tiny torque (0% torque slot), zero current lat accel (0% lat slot), persistent undershoot
      step(learner, 0.005, v, desired_la=0.9, actual_la=0.0, steering_angle_deg=0.0, steering_rate_deg=0.0)
    assert learner.speed.factors[30] == 1.0 and learner.speed.alphas[30] == 0.0
    assert learner.torque.factors[0] == 1.0 and learner.torque.alphas[0] == 0.0
    assert learner.lat.factors[0] == 1.0 and learner.lat.alphas[0] == 0.0
    assert "HondaSteerSpeedFactor30Params" not in learner.learned_values()
    # the 10% torque slot next door does pick up the weight share of the undershoot
    assert learner.torque.alphas[10] > 0.0

  def test_learning_direction(self):
    v = 50 * CV.MPH_TO_MS
    # undershoot in the torque direction -> grow factor and alpha at the active slots
    learner = make_learner()
    for _ in range(200):
      step(learner, 0.5, v, desired_la=1.2, actual_la=0.9)
    assert learner.speed.factors[50] > 1.0 and learner.speed.alphas[50] > 0.0
    assert learner.torque.factors[50] > 1.0 and learner.torque.alphas[50] > 0.0
    assert learner.lat.factors[50] > 1.0 and learner.lat.alphas[50] > 0.0
    assert learner.speed.factors[50] > 1.0 and learner.torque.factors[50] > 1.0
    # overshoot -> shrink
    learner = make_learner()
    for _ in range(200):
      step(learner, 0.5, v, desired_la=0.6, actual_la=0.9)
    assert learner.speed.factors[50] < 1.0 and learner.speed.alphas[50] < 0.0
    # mirror image: right torque with a right-side undershoot grows the same way
    learner = make_learner()
    for _ in range(200):
      step(learner, -0.5, v, desired_la=-1.2, actual_la=-0.9, steering_angle_deg=-15.0, steering_rate_deg=-5.0)
    assert learner.speed.factors[50] > 1.0 and learner.torque.factors[50] > 1.0

  def test_learning_gates(self):
    v = 50 * CV.MPH_TO_MS

    def run(**kwargs):
      learner = make_learner()
      before = learner.learned_values()
      for _ in range(50):
        step(learner, 0.5, v, desired_la=1.2, actual_la=0.9, **kwargs)
      return learner, learner.learned_values() != before

    _, learned = run()
    assert learned
    _, learned = run(lat_active=False)
    assert not learned
    _, learned = run(steer_control_active=False)
    assert not learned
    _, learned = run(steering_pressed=True)
    assert not learned
    # downstream limiter blocked a meaningful shaped request -> pause
    learner = make_learner()
    learner.prev_output = 0.5
    before = learner.learned_values()
    for _ in range(50):
      step(learner, 0.5, v, desired_la=1.2, actual_la=0.9, last_torque=0.1)
    assert learner.learned_values() == before
    # stopped car: curvature * v^2 is meaningless
    learner = make_learner()
    before = learner.learned_values()
    for _ in range(50):
      step(learner, 0.5, 0.5, desired_la=0.1, actual_la=0.0)
    assert learner.learned_values() == before

  def test_pauses_at_saturated_output(self):
    v = 50 * CV.MPH_TO_MS
    learner = make_learner()
    before = learner.learned_values()
    for _ in range(50):
      out = step(learner, 1.0, v, desired_la=1.8, actual_la=1.0)
      assert out == 1.0
    assert learner.learned_values() == before
    assert not learner.learning
    # once a factor below 1 pulls the output off the rail, learning resumes
    learner.torque.factors[100] = 0.8
    out = step(learner, 1.0, v, desired_la=1.8, actual_la=1.0)
    assert out < 1.0
    assert learner.learning

  def test_output_bounded_and_sign_preserving(self):
    v = 50 * CV.MPH_TO_MS
    learner = make_learner()
    for axis in learner.axes:
      for pos in axis.positions:
        if pos != axis.frozen:
          axis.factors[pos] = FACTOR_MAX
          axis.alphas[pos] = LAT_ALPHA_MAX if axis.name == "lat" else ALPHA_MAX
    assert step(learner, 0.9, v, 0.9, 0.9) == 1.0
    assert step(learner, -0.9, v, -0.9, -0.9) == -1.0

  def test_negative_alpha_sum_inverts_output(self):
    v = 50 * CV.MPH_TO_MS
    learner = make_learner()
    learner.lat.alphas[50] = -LAT_ALPHA_MAX
    learner.torque.alphas[50] = -ALPHA_MAX
    learner.speed.alphas[50] = -ALPHA_MAX
    out = step(learner, 0.5, v, desired_la=0.9, actual_la=0.9)
    assert out < 0.0
    assert out == -1.0  # product 0.5 + alpha sum -1.7 clips to -1.0

  def test_poisoned_alphas_can_drive_output_negative(self):
    v = 17 * CV.MPH_TO_MS
    store = {}
    for key in SteerTorqueLearner.param_keys():
      if "LatAlpha" in key:
        store[key] = -LAT_ALPHA_MAX
      elif "Alpha" in key:
        store[key] = -ALPHA_MAX
      else:
        store[key] = FACTOR_MIN
    store[LAT_AXIS_FRAME_KEY] = 2
    learner = make_learner(store)
    out = step(learner, 1.0, v, desired_la=0.5, actual_la=0.5, steering_angle_deg=12.0, steering_rate_deg=4.0)
    assert out < 0.0

  def test_path_learning_curv_err_same_sign_turns(self):
    assert path_learning_curv_err(1.2, 0.9, MAX_LAT_ACCEL) > 0.0
    assert path_learning_curv_err(0.6, 0.9, MAX_LAT_ACCEL) < 0.0
    assert path_learning_curv_err(-1.2, -0.9, MAX_LAT_ACCEL) > 0.0

  def test_depart_center_lat_index(self):
    assert lat_pct_depart_frame(0.9, MAX_LAT_ACCEL, 1.0) == 50.0
    assert lat_pct_depart_frame(0.9, MAX_LAT_ACCEL, -1.0) == -50.0
    assert lat_pct_depart_frame(0.9, MAX_LAT_ACCEL, 0.0) == 0.0
    learner = make_learner()
    v = 50 * CV.MPH_TO_MS
    step(learner, 0.5, v, 0.9, 0.9, steering_angle_deg=20.0, steering_rate_deg=8.0)
    assert learner.depart_sign == 1.0
    assert math.isclose(learner.lat_pct, 50.0, abs_tol=1e-3)
    step(learner, 0.5, v, 0.9, 0.9, steering_angle_deg=20.0, steering_rate_deg=-8.0)
    assert learner.depart_sign == -1.0
    assert math.isclose(learner.lat_pct, -50.0, abs_tol=1e-3)

  def test_legacy_lat_table_reset_without_frame_version(self):
    store = {"HondaSteerLatFactorP050Params": 0.6, "HondaSteerLatAlphaP050Params": 0.5}
    learner = make_learner(store)
    assert learner.lat.factors[50] == 1.0
    assert learner.lat.alphas[50] == 0.0

  def test_clamps_hold_under_sustained_error(self):
    v = 50 * CV.MPH_TO_MS
    learner = make_learner()
    for _ in range(200000):
      step(learner, 0.5, v, desired_la=1.8, actual_la=0.0)
    for axis in learner.axes:
      alpha_lim = LAT_ALPHA_MAX if axis.name == "lat" else ALPHA_MAX
      for pos in axis.positions:
        assert FACTOR_MIN <= axis.factors[pos] <= FACTOR_MAX
        assert -alpha_lim <= axis.alphas[pos] <= alpha_lim

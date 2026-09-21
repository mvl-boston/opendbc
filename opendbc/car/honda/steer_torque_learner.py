"""Self-learning steering torque shaping for Honda.

Sits between the lateral controller's torque request (actuators.torque) and the
carcontroller's rate limiter, and learns a multiplicative factor and an additive
alpha (``a*x + b``) as a function of three operating-point inputs, on the same
pattern as the Nidec speedfactor/speedalpha and gasfactor/gasalpha channels:

* current lateral acceleration as a percentage of the car's maxLateralAccel,
  signed in the torque frame (negative when the observed acceleration is
  against the requested torque), slots every 10% from -100% to +100%;
* the magnitude of the requested torque (|actuators.torque|, 0..1), slots every
  10% from 0% to 100%;
* vehicle speed, slots every 10 mph from 0 to 70 mph.

Each slot holds its own factor/alpha pair, each axis blends the two slots
bracketing the current input with piecewise-linear hat weights (exactly like
``band_weights`` in carcontroller) and learns in proportion to those weights.
One slot per axis is deliberately frozen at identity (factor 1, alpha 0) and is
never learned or persisted: 0% lateral accel, 0% torque and 30 mph. Those are
the operating points the openpilot torque controller (friction, latAccelFactor,
its own integrator) is expected to own, so the learned tables only describe how
the car deviates from that baseline across the curve. Because the frozen slots
are still interpolation nodes, the correction fades smoothly to nothing around
them and the three axes cannot all absorb the same constant offset.

Learning compares the requested and observed lateral acceleration. The request
is ``actuators.curvature * vEgo^2``: the desired curvature handed to the torque
controller, i.e. the pre-feedforward lateral acceleration demand which already
carries the steer actuator delay compensation upstream, so a lagged response is
not billed to the tables as steady-state error. The observation is
``CC.currentCurvature * vEgo^2`` from the vehicle model. Both are curvature
based, so roll compensation cancels in the error. The error is expressed in the
torque frame (positive means more torque in the requested direction would have
helped), normalized by maxLateralAccel, and split evenly across the three axes
so the total adaptation rate does not triple.

Learning pauses whenever the effect of a change would not be observable: the
rate limiter (or any other downstream clip) constrained the previous request,
the shaped output is already at +-1.0, lateral control is inactive or the EPS is
not accepting commands, the driver is steering, or the car is (nearly) stopped
where curvature*v^2 carries no information.

Everything is bounded: per-axis factors clip to [FACTOR_MIN, FACTOR_MAX], alphas
to +-ALPHA_MAX, and the shaped output never flips the sign of the request or
exceeds +-1.0.
"""

from opendbc.car.common.conversions import Conversions as CV

# per-tick (100 Hz) adaptation rates, applied after the hat weight and the 1/3 axis split
FACTOR_RATE = 0.003
ALPHA_RATE = 0.001
FACTOR_MIN = 0.5
FACTOR_MAX = 10.0
ALPHA_MAX = 0.1
# below this speed curvature*v^2 is too small a fraction of maxLateralAccel to learn from
MIN_LEARN_SPEED = 1.0  # m/s
# torque requests this small have no usable direction for the torque-frame sign convention
MIN_TORQUE = 1e-3
# tolerance for "the downstream limiter changed what we asked for"
CONSTRAINED_TOL = 1e-4


def _pct_key(pct):
  # -100 -> "N100", 10 -> "P010" (signed axes); 10 -> "010" (unsigned axes)
  if pct < 0:
    return f"N{-pct:03d}"
  return f"P{pct:03d}"


class LearnedAxis:
  """One input axis: sorted slots, each with a factor/alpha pair, one frozen at identity."""

  def __init__(self, name, slots, frozen, key_fmt, param_get):
    # slots: sequence of (position, key_suffix); positions strictly increasing
    self.name = name
    self.positions = [pos for pos, _ in slots]
    self.suffixes = dict(slots)
    assert frozen in self.suffixes, f"{name}: frozen slot {frozen} is not a slot"
    assert self.positions == sorted(self.positions), f"{name}: slots must be sorted"
    self.frozen = frozen
    self.keys = {pos: key_fmt.format(kind="Factor", slot=self.suffixes[pos]) for pos in self.positions if pos != frozen}
    self.alpha_keys = {pos: key_fmt.format(kind="Alpha", slot=self.suffixes[pos]) for pos in self.positions if pos != frozen}
    self.factors = {pos: 1.0 for pos in self.positions}
    self.alphas = {pos: 0.0 for pos in self.positions}
    for pos in self.positions:
      if pos == frozen:
        continue
      self.factors[pos] = _clip(_load(param_get, self.keys[pos], 1.0), FACTOR_MIN, FACTOR_MAX)
      self.alphas[pos] = _clip(_load(param_get, self.alpha_keys[pos], 0.0), -ALPHA_MAX, ALPHA_MAX)

  def weights(self, x):
    # hat-function weights of piecewise-linear interpolation across the slots: the two slots
    # bracketing x share the weight (summing to 1); outside the grid the edge slot has all of it
    weights = {pos: 0.0 for pos in self.positions}
    if x <= self.positions[0]:
      weights[self.positions[0]] = 1.0
    elif x >= self.positions[-1]:
      weights[self.positions[-1]] = 1.0
    else:
      for lo, hi in zip(self.positions[:-1], self.positions[1:], strict=True):
        if lo <= x <= hi:
          frac = (x - lo) / (hi - lo)
          weights[lo] = 1.0 - frac
          weights[hi] = frac
          break
    return weights

  def blend(self, weights):
    factor = sum(w * self.factors[pos] for pos, w in weights.items())
    alpha = sum(w * self.alphas[pos] for pos, w in weights.items())
    return factor, alpha

  def learn(self, weights, err, torque_mag):
    # multiplicative factor update scaled by the request magnitude (a factor has no authority at
    # zero torque), additive alpha update; each slot learns in proportion to its hat weight,
    # the frozen slot never moves
    for pos, w in weights.items():
      if w == 0.0 or pos == self.frozen:
        continue
      self.factors[pos] = _clip(self.factors[pos] * (1.0 + w * FACTOR_RATE * err * torque_mag), FACTOR_MIN, FACTOR_MAX)
      self.alphas[pos] = _clip(self.alphas[pos] + w * ALPHA_RATE * err, -ALPHA_MAX, ALPHA_MAX)

  def learned_values(self):
    values = {}
    for pos in self.positions:
      if pos == self.frozen:
        continue
      values[self.keys[pos]] = float(self.factors[pos])
      values[self.alpha_keys[pos]] = float(self.alphas[pos])
    return values


def _clip(value, lo, hi):
  return float(min(max(value, lo), hi))


def _load(param_get, key, default):
  # a missing or not-yet-registered key must never take the car controller down
  if param_get is None:
    return float(default)
  try:
    value = param_get(key)
  except Exception:
    return float(default)
  if value is None:
    return float(default)
  try:
    return float(value)
  except (TypeError, ValueError):
    return float(default)


LAT_SLOTS = tuple((pct, _pct_key(pct)) for pct in range(-100, 101, 10))
TORQUE_SLOTS = tuple((pct, f"{pct:03d}") for pct in range(0, 101, 10))
SPEED_SLOTS = tuple((mph, f"{mph:02d}") for mph in range(0, 71, 10))
LAT_FROZEN = 0
TORQUE_FROZEN = 0
SPEED_FROZEN = 30

LAT_KEY_FMT = "HondaSteerLat{kind}{slot}Params"
TORQUE_KEY_FMT = "HondaSteerTorque{kind}{slot}Params"
SPEED_KEY_FMT = "HondaSteerSpeed{kind}{slot}Params"


class SteerTorqueLearner:
  def __init__(self, max_lat_accel, param_get=None):
    # maxLateralAccel is 0 for cars without torque data; fall back to a typical value so the
    # percentage axis stays finite rather than disabling the learner outright
    self.max_lat_accel = float(max_lat_accel) if max_lat_accel and max_lat_accel > 0.1 else 1.8
    self.lat = LearnedAxis("lat", LAT_SLOTS, LAT_FROZEN, LAT_KEY_FMT, param_get)
    self.torque = LearnedAxis("torque", TORQUE_SLOTS, TORQUE_FROZEN, TORQUE_KEY_FMT, param_get)
    self.speed = LearnedAxis("speed", SPEED_SLOTS, SPEED_FROZEN, SPEED_KEY_FMT, param_get)
    self.prev_output = 0.0
    # telemetry for the last update() call
    self.lat_pct = 0.0
    self.err = 0.0
    self.learning = False
    self.output = 0.0

  @property
  def axes(self):
    return (self.lat, self.torque, self.speed)

  def update(self, torque, last_torque, lat_active, steer_control_active, steering_pressed,
             v_ego, desired_curvature, current_curvature):
    """Shape the torque request and learn from the lateral acceleration tracking error.

    torque:               actuators.torque, the lateral controller's request [-1, 1]
    last_torque:          the torque the carcontroller actually applied on the previous tick
                          (after its rate limiter and any other clips); compared against the
                          previous shaped output to detect a constraining limiter
    desired_curvature:    actuators.curvature (pre-feedforward, delay-compensated demand)
    current_curvature:    CC.currentCurvature (vehicle model observation)
    Returns the shaped torque to feed the rate limiter instead of actuators.torque.
    """
    torque = float(torque)
    v_ego = float(v_ego)
    constrained = abs(float(last_torque) - self.prev_output) > CONSTRAINED_TOL

    if abs(torque) < MIN_TORQUE:
      # no direction to work in: pass through, learn nothing
      self.lat_pct = 0.0
      self.err = 0.0
      self.learning = False
      self.output = self.prev_output = torque
      return torque

    sign = 1.0 if torque > 0.0 else -1.0
    torque_mag = abs(torque)
    v_sq = v_ego * v_ego
    desired_lat_accel = float(desired_curvature) * v_sq
    actual_lat_accel = float(current_curvature) * v_sq

    # torque-frame inputs: positive lateral accel means the car is already accelerating the way the
    # torque is pushing, negative means the torque is fighting the current lateral accel
    self.lat_pct = _clip(sign * actual_lat_accel / self.max_lat_accel * 100.0, -100.0, 100.0)
    torque_pct = torque_mag * 100.0
    speed_mph = _clip(v_ego * CV.MS_TO_MPH, 0.0, float(SPEED_SLOTS[-1][0]))

    lat_w = self.lat.weights(self.lat_pct)
    torque_w = self.torque.weights(torque_pct)
    speed_w = self.speed.weights(speed_mph)
    lat_f, lat_a = self.lat.blend(lat_w)
    torque_f, torque_a = self.torque.blend(torque_w)
    speed_f, speed_a = self.speed.blend(speed_w)

    # factors multiply, alphas add; the shaped magnitude keeps the request's sign and stays in [0, 1]
    shaped_mag = torque_mag * lat_f * torque_f * speed_f + lat_a + torque_a + speed_a
    shaped_mag = _clip(shaped_mag, 0.0, 1.0)
    output = sign * shaped_mag

    # tracking error in the torque frame, as a fraction of maxLateralAccel: positive when more
    # torque in the requested direction would have closed the gap
    self.err = _clip(sign * (desired_lat_accel - actual_lat_accel) / self.max_lat_accel, -1.0, 1.0)

    self.learning = bool(lat_active) and bool(steer_control_active) and (not steering_pressed) and \
                    (v_ego > MIN_LEARN_SPEED) and (not constrained) and (shaped_mag < 1.0)
    if self.learning:
      # the same error drives all three axes, so each gets a third of the rate
      err_share = self.err / 3.0
      self.lat.learn(lat_w, err_share, torque_mag)
      self.torque.learn(torque_w, err_share, torque_mag)
      self.speed.learn(speed_w, err_share, torque_mag)

    self.output = self.prev_output = output
    return output

  def learned_values(self):
    values = {}
    for axis in self.axes:
      values.update(axis.learned_values())
    return values

  @staticmethod
  def param_keys():
    # every persisted key, in a stable order (lat, torque, speed; factor before alpha per slot)
    keys = []
    for slots, frozen, fmt in ((LAT_SLOTS, LAT_FROZEN, LAT_KEY_FMT),
                               (TORQUE_SLOTS, TORQUE_FROZEN, TORQUE_KEY_FMT),
                               (SPEED_SLOTS, SPEED_FROZEN, SPEED_KEY_FMT)):
      for pos, suffix in slots:
        if pos == frozen:
          continue
        keys.append(fmt.format(kind="Factor", slot=suffix))
        keys.append(fmt.format(kind="Alpha", slot=suffix))
    return keys

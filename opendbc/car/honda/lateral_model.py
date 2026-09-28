"""Honda lateral plant model: a speed-dependent lateral-accel-per-torque gain, identified online and
used as an additive feedforward correction on top of openpilot's torque controller.

Why this exists
---------------
openpilot's torque controller assumes ``lat_accel = latAccelFactor * torque``, one constant for all
speeds. On the Nidec Hondas that is only true on the highway: the MDX 3G delivers ~0.4 m/s^2 per unit
wire torque at 7 m/s and ~2.2 at 28 m/s (routes 0000010e / 0000010f). The controller covers the gap
with a large low-speed proportional gain (``KP_INTERP``), i.e. with feedback that arrives late, which
is why turn entries and return-to-straight at town speeds lag until the driver helps.

What the model does
-------------------
``gain(v)`` is a per-speed table of the car's real m/s^2-per-unit-torque, seeded from the two routes
and refined online. Each tick the car controller adds

    correction = -desired_lat_accel / gain(v) - (-desired_lat_accel / latAccelFactor)

to the controller's request, i.e. it swaps the controller's linear feedforward for the model's. The
P/I feedback path passes through with gain exactly 1, so the model cannot change the loop gain and
cannot destabilize the loop no matter what it has learned; a wrong table only costs tracking error,
which the controller's feedback then removes. This is the difference from a multiplicative shaper on
the summed output, which scales feedback and feedforward together (the highway limit cycle in route
0000010f was that).

How it learns
-------------
Plant identification rather than tracking-error integration: the measured lateral accel is regressed
on the wire torque that actually went to the EPS (delayed by the actuator delay, both low-passed at
``FILTER_TAU`` so only the quasi-static content is fitted), with a normalized LMS update spread over
the two neighbouring speed bins. It identifies a bounded physical quantity, so it cannot drift the way
an integrator of delay-lag error does, and it is indifferent to rate limiting, clipping and saturation
because those all act on the wire torque it regresses against. The only unmodelled input is the
driver, so learning pauses while ``steeringPressed`` and for ``PRESS_HOLDOFF`` afterwards.

Reporting
---------
``gain_now`` (m/s^2 per unit torque at the current speed), ``ff_correction`` (torque added this tick)
and ``learning`` (identification ran this tick) are exposed for the actuatorsOutput telemetry slots.
"""
from collections import deque

import numpy as np

from opendbc.car import DT_CTRL
from opendbc.car.common.conversions import Conversions as CV

# speed bins (mph, for readable Params keys) and the seed gains in m/s^2 per unit wire torque, identified
# offline on routes 3792d010590cb83a|0000010e, |0000010f (torque controller) and |00000111 (PID controller,
# no shaper: 20 mph 1.0-1.3, 30 mph 2.2-2.4, 40 mph 2.6-2.9 agree with the torque routes within ~30%)
GAIN_BINS_MPH = (5, 10, 15, 20, 30, 40, 50, 60, 70)
GAIN_PRIOR = (0.10, 0.25, 0.45, 0.80, 1.90, 2.40, 2.10, 2.30, 2.60)
GAIN_BINS_MS = tuple(mph * CV.MPH_TO_MS for mph in GAIN_BINS_MPH)
GAIN_MIN = 0.10
GAIN_MAX = 3.00
GAIN_KEY_FMT = "HondaLatGain{slot:02d}Params"

# identification
FILTER_TAU = 2.0                                  # s, common low-pass on wire torque and measured lat accel
FILTER_ALPHA = DT_CTRL / (FILTER_TAU + DT_CTRL)
WIRE_DELAY = 0.30                                 # s, actuator delay applied to the wire before filtering
LEARN_RATE = 0.002                                # normalized LMS step per tick
LEARN_NORM_EPS = 0.01                             # torque^2, keeps the normalized step finite near zero
MIN_LEARN_TORQUE = 0.15                           # |filtered wire| needed for excitation
MIN_LEARN_LAT_ACCEL = 0.15                        # m/s^2, |filtered lat accel| needed to be in a real curve
MIN_LEARN_SPEED = 2.0                             # m/s, curvature from steering angle is meaningless below
PRESS_HOLDOFF = 0.5                               # s, learning stays paused this long after steeringPressed

# feedforward correction
FF_CORRECTION_MAX = 0.75                          # normalized torque, bound on |model ff - controller ff|
DEFAULT_LAT_ACCEL_FACTOR = 1.8


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


class HondaLateralModel:
  def __init__(self, lat_accel_factor, param_get=None):
    laf = float(lat_accel_factor) if lat_accel_factor and lat_accel_factor > 0.1 else DEFAULT_LAT_ACCEL_FACTOR
    self.lat_accel_factor = laf
    self.gains = [_clip(_load(param_get, GAIN_KEY_FMT.format(slot=mph), prior), GAIN_MIN, GAIN_MAX)
                  for mph, prior in zip(GAIN_BINS_MPH, GAIN_PRIOR, strict=True)]
    self.wire_hist = deque([0.0] * max(int(round(WIRE_DELAY / DT_CTRL)), 1), maxlen=max(int(round(WIRE_DELAY / DT_CTRL)), 1))
    self.wire_filt = 0.0
    self.lat_accel_filt = 0.0
    self.press_holdoff = 0.0
    # telemetry for the last update() call
    self.gain_now = float(np.interp(0.0, GAIN_BINS_MS, self.gains))
    self.ff_correction = 0.0
    self.learning = False
    self.output = 0.0

  def gain(self, v_ego):
    return float(np.interp(v_ego, GAIN_BINS_MS, self.gains))

  def feedforward_correction(self, desired_curvature, v_ego):
    """Torque to add to the controller's request so the feedforward follows gain(v) instead of
    latAccelFactor. Sign: actuators.torque is positive to the right, curvature positive to the left."""
    desired_lat_accel = desired_curvature * v_ego * v_ego
    ff_model = -desired_lat_accel / self.gain(v_ego)
    ff_controller = -desired_lat_accel / self.lat_accel_factor
    return _clip(ff_model - ff_controller, -FF_CORRECTION_MAX, FF_CORRECTION_MAX)

  def update(self, request_torque, wire_torque, lat_active, steer_control_active, steering_pressed, v_ego,
             desired_curvature, current_curvature):
    """request_torque: controller output this tick; wire_torque: what went to the EPS last tick (after
    rate limiter and clips). Returns the corrected request, to be rate limited by the caller."""
    self._identify(wire_torque, current_curvature, v_ego, lat_active and steer_control_active, steering_pressed)

    self.gain_now = self.gain(v_ego)
    if not lat_active:
      self.ff_correction = 0.0
      self.output = float(request_torque)
      return self.output

    self.ff_correction = self.feedforward_correction(desired_curvature, v_ego)
    self.output = _clip(request_torque + self.ff_correction, -1.0, 1.0)
    return self.output

  def _identify(self, wire_torque, current_curvature, v_ego, active, steering_pressed):
    delayed_wire = self.wire_hist[0]
    self.wire_hist.append(float(wire_torque))
    # measured lat accel in torque sign convention (right positive) so that lat_accel ~= gain * wire
    measured = -current_curvature * v_ego * v_ego
    self.wire_filt += FILTER_ALPHA * (delayed_wire - self.wire_filt)
    self.lat_accel_filt += FILTER_ALPHA * (measured - self.lat_accel_filt)

    self.press_holdoff = PRESS_HOLDOFF if steering_pressed else max(self.press_holdoff - DT_CTRL, 0.0)
    x = self.wire_filt
    y = self.lat_accel_filt
    self.learning = bool(active and self.press_holdoff <= 0.0 and v_ego > MIN_LEARN_SPEED
                         and abs(x) > MIN_LEARN_TORQUE and abs(y) > MIN_LEARN_LAT_ACCEL and np.sign(x) == np.sign(y))
    if not self.learning:
      return

    pos = float(np.interp(v_ego, GAIN_BINS_MS, range(len(GAIN_BINS_MS))))
    lo = int(np.floor(pos))
    hi = min(lo + 1, len(self.gains) - 1)
    frac = pos - lo
    for idx, weight in ((lo, 1.0 - frac), (hi, frac)):
      if weight <= 0.0:
        continue
      err = y - self.gains[idx] * x
      self.gains[idx] = _clip(self.gains[idx] + LEARN_RATE * weight * err * x / (x * x + LEARN_NORM_EPS), GAIN_MIN, GAIN_MAX)

  def learned_values(self):
    return {GAIN_KEY_FMT.format(slot=mph): float(g) for mph, g in zip(GAIN_BINS_MPH, self.gains, strict=True)}

  @staticmethod
  def param_keys():
    return [GAIN_KEY_FMT.format(slot=mph) for mph in GAIN_BINS_MPH]

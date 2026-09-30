"""Honda lateral plant model: a speed-dependent lateral-accel-per-torque gain times a lateral-accel-
dependent centering shape, identified online and used as an additive feedforward correction on top of
openpilot's torque controller.

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
and refined online. ``shape(|lat_accel|)`` multiplies it: the EPS's self-aligning / return torque grows
with lateral acceleration, so a hard turn needs more wire torque per m/s^2 than a gentle one at the same
speed (the "centering force" the driver has to overcome near the lateral limit). The shape is 1.0
through the gentle band that defines what ``gain(v)`` means and a learned table above it. Each tick the
car controller adds

    correction = -desired_lat_accel / (gain(v) * shape(|desired_lat_accel|)) - (-desired_lat_accel / latAccelFactor)

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
the two neighboring speed bins. It identifies a bounded physical quantity, so it cannot drift the way
an integrator of delay-lag error does, and it is indifferent to rate limiting, clipping and saturation
because those all act on the wire torque it regresses against. The only unmodelled input is the
driver, so learning pauses while ``steeringPressed`` and for ``PRESS_HOLDOFF`` afterwards.

The two tables are trained on disjoint data so they cannot trade scale: the speed table learns only
inside the anchor band (gentle curves, where shape == 1 by definition) and the shape table only above
it, treating the gain as known. The shape is bounded at 1.0 and non-increasing in lateral accel: a
saturating EPS never delivers more per unit torque in a hard turn than in a gentle one. Route 0000011f
measured it at 29-36 mph: 2.7 m/s^2 per unit of lagged wire torque below 0.3, 1.9 at 0.4-0.5, 1.4 at
0.7-0.8, 1.27 at 0.8-0.9 (shape 1.0 -> 0.5 between 0.85 and 1.05 m/s^2), and 1.1-1.3 m/s^2 with the
wire pinned at 433 counts. The tables persisted before this bound held 1.5-1.6 there, which put the
feedforward *below* the linear one exactly where the car was undershooting with the wire pinned.

The shape path uses its own short filter (``SHAPE_FILTER_TAU``) on the lag-aligned pair rather than the
``FILTER_TAU`` one the speed table uses: hard-turn dwells between the driver's corrections last 0.5-1.5
s (route 0000011f: 136 pinned, unpressed dwells, median 0.06 s, none over 1.5 s), and a 2 s filter with
a quasi-static gate never admitted one (0 pinned samples in that route). The lag model does the work
instead; a trend gate on the filtered pair (``SHAPE_MAX_LEARN_JERK`` / ``SHAPE_MAX_LEARN_TORQUE_RATE``)
drops the S-bend transitions where it is least exact. Sub-threshold hand torque does not bias the
sample (pinned frames with |driver torque| 0-50 vs 200-300 counts measured 1.08 vs 1.14 m/s^2), and the
estimate does not move with a longer press hold-off (0.5-2.0 s), so ``PRESS_HOLDOFF`` stays short.

The correction is a function of the desired lateral accel and speed only, with no state of its own, so
the return from a hard turn does not depend on anything the torque controller's integrator accumulated
while the driver was overriding (openpilot freezes that integrator on ``steeringPressed``; the fork adds
a reset). With the shape below 1 above the anchor band the model's feedforward for a hard turn is larger
than the linear one and falls steeply as the desired lateral accel drops back through the knee, so the
request leaves the torque limit as soon as the planner asks for less than the car can deliver, before
the feedback has seen any error.

The correction is added to the controller's output, which is already clipped to unit torque. When that
output is saturated the P+I+F sum is beyond the clip, so a correction that opposes it (model gain above
``latAccelFactor``) would take torque away while the loop asks for more than exists; it fades out over
the last ``FF_SATURATION_FADE`` of request headroom.

Wire limits: ISO 11270 in lateral-accel space instead of a torque rate
-------------------------------------------------------------------
openpilot bounds lateral jerk and lateral accel to ISO 11270 (``opendbc/car/lateral.py``: 3.0 m/s^2,
5.0 m/s^3; the planner's ``clip_curvature`` uses the same two numbers; the angle-car safety check in
``opendbc/safety/lateral.h`` uses 3.0 + 0.06 g of road-roll tolerance for both). On torque cars that
target has been implemented as a per-car torque rate, ``STEER_DELTA_UP/DOWN``, and
``opendbc/car/tests/test_lateral_limits.py`` checks the rate against the jerk targets assuming the
plant is linear: ``lat_accel = MAX_LAT_ACCEL_MEASURED * torque`` at every speed. On the Nidec Hondas the
plant is not: the same torque rate is 0.3 m/s^3 at 5 mph (30x under the limit, so the wheel unwinds no
faster than the rate limiter lets it) and 7-8 m/s^3 at 35 mph (above it). The Honda rate limiter also
ran in normalized torque per tick (``STEER_DELTA * DT_CTRL`` = 0.03/tick, full scale in 0.33 s) while
the test evaluated it in CAN counts per tick (3/433), so the test never saw the number the car got.

``limit()`` replaces that rate limiter. It maps the last wire torque and the new request into lateral
accel with the identified speed gain, applies the ISO jerk and accel bounds there, and maps back, all
before the torque is scaled into CAN counts. The bound is therefore a constant vehicle response at
every speed: the allowed torque rate is ``MAX_LAT_JERK / gain(v)`` per second, small on the highway
where a unit of torque buys 2.5 m/s^2 and large in town where it buys 0.3. Only the speed table is
used, not the centering shape: a shape error scales the bound directly, and whether the saturation
is a function of lateral accel (as the shape table assumes) or of torque (an EPS assist limit; the
knee in route 0000011f sits at ~0.35-0.4 of full torque at the one speed with hard turns) is not yet
settled. With the shape in the bound the unwind from a pinned wire would run the first 0.7 of torque
in ~0.1 s (it buys only ~0.4 m/s^2 there) and the rest at the anchor-band rate; that is the next step
once the shape has been learned on more than one speed. The wire torque is not the vehicle's lateral accel: it goes
through ``WIRE_DELAY`` and ``PLANT_TAU`` first, so the bound applied here is on the quasi-static
lateral accel the wire commands, which is how ``test_lateral_limits`` defines it and an upper bound
on what the car does (route 0000011f measured lateral jerk p99 0.9 m/s^3 against a wire-implied p99
of 6.9 under the old limiter).

``WIRE_RATE_MAX`` is a separate backstop on the normalized torque rate. It is not an ISO term: below
~15 mph the gain is small enough that the jerk bound alone would let the wire swing full scale in a
few ticks, and the EPS and the driver's hands see the torque step itself. It permits a full swing in
0.1 s (the old limiter took 0.33 s) and never binds above ~15 mph.

Reporting, and staying compatible with an unmodified openpilot
--------------------------------------------------------------
``gain_now`` (effective m/s^2 per unit torque at the current speed and desired lateral accel, i.e.
``gain(v) * shape``), ``applied_correction`` (torque actually added this tick) and ``learning``
(identification ran this tick) are exposed for the actuatorsOutput telemetry slots (gas / brake / speed).

controlsd freezes the torque controller's integrator whenever ``|actuators.torque - actuatorsOutput.torque|``
exceeds 0.01 (its ``steer_limited_by_safety``), and torqued fits ``latAccelFactor`` to
``-actuatorsOutput.torque``. If the car controller reported the true wire torque, the correction would
trip that check on nearly every engaged tick (93-96% in routes 0000010e/0000010f, integrator |I| stuck
near 0.03) and torqued would learn the highway plant gain, which the model then has to fight. So the
car controller reports ``request + (wire - corrected request)``: the request plus only the limiting the
rate limiter / clips actually did. The integrator then freezes only on real limiting, and torqued sees
the plant *as corrected by this model*, whose feedforward is ``latAccelFactor`` by construction, so its
live estimate settles on the same number the correction is computed against. The real wire torque is
recoverable from the log as ``actuatorsOutput.torque + actuatorsOutput.brake``.

Persisting the tables needs the ``HondaLatGainNNParams`` and ``HondaLatShapeNNParams`` keys registered
in openpilot's ``common/params_keys.h`` (``param_keys()`` lists them); unregistered keys are silently
dropped by the param writer and that table simply restarts from the priors each drive.
"""
from collections import deque

import numpy as np

from opendbc.car import DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import ISO_LATERAL_ACCEL, ISO_LATERAL_JERK

# speed bins (mph, for readable Params keys) and the seed gains in m/s^2 per unit wire torque, identified
# offline on routes 3792d010590cb83a|0000010e, |0000010f (torque controller) and |00000111 (PID controller,
# no shaper: 20 mph 1.0-1.3, 30 mph 2.2-2.4, 40 mph 2.6-2.9 agree with the torque routes within ~30%)
GAIN_BINS_MPH = (5, 10, 15, 20, 30, 40, 50, 60, 70)
GAIN_PRIOR = (0.10, 0.25, 0.45, 0.80, 1.90, 2.40, 2.10, 2.30, 2.60)
GAIN_BINS_MS = tuple(mph * CV.MPH_TO_MS for mph in GAIN_BINS_MPH)
GAIN_MIN = 0.10
GAIN_MAX = 3.00
GAIN_KEY_FMT = "HondaLatGain{slot:02d}Params"

# Centering shape: the EPS's self-aligning / return torque grows with lateral acceleration, so the m/s^2
# delivered per unit wire torque is not the same in a hard turn as in a gentle one. shape(|lat_accel|)
# multiplies gain(v): the effective plant is lat_accel = gain(v) * shape(|lat_accel|) * torque. The shape
# is anchored at 1.0 for |lat_accel| <= SHAPE_ANCHOR_LAT_ACCEL (that band defines what gain(v) means) and
# learned above it in the bins below, so the two tables cannot trade scale. Priors are neutral (1.0). The
# table is bounded at 1.0 and kept non-increasing in lateral accel (a saturating EPS: each unit of torque
# buys less in a hard turn than in a gentle one, never more), so the extra torque at high lateral accel
# is at most 1 / SHAPE_MIN times the linear feedforward. Route 0000011f measured 0.5-0.7 at 1.0-1.3
# m/s^2 (29-36 mph); a persisted value above 1.0 clips to 1.0 at load.
SHAPE_ANCHOR_LAT_ACCEL = 0.75                     # m/s^2, shape == 1.0 at and below this
SHAPE_BINS_LAT_ACCEL = (1.0, 1.5, 2.0, 2.5, 3.0)  # m/s^2, learned bins (keys use tenths: 10, 15, ...)
SHAPE_PRIOR = (1.0, 1.0, 1.0, 1.0, 1.0)
SHAPE_MIN = 0.40
SHAPE_MAX = 1.00
SHAPE_KEY_FMT = "HondaLatShape{slot:02d}Params"
SHAPE_LEARN_RATE = 0.005                          # normalized LMS step per tick (samples are scarce: ~100-300 per route)
SHAPE_BINS_INTERP = (SHAPE_ANCHOR_LAT_ACCEL,) + SHAPE_BINS_LAT_ACCEL
# shape samples come from the lag-aligned pair through a short filter with a trend gate (see the module
# docstring): hard-turn dwells between the driver's corrections are 0.5-1.5 s long
SHAPE_FILTER_TAU = 0.30                           # s
SHAPE_MAX_LEARN_JERK = 0.50                       # m/s^3, |filtered lat accel| trend allowed for a shape sample
SHAPE_MAX_LEARN_TORQUE_RATE = 0.50                # normalized torque per second, same for the lagged wire

# identification
FILTER_TAU = 2.0                                  # s, common low-pass on wire torque and measured lat accel
FILTER_ALPHA = DT_CTRL / (FILTER_TAU + DT_CTRL)
SHAPE_FILTER_ALPHA = DT_CTRL / (SHAPE_FILTER_TAU + DT_CTRL)
WIRE_DELAY = 0.30                                 # s, actuator delay applied to the wire before the plant lag
# The car's lateral accel follows the wire as a first-order lag, not instantly: a lag of 1.0-1.3 s (after the
# 0.3 s delay) maximizes the wire / lat-accel correlation at 10-16 m/s in routes 0000010e, 0000010f and
# 00000114, and 1.0 s does so above 16 m/s. Regressing on the lagged wire keeps a sample taken mid-turn
# honest; without it every turn entry read the gain low and every exit read it high.
PLANT_TAU = 1.0
PLANT_ALPHA = DT_CTRL / (PLANT_TAU + DT_CTRL)
LEARN_RATE = 0.002                                # normalized LMS step per tick
LEARN_NORM_EPS = 0.01                             # torque^2, keeps the normalized step finite near zero
MIN_LEARN_TORQUE = 0.15                           # |filtered wire| needed for excitation
MIN_LEARN_LAT_ACCEL = 0.15                        # m/s^2, |filtered lat accel| needed to be in a real curve
# coarse quasi-static gate: neither filtered signal may be changing by more than this fraction of itself
# per FILTER_TAU (guards the sign flips of an S-bend, where the lag model is least exact)
MAX_LEARN_CHANGE = 0.50
MIN_LEARN_SPEED = 2.0                             # m/s, curvature from steering angle is meaningless below
PRESS_HOLDOFF = 0.5                               # s, learning stays paused this long after steeringPressed

# feedforward correction
FF_CORRECTION_MAX = 0.75                          # normalized torque, bound on |model ff - controller ff|
# normalized torque; a correction that opposes the request fades out over this much request headroom below 1.0
FF_SATURATION_FADE = 0.10
DEFAULT_LAT_ACCEL_FACTOR = 1.8

# wire limits (see the module docstring): the same 3 m/s^2 / 5 m/s^3 the planner holds the desired
# curvature to for every car (selfdrive/controls/lib/drive_helpers.py clip_curvature), applied here to
# the wire so the limit does not depend on the controller upstream. Jerk is symmetric; test_lateral_limits'
# 2.5 m/s^3 up-rate is a comfort margin on top of the same linear calculation, and on this plant the
# 1 s response lag provides that margin. No roll allowance: the planner adds it to the desired curvature,
# and with GAIN_MAX at 3.0 a unit of wire torque cannot command more than 3 m/s^2 anyway.
MAX_LAT_ACCEL = ISO_LATERAL_ACCEL                 # m/s^2
MAX_LAT_JERK_UP = ISO_LATERAL_JERK                # m/s^3, |lat accel| increasing
MAX_LAT_JERK_DOWN = ISO_LATERAL_JERK              # m/s^3, |lat accel| decreasing (return to center)
WIRE_RATE_MAX = 10.0                              # normalized torque per second, EPS / hands-on backstop


def _clip(value, lo, hi):
  return float(min(max(value, lo), hi))


def _shape_slot(lat_accel):
  # Params key slot for a shape bin: lateral accel in tenths of m/s^2 (1.5 -> 15)
  return int(round(lat_accel * 10))


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
    self.lat_accel_factor = (
      float(lat_accel_factor) if lat_accel_factor and lat_accel_factor > 0.1 else DEFAULT_LAT_ACCEL_FACTOR
    )
    self.gains = [_clip(_load(param_get, GAIN_KEY_FMT.format(slot=mph), prior), GAIN_MIN, GAIN_MAX)
                  for mph, prior in zip(GAIN_BINS_MPH, GAIN_PRIOR, strict=True)]
    self.shapes = [_clip(_load(param_get, SHAPE_KEY_FMT.format(slot=_shape_slot(la)), prior), SHAPE_MIN, SHAPE_MAX)
                   for la, prior in zip(SHAPE_BINS_LAT_ACCEL, SHAPE_PRIOR, strict=True)]
    self._project_shapes()
    self.wire_hist = deque([0.0] * max(int(round(WIRE_DELAY / DT_CTRL)), 1), maxlen=max(int(round(WIRE_DELAY / DT_CTRL)), 1))
    self.wire_lag = 0.0             # delayed wire through the plant lag: the lateral accel the wire has "earned" so far
    self.wire_filt = 0.0            # FILTER_TAU pair, speed table
    self.lat_accel_filt = 0.0
    self.wire_fast = 0.0            # SHAPE_FILTER_TAU pair, shape table
    self.lat_accel_fast = 0.0
    self.press_holdoff = 0.0
    # telemetry for the last update() call
    self.gain_now = float(np.interp(0.0, GAIN_BINS_MS, self.gains))   # effective gain at (v, |desired lat accel|)
    self.shape_now = 1.0
    self.ff_correction = 0.0        # model ff minus controller ff, before the unit-torque clip
    self.applied_correction = 0.0   # output minus request, i.e. what was actually added this tick
    self.learning = False           # either table updated this tick
    self.learning_gain = False
    self.learning_shape = False
    self.output = 0.0
    self.jerk_limited = False       # limit() clipped the wire on the lateral jerk bound this tick
    self.accel_limited = False      # limit() clipped the wire on the lateral accel bound this tick
    self.rate_limited = False       # limit() clipped the wire on the torque-rate backstop this tick

  def gain(self, v_ego):
    return float(np.interp(v_ego, GAIN_BINS_MS, self.gains))

  def shape(self, lat_accel):
    """Centering multiplier on gain(v) at this |lateral accel|: 1.0 through the anchor band, then the
    learned table, held at the last bin beyond it."""
    return float(np.interp(abs(lat_accel), SHAPE_BINS_INTERP, [1.0] + self.shapes))

  def effective_gain(self, v_ego, lat_accel):
    return self.gain(v_ego) * self.shape(lat_accel)

  def feedforward_correction(self, desired_curvature, v_ego):
    """Torque to add to the controller's request so the feedforward follows gain(v) * shape(|lat_accel|)
    instead of latAccelFactor. Sign: actuators.torque is positive to the right, curvature positive to the left."""
    desired_lat_accel = desired_curvature * v_ego * v_ego
    ff_model = -desired_lat_accel / self.effective_gain(v_ego, desired_lat_accel)
    ff_controller = -desired_lat_accel / self.lat_accel_factor
    return _clip(ff_model - ff_controller, -FF_CORRECTION_MAX, FF_CORRECTION_MAX)

  def update(self, request_torque, wire_torque, lat_active, steer_control_active, steering_pressed, v_ego,
             desired_curvature, current_curvature):
    """request_torque: controller output this tick; wire_torque: what went to the EPS last tick (after
    rate limiter and clips). Returns the corrected request, to be rate limited by the caller."""
    self._identify(wire_torque, current_curvature, v_ego, lat_active and steer_control_active, steering_pressed)

    desired_lat_accel = desired_curvature * v_ego * v_ego
    self.shape_now = self.shape(desired_lat_accel)
    self.gain_now = self.gain(v_ego) * self.shape_now
    if not lat_active:
      self.ff_correction = 0.0
      self.applied_correction = 0.0
      self.output = float(request_torque)
      return self.output

    self.ff_correction = self.feedforward_correction(desired_curvature, v_ego)
    correction = self.ff_correction
    # The correction swaps feedforwards; it is not a license to undo feedback. A saturated request means
    # the controller's P+I+F sum is beyond the unit clip, so a correction that opposes it (model gain above
    # latAccelFactor) would take torque away while the loop is asking for more than exists: route 00000114
    # 09:23:21 held the wire at 420/433 with the request at -1.00 and the lane lines solid. Fade it out
    # over the last FF_SATURATION_FADE of request headroom rather than switching it off at 1.0.
    if np.sign(correction) == -np.sign(request_torque):
      correction *= _clip((1.0 - abs(request_torque)) / FF_SATURATION_FADE, 0.0, 1.0)
    self.output = _clip(request_torque + correction, -1.0, 1.0)
    self.applied_correction = self.output - float(request_torque)
    return self.output

  def lat_accel_from_torque(self, torque, v_ego):
    """Quasi-static lateral accel the wire commands at this speed, speed table only (see module docstring)."""
    return self.gain(v_ego) * float(torque)

  def torque_from_lat_accel(self, lat_accel, v_ego):
    return float(lat_accel) / self.gain(v_ego)

  def limit(self, torque, last_torque, v_ego):
    """Bound the wire to ISO 11270 lateral jerk and lateral accel, computed in lateral-accel space with the
    identified speed gain, then to the torque-rate backstop and unit torque. torque: corrected request this
    tick; last_torque: what went to the EPS last tick. Returns the torque to send (normalized)."""
    la_last = self.lat_accel_from_torque(last_torque, v_ego)
    la_req = self.lat_accel_from_torque(torque, v_ego)
    up = MAX_LAT_JERK_UP * DT_CTRL
    down = MAX_LAT_JERK_DOWN * DT_CTRL
    # up-rate while |lat accel| grows, down-rate while it shrinks; a move through zero is down to zero
    # and up beyond it (same structure as apply_driver_steer_torque_limits)
    if la_last > 0.0:
      lo, hi = max(la_last - down, -up), la_last + up
    else:
      lo, hi = la_last - up, min(la_last + down, up)
    la_jerk = _clip(la_req, lo, hi)
    la_out = _clip(la_jerk, -MAX_LAT_ACCEL, MAX_LAT_ACCEL)
    self.jerk_limited = la_jerk != la_req
    self.accel_limited = la_out != la_jerk
    # untouched requests pass through bit-exact; only a bound that bit is mapped back through the gain
    out = float(torque) if la_out == la_req else self.torque_from_lat_accel(la_out, v_ego)
    backstop = _clip(out, last_torque - WIRE_RATE_MAX * DT_CTRL, last_torque + WIRE_RATE_MAX * DT_CTRL)
    self.rate_limited = backstop != out
    return _clip(backstop, -1.0, 1.0)

  def _identify(self, wire_torque, current_curvature, v_ego, active, steering_pressed):
    delayed_wire = self.wire_hist[0]
    self.wire_hist.append(float(wire_torque))
    self.wire_lag += PLANT_ALPHA * (delayed_wire - self.wire_lag)
    # measured lat accel in torque sign convention (right positive) so that lat_accel ~= gain * wire_lag
    measured = -current_curvature * v_ego * v_ego
    # the filter's own step is its rate of change over one tick; scaled to FILTER_TAU it is the fraction
    # of the signal still in transit, which is what the quasi-static gate below looks at
    dx = FILTER_ALPHA * (self.wire_lag - self.wire_filt)
    dy = FILTER_ALPHA * (measured - self.lat_accel_filt)
    self.wire_filt += dx
    self.lat_accel_filt += dy
    dxf = SHAPE_FILTER_ALPHA * (self.wire_lag - self.wire_fast)
    dyf = SHAPE_FILTER_ALPHA * (measured - self.lat_accel_fast)
    self.wire_fast += dxf
    self.lat_accel_fast += dyf

    self.press_holdoff = PRESS_HOLDOFF if steering_pressed else max(self.press_holdoff - DT_CTRL, 0.0)
    self.learning_gain = False
    self.learning_shape = False
    self.learning = False
    if not (active and self.press_holdoff <= 0.0 and v_ego > MIN_LEARN_SPEED):
      return

    # plant: y = gain(v) * shape(|y|) * x, with shape == 1.0 through the anchor band. Gentle curves train
    # the speed table and only the speed table; harder turns train the shape, seeing the gain as known.
    # Both regress on what the EPS actually got, so a pinned or clipped wire is a valid sample.
    x = self.wire_filt
    y = self.lat_accel_filt
    steady = (abs(dx) * FILTER_TAU / DT_CTRL <= MAX_LEARN_CHANGE * abs(x)
              and abs(dy) * FILTER_TAU / DT_CTRL <= MAX_LEARN_CHANGE * abs(y))
    if (steady and abs(x) > MIN_LEARN_TORQUE and abs(y) > MIN_LEARN_LAT_ACCEL and np.sign(x) == np.sign(y)
        and abs(y) <= SHAPE_ANCHOR_LAT_ACCEL):
      self.learning_gain = True
      pos = float(np.interp(v_ego, GAIN_BINS_MS, range(len(GAIN_BINS_MS))))
      lo = int(np.floor(pos))
      hi = min(lo + 1, len(self.gains) - 1)
      frac = pos - lo
      for idx, weight in ((lo, 1.0 - frac), (hi, frac)):
        if weight <= 0.0:
          continue
        err = y - self.gains[idx] * x
        self.gains[idx] = _clip(self.gains[idx] + LEARN_RATE * weight * err * x / (x * x + LEARN_NORM_EPS), GAIN_MIN, GAIN_MAX)

    # shape: the short-filtered pair, admitted while neither is trending (the lag model has aligned them;
    # what is left out is the S-bend transition it is least exact in)
    xf = self.wire_fast
    yf = self.lat_accel_fast
    trending = (abs(dyf) / DT_CTRL > SHAPE_MAX_LEARN_JERK or abs(dxf) / DT_CTRL > SHAPE_MAX_LEARN_TORQUE_RATE)
    if (not trending and abs(xf) > MIN_LEARN_TORQUE and abs(yf) > SHAPE_ANCHOR_LAT_ACCEL and np.sign(xf) == np.sign(yf)):
      self.learning_shape = True
      xg = self.gain(v_ego) * xf
      # a sample above the bound (more lateral accel than the speed table predicts for this torque) only
      # says "not saturated here"; it counts as a 1.0, not as its ratio, so it cannot outvote the saturated
      # samples in the same bin (route 0000011f: 1000 samples at 0.75-1.0 m/s^2 reading 1.1-1.3 against
      # 60 at 1.0-1.1 reading 0.5)
      yf = float(np.sign(yf) * min(abs(yf), SHAPE_MAX * abs(xg)))
      pos = float(np.interp(abs(yf), SHAPE_BINS_INTERP, range(len(SHAPE_BINS_INTERP))))
      lo = int(np.floor(pos))
      hi = min(lo + 1, len(SHAPE_BINS_INTERP) - 1)
      frac = pos - lo
      for idx, weight in ((lo, 1.0 - frac), (hi, frac)):
        if weight <= 0.0 or idx == 0:   # index 0 is the anchor, fixed at 1.0
          continue
        err = yf - self.shapes[idx - 1] * xg
        self.shapes[idx - 1] = _clip(self.shapes[idx - 1] + SHAPE_LEARN_RATE * weight * err * xg / (xg * xg + LEARN_NORM_EPS),
                                     SHAPE_MIN, SHAPE_MAX)
      self._project_shapes()
    self.learning = self.learning_gain or self.learning_shape

  def _project_shapes(self):
    # non-increasing in lateral accel, from the anchor's 1.0 down: a bin with no data of its own inherits
    # the saturation the last measured one showed rather than the neutral prior
    ceiling = SHAPE_MAX
    for i, s in enumerate(self.shapes):
      ceiling = min(ceiling, s)
      self.shapes[i] = ceiling

  def learned_values(self):
    values = {GAIN_KEY_FMT.format(slot=mph): float(g) for mph, g in zip(GAIN_BINS_MPH, self.gains, strict=True)}
    values.update({SHAPE_KEY_FMT.format(slot=_shape_slot(la)): float(s)
                   for la, s in zip(SHAPE_BINS_LAT_ACCEL, self.shapes, strict=True)})
    return values

  @staticmethod
  def param_keys():
    return ([GAIN_KEY_FMT.format(slot=mph) for mph in GAIN_BINS_MPH] +
            [SHAPE_KEY_FMT.format(slot=_shape_slot(la)) for la in SHAPE_BINS_LAT_ACCEL])

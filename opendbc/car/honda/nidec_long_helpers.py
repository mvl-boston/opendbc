"""Pure Nidec longitudinal helpers (importable without openpilot)."""
from collections import deque

# Keep in sync with opendbc/car/honda/carcontroller.py
NIDEC_MIN_ACCEL_SPEED_LEAD = 0.35
# m/s of lead below which the servo is never treated as saturated (see NIDEC_DV_SAT_MIN in carcontroller)
NIDEC_DV_SAT_MIN = 2.5
# m/s below the displayed set speed where the PCM's own set-speed governor takes over (see
# nidec_set_speed_governed); the gas-channel learners have always used this margin
NIDEC_SET_SPEED_MARGIN = 2.0

# Speed nodes (m/s) of the brake gain (see NidecBrakeGain). Dense at the bottom, where the decel a
# unit of brake command delivers changes fastest.
NIDEC_BRAKE_BANDS = (
  ("005", 0.5),
  ("015", 1.5),
  ("030", 3.0),
  ("050", 5.0),
  ("080", 8.0),
  ("120", 12.0),
  ("170", 17.0),
  ("250", 25.0),
)
NIDEC_BRAKE_GAIN_KEYS: dict[str, str] = {band: f"HondaBrakePID{band}Params" for band, _ in NIDEC_BRAKE_BANDS}
# node whose value is also written to the pre-banding HondaBrakePIDParams key, so a revert to the
# single-gain code starts from a mid-speed value
NIDEC_BRAKE_GAIN_LEGACY_BAND = "120"
NIDEC_BRAKE_GAIN_KI = 2.0
# the gain applied on the wire is 1 + factor, same as the old nidec_brake_pid (neg_limit 0): never below 1.0
NIDEC_BRAKE_GAIN_MIN = 0.0
NIDEC_BRAKE_GAIN_MAX = 4.0
# Coast decel (see NidecBrakeCoast): m/s2 the car sheds by itself while the brake request is up,
# taken off the request. The gas channel's alpha/factor pair on the brake side: this is the offset,
# the brake gain is the slope.
NIDEC_BRAKE_COAST_KEYS: dict[str, str] = {band: f"HondaBrakeCoast{band}Params" for band, _ in NIDEC_BRAKE_BANDS}
NIDEC_BRAKE_COAST_KI = 0.5  # m/s2 of coast decel per second per m/s2 of error
NIDEC_BRAKE_COAST_MIN = 0.0  # never negative: that would hold brake against a zero request (launch)
NIDEC_BRAKE_COAST_MAX = 1.5
# normalized brake at which the coast decel stops learning: at light braking an error is mostly
# offset, at heavy braking mostly gain, and the gain learner is already weighted by the brake
NIDEC_BRAKE_COAST_LEARN_MAX_BRAKE = 0.5
# seconds the gas wire must have been at zero before the coast decel learns. After a gas cut the
# engine keeps pulling while it runs down: MDX 3G, 160 gas cuts above 10 m/s on routes
# 3792d010590cb83a/00000279, /0000028c and /000002b1, aEgo is still 0.6 m/s2 above its settled
# value 0.4 s after the cut, 0.15 at 1.0 s, 0.09 at 1.2 s and settled at 1.5 s.
NIDEC_BRAKE_COAST_GAS_OFF_MIN = 1.5
# normalized brake per m/s2 (compute_gb_honda_nidec)
NIDEC_BRAKE_ACCEL_SCALE = 4.8


def nidec_speed_lead_mps(sf_eff: float, accel: float, alpha_eff: float) -> float:
  """PCM_SPEED lead (m/s) from the speed-channel blend."""
  speed_lead = sf_eff * accel + alpha_eff
  if accel > 0.05:
    speed_lead = sf_eff * accel + max(alpha_eff, -sf_eff * accel + NIDEC_MIN_ACCEL_SPEED_LEAD)
  return speed_lead


# Launch window, after first motion (see nidec_launch_lead / nidec_launch_gas). The fixed template
# (gas seed 100, lead 7.9 m/s) delivered 0.54-0.71 m/s2 on 8 of 9 MDX 3G launches on routes
# 3792d010590cb83a/00000279, /00000283, /0000028c and /000002b1 (1.04 on the ninth, engine running
# on a -3.6 deg grade) against plans of 0.2 to 1.56 m/s2.
NIDEC_LAUNCH_TEMPLATE_ACCEL = 0.6
# plan at and below which the lead is the general pipeline's (0.3-0.4 m/s2 delivered at 0.4-1.5 m/s
# with a 1.8-2.0 m/s lead on the same routes)
NIDEC_LAUNCH_LEAD_MIN_ACCEL = 0.3
# plan at and above which the gas seed is the full wire
NIDEC_LAUNCH_GAS_FULL_ACCEL = 1.2


def _interp(x: float, x0: float, x1: float, y0: float, y1: float) -> float:
  return y0 + (y1 - y0) * min(max((x - x0) / (x1 - x0), 0.0), 1.0)


def nidec_launch_lead(accel: float, dv_launch: float, general_lead: float) -> float:
  """PCM_SPEED lead (m/s) in the launch window once the car is moving.

  The learned launch lead is what a plan of NIDEC_LAUNCH_TEMPLATE_ACCEL or more gets. A gentler
  plan gets less, down to the general pipeline's lead at NIDEC_LAUNCH_LEAD_MIN_ACCEL: with the full
  lead a 0.2 m/s2 plan was answered with 0.6 (routes 28c t=2512, 2b1 t=385), and the brake came on
  at window exit with the plan still positive.
  """
  return _interp(accel, NIDEC_LAUNCH_LEAD_MIN_ACCEL, NIDEC_LAUNCH_TEMPLATE_ACCEL, min(general_lead, dv_launch), dv_launch)


def nidec_launch_gas(accel: float, gas_launch: float, gas_max: float) -> float:
  """PCM_GAS seed in the launch window once the car is moving.

  The learned seed is what a plan up to NIDEC_LAUNCH_TEMPLATE_ACCEL gets. A stronger plan gets
  more, the full wire at NIDEC_LAUNCH_GAS_FULL_ACCEL: with the seed alone plans of 0.9-1.56 m/s2
  were answered with 0.6-0.9 until the window closed (routes 279 t=936, 28c t=2131, 2b1 t=415), and
  the seed cannot learn its way up on an engine-off launch (no pedal evidence).
  """
  return _interp(accel, NIDEC_LAUNCH_TEMPLATE_ACCEL, NIDEC_LAUNCH_GAS_FULL_ACCEL, gas_launch, max(gas_launch, gas_max))


def nidec_speed_channel_pinned(dv_lead: float, dv_sat: float, dv_coast: float = NIDEC_DV_SAT_MIN) -> bool:
  """True when the lead the servo is answering is past its responsive band in either direction.

  Above dv_sat the servo is at its acceleration ceiling, below -dv_coast it is at its coast floor
  (fuel cut; the PCM has no brake authority through PCM_SPEED). In both regions
  d(response)/d(speed factor) = 0, so tracking error there is not speed-factor information and
  the speed-channel learners may only reduce. The negative side matters on cars whose coast
  floor is shallow: CR-V 2015 (route a6354c3dcf16e7cf/25) coasts at -0.4 m/s2 for every lead
  from -1 to -40 m/s, so a -0.5 m/s2 plan undershoots forever and err*accel (both negative) was
  the dominant growth fuel that took the highway speed factor from 4 to 23-43.
  """
  return dv_lead > dv_sat or dv_lead < -dv_coast


def nidec_set_speed_governed(v_ego: float, v_cruise: float, margin: float = NIDEC_SET_SPEED_MARGIN) -> bool:
  """True when the PCM's own set-speed governor, not our wire, decides the acceleration.

  The Nidec PCM stops accelerating ~3% below the CRUISE_SPEED it displays regardless of PCM_SPEED
  and PCM_GAS (CR-V 2015 route a6354c3dcf16e7cf/25: 20 s of PCM_SPEED = vEgo + 11 m/s, gas 198,
  plan +0.7, car held at 116-117 kph with the display at 120; aEgo +0.5 at 95-113 kph, +0.02 at
  115-117, -0.05 at 117-119 under the same wire). Undershoot there is not plant information for
  any longitudinal learner: it collapsed sat_accel 0.69 -> 0.37 and pinned the speed alpha at its
  cap on that route. Uses the same 2 m/s margin the gas-channel learners already apply.
  """
  return v_ego >= v_cruise - margin


def nidec_band_weights(bands, v_ego: float) -> dict[str, float]:
  """Hat-function weights of piecewise-linear interpolation across the band nodes.

  Exactly the two nodes bracketing v_ego get nonzero weight (summing to 1); outside the grid the
  edge node gets all of it. The weights blend the per-band values into the value in use and gate
  how much each band learns from a frame.
  """
  weights = {band: 0.0 for band, _ in bands}
  if v_ego <= bands[0][1]:
    weights[bands[0][0]] = 1.0
  elif v_ego >= bands[-1][1]:
    weights[bands[-1][0]] = 1.0
  else:
    for (band_lo, spd_lo), (band_hi, spd_hi) in zip(bands[:-1], bands[1:], strict=True):
      if spd_lo <= v_ego <= spd_hi:
        frac = (v_ego - spd_lo) / (spd_hi - spd_lo)
        weights[band_lo] = 1.0 - frac
        weights[band_hi] = frac
        break
  return weights


def nidec_brake_scale(factor: float, brake: float) -> float:
  """Multiplier on the normalized brake request (gain = 1 + factor, minimum 1.0).

  brake is unused; kept so call sites can pass the request level if a speed-dependent scale is added
  later. Matches upstream nidec_brake_pid neg_limit 0: the learned addition never goes negative.
  """
  return 1.0 + max(factor, 0.0)


class NidecBandedIntegrator:
  """One clamped integrator per NIDEC_BRAKE_BANDS node, blended and taught by the hat weights."""

  def __init__(self, values: dict[str, float], k_i: float, lower: float, upper: float):
    self.k_i, self.lower, self.upper = k_i, lower, upper
    self.values = {band: float(min(max(values[band], lower), upper)) for band, _ in NIDEC_BRAKE_BANDS}

  def at(self, v_ego: float) -> float:
    return sum(w * self.values[band] for band, w in nidec_band_weights(NIDEC_BRAKE_BANDS, v_ego).items())

  def update(self, error: float, v_ego: float, dt: float) -> None:
    for band, w in nidec_band_weights(NIDEC_BRAKE_BANDS, v_ego).items():
      if w > 0.0:
        self.values[band] = min(max(self.values[band] + w * self.k_i * error * dt, self.lower), self.upper)


class NidecBrakeGain(NidecBandedIntegrator):
  """Speed-banded integral brake gain: the old single nidec_brake_pid integrator, one per speed node.

  The decel the brake delivers per unit of command depends on speed. MDX 3G, 573 s of steady
  openpilot braking on routes 3792d010590cb83a/00000279, /00000283, /0000028c and /0000028f, with
  grade removed: 1.04 m/s2 per nominal m/s2 below 1 m/s, 0.84 at 2-3.5, 0.71 at 5-7.5, 0.60 above
  15, on top of a coast decel (the PCM is at PCM_SPEED 0 whenever the brake is on) that grows from
  about zero below 2 m/s to 0.6-0.7 m/s2 above 5. The gain that makes delivered equal commanded was
  1.2 below 1 m/s, 1.0 at 1-2, 0.8-0.95 at 2-7.5, 1.25 at 7.5-10, 1.6 at 10-15 and 2.7 at 15-20,
  while the single gain sat at 1.05-1.5 everywhere and could not go below 1. The remainder went to
  nidec_pid, which ran at its +0.2 limit at 2-5 m/s and at -0.36 (median) at 15-20 m/s while
  braking, and carried that offset into the gas command after each release.

  Each node integrates the same error as before in proportion to its hat weight at the speed the
  error belongs to, so the total learning rate is unchanged. The value is the factor; the gain is
  1 + factor.
  """

  def __init__(self, factors: dict[str, float]):
    super().__init__(factors, NIDEC_BRAKE_GAIN_KI, NIDEC_BRAKE_GAIN_MIN, NIDEC_BRAKE_GAIN_MAX)


class NidecBrakeCoast(NidecBandedIntegrator):
  """Speed-banded coast decel (m/s2), subtracted from the brake request.

  With the brake request up the gas is off and the car already slows by itself (fuel cut, regen,
  drag). That was modeled by the wind term, one fixed curve times the gas side's windfactor: 0.20
  m/s2 at 5 m/s, 0.65 at 12, 1.5 at 25 (windfactor 2.96). The same MDX 3G braking as above measures
  0.3 at 2-3.5 m/s, 0.5-0.6 from 3.5 to 15 and 0.7 above, and coasting with no brake at all agrees
  (0.5 at 10-15 m/s, 0.55 at 15-20, 0.7 above 20). So too much brake was asked for at 2-8 m/s and
  too little on the highway, where a 0.5 m/s2 request sent no brake at all. A gain cannot express
  that: at 3.5-7.5 m/s light braking needed a gain of 0.1-0.2 and heavier braking 0.9-1.1.

  Nodes seed from the wind term at their speed, so the first drive starts from today's braking.
  """

  def __init__(self, decels: dict[str, float]):
    super().__init__(decels, NIDEC_BRAKE_COAST_KI, NIDEC_BRAKE_COAST_MIN, NIDEC_BRAKE_COAST_MAX)


def nidec_brake_coast_error(error: float, brake_applied: float, brake_requested: bool, gas_off_time: float) -> float:
  """The part of this frame's brake-side error (commanded minus delivered accel) that teaches the coast decel.

  brake_applied is the normalized brake that was on the wire, brake_requested whether a request
  was up before the coast decel came off it, gas_off_time how long (s) the gas wire has been at
  zero.
  Nothing counts until the gas has been off for NIDEC_BRAKE_COAST_GAS_OFF_MIN: until then the
  engine is still running down and the car decelerates less than it will coasting, which reads as
  a coast decel that is too high. On route 3792d010590cb83a/000002b1 those frames were the largest
  push on the estimate above 10 m/s (24 s of them against 2 s of settled coasting), it sat at
  0.3-0.4 m/s2 where the car coasts at 0.5-0.7, and every gas overshoot then drew a 1.5 s brake
  dab (7 applications a minute at 10-21 m/s against 2-4 before the coast decel was learned).
  With brake on the wire, light braking counts in full and fades to nothing at
  NIDEC_BRAKE_COAST_LEARN_MAX_BRAKE.
  With the request swallowed whole the car is answering with its coast decel alone. Too little
  decel there counts in full: without it a coast decel learned too high could never come back
  down (no brake, no error). Too much decel does not count: there is no brake left to take off,
  so the estimate would only run to its limit.
  """
  if (not brake_requested) or gas_off_time < NIDEC_BRAKE_COAST_GAS_OFF_MIN:
    return 0.0
  if brake_applied <= 0.0:
    return min(error, 0.0)
  return error * min(max(1.0 - brake_applied / NIDEC_BRAKE_COAST_LEARN_MAX_BRAKE, 0.0), 1.0)


class DelayLine:
  """Fixed transport delay: push() returns the value pushed delay seconds earlier.

  Used to compare the measured acceleration with the command that produced it. The brake answers a
  command CP.longitudinalActuatorDelay late, so an error taken against the current command is the
  actuator's dead time, not a tracking error: MDX 3G route 3792d010590cb83a/00000250--7f4c16cef3 at
  13:03:22 ET, plan step -0.1 -> -2.2 m/s2, aEgo still 0 for the first 0.2 s, and nidec_pid
  integrated that to -0.44 m/s2 of extra brake that then took a second to unwind (aEgo -2.6
  against a -2.1 plan); -0.84 on the 13:04:30 stop of the same drive.
  """

  def __init__(self, delay: float, dt: float, initial=0.0):
    self.frames = max(0, int(round(delay / dt)))
    self.buf = deque([initial] * (self.frames + 1), maxlen=self.frames + 1)

  def push(self, value):
    self.buf.append(value)
    return self.buf[0]

  def reset(self, value):
    for _ in range(self.frames + 1):
      self.buf.append(value)

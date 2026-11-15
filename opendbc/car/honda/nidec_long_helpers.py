"""Pure Nidec longitudinal helpers (importable without openpilot)."""
from collections import deque

# Keep in sync with opendbc/car/honda/carcontroller.py
NIDEC_MIN_ACCEL_SPEED_LEAD = 0.35
# m/s of lead below which the servo is never treated as saturated (see NIDEC_DV_SAT_MIN in carcontroller)
NIDEC_DV_SAT_MIN = 2.5
# m/s below the displayed set speed where the PCM's own set-speed governor takes over (see
# nidec_set_speed_governed); the gas-channel learners have always used this margin
NIDEC_SET_SPEED_MARGIN = 2.0
# Gas staging in a recovery window (see nidec_gas_staged / nidec_gas_unwound).
# PCM_GAS level the wire is held at or above while the PCM re-applies the throttle: what the factory
# camera has on the wire at its own brake releases (MDX 3G bus 2, routes 3792d010590cb83a/00000279
# and /00000250: median 116 and 100, with no ramp from zero) and inside the 104-114 it launches with.
NIDEC_GAS_STAGE = 110
# m/s2 of commanded accel above which the wire is staged
NIDEC_GAS_STAGE_MIN_ACCEL = 0.05
# 100 Hz ticks of zero gas command after which the PCM has let the pedal go (applied pedal reaches
# zero about 0.5 s after the wire does on this platform)
NIDEC_GAS_UNWOUND_TICKS = 50
NIDEC_GAS_RECOVERY_TICKS = 200


def nidec_speed_lead_mps(sf_eff: float, accel: float, alpha_eff: float) -> float:
  """PCM_SPEED lead (m/s) from the speed-channel blend."""
  speed_lead = sf_eff * accel + alpha_eff
  if accel > 0.05:
    speed_lead = sf_eff * accel + max(alpha_eff, -sf_eff * accel + NIDEC_MIN_ACCEL_SPEED_LEAD)
  return speed_lead


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


def nidec_gas_unwound(zero_ticks: int, wire_target: int, accel: float, v_ego: float, braking: bool, gas_pressed: bool,
                      launch_active: bool) -> bool:
  """True on a tick where the gas command comes back after the PCM's throttle went to zero.

  zero_ticks counts the ticks the gas command (feedforward target and wire) has been zero up to the
  previous tick. Past NIDEC_GAS_UNWOUND_TICKS the PCM has dropped the pedal, so a returning command
  is the same restart as a brake or override release and gets the same recovery window. Of 194
  pedal restarts on four MDX 3G drives, 138 followed a coast with no brake and had no window: the
  wire climbed at the cruise rate and the learners read the PCM's re-apply time as tracking error.
  Zeros of the wire alone do not count: the feedforward parks it at zero whenever the modeled
  average is above the target, which is not a pedal release. Standstill is the launch governor's.
  """
  return zero_ticks >= NIDEC_GAS_UNWOUND_TICKS and wire_target > 0 and accel > NIDEC_GAS_STAGE_MIN_ACCEL and \
         v_ego > 0.1 and not (braking or gas_pressed or launch_active)


def nidec_gas_staged(wire: int, recovery_ticks: int, accel: float, v_ego: float, braking: bool, gas_pressed: bool,
                     launch_active: bool, gas_max: int) -> int:
  """PCM_GAS for this tick, held at NIDEC_GAS_STAGE or above while a recovery window re-applies the throttle.

  The PCM starts opening the throttle 0.8-0.9 s after the wire reaches about 100, whether the wire
  got there in one frame or ten, and it never discarded a fast rise (routes 3792d010590cb83a/00000279,
  /00000250, /00000235, /000001cc). What this side controls is when that clock starts, and the wire
  used to follow the plan up from zero, which itself ramps from zero. The factory camera does not
  ramp: it already has 100-116 on the wire at the frame its brake releases, and lets a small
  PCM_SPEED lead meter the acceleration under that ceiling.

  Only a floor: the feedforward keeps everything above it. Never with a brake on the wire or at
  standstill (concurrent gas + brake and standstill ACC faults, see carcontroller).
  """
  if recovery_ticks > 0 and accel > NIDEC_GAS_STAGE_MIN_ACCEL and v_ego > 0.1 and \
     not (braking or gas_pressed or launch_active):
    return max(wire, min(NIDEC_GAS_STAGE, gas_max))
  return wire


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

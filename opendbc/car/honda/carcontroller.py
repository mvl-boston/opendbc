import math
import threading
from queue import Empty, Queue

import numpy as np
from openpilot.common.params import Params
from opendbc.car.common.conversions import Conversions as CV

from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, rate_limit, make_tester_present_msg, structs
from opendbc.car.honda import hondacan
from opendbc.car.honda.values import CAR, CruiseButtons, CruiseSettings, HONDA_BOSCH, HONDA_BOSCH_CANFD, HONDA_BOSCH_RADARLESS, \
                                     HONDA_BOSCH_TJA_CONTROL, HONDA_NIDEC_ALT_PCM_ACCEL, CarControllerParams
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.common.pid import PIDController
from opendbc.car.honda import lane_path
from opendbc.car.honda import hud_objects

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


def compute_gb_honda_bosch(accel, speed):
  # TODO returns 0s, is unused
  return 0.0, 0.0


def compute_gb_honda_nidec(accel, speed, creep_factor):
  creep_brake = 0.0
  creep_speed = 2.3
  creep_brake_value = 0.15
  if speed < creep_speed:
    creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
  gb = float(accel) / 4.8 - creep_brake * creep_factor
  creep_impact = -creep_brake
  return np.clip(-gb, 0.0, 1.0), creep_impact


def compute_gas_brake(accel, speed, fingerprint):
  if fingerprint in HONDA_BOSCH:
    return compute_gb_honda_bosch(accel, speed)
  else:
    return compute_gb_honda_bosch(accel, speed)


# TODO not clear this does anything useful
def actuator_hysteresis(brake, braking, brake_steady, v_ego, car_fingerprint):
  # hyst params
  brake_hyst_on = 0.02    # to activate brakes exceed this value
  brake_hyst_off = 0.005  # to deactivate brakes below this value
  brake_hyst_gap = 0.01   # don't change brake command for small oscillations within this value

  # *** hysteresis logic to avoid brake blinking. go above 0.1 to trigger
  if (brake < brake_hyst_on and not braking) or brake < brake_hyst_off:
    brake = 0.
  braking = brake > 0.

  # for small brake oscillations within brake_hyst_gap, don't change the brake command
  if brake == 0.:
    brake_steady = 0.
  elif brake > brake_steady + brake_hyst_gap:
    brake_steady = brake - brake_hyst_gap
  elif brake < brake_steady - brake_hyst_gap:
    brake_steady = brake + brake_hyst_gap
  brake = brake_steady

  return brake, braking, brake_steady


def brake_pump_hysteresis(apply_brake, apply_brake_last, last_pump_ts, ts):
  pump_on = False

  # reset pump timer if:
  # - there is an increment in brake request
  # - we are applying steady state brakes and we haven't been running the pump
  #   for more than 20s (to prevent pressure bleeding)
  if apply_brake > apply_brake_last or (ts - last_pump_ts > 20. and apply_brake > 0):
    last_pump_ts = ts

  # once the pump is on, run it for at least 0.2s
  if ts - last_pump_ts < 0.2 and apply_brake > 0:
    pump_on = True

  return pump_on, last_pump_ts


def process_hud_alert(hud_alert):
  alert_fcw = False
  alert_steer_required = False

  # Make sure FCW is prioritized over steering required
  # TODO: implement separate available LDW alert
  if hud_alert == VisualAlert.fcw:
    alert_fcw = True
  elif hud_alert in (VisualAlert.steerRequired, VisualAlert.ldw):
    alert_steer_required = True

  return alert_fcw, alert_steer_required


class HondaParamWriter:
  def __init__(self):
    self._params = Params()
    self._queue = Queue()
    self._thread = threading.Thread(target=self._run, name="honda-param-writer", daemon=True)
    self._thread.start()

  def put_many(self, values):
    self._queue.put({key: float(value) for key, value in values.items()})

  def _run(self):
    while True:
      pending = self._queue.get()

      # Collapse queued snapshots so delayed writes keep only the newest value per key.
      try:
        while True:
          pending.update(self._queue.get_nowait())
      except Empty:
        pass

      for key, value in pending.items():
        self._params.put(key, value)

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)
    self.CAN = hondacan.CanBus(CP)
    self.hud_object_author = hud_objects.HudObjectAuthor()
    self.lane_path_fitter = lane_path.LanePathFitter()
    self.dash_lane = lane_path.DashLane([lane_path.OFFSET_UNAVAILABLE] * lane_path.NUM_PTS, 0.0, False, False)
    self.lkas_hud_key = None
    self.lkas_state_change_frames = 0
    self.tja_control = CP.carFingerprint in HONDA_BOSCH_TJA_CONTROL
    self.param_writer = HondaParamWriter()

    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.apply_brake_last = 0
    self.last_pump_ts = 0.
    self.stopping_counter = 0

    self.accel = 0.0
    self.speed = 0.0
    self.gas = 0.0
    self.brake = 0.0
    self.last_torque = 0.0
    self.bosch_last_gas = 0

    self.lkas_button_send_remaining = 0
    self.last_lkas_button_frame = 0
    self.radar_disable_counter = 0

    self.gasalpha = 0.0 if (Params().get("HondaGasAlphaParams") is None) else Params().get("HondaGasAlphaParams")
    self.gasfactor = 1.0 if (Params().get("HondaGasFactorParams") is None) else Params().get("HondaGasFactorParams")
    self.gasfactor_before_gasmax = self.gasfactor
    self.windfactor = 1.0 if (Params().get("HondaWindFactorParams") is None) else Params().get("HondaWindFactorParams")
    self.windfactor_before_gasmax = self.windfactor_before_brake = self.windfactor
    self.pitch = 0.0
    self.radar_mux = 0
    # stock RADAR_HUD_CANFD raises its CMBS bit only for a short burst after ACC engages (see
    # create_radar_hud_canfd); counted in 10 Hz hud ticks
    self.radar_hud_pulse = 0
    self.last_acc_enabled = False

    # Bosch extra-brake controller
    self.brake_pid = PIDController(k_p=0.0,
                                   k_i=1.0,
                                   pos_limit=0.0,
                                   neg_limit=-2.0,
                                   rate=50)
    self.brake_pid.reset()

    self.nidec_pid_factor = 0.0
    self.brake_pid_factor = 0.0

    self.nidec_pid = PIDController(k_p=([0,], [0,]),
                                   k_i=([0.01, 5., 35.], [1.2, 0.8, 0.5]),
                                   k_f=1,
                                   pos_limit=0., # self.params.NIDEC_ACCEL_MAX,
                                   neg_limit=self.params.NIDEC_ACCEL_MIN)
    self.nidec_pid.reset()

    self.nidec_brake_pid = PIDController(k_p=0.0,
                                   k_i=2.0,
                                   pos_limit=4.0,
                                   neg_limit=0,
                                   rate=50)
    self.nidec_brake_pid.reset()
    self.brake_pid_factor_non_lowspeed = 0.4 if (Params().get("HondaBrakePIDParams") is None) else Params().get("HondaBrakePIDParams")
    self.nidec_brake_pid.i = self.brake_pid_factor_non_lowspeed
    self.brake_pid_factor = 0.0

    self.prior_gas_average = 0.0
    self.average_factor = 0.5 if (Params().get("HondaFeedForwardParams") is None) else Params().get("HondaFeedForwardParams")
    self.average_factor = max(0.02, self.average_factor)  # same floor as the learner, so frame 1 is not a 1000x comparator
    self.average_factor_sens = 0.0  # d(prior_gas_average)/d(average_factor), recursive model sensitivity
    self.car_gas_per_pcm_gas = 0.3 if (Params().get("HondaCarGasScaleParams") is None) else Params().get("HondaCarGasScaleParams")
    self.creep_factor = 1.0 if (Params().get("HondaCreepFactorParams") is None) else Params().get("HondaCreepFactorParams")
    self.gas_alpha = 0.0 if (Params().get("HondaGasAlphaParams") is None) else Params().get("HondaGasAlphaParams")
    self.gas_alpha_nomaxspeed = self.gas_alpha
    self.gasfactor = 1.0 if (Params().get("HondaGasFactorParams") is None) else Params().get("HondaGasFactorParams")
    self.gasfactor_before_gasmax = self.gasfactor_nomaxspeed = self.gasfactor
    # low-speed band of the gas channel (same 10-16 m/s blend as the speed channel). The gas
    # target is normalized by NIDEC_MAX_ACCEL_V, whose shape (2.4 m/s2 at 4 m/s -> 0.6 at 20)
    # is ~2x too steep for this car: with the wire pinned at 198 the plant delivers ~0.9-1.2
    # m/s2 at 4.5-13 m/s and ~0.6 at 18-32 (route 51). One scalar gasfactor equilibrates on the
    # highway frames (where the wire is already pinned 60-100% of the time) and leaves the
    # low-speed target at about half of pinned: route 51 steady accel at 2-13 m/s ran a
    # smoothed wire of 48-97 for cmd 0.3-1.5 (pinned <=16%) with err +0.18..+0.47, while
    # 13-32 m/s was pinned 43-100% with err +0.05..+0.32 (authority-limited). Each band learns
    # in proportion to its authority over the sent target. Seeded from the persisted scalar so
    # the first drive starts at today's operating point instead of halving the low-speed target.
    self.gasfactor_low = self.gasfactor if (Params().get("HondaGasFactorLowParams") is None) else Params().get("HondaGasFactorLowParams")
    self.gasfactor_low_before_gasmax = self.gasfactor_low_nomaxspeed = self.gasfactor_low
    self.windfactor = 1.0 if (Params().get("HondaWindFactorParams") is None) else Params().get("HondaWindFactorParams")
    self.windfactor_before_gasmax = self.windfactor_before_brake = self.windfactor
    self.speedfactor = 4.0 if (Params().get("HondaSpeedFactorParams") is None) else Params().get("HondaSpeedFactorParams")
    self.speedalpha = 0.0 if (Params().get("HondaSpeedAlphaParams") is None) else Params().get("HondaSpeedAlphaParams")
    # low-speed band of the speed channel (below ~30 mph): the servo's dv gain is lower there
    # (plant fit 0.33-0.39 (m/s2)/(m/s) at 20-35 kph vs 0.39-0.46 at 35-65) and the zero-accel
    # offset is speed-dependent, so city driving needs its own equilibrium instead of sharing
    # one scalar with highway cruise (routes 18/19/1a: undershoot +0.45 below 13.4 m/s vs +0.04
    # above). Blended with the main band over 10-16 m/s; each band learns in proportion to its
    # authority over the sent lead.
    self.speedfactor_low = 4.0 if (Params().get("HondaSpeedFactorLowParams") is None) else Params().get("HondaSpeedFactorLowParams")
    self.speedalpha_low = 0.0 if (Params().get("HondaSpeedAlphaLowParams") is None) else Params().get("HondaSpeedAlphaLowParams")
    self.sat_accel = 0.9 if (Params().get("HondaSatAccelParams") is None) else Params().get("HondaSatAccelParams")
    self.sat_deficit_frames = self.sat_excess_frames = 0
    self.new_accel = 0.0

    # launch governor: owns the standstill -> moving window with stock-shaped commands (small dv
    # step + immediate gas seed + X01 power flag) instead of the general pipeline. A huge dv step
    # into the PCM's low-pass produces dead time followed by a late surge. Both parameters
    # learn once per launch EVENT from direct measurements and are bracketed from both sides
    # (slow launch grows dv_launch, motion lurch shrinks it), so neither can run away; the range
    # clips are loose backstops that should not bind.
    self.dv_launch = 2.8 if (Params().get("HondaLaunchDvParams") is None) else Params().get("HondaLaunchDvParams")  # m/s; stock launches at 9.99 kph
    self.gas_launch = 110.0 if (Params().get("HondaLaunchGasParams") is None) else Params().get("HondaLaunchGasParams")  # PCM_GAS units; stock 104-114
    # breakaway lead, used only until first motion: the EV creep response scales with dv, and
    # the stock-sized lead is not enough to break away reliably engine-off.
    # Hands over to dv_launch at first motion so nothing accumulates in the servo low-pass.
    self.dv_break = 6.0 if (Params().get("HondaLaunchDvBreakParams") is None) else Params().get("HondaLaunchDvBreakParams")  # m/s
    self.launch_active = False
    # gas-channel recovery window: opened at driver-gas override release, at launch window
    # exit, at openpilot brake release and at engage while rolling. In every case the wire gas
    # is coming out of a known dead state (the concurrent gas+brake guard zeroes it for the
    # whole brake period) and has to re-wind the PCM's internal pedal tracker; the +2/tick rise
    # clip alone spends ~1s just climbing 0->198 (routes 33/34: post-override err +0.9..+1.7 for
    # 2-2.5s; route 48: 28 brake->gas transitions, wire reached 100 at +0.62s median, pedal
    # onset +1.44s, aEgo>0.2 only at +2.3s, err +0.56 over the 3s; pedal onset tracked
    # wire>=100 (corr 0.71) plus the same ~0.6s PCM lag seen for cruise rises, so the climb
    # is the only part of that chain on our side — whether the PCM also has a fixed re-apply
    # dead time after its brake period is unresolved: the one fast-wire event still saw the
    # pedal at +1.44s, so the wire shaping here is bounded-benefit, while the learner gating
    # the window provides is the certain part).
    self.gas_recovery_ticks = 0
    self.gas_pressed_prev = False
    self.long_active_prev = False
    self.launch_ticks = 0
    self.launch_release_tick = -1
    self.launch_move_tick = -1
    self.launch_lurch_sum = 0.0
    self.launch_lurch_n = 0
    self.launch_err_sum = 0.0
    self.launch_err_n = 0
    self.launch_ceiling_ticks = 0

    self.latFactors = {
      "05": 1.0 if (Params().get("HondaLatAccelFactor05Params") is None) else Params().get("HondaLatAccelFactor05Params"),
      "10": 1.0 if (Params().get("HondaLatAccelFactor10Params") is None) else Params().get("HondaLatAccelFactor10Params"),
      "15": 1.0 if (Params().get("HondaLatAccelFactor15Params") is None) else Params().get("HondaLatAccelFactor15Params"),
      "20": 1.0 if (Params().get("HondaLatAccelFactor20Params") is None) else Params().get("HondaLatAccelFactor20Params"),
      "25": 1.0 if (Params().get("HondaLatAccelFactor25Params") is None) else Params().get("HondaLatAccelFactor25Params"),
      "30": 1.0 if (Params().get("HondaLatAccelFactor30Params") is None) else Params().get("HondaLatAccelFactor30Params"),
      "35": 1.0 if (Params().get("HondaLatAccelFactor35Params") is None) else Params().get("HondaLatAccelFactor35Params"),
      "40": 1.0 if (Params().get("HondaLatAccelFactor40Params") is None) else Params().get("HondaLatAccelFactor40Params"),
      "45": 1.0 if (Params().get("HondaLatAccelFactor45Params") is None) else Params().get("HondaLatAccelFactor45Params"),
      "50": 1.0 if (Params().get("HondaLatAccelFactor50Params") is None) else Params().get("HondaLatAccelFactor50Params"),
      "55": 1.0 if (Params().get("HondaLatAccelFactor55Params") is None) else Params().get("HondaLatAccelFactor55Params"),
      "60": 1.0 if (Params().get("HondaLatAccelFactor60Params") is None) else Params().get("HondaLatAccelFactor60Params")
    }

  def update(self, CC, CS, now_nanos):
    gas_pedal_force = 0.0
    actuators = CC.actuators
    hud_control = CC.hudControl
    hud_v_cruise = hud_control.setSpeed / CS.v_cruise_factor if hud_control.speedVisible else 255
    pcm_cancel_cmd = CC.cruiseControl.cancel

    if len(CC.orientationNED) == 3:
      self.pitch = CC.orientationNED[1]
    hill_brake = math.sin(self.pitch) * ACCELERATION_DUE_TO_GRAVITY

    # wind brake from air resistance decel at high speed
    wind_brake = np.interp(CS.out.vEgo, [0.0, 2.3, 35.0], [0.001, 0.002, 0.15]) * self.windfactor # not in m/s2 units

    prior_windfactor = self.windfactor
    if CC.longActive:
      if self.CP.carFingerprint in HONDA_BOSCH:
        accel = actuators.accel
        adjust_accel = accel + hill_brake
        brake = 0.0
        gas, brake = compute_gas_brake(adjust_accel, CS.out.vEgo, self.CP.carFingerprint)
      else:
        if (actuators.longControlState in (LongCtrlState.pid, LongCtrlState.stopping)) and \
           (CS.out.vEgo > 1e-5 or actuators.accel > 1e-5) \
           and (not CS.out.stockAeb) and (not CS.out.gasPressed):
          old_i = self.nidec_pid.i
          self.nidec_pid_factor = self.nidec_pid.update(error = actuators.accel - CS.out.aEgo, speed = CS.out.vEgo)
          if actuators.accel >= 1e-5 and CS.out.vEgo < 1.0: # don't drop PID at launch lurch
             self.nidec_pid_factor = self.nidec_pid.i = max(old_i, self.nidec_pid.i)
          self.accel = actuators.accel + self.nidec_pid_factor
          adjust_accel = self.accel + hill_brake

          # copy wind tuning from Bosch code
          gas_error = self.accel - CS.out.aEgo
          wind_learn_speed = 1000
          wind_adjust = 1 + wind_brake / wind_learn_speed
          self.windfactor = np.clip(self.windfactor * (wind_adjust if (gas_error > 0) else 1.0/wind_adjust), 0.1, 3.0)
          gas_pedal_force = self.accel
          if gas_pedal_force <= 0.0: # don't reduce windfactor while braking, allow increases
            self.windfactor = max(self.windfactor, self.windfactor_before_brake)
          else:
            self.windfactor_before_brake = self.windfactor
          if CS.out.vEgo < CS.out.cruiseState.speed - 2.:
            # drop to max values when not near speed limit
            self.gasfactor = self.gasfactor_nomaxspeed
            self.gasfactor_low = self.gasfactor_low_nomaxspeed
            self.gas_alpha = self.gas_alpha_nomaxspeed
          if (gas_pedal_force >= self.params.BOSCH_ACCEL_MAX):
            # don't increase gasfactor nor windfactor at accel max, allow decreases
            self.gasfactor = min(self.gasfactor, self.gasfactor_before_gasmax)
            self.gasfactor_low = min(self.gasfactor_low, self.gasfactor_low_before_gasmax)
            self.windfactor = min(self.windfactor, self.windfactor_before_gasmax)
          else:
            self.gasfactor_before_gasmax = self.gasfactor
            self.gasfactor_low_before_gasmax = self.gasfactor_low
            self.windfactor_before_gasmax = self.windfactor

        else:
          self.accel = actuators.accel + self.nidec_pid_factor
          adjust_accel = self.accel + hill_brake

        if (CS.out.brakePressed or CS.out.gasPressed or CS.out.vEgo < 1e-5) and (self.nidec_pid.i <= 0.01):
          self.nidec_pid.i += 0.01 # clear out nidec pid integral while acc not controlling car

        brake, creep_impact = compute_gb_honda_nidec(adjust_accel, CS.out.vEgo, self.creep_factor)
        gas_error = self.accel - CS.out.aEgo
        if (actuators.longControlState == LongCtrlState.pid) and (not CS.out.stockAeb) and (not CS.out.gasPressed) \
               and (1e-5 <= CS.out.vEgo <= CS.out.cruiseState.speed - 2.):
          self.creep_factor = 0.0
          # self.creep_factor = np.clip(self.creep_factor + 0.001 * creep_impact * gas_error, 0.0, 3.0)
    else:
      self.accel = 0.0
      adjust_accel = self.accel
      brake = 0.0
      accel = 0.0
      gas, brake = 0.0, 0.0
    if CC.longActive and self.CP.carFingerprint not in HONDA_BOSCH:
      accel = self.accel
    if CS.out.gasPressed or not CC.longActive:
      if self.CP.carFingerprint not in HONDA_BOSCH:
        self.nidec_pid.reset()

    # *** rate limit steer ***
    limited_torque = rate_limit(actuators.torque, self.last_torque, -self.params.STEER_DELTA_DOWN * DT_CTRL,
                                self.params.STEER_DELTA_UP * DT_CTRL)
    if (self.CP.carFingerprint == CAR.ACURA_MDX_3G) and \
        (self.apply_brake_last > 0 or self.new_accel < 1e-5): # lower steer limits while braking
      brake_limit = float(233.0 / self.params.STEER_MAX)
      limited_torque = float(np.clip(limited_torque, -brake_limit, brake_limit))
    self.last_torque = limited_torque

    # *** apply brake hysteresis ***
    pre_limit_brake, self.braking, self.brake_steady = actuator_hysteresis(brake, self.braking, self.brake_steady,
                                                                           CS.out.vEgo, self.CP.carFingerprint)

    # *** rate limit after the enable check ***
    self.brake_last = rate_limit(pre_limit_brake, self.brake_last, -2., 3 * DT_CTRL)

    # vehicle hud display, wait for one update from 10Hz 0x304 msg
    alert_fcw, alert_steer_required = process_hud_alert(hud_control.visualAlert)

    # **** process the car messages ****

    # steer torque is converted back to CAN reference (positive when steering right)
    apply_torque = int(np.interp(-limited_torque * self.params.STEER_MAX,
                                 self.params.STEER_LOOKUP_BP, self.params.STEER_LOOKUP_V))

    speed_val = np.clip(round(CS.out.vEgo * CV.MS_TO_MPH / 5.0) * 5, 5, 60)
    currentLatSpeed = f"{speed_val:02d}"
    if currentLatSpeed in self.latFactors and not CS.out.steeringPressed and CS.steer_control_active:
      if abs(limited_torque) > 0.9 and self.latFactors[currentLatSpeed] > abs(CS.out.steeringAngleDeg):
        self.latFactors[currentLatSpeed] /= 1.001
      if abs(limited_torque) < 0.9 and self.latFactors[currentLatSpeed] < abs(CS.out.steeringAngleDeg):
        self.latFactors[currentLatSpeed] *= 1.001

    # Send CAN commands
    can_sends = []

    if self.CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS) and self.CP.openpilotLongitudinalControl:
      if self.CP.carFingerprint in HONDA_BOSCH_CANFD and CS.stock_acc_alive:
        # CAN FD: the radar is still transmitting. It is silenced from here rather than from
        # CarInterface.init(), and only once the comma relay is confirmed open: init() ran while the
        # panda was still in the ELM327 safety mode, so the replacement ACC_CONTROL stream was blocked
        # until the safety-mode switch landed, and whenever the switch took longer than ~110 ms after
        # the radar went silent the brake module latched CRUISE_FAULT (accFaulted) for the whole drive.
        # With the relay already open the stock radar keeps feeding the brake module (and, via panda
        # forwarding, the camera) right up to the switchover, and the replacement stream starts within
        # a few frames of radar silence (see CS.stock_acc_alive), well inside the fault threshold.
        if CS.canfd_relay_open:
          if self.radar_disable_counter % 50 == 0:
            # UDS extended diagnostic session, required before CommunicationControl
            can_sends.append((0x18DAB0F1, b'\x02\x10\x03\x00\x00\x00\x00\x00', self.CAN.pt))
          elif self.radar_disable_counter % 50 == 5:
            # UDS CommunicationControl disableRxAndTx (0x80 suppresses the response); the same request
            # CarInterface.init() used to send, retried every 0.5 s until the radar goes silent
            can_sends.append((0x18DAB0F1, b'\x03\x28\x83\x03\x00\x00\x00\x00', self.CAN.pt))
          self.radar_disable_counter += 1
      elif self.frame % 10 == 0:
        # tester present - w/ no response (keeps radar disabled)
        bus = 0 if self.CP.carFingerprint in HONDA_BOSCH_CANFD else 1
        can_sends.append(make_tester_present_msg(0x18DAB0F1, bus, suppress_response=True))

    # simulate canfd radar to prevent faults
    # These radar look-alikes are consumed by both the camera ECU (behind the relay, on the camera
    # bus) and the powertrain (radar bus). openpilot's own transmissions are not forwarded across the
    # open relay, so they must be sent explicitly on both buses. Each message is packed exactly once
    # so the packer's counter/checksum only advance once per cycle, then the identical bytes are
    # mirrored onto both buses (re-packing would double-increment the counter and desync the buses).
    # While the stock radar is still transmitting (drive start, before the deferred disable above has
    # silenced it), it authors all of these itself: sending look-alikes too would double them up.
    if (self.CP.carFingerprint in HONDA_BOSCH_CANFD) and self.CP.openpilotLongitudinalControl and not CS.stock_acc_alive:
      if CC.enabled and not self.last_acc_enabled:
        self.radar_hud_pulse = 30  # ~3 s at 10 Hz, matching the stock 2-6 s engage burst
      self.last_acc_enabled = CC.enabled
      radar_msgs = []
      if CS.hud_tick:
        radar_msgs.append(hondacan.create_radar_hud_canfd(self.packer, self.CAN.pt, CC.enabled, self.radar_hud_pulse > 0))
        if self.radar_hud_pulse > 0:
          self.radar_hud_pulse -= 1
      if CS.supp_tick:
        radar_msgs.append(hondacan.create_canfd_supplemental(self.packer, self.CAN.pt))
      if CS.radar_50hz_tick:
        # Cycle the radar MUX through the same banks the stock radar uses: 1-10, 17-26, 33-42, 49-58.
        # This counter also drives the LANE_PATH/HUD_OBJECTS mux below: it advances exactly one step
        # per transmitted frame, so the sweep stays contiguous even when a tick is missed (deriving
        # the mux from the frame counter skipped a mux on every missed tick, leaving holes in the
        # sweep the stock radar never produces).
        # These must be elif (not sequential if): a bare `if` that sets the bank start would fall
        # through to the `else` increment, skipping the bank-start values (17, 33, 49).
        if self.radar_mux >= 58:
          self.radar_mux = 1
        elif self.radar_mux == 10:
          self.radar_mux = 17
        elif self.radar_mux == 26:
          self.radar_mux = 33
        elif self.radar_mux == 42:
          self.radar_mux = 49
        else:
          self.radar_mux += 1
        # radar_msgs.extend(hondacan.create_canfd_50hz_radar_messages(self.packer, self.CAN.pt, self.radar_mux))
      if CS.radar_5hz_tick:
        # RADAR_LEAD's LANE_PATH_LENGTH must track the valid-point count of the LANE_PATH sweep we are
        # authoring; the stock radar keeps the two in lockstep and the dash won't draw lanes otherwise.
        # LEFT_LANE/RIGHT_LANE carry the per-side line-detected status (3/0) the same way the stock
        # radar mirrors the camera's LANE_LINES bits; the dash draws no lane lines while both are 0.
        radar_msgs.extend(hondacan.create_canfd_5hz_radar_messages(self.packer, self.CAN.pt, CS.radar_ref_counter,
                                                                   lane_path.canfd_lane_length(self.dash_lane),
                                                                   lane_path.LANE_LINE_ON if self.dash_lane.left_line else 0,
                                                                   lane_path.LANE_LINE_ON if self.dash_lane.right_line else 0))

      # mirror each packed frame onto both the powertrain bus and the camera bus
      for addr, dat, _ in radar_msgs:
        can_sends.append((addr, dat, self.CAN.pt))
        can_sends.append((addr, dat, self.CAN.camera))

    # Send steering command.
    can_sends.append(hondacan.create_steering_control(self.packer, self.CAN, apply_torque, CC.latActive, self.tja_control))

    wind_brake_ms2 = np.interp(CS.out.vEgo, [0.0, 13.4, 22.4, 31.3, 40.2], [0.000, 0.049, 0.136, 0.267, 0.441]) # in m/s2 units

    # launch governor state machine (Nidec): window from engaged standstill with a positive plan
    # until moving (>1.5 m/s) or timeout. Measurements are collected per tick, learner updates
    # happen once at a clean window exit (aborts from driver input or a negative plan learn nothing).
    if self.CP.carFingerprint not in HONDA_BOSCH:
      if not self.launch_active:
        if CC.longActive and (not CS.out.gasPressed) and (not CS.out.brakePressed) and \
             (CS.out.vEgo < 0.1) and (actuators.accel > 0.05):
          self.launch_active = True
          self.launch_ticks = 0
          self.launch_release_tick = -1
          self.launch_move_tick = -1
          self.launch_lurch_sum = 0.0
          self.launch_lurch_n = 0
          self.launch_err_sum = 0.0
          self.launch_err_n = 0
          self.launch_ceiling_ticks = 0
      else:
        self.launch_ticks += 1
        if (self.launch_release_tick < 0) and (self.apply_brake_last == 0):
          self.launch_release_tick = self.launch_ticks
        if (self.launch_move_tick < 0) and (CS.out.vEgo > 0.1):
          self.launch_move_tick = self.launch_ticks
        if (self.launch_move_tick >= 0) and (10 <= self.launch_ticks - self.launch_move_tick <= 50):
          # lurch: SUSTAINED accel beyond the plan just after breakaway (0.1-0.5s window). The
          # instantaneous aEgo at first motion spikes to ~3 m/s2 for a frame or two from wheel
          # speed breakaway quantization (routes 18/19/1a: every launch registered a "lurch"
          # that way, shrinking dv_launch 2.8 -> 2.0 while every launch was actually slow), so
          # only a windowed mean is a real lurch measurement.
          self.launch_lurch_sum += CS.out.aEgo - actuators.accel
          self.launch_lurch_n += 1
        if self.launch_move_tick >= 0:
          self.launch_err_sum += actuators.accel - CS.out.aEgo
          self.launch_err_n += 1
          # ceiling binding: engine running and applied pedal near the commanded seed, i.e. the
          # gas channel was actually delivering the ceiling, so more authority would have helped.
          # EV launches never satisfy this (pedal decoupled), so they produce no gas_launch growth.
          if (CS.engine_rpm > 500) and (CS.car_gas >= 0.8 * self.car_gas_per_pcm_gas * self.gas_launch):
            self.launch_ceiling_ticks += 1
        launch_aborted = (not CC.longActive) or CS.out.gasPressed or CS.out.brakePressed or (actuators.accel < -0.05)
        launch_done = (CS.out.vEgo > 1.5) or (self.launch_ticks >= 300)
        if launch_aborted or launch_done:
          if launch_done and not launch_aborted:
            # dv_break owns breakaway dead time vs lurch, one sign-correct update per launch:
            # motion later than 0.6s after brake release (or never, with the brake released >1s)
            # means more breakaway lead; sustained overshoot just after breakaway means less.
            # Bracketed from both sides.
            never_moved = (self.launch_move_tick < 0) and (self.launch_release_tick >= 0) and \
                          (self.launch_ticks - self.launch_release_tick > 100)
            launch_slow = never_moved or ((self.launch_move_tick >= 0) and (self.launch_release_tick >= 0) and
                                          (self.launch_move_tick - self.launch_release_tick > 60))
            if launch_slow:
              self.dv_break *= 1.05
            launch_lurch = (self.launch_lurch_sum / self.launch_lurch_n) if self.launch_lurch_n > 0 else 0.0
            if launch_lurch > 0.5:
              self.dv_break *= (1 - 0.05 * min(launch_lurch, 2.0))
            self.dv_break = float(np.clip(self.dv_break, 2.0, 12.0))
            # post-motion window tracking splits by channel ownership: undershoot with the pedal
            # riding the commanded ceiling engine-on means more gas would have helped; undershoot
            # WITHOUT ceiling evidence (EV launches, where the pedal is decoupled from PCM_GAS)
            # belongs to the speed lead. Overshoot shrinks both slowly.
            if self.launch_err_n > 50:
              launch_err = self.launch_err_sum / self.launch_err_n
              if launch_err > 0.15:
                if self.launch_ceiling_ticks > 0.5 * self.launch_err_n:
                  self.gas_launch *= 1.03
                else:
                  self.dv_launch *= 1.03
              elif launch_err < -0.15:
                self.gas_launch *= 0.99
                self.dv_launch *= 0.99
              self.gas_launch = float(np.clip(self.gas_launch, 40.0, self.params.NIDEC_GAS_MAX))
              self.dv_launch = float(np.clip(self.dv_launch, 1.0, 8.0))
          self.launch_active = False
          if launch_done and not launch_aborted:
            # hand the general pipeline a recovery window: the seed is usually below the
            # feedforward target at exit, and the +2/tick clip alone spends ~0.5s closing
            # that gap (route 33 launch t=549: gas 91 -> 183 over 0.5s, err +0.76 for 3s)
            self.gas_recovery_ticks = 200

      # driver-gas override release: the wire gas restarts from the mirror/zero while the plan
      # ramps positive immediately; open the recovery window on the falling edge.
      self.gas_recovery_ticks = max(self.gas_recovery_ticks - 1, 0)
      if CC.enabled and self.gas_pressed_prev and (not CS.out.gasPressed):
        self.gas_recovery_ticks = 200
      self.gas_pressed_prev = CS.out.gasPressed
      # engage while rolling: the wire was 0 (not longActive) and the plan is typically already
      # positive, so this is the same restart-from-dead-state as an override release (route 48:
      # 8 rolling engages without a prior gas press, err +0.35..+0.79 over 3s, wire>=100 at
      # 0.5-2.8s). Standstill engages are owned by the launch governor.
      if CC.longActive and (not self.long_active_prev) and (CS.out.vEgo > 0.1) and (not self.launch_active):
        self.gas_recovery_ticks = 200
      self.long_active_prev = CC.longActive

    if self.CP.carFingerprint not in HONDA_BOSCH:
      # all of this is only relevant for HONDA NIDEC
      max_accel = np.interp(CS.out.vEgo, self.params.NIDEC_MAX_ACCEL_BP, self.params.NIDEC_MAX_ACCEL_V)
      # two-band speed channel blend: pure low band below 10 m/s, pure high band above 16 m/s
      low_w = float(np.interp(CS.out.vEgo, [10.0, 16.0], [1.0, 0.0]))
      sf_eff = low_w * self.speedfactor_low + (1.0 - low_w) * self.speedfactor
      alpha_eff = low_w * self.speedalpha_low + (1.0 - low_w) * self.speedalpha
      # TODO this 1.44 is just to maintain previous behavior
      if not CC.longActive:
        if CC.enabled and CS.out.gasPressed and CS.car_gas_available:
          # driver-gas override: mirror the applied pedal onto the wire instead of zeroing it.
          # Stock keeps commanding through overrides (PCM_SPEED = set speed, PCM_GAS nonzero);
          # zeroing instead told the PCM "no torque" for the whole override, so its internal pedal
          # tracker unwound to zero and the release started from scratch (routes 33/34: pedal
          # stayed 0 until ~2s after release; at t=288.5 the commanded zero-torque second even
          # dropped the engine into idle-stop while moving, adding restart lag). The mirror also
          # keeps prior_gas_average — the model of the PCM's smoothed command — tracking the true
          # operating point through the override, and seeds the release rise clip from it.
          # moved to "not CC.longActive" block because powertrain sets ACC_STATUS = 0 when gasPressed
          # set pcm_speed to current speed + 9 to mirror stock
          pcm_speed = float(np.clip(CS.out.vEgo, 0.0, 100.0))
          pcm_accel = int(np.clip(CS.car_gas / max(self.car_gas_per_pcm_gas, 1e-3), 0.0, self.params.NIDEC_GAS_MAX))
        else:
          pcm_speed = 0.0
          pcm_accel = int(0.0)
      else:
        if self.launch_active:
          # stock-shaped launch lead (stock uses 9.99 kph): the general sf*accel+alpha lead is both
          # poisoned-prone and a step input the servo low-passes into dead time + late surge.
          # Until first motion the larger breakaway lead is used: EV creep response scales with dv,
          # and the post-motion lead alone was not enough to break away engine-off.
          speed_lead = self.dv_launch if CS.out.vEgo > 0.1 else self.dv_break
        else:
          speed_lead = float(sf_eff * self.accel + alpha_eff)
        pcm_speed = float(np.clip(CS.out.vEgo + speed_lead, 0.0, 100.0))
        gas_accel = adjust_accel + wind_brake_ms2 * self.windfactor
        gf_eff = low_w * self.gasfactor_low + (1.0 - low_w) * self.gasfactor
        pcm_accel = int(np.clip((self.gas_alpha + gas_accel * gf_eff / 1.44) / max_accel, 0.0, 1.0) * self.params.NIDEC_GAS_MAX)
      max_speedcontrol = (pcm_speed > 99.999)
      prior_speedfactor = self.speedfactor
      prior_speedalpha = self.speedalpha
      prior_speedfactor_low = self.speedfactor_low
      prior_speedalpha_low = self.speedalpha_low

      # feedforward for Nidec decaying-average gas pedal
      # inside a recovery window the rise clip opens to the stock launch envelope (+70/frame vs
      # stock's observed +74/frame): the +20/frame cruise clip is sized for smoothness around an
      # operating point, but coming out of an override/launch/brake there is no operating point yet —
      # holding it there spends a full second climbing 0->198 while PCM_GAS=0 gates off all power
      # (routes 33/34: aEgo fell to -0.3 against cmd +1.25 during that second, and the PCM's
      # pedal tracker + idle-stop restart stacked another ~1.5s on top).
      # The wide clip only applies while the wire is still below the feedforward target: the
      # window's job is to close the gap from the dead state to the operating point. Beyond the
      # target the 1/average_factor lead (~30x at the learned ~0.035) already overshoots, and
      # letting that run at +70/frame just triples the sawtooth amplitude the PCM has to smooth
      # (bench: 0->140 in 0.2s against a target of 50) for no gain in reaching the target.
      prior_accel = int(self.new_accel)
      max_increase = 7 if (self.gas_recovery_ticks > 0 and prior_accel < pcm_accel) else 2  # per 100Hz tick, x10 per sent frame
      # When GAS_PEDAL_2 is absent the direct-measurement learner cannot run; use a fixed factor so
      # feedforward stays stable on platforms that do not report applied pedal position.
      effective_average_factor = self.average_factor if CS.car_gas_available else 0.5
      self.new_accel = int((pcm_accel - self.prior_gas_average * (1 - effective_average_factor)) / effective_average_factor)
      self.new_accel = int(np.clip(self.new_accel, 0, min(prior_accel + max_increase, self.params.NIDEC_GAS_MAX)))
      if self.launch_active:
        # gas seed: send the learned launch gas immediately (stock jumps to 104-114 in one frame,
        # within its observed +114/frame ramp envelope; ramping from 0 at +2/tick wastes ~0.5s of the
        # launch just climbing, and PCM_GAS=0 gates off all power on this platform). While the brake
        # is still applied the concurrent gas+brake protection below keeps the wire at 0; on window
        # exit the rise clip continues from the seed via prior_accel, so there is no discontinuity.
        launch_seed = self.gas_launch
        if (CS.engine_rpm < 500) and (CS.out.vEgo <= 0.1):
          # EV breakaway: midrange gas has no pedal coupling engine-off (route 1a t=3002: gas 108
          # held 2.6s, zero motion until the driver's pedal cranked the engine). The stock camera's
          # own EV plateau is saturation (198/200 observed while accelerating in EV), so command
          # the full authority band until first motion, then hand back to the learned seed.
          launch_seed = self.params.NIDEC_GAS_MAX
        self.new_accel = int(min(launch_seed, self.params.NIDEC_GAS_MAX))
      # recursive sensitivity of the model prediction to average_factor, advanced with the model itself;
      # used by the average_factor learner below (must be computed before prior_gas_average is updated)
      self.average_factor_sens = (self.new_accel - self.prior_gas_average) + (1 - effective_average_factor) * self.average_factor_sens
      self.prior_gas_average = self.prior_gas_average * (1 - effective_average_factor) + (self.new_accel * effective_average_factor)

      if (0 < self.new_accel < self.params.NIDEC_GAS_MAX) and (not CS.out.gasPressed) and \
           (self.apply_brake_last == 0) and (not self.launch_active) and (self.gas_recovery_ticks == 0):
        gasfactor_error = (self.accel - CS.out.aEgo)
        self.gas_alpha = np.clip(self.gas_alpha + 0.0001 * gasfactor_error / 4.8, -3.0, 3.0)
        # the 0<wire<198 gate above is the gas channel's own observability condition (a pinned
        # wire cannot show what more gasfactor would do); the clip is a runaway backstop only.
        # 5.0 rather than the Bosch path's 3.0 because the Nidec target divides by
        # NIDEC_MAX_ACCEL_V (2.0-2.4 at 4-10 m/s): pinning the wire at cmd 0.85 there needs
        # gf ~3.5, which the low band is expected to learn.
        gf_growth = 0.0001 * gasfactor_error * gas_accel
        self.gasfactor_low = float(np.clip(self.gasfactor_low * (1 + low_w * gf_growth), 0.1, 5.0))
        self.gasfactor = float(np.clip(self.gasfactor * (1 + (1.0 - low_w) * gf_growth), 0.1, 5.0))
      if (not CS.out.gasPressed) and (self.apply_brake_last == 0): # adjust speedfactor and average_factor
        speedfactor_error = (self.accel - CS.out.aEgo)
        dv_sent = sf_eff * self.accel + alpha_eff
        dv_sat = max(0.1, sf_eff * self.sat_accel + alpha_eff)

        # average_factor learner: direct measurement (system ID), not tracking-error integration.
        # average_factor models the PCM's one-pole smoothing of our PCM_GAS commands, and
        # prior_gas_average is that model's prediction of the PCM's response. The actual response
        # is observable as the applied pedal (CAR_GAS ~= car_gas_per_pcm_gas * PCM_GAS), so move average_factor
        # along the model-fit gradient: prediction error times the recursive sensitivity computed
        # alongside the model above. The update sign flips around the PCM's true smoothing
        # constant, making the learner self-bounding: tracking error can't poison it, and at a
        # steady command rail the sensitivity decays away, so clipped/saturated frames teach it
        # nothing instead of teaching it the wrong direction. The per-tick step cap bounds noise
        # spikes; the range clip only protects the 1/average_factor feedforward division — the
        # equilibrium is interior (route 250 replay settles ~0.06-0.10, consistent with the PCM's
        # ~100ms pedal-apply lag), so it never binds.
        # gated on engine running (rpm>500): in EV mode the pedal is decoupled from PCM_GAS
        # (route 015: steady ratio 0.19 EV vs 0.47 engine-on, xcorr ~0), so EV frames would teach
        # both the scale EMA and the average_factor fit from an unrelated signal. Launch windows
        # are excluded too: the pedal applies seconds late there, which is not the smoothing
        # constant this model represents.
        if CC.longActive and (CS.out.vEgo > 1e-5) and CS.car_gas_available and \
             (CS.engine_rpm > 500) and (not self.launch_active) and (self.gas_recovery_ticks == 0):
          # car_gas_per_pcm_gas learner: direct ratio CAR_GAS / sent PCM_GAS (wire units).
          # Must use the wire command, NOT prior_gas_average: average_factor already nudges
          # prior_gas_average toward CAR_GAS/scale, so CAR_GAS/prior == scale at that joint
          # equilibrium and the scale learner never moves off its boot default (route 3b:
          # HondaCarGasScaleParams stuck at 0.3 for a full drive). Steady-wire gate skips
          # transients where the pedal lags the command.
          wire_gas = float(self.new_accel)
          if (wire_gas > 20.0) and (CS.car_gas > 5.0) and (abs(wire_gas - prior_accel) <= 1.0):
            scale_sample = CS.car_gas / wire_gas
            self.car_gas_per_pcm_gas += 0.0005 * (scale_sample - self.car_gas_per_pcm_gas)

          self.car_gas_per_pcm_gas = max(0.00001, self.car_gas_per_pcm_gas)
          gas_measured = CS.car_gas / self.car_gas_per_pcm_gas
          averagefactor_error = (gas_measured - self.prior_gas_average) / self.params.NIDEC_GAS_MAX
          averagefactor_step = 0.005 * averagefactor_error * self.average_factor_sens / self.params.NIDEC_GAS_MAX
          # This learner had been frozen since the rpm gate was added (CS.engine_rpm read 0 on the
          # MDX, see carstate), so the per-tick cap was never exercised on this car. Replaying it
          # on route 48 wire/pedal with the gate open: at +-0.001/tick it swings 0.001 <-> 0.10 on
          # a minute timescale from any start (a full-range move every second on a pedal signal
          # with ~0.01 wire correlation) and sits on the floor 157s of a 22min engaged drive. At
          # 0.001 the 1/average_factor lead is 1000x and prior_gas_average has a 10s memory, i.e.
          # the rail-to-rail bang-bang of failure mode #10. A 10x slower cap needs >=10s of
          # consistent evidence for a full-range move, and the 0.02 floor (50x lead, 0.5s
          # memory) keeps the feedforward in the sawtooth-tracking regime it has been driving in
          # at 0.035; replay band with both: 0.020-0.057 over the route.
          self.average_factor = float(np.clip(self.average_factor + np.clip(averagefactor_step, -0.0001, 0.0001),
                                              0.02, 1.0))

        # ceiling learner: identifies max accel capability, learn situation exist for a second before adjusting
        if (CS.out.aEgo > self.sat_accel) and (not CS.out.gasPressed) and (CC.longActive):
          self.sat_excess_frames += 1
        else:
          self.sat_excess_frames = 0
        # deficit samples are only ceiling evidence when the gas channel is in a settled state:
        # inside launch/recovery windows the pedal is provably lagging the wire (routes 33/34:
        # aEgo < 0 against cmd +1.25 while the wire ramped and the pedal applied ~2s late), so
        # counting those ticks drags sat_accel down for a gas-transient it does not own
        # (sat_accel fell 1.01 -> 0.86 between routes 33 and 34 while cruise tracking was clean)
        if (CS.out.aEgo < self.sat_accel <= self.accel) and (not CS.out.gasPressed) and (CC.longActive) and \
             (not self.launch_active) and (self.gas_recovery_ticks == 0):
          self.sat_deficit_frames += 1
        else:
          self.sat_deficit_frames = 0
        if (self.sat_excess_frames > 100) or (self.sat_deficit_frames > 100):
          self.sat_accel = float(np.clip(self.sat_accel + 0.002 * (CS.out.aEgo - self.sat_accel), 0.1, self.params.NIDEC_ACCEL_MAX - 0.1))

        # recovery windows are excluded like launch windows: the undershoot there is gas-channel
        # dead time, not speed-servo response, and it was double-billed — growing/bleeding
        # sf and alpha (sf_low fell 1.27 -> 0.60 across routes 33/34, mostly in these windows)
        if CC.longActive and (CS.out.vEgo > 1e-5) and (not self.launch_active) and (self.gas_recovery_ticks == 0):
          if (speedfactor_error > 0) and (dv_sent > dv_sat):
            # beyond the knee surplus dv provably does nothing, so undershoot there is not
            # growth fuel (that loop is what rode speedfactor to ~511): bleed toward the knee
            # instead (min-norm: prefer the smallest lead with the same output), rate-limited
            # to 0.5%/tick. The ratchet below only prevents growth; this is the convergence
            # force that actually deflates a poisoned state.
            sf_growth = -min(0.0005 * (dv_sent - dv_sat), 0.005)
          else:
            sf_growth = 0.001 * speedfactor_error * self.accel
          # each band learns in proportion to its authority over the sent lead
          self.speedfactor_low = float(np.clip(self.speedfactor_low * (1 + low_w * sf_growth), 0.01, 99.0))
          self.speedfactor = float(np.clip(self.speedfactor * (1 + (1.0 - low_w) * sf_growth), 0.01, 99.0))
          self.speedalpha_low = min(dv_sat, self.speedalpha_low + low_w * 0.001 * speedfactor_error)
          self.speedalpha = min(dv_sat, self.speedalpha + (1.0 - low_w) * 0.001 * speedfactor_error)
        if max_speedcontrol or (dv_sent > dv_sat): # only allow learning reductions
          # speed-channel saturation is a speed-channel condition. gasfactor/gas_alpha used to
          # be ratcheted here too, which made "pcm_speed at its clip" (68% of low-speed accel
          # frames on route 51, with speedfactor_low pinned at its clamp) forbid the gas target
          # from ever growing on exactly the frames that undershoot most: the ratchet was
          # active on 10% of the gas-learner's gated ticks but those ran err +0.45 vs +0.045
          # on the rest, and it discarded +0.14 of +0.18 total ln-growth over the drive. The
          # gas learners keep their own observability gate (0 < wire < 198) above; windfactor
          # has no saturation gate of its own, so it stays.
          self.speedfactor = min(prior_speedfactor, self.speedfactor)
          self.speedfactor_low = min(prior_speedfactor_low, self.speedfactor_low)
          self.speedalpha = min(prior_speedalpha, self.speedalpha)
          self.speedalpha_low = min(prior_speedalpha_low, self.speedalpha_low)
          self.windfactor = min(prior_windfactor, self.windfactor)

    else:
      # Bosch (0111): pcm_speed/pcm_accel for ACC_HUD only; gas/brake commands use separate path below
      speed_control = 0
      max_accel = np.interp(CS.out.vEgo, self.params.NIDEC_MAX_ACCEL_BP, self.params.NIDEC_MAX_ACCEL_V)
      pcm_speed_BP = [-wind_brake,
                      -wind_brake * (3 / 4),
                      0.0,
                      0.5]
      if not CC.longActive:
        pcm_speed = 0.0
        pcm_accel = int(0.0)
      else:
        pcm_speed_V = [0.0,
                       np.clip(CS.out.vEgo - 2.0, 0.0, 100.0),
                       np.clip(CS.out.vEgo + 2.0, 0.0, 100.0),
                       np.clip(CS.out.vEgo + 5.0, 0.0, 100.0)]
        pcm_speed = float(np.interp(gas - brake, pcm_speed_BP, pcm_speed_V))
        pcm_accel = int(np.clip((accel / 1.44) / max_accel, 0.0, 1.0) * self.params.NIDEC_GAS_MAX)
    if not self.CP.openpilotLongitudinalControl:
      if self.frame % 2 == 0 and self.CP.carFingerprint not in HONDA_BOSCH_RADARLESS | HONDA_BOSCH_CANFD:
        can_sends.append(hondacan.create_bosch_supplemental_1(self.packer, self.CAN))
      # If using stock ACC, spam cancel command to kill gas when OP disengages.
      if pcm_cancel_cmd:
        can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CruiseButtons.CANCEL, 0, CS.scm_ambient_light, self.CP.carFingerprint))
      elif CC.cruiseControl.resume:
        can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CruiseButtons.RES_ACCEL, 0, CS.scm_ambient_light, self.CP.carFingerprint))

    else:
      # Send gas and brake commands.
      if self.frame % 2 == 0:
        ts = self.frame * DT_CTRL

        if self.CP.carFingerprint in HONDA_BOSCH:
          if (accel < 1e-3) and (CS.out.vEgo < 3.0) and not (-1e-3 < CS.out.vEgo < 1e-3):
            brake_addon = self.brake_pid.update(error = accel - CS.out.aEgo, speed = CS.out.vEgo)
            targetaccel = min(accel,accel + brake_addon)
          else:
            if self.brake_pid.i < 0.0:
              self.brake_pid.i = min(0.0, self.brake_pid.i + 0.02) # release 1m/s2 @ 50hz
            else:
              self.brake_pid.reset()
            targetaccel = min(accel,accel + self.brake_pid.i)

          self.accel = float(np.clip(targetaccel, self.params.BOSCH_ACCEL_MIN, self.params.BOSCH_ACCEL_MAX))
          gas_pedal_force = targetaccel + wind_brake_ms2 * self.windfactor + hill_brake + self.gasalpha

          # live-learn gas pedal adjustments when openpilot is controlling gas
          if (actuators.longControlState == LongCtrlState.pid) and (not CS.out.gasPressed):
            gas_error = accel - CS.out.aEgo
            if self.CP.carFingerprint in (CAR.HONDA_INSIGHT, CAR.HONDA_CIVIC_BOSCH): # gas pedal reacts too slowly
              learn_speed = 150
            elif self.CP.carFingerprint in (CAR.ACURA_RDX_3G, CAR.ACURA_RDX_3G_MMR): # Prevent overreacting to turbo lag
              learn_speed = 300
            else:
              learn_speed = 50
            if gas_error != 0.0 and gas_pedal_force > 0:
              self.gasfactor = np.clip(self.gasfactor + gas_error / learn_speed * gas_pedal_force, 0.01, 3.0)
            if (-0.5 < gas_pedal_force - self.gasalpha < 0.1) and (CS.out.vEgo > 1.0):
              self.gasalpha = float(np.clip(self.gasalpha + gas_error / learn_speed / 10.0, 0.0, 0.4))
            if gas_error != 0.0 and (not CS.out.brakePressed) and (CS.out.vEgo > 0.0):
              if self.CP.carFingerprint in (CAR.ACURA_RDX_3G, CAR.ACURA_RDX_3G_MMR): # Faster reaction
                wind_learn_speed = 100
              else:
                wind_learn_speed = 1000
              wind_adjust = 1 + wind_brake_ms2 / wind_learn_speed
              self.windfactor = np.clip(self.windfactor * (wind_adjust if (gas_error > 0) else 1.0/wind_adjust), 0.1, 3.0)
            if gas_pedal_force <= 0.0: # don't reduce windfactor while braking, allow increases
              self.windfactor = max(self.windfactor, self.windfactor_before_brake)
            else:
              self.windfactor_before_brake = self.windfactor
            if gas_pedal_force >= self.params.BOSCH_ACCEL_MAX: # don't increase gasfactor nor windfactor at accel max, allow decreases
              self.gasfactor = min(self.gasfactor, self.gasfactor_before_gasmax)
              self.windfactor = min(self.windfactor, self.windfactor_before_gasmax)
            else:
              self.gasfactor_before_gasmax = self.gasfactor
              self.windfactor_before_gasmax = self.windfactor
          self.gas = float(np.interp((gas_pedal_force) * self.gasfactor,
                                     [0, self.params.BOSCH_GAS_LOOKUP_BP[1]], self.params.BOSCH_GAS_LOOKUP_V))

          # limit gas ramp to 60 units per frame, matches stock.  Higher sometimes causes powertrain to ignore gas command.
          max_gas = max(60, self.bosch_last_gas + 60)
          self.gas = min(self.gas, max_gas)
          self.bosch_last_gas = self.gas

          stopping = actuators.longControlState == LongCtrlState.stopping
          self.stopping_counter = self.stopping_counter + 1 if stopping else 0
          # CAN FD: never overlap the stock radar's own ACC_CONTROL stream; ours starts within a few
          # frames of the radar going silent (see the deferred radar disable above)
          if not (self.CP.carFingerprint in HONDA_BOSCH_CANFD and CS.stock_acc_alive):
            can_sends.extend(hondacan.create_acc_commands(self.packer, self.CAN, CC.enabled, CC.longActive, self.accel, self.gas,
                                                          self.stopping_counter, self.CP, gas_pedal_force))
        else:
          apply_brake = np.clip(self.brake_last - wind_brake, 0.0, 1.0)
          if (apply_brake > 0) and (actuators.longControlState == LongCtrlState.pid) and (CS.out.vEgo > 1e-5) and (not CS.out.stockAeb):
              if not ((self.accel >= 1e-5) and CS.out.vEgo < 1.0): # don't wind PID at launch lurch
                self.brake_pid_factor = self.nidec_brake_pid.update(error = -(self.accel - CS.out.aEgo) * apply_brake, speed = CS.out.vEgo)
          if (CS.out.vEgo >= 2): # save pid above 2m/s
            self.brake_pid_factor_non_lowspeed = self.brake_pid_factor
          if (CS.out.vEgo < 1e-5) and (self.accel < 1e-5): # gradually restore 2m/s pid after stopped
            self.nidec_brake_pid.i = float(np.clip(self.brake_pid_factor_non_lowspeed, self.nidec_brake_pid.i - 0.01, self.nidec_brake_pid.i + 0.01))
          brakefactor = 1 + self.brake_pid_factor
          apply_brake = int(np.clip(apply_brake * brakefactor * self.params.NIDEC_BRAKE_MAX, 0, self.params.NIDEC_BRAKE_MAX - 1))
          pump_on, self.last_pump_ts = brake_pump_hysteresis(apply_brake, self.apply_brake_last, self.last_pump_ts, ts)

          # limit brake release to 32 units per frame to match factory
          apply_brake = max(self.apply_brake_last - 32, apply_brake)

          pcm_override = CC.longActive or CS.out.stockAeb
          if apply_brake > 0: # prevent fault from concurrent gas + brake
            pcm_speed = 0.0
            self.new_accel = 0

          can_sends.append(hondacan.create_brake_command(self.packer, self.CAN, apply_brake, pump_on,
                                                         pcm_override, pcm_cancel_cmd, alert_fcw,
                                                         self.CP, CS.stock_brake))
          # during a driver-gas override the wire now carries the pedal mirror set above, so
          # the PCM tracker (and the feedforward state) stay wound to the true operating
          # point; platforms without GAS_PEDAL_2 keep the old zeroing since there is nothing
          # to mirror. (A previous branch here set 198 during gasPressed and was immediately
          # overwritten by the zeroing below — dead code, removed.)
          if (apply_brake > 0) or (CS.out.gasPressed and not CS.car_gas_available):
            self.new_accel = 0

          # openpilot brake release: the guard above held the wire at 0 for the whole brake
          # period, so the gas channel restarts from a dead state exactly like an override
          # release, and 86% of releases have the plan positive within 1s (route 48: 1.9
          # releases/min engaged). Without the window the +2/tick clip alone puts ~0.6s of the
          # ~2s post-brake deficit on the wire before the PCM's own lag even starts, and the
          # dead window was billed to the speed/gas learners as servo undershoot (45% of the
          # speedfactor_low growth fuel came from the 13% of ticks in these windows; the
          # average_factor learner saw 3.5x its usual downward per-tick pressure there).
          # Launch windows own their own release and open the window at exit.
          if (self.apply_brake_last > 0) and (apply_brake == 0) and CC.longActive and (not self.launch_active):
            self.gas_recovery_ticks = 200

          if CS.out.vEgo < CS.out.cruiseState.speed - 2.:
            self.gasfactor_nomaxspeed = self.gasfactor
            self.gasfactor_low_nomaxspeed = self.gasfactor_low
            self.gas_alpha_nomaxspeed = self.gas_alpha
          else:
            # store lower than low max speed or current
            self.gasfactor_nomaxspeed = min(self.gasfactor_nomaxspeed, self.gasfactor)
            self.gasfactor_low_nomaxspeed = min(self.gasfactor_low_nomaxspeed, self.gasfactor_low)
            self.gas_alpha_nomaxspeed = min(self.gas_alpha_nomaxspeed, self.gas_alpha)

          self.apply_brake_last = apply_brake
          self.brake = apply_brake / self.params.NIDEC_BRAKE_MAX

    # Send dashboard UI commands. On CAN FD, ACC_HUD is a radar/ADAS look-alike that openpilot only
    # owns when it has disabled the radar (op longitudinal); in stock ACC the real system sends it and
    # the non-long safety config doesn't allowlist it.
    speed_control = 0 if self.CP.carFingerprint in HONDA_BOSCH else self.launch_active
    if (self.CP.carFingerprint in HONDA_BOSCH_CANFD) and CS.hud_tick and self.CP.openpilotLongitudinalControl and not CS.stock_acc_alive:
        can_sends.append(hondacan.create_acc_hud(self.packer, self.CAN.pt, self.CP, CC.enabled, pcm_speed, actuators.accel,
                                                 hud_control, hud_v_cruise, CS.is_metric, CS.acc_hud, speed_control,
                                                 self.CP.openpilotLongitudinalControl))

    if self.frame % 10 == 0:
      if self.CP.openpilotLongitudinalControl:
        if self.CP.carFingerprint not in HONDA_BOSCH_CANFD:
          # On Nidec, this also controls longitudinal positive acceleration
          acc_hud_pcm_accel = self.new_accel if self.CP.carFingerprint not in HONDA_BOSCH else pcm_accel
          can_sends.append(hondacan.create_acc_hud(self.packer, self.CAN.pt, self.CP, CC.enabled, pcm_speed, acc_hud_pcm_accel,
                                                   hud_control, hud_v_cruise, CS.is_metric, CS.acc_hud, speed_control,
                                                   self.CP.openpilotLongitudinalControl))

      steering_available = CS.out.cruiseState.available and CS.out.vEgo > max(self.params.STEER_GLOBAL_MIN_SPEED, self.CP.minSteerSpeed)
      reduced_steering = CS.out.steeringPressed
      if self.CP.carFingerprint in HONDA_BOSCH:
        steer_maxed = abs(apply_torque) >= self.params.STEER_MAX
      else:
        steer_maxed = (abs(apply_torque) >= self.params.STEER_MAX) or not (CS.steer_control_active)

      lkas_state_change = None
      if self.CP.carFingerprint in HONDA_BOSCH_CANFD:
        # The stock camera holds LKAS_STATE_CHANGE low and pulses it high for ~3s around HUD state
        # changes; holding it high permanently (the default below) suppresses the dash lane lines.
        # The key must contain exactly the signals that change the LKAS_HUD payload, nothing more:
        # steer_maxed used to be in here (via SOLID_LANES) and its 10Hz flicker during city-speed
        # steering re-triggered the pulse continuously, keeping LKAS_STATE_CHANGE high whenever a
        # real lane path was being sent - which suppressed the dash lane lines entirely.
        # latActive drives SOLID_LANES (under sunnypilot MADS the lateral control stays engaged
        # when ACC disengages, and the dash LKAS indication follows it); DASHED_LANES is overridden
        # to 1 on CAN FD, so hud_control.lanesVisible no longer affects the payload.
        hud_key = (bool(CC.latActive), bool(alert_steer_required), bool(CS.out.steerFaultPermanent))
        if hud_key != self.lkas_hud_key:
          self.lkas_hud_key = hud_key
          self.lkas_state_change_frames = 30  # 3s at the 10Hz LKAS_HUD rate, matching stock pulse length
        lkas_state_change = self.lkas_state_change_frames > 0
        self.lkas_state_change_frames = max(0, self.lkas_state_change_frames - 1)

      can_sends.extend(hondacan.create_lkas_hud(self.packer, self.CAN.lkas, self.CP, hud_control, CC.latActive,
                                                steering_available, reduced_steering, alert_steer_required, CS.lkas_hud, steer_maxed, CS,
                                                lkas_state_change=lkas_state_change))

      if self.CP.openpilotLongitudinalControl:
        # TODO: combining with create_acc_hud block above will change message order and will need replay logs regenerated
        if self.CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS - HONDA_BOSCH_CANFD):
          can_sends.append(hondacan.create_radar_hud(self.packer, self.CAN.pt))
        if self.CP.carFingerprint == CAR.HONDA_CIVIC_BOSCH:
          can_sends.append(hondacan.create_legacy_brake_command(self.packer, self.CAN.pt))
        if self.CP.carFingerprint not in HONDA_BOSCH:
          self.speed = pcm_speed
          self.gas = pcm_accel / self.params.NIDEC_GAS_MAX

    # Render OP's lane and lead car on the dash. On CAN FD these are radar look-alikes that only exist
    # (and are only allowed by panda safety) when the radar is disabled, i.e. openpilot longitudinal;
    # in stock ACC the real radar still owns LANE_PATH/HUD_OBJECTS, so don't author them.
    if ((self.frame % 2 == 0 and self.CP.carFingerprint in HONDA_BOSCH_RADARLESS) or
        (CS.radar_50hz_tick and self.CP.carFingerprint in HONDA_BOSCH_CANFD and self.CP.openpilotLongitudinalControl
         and not CS.stock_acc_alive)):
      leads = hud_objects.leads_from_model(self.model, CS.out.vEgo)
      lead = leads[0]
      lead_d = lead.dRel if lead.status else 0.0  # extend the lane out to the lead (0 = no lead)
      self.dash_lane = self.lane_path_fitter.update(self.model, CS.out.vEgo, lead_d)
      # Important: same mux for lane_path and hud_objects. Lane display freezes if muxes don't match.
      if self.CP.carFingerprint in HONDA_BOSCH_CANFD:
        # self.radar_mux advances one step per 50Hz tick (above), so the mux sweep stays contiguous
        # across missed ticks, unlike a frame-derived mux.
        mux = self.radar_mux
        # No LKAS_HUD_2 on CAN FD: the dash reads the lane length from the stock radar's in-band
        # terminator, so reshape the path into the terminated-prefix form (see lane_path.py).
        lane_offsets = lane_path.canfd_lane_offsets(self.dash_lane)
      else:
        mux = lane_path.MUX_CYCLE[(self.frame // 2) % len(lane_path.MUX_CYCLE)]
        lane_offsets = self.dash_lane.offsets
      lane_msg = lane_path.create_lane_path(self.packer, self.CAN.lkas, lane_offsets, mux)
      can_sends.append(lane_msg)

      # CAN FD cars have no camera HUD_OBJECTS to poll (the disabled radar owned it), so there are no
      # secondary vehicle locations: author OP's lead in slot 0 with the other slots blank (tracks=None).
      tracks = CS.hud_object_tracker.snapshot() if CS.hud_object_tracker is not None else None
      if self.CP.openpilotLongitudinalControl:
        # For OP long, replace lead car and forward rest of objects
        hud_msg = self.hud_object_author.create(self.packer, self.CAN.lkas, lead, tracks, mux, now_nanos * 1e-9,
                                                extra_leads=leads[1:])
      else:
        # For ACC, forward objects but with our mux
        hud_msg = hud_objects.forward_hud_object(self.packer, self.CAN.lkas, mux, tracks)
      can_sends.append(hud_msg)

      # On CAN FD the camera (behind the relay) also consumes these radar look-alikes, and openpilot's
      # own TX is not forwarded across the open relay. Mirror the identical packed bytes onto the camera
      # bus (packed once above, so the counter/checksum don't double-increment and both buses match).
      if self.CP.carFingerprint in HONDA_BOSCH_CANFD:
        for addr, dat, _ in (lane_msg, hud_msg):
          can_sends.append((addr, dat, self.CAN.camera))

    if self.frame % 20 == 0 and self.CP.carFingerprint in HONDA_BOSCH_RADARLESS:
      # COUNTER_2 trails the packer's COUNTER (frame//20 % 4) by one. TODO: do we need the - 1 trail?
      dl = self.dash_lane
      can_sends.append(lane_path.create_lkas_hud_2(self.packer, self.CAN.lkas, (self.frame // 20 - 1) % 4,
                                                   dl.reach, dl.lane_cross, dl.left_line, dl.right_line))

    # Radarless + CAN FD: when stock LKAS is active, the touch-steering-wheel timer/nag eventually forces an ACC
    # disengagement (on CAN FD it shows up as a brake tap from the VSA). Disable LKAS automatically and block the
    # driver's LKAS button from reaching the camera by taking over SCM_BUTTONS on the camera bus while engaged
    # (panda blocks the forwarded stock SCM_BUTTONS when engaged; the standard button spamming isn't reliably
    # accepted by the camera).
    if self.CP.carFingerprint in (HONDA_BOSCH_RADARLESS | HONDA_BOSCH_CANFD) and CC.enabled and self.frame % 4 == 0 and \
        not pcm_cancel_cmd and not CC.cruiseControl.resume:
      if self.lkas_button_send_remaining == 0 and CS.lkas_hud["LKAS_READY"] and self.frame >= self.last_lkas_button_frame + 500:
        self.lkas_button_send_remaining = 3

      if self.lkas_button_send_remaining > 0:
        self.last_lkas_button_frame = self.frame
        self.lkas_button_send_remaining -= 1
        cruise_setting = CruiseSettings.LKAS
      elif CS.cruise_setting == CruiseSettings.LKAS:
        cruise_setting = 0  # block driver's LKAS button press
      else:
        cruise_setting = CS.cruise_setting

      can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CS.cruise_buttons, cruise_setting,
                                                     CS.scm_ambient_light, self.CP.carFingerprint, bus=self.CAN.camera))

    new_actuators = actuators.as_builder()
    if self.CP.carFingerprint in HONDA_BOSCH:
      new_actuators.speed = float(self.gasalpha)
      new_actuators.accel = self.accel
      new_actuators.gas = float(self.gasfactor)
      new_actuators.brake = float(self.windfactor)
    else:
      new_actuators.speed = float(self.nidec_pid_factor)
      new_actuators.accel = float(self.accel)
      new_actuators.gas = float(self.average_factor)
      new_actuators.brake = float(self.sat_accel)
    new_actuators.torque = self.last_torque
    if self.CP.carFingerprint in HONDA_BOSCH:
      new_actuators.torqueOutputCan = apply_torque
    else:
      new_actuators.torqueOutputCan = float(self.speedfactor_low)

    if self.frame % 6000 == 0:
      if self.CP.carFingerprint in HONDA_BOSCH:
        self.param_writer.put_many({
          "HondaGasAlphaParams": self.gasalpha,
          "HondaGasFactorParams": self.gasfactor,
          "HondaWindFactorParams": self.windfactor,
        })
      else:
        self.param_writer.put_many({
          "HondaFeedForwardParams": self.average_factor,
          "HondaBrakePIDParams": self.brake_pid_factor_non_lowspeed,
          "HondaCreepFactorParams": self.creep_factor,
          "HondaGasAlphaParams": self.gas_alpha_nomaxspeed,
          "HondaGasFactorParams": self.gasfactor_nomaxspeed,
          "HondaGasFactorLowParams": self.gasfactor_low_nomaxspeed,
          "HondaWindFactorParams": self.windfactor,
          "HondaSpeedAlphaParams": self.speedalpha,
          "HondaSpeedFactorParams": self.speedfactor,
          "HondaSpeedAlphaLowParams": self.speedalpha_low,
          "HondaSpeedFactorLowParams": self.speedfactor_low,
          "HondaSatAccelParams": self.sat_accel,
          "HondaCarGasScaleParams": self.car_gas_per_pcm_gas,
          "HondaLaunchDvParams": self.dv_launch,
          "HondaLaunchGasParams": self.gas_launch,
          "HondaLaunchDvBreakParams": self.dv_break,
        })

    if self.frame % 12000 == 30 and self.CP.carFingerprint not in HONDA_BOSCH:
      self.param_writer.put_many({
        "HondaLatAccelFactor05Params": self.latFactors["05"],
        "HondaLatAccelFactor10Params": self.latFactors["10"],
        "HondaLatAccelFactor15Params": self.latFactors["15"],
        "HondaLatAccelFactor20Params": self.latFactors["20"],
        "HondaLatAccelFactor25Params": self.latFactors["25"],
        "HondaLatAccelFactor30Params": self.latFactors["30"],
        "HondaLatAccelFactor35Params": self.latFactors["35"],
        "HondaLatAccelFactor40Params": self.latFactors["40"],
        "HondaLatAccelFactor45Params": self.latFactors["45"],
        "HondaLatAccelFactor50Params": self.latFactors["50"],
        "HondaLatAccelFactor55Params": self.latFactors["55"],
        "HondaLatAccelFactor60Params": self.latFactors["60"],
      })

    self.frame += 1
    return new_actuators, can_sends

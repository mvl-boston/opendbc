import math
import threading
from queue import Empty, Queue

import numpy as np
from openpilot.common.params import Params
from opendbc.car.common.conversions import Conversions as CV

from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, rate_limit, make_tester_present_msg, structs
from opendbc.car.honda import hondacan
from opendbc.car.honda.values import CAR, CruiseButtons, CruiseSettings, HondaFlags, CarControllerParams
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.common.pid import PIDController
from opendbc.car.honda import lane_path
from opendbc.car.honda import hud_objects

from opendbc.sunnypilot.car.honda.mads import MadsCarController
from opendbc.sunnypilot.car.honda.gas_interceptor import GasInterceptorCarController
from opendbc.sunnypilot.car.honda.icbm import IntelligentCruiseButtonManagementInterface

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


def compute_gas_brake(accel, speed, CP):
  if CP.flags & HondaFlags.BOSCH:
    return compute_gb_honda_bosch(accel, speed)
  else:
    creep_brake = 0.0
    creep_speed = 2.3
    creep_brake_value = 0.15
    if speed < creep_speed:
      creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
    gb = float(accel) / 4.8 - creep_brake
    return np.clip(gb, 0.0, 1.0), np.clip(-gb, 0.0, 1.0)


# TODO not clear this does anything useful
def actuator_hysteresis(brake, braking, brake_steady):
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


class CarController(CarControllerBase, MadsCarController, GasInterceptorCarController, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    MadsCarController.__init__(self)
    GasInterceptorCarController.__init__(self, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)
    self.CAN = hondacan.CanBus(CP)
    self.hud_object_author = hud_objects.HudObjectAuthor()
    self.lane_path_fitter = lane_path.LanePathFitter()
    self.dash_lane = lane_path.DashLane([lane_path.OFFSET_UNAVAILABLE] * lane_path.NUM_PTS, 0.0, False, False)
    self.lkas_hud_key = None
    self.lkas_state_change_frames = 0
    self.tja_control = bool(CP.flags & HondaFlags.BOSCH_TJA_CONTROL)
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
    self.gasfactor_nomaxspeed = self.gasfactor
    self.gasfactor_low = self.gasfactor if (Params().get("HondaGasFactorLowParams") is None) else Params().get("HondaGasFactorLowParams")
    self.gasfactor_low_before_gasmax = self.gasfactor_low_nomaxspeed = self.gasfactor_low
    self.speedfactor = 4.0 if (Params().get("HondaSpeedFactorParams") is None) else Params().get("HondaSpeedFactorParams")
    self.speedalpha = 0.0 if (Params().get("HondaSpeedAlphaParams") is None) else Params().get("HondaSpeedAlphaParams")
    self.speedfactor_low = 4.0 if (Params().get("HondaSpeedFactorLowParams") is None) else Params().get("HondaSpeedFactorLowParams")
    self.speedalpha_low = 0.0 if (Params().get("HondaSpeedAlphaLowParams") is None) else Params().get("HondaSpeedAlphaLowParams")
    self.sat_accel = 0.9 if (Params().get("HondaSatAccelParams") is None) else Params().get("HondaSatAccelParams")
    self.sat_deficit_frames = self.sat_excess_frames = 0
    self.new_accel = 0.0

    self.dv_launch = 2.8 if (Params().get("HondaLaunchDvParams") is None) else Params().get("HondaLaunchDvParams")
    self.gas_launch = 110.0 if (Params().get("HondaLaunchGasParams") is None) else Params().get("HondaLaunchGasParams")
    self.dv_break = 6.0 if (Params().get("HondaLaunchDvBreakParams") is None) else Params().get("HondaLaunchDvBreakParams")
    self.launch_active = False
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

  def update(self, CC, CC_SP, CS, now_nanos):
    MadsCarController.update(self, self.CP, CC, CC_SP)
    gas_pedal_force = 0.0
    actuators = CC.actuators
    hud_control = CC.hudControl
    hud_v_cruise = hud_control.setSpeed / CS.v_cruise_factor if hud_control.speedVisible else 255
    pcm_cancel_cmd = CC.cruiseControl.cancel

    is_bosch = self.CP.flags & HondaFlags.BOSCH
    is_wire_gas = not is_bosch and not self.CP_SP.enableGasInterceptor

    if len(CC.orientationNED) == 3:
      self.pitch = CC.orientationNED[1]
    hill_brake = math.sin(self.pitch) * ACCELERATION_DUE_TO_GRAVITY

    # wind brake from air resistance decel at high speed
    wind_brake = np.interp(CS.out.vEgo, [0.0, 2.3, 35.0], [0.001, 0.002, 0.15]) * self.windfactor # not in m/s2 units
    prior_windfactor = self.windfactor

    accel = 0.0
    gas = 0.0
    brake = 0.0
    adjust_accel = 0.0

    if CC.longActive:
      if is_bosch:
        accel = actuators.accel
        adjust_accel = accel + hill_brake
        brake = 0.0
        gas, brake = compute_gas_brake(adjust_accel, CS.out.vEgo, self.CP)
      elif self.CP_SP.enableGasInterceptor:
        accel = actuators.accel
        gas, brake = compute_gas_brake(actuators.accel + hill_brake, CS.out.vEgo, self.CP)
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
            self.gasfactor = self.gasfactor_nomaxspeed
            self.gasfactor_low = self.gasfactor_low_nomaxspeed
            self.gas_alpha = self.gas_alpha_nomaxspeed
          if (gas_pedal_force >= self.params.BOSCH_ACCEL_MAX):
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
    else:
      self.accel = 0.0
      adjust_accel = self.accel
      brake = 0.0
      accel = 0.0
      gas, brake = 0.0, 0.0

    if CC.longActive and is_wire_gas:
      accel = self.accel
    if CS.out.gasPressed or not CC.longActive:
      if is_wire_gas:
        self.nidec_pid.reset()

    # *** rate limit steer ***
    limited_torque = rate_limit(actuators.torque, self.last_torque, -self.params.STEER_DELTA_DOWN * DT_CTRL,
                                self.params.STEER_DELTA_UP * DT_CTRL)
    if is_wire_gas and (self.CP.carFingerprint == CAR.ACURA_MDX_3G) and \
        (self.apply_brake_last > 0 or self.new_accel < 1e-5): # lower steer limits while braking
      brake_limit = float(233.0 / self.params.STEER_MAX)
      limited_torque = float(np.clip(limited_torque, -brake_limit, brake_limit))
    self.last_torque = limited_torque

    # *** apply brake hysteresis ***
    pre_limit_brake, self.braking, self.brake_steady = actuator_hysteresis(brake, self.braking, self.brake_steady)

    # *** rate limit after the enable check ***
    self.brake_last = rate_limit(pre_limit_brake, self.brake_last, -2., 3 * DT_CTRL)

    # vehicle hud display, wait for one update from 10Hz 0x304 msg
    alert_fcw, alert_steer_required = process_hud_alert(hud_control.visualAlert)

    # **** process the car messages ****

    # steer torque is converted back to CAN reference (positive when steering right)
    apply_torque = int(np.interp(-limited_torque * self.params.STEER_MAX,
                                 self.params.STEER_LOOKUP_BP, self.params.STEER_LOOKUP_V))

    if is_wire_gas:
      speed_val = np.clip(round(CS.out.vEgo * CV.MS_TO_MPH / 5.0) * 5, 5, 60)
      currentLatSpeed = f"{speed_val:02d}"
      if currentLatSpeed in self.latFactors and not CS.out.steeringPressed and CS.steer_control_active:
        if abs(limited_torque) > 0.9 and self.latFactors[currentLatSpeed] > abs(CS.out.steeringAngleDeg):
          self.latFactors[currentLatSpeed] /= 1.001
        if abs(limited_torque) < 0.9 and self.latFactors[currentLatSpeed] < abs(CS.out.steeringAngleDeg):
          self.latFactors[currentLatSpeed] *= 1.001

    # Send CAN commands
    can_sends = []

    if is_bosch and not (self.CP.flags & HondaFlags.BOSCH_RADARLESS) and self.CP.openpilotLongitudinalControl:
      if self.CP.flags & HondaFlags.BOSCH_CANFD and CS.stock_acc_alive:
        if CS.canfd_relay_open:
          if self.radar_disable_counter % 50 == 0:
            can_sends.append((0x18DAB0F1, b'\x02\x10\x03\x00\x00\x00\x00\x00', self.CAN.pt))
          elif self.radar_disable_counter % 50 == 5:
            can_sends.append((0x18DAB0F1, b'\x03\x28\x83\x03\x00\x00\x00\x00', self.CAN.pt))
          self.radar_disable_counter += 1
      elif self.frame % 10 == 0:
        bus = 0 if self.CP.flags & HondaFlags.BOSCH_CANFD else 1
        can_sends.append(make_tester_present_msg(0x18DAB0F1, bus, suppress_response=True))

    if (self.CP.flags & HondaFlags.BOSCH_CANFD) and self.CP.openpilotLongitudinalControl and not CS.stock_acc_alive:
      if CC.enabled and not self.last_acc_enabled:
        self.radar_hud_pulse = 30
      self.last_acc_enabled = CC.enabled
      radar_msgs = []
      if CS.hud_tick:
        radar_msgs.append(hondacan.create_radar_hud_canfd(self.packer, self.CAN.pt, CC.enabled, self.radar_hud_pulse > 0))
        if self.radar_hud_pulse > 0:
          self.radar_hud_pulse -= 1
      if CS.supp_tick:
        radar_msgs.append(hondacan.create_canfd_supplemental(self.packer, self.CAN.pt))
      if CS.radar_50hz_tick:
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
      if CS.radar_5hz_tick:
        radar_msgs.extend(hondacan.create_canfd_5hz_radar_messages(self.packer, self.CAN.pt, CS.radar_ref_counter,
                                                                   lane_path.canfd_lane_length(self.dash_lane),
                                                                   lane_path.LANE_LINE_ON if self.dash_lane.left_line else 0,
                                                                   lane_path.LANE_LINE_ON if self.dash_lane.right_line else 0))

      for addr, dat, _ in radar_msgs:
        can_sends.append((addr, dat, self.CAN.pt))
        can_sends.append((addr, dat, self.CAN.camera))

    # Send steering command.
    can_sends.append(hondacan.create_steering_control(self.packer, self.CAN, apply_torque, CC.latActive, self.tja_control))

    wind_brake_ms2 = np.interp(CS.out.vEgo, [0.0, 13.4, 22.4, 31.3, 40.2], [0.000, 0.049, 0.136, 0.267, 0.441]) # in m/s2 units

    # launch governor state machine (wire-gas Nidec)
    if is_wire_gas:
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
          self.launch_lurch_sum += CS.out.aEgo - actuators.accel
          self.launch_lurch_n += 1
        if self.launch_move_tick >= 0:
          self.launch_err_sum += actuators.accel - CS.out.aEgo
          self.launch_err_n += 1
          if (CS.engine_rpm > 500) and (CS.car_gas >= 0.8 * self.car_gas_per_pcm_gas * self.gas_launch):
            self.launch_ceiling_ticks += 1
        launch_aborted = (not CC.longActive) or CS.out.gasPressed or CS.out.brakePressed or (actuators.accel < -0.05)
        launch_done = (CS.out.vEgo > 1.5) or (self.launch_ticks >= 300)
        if launch_aborted or launch_done:
          if launch_done and not launch_aborted:
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
            self.gas_recovery_ticks = 200

      self.gas_recovery_ticks = max(self.gas_recovery_ticks - 1, 0)
      if CC.enabled and self.gas_pressed_prev and (not CS.out.gasPressed):
        self.gas_recovery_ticks = 200
      self.gas_pressed_prev = CS.out.gasPressed
      if CC.longActive and (not self.long_active_prev) and (CS.out.vEgo > 0.1) and (not self.launch_active):
        self.gas_recovery_ticks = 200
      self.long_active_prev = CC.longActive

    if is_wire_gas:
      max_accel = np.interp(CS.out.vEgo, self.params.NIDEC_MAX_ACCEL_BP, self.params.NIDEC_MAX_ACCEL_V)
      low_w = float(np.interp(CS.out.vEgo, [10.0, 16.0], [1.0, 0.0]))
      sf_eff = low_w * self.speedfactor_low + (1.0 - low_w) * self.speedfactor
      alpha_eff = low_w * self.speedalpha_low + (1.0 - low_w) * self.speedalpha
      if not CC.longActive:
        if CC.enabled and CS.out.gasPressed and CS.car_gas_available:
          pcm_speed = float(np.clip(CS.out.vEgo, 0.0, 100.0))
          pcm_accel = int(np.clip(CS.car_gas / max(self.car_gas_per_pcm_gas, 1e-3), 0.0, self.params.NIDEC_GAS_MAX))
        else:
          pcm_speed = 0.0
          pcm_accel = int(0.0)
      else:
        if self.launch_active:
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

      prior_accel = int(self.new_accel)
      max_increase = 7 if (self.gas_recovery_ticks > 0 and prior_accel < pcm_accel) else 2
      effective_average_factor = self.average_factor if CS.car_gas_available else 0.5
      self.new_accel = int((pcm_accel - self.prior_gas_average * (1 - effective_average_factor)) / effective_average_factor)
      self.new_accel = int(np.clip(self.new_accel, 0, min(prior_accel + max_increase, self.params.NIDEC_GAS_MAX)))
      if self.launch_active:
        launch_seed = self.gas_launch
        if (CS.engine_rpm < 500) and (CS.out.vEgo <= 0.1):
          launch_seed = self.params.NIDEC_GAS_MAX
        self.new_accel = int(min(launch_seed, self.params.NIDEC_GAS_MAX))
      self.average_factor_sens = (self.new_accel - self.prior_gas_average) + (1 - effective_average_factor) * self.average_factor_sens
      self.prior_gas_average = self.prior_gas_average * (1 - effective_average_factor) + (self.new_accel * effective_average_factor)

      if (0 < self.new_accel < self.params.NIDEC_GAS_MAX) and (not CS.out.gasPressed) and \
           (self.apply_brake_last == 0) and (not self.launch_active) and (self.gas_recovery_ticks == 0):
        gasfactor_error = (self.accel - CS.out.aEgo)
        self.gas_alpha = np.clip(self.gas_alpha + 0.0001 * gasfactor_error / 4.8, -3.0, 3.0)
        gf_growth = 0.0001 * gasfactor_error * gas_accel
        self.gasfactor_low = float(np.clip(self.gasfactor_low * (1 + low_w * gf_growth), 0.1, 5.0))
        self.gasfactor = float(np.clip(self.gasfactor * (1 + (1.0 - low_w) * gf_growth), 0.1, 5.0))
      if (not CS.out.gasPressed) and (self.apply_brake_last == 0):
        speedfactor_error = (self.accel - CS.out.aEgo)
        dv_sent = sf_eff * self.accel + alpha_eff
        dv_sat = max(0.1, sf_eff * self.sat_accel + alpha_eff)

        if CC.longActive and (CS.out.vEgo > 1e-5) and CS.car_gas_available and \
             (CS.engine_rpm > 500) and (not self.launch_active) and (self.gas_recovery_ticks == 0):
          wire_gas = float(self.new_accel)
          if (wire_gas > 20.0) and (CS.car_gas > 5.0) and (abs(wire_gas - prior_accel) <= 1.0):
            scale_sample = CS.car_gas / wire_gas
            self.car_gas_per_pcm_gas += 0.0005 * (scale_sample - self.car_gas_per_pcm_gas)

          self.car_gas_per_pcm_gas = max(0.00001, self.car_gas_per_pcm_gas)
          gas_measured = CS.car_gas / self.car_gas_per_pcm_gas
          averagefactor_error = (gas_measured - self.prior_gas_average) / self.params.NIDEC_GAS_MAX
          averagefactor_step = 0.005 * averagefactor_error * self.average_factor_sens / self.params.NIDEC_GAS_MAX
          self.average_factor = float(np.clip(self.average_factor + np.clip(averagefactor_step, -0.0001, 0.0001),
                                              0.02, 1.0))

        if (CS.out.aEgo > self.sat_accel) and (not CS.out.gasPressed) and (CC.longActive):
          self.sat_excess_frames += 1
        else:
          self.sat_excess_frames = 0
        if (CS.out.aEgo < self.sat_accel <= self.accel) and (not CS.out.gasPressed) and (CC.longActive) and \
             (not self.launch_active) and (self.gas_recovery_ticks == 0):
          self.sat_deficit_frames += 1
        else:
          self.sat_deficit_frames = 0
        if (self.sat_excess_frames > 100) or (self.sat_deficit_frames > 100):
          self.sat_accel = float(np.clip(self.sat_accel + 0.002 * (CS.out.aEgo - self.sat_accel), 0.1, self.params.NIDEC_ACCEL_MAX - 0.1))

        if CC.longActive and (CS.out.vEgo > 1e-5) and (not self.launch_active) and (self.gas_recovery_ticks == 0):
          if (speedfactor_error > 0) and (dv_sent > dv_sat):
            sf_growth = -min(0.0005 * (dv_sent - dv_sat), 0.005)
          else:
            sf_growth = 0.001 * speedfactor_error * self.accel
          self.speedfactor_low = float(np.clip(self.speedfactor_low * (1 + low_w * sf_growth), 0.01, 99.0))
          self.speedfactor = float(np.clip(self.speedfactor * (1 + (1.0 - low_w) * sf_growth), 0.01, 99.0))
          self.speedalpha_low = min(dv_sat, self.speedalpha_low + low_w * 0.001 * speedfactor_error)
          self.speedalpha = min(dv_sat, self.speedalpha + (1.0 - low_w) * 0.001 * speedfactor_error)
        if max_speedcontrol or (dv_sent > dv_sat):
          self.speedfactor = min(prior_speedfactor, self.speedfactor)
          self.speedfactor_low = min(prior_speedfactor_low, self.speedfactor_low)
          self.speedalpha = min(prior_speedalpha, self.speedalpha)
          self.speedalpha_low = min(prior_speedalpha_low, self.speedalpha_low)
          self.windfactor = min(prior_windfactor, self.windfactor)

    elif is_bosch:
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
    else:
      # gas interceptor: PCM HUD not used
      pcm_speed = 0.0
      pcm_accel = int(0.0)

    if not self.CP.openpilotLongitudinalControl:
      if self.frame % 2 == 0 and not (self.CP.flags & (HondaFlags.BOSCH_RADARLESS | HondaFlags.BOSCH_CANFD)) and not (self.CP.flags & HondaFlags.NIDEC):
        can_sends.append(hondacan.create_bosch_supplemental_1(self.packer, self.CAN))
      if pcm_cancel_cmd:
        can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CruiseButtons.CANCEL, 0, CS.scm_ambient_light, self.CP))
      elif CC.cruiseControl.resume:
        can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CruiseButtons.RES_ACCEL, 0, CS.scm_ambient_light, self.CP))

    else:
      if self.frame % 2 == 0:
        ts = self.frame * DT_CTRL

        if is_bosch:
          if (accel < 1e-3) and (CS.out.vEgo < 3.0) and not (-1e-3 < CS.out.vEgo < 1e-3):
            brake_addon = self.brake_pid.update(error = accel - CS.out.aEgo, speed = CS.out.vEgo)
            targetaccel = min(accel,accel + brake_addon)
          else:
            if self.brake_pid.i < 0.0:
              self.brake_pid.i = min(0.0, self.brake_pid.i + 0.02)
            else:
              self.brake_pid.reset()
            targetaccel = min(accel,accel + self.brake_pid.i)

          self.accel = float(np.clip(targetaccel, self.params.BOSCH_ACCEL_MIN, self.params.BOSCH_ACCEL_MAX))
          gas_pedal_force = targetaccel + wind_brake_ms2 * self.windfactor + hill_brake + self.gasalpha

          if (actuators.longControlState == LongCtrlState.pid) and (not CS.out.gasPressed):
            gas_error = accel - CS.out.aEgo
            if self.CP.carFingerprint in (CAR.HONDA_INSIGHT, CAR.HONDA_CIVIC_BOSCH):
              learn_speed = 150
            elif self.CP.carFingerprint in (CAR.ACURA_RDX_3G, CAR.ACURA_RDX_3G_MMR):
              learn_speed = 300
            else:
              learn_speed = 50
            if gas_error != 0.0 and gas_pedal_force > 0:
              self.gasfactor = np.clip(self.gasfactor + gas_error / learn_speed * gas_pedal_force, 0.01, 3.0)
            if (-0.5 < gas_pedal_force - self.gasalpha < 0.1) and (CS.out.vEgo > 1.0):
              self.gasalpha = float(np.clip(self.gasalpha + gas_error / learn_speed / 10.0, 0.0, 0.4))
            if gas_error != 0.0 and (not CS.out.brakePressed) and (CS.out.vEgo > 0.0):
              if self.CP.carFingerprint in (CAR.ACURA_RDX_3G, CAR.ACURA_RDX_3G_MMR):
                wind_learn_speed = 100
              else:
                wind_learn_speed = 1000
              wind_adjust = 1 + wind_brake_ms2 / wind_learn_speed
              self.windfactor = np.clip(self.windfactor * (wind_adjust if (gas_error > 0) else 1.0/wind_adjust), 0.1, 3.0)
            if gas_pedal_force <= 0.0:
              self.windfactor = max(self.windfactor, self.windfactor_before_brake)
            else:
              self.windfactor_before_brake = self.windfactor
            if gas_pedal_force >= self.params.BOSCH_ACCEL_MAX:
              self.gasfactor = min(self.gasfactor, self.gasfactor_before_gasmax)
              self.windfactor = min(self.windfactor, self.windfactor_before_gasmax)
            else:
              self.gasfactor_before_gasmax = self.gasfactor
              self.windfactor_before_gasmax = self.windfactor
          self.gas = float(np.interp((gas_pedal_force) * self.gasfactor,
                                     [0, self.params.BOSCH_GAS_LOOKUP_BP[1]], self.params.BOSCH_GAS_LOOKUP_V))

          max_gas = max(60, self.bosch_last_gas + 60)
          self.gas = min(self.gas, max_gas)
          self.bosch_last_gas = self.gas

          stopping = actuators.longControlState == LongCtrlState.stopping
          self.stopping_counter = self.stopping_counter + 1 if stopping else 0
          if not (self.CP.flags & HondaFlags.BOSCH_CANFD and CS.stock_acc_alive):
            can_sends.extend(hondacan.create_acc_commands(self.packer, self.CAN, CC.enabled, CC.longActive, self.accel, self.gas,
                                                          self.stopping_counter, self.CP, gas_pedal_force))
        elif self.CP_SP.enableGasInterceptor:
          apply_brake = np.clip(self.brake_last - wind_brake, 0.0, 1.0)
          apply_brake = int(np.clip(apply_brake * self.params.NIDEC_BRAKE_MAX, 0, self.params.NIDEC_BRAKE_MAX - 1))
          pump_on, self.last_pump_ts = brake_pump_hysteresis(apply_brake, self.apply_brake_last, self.last_pump_ts, ts)

          pcm_override = True
          can_sends.append(hondacan.create_brake_command(self.packer, self.CAN, apply_brake, pump_on,
                                                         pcm_override, pcm_cancel_cmd, alert_fcw,
                                                         CS.stock_brake, self.CP_SP))
          self.apply_brake_last = apply_brake
          self.brake = apply_brake / self.params.NIDEC_BRAKE_MAX

          gas_error = actuators.accel - CS.out.aEgo
          if (not CS.out.gasPressed) and (actuators.longControlState == LongCtrlState.pid) and self.CP_SP.enableGasInterceptor:
            if gas_error != 0.0 and gas > 0.0:
              self.gasfactor = np.clip(self.gasfactor + gas_error / 150 * (gas * 4.8), 0.1, 3.0)
            if gas_error != 0.0 and (not CS.out.brakePressed) and (CS.out.vEgo > 0.0):
              wind_adjust = 1 + (wind_brake * 4.8) / 1000
              self.windfactor = np.clip(self.windfactor * (wind_adjust if (gas_error > 0) else 1.0/wind_adjust), 0.1, 5.0)
            if gas <= 0.0:
              self.windfactor = max(self.windfactor, self.windfactor_before_brake)
            else:
              self.windfactor_before_brake = self.windfactor

          can_sends.extend(GasInterceptorCarController.update(self, CC, CS, gas * self.gasfactor, brake, wind_brake, self.packer, self.frame))
        else:
          apply_brake = np.clip(self.brake_last - wind_brake, 0.0, 1.0)
          if (apply_brake > 0) and (actuators.longControlState == LongCtrlState.pid) and (CS.out.vEgo > 1e-5) and (not CS.out.stockAeb):
              if not ((self.accel >= 1e-5) and CS.out.vEgo < 1.0):
                self.brake_pid_factor = self.nidec_brake_pid.update(error = -(self.accel - CS.out.aEgo) * apply_brake, speed = CS.out.vEgo)
          if (CS.out.vEgo >= 2):
            self.brake_pid_factor_non_lowspeed = self.brake_pid_factor
          if (CS.out.vEgo < 1e-5) and (self.accel < 1e-5):
            self.nidec_brake_pid.i = float(np.clip(self.brake_pid_factor_non_lowspeed, self.nidec_brake_pid.i - 0.01, self.nidec_brake_pid.i + 0.01))
          brakefactor = 1 + self.brake_pid_factor
          apply_brake = int(np.clip(apply_brake * brakefactor * self.params.NIDEC_BRAKE_MAX, 0, self.params.NIDEC_BRAKE_MAX - 1))
          pump_on, self.last_pump_ts = brake_pump_hysteresis(apply_brake, self.apply_brake_last, self.last_pump_ts, ts)

          apply_brake = max(self.apply_brake_last - 32, apply_brake)

          pcm_override = CC.longActive or CS.out.stockAeb
          if apply_brake > 0:
            pcm_speed = 0.0
            self.new_accel = 0

          can_sends.append(hondacan.create_brake_command(self.packer, self.CAN, apply_brake, pump_on,
                                                         pcm_override, pcm_cancel_cmd, alert_fcw,
                                                         CS.stock_brake, self.CP_SP))
          if (apply_brake > 0) or (CS.out.gasPressed and not CS.car_gas_available):
            self.new_accel = 0

          if (self.apply_brake_last > 0) and (apply_brake == 0) and CC.longActive and (not self.launch_active):
            self.gas_recovery_ticks = 200

          if CS.out.vEgo < CS.out.cruiseState.speed - 2.:
            self.gasfactor_nomaxspeed = self.gasfactor
            self.gasfactor_low_nomaxspeed = self.gasfactor_low
            self.gas_alpha_nomaxspeed = self.gas_alpha
          else:
            self.gasfactor_nomaxspeed = min(self.gasfactor_nomaxspeed, self.gasfactor)
            self.gasfactor_low_nomaxspeed = min(self.gasfactor_low_nomaxspeed, self.gasfactor_low)
            self.gas_alpha_nomaxspeed = min(self.gas_alpha_nomaxspeed, self.gas_alpha)

          self.apply_brake_last = apply_brake
          self.brake = apply_brake / self.params.NIDEC_BRAKE_MAX

    speed_control = 0 if is_bosch else self.launch_active

    if (self.CP.flags & HondaFlags.BOSCH_CANFD) and CS.hud_tick and self.CP.openpilotLongitudinalControl and not CS.stock_acc_alive:
        can_sends.append(hondacan.create_acc_hud(self.packer, self.CAN.pt, self.CP, CC.enabled, pcm_speed, actuators.accel,
                                                 hud_control, hud_v_cruise, CS.is_metric, CS.acc_hud, speed_control,
                                                 self.CP.openpilotLongitudinalControl))

    if self.frame % 10 == 0:
      if self.CP.openpilotLongitudinalControl:
        if not (self.CP.flags & HondaFlags.BOSCH_CANFD):
          acc_hud_pcm_accel = self.new_accel if is_wire_gas else pcm_accel
          can_sends.append(hondacan.create_acc_hud(self.packer, self.CAN.pt, self.CP, CC.enabled, pcm_speed, acc_hud_pcm_accel,
                                                   hud_control, hud_v_cruise, CS.is_metric, CS.acc_hud, speed_control,
                                                   self.CP.openpilotLongitudinalControl))

      steering_available = CS.out.cruiseState.available and CS.out.vEgo > max(self.params.STEER_GLOBAL_MIN_SPEED, self.CP.minSteerSpeed)
      reduced_steering = CS.out.steeringPressed
      if is_bosch:
        steer_maxed = abs(apply_torque) >= self.params.STEER_MAX
      else:
        steer_maxed = (abs(apply_torque) >= self.params.STEER_MAX) or not CS.steer_control_active

      lkas_state_change = None
      if self.CP.flags & HondaFlags.BOSCH_CANFD:
        hud_key = (bool(CC.latActive), bool(self.dashed_lanes), bool(alert_steer_required), bool(CS.out.steerFaultPermanent))
        if hud_key != self.lkas_hud_key:
          self.lkas_hud_key = hud_key
          self.lkas_state_change_frames = 30
        lkas_state_change = self.lkas_state_change_frames > 0
        self.lkas_state_change_frames = max(0, self.lkas_state_change_frames - 1)

      can_sends.extend(hondacan.create_lkas_hud(self.packer, self.CAN.lkas, self.CP, hud_control, CC.latActive,
                                                steering_available, reduced_steering, alert_steer_required, CS.lkas_hud, self.dashed_lanes,
                                                steer_maxed, CS, lkas_state_change=lkas_state_change))

      if self.CP.openpilotLongitudinalControl:
        if is_bosch and not (self.CP.flags & HondaFlags.BOSCH_RADARLESS) and not (self.CP.flags & HondaFlags.BOSCH_CANFD):
          can_sends.append(hondacan.create_radar_hud(self.packer, self.CAN.pt))
        if self.CP.carFingerprint == CAR.HONDA_CIVIC_BOSCH:
          can_sends.append(hondacan.create_legacy_brake_command(self.packer, self.CAN.pt))
        if not is_bosch:
          self.speed = pcm_speed
          if is_wire_gas:
            self.gas = pcm_accel / self.params.NIDEC_GAS_MAX

    # Render OP's lane and lead car on the dash.
    if ((self.frame % 2 == 0 and self.CP.flags & HondaFlags.BOSCH_RADARLESS) or
        (CS.radar_50hz_tick and self.CP.flags & HondaFlags.BOSCH_CANFD and self.CP.openpilotLongitudinalControl
         and not CS.stock_acc_alive)):
      leads = hud_objects.leads_from_model(self.model, CS.out.vEgo)
      lead = leads[0]
      lead_d = lead.dRel if lead.status else 0.0
      self.dash_lane = self.lane_path_fitter.update(self.model, CS.out.vEgo, lead_d)
      if self.CP.flags & HondaFlags.BOSCH_CANFD:
        mux = self.radar_mux
        lane_offsets = lane_path.canfd_lane_offsets(self.dash_lane)
      else:
        mux = lane_path.MUX_CYCLE[(self.frame // 2) % len(lane_path.MUX_CYCLE)]
        lane_offsets = self.dash_lane.offsets
      lane_msg = lane_path.create_lane_path(self.packer, self.CAN.lkas, lane_offsets, mux)
      can_sends.append(lane_msg)

      tracks = CS.hud_object_tracker.snapshot() if CS.hud_object_tracker is not None else None
      if self.CP.openpilotLongitudinalControl:
        hud_msg = self.hud_object_author.create(self.packer, self.CAN.lkas, lead, tracks, mux, now_nanos * 1e-9,
                                                extra_leads=leads[1:])
      else:
        hud_msg = hud_objects.forward_hud_object(self.packer, self.CAN.lkas, mux, tracks)
      can_sends.append(hud_msg)

      if self.CP.flags & HondaFlags.BOSCH_CANFD:
        for addr, dat, _ in (lane_msg, hud_msg):
          can_sends.append((addr, dat, self.CAN.camera))

    if self.frame % 20 == 0 and self.CP.flags & HondaFlags.BOSCH_RADARLESS:
      dl = self.dash_lane
      can_sends.append(lane_path.create_lkas_hud_2(self.packer, self.CAN.lkas, (self.frame // 20 - 1) % 4,
                                                   dl.reach, dl.lane_cross, dl.left_line, dl.right_line))

    if self.CP.flags & (HondaFlags.BOSCH_RADARLESS | HondaFlags.BOSCH_CANFD) and CC.enabled and self.frame % 4 == 0 and \
        not pcm_cancel_cmd and not CC.cruiseControl.resume:
      if self.lkas_button_send_remaining == 0 and CS.lkas_hud["LKAS_READY"] and self.frame >= self.last_lkas_button_frame + 500:
        self.lkas_button_send_remaining = 3

      if self.lkas_button_send_remaining > 0:
        self.last_lkas_button_frame = self.frame
        self.lkas_button_send_remaining -= 1
        cruise_setting = CruiseSettings.LKAS
      elif CS.cruise_setting == CruiseSettings.LKAS:
        cruise_setting = 0
      else:
        cruise_setting = CS.cruise_setting

      can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CS.cruise_buttons, cruise_setting,
                                                     CS.scm_ambient_light, self.CP, bus=self.CAN.camera))

    can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer, self.frame,
                                                                       self.last_button_frame, self.CAN))

    new_actuators = actuators.as_builder()
    if is_bosch:
      new_actuators.speed = float(self.gasalpha)
      new_actuators.accel = self.accel
      new_actuators.gas = float(self.gasfactor)
      new_actuators.brake = float(self.windfactor)
      new_actuators.torqueOutputCan = apply_torque
    elif is_wire_gas:
      new_actuators.speed = float(self.nidec_pid_factor)
      new_actuators.accel = float(self.accel)
      new_actuators.gas = float(self.average_factor)
      new_actuators.brake = float(self.sat_accel)
      new_actuators.torqueOutputCan = float(self.speedfactor_low)
    else:
      new_actuators.speed = float(self.gasalpha)
      new_actuators.accel = self.accel
      new_actuators.gas = float(self.gasfactor)
      new_actuators.brake = float(self.windfactor)
      new_actuators.torqueOutputCan = apply_torque
    new_actuators.torque = self.last_torque

    if self.frame % 6000 == 0:
      if is_bosch:
        self.param_writer.put_many({
          "HondaGasAlphaParams": self.gasalpha,
          "HondaGasFactorParams": self.gasfactor,
          "HondaWindFactorParams": self.windfactor,
        })
      elif is_wire_gas:
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
      else:
        self.param_writer.put_many({
          "HondaGasAlphaParams": self.gasalpha,
          "HondaGasFactorParams": self.gasfactor,
          "HondaWindFactorParams": self.windfactor,
        })

    if self.frame % 12000 == 30 and is_wire_gas:
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

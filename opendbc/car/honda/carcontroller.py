import math
import threading
from queue import Empty, Queue

import numpy as np
from openpilot.common.params import Params
from opendbc.car.common.conversions import Conversions as CV

from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, rate_limit, make_tester_present_msg, structs
from opendbc.car.honda import hondacan
from opendbc.car.honda.values import CAR, CruiseButtons, HONDA_BOSCH, HONDA_BOSCH_CANFD, HONDA_BOSCH_RADARLESS, \
                                     HONDA_BOSCH_TJA_CONTROL, HONDA_NIDEC_ALT_PCM_ACCEL, CarControllerParams
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.common.pid import PIDController

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
  return np.clip(gb, 0.0, 1.0), np.clip(-gb, 0.0, 1.0), creep_impact


def compute_gas_brake(accel, speed, fingerprint):
  if fingerprint in HONDA_BOSCH:
    return compute_gb_honda_bosch(accel, speed)
  else:
    return compute_gb_honda_bosch(accel, speed)
    # return compute_gb_honda_nidec(accel, speed)


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
        self._params.put_nonblocking(key, value)


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)
    self.CAN = hondacan.CanBus(CP)
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

    self.nidec_pid_factor = 0.0
    self.brake_pid_factor = 0.0

    self.nidec_pid = PIDController(k_p=([0,], [0,]),
                                   k_i=([0., 5., 35.], [1.2, 0.8, 0.5]),
                                   k_f=1,
                                   pos_limit=0., # self.params.NIDEC_ACCEL_MAX,
                                   neg_limit=self.params.NIDEC_ACCEL_MIN)
    self.nidec_pid.reset()

    self.brake_pid = PIDController(k_p=([0,], [0,]),
                                   k_i=([0.], [2.0]),
                                   pos_limit=4.0,
                                   neg_limit=0,
                                   rate=50)
    self.brake_pid.reset()
    self.brake_pid_factor_non_lowspeed = 0.4 if (Params().get("HondaBrakePIDParams") is None) else Params().get("HondaBrakePIDParams")
    self.brake_pid.i = self.brake_pid_factor_non_lowspeed

    self.pitch = 0.0

    self.prior_gas_average = 0.0
    self.average_factor = 0.95 if (Params().get("HondaFeedForwardParams") is None) else Params().get("HondaFeedForwardParams")
    self.creep_factor = 1.0 if (Params().get("HondaCreepFactorParams") is None) else Params().get("HondaCreepFactorParams")
    self.creep_always = 0.0 if (Params().get("HondaCreepAlwaysParams") is None) else Params().get("HondaCreepAlwaysParams")
    self.gasfactor = 1.0 if (Params().get("HondaGasFactorParams") is None) else Params().get("HondaGasFactorParams")
    self.gasfactor_before_gasmax = self.gasfactor
    self.windfactor = 1.0 if (Params().get("HondaWindFactorParams") is None) else Params().get("HondaWindFactorParams")
    self.windfactor_before_gasmax = self.windfactor_before_brake = self.windfactor
    self.new_accel = 0.0

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
    actuators = CC.actuators
    hud_control = CC.hudControl
    hud_v_cruise = hud_control.setSpeed / CS.v_cruise_factor if hud_control.speedVisible else 255
    pcm_cancel_cmd = CC.cruiseControl.cancel

    if len(CC.orientationNED) == 3:
      self.pitch = CC.orientationNED[1]
    hill_brake = math.sin(self.pitch) * ACCELERATION_DUE_TO_GRAVITY

    # wind brake from air resistance decel at high speed
    wind_brake = np.interp(CS.out.vEgo, [0.0, 2.3, 35.0], [0.001, 0.002, 0.15]) * self.windfactor # not in m/s2 units

    if CC.longActive:
      if (actuators.longControlState == LongCtrlState.pid) and (not CS.out.stockAeb) and (not CS.out.gasPressed):

        # NIDECs don't allow acceleration near cruise_speed,
        # so limit limits of pid to prevent windup
        ACCEL_MAX_VALS = [self.params.NIDEC_ACCEL_MAX, 0.2]
        cruise_speed = CS.out.cruiseState.speed
        ACCEL_MAX_BP = [cruise_speed - 2., cruise_speed - .2]
        self.nidec_pid.pos_limit = np.interp(CS.out.vEgo, ACCEL_MAX_BP, ACCEL_MAX_VALS)

        self.nidec_pid_factor = self.nidec_pid.update(error = actuators.accel - CS.out.aEgo, speed = CS.out.vEgo)
        if (actuators.accel < 0.0):
          blendtarget = min(actuators.accel, self.nidec_pid.i) # force faster negative slope while hard braking
          self.nidec_pid.i = np.interp(actuators.accel, [0.0, -0.2], [self.nidec_pid.i, blendtarget])
        self.accel = self.nidec_pid_factor
        adjust_accel = self.accel + hill_brake + self.creep_always

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
        if (gas_pedal_force >= self.params.BOSCH_ACCEL_MAX) or (CS.out.vEgo >= CS.out.cruiseState.speed - 2.):
          # don't increase gasfactor nor windfactor at accel max, allow decreases
          self.gasfactor = min(self.gasfactor, self.gasfactor_before_gasmax)
          self.windfactor = min(self.windfactor, self.windfactor_before_gasmax)
        else:
          self.gasfactor_before_gasmax = self.gasfactor
          self.windfactor_before_gasmax = self.windfactor

      else:
        self.accel = actuators.accel
        adjust_accel = self.accel
        self.nidec_pid.reset()
        self.nidec_pid_factor = 0

      gas, brake, creep_impact = compute_gb_honda_nidec(adjust_accel, CS.out.vEgo, self.creep_factor)
      gas_error = self.accel - CS.out.aEgo
      if (actuators.longControlState == LongCtrlState.pid) and (not CS.out.stockAeb) and (not CS.out.gasPressed) \
             and (1e-5 <= CS.out.vEgo <= CS.out.cruiseState.speed - 2.):
        self.creep_factor = np.clip(self.creep_factor + 0.001 * creep_impact * gas_error, 0.0, 3.0)
        self.creep_always = np.clip(self.creep_always + 0.0001 * gas_error, -3.5, 0.0)
    else:
      self.accel = 0.0
      adjust_accel = self.accel
      gas, brake = 0.0, 0.0

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

    speed_val = int(CS.out.vEgo * CV.MS_TO_MPH / 5.0) * 5 + 100
    currentLatSpeed = f"{speed_val:02d}"
    if currentLatSpeed in self.latFactors:
      if not CS.out.steeringPressed and abs(limited_torque) > 0.9 and self.latFactors[currentLatSpeed] > abs(CS.out.steeringAngleDeg):
        self.latFactors[currentLatSpeed] /= 1.001
      if not CS.out.steeringPressed and abs(limited_torque) < 0.9 and self.latFactors[currentLatSpeed] < abs(CS.out.steeringAngleDeg):
        self.latFactors[currentLatSpeed] *= 1.001

    # Send CAN commands
    can_sends = []

    # tester present - w/ no response (keeps radar disabled)
    if self.CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS) and self.CP.openpilotLongitudinalControl:
      if self.frame % 10 == 0:
        can_sends.append(make_tester_present_msg(0x18DAB0F1, 1, suppress_response=True))

    # Send steering command.
    can_sends.append(hondacan.create_steering_control(self.packer, self.CAN, apply_torque, CC.latActive, self.tja_control))

    # all of this is only relevant for HONDA NIDEC
    max_accel = np.interp(CS.out.vEgo, self.params.NIDEC_MAX_ACCEL_BP, self.params.NIDEC_MAX_ACCEL_V)
    # TODO this 1.44 is just to maintain previous behavior
    pcm_speed_BP = [-wind_brake,
                    -wind_brake * (3 / 4),
                    0.0,
                    0.5]
    # The Honda ODYSSEY seems to have different PCM_ACCEL
    # msgs, is it other cars too?
    if not CC.longActive:
      pcm_speed = 0.0
      pcm_accel = int(0.0)
    elif self.CP.carFingerprint in HONDA_NIDEC_ALT_PCM_ACCEL:
      pcm_speed_V = [0.0,
                     np.clip(CS.out.vEgo - 3.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 0.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 5.0, 0.0, 100.0)]
      pcm_speed = float(np.interp(gas - brake, pcm_speed_BP, pcm_speed_V))
      pcm_accel = int(1.0 * self.params.NIDEC_GAS_MAX)
    else:
      pcm_speed_V = [0.0,
                     np.clip(CS.out.vEgo - 2.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 2.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 10.0, 0.0, 100.0)]
      pcm_speed = float(np.interp(gas - brake, pcm_speed_BP, pcm_speed_V))
      pcm_accel = int(np.clip((adjust_accel * self.gasfactor / 1.44) / max_accel, 0.0, 1.0) * self.params.NIDEC_GAS_MAX)

    # feedforward for Nidec decaying-average gas pedal
    max_increase = 20
    prior_accel = int(self.new_accel)
    self.new_accel = int((pcm_accel - self.prior_gas_average * (1 - self.average_factor)) / self.average_factor)
    self.new_accel = int(np.clip(self.new_accel, 0, min(prior_accel + max_increase, self.params.NIDEC_GAS_MAX)))
    self.prior_gas_average = self.prior_gas_average * (1 - self.average_factor) + (self.new_accel * self.average_factor)

    if self.CP.carFingerprint in HONDA_BOSCH:
      self.new_accel = pcm_accel
    elif (0 < self.new_accel < self.params.NIDEC_GAS_MAX) and (not CS.out.gasPressed) and (CS.out.vEgo <= CS.out.cruiseState.speed - 2.):
      gasfactor_error = (self.nidec_pid_factor - CS.out.aEgo)
      self.gasfactor *= (1 + 0.0001 * gasfactor_error)
      more_new_accel_needed = (self.new_accel > pcm_accel and self.nidec_pid_factor > CS.out.aEgo) or \
                              (self.new_accel < pcm_accel and self.nidec_pid_factor < CS.out.aEgo)
      new_accel_factor = abs(gasfactor_error * (self.new_accel - pcm_accel))
      if more_new_accel_needed:
        self.average_factor /= (1 + 0.0001 * new_accel_factor)
      else:
        self.average_factor = min(1.0, self.average_factor * (1 + 0.0001 * new_accel_factor))

    if not self.CP.openpilotLongitudinalControl:
      if self.frame % 2 == 0 and self.CP.carFingerprint not in HONDA_BOSCH_RADARLESS | HONDA_BOSCH_CANFD:
        can_sends.append(hondacan.create_bosch_supplemental_1(self.packer, self.CAN))
      # If using stock ACC, spam cancel command to kill gas when OP disengages.
      if pcm_cancel_cmd:
        can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CruiseButtons.CANCEL, self.CP.carFingerprint))
      elif CC.cruiseControl.resume:
        can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CruiseButtons.RES_ACCEL, self.CP.carFingerprint))

    else:
      # Send gas and brake commands.
      if self.frame % 2 == 0:
        ts = self.frame * DT_CTRL

        if self.CP.carFingerprint in HONDA_BOSCH:
          self.accel = float(np.clip(self.accel, self.params.BOSCH_ACCEL_MIN, self.params.BOSCH_ACCEL_MAX))
          self.gas = float(np.interp(self.accel, self.params.BOSCH_GAS_LOOKUP_BP, self.params.BOSCH_GAS_LOOKUP_V))

          stopping = actuators.longControlState == LongCtrlState.stopping
          self.stopping_counter = self.stopping_counter + 1 if stopping else 0
          can_sends.extend(hondacan.create_acc_commands(self.packer, self.CAN, CC.enabled, CC.longActive, self.accel, self.gas,
                                                        self.stopping_counter, self.CP.carFingerprint))
        else:
          apply_brake = np.clip(self.brake_last - wind_brake, 0.0, 1.0)
          if (apply_brake > 0) and (actuators.longControlState == LongCtrlState.pid) and (CS.out.vEgo > 0) and (not CS.out.stockAeb):
            self.brake_pid_factor = self.brake_pid.update(error = -(self.nidec_pid_factor - CS.out.aEgo)/apply_brake, speed = CS.out.vEgo)
          if (CS.out.vEgo >= 2): # save pid above 2m/s
            self.brake_pid_factor_non_lowspeed = self.brake_pid_factor
          if (CS.out.vEgo < 1e-3): # restore 2m/s pid after stopped
            self.brake_pid.i = self.brake_pid_factor_non_lowspeed
          brakefactor = 1 + self.brake_pid_factor
          apply_brake = int(np.clip(apply_brake * brakefactor * self.params.NIDEC_BRAKE_MAX, 0, self.params.NIDEC_BRAKE_MAX - 1))
          pump_on, self.last_pump_ts = brake_pump_hysteresis(apply_brake, self.apply_brake_last, self.last_pump_ts, ts)

          # limit brake release to 32 units per frame to match factory
          apply_brake = max(self.apply_brake_last - 32, apply_brake)

          pcm_override = CC.longActive or CS.out.stockAeb
          if apply_brake > 0: # prevent fault from concurrent gas + brake
            pcm_speed = 0.0
            self.new_accel = 0
          elif CS.out.gasPressed: # prevent fault from user gas with a pcm_gas of 198
            self.new_accel = 198

          can_sends.append(hondacan.create_brake_command(self.packer, self.CAN, apply_brake, pump_on,
                                                         pcm_override, pcm_cancel_cmd, alert_fcw,
                                                         self.CP, CS.stock_brake))
          self.apply_brake_last = apply_brake
          self.brake = apply_brake / self.params.NIDEC_BRAKE_MAX

    # Send dashboard UI commands.
    if self.frame % 10 == 0:
      if self.CP.openpilotLongitudinalControl:
        # On Nidec, this also controls longitudinal positive acceleration
        can_sends.append(hondacan.create_acc_hud(self.packer, self.CAN.pt, self.CP, CC.enabled, pcm_speed, self.new_accel,
                                                 hud_control, hud_v_cruise, CS.is_metric, CS.acc_hud))

      steering_available = CS.out.cruiseState.available and CS.out.vEgo > max(self.params.STEER_GLOBAL_MIN_SPEED, self.CP.minSteerSpeed)
      can_sends.extend(hondacan.create_lkas_hud(self.packer, self.CAN.lkas, self.CP, hud_control, CC.latActive,
                                                steering_available, alert_steer_required, CS.lkas_hud))

      if self.CP.openpilotLongitudinalControl:
        # TODO: combining with create_acc_hud block above will change message order and will need replay logs regenerated
        if self.CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS):
          can_sends.append(hondacan.create_radar_hud(self.packer, self.CAN.pt))
        if self.CP.carFingerprint == CAR.HONDA_CIVIC_BOSCH:
          can_sends.append(hondacan.create_legacy_brake_command(self.packer, self.CAN.pt))
        if self.CP.carFingerprint not in HONDA_BOSCH:
          self.speed = pcm_speed
          self.gas = pcm_accel / self.params.NIDEC_GAS_MAX

    new_actuators = actuators.as_builder()
    new_actuators.speed = float(self.nidec_pid_factor)
    new_actuators.accel = float(adjust_accel)
    # new_actuators.gas = float(self.gasfactor)
    new_actuators.gas = float(self.creep_always)
    new_actuators.brake = float(self.brake_pid_factor)
    new_actuators.torque = self.last_torque
    # new_actuators.torqueOutputCan = float(self.average_factor)
    new_actuators.torqueOutputCan = float(self.creep_factor)

    if self.frame % 6000 == 0:
      self.param_writer.put_many({
        "HondaFeedForwardParams": self.average_factor,
        "HondaBrakePIDParams": self.brake_pid_factor_non_lowspeed,
        "HondaCreepFactorParams": self.creep_factor,
        "HondaCreepAlwaysParams": self.creep_always,
        "HondaGasFactorParams": self.gasfactor,
        "HondaWindFactorParams": self.windfactor,
      })

    if self.frame % 12000 == 30:
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

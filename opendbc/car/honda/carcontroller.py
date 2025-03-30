import numpy as np
from collections import namedtuple

from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL, rate_limit, make_tester_present_msg, structs
from opendbc.car.honda import hondacan
from opendbc.car.honda.values import CruiseButtons, VISUAL_HUD, HONDA_BOSCH, HONDA_BOSCH_RADARLESS, CarControllerParams
from opendbc.car.interfaces import CarControllerBase

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


def compute_gb_honda_bosch(accel, speed):
  # TODO returns 0s, is unused
  return 0.0, 0.0


def compute_gb_honda_nidec(accel, speed):

  # ------------------ new scaling start-----------------------#

  if accel > 0:
    scale_factor = float ( np.interp ( speed, [0.0, 10.0, 20.0, 90.0], [10000.0, 25.0, 8.0, 1.0] ) )
    scaled_accel = accel * scale_factor
  else:
    scaled_accel = accel

  # ------------------ new scaling end -----------------------#

  creep_brake = 0.0
  creep_speed = 2.3
  creep_brake_value = 0.15
  if speed < creep_speed:
    creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
  gb = float(scaled_accel) / 4.8 - creep_brake
  return np.clip(gb, 0.0, 1.0), np.clip(-gb, 0.0, 1.0)


def compute_gas_brake(accel, speed, fingerprint):
  if fingerprint in HONDA_BOSCH:
    return compute_gb_honda_bosch(accel, speed)
  else:
    return compute_gb_honda_nidec(accel, speed)


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
  # initialize to no alert
  fcw_display = 0
  steer_required = 0
  acc_alert = 0

  # priority is: FCW, steer required, all others
  if hud_alert == VisualAlert.fcw:
    fcw_display = VISUAL_HUD[hud_alert.raw]
  elif hud_alert in (VisualAlert.steerRequired, VisualAlert.ldw):
    steer_required = VISUAL_HUD[hud_alert.raw]
  else:
    acc_alert = VISUAL_HUD[hud_alert.raw]

  return fcw_display, steer_required, acc_alert


HUDData = namedtuple("HUDData",
                     ["pcm_accel", "v_cruise", "lead_visible",
                      "lanes_visible", "fcw", "acc_alert", "steer_required", "lead_distance_bars"])


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)
    self.CAN = hondacan.CanBus(CP)

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
    self.blend_pcm_accel = 0.0
    self.blend_pcm_speed = 0.0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    conversion = hondacan.get_cruise_speed_conversion(self.CP.carFingerprint, CS.is_metric)
    hud_v_cruise = hud_control.setSpeed / conversion if hud_control.speedVisible else 255
    pcm_cancel_cmd = CC.cruiseControl.cancel

    # *** rate limit steer ***
    limited_torque = rate_limit(actuators.torque, self.last_torque, -self.params.STEER_DELTA_DOWN * DT_CTRL,
                                self.params.STEER_DELTA_UP * DT_CTRL)
    self.last_torque = limited_torque

    # steer torque is converted back to CAN reference (positive when steering right)
    apply_torque = int(np.interp(-limited_torque * self.params.STEER_MAX,
                                 self.params.STEER_LOOKUP_BP, self.params.STEER_LOOKUP_V))

    steerfactor = 200 if actuators.torque == 0 else abs ( self.params.STEER_MAX / max ( abs(actuators.torque), abs(apply_torque) ) )

    if CC.longActive:
      accel = float (np.clip ( actuators.accel, -100.0, np.interp (steerfactor, [ 0.0, 1.0], [-3.5, 3.5]) ) )
    else:
      accel = 0.0
      # gas, brake = 0.0, 0.0
      brake = 0.0

    # vehicle hud display, wait for one update from 10Hz 0x304 msg
    fcw_display, steer_required, acc_alert = process_hud_alert(hud_control.visualAlert)

    # **** process the car messages ****

    # Send CAN commands
    can_sends = []

    # tester present - w/ no response (keeps radar disabled)
    if self.CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS) and self.CP.openpilotLongitudinalControl:
      if self.frame % 10 == 0:
        can_sends.append(make_tester_present_msg(0x18DAB0F1, 1, suppress_response=True))

    # Send steering command.
    can_sends.append(hondacan.create_steering_control(self.packer, self.CAN, apply_torque, CC.latActive))

    if not CC.longActive:
      pcm_speed = 0.0
      pcm_accel = int(0.0)
    else:
      if accel > 0:
        if CS.out.vEgo < 0.02: # disengage standstill
          pcm_accel = 198.0
          pcm_speed = 10.0
          brake = 0.0
        else:
          pcm_accel = max ( 1, int ( accel * 200 ) )
          pcm_speed = CS.out.vEgo + accel * 3.0
          brake = 0.0
      else:
        if CS.out.vEgo < 0.02: # standstill
          pcm_accel = 0.0
          pcm_speed = 0.0
          brake = 140.0
        else:
          pcm_accel = 0.0
          pcm_speed = 0.0
          brake = max ( 1, int (accel * 40.0) )

      pcm_accel = float ( np.clip ( pcm_accel, 0, self.params.NIDEC_GAS_MAX - 1) )
      pcm_speed = float ( np.clip ( pcm_speed, 0, 100 ) )
      brake = float ( np.clip ( pcm_speed, 0, self.params.NIDEC_BRAKE_MAX ) )

    if not self.CP.openpilotLongitudinalControl:
      if self.frame % 2 == 0 and self.CP.carFingerprint not in HONDA_BOSCH_RADARLESS:  # radarless cars don't have supplemental message
        can_sends.append(hondacan.create_bosch_supplemental_1(self.packer, self.CAN))
      # If using stock ACC, spam cancel command to kill gas when OP disengages.
      if pcm_cancel_cmd:
        can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CruiseButtons.CANCEL, self.CP.carFingerprint))
      elif CC.cruiseControl.resume:
        can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CruiseButtons.RES_ACCEL, self.CP.carFingerprint))

    else:
      # Send gas and brake commands.
      if self.frame % 2 == 0:
        # ts = self.frame * DT_CTRL

        if self.CP.carFingerprint in HONDA_BOSCH:
          self.accel = float(np.clip(accel, self.params.BOSCH_ACCEL_MIN, self.params.BOSCH_ACCEL_MAX))
          self.gas = float(np.interp(accel, self.params.BOSCH_GAS_LOOKUP_BP, self.params.BOSCH_GAS_LOOKUP_V))

          stopping = actuators.longControlState == LongCtrlState.stopping
          self.stopping_counter = self.stopping_counter + 1 if stopping else 0
          can_sends.extend(hondacan.create_acc_commands(self.packer, self.CAN, CC.enabled, CC.longActive, self.accel, self.gas,
                                                        self.stopping_counter, self.CP.carFingerprint))
        else:
          pcm_override = True
          pump_on = True
          can_sends.append(hondacan.create_brake_command(self.packer, self.CAN, brake, pump_on,
                                                         pcm_override, pcm_cancel_cmd, fcw_display,
                                                         self.CP.carFingerprint, CS.stock_brake))

          self.brake = brake

    # Send dashboard UI commands.
    # On Nidec, this controls longitudinal positive acceleration
    if self.frame % 10 == 0:

      # ----------------- new test logic start ---------------------

#      if CC.longActive:

        # blending logic to fastforward, assume engine uses 98% of prior logic each frame
#        PERCENT_BLEND = 0.99
#
#        # pcm_accel = pcm_accel if self.accel <= 0 else float (np.clip ( ( pcm_accel - self.blend_pcm_accel * PERCENT_BLEND ) / ( 1 - PERCENT_BLEND ), \
#        #                                                               0, self.params.NIDEC_GAS_MAX ) )
#
#        self.blend_pcm_accel =  self.blend_pcm_accel * PERCENT_BLEND + pcm_accel * ( 1 - PERCENT_BLEND )
#
#        pcm_speed = pcm_speed if self.accel <= 0 else float (np.clip ( ( pcm_speed - self.blend_pcm_speed * PERCENT_BLEND ) / ( 1 - PERCENT_BLEND ), \
#                                                                     0, 100.0 ) )

        # reduce speed if above 50% steering max
#        pcm_speed = float ( np.clip ( pcm_speed, 0, 100 if CS.out.vEgo < 10.0 else steerfactor * 0.50 * CS.out.vEgo ) )

#        self.blend_pcm_speed =  self.blend_pcm_speed * PERCENT_BLEND + pcm_speed * ( 1 - PERCENT_BLEND )


      # ----------------- new test logic end ---------------------

      hud = HUDData(int(pcm_accel), int(round(hud_v_cruise)), hud_control.leadVisible,
                    hud_control.lanesVisible, fcw_display, acc_alert, steer_required, hud_control.leadDistanceBars)

      # pcm_speed_send = 0.0 if (self.brake_last > wind_brake ) and ( self.CP.carFingerprint in HONDA_NIDEC_HYBRID ) else int ( pcm_speed )
      can_sends.extend(hondacan.create_ui_commands(self.packer, self.CAN, self.CP, CC.enabled, pcm_speed, hud, CS.is_metric, CS.acc_hud, CS.lkas_hud))

      if self.CP.openpilotLongitudinalControl and self.CP.carFingerprint not in HONDA_BOSCH:
        self.speed = pcm_speed * 3.6 # conversion done in hondacan
        # self.gas = pcm_accel / self.params.NIDEC_GAS_MAX
        self.gas = pcm_accel

    new_actuators = actuators.as_builder()
    new_actuators.speed = self.speed
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas
    new_actuators.brake = self.brake
    new_actuators.torque = self.last_torque
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends

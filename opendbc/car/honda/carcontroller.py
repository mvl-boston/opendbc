import numpy as np
from collections import namedtuple
import math
from opendbc.car.common.conversions import Conversions as CV

from opendbc.can.packer import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, rate_limit, make_tester_present_msg, structs
from opendbc.car.honda import hondacan
from opendbc.car.honda.values import CruiseButtons, VISUAL_HUD, HONDA_BOSCH, HONDA_BOSCH_RADARLESS, \
                                     CarControllerParams
from opendbc.car.interfaces import CarControllerBase

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState




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
    self.pitch = 0.0
    self.calc_accel = 0.0
    self.man_step = 0
    self.last_time_frame = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    conversion = hondacan.get_cruise_speed_conversion(self.CP.carFingerprint, CS.is_metric)
    hud_v_cruise = hud_control.setSpeed / conversion if hud_control.speedVisible else 255
    pcm_cancel_cmd = CC.cruiseControl.cancel
    setgas = 0

    # *** rate limit steer ***
    limited_torque = rate_limit(actuators.torque, self.last_torque, -self.params.STEER_DELTA_DOWN * DT_CTRL,
                                self.params.STEER_DELTA_UP * DT_CTRL)
    self.last_torque = limited_torque

    # steer torque is converted back to CAN reference (positive when steering right)
    apply_torque = int(np.interp(-limited_torque * self.params.STEER_MAX,
                                 self.params.STEER_LOOKUP_BP, self.params.STEER_LOOKUP_V))


    if len(CC.orientationNED) == 3:
      self.pitch = CC.orientationNED[1]

    if CC.longActive:
      # accel = actuators.accel
# ----------------- test forced accel start -------------------
      accel = 0.0
      setgas = 30
      # plan: 10 / 30 / 50 / 100 / 150

      if self.man_step == 0:
        if CS.out.vEgo > 0.0:
          accel = -1.0 # -0.5
        else:
          self.last_time_frame = self.frame
          self.man_step = 1

      if self.man_step == 1:
        if self.frame < self.last_time_frame + 300: # 3 seconds
          accel = -2.0
        else:
          self.man_step = 2

      if self.man_step == 2:
        if CS.out.vEgo < 13.4: # 30 mph (1.0 * 7):
          accel = 1.0
        else:
          self.last_time_frame = self.frame
          self.man_step = 3

      if self.man_step == 3:
        if self.frame < self.last_time_frame + 300: # 3 seconds
          accel = 0.0
        else:
          self.man_step = 4

      if self.man_step == 4:
        if CS.out.vEgo > 0.0:
          accel = -1.0
        else:
          self.last_time_frame = self.frame
          self.man_step = 5

      if self.man_step == 5:
        if self.frame < self.last_time_frame + 300: # 3 seconds
          accel = -2.0
        else:
          self.man_step = 6

      if self.man_step == 6:
        if CS.out.vEgo <  13.4: # 30 mph (1.5 * 7):
          accel = 1.5
        else:
          self.last_time_frame = self.frame
          self.man_step = 7

      if self.man_step == 7:
        if self.frame < self.last_time_frame + 300: # 3 seconds
          accel = 0.0
        else:
          self.man_step = 8

      if self.man_step == 8:
        if CS.out.vEgo > 0.0:
          accel = -1.0 # 3.5
        else:
          self.last_time_frame = self.frame
          self.man_step = 9

      if self.man_step == 9:
        if self.frame < self.last_time_frame + 300: # 3 seconds
          accel = -2.0
        else:
          self.man_step = 10

      if self.man_step == 10:
        if CS.out.vEgo <  13.4: # 30 mph (2.0 * 7):
          accel = 2.0
        else:
          self.last_time_frame = self.frame
          self.man_step = 11

      if self.man_step == 11:
        if self.frame < self.last_time_frame + 300: # 3 seconds
          accel = 0.0
        else:
          self.man_step = 0
          accel = -0.5

# ----------------- test forced accel end -------------------
    else:
      accel = 0.0
      self.calc_accel = 0.0
      self.man_step = 0
      self.last_time_frame = 0

    speed_control = 1 if ( (self.calc_accel <= 0.0) and (CS.out.vEgo == 0) ) else 0

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
      self.calc_accel = 0.0

    else:

# ----------------- test override gas start -------------------
      wind_brake_ms2 = np.interp(CS.out.vEgo, [0.0, 13.4, 22.4, 31.3, 40.2], [0.000, 0.049, 0.136, 0.267, 0.441]) # in m/s2 units
      hill_brake = math.sin(self.pitch) * ACCELERATION_DUE_TO_GRAVITY
      hybrid_regen_brake = float(np.interp(CS.out.vEgo, [0.0,  0.1,  1.0,  2.5, 3.25, 4.2, 5.0, 6.0, 7.0, 7.7], \
                                                        [0.0, -1.7, -1.0, -0.5,  0.1, 0.1, 0.1, 0.3, 0.2, 0.0]))
#      hybrid_regen_brake = float(np.interp(CS.out.vEgo, [0.0, 1.0, 2.5, 3.25, 4.2, 5.0, 6.0, 7.0, 7.7, 10.8, 13.9, 17.0], \
#                                                        [0.6, 0.4, 0.9,  1.0, 1.0, 1.2, 1.4, 1.4, 1.4,  1.8,  1.6,  1.5]))

      self.calc_accel = float(accel + wind_brake_ms2 + hill_brake + hybrid_regen_brake)

      gas_accel_addon = np.interp(CS.out.vEgo, [0.0, 1.0, 1.8, 3.5, 5.0, 6.2, 7.2], [9.6, 9.0, 5.6, 1.2, 1.7, 1.6, 3.7])
      vfactor = np.interp(CS.out.vEgo, [0.0, 0.5, 1.5, 3.0, 100.0], [40.0, 40.0, 40.0, 40.0, 40.0])
      pcm_accel = 0 if self.calc_accel <= 0 else int (np.clip( (self.calc_accel + gas_accel_addon) * vfactor, 10, self.params.NIDEC_GAS_MAX -1) )
#      pcm_speed = max (0.0, CS.out.vEgo + float (np.clip ( self.calc_accel * 1000.0 * CV.KPH_TO_MS, -9.0, +41.0 ) ) )
      pcm_speed = float (np.clip ( CS.out.vEgo + self.calc_accel * 1000.0 * CV.KPH_TO_MS, 0.0, 144.9 ) )

      if speed_control == 1 and CC.longActive:
        pcm_accel = 198
# ----------------- test override gas end -------------------

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

        if self.CP.carFingerprint in HONDA_BOSCH:
          pass
        else:
# ------------------ brake override begin
          vfactorBrake = np.interp(CS.out.vEgo, [0.0, 3.9, 100.0], [-25.0, -40.0, -40.0])
          # vfactorBrake = float(np.interp(CS.out.vEgo, [0.0, 1.0, 2.5, 3.25, 4.2, 5.0, 6.0, 7.0, 7.7, 10.8, 13.9, 17.0], \
          #                                             [-80, -48, -61,  -53, -49, -67, -72, -57, -70, -100,  -81,  -81]))
          vAlphaBrake = -0.0
          apply_brake = 0 if (self.calc_accel >= 0) else int(np.clip( (self.calc_accel + vAlphaBrake) * vfactorBrake, 0, self.params.NIDEC_BRAKE_MAX - 1))
# ------------------ brake override end

          pcm_override = True
          pump_send = ( apply_brake > 0 )
          can_sends.append(hondacan.create_brake_command(self.packer, self.CAN, apply_brake, pump_send,
                                                         pcm_override, pcm_cancel_cmd, fcw_display,
                                                         self.CP.carFingerprint, CS.stock_brake))
          self.brake = apply_brake / self.params.NIDEC_BRAKE_MAX

    # Send dashboard UI commands.
    # On Nidec, this controls longitudinal positive acceleration
    if self.frame % 10 == 0:

      hud = HUDData(int(pcm_accel if pcm_accel <= 0 else setgas), int(round(hud_v_cruise)), hud_control.leadVisible,
                    hud_control.lanesVisible, fcw_display, acc_alert, steer_required, hud_control.leadDistanceBars)

      pcm_speed_send = int ( pcm_speed )
      can_sends.extend(hondacan.create_ui_commands(self.packer, self.CAN, self.CP, CC.enabled, pcm_speed_send, hud, CS.is_metric, CS.acc_hud, CS.lkas_hud, \
                                                  speed_control))

      if self.CP.openpilotLongitudinalControl and self.CP.carFingerprint not in HONDA_BOSCH:
        self.speed = pcm_speed * 3.6 # conversion done in hondacan
        self.gas = pcm_accel

    new_actuators = actuators.as_builder()
    new_actuators.speed = self.speed
    new_actuators.accel = accel
    new_actuators.gas = self.gas
    new_actuators.brake = self.brake
    new_actuators.torque = self.last_torque
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends

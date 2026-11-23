"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.honda.values import (HONDA_BOSCH, HONDA_BOSCH_RADARLESS, HONDA_BOSCH_CANFD, GearShifter)
from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP
from opendbc.car.common.conversions import Conversions as CV


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self._speed_limit_msgs_registered = False

  def _speed_bus(self, cp: CANParser, cp_cam: CANParser) -> CANParser:
    if self.CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS - HONDA_BOSCH_CANFD):
      return cp
    return cp_cam

  def _register_speed_limit_messages(self, speed_bus: CANParser) -> None:
    if self._speed_limit_msgs_registered:
      return
    messages = []
    if self.CP_SP.flags & HondaFlagsSP.METRIC_CAMERA_SPEED_LIMIT:
      messages = ["TSR_SPEED_LIMIT", "CAMERA_MESSAGES"]
    elif self.CP_SP.flags & HondaFlagsSP.HAS_CAMERA_MESSAGES:
      messages = ["CAMERA_MESSAGES"]
    for msg in messages:
      if msg not in speed_bus.dbc.name_to_msg:
        continue
      addr = speed_bus.dbc.name_to_msg[msg].address
      if addr not in speed_bus.message_states:
        speed_bus._add_message(msg, float('nan'))
    self._speed_limit_msgs_registered = True

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    speed_bus = self._speed_bus(cp, cp_cam)

    if self.CP_SP.flags & HondaFlagsSP.METRIC_CAMERA_SPEED_LIMIT:
      self._register_speed_limit_messages(speed_bus)
      if "TSR_SPEED_LIMIT" in speed_bus.dbc.name_to_msg:
        cam_limit = int(speed_bus.vl["TSR_SPEED_LIMIT"]["RECOGNIZED_LIMIT"])
        if cam_limit > 0:
          ret_sp.speedLimit = cam_limit * CV.KPH_TO_MS
        elif "CAMERA_MESSAGES" in speed_bus.dbc.name_to_msg:
          raw = int(speed_bus.vl["CAMERA_MESSAGES"]["SPEED_LIMIT_SIGN"])
          ret_sp.speedLimit = self._metric_speed_limit_from_sign(raw)
      elif "CAMERA_MESSAGES" in speed_bus.dbc.name_to_msg:
        raw = int(speed_bus.vl["CAMERA_MESSAGES"]["SPEED_LIMIT_SIGN"])
        ret_sp.speedLimit = self._metric_speed_limit_from_sign(raw)
    elif self.CP_SP.flags & HondaFlagsSP.HAS_CAMERA_MESSAGES:
      self._register_speed_limit_messages(speed_bus)
      speed_limit_raw = int(speed_bus.vl["CAMERA_MESSAGES"]["SPEED_LIMIT_SIGN"]) & 0x1F
      ret_sp.speedLimit = speed_limit_raw * 5.0 * CV.MPH_TO_MS if (1 <= speed_limit_raw <= 17) else 0.0

    if self.CP_SP.flags & HondaFlagsSP.NIDEC_HYBRID:
      ret.accFaulted = bool(cp.vl["HYBRID_BRAKE_ERROR"]["BRAKE_ERROR_1"] or cp.vl["HYBRID_BRAKE_ERROR"]["BRAKE_ERROR_2"])
      ret.stockAeb = bool(cp_cam.vl["BRAKE_COMMAND"]["AEB_REQ_1"] and cp_cam.vl["BRAKE_COMMAND"]["COMPUTER_BRAKE_HYBRID"] > 1e-5)
      ret.blockPcmEnable = ret.brakeHoldActive # Nidec Hybrids fault if resuming cruise from brake hold

    if self.CP_SP.flags & HondaFlagsSP.HYBRID_ALT_BRAKEHOLD:
      ret.brakeHoldActive = cp.vl["BRAKE_HOLD_HYBRID_ALT"]["BRAKE_HOLD_ACTIVE"] == 1
      ret.blockPcmEnable = ret.brakeHoldActive and not self.CP_SP.enableGasInterceptor

    if self.CP_SP.enableGasInterceptor:
      # Same threshold as panda, equivalent to 1e-5 with previous DBC scaling
      gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) // 2
      ret.gasPressed = gas > 492

    if ret.gearShifter == GearShifter.brake:
      ret.brakePressed = True # allows MADS in B (regen braking) mode

  @staticmethod
  def _imperial_speed_limit_m_s(raw: int) -> float:
    speed_limit_raw = raw & 0x1F
    return speed_limit_raw * 5.0 * CV.MPH_TO_MS if (1 <= speed_limit_raw <= 17) else 0.0

  @staticmethod
  def _metric_speed_limit_from_sign(raw: int) -> float:
    if raw in (0, 17, 61) or not (1 <= raw <= 25):
      return 0.0
    return raw * 10.0 * CV.KPH_TO_MS
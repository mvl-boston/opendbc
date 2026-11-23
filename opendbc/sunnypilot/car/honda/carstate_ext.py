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

# CAMERA_MESSAGES SPEED_LIMIT_SIGN on EU Honda: limit is encoded as (km/h / 10)
_METRIC_SIGN_NO_LIMIT = {0, 17, 61}


def imperial_speed_limit_from_sign(raw: int) -> float:
  speed_limit_raw = raw % 32
  return speed_limit_raw * 5.0 * CV.MPH_TO_MS if (1 <= speed_limit_raw <= 17) else 0.0


def metric_speed_limit_from_sign(raw: int) -> float:
  if raw in _METRIC_SIGN_NO_LIMIT:
    return 0.0
  if 1 <= raw <= 25:
    return raw * 10.0 * CV.KPH_TO_MS
  return 0.0


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

  @staticmethod
  def _camera_messages_on_pt(car_fingerprint: str) -> bool:
    return car_fingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS - HONDA_BOSCH_CANFD)

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    if self.CP_SP.flags & HondaFlagsSP.METRIC_CAMERA_SPEED_LIMIT:
      pt_dbc_names = cp_cam.dbc.name_to_msg
      if "TSR_SPEED_LIMIT" in pt_dbc_names:
        cam_limit = int(cp_cam.vl["TSR_SPEED_LIMIT"]["RECOGNIZED_LIMIT"])
        if cam_limit > 0:
          ret_sp.speedLimit = cam_limit * CV.KPH_TO_MS
        elif "CAMERA_MESSAGES" in pt_dbc_names:
          raw = int(cp_cam.vl["CAMERA_MESSAGES"]["SPEED_LIMIT_SIGN"])
          ret_sp.speedLimit = metric_speed_limit_from_sign(raw)
      elif "CAMERA_MESSAGES" in pt_dbc_names:
        raw = int(cp_cam.vl["CAMERA_MESSAGES"]["SPEED_LIMIT_SIGN"])
        ret_sp.speedLimit = metric_speed_limit_from_sign(raw)
    elif self.CP_SP.flags & HondaFlagsSP.HAS_CAMERA_MESSAGES:
      speed_bus = cp if self._camera_messages_on_pt(self.CP.carFingerprint) else cp_cam
      raw = int(speed_bus.vl["CAMERA_MESSAGES"]["SPEED_LIMIT_SIGN"])
      ret_sp.speedLimit = imperial_speed_limit_from_sign(raw)

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
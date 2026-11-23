"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.honda.values import GearShifter
from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP
from opendbc.car.common.conversions import Conversions as CV


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self._metric_cam_msgs_registered = False

  @staticmethod
  def _ensure_camera_messages(parser: CANParser) -> None:
    if "CAMERA_MESSAGES" not in parser.dbc.name_to_msg:
      return
    addr = parser.dbc.name_to_msg["CAMERA_MESSAGES"].address
    if addr not in parser.message_states:
      parser._add_message("CAMERA_MESSAGES", float('nan'))

  def _register_metric_cam_messages(self, cp_cam: CANParser) -> None:
    if self._metric_cam_msgs_registered or not (self.CP_SP.flags & HondaFlagsSP.METRIC_CAMERA_SPEED_LIMIT):
      return
    for msg in ("TSR_SPEED_LIMIT", "CAMERA_MESSAGES"):
      if msg in cp_cam.dbc.name_to_msg:
        addr = cp_cam.dbc.name_to_msg[msg].address
        if addr not in cp_cam.message_states:
          cp_cam._add_message(msg, float('nan'))
    self._metric_cam_msgs_registered = True

  @staticmethod
  def _imperial_speed_limit_m_s(raw: int) -> float:
    speed_limit_raw = raw & 0x1F
    return speed_limit_raw * 5.0 * CV.MPH_TO_MS if (1 <= speed_limit_raw <= 17) else 0.0

  def _imperial_from_parser(self, parser: CANParser) -> float:
    self._ensure_camera_messages(parser)
    if "CAMERA_MESSAGES" not in parser.vl:
      return 0.0
    raw = int(parser.vl["CAMERA_MESSAGES"]["SPEED_LIMIT_SIGN"])
    return self._imperial_speed_limit_m_s(raw)

  @staticmethod
  def _metric_speed_limit_from_sign(raw: int) -> float:
    if raw in (0, 17, 61) or not (1 <= raw <= 25):
      return 0.0
    return raw * 10.0 * CV.KPH_TO_MS

  def _metric_from_sign_parser(self, parser: CANParser) -> float:
    self._ensure_camera_messages(parser)
    if "CAMERA_MESSAGES" not in parser.vl:
      return 0.0
    raw = int(parser.vl["CAMERA_MESSAGES"]["SPEED_LIMIT_SIGN"])
    return self._metric_speed_limit_from_sign(raw)

  def _speed_limit_candidates_m_s(self, cp: CANParser, cp_cam: CANParser) -> list[float]:
    candidates: list[float] = []
    flags = self.CP_SP.flags
    pt_cam = (cp,) if (flags & HondaFlagsSP.CAMERA_MESSAGES_ON_PT) else ()
    cam_bus = (cp_cam,) if (flags & HondaFlagsSP.CAMERA_MESSAGES_ON_CAM) else ()
    camera_msg_parsers = (*cam_bus, *pt_cam)
    if not camera_msg_parsers:
      camera_msg_parsers = (cp_cam, cp)

    if flags & HondaFlagsSP.METRIC_CAMERA_SPEED_LIMIT:
      self._register_metric_cam_messages(cp_cam)
      if "TSR_SPEED_LIMIT" in cp_cam.dbc.name_to_msg:
        cam_limit = int(cp_cam.vl["TSR_SPEED_LIMIT"]["RECOGNIZED_LIMIT"])
        if cam_limit > 0:
          candidates.append(cam_limit * CV.KPH_TO_MS)
      for parser in camera_msg_parsers:
        limit = self._metric_from_sign_parser(parser)
        if limit > 0.0:
          candidates.append(limit)

    if flags & HondaFlagsSP.HAS_CAMERA_MESSAGES:
      for parser in camera_msg_parsers:
        limit = self._imperial_from_parser(parser)
        if limit > 0.0:
          candidates.append(limit)

    return candidates

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    if self.CP_SP.flags & (HondaFlagsSP.METRIC_CAMERA_SPEED_LIMIT | HondaFlagsSP.HAS_CAMERA_MESSAGES):
      candidates = self._speed_limit_candidates_m_s(cp, cp_cam)
      ret_sp.speedLimit = candidates[0] if candidates else 0.0

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
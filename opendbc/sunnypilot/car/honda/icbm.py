"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.honda import hondacan
from opendbc.car.honda.values import CruiseButtons
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BUTTONS = {
  SendButtonState.increase: CruiseButtons.RES_ACCEL,
  SendButtonState.decrease: CruiseButtons.DECEL_SET,
}


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  BUTTON_SEND_DURATION = 4  # frames to send button
  BUTTON_PAUSE_DURATION = 20  # frames to pause between sends

  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.current_button = SendButtonState.none
    self.button_send_frame = 0

  def update(self, CC_SP, packer, frame, last_button_frame, CAN) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    frames_elapsed = frame - self.button_send_frame
    total_cycle = self.BUTTON_SEND_DURATION + self.BUTTON_PAUSE_DURATION

    # Reset to pause state after send duration
    if frames_elapsed >= self.BUTTON_SEND_DURATION and self.current_button != SendButtonState.none:
      self.current_button = SendButtonState.none
    # Start new tap if button requested and cycle complete
    if self.ICBM.sendButton != SendButtonState.none and self.current_button == SendButtonState.none and frames_elapsed >= total_cycle:
      self.current_button = self.ICBM.sendButton
      self.button_send_frame = frame

    # Send button if currently in send phase
    if self.current_button != SendButtonState.none and (frame - self.button_send_frame) < self.BUTTON_SEND_DURATION:
      send_button = BUTTONS[self.current_button]
      can_sends.append(hondacan.spam_buttons_command(packer, CAN, send_button, 0, self.CP.carFingerprint))

    return can_sends

"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car import create_gas_interceptor_command

# Pedal rise-rate limit, in fraction of full pedal travel per second, by speed. The interceptor
# drives the pedal directly, so a step in the accel command lands on the throttle as a step.
# At creep speed on a dual-clutch/torque-converter car that flares the engine before the clutch
# takes up and the car lurches when it does: ACURA_ILX route 2752303cce1f0aba|0000000c seg 24,
# a 0.8 -> 2.0 m/s2 command step at 1.1 m/s put the wire 0.04 -> 0.23 in 0.3 s, rpm 750 -> 2460,
# and the car hit 4.7 m/s2 against the 2.0 command (openpilot's excessiveActuation check fires at
# 4.0 for 0.25 s and latches until the next ignition). The launch two minutes earlier ramped the
# wire at ~0.14/s and the engine came up smoothly; the stock Honda PCM applies pedal at roughly
# 0.1/s. Decreases are never limited. The limit loosens with speed: once the driveline is locked
# a fast tip-in is not a lurch, and a highway pass should not wait on it.
GAS_RISE_RATE_BP = [0., 5., 15.]  # m/s
GAS_RISE_RATE_V = [0.15, 0.25, 0.5]  # fraction of full pedal per second
# The pedal -> accel response lags ~0.5 s on this platform. For that long after the wire reaches
# its target (or after any window where the rise limit held it below the target) the tracking
# error is the plant's lag, not evidence about the gas gain, so the gain learners must not see it.
GAS_SETTLE_FRAMES = 25  # 50 Hz frames


class GasInterceptorCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.gas = 0.
    self.interceptor_gas_cmd = 0.
    self.gas_settled_frames = 0
    # True once the wire has been at its (unlimited) target for GAS_SETTLE_FRAMES; the gas gain
    # learners in the car controller gate on this
    self.interceptor_gas_settled = False

  def update(self, CC: structs.CarControl, CS: structs.CarState, gas: float, brake: float, wind_brake: float,
             packer, frame: int) -> list[CanData]:
    can_sends = []

    if self.CP_SP.enableGasInterceptor:
      # way too aggressive at low speed without this
      gas_mult = np.interp(CS.out.vEgo, [0., 10.], [0.4, 1.0])
      # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
      # This prevents unexpected pedal range rescaling
      # Sending non-zero gas when OP is not enabled will cause the PCM not to respond to throttle as expected
      # when you do enable.
      rise_limited = False
      if CC.longActive:
        gas_target = float(np.clip(gas_mult * (gas - brake + wind_brake * 3 / 4), 0., 1.))
        # this runs on even frames only (50 Hz), so one call spans two control ticks
        max_rise = float(np.interp(CS.out.vEgo, GAS_RISE_RATE_BP, GAS_RISE_RATE_V)) * 2 * DT_CTRL
        rise_limited = gas_target > self.gas + max_rise
        self.gas = min(gas_target, self.gas + max_rise)
      else:
        self.gas = 0.0
      self.gas_settled_frames = 0 if (rise_limited or not CC.longActive) else self.gas_settled_frames + 1
      self.interceptor_gas_settled = self.gas_settled_frames > GAS_SETTLE_FRAMES
      can_sends.append(create_gas_interceptor_command(packer, self.gas, frame // 2))

    return can_sends

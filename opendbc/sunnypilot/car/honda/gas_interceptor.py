"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car import create_gas_interceptor_command

# Pedal increase-per-tick limit. The interceptor drives the pedal directly, so a step in the accel
# command lands on the throttle as a step. At creep speed that flares the engine before the
# driveline takes up and the car lurches when it does: ACURA_ILX route 2752303cce1f0aba|0000000c
# seg 24, a 0.8 -> 2.0 m/s2 command step at 1.1 m/s put the wire 0.04 -> 0.23 in 0.3 s, rpm
# 750 -> 2460, and the car hit 4.7 m/s2 against the 2.0 command (openpilot's excessiveActuation
# check fires at 4.0 for 0.25 s and latches until the next ignition). Replaying that event through
# a plant model fitted on it: the peak passes below 4.0 only at <= 0.008/tick and reaches 3.7 at
# 0.004/tick (the launch two minutes earlier, which came up smoothly, ramped at ~0.003/tick).
# Decreases are never limited. Above city speed the driveline is locked and a tip-in is not a
# lurch, so the limit opens to 0.010/tick (0.5 pedal/s) so a highway pass does not wait on it.
GAS_RISE_PER_TICK_BP = [10., 15.]  # m/s
GAS_RISE_PER_TICK_V = [0.004, 0.010]  # fraction of full pedal per 50 Hz tick (0.2/s, 0.5/s)
# The pedal -> accel response lags ~0.5 s on this platform. While the limit is holding the wire
# below its target, and for this long afterwards, the tracking error is the plant's lag and
# overshoot, not evidence about the gas gain, so the gain learner must not see it.
GAS_LEARN_HOLD_TICKS = 25  # 50 Hz ticks


class GasInterceptorCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.gas = 0.
    self.interceptor_gas_cmd = 0.
    self.gas_learn_hold = 0
    # True while the rise limit is active or within GAS_LEARN_HOLD_TICKS after it released; the
    # gas gain learner in the car controller gates on this
    self.interceptor_gas_learn_paused = False

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
        max_rise = float(np.interp(CS.out.vEgo, GAS_RISE_PER_TICK_BP, GAS_RISE_PER_TICK_V))
        rise_limited = gas_target > self.gas + max_rise
        self.gas = min(gas_target, self.gas + max_rise)
      else:
        self.gas = 0.0
      self.gas_learn_hold = GAS_LEARN_HOLD_TICKS if (rise_limited or not CC.longActive) else max(0, self.gas_learn_hold - 1)
      self.interceptor_gas_learn_paused = self.gas_learn_hold > 0
      can_sends.append(create_gas_interceptor_command(packer, self.gas, frame // 2))

    return can_sends

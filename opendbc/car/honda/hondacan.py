from opendbc.car import CanBusBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.honda.values import HondaFlags, CarControllerParams

# CAN bus layout with relay
# 0 = ACC-CAN - radar side
# 1 = F-CAN B - powertrain
# 2 = ACC-CAN - camera side
# 3 = F-CAN A - OBDII port


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    # use fingerprint if specified
    super().__init__(CP if fingerprint is None else None, fingerprint)

    # powertrain bus is split instead of radar on radarless and CAN FD Bosch
    if CP is not None and CP.flags & HondaFlags.BOSCH and not (CP.flags & (HondaFlags.BOSCH_RADARLESS | HondaFlags.BOSCH_CANFD)):
      self._pt, self._radar = self.offset + 1, self.offset
      # normally steering commands are sent to radar, which forwards them to powertrain bus
      # when radar is disabled, steering commands are sent directly to powertrain bus
      self._lkas = self._pt if CP.openpilotLongitudinalControl else self._radar
    else:
      self._pt, self._radar, self._lkas = self.offset, self.offset + 1, self.offset

  @property
  def pt(self) -> int:
    return self._pt

  @property
  def radar(self) -> int:
    return self._radar

  @property
  def camera(self) -> int:
    return self.offset + 2

  @property
  def lkas(self) -> int:
    return self._lkas

  # B-CAN is forwarded to ACC-CAN radar side (CAN 0 on fake ethernet port)
  @property
  def body(self) -> int:
    return self.offset


def create_brake_command(packer, CAN, apply_brake, pump_on, pcm_override, pcm_cancel_cmd, fcw, stock_brake):
  # TODO: do we loose pressure if we keep pump off for long?
  brakelights = apply_brake > 0
  brake_rq = apply_brake > 0
  pcm_fault_cmd = False

  values = {
    "COMPUTER_BRAKE": apply_brake,
    "BRAKE_PUMP_REQUEST": pump_on,
    "CRUISE_OVERRIDE": pcm_override,
    "CRUISE_FAULT_CMD": pcm_fault_cmd,
    "CRUISE_CANCEL_CMD": pcm_cancel_cmd,
    "COMPUTER_BRAKE_REQUEST": brake_rq,
    "SET_ME_1": 1,
    "BRAKE_LIGHTS": brakelights,
    "CHIME": stock_brake["CHIME"] if fcw else 0,  # send the chime for stock fcw
    "FCW": fcw << 1,  # TODO: Why are there two bits for fcw?
    "AEB_REQ_1": 0,
    "AEB_REQ_2": 0,
    "AEB_STATUS": 0,
  }
  return packer.make_can_msg("BRAKE_COMMAND", CAN.pt, values)


def create_acc_commands(packer, CAN, enabled, active, accel, gas, stopping_counter, CP):
  commands = []
  min_gas_accel = CarControllerParams.BOSCH_GAS_LOOKUP_BP[0]

  control_on = 5 if enabled else 0
  gas_command = gas if active and accel > min_gas_accel else -30000
  accel_command = accel if active else 0
  braking = 1 if active and accel < min_gas_accel else 0
  standstill = 1 if active and stopping_counter > 0 else 0
  standstill_release = 1 if active and stopping_counter == 0 else 0

  # common ACC_CONTROL values
  acc_control_values = {
    'ACCEL_COMMAND': accel_command,
    'STANDSTILL': standstill,
  }

  if CP.flags & HondaFlags.BOSCH_RADARLESS:
    acc_control_values.update({
      "CONTROL_ON": enabled,
      "IDLESTOP_ALLOW": stopping_counter > 200,  # allow idle stop after 4 seconds (50 Hz)
    })
  else:
    acc_control_values.update({
      # setting CONTROL_ON causes car to set POWERTRAIN_DATA->ACC_STATUS = 1
      "CONTROL_ON": control_on,
      "GAS_COMMAND": gas_command,  # used for gas
      "BRAKE_LIGHTS": braking,
      "BRAKE_REQUEST": braking,
      "STANDSTILL_RELEASE": standstill_release,
    })
    acc_control_on_values = {
      "SET_TO_3": 0x03,
      "CONTROL_ON": enabled,
      "SET_TO_FF": 0xff,
      "SET_TO_75": 0x75,
      "SET_TO_30": 0x30,
    }
    commands.append(packer.make_can_msg("ACC_CONTROL_ON", CAN.pt, acc_control_on_values))

  commands.append(packer.make_can_msg("ACC_CONTROL", CAN.pt, acc_control_values))
  return commands


def create_steering_control(packer, CAN, apply_torque, lkas_active, tja_control):
  values = {
    "STEER_TORQUE": apply_torque if lkas_active else 0,
    "STEER_TORQUE_REQUEST": lkas_active,
  }

  if tja_control:
    values["STEER_DOWN_TO_ZERO"] = lkas_active

  return packer.make_can_msg("STEERING_CONTROL", CAN.lkas, values)


def create_bosch_supplemental_1(packer, CAN):
  # non-active params
  values = {
    "SET_ME_X04": 0x04,
    "SET_ME_X80": 0x80,
    "SET_ME_X10": 0x10,
  }
  return packer.make_can_msg("BOSCH_SUPPLEMENTAL_1", CAN.lkas, values)


def create_acc_hud(packer, bus, CP, enabled, pcm_speed, pcm_accel, hud_control, hud_v_cruise, is_metric, acc_hud):
  acc_hud_values = {
    'CRUISE_SPEED': hud_v_cruise,
    'ENABLE_MINI_CAR': 1 if enabled else 0,
    # only moves the lead car without ACC_ON
    'HUD_DISTANCE': hud_control.leadDistanceBars,  # wraps to 0 at 4 bars
    'IMPERIAL_UNIT': int(not is_metric),
    'HUD_LEAD': 2 if enabled and hud_control.leadVisible else 1 if enabled else 0,
    'SET_ME_X01_2': 1,
  }

  if CP.flags & HondaFlags.BOSCH:
    acc_hud_values['ACC_ON'] = int(enabled)
    acc_hud_values['FCM_OFF'] = 1
    acc_hud_values['FCM_OFF_2'] = 1
  else:
    # Shows the distance bars, TODO: stock camera shows updates temporarily while disabled
    acc_hud_values['ACC_ON'] = int(enabled)
    acc_hud_values['PCM_SPEED'] = pcm_speed * CV.MS_TO_KPH
    acc_hud_values['PCM_GAS'] = pcm_accel
    acc_hud_values['SET_ME_X01'] = 1
    acc_hud_values['FCM_OFF'] = acc_hud['FCM_OFF']
    acc_hud_values['FCM_OFF_2'] = acc_hud['FCM_OFF_2']
    acc_hud_values['FCM_PROBLEM'] = acc_hud['FCM_PROBLEM']
    acc_hud_values['ICONS'] = acc_hud['ICONS']

  return packer.make_can_msg("ACC_HUD", bus, acc_hud_values)


def create_lkas_hud(packer, bus, CP, hud_control, lat_active, steering_available, alert_steer_required, lkas_hud):
  commands = []

  lkas_hud_values = {
    'LKAS_READY': 1,
    'LKAS_STATE_CHANGE': 1,
    'STEERING_REQUIRED': alert_steer_required,
    'SOLID_LANES': hud_control.lanesVisible,
    'BEEP': 0,
  }

  if CP.flags & (HondaFlags.BOSCH_RADARLESS | HondaFlags.BOSCH_CANFD):
    lkas_hud_values['LANE_LINES'] = 3
    lkas_hud_values['DASHED_LANES'] = hud_control.lanesVisible

    # car likely needs to see LKAS_PROBLEM fall within a specific time frame, so forward from camera
    # TODO: needed for Bosch CAN FD?
    if CP.flags & HondaFlags.BOSCH_RADARLESS:
      lkas_hud_values['LKAS_PROBLEM'] = lkas_hud['LKAS_PROBLEM']

  if not (CP.flags & HondaFlags.BOSCH_EXT_HUD):
    lkas_hud_values['RDM_OFF'] = 1
    lkas_hud_values['LANE_ASSIST_BEEP_OFF'] = 1

  # New HUD concept for selected Bosch cars, overwrites some of the above
  # TODO: make global across all Honda if feedback is favorable
  if CP.flags & HondaFlags.BOSCH_ALT_RADAR:
    lkas_hud_values['DASHED_LANES'] = steering_available
    lkas_hud_values['SOLID_LANES'] = lat_active

  if CP.flags & HondaFlags.BOSCH_EXT_HUD and not CP.openpilotLongitudinalControl:
    commands.append(packer.make_can_msg('LKAS_HUD_A', bus, lkas_hud_values))
    commands.append(packer.make_can_msg('LKAS_HUD_B', bus, lkas_hud_values))
  else:
    commands.append(packer.make_can_msg('LKAS_HUD', bus, lkas_hud_values))

  return commands


def create_radar_hud(packer, bus):
  radar_hud_values = {
    'CMBS_OFF': 0x01,
    'SET_TO_1': 0x01,
  }

  return packer.make_can_msg('RADAR_HUD', bus, radar_hud_values)


def create_legacy_brake_command(packer, bus):
  return packer.make_can_msg("LEGACY_BRAKE_COMMAND", bus, {})


def spam_buttons_command(packer, CAN, button_val, CP):
  values = {
    'CRUISE_BUTTONS': button_val,
    'CRUISE_SETTING': 0,
  }
  # send buttons to camera on radarless (camera does ACC) cars
  bus = CAN.camera if CP.flags & HondaFlags.BOSCH_RADARLESS else CAN.pt
  return packer.make_can_msg("SCM_BUTTONS", bus, values)


# --- Experimental lead spoof -------------------------------------------------------------------
# The Bosch-A camera streams a 16-slot forward-object bank (slot 0 = nearest object) that the radar
# fuses for its low-speed Traffic Jam Assist steering-relay lead gate. When the camera reports no
# lead, the radar drops the relay and openpilot can no longer steer below ~45 mph. To keep steering
# available we inject a phantom slot-0 object toward the radar. We start from real captured camera
# frames so every unknown/reserved bit matches a genuine object, then override only range, azimuth,
# relative velocity, the rolling frame index and the lifecycle counter. The phantom is placed far
# away (50 m) with zero relative velocity so it should not induce stock-ACC gas/brake response.
# Object layout/scaling: opendbc PR mvl-boston/opendbc#669.
LEAD_SPOOF_MSGS = (0x280, 0x281, 0x282, 0x283, 0x2C8)
LEAD_SPOOF_MAX_SPEED = 20.  # m/s (~72 kph); above the relay band the radar relays unconditionally

_LEAD_SPOOF_TEMPLATES = {
  0x280: bytes.fromhex("047422ab94c00238"),  # F0: STATUS/RANGE/AZIMUTH
  0x281: bytes.fromhex("00020db67cfefe3d"),  # F1: existence probability
  0x282: bytes.fromhex("10bb80007d7e25b2"),  # F2: lifecycle
  0x283: bytes.fromhex("9bb68de080c0113a"),  # F3: track id / azimuth edges
  0x2C8: bytes.fromhex("7ef6164081007bf3"),  # AUX: relative velocity / range ratio
}
_LEAD_SPOOF_FRAME_IDX_BIT = {0x280: 27, 0x281: 28, 0x282: 11, 0x283: 12, 0x2C8: 12}
_LEAD_SPOOF_RANGE_M = 50.
_LEAD_SPOOF_RANGE_RAW = round((_LEAD_SPOOF_RANGE_M + 3.0) / 0.05712)  # RANGE = 0.05712*raw - 3.0
_LEAD_SPOOF_AZIMUTH_CENTER = 1024  # straight ahead
_LEAD_SPOOF_VREL_CENTER = 864      # zero relative velocity (scale 1/64 m/s)


def _set_be(dat: bytearray, start: int, length: int, value: int) -> None:
  # Set a big-endian (Motorola, @0) DBC signal by its start bit and length.
  bp = start
  for k in range(length):
    b = (value >> (length - 1 - k)) & 1
    byte, off = bp // 8, bp % 8
    dat[byte] = (dat[byte] & ~(1 << off)) | (b << off)
    bp = bp + 15 if bp % 8 == 0 else bp - 1


def create_lead_spoof(CAN, frame_idx: int, lifecycle: int):
  msgs = []
  for addr in LEAD_SPOOF_MSGS:
    dat = bytearray(_LEAD_SPOOF_TEMPLATES[addr])
    _set_be(dat, _LEAD_SPOOF_FRAME_IDX_BIT[addr], 4, frame_idx & 0xF)
    if addr == 0x280:
      _set_be(dat, 15, 4, 7)                            # STATUS: valid object (not 0xF)
      _set_be(dat, 23, 12, _LEAD_SPOOF_RANGE_RAW)       # RANGE_RAW
      _set_be(dat, 39, 11, _LEAD_SPOOF_AZIMUTH_CENTER)  # AZIMUTH_RAW: straight ahead
    elif addr == 0x282:
      _set_be(dat, 7, 12, lifecycle & 0xFFF)            # LIFECYCLE_RAW (+2 per sweep)
    elif addr == 0x2C8:
      _set_be(dat, 7, 11, _LEAD_SPOOF_VREL_CENTER)      # REL_VELOCITY_RAW: 0 m/s
    msgs.append((addr, bytes(dat), CAN.radar))
  return msgs


def honda_checksum(address: int, sig, d: bytearray) -> int:
  s = 0
  extended = address > 0x7FF
  addr = address
  while addr:
    s += addr & 0xF
    addr >>= 4
  for i in range(len(d)):
    x = d[i]
    if i == len(d) - 1:
      x >>= 4
    s += (x & 0xF) + (x >> 4)
  s = 8 - s
  if extended:
    s += 3
  return s & 0xF

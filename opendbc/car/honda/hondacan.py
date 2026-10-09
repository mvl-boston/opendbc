import numpy as np

from opendbc.car import CanBusBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.honda.values import (HondaFlags, HONDA_BOSCH, HONDA_BOSCH_RADARLESS,
                                      HONDA_BOSCH_CANFD, HONDA_BOSCH_VISION_CTRL)

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
    if CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS - HONDA_BOSCH_CANFD):
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


def create_brake_command(packer, CAN, apply_brake, pump_on, pcm_override, pcm_cancel_cmd, fcw, CP, stock_brake):
  # TODO: do we loose pressure if we keep pump off for long?
  brakelights = apply_brake > 0
  brake_rq = apply_brake > 0
  pcm_fault_cmd = False

  values = {
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
  if (CP.flags & HondaFlags.HYBRID):
    values.update({
      "COMPUTER_BRAKE_HYBRID": apply_brake,
      "BRAKE_PUMP_REQUEST_HYBRID": (apply_brake > 0),
    })
  else:
    values.update({
      "COMPUTER_BRAKE": apply_brake,
      "BRAKE_PUMP_REQUEST": pump_on,
    })
  return packer.make_can_msg("BRAKE_COMMAND", CAN.pt, values)


def create_acc_commands(packer, CAN, enabled, active, accel, gas, stopping_counter, CP, gas_force, park_or_reverse=False):

  commands = []

  control_on = 5 if enabled else 0
  gas_command = gas if active and gas_force > 0 else -30000
  accel_command = accel if active else 0
  braking = 1 if active and gas_force < 0 else 0
  standstill = 1 if active and stopping_counter > 0 else 0
  standstill_release = 1 if active and stopping_counter == 0 else 0

  # common ACC_CONTROL values
  acc_control_values = {
    'ACCEL_COMMAND': accel_command,
    'STANDSTILL': standstill,
  }

  # Vision controller cars (EU CR-V, MDX Type S) have the CAN FD body but the brake module listens to the
  # radarless-style ACC_CONTROL (0x1C8): with only the radar-style 0x1DF on the bus it latched CRUISE_FAULT
  # 0.28 s after the controller was silenced (route ad9840558640c31d/00000009--b2e159e05d)
  if CP.carFingerprint in (HONDA_BOSCH_RADARLESS | HONDA_BOSCH_VISION_CTRL):
    # required whenever braking for Hybrid and Bosch Alt Brake vehicles, allow idle stop after 4 seconds (50 Hz) for other vehicles
    brake_assist = braking if CP.flags & (HondaFlags.HYBRID | HondaFlags.BOSCH_ALT_BRAKE) else stopping_counter > 200
    acc_control_values.update({
      "CONTROL_ON": enabled,
      "COMPUTER_BRAKE_ASSIST": brake_assist,
    })
    if CP.carFingerprint in HONDA_BOSCH_VISION_CTRL:
      # Match the silenced controller's idle frame byte for byte: the stock MDX Type S RVU keeps bit 8 set whenever
      # it is not controlling (idle stop allowed) and bits 20/23 set in P and R (00 01 90 .. parked, 00 01 00 .. in D
      # unengaged, 00 04 00 .. engaged). On route ad9840558640c31d/0000000a--cffee2dde2 the brake module latched
      # CRUISE_FAULT 0.34 s after the switchover even though our 0x1C8 was flowing, and ours read 00 00 00 ..
      acc_control_values.update({
        "COMPUTER_BRAKE_ASSIST": brake_assist or not enabled,
        "BOH": park_or_reverse,
        "BOH_2": park_or_reverse,
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


def _vision_ctrl_idle_steering_bytes(packer, dat: bytes) -> bytes:
  """Stock MDX Type S RVU idle STEERING_CONTROL is 00 00 40 00 .. (route ad9840558640c31d/00000003); the Bosch
  DBC repacks that constant byte away when torque is zero, but the checksum still validates on the stock bytes."""
  from opendbc.can.packer import set_value
  d = bytearray(dat)
  d[2] = 0x40
  cs_sig = packer.dbc.addr_to_msg[0xE4].sigs['CHECKSUM']
  set_value(d, cs_sig, 0)
  set_value(d, cs_sig, honda_checksum(0xE4, cs_sig, d))
  return bytes(d)


def create_steering_control(packer, CAN, apply_torque, lkas_active, tja_control, vision_ctrl=False):
  values = {
    "STEER_TORQUE": apply_torque if lkas_active else 0,
    "STEER_TORQUE_REQUEST": lkas_active,
  }

  if tja_control:
    values["STEER_DOWN_TO_ZERO"] = lkas_active

  addr, dat, bus = packer.make_can_msg("STEERING_CONTROL", CAN.lkas, values)
  if vision_ctrl and not lkas_active and apply_torque == 0:
    dat = _vision_ctrl_idle_steering_bytes(packer, dat)
  return addr, dat, bus


def create_bosch_supplemental_1(packer, CAN):
  # non-active params
  values = {
    "SET_ME_X04": 0x04,
    "SET_ME_X80": 0x80,
    "SET_ME_X10": 0x10,
  }
  return packer.make_can_msg("BOSCH_SUPPLEMENTAL_1", CAN.lkas, values)


def create_acc_hud(packer, bus, CP, enabled, pcm_speed, pcm_accel, hud_control, hud_v_cruise, is_metric, acc_hud, speed_control,
                   alphalong):
  acc_hud_values = {
    'CRUISE_SPEED': hud_v_cruise,
    'ENABLE_MINI_CAR': 1 if enabled else 0,
    # only moves the lead car without ACC_ON
    'HUD_DISTANCE': hud_control.leadDistanceBars,  # wraps to 0 at 4 bars
    'IMPERIAL_UNIT': int(not is_metric),
    'HUD_LEAD': 2 if enabled and hud_control.leadVisible else 1 if enabled else 0,
    'SET_ME_X01_2': 1,
  }

  if CP.carFingerprint in HONDA_BOSCH_VISION_CTRL:
    # the MDX Type S controller holds bit 55 and never sets bit 48 (idle 00 00 00 ff 00 c0 c0, engaged .. d0 d0)
    acc_hud_values['SET_ME_X01'] = 0
    acc_hud_values['SET_ME_X01_2'] = 1
  elif CP.carFingerprint in HONDA_BOSCH_CANFD:
    acc_hud_values['SET_ME_X01'] = int(enabled and (bool(acc_hud_values['HUD_LEAD']) or (pcm_accel < 0.2)))
    acc_hud_values['SET_ME_X01_2'] = int(enabled and (bool(acc_hud_values['HUD_LEAD']) or (pcm_accel < 0.2)))

  if CP.carFingerprint in HONDA_BOSCH:
    acc_hud_values['ACC_ON'] = int(enabled)
    acc_hud_values['FCM_OFF'] = bool(0)
    acc_hud_values['FCM_OFF_2'] = bool(0)
  else:
    # Shows the distance bars, TODO: stock camera shows updates temporarily while disabled
    acc_hud_values['ACC_ON'] = int(enabled)
    acc_hud_values['PCM_SPEED'] = pcm_speed * CV.MS_TO_KPH
    acc_hud_values['PCM_GAS'] = pcm_accel
    # stock holds X01 through accelerate-to-target ramps ("1 gives power"), so keep it set for the
    # whole launch window; outside launches keep the existing at-the-rails behavior
    acc_hud_values['SET_ME_X01'] = 1 if (speed_control or pcm_accel == 0 or pcm_accel == 198) else 0
    acc_hud_values['FCM_OFF'] = acc_hud['FCM_OFF']
    acc_hud_values['FCM_OFF_2'] = acc_hud['FCM_OFF_2']
    acc_hud_values['FCM_PROBLEM'] = acc_hud['FCM_PROBLEM']
    acc_hud_values['ICONS'] = acc_hud['ICONS']

  return packer.make_can_msg("ACC_HUD", bus, acc_hud_values)


def create_lkas_hud(packer, bus, CP, hud_control, lat_active, steering_available, reduced_steering, alert_steer_required, lkas_hud, steer_maxed, CS,
                    lkas_state_change=None):
  commands = []

  if CP.carFingerprint in HONDA_BOSCH:
    solid_lanes = hud_control.lanesVisible and not steer_maxed
    dashed_lanes = lat_active
  else:
    solid_lanes = lat_active and not steer_maxed
    dashed_lanes = hud_control.lanesVisible

  lkas_hud_values = {
    'LKAS_READY': 1,
    'LKAS_STATE_CHANGE': 1,
    'STEERING_REQUIRED': alert_steer_required,
    'SOLID_LANES': solid_lanes,
    'DASHED_LANES': dashed_lanes,
    'BEEP': 0,
  }

  # MDX CAN FD factory logs show the stock camera holds LKAS_STATE_CHANGE low, pulsing it high for ~3s
  # only when the HUD state changes; holding it high permanently suppresses the dash lane-line rendering.
  if lkas_state_change is not None:
    lkas_hud_values['LKAS_STATE_CHANGE'] = int(lkas_state_change)

  if CP.carFingerprint in (HONDA_BOSCH_RADARLESS | HONDA_BOSCH_CANFD):
    lkas_hud_values['LANE_LINES'] = 3
    lkas_hud_values['DASHED_LANES'] = hud_control.lanesVisible

    # car likely needs to see LKAS_PROBLEM fall within a specific time frame, so forward from camera
    # TODO: needed for Bosch CAN FD?
    if CP.carFingerprint in HONDA_BOSCH_RADARLESS:
      lkas_hud_values['LKAS_PROBLEM'] = lkas_hud['LKAS_PROBLEM']

    if CP.carFingerprint in (HONDA_BOSCH_RADARLESS | HONDA_BOSCH_CANFD):
      lkas_hud_values['LKAS_PROBLEM'] = CS.out.steerFaultPermanent # CS.lkas_hud['LKAS_PROBLEM']
      lkas_hud_values['DASHED_LANES'] = 1  # show gray lanes when disengaged

    if CP.carFingerprint in HONDA_BOSCH_CANFD:
      # Don't let steer saturation flicker SOLID_LANES: every payload change must coincide with an
      # LKAS_STATE_CHANGE pulse (see carcontroller), and a 10Hz flicker would keep the pulse
      # re-triggering, which suppresses the dash lane lines.
      # Keyed on lat_active, NOT hud_control.lanesVisible (== CC.enabled): under sunnypilot MADS
      # the lateral control stays engaged when a brake press disengages ACC, and the dash LKAS
      # indication must follow the lateral state or the driver can't tell MADS is still steering
      # (drivers pressing the LKAS button to "re-engage" silently toggled MADS off). On forks
      # without MADS, lat_active == enabled while moving, so this stays equivalent there.
      lkas_hud_values['SOLID_LANES'] = lat_active

  if not (CP.flags & HondaFlags.BOSCH_EXT_HUD):
    lkas_hud_values['RDM_OFF'] = 1
    lkas_hud_values['LANE_ASSIST_BEEP_OFF'] = 1

  if CP.carFingerprint in HONDA_BOSCH_VISION_CTRL:
    # the silenced controller reported RDM on (RDM_ON set, RDM_OFF clear: idle 00 00 10 40 ..); carry its last
    # setting forward instead of telling the dash RDM is off
    lkas_hud_values['RDM_ON'] = lkas_hud['RDM_ON']
    lkas_hud_values['RDM_OFF'] = lkas_hud['RDM_OFF']

  # New HUD concept for selected Bosch cars, overwrites some of the above
  # TODO: make global across all Honda if feedback is favorable
  # Try all Bosch A, didn't work on Nidec, and caused LKAS error in Bosch B/C
  if CP.carFingerprint in HONDA_BOSCH and CP.carFingerprint not in (HONDA_BOSCH_RADARLESS | HONDA_BOSCH_CANFD):
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


def spam_buttons_command(packer, CAN, cruise_button, cruise_setting, ambient_light, car_fingerprint, bus=None):
  values = {
    'CRUISE_BUTTONS': cruise_button,
    'CRUISE_SETTING': cruise_setting,
    # the camera consumes this byte too (adaptive high beam); echo the SCM's live value
    'AMBIENT_LIGHT_MAYBE': ambient_light,
  }
  if bus is None:
    # send buttons to camera on radarless (camera does ACC) cars
    bus = CAN.camera if car_fingerprint in HONDA_BOSCH_RADARLESS else CAN.pt
  return packer.make_can_msg("SCM_BUTTONS", bus, values)


def create_radar_hud_canfd(packer, bus, acc, acc_pulse=False):
  values = {
    # The stock radar only raises this bit in short (~2-6 s) bursts right after ACC engages/resumes,
    # then drops it for the rest of the drive; it is never held for a whole engagement.
    'CMBS_ENABLED_MAYBE': 1 if (acc and acc_pulse) else 0,
    'ACC_ON': acc,
    'SET_ME_X01': 0x01,
    'SET_ME_X01_2': 0x01,
  }
  return packer.make_can_msg("RADAR_HUD_CANFD", bus, values)


def create_canfd_supplemental(packer, bus):
  values = {
    'SET_ME_X01': 0x01,
    'SET_ME_X41': 0x41,
  }
  return packer.make_can_msg("BOSCH_SUPPLEMENTAL_CANFD", bus, values)


# Radar MUX banks: 1-10, 17-26, 33-42, 49-58. Each bank is a fresh sweep of path points.
RADAR_MUX_BANK_STARTS = (1, 17, 33, 49)
# "no detection" sentinel the stock radar uses for an empty path point / object slot
PATH_OFFSET_INVALID = 2047


def _lane_path_offsets(radar_mux):
  # The stock radar reports path points as a sweep within each MUX bank: the first point (bank start)
  # is 0, the second has the first two offsets valid (0) and the rest invalid, and the remaining points
  # are fully invalid. Match that exact per-MUX pattern so the camera sees a consistent empty path.
  pos = next((radar_mux - start for start in RADAR_MUX_BANK_STARTS if start <= radar_mux <= start + 9), 0)
  if pos == 0:
    return (0, 0, 0, 0)
  if pos == 1:
    return (0, 0, PATH_OFFSET_INVALID, PATH_OFFSET_INVALID)
  return (PATH_OFFSET_INVALID,) * 4


def create_canfd_50hz_radar_messages(packer, bus, radar_mux):
  commands = []

  offsets = _lane_path_offsets(radar_mux)
  lane_path_values = {
    'MUX': radar_mux,
    'PATH_OFFSET_1': offsets[0],
    'PATH_OFFSET_2': offsets[1],
    'PATH_OFFSET_3': offsets[2],
    'PATH_OFFSET_4': offsets[3],
  }
  commands.append(packer.make_can_msg('LANE_PATH', bus, lane_path_values))

  # Empty-object-slot sentinel the stock radar transmits (no lead/object): max distances, CAR_TYPE=-1.
  hud_objects_values = {
    'MUX': radar_mux,
    'OBJECT_ID': 0,
    'IS_LEAD_CAR': 0,
    'CAR_TYPE': -1,
    'ROTATION': -128,
    'LONG_DIST': 196.9,
    'LAT_DIST': 204.7,
  }
  commands.append(packer.make_can_msg('HUD_OBJECTS', bus, hud_objects_values))

  return commands


def create_canfd_5hz_radar_messages(packer, bus, radar_ref_cntr, lane_path_length=6, left_lane=0, right_lane=0, radar_lead2=True,
                                    target_speed=140):
  commands = []

  radar_lead_values = {
    'CNTR_REF': radar_ref_cntr,
    'SET_ME_X01': 0x01,
    # stock radar transmits a constant 140 here (confirmed from logs); 120 causes a camera mismatch. The MDX
    # Type S vision controller sends 0 (routes 00000003..0000000b)
    'TARGET_SPEED_MAYBE': target_speed,
    # stock: per-side lane-line detection status (3 = line present, 0 = none), in lockstep with the
    # camera's LKAS_HUD LANE_LINES bits. This is the CAN FD counterpart of radarless LKAS_HUD_2's
    # LEFT_LANE/RIGHT_LANE; the dash does not draw lane lines while both are 0.
    'LEFT_LANE': left_lane,
    'RIGHT_LANE': right_lane,
    # stock: number of valid points in the current LANE_PATH sweep (6 = idle). The dash cross-checks
    # this against the path's in-band 2047 terminator; a mismatch suppresses the lane-line rendering.
    'LANE_PATH_LENGTH': lane_path_length,
  }
  commands.append(packer.make_can_msg('RADAR_LEAD', bus, radar_lead_values))

  # vision ctrl cars: RADAR_LEAD2 is the camera's message there (it moves to the camera bus when the relay
  # opens and the panda forwards it), so OP must not author a second copy
  if radar_lead2:
    radar_lead2_values = {
      'SET_ME_X88': 136,
      'SET_ME_X78': 120,
      'LEAD_DISTANCE_MAYBE': 0,
    }
    commands.append(packer.make_can_msg('RADAR_LEAD2', bus, radar_lead2_values))

  return commands


# ACC_CONTROL_2 GAP_DISTANCE_MAYBE (m) against vehicle speed, stock MDX Type S drive ad9840558640c31d/00000003--3f42518a26
# with 3 distance bars (4.66 m at standstill in 19k frames, medians of 1.5 kph wide bins while driving; about 0.51 m
# per kph above 25 kph, steeper below). Looks like the desired following distance.
VISION_CTRL_GAP_DISTANCE_BP = [0., 5., 10., 20., 30., 40., 50., 60., 70.]
VISION_CTRL_GAP_DISTANCE_V = [4.66, 7.96, 10.79, 15.15, 20.35, 25.30, 30.20, 35.55, 40.50]
# LEAD_DISTANCE_MAYBE with nothing ahead (0x639C)
VISION_CTRL_NO_LEAD_DISTANCE = 255.0
# SET_SPEED with no set speed yet (32 kph = 20 mph, the ACC minimum)
VISION_CTRL_SET_SPEED_MIN = 32


def create_vision_ctrl_acc_status(packer, bus, set_speed_kph, v_ego):
  """ACC_CONTROL_2 (0x1C9): the 50 Hz message the vision controller sends in the frame after ACC_CONTROL. The
  stock drive shows the brake module losing it as well when the controller is silenced, so it is replaced on the
  same cadence as ACC_CONTROL with the values the stock controller sends with nothing ahead."""
  values = {
    "SET_SPEED": int(np.clip(round(set_speed_kph), VISION_CTRL_SET_SPEED_MIN, 255)),
    "LEAD_DISTANCE_MAYBE": VISION_CTRL_NO_LEAD_DISTANCE,
    "GAP_DISTANCE_MAYBE": float(np.interp(v_ego * CV.MS_TO_KPH, VISION_CTRL_GAP_DISTANCE_BP, VISION_CTRL_GAP_DISTANCE_V)),
  }
  return packer.make_can_msg("ACC_CONTROL_2", bus, values)


def create_vision_ctrl_status(packer, bus, silent_frames, hud_tick, state):
  """The constant status broadcasts the vision controller authors alongside its control messages: 25 Hz 0x29B
  (all zero), 10 Hz 0x2E8 (byte 2 = 0x80, in the ACC_HUD/LKAS_HUD frame) and 1 Hz 0x1A45AA24 (byte 0 = the
  controller's last state byte, byte 2 = 0x01). All of them disappear with the controller; the MDX Type S cluster
  raised transmission, lane change CMBS and front cross traffic faults once it was silenced (route
  ad9840558640c31d/0000000a--cffee2dde2). silent_frames counts from the switchover so each goes out at once."""
  commands = []
  if silent_frames % 4 == 0:
    commands.append(packer.make_can_msg("VISION_CTRL_STATUS_25HZ", bus, {}))
  if hud_tick:
    commands.append(packer.make_can_msg("VISION_CTRL_STATUS_10HZ", bus, {"SET_ME_X80": 0x80}))
  if silent_frames % 100 == 0:
    commands.append(packer.make_can_msg("VISION_CTRL_STATUS_1HZ", bus, {"STATE_MAYBE": state, "SET_ME_X01": 0x01}))
  return commands


# The vision controller's PT-bus messages the camera used to see through panda forwarding: STEERING_CONTROL,
# ACC_CONTROL, ACC_CONTROL_2, ACC_HUD and LKAS_HUD (route ad9840558640c31d/0000000b: the forwarded copies stop on
# the camera bus the moment the controller is silenced, and openpilot's own transmissions are never forwarded).
# CarController mirrors openpilot's replacements onto the camera bus byte-identically, like the radar look-alikes.
VISION_CTRL_CAMERA_MIRROR_ADDRS = frozenset({0xE4, 0x1C8, 0x1C9, 0x30C, 0x33D})


def create_vision_ctrl_private_link(packer, bus):
  """0x334 @ 100 Hz on the harness radar bus: the RVU's private-link heartbeat (paired with 0xE6)."""
  return packer.make_can_msg("RVU_PRIVATE_LINK_100HZ", bus, {})


def honda_checksum(address: int, sig, d: bytearray) -> int:
  s = 0
  extended = address > 0x7FF
  # Higher extended-ID range adds 10, lower adds 3. TODO: confirm the exact boundary.
  high_extended = address > 0x100000
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
    s += 10 if high_extended else 3
  return s & 0xF

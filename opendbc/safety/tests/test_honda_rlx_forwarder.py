#!/usr/bin/env python3
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_defaults import TestDefaultRxHookBase

BUS_EPS = 0      # steer bus, car side
BUS_PT = 1       # powertrain bus
BUS_CAMERA = 2   # steer bus, camera side

STEER_STATUS = 0x18F
CAR_SPEED = 0x309
STEERING_CONTROL = 0x194
LKAS_HUD = 0x33D

STEER_CMD_ADDRS = {STEERING_CONTROL, LKAS_HUD}
EPS_STATUS_ADDRS = {STEER_STATUS, CAR_SPEED}

# everything the car side of the steer bus sends (a5cd616a92467aed|0000013b, bus 4) and the camera sends (bus 6)
CAR_SIDE_ADDRS = (0x39, 0x91, 0x130, 0x13B, 0x156, 0x158, 0x17C, 0x18E, 0x18F, 0x1A3, 0x1A4, 0x1A6, 0x1AC, 0x1B0, 0x1D0, 0x1EA,
                  0x1FA, 0x221, 0x294, 0x305, 0x309, 0x30C, 0x324, 0x328, 0x32F, 0x335, 0x340, 0x346, 0x374, 0x37C, 0x388,
                  0x3A1, 0x3A2, 0x3A3, 0x3D9, 0x400, 0x403, 0x405, 0x406, 0x40A, 0x40C, 0x40F, 0x454, 0x471, 0x52A, 0x53D,
                  0x544, 0x551, 0x555, 0x595, 0x59D, 0x59F, 0x5A0)
CAMERA_ADDRS = (0x194, 0x33D, 0x39F, 0x640, 0x641)

# openpilot's STEERING_CONTROL on the powertrain bus is considered gone after this long
OP_STEER_TIMEOUT_US = 200_000

# param bit 0: the car's gateway already relays openpilot's steering command/HUD onto the steer bus
PARAM_GATEWAY_RELAYS_STEER_CMDS = 1


def expected_fwd_bus(bus: int, addr: int, fwd_steer_cmds: bool = True, op_steering: bool = False) -> int:
  if bus == BUS_CAMERA:
    return -1 if (addr in STEER_CMD_ADDRS and op_steering) else BUS_EPS
  if bus == BUS_EPS:
    return BUS_CAMERA
  if bus == BUS_PT:
    return BUS_EPS if (addr in STEER_CMD_ADDRS and fwd_steer_cmds) else -1
  return -1


def expected_copy_bus(bus: int, addr: int) -> int:
  if bus == BUS_EPS and addr in EPS_STATUS_ADDRS:
    return BUS_PT
  if bus == BUS_CAMERA and addr == LKAS_HUD:
    return BUS_PT
  return -1


class TestHondaRlxForwarder(TestDefaultRxHookBase):
  """
    Standalone bridge panda sitting on the RLX steer bus (intercepting the LKAS camera) with a tap
    on the powertrain bus. The two halves of the steer bus see everything of each other, openpilot
    gets copies of the EPS status and the camera HUD, and nothing may be sent over USB.
  """
  TX_MSGS = []
  PARAM = 0
  FWD_STEER_CMDS = True

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hondaRlxForwarder, self.PARAM)
    self.safety.init_tests()
    self.safety.set_timer(0)

  def _openpilot_steers(self, t_us: int = 1_000_000):
    # openpilot's STEERING_CONTROL is only observed through the fwd hook, like in the firmware
    self.safety.set_timer(t_us)
    self.assertEqual(BUS_EPS if self.FWD_STEER_CMDS else -1, self.safety.safety_fwd_hook(BUS_PT, STEERING_CONTROL))

  def _check_fwd_hooks(self, op_steering: bool):
    # scanning the powertrain bus marks openpilot as steering, so check the camera bus first
    for bus in (BUS_CAMERA, BUS_EPS, BUS_PT, 3):
      for addr in self.SCANNED_ADDRS:
        self.assertEqual(expected_fwd_bus(bus, addr, self.FWD_STEER_CMDS, op_steering), self.safety.safety_fwd_hook(bus, addr),
                         f"{addr=:#x} from {bus=} {op_steering=}")
        self.assertEqual(expected_copy_bus(bus, addr), self.safety.safety_fwd_copy_hook(bus, addr), f"{addr=:#x} from {bus=} copy")

  def test_fwd_hook(self):
    # routing is per address, so the generic bus lookup test doesn't apply
    # boot: openpilot has never steered, the camera drives the EPS
    self._check_fwd_hooks(op_steering=False)

    # openpilot steering: its steering command and HUD replace the camera's on the steer bus
    self._openpilot_steers()
    self._check_fwd_hooks(op_steering=True)

    # openpilot gone: back to the camera
    self.safety.set_timer(1_000_000 + OP_STEER_TIMEOUT_US)
    self._check_fwd_hooks(op_steering=False)

  def test_stock_ecus_always_see_each_other(self):
    # regardless of openpilot, every car side message reaches the camera and every camera message reaches
    # the car side, except the two the camera and openpilot both produce while openpilot steers
    for op_steering, t_us in ((False, 0), (True, 1_000_000), (False, 1_000_000 + OP_STEER_TIMEOUT_US)):
      if op_steering:
        self._openpilot_steers(t_us)
      else:
        self.safety.set_timer(t_us)
      for addr in CAR_SIDE_ADDRS:
        self.assertEqual(BUS_CAMERA, self.safety.safety_fwd_hook(BUS_EPS, addr), f"{addr=:#x} {op_steering=}")
      for addr in CAMERA_ADDRS:
        replaced = op_steering and addr in STEER_CMD_ADDRS
        self.assertEqual(-1 if replaced else BUS_EPS, self.safety.safety_fwd_hook(BUS_CAMERA, addr), f"{addr=:#x} {op_steering=}")
      # openpilot's replacements are on the powertrain bus and get relayed to the steer bus
      if op_steering:
        for addr in STEER_CMD_ADDRS:
          self.assertEqual(BUS_EPS if self.FWD_STEER_CMDS else -1, self.safety.safety_fwd_hook(BUS_PT, addr), f"{addr=:#x}")

  def test_copies_to_powertrain(self):
    # openpilot reads the EPS status and the camera HUD from copies on the powertrain bus, at all times
    for op_steering in (False, True):
      if op_steering:
        self._openpilot_steers()
      for addr in EPS_STATUS_ADDRS:
        self.assertEqual(BUS_PT, self.safety.safety_fwd_copy_hook(BUS_EPS, addr), f"{addr=:#x} {op_steering=}")
        self.assertEqual(BUS_CAMERA, self.safety.safety_fwd_hook(BUS_EPS, addr), f"{addr=:#x} {op_steering=}")
      self.assertEqual(BUS_PT, self.safety.safety_fwd_copy_hook(BUS_CAMERA, LKAS_HUD), f"{op_steering=}")
      # nothing else is duplicated
      for addr in (*CAR_SIDE_ADDRS, *CAMERA_ADDRS):
        for bus in (BUS_EPS, BUS_CAMERA, BUS_PT):
          if expected_copy_bus(bus, addr) == -1:
            self.assertEqual(-1, self.safety.safety_fwd_copy_hook(bus, addr), f"{addr=:#x} from {bus=}")

  def test_copies_blocked_on_relay_malfunction(self):
    self.safety.set_relay_malfunction(True)
    self.assertEqual(-1, self.safety.safety_fwd_copy_hook(BUS_EPS, STEER_STATUS))
    self.assertEqual(-1, self.safety.safety_fwd_hook(BUS_EPS, STEER_STATUS))
    self.safety.set_relay_malfunction(False)
    self.assertEqual(BUS_PT, self.safety.safety_fwd_copy_hook(BUS_EPS, STEER_STATUS))

  def test_openpilot_steer_timeout(self):
    # the camera takes over exactly when openpilot's steering command has been gone for the timeout
    self._openpilot_steers(t_us=1_000_000)
    for elapsed_us, camera_replaced in ((0, True), (OP_STEER_TIMEOUT_US - 1, True), (OP_STEER_TIMEOUT_US, False), (10_000_000, False)):
      self.safety.set_timer(1_000_000 + elapsed_us)
      for addr in STEER_CMD_ADDRS:
        self.assertEqual(-1 if camera_replaced else BUS_EPS, self.safety.safety_fwd_hook(BUS_CAMERA, addr), f"{addr=:#x} {elapsed_us=}")

    # each openpilot steering command restarts the timeout
    self._openpilot_steers(t_us=20_000_000)
    self.safety.set_timer(20_000_000 + OP_STEER_TIMEOUT_US - 1)
    self.assertEqual(-1, self.safety.safety_fwd_hook(BUS_CAMERA, STEERING_CONTROL))

    # only openpilot's steering command counts, not its LKAS_HUD or anything else on the powertrain bus
    self.safety.set_timer(30_000_000)
    for addr in (LKAS_HUD, 0x158, 0x17C, 0x1FA, 0x30C):
      self.safety.safety_fwd_hook(BUS_PT, addr)
    self.assertEqual(BUS_EPS, self.safety.safety_fwd_hook(BUS_CAMERA, STEERING_CONTROL))

  def test_openpilot_steer_timer_wraparound(self):
    # the microsecond timer wraps every ~71 minutes; the timeout must survive that
    self._openpilot_steers(t_us=2**32 - OP_STEER_TIMEOUT_US // 2)
    self.safety.set_timer(OP_STEER_TIMEOUT_US // 4)
    self.assertEqual(-1, self.safety.safety_fwd_hook(BUS_CAMERA, STEERING_CONTROL))
    self.safety.set_timer(OP_STEER_TIMEOUT_US)
    self.assertEqual(BUS_EPS, self.safety.safety_fwd_hook(BUS_CAMERA, STEERING_CONTROL))

  def test_init_resets_openpilot_steering(self):
    self._openpilot_steers()
    self.assertEqual(-1, self.safety.safety_fwd_hook(BUS_CAMERA, STEERING_CONTROL))
    self.safety.set_safety_hooks(CarParams.SafetyModel.hondaRlxForwarder, self.PARAM)
    self.assertEqual(BUS_EPS, self.safety.safety_fwd_hook(BUS_CAMERA, STEERING_CONTROL))

  def test_powertrain_to_eps(self):
    # openpilot's steering command and LKAS HUD are relayed from the powertrain bus to the EPS,
    # unless the car's gateway is known to do that already
    for addr in STEER_CMD_ADDRS:
      self.assertEqual(BUS_EPS if self.FWD_STEER_CMDS else -1, self.safety.safety_fwd_hook(BUS_PT, addr), f"{addr=:#x}")
    # nothing else from the powertrain bus leaks onto the steer bus, and nothing is copied off it
    for addr in (0x158, 0x17C, 0x1FA, 0x30C, *EPS_STATUS_ADDRS):
      self.assertEqual(-1, self.safety.safety_fwd_hook(BUS_PT, addr), f"{addr=:#x}")
    for addr in (*STEER_CMD_ADDRS, *EPS_STATUS_ADDRS, 0x158):
      self.assertEqual(-1, self.safety.safety_fwd_copy_hook(BUS_PT, addr), f"{addr=:#x}")

  def test_no_usb_tx(self):
    # even with controls allowed, nothing may be transmitted from the bridge panda
    self.safety.set_controls_allowed(True)
    for bus in range(4):
      for addr in STEER_CMD_ADDRS | EPS_STATUS_ADDRS:
        self.assertFalse(self._tx(common.make_msg(bus, addr, 8)), f"allowed TX {addr=:#x} {bus=}")

  def test_forwarding_survives_rx(self):
    # RX traffic on any bus must not disturb forwarding (no relay malfunction detection possible)
    self._openpilot_steers()
    for bus in (BUS_EPS, BUS_PT, BUS_CAMERA):
      for addr in STEER_CMD_ADDRS | EPS_STATUS_ADDRS:
        self.assertTrue(self._rx(common.make_msg(bus, addr, 8)))
    self.assertFalse(self.safety.get_relay_malfunction())
    self.assertEqual(BUS_CAMERA, self.safety.safety_fwd_hook(BUS_EPS, STEER_STATUS))
    self.assertEqual(BUS_PT, self.safety.safety_fwd_copy_hook(BUS_EPS, STEER_STATUS))
    self.assertEqual(BUS_EPS if self.FWD_STEER_CMDS else -1, self.safety.safety_fwd_hook(BUS_PT, STEERING_CONTROL))
    self.assertEqual(-1, self.safety.safety_fwd_hook(BUS_CAMERA, STEERING_CONTROL))
    self.assertEqual(BUS_PT, self.safety.safety_fwd_copy_hook(BUS_CAMERA, LKAS_HUD))


class TestHondaRlxForwarderGatewayRelay(TestHondaRlxForwarder):
  """
    Same bridge, but the gateway already relays openpilot's 0x194/0x33D onto the steer bus,
    so the bridge must not forward them itself. The camera gating and the copies are unchanged.
  """
  PARAM = PARAM_GATEWAY_RELAYS_STEER_CMDS
  FWD_STEER_CMDS = False


if __name__ == "__main__":
  unittest.main()

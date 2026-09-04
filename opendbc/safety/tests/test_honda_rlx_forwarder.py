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


def expected_fwd_bus(bus: int, addr: int) -> int:
  if bus == BUS_CAMERA:
    return -1 if addr in STEER_CMD_ADDRS else BUS_EPS
  if bus == BUS_EPS:
    return BUS_PT if addr in EPS_STATUS_ADDRS else BUS_CAMERA
  if bus == BUS_PT:
    return BUS_EPS if addr in STEER_CMD_ADDRS else -1
  return -1


class TestHondaRlxForwarder(TestDefaultRxHookBase):
  """
    Standalone bridge panda sitting on the RLX steer bus (intercepting the LKAS camera) with a tap
    on the powertrain bus. Only the steering messages cross to/from the powertrain bus and nothing
    may be sent over USB.
  """
  TX_MSGS = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hondaRlxForwarder, 0)
    self.safety.init_tests()

  def test_fwd_hook(self):
    # routing is per address, so the generic bus lookup test doesn't apply
    for bus in range(4):
      for addr in self.SCANNED_ADDRS:
        self.assertEqual(expected_fwd_bus(bus, addr), self.safety.safety_fwd_hook(bus, addr), f"{addr=:#x} from {bus=}")

  def test_camera_intercept(self):
    # the stock camera's steering command and HUD never reach the EPS, everything else passes through
    for addr in STEER_CMD_ADDRS:
      self.assertEqual(-1, self.safety.safety_fwd_hook(BUS_CAMERA, addr), f"{addr=:#x}")
    for addr in (0x1FA, 0x30C, 0x158, 0x17C):
      self.assertEqual(BUS_EPS, self.safety.safety_fwd_hook(BUS_CAMERA, addr), f"{addr=:#x}")

  def test_eps_status_to_powertrain(self):
    # EPS feedback goes to the powertrain bus instead of the camera so openpilot can read it
    for addr in EPS_STATUS_ADDRS:
      self.assertEqual(BUS_PT, self.safety.safety_fwd_hook(BUS_EPS, addr), f"{addr=:#x}")
    # anything else from the EPS side still reaches the camera, never the powertrain bus
    for addr in (0x156, 0x1D0, 0x326, *STEER_CMD_ADDRS):
      self.assertEqual(BUS_CAMERA, self.safety.safety_fwd_hook(BUS_EPS, addr), f"{addr=:#x}")

  def test_powertrain_to_eps(self):
    # openpilot's steering command and LKAS HUD are relayed from the powertrain bus to the EPS
    for addr in STEER_CMD_ADDRS:
      self.assertEqual(BUS_EPS, self.safety.safety_fwd_hook(BUS_PT, addr), f"{addr=:#x}")
    # nothing else from the powertrain bus leaks onto the steer bus
    for addr in (0x158, 0x17C, 0x1FA, 0x30C, *EPS_STATUS_ADDRS):
      self.assertEqual(-1, self.safety.safety_fwd_hook(BUS_PT, addr), f"{addr=:#x}")

  def test_no_usb_tx(self):
    # even with controls allowed, nothing may be transmitted from the bridge panda
    self.safety.set_controls_allowed(True)
    for bus in range(4):
      for addr in STEER_CMD_ADDRS | EPS_STATUS_ADDRS:
        self.assertFalse(self._tx(common.make_msg(bus, addr, 8)), f"allowed TX {addr=:#x} {bus=}")

  def test_forwarding_survives_rx(self):
    # RX traffic on any bus must not disturb forwarding (no relay malfunction detection possible)
    for bus in (BUS_EPS, BUS_PT, BUS_CAMERA):
      for addr in STEER_CMD_ADDRS | EPS_STATUS_ADDRS:
        self.assertTrue(self._rx(common.make_msg(bus, addr, 8)))
    self.assertFalse(self.safety.get_relay_malfunction())
    self.assertEqual(BUS_PT, self.safety.safety_fwd_hook(BUS_EPS, STEER_STATUS))
    self.assertEqual(BUS_EPS, self.safety.safety_fwd_hook(BUS_PT, STEERING_CONTROL))
    self.assertEqual(-1, self.safety.safety_fwd_hook(BUS_CAMERA, STEERING_CONTROL))


if __name__ == "__main__":
  unittest.main()

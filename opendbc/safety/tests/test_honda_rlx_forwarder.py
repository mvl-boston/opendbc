#!/usr/bin/env python3
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_defaults import TestDefaultRxHookBase

STEER_STATUS = 0x18F
CAR_SPEED = 0x309
STEERING_CONTROL = 0x194
LKAS_HUD = 0x33D

# messages allowed from the steer bus to the powertrain bus and vice versa
STEER_TO_PT_ADDRS = {STEER_STATUS, CAR_SPEED}
PT_TO_STEER_ADDRS = {STEERING_CONTROL, LKAS_HUD}


class TestHondaRlxForwarderBase(TestDefaultRxHookBase):
  """
    Standalone bridge panda between the RLX steer bus and the powertrain bus.
    Only a handful of steering messages cross between the buses and nothing may be sent over USB.
  """
  TX_MSGS = []
  FWD_BUS_LOOKUP = {0: 2, 2: 0}
  PARAM = 0
  PT_BUS = 0
  STEER_BUS = 2

  @classmethod
  def setUpClass(cls):
    if cls.__name__.endswith('Base'):
      cls.safety = None
      raise unittest.SkipTest

    scanned = set(cls.SCANNED_ADDRS)
    cls.FWD_BLACKLISTED_ADDRS = {
      cls.PT_BUS: scanned - PT_TO_STEER_ADDRS,
      cls.STEER_BUS: scanned - STEER_TO_PT_ADDRS,
    }

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hondaRlxForwarder, self.PARAM)
    self.safety.init_tests()

  def test_steer_bus_to_powertrain(self):
    for addr in STEER_TO_PT_ADDRS:
      self.assertEqual(self.PT_BUS, self.safety.safety_fwd_hook(self.STEER_BUS, addr), f"{addr=:#x}")
      # never reflected back onto the steer bus
      self.assertEqual(-1, self.safety.safety_fwd_hook(self.PT_BUS, addr), f"{addr=:#x}")

  def test_powertrain_to_steer_bus(self):
    for addr in PT_TO_STEER_ADDRS:
      self.assertEqual(self.STEER_BUS, self.safety.safety_fwd_hook(self.PT_BUS, addr), f"{addr=:#x}")
      # the stock camera's steering messages must not leak onto the powertrain bus
      self.assertEqual(-1, self.safety.safety_fwd_hook(self.STEER_BUS, addr), f"{addr=:#x}")

  def test_nothing_forwarded_from_bus_1(self):
    for addr in STEER_TO_PT_ADDRS | PT_TO_STEER_ADDRS:
      self.assertEqual(-1, self.safety.safety_fwd_hook(1, addr), f"{addr=:#x}")

  def test_no_usb_tx(self):
    # even with controls allowed, nothing may be transmitted from the bridge panda
    self.safety.set_controls_allowed(True)
    for bus in range(4):
      for addr in STEER_TO_PT_ADDRS | PT_TO_STEER_ADDRS:
        self.assertFalse(self._tx(common.make_msg(bus, addr, 8)), f"allowed TX {addr=:#x} {bus=}")

  def test_forwarding_survives_rx(self):
    # RX traffic on either bus must not disturb forwarding (no relay malfunction detection possible)
    for bus in (self.PT_BUS, self.STEER_BUS):
      for addr in STEER_TO_PT_ADDRS | PT_TO_STEER_ADDRS:
        self.assertTrue(self._rx(common.make_msg(bus, addr, 8)))
    self.assertFalse(self.safety.get_relay_malfunction())
    self.assertEqual(self.PT_BUS, self.safety.safety_fwd_hook(self.STEER_BUS, STEER_STATUS))
    self.assertEqual(self.STEER_BUS, self.safety.safety_fwd_hook(self.PT_BUS, STEERING_CONTROL))


class TestHondaRlxForwarder(TestHondaRlxForwarderBase):
  pass


class TestHondaRlxForwarderSwapped(TestHondaRlxForwarderBase):
  """
    Same bridge with the steer bus on bus 0 and the powertrain bus on bus 2
  """
  PARAM = 1
  PT_BUS = 2
  STEER_BUS = 0


if __name__ == "__main__":
  unittest.main()

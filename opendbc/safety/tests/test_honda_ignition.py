#!/usr/bin/env python3
import unittest

from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety


class TestHondaIgnition(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.packer = CANPackerSafety("acura_rlx_2017_can_generated")

  def _msg_1a6(self, counter, main_on):
    return self.packer.make_can_msg_safety("SCM_BUTTONS", 0, {"MAIN_ON": main_on, "COUNTER": counter})

  def _msg_326(self, counter, main_on):
    return self.packer.make_can_msg_safety("SCM_FEEDBACK", 0, {"MAIN_ON": main_on, "COUNTER": counter})

  def test_ignition_on_1a6(self):
    for i in range(4):
      self.safety.init_tests()
      self.safety.ignition_can_hook(self._msg_1a6(i, 1))
      self.assertFalse(self.safety.get_ignition_can())
      self.safety.ignition_can_hook(self._msg_1a6((i + 1) % 4, 1))
      self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off_1a6(self):
    self.safety.ignition_can_hook(self._msg_1a6(0, 1))
    self.safety.ignition_can_hook(self._msg_1a6(1, 1))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg_1a6(2, 0))
    self.safety.ignition_can_hook(self._msg_1a6(3, 0))
    self.assertFalse(self.safety.get_ignition_can())

  def test_ignition_on_326(self):
    self.packer = CANPackerSafety("honda_civic_touring_2016_can_generated")
    for i in range(4):
      self.safety.init_tests()
      self.safety.ignition_can_hook(self._msg_326(i, 1))
      self.assertFalse(self.safety.get_ignition_can())
      self.safety.ignition_can_hook(self._msg_326((i + 1) % 4, 1))
      self.assertTrue(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()

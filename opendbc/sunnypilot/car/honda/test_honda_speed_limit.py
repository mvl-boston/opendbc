"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest

from opendbc.car.common.conversions import Conversions as CV
from opendbc.sunnypilot.car.honda.carstate_ext import CarStateExt


class TestHondaSpeedLimit(unittest.TestCase):

  def test_metric_speed_limit_from_sign(self):
    self.assertAlmostEqual(CarStateExt._metric_speed_limit_from_sign(3), 30 * CV.KPH_TO_MS)
    self.assertAlmostEqual(CarStateExt._metric_speed_limit_from_sign(10), 100 * CV.KPH_TO_MS)
    self.assertAlmostEqual(CarStateExt._metric_speed_limit_from_sign(61), 0.0)

  def test_imperial_speed_limit_with_counter_nibble(self):
    self.assertAlmostEqual(CarStateExt._imperial_speed_limit_m_s(199), 35 * CV.MPH_TO_MS)
    self.assertAlmostEqual(CarStateExt._imperial_speed_limit_m_s(201), 45 * CV.MPH_TO_MS)
    self.assertAlmostEqual(CarStateExt._imperial_speed_limit_m_s(125), 0.0)
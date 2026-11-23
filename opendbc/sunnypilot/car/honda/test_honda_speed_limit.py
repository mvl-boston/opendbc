"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest

from types import SimpleNamespace

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.honda.values import CAR
from opendbc.sunnypilot.car.honda.carstate_ext import CarStateExt, imperial_speed_limit_from_sign, metric_speed_limit_from_sign
from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP


class TestHondaSpeedLimit(unittest.TestCase):

  def test_imperial_speed_limit_from_sign(self):
    self.assertAlmostEqual(imperial_speed_limit_from_sign(102), 30 * CV.MPH_TO_MS)
    self.assertAlmostEqual(imperial_speed_limit_from_sign(125), 0.0)
    self.assertAlmostEqual(imperial_speed_limit_from_sign(0), 0.0)

  def test_metric_speed_limit_from_sign(self):
    self.assertAlmostEqual(metric_speed_limit_from_sign(3), 30 * CV.KPH_TO_MS)
    self.assertAlmostEqual(metric_speed_limit_from_sign(10), 100 * CV.KPH_TO_MS)
    self.assertAlmostEqual(metric_speed_limit_from_sign(61), 0.0)
    self.assertAlmostEqual(metric_speed_limit_from_sign(17), 0.0)

  def test_metric_speed_limit_parsers_use_camera_bus_only(self):
    CP = SimpleNamespace(carFingerprint=CAR.HONDA_E)
    CP_SP = SimpleNamespace(flags=HondaFlagsSP.HAS_CAMERA_MESSAGES | HondaFlagsSP.METRIC_CAMERA_SPEED_LIMIT)
    pt_msgs, cam_msgs = CarStateExt.speed_limit_parsers(CP, CP_SP)
    self.assertEqual(pt_msgs, [])
    self.assertEqual(cam_msgs, [("TSR_SPEED_LIMIT", 50), ("CAMERA_MESSAGES", 10)])
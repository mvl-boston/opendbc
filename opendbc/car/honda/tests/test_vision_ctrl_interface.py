import unittest
from unittest import mock

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.honda import obd_dtcs, vision_ctrl
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR, DBC

ADDR = 0x18DAB8F1
REPLY_ADDR = 0x18DAF1B8
ACCEPT = b"\x02\x68\x03\x55\x55\x55\x55\x55"


class TestVisionCtrlInterfaceUpdate(unittest.TestCase):
  """CarInterface.update() takes a list of (nanos, frames) packets, but CANParser.update also accepts a single packet and
  openpilot's test_models relies on that; the vision ctrl raw-packet hooks must accept both shapes too."""

  def setUp(self):
    CP = CarInterface.get_params(CAR.ACURA_MDX_4G_TYPE_S, gen_empty_fingerprint(), [], alpha_long=True, is_release=False, docs=False)
    self.CI = CarInterface(CP)
    self.packer = CANPacker(DBC[CAR.ACURA_MDX_4G_TYPE_S][Bus.pt])

  def tearDown(self):
    vision_ctrl._set_awaiting_response(None)

  def _hud_packet(self, hud_distance):
    addr, dat, bus = self.packer.make_can_msg("ACC_HUD", 0, {"HUD_DISTANCE": hud_distance, "CRUISE_SPEED": 255, "COUNTER": 0})
    return (0, [CanData(addr, dat, bus)])

  def test_single_packet_captures_stock_frames(self):
    packet = self._hud_packet(2)
    self.CI.update(packet)
    self.assertEqual(self.CI.CS.vision_stock_payloads["ACC_HUD"], packet[1][0].dat)

  def test_packet_list_captures_stock_frames(self):
    packets = [self._hud_packet(2), self._hud_packet(3)]
    self.CI.update(packets)
    self.assertEqual(self.CI.CS.vision_stock_payloads["ACC_HUD"], packets[1][1][0].dat)

  def test_empty_update(self):
    self.CI.update([])
    self.assertEqual(self.CI.CS.vision_stock_payloads, {})

  def test_single_packet_records_diag_response(self):
    vision_ctrl._set_awaiting_response(ADDR)
    self.CI.update((0, [CanData(REPLY_ADDR, ACCEPT, 0)]))
    self.assertEqual(vision_ctrl._take_diag_responses(), [ACCEPT])


class TestVisionCtrlFaultClearRun(unittest.TestCase):
  """Without alpha long the vision ctrl cars run the fault-clear mode: not dashcam-only (so card calls init()), panda
  in noOutput, openpilot never engages and never transmits."""

  def setUp(self):
    self.CP = CarInterface.get_params(CAR.ACURA_MDX_4G_TYPE_S, gen_empty_fingerprint(), [], alpha_long=False, is_release=False, docs=False)
    self.CI = CarInterface(self.CP)

  def test_car_params(self):
    self.assertFalse(self.CP.openpilotLongitudinalControl)
    self.assertFalse(self.CP.dashcamOnly)
    self.assertEqual([c.safetyModel for c in self.CP.safetyConfigs], [structs.CarParams.SafetyModel.noOutput])

  def test_alpha_long_keeps_the_car_safety_mode(self):
    CP = CarInterface.get_params(CAR.ACURA_MDX_4G_TYPE_S, gen_empty_fingerprint(), [], alpha_long=True, is_release=False, docs=False)
    self.assertEqual([c.safetyModel for c in CP.safetyConfigs], [structs.CarParams.SafetyModel.hondaBosch])

  def test_controller_sends_nothing(self):
    self.CI.update([])
    for _ in range(300):
      _, can_sends = self.CI.apply(structs.CarControl().as_reader(), 0)
      self.assertEqual(can_sends, [])
    self.assertIsNone(self.CI.CC.vision_ctrl_silencer)

  def test_stock_acc_never_engages_openpilot(self):
    packer = CANPacker(DBC[CAR.ACURA_MDX_4G_TYPE_S][Bus.pt])
    addr, dat, bus = packer.make_can_msg("POWERTRAIN_DATA", 0, {"ACC_STATUS": 1})
    CS = self.CI.update([(0, [CanData(addr, dat, bus)])])
    self.assertFalse(CS.cruiseState.enabled)
    self.assertFalse(CS.cruiseState.available)

  def test_init_without_params_only_broadcast_clears(self):
    sent = []
    with mock.patch.object(obd_dtcs, "params_obd_multiplexing", return_value=None):
      CarInterface.init(self.CP, lambda wait_for_one=False: [], sent.extend)
    self.assertEqual([(m.address, m.src) for m in sent], [(0x18DB33F1, 0), (0x18DB33F1, 2)])

  def test_init_inquires_and_clears_over_obd(self):
    calls = []
    with mock.patch.object(obd_dtcs, "params_obd_multiplexing", return_value=lambda enabled: True), \
         mock.patch.object(obd_dtcs, "inquire_obd_dtcs", side_effect=lambda *a, **k: calls.append(k)):
      CarInterface.init(self.CP, lambda wait_for_one=False: [], lambda msgs: None)
    self.assertEqual(calls, [{"clear": True}])


if __name__ == "__main__":
  unittest.main()

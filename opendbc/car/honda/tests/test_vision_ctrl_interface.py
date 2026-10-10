import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.can_definitions import CanData
from opendbc.car.honda import vision_ctrl
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


if __name__ == "__main__":
  unittest.main()

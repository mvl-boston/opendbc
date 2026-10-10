import unittest

from opendbc.can import CANPacker
from opendbc.car.honda import hondacan
from opendbc.car.honda.values import CAR, DBC
from opendbc.car import Bus

# last stock frames of route ad9840558640c31d/00000012 (car in Park)
STOCK = {
  "ACC_CONTROL": bytes.fromhex("0001900000000009"),
  "ACC_CONTROL_2": bytes.fromhex("20639c01d2000002"),
  "ACC_HUD": bytes.fromhex("000000ff00c0c003"),
  "LKAS_HUD": bytes.fromhex("0000104000000000"),
  "RADAR_LEAD": bytes.fromhex("e000001800000021"),
}


class TestVisionCtrlParkHold(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker(DBC[CAR.ACURA_MDX_4G_TYPE_S][Bus.pt])

  def _counter(self, name, dat):
    msg = self.packer.dbc.name_to_msg[name]
    return (dat[msg.sigs["COUNTER"].lsb // 8] >> (msg.sigs["COUNTER"].lsb % 8)) & 0x3

  def test_stock_frames_are_self_consistent(self):
    # the samples carry the checksum the DBC computes, so a restamped copy with the same counter is the sample itself
    for name, stock in STOCK.items():
      values = {"COUNTER": self._counter(name, stock)}
      if name == "RADAR_LEAD":
        values["CNTR_REF"] = stock[0] >> 6  # kept from openpilot's frame, so hand it the stock value
      op = self.packer.make_can_msg(name, 0, values)[1]
      self.assertEqual(hondacan.restamp_stock_frame(self.packer, name, stock, op), stock, name)

  def test_restamp_keeps_stock_bytes_continues_counter(self):
    # openpilot's ACC_CONTROL_2 with a lead 7.26 m ahead and the 2-bar gap, counter 1: the held frame is the stock
    # content (no lead, 3-bar gap) with counter 1 and a checksum that matches it
    op = self.packer.make_can_msg("ACC_CONTROL_2", 0, {"SET_SPEED": 32, "LEAD_DISTANCE_MAYBE": 7.26, "GAP_DISTANCE_MAYBE": 4.33,
                                                       "COUNTER": 1})[1]
    held = hondacan.restamp_stock_frame(self.packer, "ACC_CONTROL_2", STOCK["ACC_CONTROL_2"], op)
    self.assertEqual(held[:7], bytes.fromhex("20639c01d20000"))
    self.assertEqual(self._counter("ACC_CONTROL_2", held), 1)
    expected = self.packer.make_can_msg("ACC_CONTROL_2", 0, {"SET_SPEED": 32, "LEAD_DISTANCE_MAYBE": 255., "GAP_DISTANCE_MAYBE": 4.66,
                                                             "COUNTER": 1})[1]
    self.assertEqual(held, expected)

  def test_radar_lead_keeps_openpilot_cntr_ref(self):
    # CNTR_REF cycles with the counter (stock: COUNTER + 1), so it is openpilot's, the rest is stock
    op = self.packer.make_can_msg("RADAR_LEAD", 0, {"CNTR_REF": 1, "SET_ME_X01": 1, "LANE_PATH_LENGTH": 10, "LEFT_LANE": 3, "COUNTER": 0})[1]
    held = hondacan.restamp_stock_frame(self.packer, "RADAR_LEAD", STOCK["RADAR_LEAD"], op)
    self.assertEqual(held, bytes.fromhex("600000180000000b"))  # the stock frame of that counter on route 12

  def test_hold_only_known_frames_on_given_buses(self):
    acc = self.packer.make_can_msg("ACC_CONTROL", 0, {"COUNTER": 2})
    hud = self.packer.make_can_msg("ACC_HUD", 0, {"HUD_DISTANCE": 2, "CRUISE_SPEED": 255, "COUNTER": 2})
    steer = self.packer.make_can_msg("STEERING_CONTROL", 0, {"COUNTER": 2})
    other_bus = self.packer.make_can_msg("ACC_HUD", 1, {"HUD_DISTANCE": 2, "CRUISE_SPEED": 255, "COUNTER": 2})
    payloads = {"ACC_HUD": STOCK["ACC_HUD"]}
    out = hondacan.hold_vision_stock_frames(self.packer, [acc, hud, steer, other_bus], payloads, (0, 2))
    self.assertEqual(out[0], acc)  # no stock frame seen: openpilot's
    self.assertEqual(out[1][1][:7], STOCK["ACC_HUD"][:7])
    self.assertEqual(self._counter("ACC_HUD", out[1][1]), 2)
    self.assertEqual(out[2], steer)  # not a held message
    self.assertEqual(out[3], other_bus)  # not a held bus

  def test_size_mismatch_keeps_openpilot_frame(self):
    op = self.packer.make_can_msg("ACC_HUD", 0, {"COUNTER": 0})[1]
    self.assertEqual(hondacan.restamp_stock_frame(self.packer, "ACC_HUD", b"\x00\x01", op), op)


if __name__ == "__main__":
  unittest.main()

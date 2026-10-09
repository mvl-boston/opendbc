import unittest

from opendbc.car.honda import vision_ctrl
from opendbc.car.honda.vision_ctrl import (COMM_CONTROL_ALL_NETWORKS_VARIANT, COMM_CONTROL_DISABLE_MSGS, COMM_CONTROL_ENABLE_MSG,
                                           EXT_DIAG_SESSION_MSG, VisionControllerSilencer)

ADDR = 0x18DAB8F1
BUS = 0


class TestVisionControllerSilencer(unittest.TestCase):
  def setUp(self):
    vision_ctrl.set_shutdown_allowed(True)
    self.silencer = VisionControllerSilencer([ADDR])

  def tearDown(self):
    vision_ctrl.set_shutdown_allowed(False)
    vision_ctrl._set_silenced_addr(None)
    vision_ctrl._set_expecting_silence(False)

  def _run(self, frames, stock_alive):
    sent = []
    for _ in range(frames):
      sent.extend(self.silencer.update(stock_alive, BUS))
    return [bytes(dat) for _, dat, _ in sent]

  def test_this_network_variant_first(self):
    sent = self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self.assertEqual(sent, [EXT_DIAG_SESSION_MSG, COMM_CONTROL_DISABLE_MSGS[0]])
    self.assertEqual(COMM_CONTROL_DISABLE_MSGS[0], b"\x03\x28\x01\xF3\x00\x00\x00\x00")
    self.assertTrue(vision_ctrl.expecting_silence())

    # the controller stops: locked with the this-network variant, its private links are its own
    self.assertEqual(self._run(1, stock_alive=False), [])
    self.assertEqual(self.silencer.silenced_addr, ADDR)
    self.assertEqual(vision_ctrl.get_silenced_addr(), ADDR)
    self.assertFalse(vision_ctrl.private_link_silenced())

  def test_fallback_to_every_network(self):
    sent = self._run(VisionControllerSilencer.PROBE_FRAMES, stock_alive=True)
    # session, variant 0, then variant 1 once the stock STEERING_CONTROL survived the probe; no restore, same
    # candidate
    self.assertEqual(sent, [EXT_DIAG_SESSION_MSG, COMM_CONTROL_DISABLE_MSGS[0], COMM_CONTROL_DISABLE_MSGS[1]])
    self.assertEqual(COMM_CONTROL_DISABLE_MSGS[1], b"\x03\x28\x83\x03\x00\x00\x00\x00")
    self.assertEqual(self.silencer.variant, COMM_CONTROL_ALL_NETWORKS_VARIANT)
    self.assertEqual(self.silencer.idx, 0)
    self.assertTrue(vision_ctrl.expecting_silence())
    # the second variant gets the fast-detect window too
    self.assertTrue(vision_ctrl.fast_detect())

    self.assertEqual(self._run(1, stock_alive=False), [])
    self.assertEqual(self.silencer.silenced_addr, ADDR)
    self.assertTrue(vision_ctrl.private_link_silenced())

  def test_both_variants_fail_restores_candidate(self):
    # the second variant restarts the probe clock at DISABLE_FRAME + 1
    frames = 2 * VisionControllerSilencer.PROBE_FRAMES - VisionControllerSilencer.DISABLE_FRAME - 1
    sent = self._run(frames, stock_alive=True)
    self.assertEqual(sent[-1], COMM_CONTROL_ENABLE_MSG)
    self.assertEqual(sent.count(COMM_CONTROL_DISABLE_MSGS[0]), 1)
    self.assertEqual(sent.count(COMM_CONTROL_DISABLE_MSGS[1]), 1)
    self.assertEqual(self.silencer.variant, 0)
    self.assertFalse(vision_ctrl.expecting_silence())
    self.assertIsNone(vision_ctrl.get_silenced_addr())

  def test_unlock_clears_variant(self):
    self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self._run(1, stock_alive=False)
    self._run(VisionControllerSilencer.RELOCK_FRAMES + 1, stock_alive=False)
    self.assertEqual(vision_ctrl.get_silenced_addr(), ADDR)
    # the controller comes back after the relock window: handshake restarts, nothing is silenced
    self._run(1, stock_alive=True)
    self.assertIsNone(vision_ctrl.get_silenced_addr())
    self.assertFalse(vision_ctrl.private_link_silenced())


if __name__ == "__main__":
  unittest.main()

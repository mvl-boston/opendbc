import unittest

from opendbc.car.honda import vision_ctrl
from opendbc.car.honda.vision_ctrl import (COMM_CONTROL_ALL_NETWORKS_VARIANT, COMM_CONTROL_DISABLE_MSGS, COMM_CONTROL_ENABLE_MSG,
                                           EXT_DIAG_SESSION_MSG, VisionControllerSilencer)

ADDR = 0x18DAB8F1
REPLY_ADDR = 0x18DAF1B8
BUS = 0


class TestVisionControllerSilencer(unittest.TestCase):
  def setUp(self):
    vision_ctrl.set_shutdown_allowed(True)
    self.silencer = VisionControllerSilencer([ADDR])

  def tearDown(self):
    vision_ctrl.set_shutdown_allowed(False)
    vision_ctrl._set_silenced_addr(None)
    vision_ctrl._set_expecting_silence(False)
    vision_ctrl._set_awaiting_response(None)

  def _run(self, frames, stock_alive):
    sent = []
    for _ in range(frames):
      sent.extend(self.silencer.update(stock_alive, BUS))
    return [bytes(dat) for _, dat, _ in sent]

  def _answer(self, dat, reply_addr=REPLY_ADDR, bus=BUS):
    # the controller's UDS reply as CarInterface.update() sees it in the raw packets of the next frame
    vision_ctrl.record_diag_responses([(0, [(reply_addr, dat, bus)])], BUS)

  def test_this_network_variant_first(self):
    sent = self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self.assertEqual(sent, [EXT_DIAG_SESSION_MSG, COMM_CONTROL_DISABLE_MSGS[0]])
    self.assertEqual(COMM_CONTROL_DISABLE_MSGS[0], b"\x03\x28\x03\xF3\x00\x00\x00\x00")
    self.assertTrue(vision_ctrl.expecting_silence())
    self.assertTrue(vision_ctrl.awaiting_response())

    # the controller stops: locked with the this-network variant, its private links are its own
    self.assertEqual(self._run(1, stock_alive=False), [])
    self.assertEqual(self.silencer.silenced_addr, ADDR)
    self.assertEqual(vision_ctrl.get_silenced_addr(), ADDR)
    self.assertFalse(vision_ctrl.private_link_silenced())
    self.assertFalse(vision_ctrl.awaiting_response())

  def test_rejected_variant_falls_back_at_once(self):
    # route ad9840558640c31d/00000011: 7F 28 12 (sub-function not supported) came back within 10 ms, the
    # fallback must not wait out the 1 s probe
    self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self._answer(b"\x03\x7F\x28\x12\x55\x55\x55\x55")
    sent = self._run(1, stock_alive=True)
    self.assertEqual(sent, [COMM_CONTROL_DISABLE_MSGS[1]])
    self.assertEqual(self.silencer.variant, COMM_CONTROL_ALL_NETWORKS_VARIANT)
    self.assertEqual(self.silencer.idx, 0)
    self.assertTrue(vision_ctrl.expecting_silence())
    self.assertTrue(vision_ctrl.fast_detect())
    self.assertTrue(vision_ctrl.awaiting_response())
    # a rejected last variant restores the candidate right away too
    self._answer(b"\x03\x7F\x28\x31\x55\x55\x55\x55")
    sent = self._run(1, stock_alive=True)
    self.assertEqual(sent, [COMM_CONTROL_ENABLE_MSG])
    self.assertEqual(self.silencer.variant, 0)
    self.assertFalse(vision_ctrl.expecting_silence())
    self.assertFalse(vision_ctrl.awaiting_response())

  def test_responses_only_from_the_probed_candidate(self):
    # a positive response, another ECU's reply and a different service's negative response keep the probe going
    self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self._answer(b"\x02\x68\x03\x55\x55\x55\x55\x55")
    self._answer(b"\x03\x7F\x28\x12\x55\x55\x55\x55", reply_addr=0x18DAF1B9)
    self._answer(b"\x03\x7F\x28\x12\x55\x55\x55\x55", bus=2)
    self._answer(b"\x03\x7F\x3E\x12\x55\x55\x55\x55")
    self.assertEqual(self._run(1, stock_alive=True), [])
    self.assertEqual(self.silencer.variant, 0)
    self.assertTrue(self.silencer.probing)
    # nothing is recorded while no answer is awaited
    vision_ctrl._set_awaiting_response(None)
    self._answer(b"\x03\x7F\x28\x12\x55\x55\x55\x55")
    self.assertEqual(vision_ctrl._take_diag_responses(), [])

  def test_describe_uds_response(self):
    self.assertEqual(vision_ctrl.describe_uds_response(b"\x03\x7F\x28\x12\x55\x55\x55\x55"), "7F 28 12 (sub-function not supported)")
    self.assertEqual(vision_ctrl.describe_uds_response(b"\x03\x7F\x28\x31\x55\x55\x55\x55"), "7F 28 31 (request out of range)")
    self.assertEqual(vision_ctrl.describe_uds_response(b"\x02\x68\x03\x55\x55\x55\x55\x55"), "68 03 (positive response)")

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

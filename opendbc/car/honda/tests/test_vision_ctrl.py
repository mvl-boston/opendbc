import unittest

from opendbc.car.honda import vision_ctrl
from opendbc.car.honda.vision_ctrl import (COMM_CONTROL_ALL_NETWORKS_VARIANT, COMM_CONTROL_DISABLE_MSGS, COMM_CONTROL_DISABLE_NAMES,
                                           COMM_CONTROL_ENABLE_MSG, COMM_TYPE_SUBNETS, EXT_DIAG_SESSION_MSG, VisionControllerSilencer)

ADDR = 0x18DAB8F1
REPLY_ADDR = 0x18DAF1B8
BUS = 0
REJECT_OUT_OF_RANGE = b"\x03\x7F\x28\x31\x55\x55\x55\x55"
REJECT_SUB_FUNCTION = b"\x03\x7F\x28\x12\x55\x55\x55\x55"
ACCEPT = b"\x02\x68\x03\x55\x55\x55\x55\x55"
NORMAL_ONLY_VARIANT = COMM_CONTROL_ALL_NETWORKS_VARIANT - 1


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

  def _reject_all_subnets(self):
    """Session, subnet 1, then every subnet rejected out of range in turn: the normal-only variant goes out."""
    sent = self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self.assertEqual(sent, [EXT_DIAG_SESSION_MSG, COMM_CONTROL_DISABLE_MSGS[0]])
    for variant in range(1, NORMAL_ONLY_VARIANT + 1):
      self._answer(REJECT_OUT_OF_RANGE)
      self.assertEqual(self._run(1, stock_alive=True), [COMM_CONTROL_DISABLE_MSGS[variant]])
      self.assertEqual(self.silencer.variant, variant)
      self.assertTrue(vision_ctrl.awaiting_response())

  def test_variant_table(self):
    # subnets 1..14 with normal + NM messages, normal messages only everywhere, then normal + NM everywhere
    # (suppressed positive response: the known fallback); positive responses requested from all the others
    self.assertEqual(COMM_TYPE_SUBNETS, tuple(range(1, 15)))
    self.assertEqual(len(COMM_CONTROL_DISABLE_MSGS), len(COMM_CONTROL_DISABLE_NAMES))
    self.assertEqual(len(COMM_CONTROL_DISABLE_MSGS), 16)
    for subnet in COMM_TYPE_SUBNETS:
      self.assertEqual(COMM_CONTROL_DISABLE_MSGS[subnet - 1], bytes([0x03, 0x28, 0x03, (subnet << 4) | 0x03]) + b"\x00" * 4)
      self.assertEqual(COMM_CONTROL_DISABLE_NAMES[subnet - 1], f"disableRxAndTx on subnet {subnet}")
    self.assertEqual(COMM_CONTROL_DISABLE_MSGS[NORMAL_ONLY_VARIANT], b"\x03\x28\x03\x01\x00\x00\x00\x00")
    self.assertEqual(COMM_CONTROL_DISABLE_MSGS[COMM_CONTROL_ALL_NETWORKS_VARIANT], b"\x03\x28\x83\x03\x00\x00\x00\x00")
    self.assertEqual(COMM_CONTROL_ALL_NETWORKS_VARIANT, 15)
    # neither of the forms the RVU rejected is tried again (28 01 F3 -> 7F 28 12, 28 03 F3 -> 7F 28 31)
    for dat in COMM_CONTROL_DISABLE_MSGS:
      self.assertNotEqual(dat[3] >> 4, 0xF)
      self.assertEqual(dat[2] & 0x7F, 0x03)

  def test_subnet_variant_first(self):
    sent = self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self.assertEqual(sent, [EXT_DIAG_SESSION_MSG, COMM_CONTROL_DISABLE_MSGS[0]])
    self.assertEqual(COMM_CONTROL_DISABLE_MSGS[0], b"\x03\x28\x03\x13\x00\x00\x00\x00")
    self.assertTrue(vision_ctrl.expecting_silence())
    self.assertTrue(vision_ctrl.awaiting_response())

    # the controller stops: locked with the subnet variant, its private links are its own
    self._answer(ACCEPT)
    self.assertEqual(self._run(1, stock_alive=False), [])
    self.assertEqual(self.silencer.silenced_addr, ADDR)
    self.assertEqual(vision_ctrl.get_silenced_addr(), ADDR)
    self.assertFalse(vision_ctrl.private_link_silenced())
    self.assertIsNone(self.silencer.accepted_at)
    # the answer keeps being collected through the relock window, then no more
    self.assertTrue(vision_ctrl.awaiting_response())
    self._run(VisionControllerSilencer.RELOCK_FRAMES + 1, stock_alive=False)
    self.assertFalse(vision_ctrl.awaiting_response())

  def test_early_lock_keeps_the_answer(self):
    # a dropped control batch the frame the NRC arrives locks early; the stock STEERING_CONTROL is back the next
    # frame and the resumed probe must act on the NRC instead of sitting out the full second
    self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self._answer(REJECT_OUT_OF_RANGE)
    self.assertEqual(self._run(1, stock_alive=False), [])
    self.assertEqual(self.silencer.silenced_addr, ADDR)
    # resumed and the NRC acted on in the same frame
    self.assertEqual(self._run(1, stock_alive=True), [COMM_CONTROL_DISABLE_MSGS[1]])
    self.assertIsNone(self.silencer.silenced_addr)
    self.assertTrue(self.silencer.probing)
    self.assertEqual(self.silencer.variant, 1)
    self.assertTrue(vision_ctrl.awaiting_response())

  def test_rejected_variants_fall_back_at_once(self):
    # routes ad9840558640c31d/00000011 and 00000012: the NRC came back within 10-20 ms, the next variant must
    # not wait out the 1 s probe; a rejected variant changed nothing, so no restore in between
    self._reject_all_subnets()
    sent_before = self.silencer.variant
    self.assertEqual(sent_before, NORMAL_ONLY_VARIANT)
    self.assertTrue(vision_ctrl.expecting_silence())
    self.assertTrue(vision_ctrl.fast_detect())
    self._answer(REJECT_OUT_OF_RANGE)
    sent = self._run(1, stock_alive=True)
    self.assertEqual(sent, [COMM_CONTROL_DISABLE_MSGS[COMM_CONTROL_ALL_NETWORKS_VARIANT]])
    self.assertEqual(self.silencer.variant, COMM_CONTROL_ALL_NETWORKS_VARIANT)
    self.assertEqual(self.silencer.idx, 0)
    self.assertTrue(vision_ctrl.fast_detect())
    self.assertNotIn(COMM_CONTROL_ENABLE_MSG, sent)
    # a rejected last variant restores the candidate right away too
    self._answer(REJECT_SUB_FUNCTION)
    sent = self._run(1, stock_alive=True)
    self.assertEqual(sent, [COMM_CONTROL_ENABLE_MSG])
    self.assertEqual(self.silencer.variant, 0)
    self.assertFalse(vision_ctrl.expecting_silence())
    self.assertFalse(vision_ctrl.awaiting_response())

  def test_accepted_but_ineffective_variant_is_restored(self):
    # 68 03 to a subnet that is not the car bus: the controller went quiet somewhere openpilot cannot see.
    # Restore it after ACCEPTED_PROBE_FRAMES (not the full probe), next variant one frame later.
    self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self._answer(ACCEPT)
    self.assertEqual(self._run(1, stock_alive=True), [])
    self.assertEqual(self.silencer.accepted_at, VisionControllerSilencer.DISABLE_FRAME + 1)
    sent = self._run(VisionControllerSilencer.ACCEPTED_PROBE_FRAMES - 1, stock_alive=True)
    self.assertEqual(sent, [])
    sent = self._run(1, stock_alive=True)
    self.assertEqual(sent, [COMM_CONTROL_ENABLE_MSG])
    self.assertEqual(self.silencer.variant, 1)
    self.assertIsNone(self.silencer.accepted_at)
    self.assertFalse(vision_ctrl.awaiting_response())
    self.assertTrue(self.silencer.probing)
    sent = self._run(1, stock_alive=True)
    self.assertEqual(sent, [COMM_CONTROL_DISABLE_MSGS[1]])
    self.assertTrue(vision_ctrl.awaiting_response())
    self.assertTrue(vision_ctrl.fast_detect())
    self.assertEqual(self.silencer.idx, 0)

  def test_responses_only_from_the_probed_candidate(self):
    # another ECU's reply, a reply on the camera bus and a different service's negative response are ignored
    self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self._answer(REJECT_SUB_FUNCTION, reply_addr=0x18DAF1B9)
    self._answer(REJECT_SUB_FUNCTION, bus=2)
    self._answer(b"\x03\x7F\x3E\x12\x55\x55\x55\x55")
    self.assertEqual(self._run(1, stock_alive=True), [])
    self.assertEqual(self.silencer.variant, 0)
    self.assertIsNone(self.silencer.accepted_at)
    self.assertTrue(self.silencer.probing)
    # nothing is recorded while no answer is awaited
    vision_ctrl._set_awaiting_response(None)
    self._answer(REJECT_SUB_FUNCTION)
    self.assertEqual(vision_ctrl._take_diag_responses(), [])

  def test_describe_uds_response(self):
    self.assertEqual(vision_ctrl.describe_uds_response(REJECT_SUB_FUNCTION), "7F 28 12 (sub-function not supported)")
    self.assertEqual(vision_ctrl.describe_uds_response(REJECT_OUT_OF_RANGE), "7F 28 31 (request out of range)")
    self.assertEqual(vision_ctrl.describe_uds_response(ACCEPT), "68 03 (positive response)")

  def test_normal_only_then_every_network(self):
    self._reject_all_subnets()
    # the normal-only variant takes the controller down: locked with it, 0x334 is left to CarState's own check
    self._answer(ACCEPT)
    self.assertEqual(self._run(1, stock_alive=False), [])
    self.assertEqual(self.silencer.silenced_addr, ADDR)
    self.assertFalse(vision_ctrl.private_link_silenced())

  def test_unanswered_variant_expires_into_a_restore(self):
    # no reply at all (lost request): the full probe, then restore and the next variant, as for an accepted one
    sent = self._run(VisionControllerSilencer.PROBE_FRAMES, stock_alive=True)
    self.assertEqual(sent, [EXT_DIAG_SESSION_MSG, COMM_CONTROL_DISABLE_MSGS[0], COMM_CONTROL_ENABLE_MSG])
    self.assertEqual(self._run(1, stock_alive=True), [COMM_CONTROL_DISABLE_MSGS[1]])
    self.assertEqual(self.silencer.idx, 0)
    self.assertTrue(vision_ctrl.expecting_silence())
    self.assertTrue(vision_ctrl.fast_detect())

  def test_every_network_fallback_locks_private_link(self):
    self._reject_all_subnets()
    self._answer(REJECT_OUT_OF_RANGE)
    self.assertEqual(self._run(1, stock_alive=True), [COMM_CONTROL_DISABLE_MSGS[COMM_CONTROL_ALL_NETWORKS_VARIANT]])
    self.assertEqual(self._run(1, stock_alive=False), [])
    self.assertEqual(self.silencer.silenced_addr, ADDR)
    self.assertTrue(vision_ctrl.private_link_silenced())

  def test_all_variants_fail_restores_candidate(self):
    self._reject_all_subnets()
    self._answer(REJECT_OUT_OF_RANGE)
    self._run(1, stock_alive=True)
    # the suppressed fallback gets no answer: the rest of the probe (its clock restarted at DISABLE_FRAME + 1),
    # then the candidate is restored and dropped
    sent = self._run(VisionControllerSilencer.PROBE_FRAMES - VisionControllerSilencer.DISABLE_FRAME - 1, stock_alive=True)
    self.assertEqual(sent, [COMM_CONTROL_ENABLE_MSG])
    self.assertEqual(self.silencer.variant, 0)
    self.assertEqual(self.silencer.cycles, 1)
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

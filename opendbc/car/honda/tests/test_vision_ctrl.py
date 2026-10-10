import unittest
from unittest import mock

from opendbc.car.honda import vision_ctrl
from opendbc.car.honda.vision_ctrl import (COMM_CONTROL_ALL_NETWORKS_VARIANT, COMM_CONTROL_DISABLE_MSG, COMM_CONTROL_DISABLE_MSGS,
                                           COMM_CONTROL_DISABLE_NAMES, COMM_CONTROL_ENABLE_MSG, EXT_DIAG_SESSION_MSG,
                                           VisionControllerSilencer)

ADDR = 0x18DAB8F1
REPLY_ADDR = 0x18DAF1B8
BUS = 0
REJECT_OUT_OF_RANGE = b"\x03\x7F\x28\x31\x55\x55\x55\x55"
REJECT_SUB_FUNCTION = b"\x03\x7F\x28\x12\x55\x55\x55\x55"
ACCEPT = b"\x02\x68\x03\x55\x55\x55\x55\x55"

# a hypothetical scoped variant in front of the every-network one, to exercise the multi-variant machinery
SCOPED_MSG = b"\x03\x28\x03\x13\x00\x00\x00\x00"


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

  def test_variant_table(self):
    # the MDX Type S RVU rejected every scoped communicationType (F3, subnets 1..14, normal messages only) and
    # controlType 01: only disableRxAndTx of normal + NM messages on every network, suppressed positive response
    self.assertEqual(COMM_CONTROL_DISABLE_MSGS, (b"\x03\x28\x83\x03\x00\x00\x00\x00",))
    self.assertEqual(COMM_CONTROL_DISABLE_NAMES, ("disableRxAndTx on every network",))
    self.assertEqual(COMM_CONTROL_ALL_NETWORKS_VARIANT, 0)
    self.assertEqual(COMM_CONTROL_DISABLE_MSG, COMM_CONTROL_DISABLE_MSGS[0])
    self.assertEqual(COMM_CONTROL_ENABLE_MSG, b"\x03\x28\x80\x03\x00\x00\x00\x00")

  def test_handshake_goes_straight_to_every_network(self):
    # route ad9840558640c31d/00000016: the subnet scan cost 0.42 s of rejections before the same fallback
    sent = self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self.assertEqual(sent, [EXT_DIAG_SESSION_MSG, COMM_CONTROL_DISABLE_MSG])
    self.assertEqual(self.silencer.variant, COMM_CONTROL_ALL_NETWORKS_VARIANT)
    self.assertTrue(vision_ctrl.expecting_silence())
    self.assertTrue(vision_ctrl.awaiting_response())
    self.assertTrue(vision_ctrl.fast_detect())

    # the controller stops: locked, its private links went down with it
    self.assertEqual(self._run(1, stock_alive=False), [])
    self.assertEqual(self.silencer.silenced_addr, ADDR)
    self.assertEqual(vision_ctrl.get_silenced_addr(), ADDR)
    self.assertTrue(vision_ctrl.private_link_silenced())
    # the answer keeps being collected through the relock window, then no more
    self.assertTrue(vision_ctrl.awaiting_response())
    self._run(VisionControllerSilencer.RELOCK_FRAMES + 1, stock_alive=False)
    self.assertFalse(vision_ctrl.awaiting_response())

  def test_tester_present_while_locked(self):
    self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self._run(1, stock_alive=False)
    sent = self._run(VisionControllerSilencer.TESTER_PRESENT_PERIOD * 3, stock_alive=False)
    self.assertEqual(sent, [b"\x02\x3E\x80\x00\x00\x00\x00\x00"] * 3)

  def test_rejected_only_variant_restores_candidate_at_once(self):
    # a rejection of the last (here: only) variant drops the candidate right away, no 1 s probe
    self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self._answer(REJECT_OUT_OF_RANGE)
    sent = self._run(1, stock_alive=True)
    self.assertEqual(sent, [COMM_CONTROL_ENABLE_MSG])
    self.assertEqual(self.silencer.variant, 0)
    self.assertEqual(self.silencer.cycles, 1)  # single candidate: a full pass
    self.assertFalse(self.silencer.probing)
    self.assertFalse(vision_ctrl.expecting_silence())
    self.assertFalse(vision_ctrl.awaiting_response())

  def test_early_lock_keeps_the_answer(self):
    # a dropped control batch the frame the NRC arrives locks early; the stock STEERING_CONTROL is back the next
    # frame and the resumed probe must act on the NRC instead of sitting out the full second
    self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self._answer(REJECT_OUT_OF_RANGE)
    self.assertEqual(self._run(1, stock_alive=False), [])
    self.assertEqual(self.silencer.silenced_addr, ADDR)
    # resumed and the NRC acted on in the same frame
    self.assertEqual(self._run(1, stock_alive=True), [COMM_CONTROL_ENABLE_MSG])
    self.assertIsNone(self.silencer.silenced_addr)
    self.assertFalse(self.silencer.probing)

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

  def test_unanswered_probe_expires_into_a_restore(self):
    # the suppressed every-network request gets no answer and the stock STEERING_CONTROL survives: the full
    # probe, then the candidate is restored and dropped
    sent = self._run(VisionControllerSilencer.PROBE_FRAMES, stock_alive=True)
    self.assertEqual(sent, [EXT_DIAG_SESSION_MSG, COMM_CONTROL_DISABLE_MSG, COMM_CONTROL_ENABLE_MSG])
    self.assertEqual(self.silencer.variant, 0)
    self.assertEqual(self.silencer.cycles, 1)
    self.assertFalse(vision_ctrl.expecting_silence())
    self.assertIsNone(vision_ctrl.get_silenced_addr())

  def test_gives_up_after_max_cycles(self):
    for cycle in range(VisionControllerSilencer.MAX_CYCLES):
      self.assertFalse(self.silencer.gave_up, cycle)
      self._run(VisionControllerSilencer.PROBE_FRAMES, stock_alive=True)
    self.assertTrue(self.silencer.gave_up)
    self.assertEqual(self._run(10, stock_alive=True), [])

  def test_unlock_clears_variant(self):
    self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self._run(1, stock_alive=False)
    self._run(VisionControllerSilencer.RELOCK_FRAMES + 1, stock_alive=False)
    self.assertEqual(vision_ctrl.get_silenced_addr(), ADDR)
    # the controller comes back after the relock window: handshake restarts, nothing is silenced
    self._run(1, stock_alive=True)
    self.assertIsNone(vision_ctrl.get_silenced_addr())
    self.assertFalse(vision_ctrl.private_link_silenced())

  def test_nothing_sent_without_stock_steering(self):
    # the harness did isolate the controller after all: leave every ECU alone
    self.assertEqual(self._run(VisionControllerSilencer.PROBE_FRAMES, stock_alive=False), [])
    self.assertIsNone(vision_ctrl.get_silenced_addr())


@mock.patch.multiple(vision_ctrl, COMM_CONTROL_DISABLE_MSGS=(SCOPED_MSG, COMM_CONTROL_DISABLE_MSG),
                     COMM_CONTROL_DISABLE_NAMES=("disableRxAndTx on subnet 1", "disableRxAndTx on every network"),
                     COMM_CONTROL_ALL_NETWORKS_VARIANT=1)
class TestVisionControllerSilencerVariants(TestVisionControllerSilencer):
  """The variant machinery with a scoped form in front of the every-network one (what PR #789-#791 ran; kept
  so a new form can be slotted in): rejection moves on at once, acceptance without effect is restored."""

  def test_variant_table(self):
    self.assertEqual(vision_ctrl.COMM_CONTROL_DISABLE_MSGS, (SCOPED_MSG, COMM_CONTROL_DISABLE_MSG))

  def test_handshake_goes_straight_to_every_network(self):
    sent = self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self.assertEqual(sent, [EXT_DIAG_SESSION_MSG, SCOPED_MSG])
    # the controller stops on the scoped variant: locked with it, its private links are its own
    self._answer(ACCEPT)
    self.assertEqual(self._run(1, stock_alive=False), [])
    self.assertEqual(self.silencer.silenced_addr, ADDR)
    self.assertFalse(vision_ctrl.private_link_silenced())
    self.assertIsNone(self.silencer.accepted_at)

  def test_rejected_only_variant_restores_candidate_at_once(self):
    # routes ad9840558640c31d/00000011, 00000012, 00000016: the NRC came back within 10-30 ms, the next variant
    # must not wait out the 1 s probe; a rejected variant changed nothing, so no restore in between
    self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self.assertTrue(vision_ctrl.fast_detect())
    self._answer(REJECT_OUT_OF_RANGE)
    sent = self._run(1, stock_alive=True)
    self.assertEqual(sent, [COMM_CONTROL_DISABLE_MSG])
    self.assertEqual(self.silencer.variant, 1)
    self.assertEqual(self.silencer.idx, 0)
    self.assertTrue(vision_ctrl.fast_detect())
    self.assertTrue(vision_ctrl.awaiting_response())
    # the fallback takes the controller down: locked, private links gone with it
    self.assertEqual(self._run(1, stock_alive=False), [])
    self.assertEqual(self.silencer.silenced_addr, ADDR)
    self.assertTrue(vision_ctrl.private_link_silenced())

  def test_early_lock_keeps_the_answer(self):
    self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self._answer(REJECT_OUT_OF_RANGE)
    self.assertEqual(self._run(1, stock_alive=False), [])
    self.assertEqual(self.silencer.silenced_addr, ADDR)
    # resumed and the NRC acted on in the same frame: next variant out
    self.assertEqual(self._run(1, stock_alive=True), [COMM_CONTROL_DISABLE_MSG])
    self.assertIsNone(self.silencer.silenced_addr)
    self.assertTrue(self.silencer.probing)
    self.assertEqual(self.silencer.variant, 1)

  def test_accepted_but_ineffective_variant_is_restored(self):
    # 68 03 to a subnet that is not the car bus: the controller went quiet somewhere openpilot cannot see.
    # Restore it after ACCEPTED_PROBE_FRAMES (not the full probe), next variant one frame later.
    self._run(VisionControllerSilencer.DISABLE_FRAME + 1, stock_alive=True)
    self._answer(ACCEPT)
    self.assertEqual(self._run(1, stock_alive=True), [])
    self.assertEqual(self.silencer.accepted_at, VisionControllerSilencer.DISABLE_FRAME + 1)
    self.assertEqual(self._run(VisionControllerSilencer.ACCEPTED_PROBE_FRAMES - 1, stock_alive=True), [])
    self.assertEqual(self._run(1, stock_alive=True), [COMM_CONTROL_ENABLE_MSG])
    self.assertEqual(self.silencer.variant, 1)
    self.assertIsNone(self.silencer.accepted_at)
    self.assertFalse(vision_ctrl.awaiting_response())
    self.assertTrue(self.silencer.probing)
    self.assertEqual(self._run(1, stock_alive=True), [COMM_CONTROL_DISABLE_MSG])
    self.assertTrue(vision_ctrl.awaiting_response())
    self.assertTrue(vision_ctrl.fast_detect())

  def test_unanswered_probe_expires_into_a_restore(self):
    # no reply at all (lost request): the full probe, then restore and the next variant, as for an accepted one
    sent = self._run(VisionControllerSilencer.PROBE_FRAMES, stock_alive=True)
    self.assertEqual(sent, [EXT_DIAG_SESSION_MSG, SCOPED_MSG, COMM_CONTROL_ENABLE_MSG])
    self.assertEqual(self._run(1, stock_alive=True), [COMM_CONTROL_DISABLE_MSG])
    self.assertEqual(self.silencer.idx, 0)
    self.assertTrue(vision_ctrl.expecting_silence())
    self.assertTrue(vision_ctrl.fast_detect())

  def test_gives_up_after_max_cycles(self):
    # two variants per candidate per pass
    for _ in range(VisionControllerSilencer.MAX_CYCLES):
      self.assertFalse(self.silencer.gave_up)
      self._run(VisionControllerSilencer.PROBE_FRAMES, stock_alive=True)
      self._run(VisionControllerSilencer.PROBE_FRAMES, stock_alive=True)
    self.assertTrue(self.silencer.gave_up)


if __name__ == "__main__":
  unittest.main()

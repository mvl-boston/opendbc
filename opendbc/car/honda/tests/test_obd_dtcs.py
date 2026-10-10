import unittest

from opendbc.car import uds
from opendbc.car.can_definitions import CanData
from opendbc.car.honda import obd_dtcs

OBD_BUS = 1


class FakeObdPort:
  """ECUs on the OBD port: tx address -> the bytes after 59 02 of their ReadDTCInformation answer (None: negative
  response). Every ECU answers TesterPresent."""

  def __init__(self, ecus):
    self.ecus = ecus
    self.queue = []
    self.sent = []

  def send(self, msgs):
    for m in msgs:
      self.sent.append(m)
      if m.src != OBD_BUS or m.address not in self.ecus:
        continue
      rx = uds.get_rx_addr_for_tx_addr(m.address)
      if m.dat[:2] == bytes([0x02, uds.SERVICE_TYPE.TESTER_PRESENT]):
        self.queue.append(CanData(rx, b"\x02\x7e\x00".ljust(8, b"\x00"), OBD_BUS))
      elif m.dat[:4] == bytes([0x03]) + obd_dtcs.READ_DTC_REQUEST:
        payload = self.ecus[m.address]
        body = b"\x7f\x19\x12" if payload is None else obd_dtcs.READ_DTC_RESPONSE + payload
        assert len(body) <= 7
        self.queue.append(CanData(rx, (bytes([len(body)]) + body).ljust(8, b"\x00"), OBD_BUS))

  def recv(self, wait_for_one=False):
    out, self.queue = self.queue, []
    return [out] if out else []


class TestDtcDecoding(unittest.TestCase):
  def test_describe_dtc(self):
    self.assertEqual(obd_dtcs.describe_dtc(b"\xc1\x22\x87"), "U0122-87")
    self.assertEqual(obd_dtcs.describe_dtc(b"\x00\x00\x00"), "P0000-00")
    self.assertEqual(obd_dtcs.describe_dtc(b"\x42\x34\x56"), "C0234-56")
    self.assertEqual(obd_dtcs.describe_dtc(b"\x9f\x12\x00"), "B1F12-00")
    self.assertEqual(obd_dtcs.describe_dtc(b"\x30\x6a\x1c"), "P306A-1C")

  def test_describe_dtc_status(self):
    self.assertEqual(obd_dtcs.describe_dtc_status(0x00), "none")
    self.assertEqual(obd_dtcs.describe_dtc_status(0x09), "testFailed,confirmed")
    self.assertEqual(obd_dtcs.describe_dtc_status(0xAF), "testFailed,failedThisCycle,pending,confirmed,failedSinceClear,warningIndicator")

  def test_parse_dtc_response(self):
    self.assertEqual(obd_dtcs.parse_dtc_response(b"\xff"), [])
    self.assertEqual(obd_dtcs.parse_dtc_response(b"\xff\xc1\x22\x87\x09\x30\x6a\x1c\x2c"),
                     [(b"\xc1\x22\x87", 0x09), (b"\x30\x6a\x1c", 0x2c)])
    # a truncated trailing record is dropped
    self.assertEqual(obd_dtcs.parse_dtc_response(b"\xff\xc1\x22\x87\x09\x30\x6a"), [(b"\xc1\x22\x87", 0x09)])


class TestObdInquiry(unittest.TestCase):
  def setUp(self):
    self.switch_calls = []

  def _switch(self, enabled):
    self.switch_calls.append(enabled)
    return True

  def test_inquiry_reads_every_responder(self):
    port = FakeObdPort({0x18DA10F1: b"\xff\xc1\x22\x87\x09", 0x7E0: b"\xff", 0x18DAB0F1: None})
    dtcs = obd_dtcs.inquire_obd_dtcs(port.recv, port.send, OBD_BUS, self._switch, timeout=0.05)
    self.assertEqual(dtcs, {0x18DA10F1: [(b"\xc1\x22\x87", 0x09)], 0x7E0: []})  # the negative responder is left out
    self.assertEqual(self.switch_calls, [True, False])
    # the scan covered the OBD 11-bit ids and the Honda 29-bit range (tester id excepted), all on the OBD bus
    tp = {m.address for m in port.sent if m.dat[:2] == b"\x02\x3e"}
    self.assertTrue({0x7E0, 0x18DAB8F1} <= tp)
    self.assertFalse(0x18DAF1F1 in tp)
    self.assertTrue(all(m.src == OBD_BUS for m in port.sent))
    # read-only: no ClearDiagnosticInformation, no session change, no CommunicationControl
    services = {m.dat[1] for m in port.sent}
    self.assertEqual(services, {uds.SERVICE_TYPE.TESTER_PRESENT, uds.SERVICE_TYPE.READ_DTC_INFORMATION})

  def test_nothing_on_the_port(self):
    port = FakeObdPort({})
    self.assertEqual(obd_dtcs.inquire_obd_dtcs(port.recv, port.send, OBD_BUS, self._switch, timeout=0.05), {})
    self.assertEqual(self.switch_calls, [True, False])
    self.assertFalse(any(m.dat[1] == uds.SERVICE_TYPE.READ_DTC_INFORMATION for m in port.sent))

  def test_multiplexing_not_applied_sends_nothing(self):
    port = FakeObdPort({0x18DA10F1: b"\xff"})
    self.assertEqual(obd_dtcs.inquire_obd_dtcs(port.recv, port.send, OBD_BUS, lambda enabled: False, timeout=0.05), {})
    self.assertEqual(port.sent, [])

  def test_multiplexing_released_after_exception(self):
    port = FakeObdPort({0x18DA10F1: b"\xff"})

    def recv(wait_for_one=False):
      if any(m.dat[1] == uds.SERVICE_TYPE.READ_DTC_INFORMATION for m in port.sent):
        raise RuntimeError("panda gone")
      return port.recv(wait_for_one)

    self.assertEqual(obd_dtcs.inquire_obd_dtcs(recv, port.send, OBD_BUS, self._switch, timeout=0.05), {})
    self.assertEqual(self.switch_calls, [True, False])


if __name__ == "__main__":
  unittest.main()

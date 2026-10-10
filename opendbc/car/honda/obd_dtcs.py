"""Read-only DTC inquiry over the OBD-II port at CarInterface.init(), for the vision-controller Hondas.

Silencing the Radar Vision Unit (see vision_ctrl) takes it off AF-CAN B and its six private sensor pairs as well,
and the PCM, the brake module and the cluster react within 1-2 s (routes ad9840558640c31d/0000000b .. 00000016).
Which data they miss is stored as DTCs in those modules (PCM/TCM, VSA, EPS, meter, SRS, the corner radars via the
RVU ...), most of them behind the gateway on PF-/VF-/IF-CAN where the harness cannot reach them; a scan tool reads
them at the OBD-II port. With comma power plugged into that port the panda can multiplex its bus 1 onto it (ELM327
safety param 0, the switch the firmware query uses for OBD-port cars; pandad applies the ObdMultiplexingEnabled
param whenever the car safety mode is not live yet, which is the case throughout init()), so init() can do the
inquiry a scan tool would: TesterPresent every OBD 11-bit and Honda 29-bit physical address on the OBD bus, then UDS
0x19 ReadDTCInformation reportDTCByStatusMask with status mask 0xFF (every DTC the ECU has: stored, pending or
failed this cycle) to each responder, and log the decoded codes with their status bits. With openpilot longitudinal
on nothing is cleared. Without it (the fault-clear run, see CarInterface._get_params) the codes are cleared after the
read, physically per ECU and by functional broadcast, and read again: the broadcast clear on the car buses only ever
reached the AF-CAN A ECUs (no module behind the gateway answers there), so the PCM/brake module/meter codes of every
failed silencing drive are still stored until this runs with comma power.

Costs the scan timeout plus the read on every drive; with comma power not plugged in nothing answers on the OBD bus
and only the scan timeout is paid. Runs before clear_all_dtcs, so the codes stored by the previous drive are read
before the car-bus broadcast clear reaches any of them.
"""
import time
from collections.abc import Callable

from opendbc.car import uds
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.disable_ecu import CLEAR_DTC_ISOTP_SF, CLEAR_DTC_REQUEST, CLEAR_DTC_RESPONSE, EXT_DIAG_REQUEST, EXT_DIAG_RESPONSE, \
                                    FUNCTIONAL_ADDR_29BIT
from opendbc.car.ecu_addrs import get_ecu_addrs
from opendbc.car.fw_query_definitions import EcuAddrBusType
from opendbc.car.honda.values import HONDA_DIAG_RX_BASE, HONDA_DIAG_TX_BASE
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery

# the legislated OBD request ids (the PCM answers 0x7E0 on 0x7E8); every other Honda ECU answers its 29-bit physical id
OBD_11BIT_TX_ADDRS = tuple(range(0x7E0, 0x7E8))
OBD_11BIT_FUNCTIONAL_ADDR = 0x7DF
# the tester's own id is never scanned
HONDA_TESTER_ID = 0xF1

READ_DTC_REQUEST = bytes([uds.SERVICE_TYPE.READ_DTC_INFORMATION, uds.DTC_REPORT_TYPE.DTC_BY_STATUS_MASK, uds.DTC_STATUS_MASK_TYPE.ALL])
READ_DTC_RESPONSE = bytes([0x40 | uds.SERVICE_TYPE.READ_DTC_INFORMATION, uds.DTC_REPORT_TYPE.DTC_BY_STATUS_MASK])

DTC_STATUS_NAMES = (
  (uds.DTC_STATUS_MASK_TYPE.TEST_FAILED, "testFailed"),
  (uds.DTC_STATUS_MASK_TYPE.TEST_FAILED_THIS_OPERATION_CYCLE, "failedThisCycle"),
  (uds.DTC_STATUS_MASK_TYPE.PENDING_DTC, "pending"),
  (uds.DTC_STATUS_MASK_TYPE.CONFIRMED_DTC, "confirmed"),
  (uds.DTC_STATUS_MASK_TYPE.TEST_NOT_COMPLETED_SINCE_LAST_CLEAR, "notCompletedSinceClear"),
  (uds.DTC_STATUS_MASK_TYPE.TEST_FAILED_SINCE_LAST_CLEAR, "failedSinceClear"),
  (uds.DTC_STATUS_MASK_TYPE.TEST_NOT_COMPLETED_THIS_OPERATION_CYCLE, "notCompletedThisCycle"),
  (uds.DTC_STATUS_MASK_TYPE.WARNING_INDICATOR_REQUESTED, "warningIndicator"),
)

# how long pandad gets to switch the panda between the harness pair and the OBD port on bus 1
OBD_MULTIPLEXING_TIMEOUT = 2.0

Dtc = tuple[bytes, int]  # 3-byte DTC, statusOfDTC


def describe_dtc(dtc: bytes) -> str:
  """ISO 14229 3-byte DTC as the scan-tool string, e.g. b'\\xc1\\x22\\x87' -> 'U0122-87' (SAE J2012 letters and digits
  from the high byte, the failure type byte after the dash)."""
  letter = "PCBU"[dtc[0] >> 6]
  return f"{letter}{(dtc[0] >> 4) & 0x3}{dtc[0] & 0xF:X}{dtc[1]:02X}-{dtc[2]:02X}"


def describe_dtc_status(status: int) -> str:
  names = [name for bit, name in DTC_STATUS_NAMES if status & bit]
  return ",".join(names) if names else "none"


def parse_dtc_response(dat: bytes) -> list[Dtc]:
  """The DTC records of a reportDTCByStatusMask positive response (the bytes after 59 02): the availability mask,
  then 4 bytes per DTC. A trailing partial record is dropped."""
  records = dat[1:]
  return [(bytes(records[i:i + 3]), records[i + 3]) for i in range(0, len(records) - len(records) % 4, 4)]


def scan_obd_ecus(can_recv, can_send, bus: int, timeout: float = 1.0) -> set[int]:
  """TesterPresent the OBD 11-bit ids and every Honda 29-bit physical address on `bus`; the tx addresses that answered."""
  ecu_ids = [i for i in range(256) if i != HONDA_TESTER_ID]
  queries: set[EcuAddrBusType] = {(HONDA_DIAG_TX_BASE + (i << 8), None, bus) for i in ecu_ids}
  queries |= {(addr, None, bus) for addr in OBD_11BIT_TX_ADDRS}
  responses: set[EcuAddrBusType] = {(HONDA_DIAG_RX_BASE + i, None, bus) for i in ecu_ids}
  responses |= {(addr + 8, None, bus) for addr in OBD_11BIT_TX_ADDRS}
  responders = set()
  for addr, _, _ in get_ecu_addrs(can_recv, can_send, queries, responses, timeout=timeout):
    responders.add(addr - 8 if addr < 0x800 else HONDA_DIAG_TX_BASE + ((addr - HONDA_DIAG_RX_BASE) << 8))
  return responders


def read_dtcs(can_recv, can_send, bus: int, addrs: set[int], timeout: float = 1.0) -> dict[int, list[Dtc]]:
  """UDS 0x19 reportDTCByStatusMask(0xFF) to every ECU in `addrs` at once; tx address -> its DTCs, for the ECUs that
  answered positively (a negative response or no answer is logged by the query and leaves the ECU out)."""
  if not addrs:
    return {}
  query = IsoTpParallelQuery(can_send, can_recv, bus, sorted(addrs), [READ_DTC_REQUEST], [READ_DTC_RESPONSE])
  return {tx_addr: parse_dtc_response(dat) for (tx_addr, _), dat in query.get_data(timeout).items()}


def log_dtcs(tag: str, responders: set[int], dtcs: dict[int, list[Dtc]]) -> None:
  for addr in sorted(responders):
    if addr not in dtcs:
      carlog.error(f"{tag} {hex(addr)}: no ReadDTCInformation answer")
    elif not dtcs[addr]:
      carlog.error(f"{tag} {hex(addr)}: no DTCs")
    else:
      codes = ", ".join(f"{describe_dtc(dtc)} [{describe_dtc_status(status)}]" for dtc, status in dtcs[addr])
      carlog.error(f"{tag} {hex(addr)}: {len(dtcs[addr])} DTCs: {codes}")
  carlog.error(f"{tag}: {len(responders)} ECUs on the OBD port, {sum(1 for d in dtcs.values() if d)} with DTCs")


def clear_dtcs(can_recv, can_send, bus: int, addrs: set[int], timeout: float = 1.0) -> set[int]:
  """UDS 0x14 ClearDiagnosticInformation (all groups) on the OBD port: the functional broadcasts first (29-bit
  0x18DB33F1 and 11-bit 0x7DF, for ECUs that did not answer TesterPresent), then physically to every ECU in `addrs`
  in the default session, and once more behind an extended session request to the ones that did not answer
  positively. Returns the ECUs that acknowledged the clear (0x54).

  WARNING: this erases the stored DTCs of every ECU reachable over the port, safety-relevant modules included."""
  for functional_addr in (FUNCTIONAL_ADDR_29BIT, OBD_11BIT_FUNCTIONAL_ADDR):
    can_send([CanData(functional_addr, CLEAR_DTC_ISOTP_SF, bus)])
  if not addrs:
    return set()
  query = IsoTpParallelQuery(can_send, can_recv, bus, sorted(addrs), [CLEAR_DTC_REQUEST], [CLEAR_DTC_RESPONSE])
  cleared = {tx_addr for tx_addr, _ in query.get_data(timeout)}
  remaining = addrs - cleared
  if remaining:
    query = IsoTpParallelQuery(can_send, can_recv, bus, sorted(remaining), [EXT_DIAG_REQUEST, CLEAR_DTC_REQUEST],
                               [EXT_DIAG_RESPONSE, CLEAR_DTC_RESPONSE])
    cleared |= {tx_addr for tx_addr, _ in query.get_data(timeout)}
  return cleared


def inquire_obd_dtcs(can_recv, can_send, bus: int, set_obd_multiplexing: Callable[[bool], bool],
                     timeout: float = 1.0, clear: bool = False) -> dict[int, list[Dtc]]:
  """The inquiry: multiplex `bus` onto the OBD port, scan, read, log, and put the bus back. With `clear`, the DTCs
  are cleared after the read and read again, so the log shows what was stored and what survived the clear. Returns
  the DTCs per responding ECU as read first (empty when the port could not be reached or nothing answered)."""
  if not set_obd_multiplexing(True):
    carlog.error("obd dtc inquiry: OBD multiplexing was not applied, skipped")
    return {}
  dtcs: dict[int, list[Dtc]] = {}
  try:
    responders = scan_obd_ecus(can_recv, can_send, bus, timeout=timeout)
    if not responders:
      carlog.error("obd dtc inquiry: nothing answered on the OBD port (comma power not connected?)")
      return {}
    dtcs = read_dtcs(can_recv, can_send, bus, responders, timeout=timeout)
    log_dtcs("obd dtc inquiry", responders, dtcs)
    if clear:
      cleared = clear_dtcs(can_recv, can_send, bus, responders, timeout=timeout)
      not_cleared = [hex(a) for a in sorted(responders - cleared)]
      carlog.error(f"obd dtc clear: acknowledged by {[hex(a) for a in sorted(cleared)]}, not by {not_cleared}")
      log_dtcs("obd dtc after clear", responders, read_dtcs(can_recv, can_send, bus, responders, timeout=timeout))
  except Exception:
    carlog.exception("obd dtc inquiry exception")
  finally:
    if not set_obd_multiplexing(False):
      carlog.error("obd dtc inquiry: OBD multiplexing was not released")
  return dtcs


def params_obd_multiplexing() -> Callable[[bool], bool] | None:
  """The multiplexing switch card.py's obd_callback implements, as init() has no access to it: request the mode
  through the ObdMultiplexingEnabled param and wait (bounded, so a replay without pandad cannot hang) for pandad's
  ObdMultiplexingChanged acknowledgment. None outside openpilot."""
  try:
    from openpilot.common.params import Params
  except ImportError:
    return None
  params = Params()

  def set_obd_multiplexing(enabled: bool) -> bool:
    if params.get_bool("ObdMultiplexingEnabled") == enabled:
      return True
    params.remove("ObdMultiplexingChanged")
    params.put_bool("ObdMultiplexingEnabled", enabled, block=True)
    deadline = time.monotonic() + OBD_MULTIPLEXING_TIMEOUT
    while time.monotonic() < deadline:
      if params.get_bool("ObdMultiplexingChanged"):
        return True
      time.sleep(0.01)
    return False

  return set_obd_multiplexing

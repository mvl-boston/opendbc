"""EU CR-V 6G radar/vision controller discovery and silencing.

On this car the ECU that authors STEERING_CONTROL (0xE4) and the ACC messages sits on the car side of the
comma harness: opening the relay does not take its STEERING_CONTROL off the powertrain bus, so openpilot
cannot simply replace the stream the way it does with the camera on the other CAN FD Hondas. The approach
is the same one alphalong uses for the Bosch radar: put the ECU in the extended diagnostic session, send
UDS CommunicationControl disableRxAndTx, and keep it silent with TesterPresent.

The controller's diagnostic address is not known for certain, so it is searched for:
  1. CarInterface.init() (ELM327 safety mode, every diagnostic address allowed) scans the whole Honda
     29-bit physical address range (gateway excepted) with TesterPresent on the powertrain bus (and, log
     only, on the radar bus), logs every responder, and moves the responding VISION_CTRL_CANDIDATE_ADDRS to
     the front of the candidate list; the camera, a sensor rather than the author, always goes last.
  2. CarController, once the relay is open, silences the candidates one at a time and checks whether the
     stock STEERING_CONTROL actually stops. A candidate that did not take the steering with it is
     restored (CommunicationControl enableRxAndTx) before the next one is tried. The one that did is
     kept silent with TesterPresent, and openpilot starts authoring STEERING_CONTROL.

Panda safety allowlists exactly VISION_CTRL_CANDIDATE_ADDRS for this handshake, so a bug here can never
silence any other ECU.
"""
from collections.abc import Iterable

from opendbc.car import make_tester_present_msg, uds
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.ecu_addrs import get_ecu_addrs
from opendbc.car.honda.values import HONDA_DIAG_RX_BASE, HONDA_DIAG_TX_BASE, HONDA_FWD_CAMERA_DIAG_ADDR, HONDA_GATEWAY_DIAG_ADDR, \
                                     VISION_CTRL_CANDIDATE_ADDRS

HONDA_TESTER_ID = 0xF1
# Never scanned: the gateway (the EU CR-V field notes rule out any UDS towards it) and the tester's own id
SCAN_SKIP_ECU_IDS = {(HONDA_GATEWAY_DIAG_ADDR - HONDA_DIAG_TX_BASE) >> 8, HONDA_TESTER_ID}

# UDS payloads of the handshake, as ISO-TP single frames (exactly what panda safety gates on)
EXT_DIAG_SESSION_MSG = bytes([0x02, uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.EXTENDED_DIAGNOSTIC]) + b'\x00' * 5
COMM_CONTROL_DISABLE_MSG = bytes([0x03, uds.SERVICE_TYPE.COMMUNICATION_CONTROL, 0x80 | uds.CONTROL_TYPE.DISABLE_RX_DISABLE_TX,
                                  uds.MESSAGE_TYPE.NORMAL_AND_NETWORK_MANAGEMENT]) + b'\x00' * 4
COMM_CONTROL_ENABLE_MSG = bytes([0x03, uds.SERVICE_TYPE.COMMUNICATION_CONTROL, 0x80 | uds.CONTROL_TYPE.ENABLE_RX_ENABLE_TX,
                                 uds.MESSAGE_TYPE.NORMAL_AND_NETWORK_MANAGEMENT]) + b'\x00' * 4


def ecu_id(addr: int) -> int:
  return (addr - HONDA_DIAG_TX_BASE) >> 8


def rx_addr(addr: int) -> int:
  return HONDA_DIAG_RX_BASE + ecu_id(addr)


def scan_ecus(can_recv, can_send, buses: Iterable[int], timeout: float = 1.0) -> dict[int, set[int]]:
  """TesterPresent every Honda 29-bit physical diagnostic address on each bus (all buses in one pass, so the
  timeout is paid once) and return, per bus, the tx addresses of the ECUs that answered. Only usable while
  the panda is in a safety mode that allows arbitrary diagnostic addresses (ELM327 during CarInterface.init())."""
  buses = tuple(buses)
  ecu_ids = [i for i in range(256) if i not in SCAN_SKIP_ECU_IDS]
  queries = {(HONDA_DIAG_TX_BASE + (i << 8), None, bus) for i in ecu_ids for bus in buses}
  responses = {(HONDA_DIAG_RX_BASE + i, None, bus) for i in ecu_ids for bus in buses}
  responders: dict[int, set[int]] = {bus: set() for bus in buses}
  for addr, _, bus in get_ecu_addrs(can_recv, can_send, queries, responses, timeout=timeout):
    responders[bus].add(HONDA_DIAG_TX_BASE + ((addr - HONDA_DIAG_RX_BASE) << 8))
  return responders


def order_candidates(responders: set[int], known_ecu_addrs: set[int]) -> list[int]:
  """Order the allowlisted candidates for the CarController search: the candidates that answered the scan
  first (in preference order), then the rest, with the camera always last (it answers on every car but is a
  sensor, not the author; see VISION_CTRL_CANDIDATE_ADDRS). Any other unknown responder is only logged: it
  cannot be probed without first being added to VISION_CTRL_CANDIDATE_ADDRS (and the panda safety allowlist)."""
  preferred = [a for a in VISION_CTRL_CANDIDATE_ADDRS if a != HONDA_FWD_CAMERA_DIAG_ADDR]
  present = [a for a in preferred if a in responders]
  absent = [a for a in preferred if a not in responders]
  unknown = sorted(responders - set(VISION_CTRL_CANDIDATE_ADDRS) - known_ecu_addrs)
  carlog.error(f"vision controller scan: responders {[hex(a) for a in sorted(responders)]}, candidates present "
               + f"{[hex(a) for a in present]}, unknown ECUs {[hex(a) for a in unknown]}")
  return present + absent + [a for a in VISION_CTRL_CANDIDATE_ADDRS if a == HONDA_FWD_CAMERA_DIAG_ADDR]


# Handoff from CarInterface.init() (which has no access to the CarController instance) to the CarController
# search, which only starts after init() has returned.
_candidates: list[int] = list(VISION_CTRL_CANDIDATE_ADDRS)
_silenced_addr: int | None = None


def set_candidates(candidates: list[int]) -> None:
  global _candidates
  _candidates = list(candidates)


def get_candidates() -> list[int]:
  return list(_candidates)


def get_silenced_addr() -> int | None:
  return _silenced_addr


def _set_silenced_addr(addr: int | None) -> None:
  global _silenced_addr
  _silenced_addr = addr


class VisionControllerSilencer:
  """Frame-driven (100 Hz) search-and-silence state machine, run from CarController.update()."""
  SESSION_FRAME = 0      # extended diagnostic session request
  DISABLE_FRAME = 5      # CommunicationControl disableRxAndTx, 50 ms later (same spacing as the radar disable)
  PROBE_FRAMES = 100     # a candidate gets 1 s to take the stock STEERING_CONTROL down
  TESTER_PRESENT_PERIOD = 10
  MAX_CYCLES = 3         # full passes over the candidates before giving up (avoids flapping ECUs forever)

  def __init__(self, candidates: list[int] | None = None):
    self.candidates = list(candidates) if candidates is not None else get_candidates()
    self.idx = 0
    self.counter = 0
    self.cycles = 0
    self.probing = False
    self.silenced_addr: int | None = None
    self.gave_up = False
    _set_silenced_addr(None)

  @property
  def addr(self) -> int:
    return self.candidates[self.idx]

  def _lock(self) -> None:
    self.silenced_addr = self.addr
    self.probing = False
    self.counter = 0
    _set_silenced_addr(self.silenced_addr)
    carlog.error(f"vision controller silenced at {hex(self.silenced_addr)}: stock STEERING_CONTROL stopped")

  def _unlock(self) -> None:
    carlog.error(f"vision controller at {hex(self.silenced_addr)} came back, redoing the handshake")
    self.silenced_addr = None
    self.counter = 0
    _set_silenced_addr(None)

  def update(self, stock_alive: bool, bus: int) -> list[CanData]:
    """stock_alive: the stock STEERING_CONTROL is still being received on the powertrain bus."""
    msgs: list[CanData] = []

    if self.silenced_addr is not None:
      if stock_alive:
        self._unlock()
      else:
        if self.counter % self.TESTER_PRESENT_PERIOD == 0:
          msgs.append(make_tester_present_msg(self.silenced_addr, bus, suppress_response=True))
        self.counter += 1
        return msgs

    if self.probing and not stock_alive:
      self._lock()
      return msgs

    if not stock_alive or self.gave_up:
      # nothing to silence (the harness did isolate the controller after all), or the author is not
      # among the candidates: leave every ECU alone
      return msgs

    if self.counter == self.SESSION_FRAME:
      msgs.append(CanData(self.addr, EXT_DIAG_SESSION_MSG, bus))
    elif self.counter == self.DISABLE_FRAME:
      msgs.append(CanData(self.addr, COMM_CONTROL_DISABLE_MSG, bus))
      self.probing = True
    elif self.counter >= self.PROBE_FRAMES - 1:
      # the stock STEERING_CONTROL survived this candidate: it is not the author, restore it and move on
      msgs.append(CanData(self.addr, COMM_CONTROL_ENABLE_MSG, bus))
      carlog.error(f"vision controller candidate {hex(self.addr)} did not stop STEERING_CONTROL, restored")
      self.probing = False
      self.counter = -1
      self.idx += 1
      if self.idx >= len(self.candidates):
        self.idx = 0
        self.cycles += 1
        if self.cycles >= self.MAX_CYCLES:
          self.gave_up = True
          carlog.error("vision controller not found among the candidates, giving up: no steering this drive")
    self.counter += 1
    return msgs

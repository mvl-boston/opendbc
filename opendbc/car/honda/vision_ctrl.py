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
_expecting_silence: bool = False
# True for the few frames right after CommunicationControl disable in which the controller is expected to
# stop (measured 0-2 ticks on the MDX): CarState then calls the stock STEERING_CONTROL dead on the first
# missed frame so openpilot's replacement goes out on the very next control frame.
_fast_detect: bool = False
# CarState sets this once every stock frame openpilot replaces has been seen at least once, so the phase
# counters are seeded before CommunicationControl disable takes their author down.
_shutdown_allowed: bool = False


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


def expecting_silence() -> bool:
  """True from the frame CommunicationControl disable is sent to a candidate until that probe is given up, and
  for as long as the controller is held silent. CarState uses it to call the stock STEERING_CONTROL dead after
  two missed frames instead of five: the switchover gap is what the other ECUs time out on (the 50 Hz
  ACC_CONTROL/0x1C9 pair lost 3 frames with the slow detection), and a short RX dropout outside this window
  must not start openpilot's streams alongside a live controller. Two frames is the floor outside
  fast_detect(): ~2% of 10 ms control batches see no stock STEERING_CONTROL while it is alive (the 100 Hz
  bus period beating against the batch cadence, always a single-batch gap), so one missed frame on its own
  cannot tell a dropped batch from the controller stopping."""
  return _expecting_silence


def _set_expecting_silence(expecting: bool) -> None:
  global _expecting_silence
  _expecting_silence = expecting


def fast_detect() -> bool:
  """True for FAST_DETECT_FRAMES frames after CommunicationControl disable is sent, while the controller is
  expected to stop any tick now. CarState calls the stock STEERING_CONTROL dead on the first missed frame
  here, so openpilot's STEERING_CONTROL (and the phased 50 Hz ACC_CONTROL/LANE_PATH pair) go out on the frame
  the stock ones would have. A batch gap in this window locks one frame early and sends one idle
  STEERING_CONTROL next to the stock one; VisionControllerSilencer then resumes the probe (see update)."""
  return _fast_detect


def _set_fast_detect(fast: bool) -> None:
  global _fast_detect
  _fast_detect = fast


def set_shutdown_allowed(allowed: bool) -> None:
  global _shutdown_allowed
  _shutdown_allowed = allowed


def shutdown_allowed() -> bool:
  return _shutdown_allowed


class VisionControllerSilencer:
  """Frame-driven (100 Hz) search-and-silence state machine, run from CarController.update()."""
  SESSION_FRAME = 0      # extended diagnostic session request
  DISABLE_FRAME = 5      # CommunicationControl disableRxAndTx, 50 ms later (same spacing as the radar disable)
  # frames after DISABLE_FRAME in which the stock STEERING_CONTROL is called dead on its first missed frame
  # (route ad9840558640c31d: the RVU stops 0-2 ticks after the disable; +1 for the parse/transmit frame)
  FAST_DETECT_FRAMES = 4
  # a lock this soon after the disable that sees the stock STEERING_CONTROL again was a dropped control
  # batch, not the controller coming back: resume the probe instead of redoing the handshake
  RELOCK_FRAMES = 3
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
    _set_expecting_silence(False)
    _set_fast_detect(False)

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
    _set_expecting_silence(False)

  def _resume_probe(self) -> None:
    carlog.error(f"vision controller at {hex(self.silenced_addr)} still transmitting after an early lock, resuming the probe")
    self.silenced_addr = None
    self.probing = True
    self.counter = self.DISABLE_FRAME + 1
    _set_silenced_addr(None)

  def update(self, stock_alive: bool, bus: int) -> list[CanData]:
    """stock_alive: the stock STEERING_CONTROL is still being received on the powertrain bus."""
    msgs = self._update(stock_alive, bus)
    # Published after this frame's transitions so CarState sees it on the next frame, the first one in which
    # the controller can have gone quiet. counter is already DISABLE_FRAME + 1 on the disable frame itself.
    _set_fast_detect(self.probing and self.silenced_addr is None and
                     0 < self.counter - self.DISABLE_FRAME <= self.FAST_DETECT_FRAMES)
    return msgs

  def _update(self, stock_alive: bool, bus: int) -> list[CanData]:
    msgs: list[CanData] = []

    if self.silenced_addr is not None:
      if stock_alive:
        if self.counter < self.RELOCK_FRAMES:
          self._resume_probe()
        else:
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
      if not shutdown_allowed():
        return msgs
      msgs.append(CanData(self.addr, COMM_CONTROL_DISABLE_MSG, bus))
      self.probing = True
      _set_expecting_silence(True)
    elif self.counter >= self.PROBE_FRAMES - 1:
      # the stock STEERING_CONTROL survived this candidate: it is not the author, restore it and move on
      msgs.append(CanData(self.addr, COMM_CONTROL_ENABLE_MSG, bus))
      carlog.error(f"vision controller candidate {hex(self.addr)} did not stop STEERING_CONTROL, restored")
      self.probing = False
      _set_expecting_silence(False)
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

"""
Honda Bosch powertrain-bus steering bypass (HondaFlags.BOSCH_PT_STEER_BYPASS, stock ACC only).

On the ACURA_RDX_3G_MMR, ACURA_MDX_4G, HONDA_ODYSSEY_5G_MMR and ACURA_TLX_2G the radar sits between
the camera bus (where openpilot's STEERING_CONTROL goes, CanBus.lkas) and the powertrain bus the EPS
listens on. It re-authors STEERING_CONTROL (0xE4) onto the powertrain bus at 100 Hz with its own 2-bit
counter, and below ~70 kph it blanks the torque, so with stock ACC these cars do not steer at low speed
even though the EPS itself accepts torque down to ~3 mph.

The EPS acts on the first STEERING_CONTROL it sees for a counter value and ignores a repeat of that
counter. So, in addition to the normal frame towards the radar, the car controller sends openpilot's
torque directly on the powertrain bus carrying the counter the radar is about to use, so that our frame
is the one the EPS acts on and the radar's copy is the repeat that gets ignored.

Timing. The radar's frames reach the car controller inside the 100 Hz `can` packets, which carry no
per-frame timestamps, and our own frame leaves ~TX_LATENCY_MS after the packet (card -> pandad -> USB
-> bus). We therefore cannot place our frame at an arbitrary point inside the radar's 10 ms gap; what we
can do is pick the counter so that our frame always leads the radar frame that carries the same counter:
  - if the radar's next frame lands after our transmit, send last_radar + 1 (we lead it by up to ~6 ms);
  - if it lands before our transmit (right after the poll), the radar's +1 is already consumed by the time
    ours gets out, so send last_radar + 2 and lead the frame after it instead.
Where the radar's frame sits inside the poll period is only observable through aliasing between the
radar's 100 Hz and pandad's 100 Hz poll: a poll holding two radar frames means the newest landed just
before the poll (next one a full period out), a poll holding none means the frame slipped past the poll
and lands right after it. Between these wrap events the phase drifts linearly at the clock mismatch,
which is estimated from the interval between wraps, so the +1 / +2 switch mid-cycle is predicted too.
In recorded routes the two clocks are close enough that the frame sits near a poll boundary for long
stretches and jitters across it, showing up as double/empty poll pairs: each one is a real flip of which
side of our transmit the radar's next frame lands on, and is tracked as such (with zero drift).

The EPS reports whether it is acting on a torque request (STEER_STATUS.STEER_CONTROL_ACTIVE). While we
request torque and the EPS stays inactive we are losing the race, and the transmit-latency estimate is
pushed across the modeled phase so the offset flips. This closes the loop on everything the model
cannot see (actual latency, poll jitter, an EPS that dedupes differently than assumed).
"""

from collections import deque

COUNTER_MOD = 4  # STEERING_CONTROL.COUNTER is 2 bits
PERIOD_MS = 10.0  # STEERING_CONTROL is 100 Hz

# nominal time from the `can` packet timestamp to our frame being on the powertrain bus
TX_LATENCY_MS = 4.0
# a feedback correction moves the latency estimate this far across the modeled phase; the clamp is
# wide enough that a correction at either phase anchor (0 / PERIOD_MS) still flips the offset
LATENCY_FLIP_MARGIN_MS = 1.0
LATENCY_MIN_MS = -LATENCY_FLIP_MARGIN_MS
LATENCY_MAX_MS = PERIOD_MS + LATENCY_FLIP_MARGIN_MS

# clock mismatch between the radar and pandad; 1% is far beyond any crystal, so this only binds on jitter
MAX_DRIFT_MS_PER_TICK = 0.1
# two same-kind wraps closer than this are poll jitter, not a full phase cycle
MIN_WRAP_INTERVAL_TICKS = 100
# radar frames missing for this many ticks: no phase model, free-run our own counter
RADAR_STALE_TICKS = 5

# EPS feedback: judge the race over a window of requesting ticks, after letting the EPS settle
FEEDBACK_SETTLE_TICKS = 10
FEEDBACK_WINDOW_TICKS = 10
# alternating accept/ignore (a wrong +2 under a same-as-last dedupe) shows up as ~half inactive
FEEDBACK_MIN_INACTIVE_TICKS = 4


class RadarSteerBypass:
  def __init__(self):
    self.last_radar_counter: int | None = None
    self.ticks_since_radar = RADAR_STALE_TICKS
    self.last_sent_counter: int | None = None

    # phase model: estimated time from the poll to the radar's next frame
    self.lead_ms: float | None = None
    # per-tick change of lead_ms: negative when the radar runs faster than the poll
    self.drift_ms_per_tick = 0.0
    self.last_wrap_kind = 0  # -1: double poll (radar faster), +1: empty poll (radar slower)
    self.ticks_since_wrap = 0
    # phase state saved at an empty poll, restored if the next poll shows the frame was dropped rather than slipped
    self.pre_empty_poll: tuple[int, float | None, float, int, int] | None = None
    self.tx_latency_ms = TX_LATENCY_MS
    # used until the first wrap event gives the model something to work with
    self.unmodeled_offset = 1

    self.eps_history: deque[bool] = deque(maxlen=FEEDBACK_WINDOW_TICKS)
    self.settle_ticks = FEEDBACK_SETTLE_TICKS

  @property
  def offset(self) -> int:
    if self.lead_ms is None:
      return self.unmodeled_offset
    return 1 if self.lead_ms > self.tx_latency_ms else 2

  def update(self, radar_counters: list[int], requesting: bool, eps_active: bool, feedback_valid: bool) -> None:
    """
    radar_counters: COUNTER of every radar STEERING_CONTROL seen on the powertrain bus this tick, in order
    requesting: whether our frames of the last ticks carried STEER_TORQUE_REQUEST
    eps_active: STEER_STATUS.STEER_CONTROL_ACTIVE this tick
    feedback_valid: EPS inactivity can be read as losing the race (no driver override, no steer fault, above
                    the EPS's own low-speed lockout)
    """
    self._update_phase(radar_counters)
    self._update_feedback(requesting, eps_active, feedback_valid)

  def _update_phase(self, radar_counters: list[int]) -> None:
    had_radar = self.ticks_since_radar == 0
    prev_radar_counter = self.last_radar_counter
    if radar_counters:
      self.last_radar_counter = int(radar_counters[-1]) % COUNTER_MOD
      self.ticks_since_radar = 0
    else:
      self.ticks_since_radar += 1
    self.ticks_since_wrap += 1

    if self.pre_empty_poll is not None:
      # tick after an empty poll: a slipped frame shows up now with the next counter; a jump of two means the
      # radar frame was lost outright and the phase did not move, so undo the anchor
      counter_before_empty, lead, drift, wrap_kind, ticks_since_wrap = self.pre_empty_poll
      self.pre_empty_poll = None
      if len(radar_counters) == 1 and self.last_radar_counter == (counter_before_empty + 2) % COUNTER_MOD:
        self.lead_ms, self.drift_ms_per_tick, self.last_wrap_kind = lead, drift, wrap_kind
        self.ticks_since_wrap = ticks_since_wrap + 2
        if self.lead_ms is not None:
          self.lead_ms = min(max(self.lead_ms + 2 * self.drift_ms_per_tick, 0.0), PERIOD_MS)
        return

    wrap = 0
    if len(radar_counters) >= 2:
      wrap = -1
    elif not radar_counters and had_radar:
      wrap = 1
      # had_radar implies a counter has been seen
      self.pre_empty_poll = (prev_radar_counter or 0, self.lead_ms, self.drift_ms_per_tick, self.last_wrap_kind,
                             self.ticks_since_wrap - 1)

    if wrap != 0:
      if wrap == self.last_wrap_kind and self.ticks_since_wrap >= MIN_WRAP_INTERVAL_TICKS:
        self.drift_ms_per_tick = min(PERIOD_MS / self.ticks_since_wrap, MAX_DRIFT_MS_PER_TICK)
      # the wrap kind is the drift direction even when its magnitude is not known yet
      self.drift_ms_per_tick = wrap * abs(self.drift_ms_per_tick)
      self.lead_ms = PERIOD_MS if wrap < 0 else 0.0
      self.last_wrap_kind = wrap
      self.ticks_since_wrap = 0
    elif self.ticks_since_radar > RADAR_STALE_TICKS:
      self.lead_ms = None
    elif self.lead_ms is not None:
      self.lead_ms = min(max(self.lead_ms + self.drift_ms_per_tick, 0.0), PERIOD_MS)

  def _update_feedback(self, requesting: bool, eps_active: bool, feedback_valid: bool) -> None:
    if not (requesting and feedback_valid):
      self.eps_history.clear()
      self.settle_ticks = FEEDBACK_SETTLE_TICKS
      return
    if self.settle_ticks > 0:
      self.settle_ticks -= 1
      return
    self.eps_history.append(eps_active)
    if len(self.eps_history) == FEEDBACK_WINDOW_TICKS and \
         sum(1 for active in self.eps_history if not active) >= FEEDBACK_MIN_INACTIVE_TICKS:
      self._flip_offset()
      self.eps_history.clear()
      self.settle_ticks = FEEDBACK_SETTLE_TICKS

  def _flip_offset(self) -> None:
    # losing the race with the current offset: the modeled lead is on the wrong side of our transmit
    # latency, so move the latency estimate across it (the lead itself is anchored by the radar stream)
    if self.lead_ms is None:
      self.unmodeled_offset = 2 if self.unmodeled_offset == 1 else 1
    elif self.offset == 1:
      self.tx_latency_ms = min(self.lead_ms + LATENCY_FLIP_MARGIN_MS, LATENCY_MAX_MS)
    else:
      self.tx_latency_ms = max(self.lead_ms - LATENCY_FLIP_MARGIN_MS, LATENCY_MIN_MS)

  def next_counter(self) -> int:
    if self.last_radar_counter is not None and self.ticks_since_radar == 0:
      counter = (self.last_radar_counter + self.offset) % COUNTER_MOD
      if counter == self.last_sent_counter:
        # the offset just stepped down: the EPS has already consumed this counter from us, move on
        counter = (counter + 1) % COUNTER_MOD
    elif self.last_sent_counter is not None:
      # the radar's frame slipped past this poll (or the stream is gone): keep our sequence running
      counter = (self.last_sent_counter + 1) % COUNTER_MOD
    else:
      counter = 0
    return counter

  def sent(self, counter: int) -> None:
    self.last_sent_counter = counter

import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, CanData, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR, DBC, HondaFlags, HondaSafetyFlags
from opendbc.car.honda.radar_steer_bypass import (RadarSteerBypass, COUNTER_MOD, PERIOD_MS, TX_LATENCY_MS, RADAR_STALE_TICKS,
                                                  FEEDBACK_SETTLE_TICKS, FEEDBACK_WINDOW_TICKS, MIN_WRAP_INTERVAL_TICKS)

STEERING_CONTROL = 0xE4
BYPASS_CARS = (CAR.ACURA_RDX_3G_MMR, CAR.ACURA_MDX_4G, CAR.HONDA_ODYSSEY_5G_MMR, CAR.ACURA_TLX_2G)


class RadarSim:
  """
  A radar putting STEERING_CONTROL on the powertrain bus every radar_period_ms, polled every PERIOD_MS by pandad.
  Returns, per poll, the counters of the frames that landed since the previous poll. first_frame_ms is the radar
  frame's offset from the first poll, i.e. the initial phase.
  """
  def __init__(self, radar_period_ms: float = PERIOD_MS, first_frame_ms: float = 5.0, counter: int = 0):
    self.radar_period_ms = radar_period_ms
    self.next_frame_ms = first_frame_ms
    self.counter = counter
    self.poll_ms = 0.0
    self.dropped = 0

  def poll(self) -> list[int]:
    self.poll_ms += PERIOD_MS
    frames = []
    while self.next_frame_ms <= self.poll_ms:
      if self.dropped > 0:
        self.dropped -= 1
      else:
        frames.append(self.counter)
      self.counter = (self.counter + 1) % COUNTER_MOD
      self.next_frame_ms += self.radar_period_ms
    return frames

  def next_frame_lead_ms(self) -> float:
    # time from the poll that just happened to the radar's next frame
    return self.next_frame_ms - self.poll_ms


def run(bypass: RadarSteerBypass, sim: RadarSim, ticks: int, requesting=True, eps_active=True, feedback_valid=True):
  sent = []
  for _ in range(ticks):
    bypass.update(sim.poll(), requesting, eps_active, feedback_valid)
    counter = bypass.next_counter()
    bypass.sent(counter)
    sent.append(counter)
  return sent


def run_until_poll_count(bypass: RadarSteerBypass, sim: RadarSim, count: int, max_ticks: int = 1000):
  # advance until a poll holds exactly `count` radar frames (0: the frame slipped past the poll, 2: two landed in one)
  for _ in range(max_ticks):
    frames = sim.poll()
    bypass.update(frames, True, True, True)
    counter = bypass.next_counter()
    bypass.sent(counter)
    if len(frames) == count:
      return counter
  raise AssertionError(f"no poll with {count} frames within {max_ticks} ticks")


def steer_counter(dat: bytes) -> int:
  # STEERING_CONTROL.COUNTER is bits 37..36: the high nibble of byte 4 (the low nibble is the checksum)
  return (dat[4] >> 4) & 0x3


class TestCounterPrediction(unittest.TestCase):
  def test_radar_frame_after_our_transmit_sends_next_counter(self):
    # radar frame lands 7 ms after the poll, well after our ~4 ms transmit: the radar's next frame is last + 1
    bypass = RadarSteerBypass()
    sim = RadarSim(first_frame_ms=7.0)
    sent = run(bypass, sim, 20)
    self.assertEqual(bypass.offset, 1)
    self.assertEqual(sent[-1], (bypass.last_radar_counter + 1) % COUNTER_MOD)
    # the stream we hand the EPS is continuous
    for prev, cur in zip(sent[1:-1], sent[2:], strict=True):
      self.assertEqual(cur, (prev + 1) % COUNTER_MOD)

  def test_empty_poll_means_radar_frame_lands_before_our_transmit(self):
    # radar slightly slower than the poll: the frame slides later each tick until one poll has none. That frame
    # lands right after the poll, before our frame gets out, so the counter to beat is last + 2.
    bypass = RadarSteerBypass()
    sim = RadarSim(radar_period_ms=PERIOD_MS + 0.05, first_frame_ms=9.8)
    counter = run_until_poll_count(bypass, sim, 0)
    self.assertEqual(bypass.lead_ms, 0.0)
    self.assertEqual(bypass.offset, 2)
    # the slipped frame lands within the next ms, before ours: we lead the one after it
    self.assertLess(sim.next_frame_lead_ms(), TX_LATENCY_MS)
    self.assertEqual(counter, (bypass.last_radar_counter + 2) % COUNTER_MOD)
    # which is the counter the radar puts on the frame after the slipped one
    self.assertEqual(counter, (sim.counter + 1) % COUNTER_MOD)

  def test_double_poll_means_full_period_lead(self):
    # radar slightly faster than the poll: eventually two frames fall in one poll; the newest landed just before
    # the poll so the radar's next frame is a full period away and last + 1 wins comfortably
    bypass = RadarSteerBypass()
    sim = RadarSim(radar_period_ms=PERIOD_MS - 0.05, first_frame_ms=0.1)
    counter = run_until_poll_count(bypass, sim, 2)
    self.assertEqual(bypass.lead_ms, PERIOD_MS)
    self.assertEqual(bypass.offset, 1)
    self.assertGreater(sim.next_frame_lead_ms(), TX_LATENCY_MS)
    self.assertEqual(counter, (bypass.last_radar_counter + 1) % COUNTER_MOD)
    self.assertEqual(counter, sim.counter)

  def test_no_repeated_counter_when_offset_steps_down(self):
    bypass = RadarSteerBypass()
    bypass.last_radar_counter = 0
    bypass.ticks_since_radar = 0
    bypass.last_sent_counter = 2  # sent with offset 2 last tick
    bypass.unmodeled_offset = 1
    # radar now reports 1; +1 would repeat the 2 the EPS has already consumed from us
    bypass._update_phase([1])
    self.assertEqual(bypass.next_counter(), 3)

  def test_stale_radar_free_runs_our_counter(self):
    bypass = RadarSteerBypass()
    sim = RadarSim(first_frame_ms=7.0)
    run(bypass, sim, 5)
    last = bypass.last_sent_counter
    for i in range(1, RADAR_STALE_TICKS + 5):
      bypass.update([], True, True, True)
      counter = bypass.next_counter()
      bypass.sent(counter)
      self.assertEqual(counter, (last + i) % COUNTER_MOD)
    self.assertIsNone(bypass.lead_ms)

  def test_dropped_frame_is_not_a_phase_wrap(self):
    bypass = RadarSteerBypass()
    sim = RadarSim(radar_period_ms=PERIOD_MS - 0.05, first_frame_ms=0.1)
    run_until_poll_count(bypass, sim, 2)  # anchors lead at a full period
    run(bypass, sim, 50)
    lead_before = bypass.lead_ms
    self.assertIsNotNone(lead_before)
    sim.dropped = 1
    run(bypass, sim, 1)
    self.assertEqual(bypass.lead_ms, 0.0)  # provisionally read as a wrap
    run(bypass, sim, 1)
    # the next frame skipped a counter: the frame was lost, the phase model is restored
    self.assertAlmostEqual(bypass.lead_ms, lead_before + 2 * bypass.drift_ms_per_tick, places=6)
    self.assertEqual(bypass.offset, 1)


class TestPhaseDrift(unittest.TestCase):
  def test_drift_is_learned_and_switch_is_predicted(self):
    # radar runs 0.02 ms/tick slow (2000 ppm, exaggerated to keep the test short): the lead grows from 0 after
    # each empty poll and the model must predict when it crosses the transmit latency and flip to +1
    drift = 0.02
    bypass = RadarSteerBypass()
    sim = RadarSim(radar_period_ms=PERIOD_MS + drift, first_frame_ms=9.9)
    # two full cycles so that the second empty poll gives a drift estimate
    ticks_per_cycle = int(round(PERIOD_MS / drift))
    self.assertGreaterEqual(ticks_per_cycle, MIN_WRAP_INTERVAL_TICKS)
    run(bypass, sim, 2 * ticks_per_cycle + 10)
    self.assertAlmostEqual(bypass.drift_ms_per_tick, drift, delta=drift * 0.1)
    self.assertEqual(bypass.last_wrap_kind, 1)
    # just after the wrap the radar's frame beats our transmit, so +2
    self.assertEqual(bypass.offset, 2)
    # walk forward until the modeled lead passes the latency: the model flips to +1 and the simulated radar
    # frame really does land after our transmit by then
    for _ in range(ticks_per_cycle):
      if bypass.offset == 1:
        break
      run(bypass, sim, 1)
    self.assertEqual(bypass.offset, 1)
    self.assertGreater(sim.next_frame_lead_ms(), TX_LATENCY_MS * 0.9)

  def test_offset_follows_true_phase_over_a_cycle(self):
    # with the drift learned, compare the model's +1/+2 choice against the simulated truth every tick of a cycle
    drift = 0.02
    bypass = RadarSteerBypass()
    sim = RadarSim(radar_period_ms=PERIOD_MS - drift, first_frame_ms=0.05)
    ticks_per_cycle = int(round(PERIOD_MS / drift))
    run(bypass, sim, 2 * ticks_per_cycle + 5)
    wrong = 0
    for _ in range(ticks_per_cycle):
      run(bypass, sim, 1)
      truth = 1 if sim.next_frame_lead_ms() > TX_LATENCY_MS else 2
      if bypass.offset != truth:
        wrong += 1
    # the boundary itself is unobservable to within the drift estimate's error
    self.assertLess(wrong, 0.05 * ticks_per_cycle)


class TestEpsFeedback(unittest.TestCase):
  def test_losing_race_flips_offset(self):
    bypass = RadarSteerBypass()
    sim = RadarSim(first_frame_ms=7.0)
    run(bypass, sim, 5)
    self.assertEqual(bypass.offset, 1)
    # EPS never goes active while we request: after the settle + window, the latency estimate is pushed past the lead
    run(bypass, sim, FEEDBACK_SETTLE_TICKS + FEEDBACK_WINDOW_TICKS, eps_active=False)
    self.assertEqual(bypass.offset, 2)
    # and back again if that does not help either
    run(bypass, sim, FEEDBACK_SETTLE_TICKS + FEEDBACK_WINDOW_TICKS, eps_active=False)
    self.assertEqual(bypass.offset, 1)

  def test_flip_works_at_both_phase_anchors(self):
    for poll_count, period, expected_offset in ((2, PERIOD_MS - 0.05, 1), (0, PERIOD_MS + 0.05, 2)):
      bypass = RadarSteerBypass()
      sim = RadarSim(radar_period_ms=period, first_frame_ms=5.0)
      run_until_poll_count(bypass, sim, poll_count)
      self.assertEqual(bypass.offset, expected_offset, (poll_count, period))
      run(bypass, sim, FEEDBACK_SETTLE_TICKS + FEEDBACK_WINDOW_TICKS, eps_active=False)
      self.assertNotEqual(bypass.offset, expected_offset, (poll_count, period))

  def test_alternating_acceptance_counts_as_losing(self):
    bypass = RadarSteerBypass()
    sim = RadarSim(first_frame_ms=7.0)
    run(bypass, sim, 5)
    for i in range(FEEDBACK_SETTLE_TICKS + FEEDBACK_WINDOW_TICKS):
      run(bypass, sim, 1, eps_active=bool(i % 2))
    self.assertEqual(bypass.offset, 2)

  def test_feedback_ignored_when_not_requesting_or_invalid(self):
    bypass = RadarSteerBypass()
    sim = RadarSim(first_frame_ms=7.0)
    run(bypass, sim, 5)
    run(bypass, sim, 3 * (FEEDBACK_SETTLE_TICKS + FEEDBACK_WINDOW_TICKS), requesting=False, eps_active=False)
    self.assertEqual(bypass.offset, 1)
    run(bypass, sim, 3 * (FEEDBACK_SETTLE_TICKS + FEEDBACK_WINDOW_TICKS), eps_active=False, feedback_valid=False)
    self.assertEqual(bypass.offset, 1)

  def test_healthy_eps_does_not_flip(self):
    bypass = RadarSteerBypass()
    sim = RadarSim(first_frame_ms=7.0)
    run(bypass, sim, 500, eps_active=True)
    self.assertEqual(bypass.offset, 1)
    self.assertEqual(bypass.tx_latency_ms, TX_LATENCY_MS)


class TestCarController(unittest.TestCase):
  def _interface(self, car, alpha_long):
    fingerprint = {bus: {} for bus in range(7)}
    CarInterface = interfaces[car]
    CP = CarInterface.get_params(car, fingerprint, [], alpha_long=alpha_long, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, car, fingerprint, [], alpha_long=alpha_long, is_release_sp=False, docs=False)
    return CarInterface(CP, CP_SP)

  def test_flag_limited_to_the_four_cars(self):
    flagged = {c for c in CAR if c.config.flags & HondaFlags.BOSCH_PT_STEER_BYPASS}
    self.assertEqual(flagged, set(BYPASS_CARS))

  def test_safety_param_only_with_stock_acc(self):
    for car in BYPASS_CARS:
      self.assertTrue(self._interface(car, False).CP.safetyConfigs[-1].safetyParam & HondaSafetyFlags.BOSCH_PT_STEER_BYPASS)
      self.assertFalse(self._interface(car, True).CP.safetyConfigs[-1].safetyParam & HondaSafetyFlags.BOSCH_PT_STEER_BYPASS)
    self.assertFalse(self._interface(CAR.HONDA_ACCORD, False).CP.safetyConfigs[-1].safetyParam & HondaSafetyFlags.BOSCH_PT_STEER_BYPASS)

  def test_steering_control_sent_on_both_buses_tracking_the_radar(self):
    for car in BYPASS_CARS:
      with self.subTest(car=car):
        CI = self._interface(car, alpha_long=False)
        radar = CANPacker(DBC[car][Bus.pt])
        CC = structs.CarControl()
        CC.enabled = True
        CC.latActive = True
        CC = CC.as_reader()
        CC_SP = structs.CarControlSP()
        radar_counter = 2
        now_nanos = 0
        bus0_counters, bus1_counters = [], []
        for _ in range(8):
          # the radar's STEERING_CONTROL on the powertrain bus (bus 1), one per tick
          radar_frame = radar.make_can_msg("STEERING_CONTROL", 1, {"COUNTER": radar_counter})
          CI.update([(now_nanos, [CanData(radar_frame[0], radar_frame[1], 1)])])
          _, can_sends = CI.apply(CC, CC_SP, now_nanos)
          steer = [m for m in can_sends if m[0] == STEERING_CONTROL]
          self.assertEqual([m[2] for m in steer], [1, 0])  # direct-to-EPS frame first, then the one for the radar
          bus1_counters.append(steer_counter(steer[0][1]))
          bus0_counters.append(steer_counter(steer[1][1]))
          # same torque payload on both
          self.assertEqual(steer[0][1][:4], steer[1][1][:4])
          self.assertEqual(bus1_counters[-1], (radar_counter + 1) % COUNTER_MOD)
          radar_counter = (radar_counter + 1) % COUNTER_MOD
          now_nanos += int(DT_CTRL * 1e9)
        # the bus-0 stream towards the radar keeps its own continuous counter, undisturbed by the pinned one
        for prev, cur in zip(bus0_counters[:-1], bus0_counters[1:], strict=True):
          self.assertEqual(cur, (prev + 1) % COUNTER_MOD)

  def test_no_bypass_with_openpilot_longitudinal(self):
    CI = self._interface(CAR.ACURA_MDX_4G, alpha_long=True)
    self.assertIsNone(CI.CC.pt_steer_bypass)
    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = True
    CI.update([])
    _, can_sends = CI.apply(CC.as_reader(), structs.CarControlSP(), 0)
    self.assertEqual(len([m for m in can_sends if m[0] == STEERING_CONTROL]), 1)


if __name__ == "__main__":
  unittest.main()

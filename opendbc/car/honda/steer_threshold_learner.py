"""Learn a multiplier on Honda's fixed ``STEER_THRESHOLD`` when the EPS oscillates around it.

On Nidec Hondas (notably the MDX 3G) the torque sensor can chatter through the hand-torque threshold
during hard LKAS turns even with no driver input. That sets ``steeringPressed`` for a large fraction
of ticks and starves lateral learners that gate on an unpressed wheel.

When the wheel crosses the effective threshold many times in a short window, that pattern is treated
as sensor chatter rather than deliberate driver input. The learned ``HondaSteerThresholdFactorParams``
multiplier is raised so later crossings need more torque before ``steeringPressed`` is set.

Register ``HondaSteerThresholdFactorParams`` in openpilot ``common/params_keys.h`` or the value will
not persist across drives.
"""

from collections import deque

PARAM_KEY = "HondaSteerThresholdFactorParams"

INCIDENT_COUNT = 10
INCIDENT_WINDOW = 10.0  # s
FACTOR_STEP = 1.10
FACTOR_DEFAULT = 1.0
FACTOR_MIN = 1.0
FACTOR_MAX = 5.0


class HondaSteerThresholdLearner:
  def __init__(self, get_param, put_param=None):
    self._get_param = get_param
    self._put_param = put_param
    raw = get_param(PARAM_KEY)
    self.factor = FACTOR_DEFAULT if raw is None else float(raw)
    self.factor = max(FACTOR_MIN, min(FACTOR_MAX, self.factor))
    self._incident_times: deque[float] = deque()
    self._prev_instant = False

  def effective_threshold(self, base_threshold: float) -> float:
    return base_threshold * self.factor

  def update(self, steering_torque: float, base_threshold: float, steer_control_active: bool, t: float) -> None:
    """Record threshold crossings at monotonic time ``t`` (seconds)."""
    eff = self.effective_threshold(base_threshold)
    instant = abs(steering_torque) > eff

    if steer_control_active and instant and not self._prev_instant:
      self._incident_times.append(t)
      cutoff = t - INCIDENT_WINDOW
      while self._incident_times and self._incident_times[0] < cutoff:
        self._incident_times.popleft()
      if len(self._incident_times) >= INCIDENT_COUNT:
        self.factor = min(self.factor * FACTOR_STEP, FACTOR_MAX)
        self._incident_times.clear()
        if self._put_param is not None:
          self._put_param(PARAM_KEY, self.factor)

    self._prev_instant = instant

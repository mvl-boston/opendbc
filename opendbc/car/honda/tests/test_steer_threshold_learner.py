import unittest

from opendbc.car.honda.steer_threshold_learner import (
  FACTOR_STEP,
  INCIDENT_COUNT,
  INCIDENT_WINDOW,
  PARAM_KEY,
  HondaSteerThresholdLearner,
)


class TestHondaSteerThresholdLearner(unittest.TestCase):
  def make_learner(self, factor=None):
    store = {}
    if factor is not None:
      store[PARAM_KEY] = factor

    def put_param(key, value):
      store[key] = value

    return HondaSteerThresholdLearner(store.get, put_param), store

  def press_edge(self, learner, base, t, above=True, active=True):
    torque = base * learner.factor * (1.5 if above else 0.5)
    learner.update(torque, base, active, t)

  def test_default_factor(self):
    learner, _ = self.make_learner()
    self.assertEqual(learner.factor, 1.0)
    self.assertEqual(learner.effective_threshold(300), 300)

  def test_nine_incidents_do_not_learn(self):
    learner, store = self.make_learner()
    base = 300.0
    t = 0.0
    for i in range(INCIDENT_COUNT - 1):
      self.press_edge(learner, base, t)
      t += 0.2
      self.press_edge(learner, base, t, above=False)
      t += 0.2
    self.assertEqual(learner.factor, 1.0)
    self.assertNotIn(PARAM_KEY, store)

  def test_tenth_incident_raises_factor(self):
    learner, store = self.make_learner()
    base = 300.0
    t = 0.0
    for _ in range(INCIDENT_COUNT):
      self.press_edge(learner, base, t)
      t += 0.15
      self.press_edge(learner, base, t, above=False)
      t += 0.15
    self.assertAlmostEqual(learner.factor, FACTOR_STEP)
    self.assertAlmostEqual(store[PARAM_KEY], FACTOR_STEP)

  def test_count_resets_after_learn(self):
    learner, _ = self.make_learner()
    base = 300.0
    t = 0.0
    for _ in range(INCIDENT_COUNT):
      self.press_edge(learner, base, t)
      t += 0.1
      self.press_edge(learner, base, t, above=False)
      t += 0.1
    first_factor = learner.factor
    for _ in range(INCIDENT_COUNT - 1):
      self.press_edge(learner, base, t)
      t += 0.1
      self.press_edge(learner, base, t, above=False)
      t += 0.1
    self.assertEqual(learner.factor, first_factor)

  def test_incidents_outside_window_drop(self):
    learner, _ = self.make_learner()
    base = 300.0
    t = 0.0
    for _ in range(INCIDENT_COUNT - 1):
      self.press_edge(learner, base, t)
      t += 0.1
      self.press_edge(learner, base, t, above=False)
      t += 0.1
    t += INCIDENT_WINDOW + 1.0
    self.press_edge(learner, base, t)
    self.assertEqual(learner.factor, 1.0)

  def test_inactive_eps_does_not_count(self):
    learner, _ = self.make_learner()
    base = 300.0
    t = 0.0
    for _ in range(INCIDENT_COUNT):
      self.press_edge(learner, base, t, active=False)
      t += 0.1
      self.press_edge(learner, base, t, above=False, active=False)
      t += 0.1
    self.assertEqual(learner.factor, 1.0)

  def test_load_sanitizes_factor(self):
    learner, _ = self.make_learner(factor=99.0)
    self.assertEqual(learner.factor, 5.0)


if __name__ == "__main__":
  unittest.main()

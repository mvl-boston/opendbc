import unittest

import numpy as np


def _simulate_prior_based_scale_learner(car_gas: float, true_scale: float, ticks: int = 200_000):
  """Old learner: scale_sample = CAR_GAS / prior_gas_average."""
  scale = 0.3
  prior = 50.0
  af = 0.1
  for _ in range(ticks):
    gas_measured = car_gas / scale
    prior = prior * (1 - af) + gas_measured * af
    if prior > 20.0 and car_gas > 5.0:
      scale_sample = car_gas / prior
      scale += 0.00001 * (scale_sample - scale)
  return scale


def _simulate_wire_based_scale_learner(car_gas: float, wire_gas: float, ticks: int = 5_000):
  """Fixed learner: scale_sample = CAR_GAS / sent PCM_GAS."""
  scale = 0.3
  for _ in range(ticks):
    if wire_gas > 20.0 and car_gas > 5.0:
      scale_sample = car_gas / wire_gas
      scale += 0.0005 * (scale_sample - scale)
  return scale


class TestNidecGasScaleLearner(unittest.TestCase):
  def test_prior_denominator_locks_scale_at_default(self):
    true_scale = 0.47
    car_gas = true_scale * 100.0
    learned = _simulate_prior_based_scale_learner(car_gas, true_scale)
    self.assertAlmostEqual(learned, 0.3, delta=0.01)

  def test_wire_denominator_converges_to_true_scale(self):
    true_scale = 0.47
    wire_gas = 100.0
    car_gas = true_scale * wire_gas
    learned = _simulate_wire_based_scale_learner(car_gas, wire_gas, ticks=20_000)
    self.assertAlmostEqual(learned, true_scale, delta=0.02)


if __name__ == "__main__":
  unittest.main()

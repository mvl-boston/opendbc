import unittest


def _brake_fraction(accel: float, speed: float, creep_factor: float = 0.0) -> float:
  """Mirror compute_gb_honda_nidec above the creep band (v > 2.3 m/s)."""
  return min(max(-float(accel) / 4.8, 0.0), 1.0)


class TestNidecBrakeOpenLoop(unittest.TestCase):
  def test_plan_accel_not_diluted_by_uphill_hill_brake_term(self):
    # Route a6354c3 seg 21: ~+0.35 m/s2 uphill in adjust_accel pulled wire to ~64% of max
    plan = -4.0
    hill = 0.35
    b_plan = _brake_fraction(plan, 12.0)
    b_diluted = _brake_fraction(plan + hill, 12.0)
    self.assertGreater(b_plan, b_diluted)
    self.assertAlmostEqual(b_plan, 4.0 / 4.8, places=3)


if __name__ == "__main__":
  unittest.main()

import unittest

from opendbc.car.honda.zero_speed import zero_speed_rundown

DT = 0.01


def rundown(v_ego_prev: float, plan_accel: float) -> float:
  v_ego = zero_speed_rundown(0.0, v_ego_prev, plan_accel, True, DT)
  assert v_ego is not None
  return v_ego


class TestZeroSpeedRundown(unittest.TestCase):
  def test_reading_is_used_when_it_is_not_zero(self):
    self.assertIsNone(zero_speed_rundown(0.36, 0.38, -0.5, True, DT))

  def test_reading_is_used_when_not_engaged(self):
    self.assertIsNone(zero_speed_rundown(0.0, 0.36, -0.5, False, DT))

  def test_reading_is_used_when_the_plan_is_positive(self):
    self.assertIsNone(zero_speed_rundown(0.0, 0.36, 0.001, True, DT))
    self.assertIsNone(zero_speed_rundown(0.0, 0.36, 0.5, True, DT))

  def test_zero_reading_runs_down_at_the_commanded_accel(self):
    self.assertAlmostEqual(rundown(0.36, -0.5), 0.355)

  def test_never_below_zero(self):
    self.assertEqual(rundown(0.001, -2.0), 0.0)
    self.assertEqual(rundown(0.0, -2.0), 0.0)

  def test_reaches_zero_in_the_time_the_command_takes(self):
    v, frames = 0.36, 0
    while v > 0.0:
      v = rundown(v, -0.2)
      frames += 1
    self.assertAlmostEqual(frames * DT, 0.36 / 0.2, delta=0.02)

  def test_holds_while_the_command_is_zero(self):
    self.assertEqual(rundown(0.3, 0.0), 0.3)


if __name__ == "__main__":
  unittest.main()

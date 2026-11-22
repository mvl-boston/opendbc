def zero_speed_rundown(v_ego_raw: float, v_ego_prev: float, plan_accel: float, long_active: bool, dt: float) -> float | None:
  """Speed to report in place of a zero reading, or None to use the reading.

  Honda wheel and transmission speeds snap to zero below about 0.3 m/s with the car still rolling
  (MDX 3G, route 3792d010590cb83a/000002b7 at 382.7 s: 0.36 m/s to zero in one step). While
  openpilot is braking, the speed is run down from the last value at the commanded accel
  (actuators.accel), as if the car did what was asked, and never below zero.
  v_ego_prev is the speed reported the frame before, plan_accel and long_active the last
  CarControl's actuators.accel and longActive.
  """
  if v_ego_raw < 1e-5 and plan_accel < 1e-3 and long_active:
    return max(v_ego_prev + plan_accel * dt, 0.0)
  return None

"""Pure Nidec longitudinal helpers (importable without openpilot)."""

# Keep in sync with opendbc/car/honda/carcontroller.py
NIDEC_MIN_ACCEL_SPEED_LEAD = 0.35


def nidec_speed_lead_mps(sf_eff: float, accel: float, alpha_eff: float) -> float:
  """PCM_SPEED lead (m/s) from the speed-channel blend."""
  speed_lead = sf_eff * accel + alpha_eff
  if accel > 0.05:
    speed_lead = sf_eff * accel + max(alpha_eff, -sf_eff * accel + NIDEC_MIN_ACCEL_SPEED_LEAD)
  return speed_lead

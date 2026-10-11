from openpilot.common.params import Params

from opendbc.car import structs


def op_long_active(CP: structs.CarParams, params: Params) -> bool:
  """Live openpilot-longitudinal state for Bosch alpha longitudinal.

  CarParams.openpilotLongitudinalControl is fixed for the drive: it selects the panda safety config
  (BOSCH_LONG TX allowlist), the LKAS bus and whether the planner runs longitudinal at all. The
  Developer toggle can therefore only hand longitudinal back to the stock system mid-drive; turning it
  on takes effect at the next drive, where CarController silences the radar once the comma relay is
  open. Falls back to CarParams when the param was never written (replay / test_models).
  """
  if not (CP.alphaLongitudinalAvailable and CP.openpilotLongitudinalControl):
    return bool(CP.openpilotLongitudinalControl)
  enabled = params.get("AlphaLongitudinalEnabled")
  return True if enabled is None else bool(params.get_bool("AlphaLongitudinalEnabled"))

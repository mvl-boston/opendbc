from openpilot.common.params import Params

from opendbc.car import structs


def op_long_active(CP: structs.CarParams, params: Params) -> bool:
  if not CP.alphaLongitudinalAvailable:
    return CP.openpilotLongitudinalControl
  # Live Developer toggle; fall back to CarParams when unset (replay / test_models).
  if params.get("AlphaLongitudinalEnabled") is not None:
    return params.get_bool("AlphaLongitudinalEnabled")
  return CP.openpilotLongitudinalControl

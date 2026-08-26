"""Honda longitudinal learner Params() keys persisted by HondaParamWriter in carcontroller."""

from opendbc.car.honda.values import CAR

# --- Params() key strings (keep in sync with carcontroller put_many) ---
HondaBrakePIDParams = "HondaBrakePIDParams"
HondaCreepFactorParams = "HondaCreepFactorParams"
HondaFeedForwardParams = "HondaFeedForwardParams"
HondaGasAlphaParams = "HondaGasAlphaParams"
HondaGasFactorParams = "HondaGasFactorParams"
HondaWindFactorParams = "HondaWindFactorParams"
HondaSpeedAlphaParams = "HondaSpeedAlphaParams"
HondaSpeedFactorParams = "HondaSpeedFactorParams"
HondaSatAccelParams = "HondaSatAccelParams"
HondaCarGasScaleParams = "HondaCarGasScaleParams"

HondaLatAccelFactor05Params = "HondaLatAccelFactor05Params"
HondaLatAccelFactor10Params = "HondaLatAccelFactor10Params"
HondaLatAccelFactor15Params = "HondaLatAccelFactor15Params"
HondaLatAccelFactor20Params = "HondaLatAccelFactor20Params"
HondaLatAccelFactor25Params = "HondaLatAccelFactor25Params"
HondaLatAccelFactor30Params = "HondaLatAccelFactor30Params"
HondaLatAccelFactor35Params = "HondaLatAccelFactor35Params"
HondaLatAccelFactor40Params = "HondaLatAccelFactor40Params"
HondaLatAccelFactor45Params = "HondaLatAccelFactor45Params"
HondaLatAccelFactor50Params = "HondaLatAccelFactor50Params"
HondaLatAccelFactor55Params = "HondaLatAccelFactor55Params"
HondaLatAccelFactor60Params = "HondaLatAccelFactor60Params"

# Nidec longitudinal learners written every ~60s from carcontroller
HONDA_NIDEC_LEARNER_PARAM_KEYS = (
  HondaFeedForwardParams,
  HondaBrakePIDParams,
  HondaCreepFactorParams,
  HondaGasAlphaParams,
  HondaGasFactorParams,
  HondaWindFactorParams,
  HondaSpeedAlphaParams,
  HondaSpeedFactorParams,
  HondaSatAccelParams,
  HondaCarGasScaleParams,
)

# Route-derived seeds for CAR_GAS byte -> PCM_GAS command scale (commaCarSegments, PCM_GAS>=80).
DEFAULT_CAR_GAS_SCALE = 0.32
GAS_PEDAL_CAR_GAS_SCALE_BY_CAR = {
  CAR.HONDA_HRV: 0.32,
  CAR.HONDA_CRV: 0.27,
  CAR.HONDA_CRV_EU: 0.27,
  CAR.HONDA_PILOT: 0.21,
  CAR.HONDA_RIDGELINE: 0.28,
}
CAR_GAS_SCALE_MIN = 0.05
CAR_GAS_SCALE_MAX = 0.5


def default_car_gas_scale(car_fingerprint: str) -> float:
  return GAS_PEDAL_CAR_GAS_SCALE_BY_CAR.get(car_fingerprint, DEFAULT_CAR_GAS_SCALE)

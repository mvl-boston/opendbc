import os
import capnp
from opendbc.car.common.basedir import BASEDIR

capnp.remove_import_hook()


def _load_car_schema():
  if os.environ.get("CAPNP_INCLUDE"):
    import cereal
    return cereal.car
  return capnp.load(os.path.join(BASEDIR, "car.capnp"), imports=[BASEDIR])


car = _load_car_schema()

CarState = car.CarState
RadarData = car.RadarData
CarControl = car.CarControl
CarParams = car.CarParams

CarStateT = capnp.lib.capnp._StructModule
RadarDataT = capnp.lib.capnp._StructModule
CarControlT = capnp.lib.capnp._StructModule
CarParamsT = capnp.lib.capnp._StructModule

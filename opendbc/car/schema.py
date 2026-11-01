import os

import capnp

from opendbc.car.common.basedir import BASEDIR

capnp.remove_import_hook()

_use_cereal = os.environ.get("OPENDBC_HOST_CAPNP") == "1"
_ws = os.environ.get("GITHUB_WORKSPACE")
_cereal_dir = os.path.join(_ws, "cereal") if _use_cereal and _ws and os.path.isdir(os.path.join(_ws, "cereal")) else None

if _cereal_dir and os.path.isfile(os.path.join(_cereal_dir, "car.capnp")):
  car = capnp.load(os.path.join(_cereal_dir, "car.capnp"), imports=[_cereal_dir])
  capnp_log = capnp.load(os.path.join(BASEDIR, "rlog.capnp"), imports=[_cereal_dir])
else:
  car = capnp.load(os.path.join(BASEDIR, "car.capnp"), imports=[BASEDIR])
  capnp_log = capnp.load(os.path.join(BASEDIR, "rlog.capnp"), imports=[BASEDIR])

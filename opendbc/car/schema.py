import os

import capnp

from opendbc.car.common.basedir import BASEDIR

capnp.remove_import_hook()

_car_capnp = os.path.join(BASEDIR, "car.capnp")
car = capnp.load(_car_capnp, imports=[BASEDIR])
capnp_log = capnp.load(os.path.join(BASEDIR, "rlog.capnp"), imports=[BASEDIR])

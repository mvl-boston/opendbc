import os

import capnp

from opendbc.car.common.basedir import BASEDIR

capnp.remove_import_hook()

car = capnp.load(os.path.join(BASEDIR, "car.capnp"), imports=[BASEDIR])
capnp_log = capnp.load(os.path.join(BASEDIR, "rlog.capnp"), imports=[BASEDIR])

#!/usr/bin/env python3
"""Patch openpilot cereal/__init__.py for host opendbc CI (single car.capnp load)."""
import os
import sys
from pathlib import Path

root = Path(os.environ["GITHUB_WORKSPACE"])
init = Path(os.environ.get("CEREAL_INIT", root / "cereal" / "__init__.py"))
init.write_text(
  """import os
import capnp
from importlib.resources import as_file, files

capnp.remove_import_hook()

with as_file(files("cereal")) as fspath:
  CEREAL_PATH = fspath.as_posix()
  from opendbc.car.schema import car
  log = capnp.load(os.path.join(CEREAL_PATH, "log.capnp"))
  custom = capnp.load(os.path.join(CEREAL_PATH, "custom.capnp"))
"""
)
print(f"Patched {init}", file=sys.stderr)

#!/usr/bin/env python3
"""Patch openpilot cereal/__init__.py to reuse opendbc.car.schema.car (host CI)."""
import os
import sys
from pathlib import Path

root = Path(os.environ["GITHUB_WORKSPACE"])
init = root / "cereal" / "__init__.py"
text = init.read_text()
old = 'car = capnp.load(os.path.join(CEREAL_PATH, "car.capnp"))'
new = "from opendbc.car.schema import car  # opendbc host CI"
if old not in text:
  print("cereal/__init__.py: expected car load line not found", file=sys.stderr)
  sys.exit(1)
init.write_text(text.replace(old, new, 1))

#!/usr/bin/env python3
"""Log reader for test_models when openpilot/cereal is co-installed."""

import os
import bz2
import capnp
import urllib.parse
import warnings
from urllib.request import urlopen

import zstandard as zstd

from opendbc.car.common.basedir import BASEDIR

# Use cereal's log schema when openpilot is on PYTHONPATH to avoid loading
# opendbc car.capnp twice (opendbc.car.structs already imports cereal.car).
try:
  from cereal import log as capnp_log
except ImportError:
  capnp.remove_import_hook()
  capnp_log = capnp.load(os.path.join(BASEDIR, "rlog.capnp"), imports=[BASEDIR])


def decompress_stream(data: bytes):
  dctx = zstd.ZstdDecompressor()

  with dctx.stream_reader(data) as reader:
    decompressed_data = reader.read()

  return decompressed_data


class LogReader:
  def __init__(self, fn, only_union_types=False, sort_by_time=False):
    self._only_union_types = only_union_types
    _, ext = os.path.splitext(urllib.parse.urlparse(fn).path)

    if fn.startswith("http"):
      with urlopen(fn) as f:
        dat = f.read()
    else:
      with open(fn, "rb") as f:
        dat = f.read()

    if ext == ".bz2" or dat.startswith(b"BZh"):
      dat = bz2.decompress(dat)
    elif ext == ".zst" or dat.startswith(b'\x28\xB5\x2F\xFD'):
      dat = decompress_stream(dat)

    ents = capnp_log.Event.read_multiple_bytes(dat)

    self._ents = []
    try:
      for e in ents:
        self._ents.append(e)
    except capnp.KjException:
      warnings.warn("Corrupted events detected", RuntimeWarning, stacklevel=1)

    if sort_by_time:
      self._ents.sort(key=lambda x: x.logMonoTime)

  def __iter__(self):
    for ent in self._ents:
      if self._only_union_types:
        try:
          ent.which()
          yield ent
        except capnp.lib.capnp.KjException:
          pass
      else:
        yield ent

  def filter(self, msg_type: str):
    return (getattr(m, m.which()) for m in filter(lambda m: m.which() == msg_type, self))

  def first(self, msg_type: str):
    return next(self.filter(msg_type), None)

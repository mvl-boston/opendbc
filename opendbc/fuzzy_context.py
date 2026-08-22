"""Synthetic fuzzy-test scope shared across opendbc imports.

@fuzzy_test sets a process-wide flag for each example so Honda CAN validation
diagnostics stay enabled for route replay and on-road use, but not for fuzzing.

The flag is stored in the environment so duplicate opendbc module loads (for
example venv install plus PYTHONPATH) still agree on whether fuzzing is active.
"""

import contextlib
import os

_FUZZY_TEST_ENV = "OPENDBC_FUZZY_TEST"


def is_fuzzy_test() -> bool:
  return int(os.environ.get(_FUZZY_TEST_ENV, "0")) > 0


@contextlib.contextmanager
def fuzzy_test_scope():
  """Mark the current process as executing synthetic @fuzzy_test data."""
  count = int(os.environ.get(_FUZZY_TEST_ENV, "0"))
  os.environ[_FUZZY_TEST_ENV] = str(count + 1)
  try:
    yield
  finally:
    if count == 0:
      os.environ.pop(_FUZZY_TEST_ENV, None)
    else:
      os.environ[_FUZZY_TEST_ENV] = str(count)

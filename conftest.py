# pytest attempts to execute shell scripts while collecting
collect_ignore_glob = [
  "opendbc/safety/tests/misra/*.sh",
  "opendbc/safety/tests/misra/cppcheck/",
  "opendbc/car/tests/test_models.py",
]

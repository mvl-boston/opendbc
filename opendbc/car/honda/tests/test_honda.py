import re
import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.honda.fingerprints import FW_VERSIONS
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR, HONDA_BOSCH, HONDA_BOSCH_TJA_CONTROL, HondaFlags

HONDA_FW_VERSION_RE = br"[A-Z0-9]{5}-[A-Z0-9]{3}(-|,)[A-Z0-9]{4}(\x00){2}$"


class TestHondaFingerprint(unittest.TestCase):
  def test_fw_version_format(self):
    # Asserts all FW versions follow an expected format
    for fw_by_ecu in FW_VERSIONS.values():
      for fws in fw_by_ecu.values():
        for fw in fws:
          assert re.match(HONDA_FW_VERSION_RE, fw) is not None, fw

  def test_tja_bosch_only(self):
    assert set(HONDA_BOSCH_TJA_CONTROL).issubset(set(HONDA_BOSCH)), "Nidec car found in TJA control list"

  def test_mdx_3g_steer_status_autodetect(self):
    for candidate in (CAR.ACURA_MDX_3G, CAR.ACURA_TLX_1G):
      fp = gen_empty_fingerprint()
      fp[0][0x18f] = 7
      fp[0][0x190] = 5
      CP = CarInterface.get_params(candidate, fp, [], False, False, False)
      assert not (CP.flags & HondaFlags.LEGACY_MDX_STEER)

      fp = gen_empty_fingerprint()
      fp[0][0x190] = 5
      CP = CarInterface.get_params(candidate, fp, [], False, False, False)
      assert CP.flags & HondaFlags.LEGACY_MDX_STEER

      fp = gen_empty_fingerprint()
      CP = CarInterface.get_params(candidate, fp, [], False, False, False)
      assert not (CP.flags & HondaFlags.LEGACY_MDX_STEER)

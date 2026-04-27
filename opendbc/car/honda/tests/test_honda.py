import re

from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.fingerprints import FW_VERSIONS
from opendbc.car.honda.values import CAR, HONDA_BOSCH, HONDA_BOSCH_TJA_CONTROL

HONDA_FW_VERSION_RE = br"[A-Z0-9]{5}-[A-Z0-9]{3}(-|,)[A-Z0-9]{4}(\x00){2}$"


class TestHondaFingerprint:
  def test_fw_version_format(self):
    # Asserts all FW versions follow an expected format
    for fw_by_ecu in FW_VERSIONS.values():
      for fws in fw_by_ecu.values():
        for fw in fws:
          assert re.match(HONDA_FW_VERSION_RE, fw) is not None, fw

  def test_tja_bosch_only(self):
    assert set(HONDA_BOSCH_TJA_CONTROL).issubset(set(HONDA_BOSCH)), "Nidec car found in TJA control list"


class TestHondaBrakePIDPersistence:
  def test_nidec_persistent_state_round_trip(self):
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    controller = CarInterface(CP).CC

    controller.set_persistent_state({
      "version": 1,
      "carFingerprint": CP.carFingerprint,
      "brakePIDFactorNonLowSpeed": 0.75,
    })

    assert controller.brake_pid.i == 0.75
    assert controller.brake_pid_factor_non_lowspeed == 0.75
    assert controller.brake_pid_factor == 0.75
    assert controller.get_persistent_state() == {
      "version": 1,
      "carFingerprint": CP.carFingerprint,
      "brakePIDFactorNonLowSpeed": 0.75,
    }

  def test_nidec_rejects_wrong_fingerprint(self):
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    controller = CarInterface(CP).CC

    controller.set_persistent_state({
      "version": 1,
      "carFingerprint": CAR.HONDA_ACCORD,
      "brakePIDFactorNonLowSpeed": 0.75,
    })

    assert controller.brake_pid.i == 0.4
    assert controller.brake_pid_factor_non_lowspeed == 0.4
    assert controller.brake_pid_factor == 0.0

  def test_bosch_cars_do_not_export_state(self):
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_BOSCH)
    controller = CarInterface(CP).CC

    controller.set_persistent_state({
      "version": 1,
      "carFingerprint": CP.carFingerprint,
      "brakePIDFactorNonLowSpeed": 0.75,
    })

    assert controller.get_persistent_state() is None

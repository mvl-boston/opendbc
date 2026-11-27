import unittest

from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.honda import hondacan
from opendbc.car.honda.values import CAR, DBC, HondaFlags


class TestScmSpamCounter(unittest.TestCase):
  def test_seed_scm_packer_counter(self):
    packer = CANPacker(DBC[CAR.HONDA_NVAN][Bus.pt])
    CP = structs.CarParams()
    CP.carFingerprint = str(CAR.HONDA_NVAN)
    CP.flags = int(HondaFlags.BOSCH)
    CAN = hondacan.CanBus(CP)
    hondacan.seed_scm_packer_counter(packer, 1)
    _, dat, _ = hondacan.spam_buttons_command(packer, CAN, 0, 0, 0, CAR.HONDA_NVAN)
    self.assertEqual((dat[-1] >> 4) & 0x3, 2)
    _, dat2, _ = hondacan.spam_buttons_command(packer, CAN, 0, 0, 0, CAR.HONDA_NVAN)
    self.assertEqual((dat2[-1] >> 4) & 0x3, 3)


class TestHondaFingerprint(unittest.TestCase):
  def test_tja_bosch_only(self):
    for car_model in CAR:
      if car_model.config.flags & HondaFlags.BOSCH_TJA_CONTROL:
        assert car_model.config.flags & HondaFlags.BOSCH, "Nidec car found with TJA control"

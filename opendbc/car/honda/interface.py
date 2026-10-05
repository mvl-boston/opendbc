#!/usr/bin/env python3
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, get_safety_config, structs, uds
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.disable_ecu import disable_ecu, clear_all_dtcs, clear_ecu_dtcs
from opendbc.car.honda import hondacan
from opendbc.car.honda.hondacan import CanBus
from opendbc.car.honda.values import CarControllerParams, HondaFlags, CAR, DBC, HONDA_BOSCH, HONDA_BOSCH_CANFD, \
                                                 HONDA_NIDEC_ALT_SCM_MESSAGES, HONDA_BOSCH_RADARLESS, HondaSafetyFlags
from opendbc.car.honda.carcontroller import CarController
from opendbc.car.honda.carstate import CarState
from opendbc.car.honda.radar_interface import RadarInterface
from opendbc.car.interfaces import CarInterfaceBase

TransmissionType = structs.CarParams.TransmissionType

# Bosch radar handback (see CarInterface._reenable_bosch_radar), all counts in 100 Hz CAN frames
BOSCH_RADAR_DIAG_ADDR = 0x18DAB0F1
BOSCH_RADAR_DIAG_RESP_ADDR = 0x18DAF1B0
BOSCH_RADAR_ACC_CONTROL_ADDR = 0x1DF
BOSCH_RADAR_SILENT_FRAMES = 3           # ACC_CONTROL is 50 Hz: 3 frames without it means the radar is silenced
BOSCH_RADAR_RESPONSE_FRAMES = 10        # max wait for the extended session positive response
BOSCH_RADAR_RESUME_FRAMES = 30          # max wait for ACC_CONTROL to resume after CommunicationControl enable
BOSCH_RADAR_REENABLE_ATTEMPTS = 5
BOSCH_RADAR_REENABLE_MAX_FRAMES = 300   # overall cap, well under the 5 s S3 fallback this replaces
BOSCH_RADAR_REENABLE_IDLE_FRAMES = 25   # consecutive empty CAN reads before assuming pandad is gone


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  DRIVABLE_GEARS = (structs.CarState.GearShifter.sport,)

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    if CP.carFingerprint in HONDA_BOSCH:
      return CarControllerParams.BOSCH_ACCEL_MIN, CarControllerParams.BOSCH_ACCEL_MAX
    else:
      # NIDECs don't allow acceleration near cruise_speed,
      # so limit limits of pid to prevent windup
      ACCEL_MAX_VALS = [CarControllerParams.NIDEC_ACCEL_MAX, 0.2]
      ACCEL_MAX_BP = [cruise_speed - 2., cruise_speed - .2]
      return CarControllerParams.NIDEC_ACCEL_MIN, np.interp(current_speed, ACCEL_MAX_BP, ACCEL_MAX_VALS)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "honda"

    CAN = CanBus(ret, fingerprint)

    if candidate in HONDA_BOSCH:
      cfgs = [get_safety_config(structs.CarParams.SafetyModel.hondaBosch)]
      if candidate in HONDA_BOSCH_CANFD and CAN.pt >= 4:
        cfgs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))
      ret.safetyConfigs = cfgs

      ret.radarUnavailable = True
      # Disable the radar and let openpilot control longitudinal
      # WARNING: THIS DISABLES AEB!
      # If Bosch radarless, this blocks ACC messages from the camera
      ret.alphaLongitudinalAvailable = True
      ret.openpilotLongitudinalControl = alpha_long
      ret.pcmCruise = not ret.openpilotLongitudinalControl
    else:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hondaNidec)]
      ret.openpilotLongitudinalControl = True

      ret.pcmCruise = True

    if candidate == CAR.HONDA_CRV_5G:
      ret.enableBsm = 0x12f8bfa7 in fingerprint[CAN.radar]

    # Detect Bosch cars with new HUD msgs
    if any(0x33DA in f for f in fingerprint.values()):
      ret.flags |= HondaFlags.BOSCH_EXT_HUD.value

    if 0x184 in fingerprint[CAN.pt]:
      ret.flags |= HondaFlags.HYBRID.value

    if (ret.flags & HondaFlags.NIDEC) and (ret.flags & HondaFlags.HYBRID) and (0x223 in fingerprint[CAN.pt]):
      ret.flags |= HondaFlags.HYBRID_ALT_BRAKEHOLD.value

    if (ret.flags & HondaFlags.NIDEC) and (ret.flags & HondaFlags.HYBRID):
      ret.stoppingDecelRate = 0.3

    if all(msg not in fingerprint[CAN.pt] for msg in (0x191, 0x1A3)):
      ret.transmissionType = TransmissionType.manual
    elif 0x191 in fingerprint[CAN.pt] and candidate != CAR.ACURA_RDX:
      # Traditional CVTs, gearshift position in GEARBOX_CVT
      ret.transmissionType = TransmissionType.cvt
    else:
      # Traditional autos, direct-drive EVs and eCVTs, gearshift position in GEARBOX_AUTO
      ret.transmissionType = TransmissionType.automatic

    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0], [0]]
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.lateralTuning.pid.kf = 0.00006  # conservative feed-forward
    ret.steerActuatorDelay = 0.1

    if candidate in HONDA_BOSCH:
      # longitudinal gas-only tuning for Bosch hondas is in carcontroller
      if candidate in HONDA_BOSCH_RADARLESS:
        ret.stopAccel = CarControllerParams.BOSCH_ACCEL_MIN  # stock uses -4.0 m/s^2 once stopped but limited by safety model
        ret.longitudinalActuatorDelay = 0.25 # s
      elif candidate in HONDA_BOSCH_CANFD:
        ret.longitudinalActuatorDelay = 0.05 # set to near zero, canfd seems to have stock feedforward correction
      else:
        ret.longitudinalActuatorDelay = 0.25 # s, per Bosch A log
    else:
      # default longitudinal tuning for all Nidec hondas
      # ret.longitudinalTuning.kiBP = [0., 5., 35.]
      # ret.longitudinalTuning.kiV = [1.2, 0.8, 0.5]
      pass  # moved to opendbc controller

    ret.stoppingDecelRate = 0.1
    ret.vEgoStopping = 0.3
    ret.vEgoStarting = ret.vEgoStopping

    # Disable control if EPS mod detected
    for fw in car_fw:
      if fw.ecu == "eps" and b"," in fw.fwVersion:
        ret.dashcamOnly = True

    if candidate == CAR.HONDA_CIVIC:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 2560], [0, 2560]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[1.1], [0.33]]

    elif candidate in (CAR.HONDA_CIVIC_BOSCH, CAR.HONDA_CIVIC_BOSCH_DIESEL, CAR.ACURA_INTEGRA):
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]

    elif candidate == CAR.HONDA_CIVIC_2022:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 5120], [0, 5120]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kpV = [[0, 10], [0.05, 0.5]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kiV = [[0, 10], [0.0125, 0.125]]

    elif candidate == CAR.HONDA_PRELUDE_6G:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kpV = [[0, 10], [0.05, 0.5]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kiV = [[0, 10], [0.0125, 0.125]]

    elif candidate == CAR.HONDA_ACCORD:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.6], [0.18]]

    elif candidate == CAR.HONDA_ACCORD_11G:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 12789], [0, 12789]]
      ret.steerActuatorDelay = 0.3
      ret.lateralTuning.pid.kf = 0.000035
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.115], [0.052]]

    elif candidate == CAR.ACURA_ILX:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 3840], [0, 3840]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]

    elif candidate in (CAR.HONDA_CRV, CAR.HONDA_CRV_EU, CAR.HONDA_CRV_SA):
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 1000], [0, 1000]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]
      ret.wheelSpeedFactor = 1.025

    elif candidate == CAR.HONDA_CRV_5G:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.64], [0.192]]
      ret.wheelSpeedFactor = 1.025

    elif candidate == CAR.HONDA_CRV_HYBRID:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.6], [0.18]]
      ret.wheelSpeedFactor = 1.025

    elif candidate in (CAR.HONDA_CRV_6G):
      ret.steerActuatorDelay = 0.15
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 5100], [0, 5100]]
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.HONDA_FIT:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.05]]

    elif candidate == CAR.HONDA_FREED:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.05]]

    elif candidate in (CAR.HONDA_HRV, CAR.HONDA_HRV_3G):
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]
      if candidate == CAR.HONDA_HRV:
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.16], [0.025]]
        ret.wheelSpeedFactor = 1.025
      else:
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]  # TODO: can probably use some tuning

    elif candidate == CAR.ACURA_RDX:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 1000], [0, 1000]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]

    elif candidate == CAR.ACURA_RDX_3G:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4095], [0, 4095]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.06]]
      CarControllerParams.BOSCH_GAS_LOOKUP_V = [0, 2200]

    elif candidate == CAR.ACURA_RDX_3G_MMR:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4076], [0, 4076]]
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      CarControllerParams.BOSCH_GAS_LOOKUP_V = [0, 2000]
      if not ret.openpilotLongitudinalControl:
        # When using stock ACC, the radar intercepts and filters steering commands the EPS would otherwise accept
        ret.minSteerSpeed = 70. * CV.KPH_TO_MS

    elif candidate == CAR.HONDA_ODYSSEY:
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.28], [0.08]]
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end

    elif candidate == CAR.HONDA_ODYSSEY_TWN:
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.28], [0.08]]
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 32767], [0, 32767]]  # TODO: determine if there is a dead zone at the top end

    elif candidate in (CAR.HONDA_PILOT, CAR.HONDA_PILOT_4G):
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      # ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.38], [0.11]] replace w Marco tune below
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kpV = [[0, 10], [0.05, 0.5]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kiV = [[0, 10], [0.0125, 0.125]]
      if candidate == CAR.HONDA_PILOT_4G:
          CarControllerParams.BOSCH_GAS_LOOKUP_V = [0, 2200]

    elif candidate == CAR.ACURA_MDX_4G_MMR:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 12789], [0, 12789]]
      ret.steerActuatorDelay = 0.3
      ret.lateralTuning.pid.kf = 0.000035
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.115], [0.052]]

    elif candidate == CAR.ACURA_TLX_2G_MMR:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # start with 4096
      ret.steerActuatorDelay = 0.15
      # try Marco tune below
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kpV = [[0, 10], [0.05, 0.5]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kiV = [[0, 10], [0.0125, 0.125]]

    elif candidate == CAR.HONDA_RIDGELINE:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.38], [0.11]]

    elif candidate in (CAR.HONDA_INSIGHT, CAR.HONDA_NBOX_2G):
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.6], [0.18]]

    elif candidate in (CAR.HONDA_E, CAR.HONDA_E_ADVANCE):
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.6], [0.18]] # TODO: can probably use some tuning

    elif candidate == CAR.HONDA_ODYSSEY_5G_MMR:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]  # TODO: determine if there is a dead zone at the top end
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.06]]
      ret.steerActuatorDelay = 0.15
      CarControllerParams.BOSCH_GAS_LOOKUP_V = [0, 2000]
      if not ret.openpilotLongitudinalControl:
        # When using stock ACC, the radar intercepts and filters steering commands the EPS would otherwise accept
        ret.minSteerSpeed = 70. * CV.KPH_TO_MS

    elif candidate == CAR.HONDA_ACCORD_9G: # source mlocoteta
      ret.steerActuatorDelay = 0.3
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 239], [0, 239]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.,20], [0.,20]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.4,0.3], [0,0]]

    elif candidate == CAR.ACURA_TLX_1G: # source mlocoteta
      ret.steerActuatorDelay = 0.3
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 179, 239], [0, 179, 830]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.,20], [0.,20]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.4,0.3], [0,0]]

    elif candidate == CAR.ACURA_MDX_3G: # source mlocoteta
      ret.steerActuatorDelay = 0.3
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 433], [0, 433]]
      ret.lateralTuning.pid.kf = 0.000035
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.115], [0.052]]

    elif candidate == CAR.ACURA_RLX_HYBRID:
      # STEERING_CONTROL is bridged to the EPS on the steer bus by a pre-flashed red panda.
      ret.autoResumeSng = True
      ret.minEnableSpeed = -1
      ret.steerActuatorDelay = 0.3
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 32767], [0, 32767]]
      ret.lateralTuning.pid.kf = 0.000035
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.115], [0.052]]

    elif candidate == CAR.HONDA_CLARITY: # source Sunnypilot
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 2560], [0, 2560]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.8], [0.24]]

    elif candidate == CAR.ACURA_ADX:
      ret.steerActuatorDelay = 0.15
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 5000], [0, 5000]]
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kpV = [[0, 10], [0.05, 0.5]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kiV = [[0, 10], [0.0125, 0.125]]

    elif candidate == CAR.HONDA_FIT_4G:
      ret.steerActuatorDelay = 0.15
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 4096], [0, 4096]]
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.ACURA_MDX_4G:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 12789], [0, 12789]]
      ret.steerActuatorDelay = 0.3
      ret.lateralTuning.pid.kf = 0.000035
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.115], [0.052]]

    elif candidate in (CAR.HONDA_PASSPORT_4G, CAR.HONDA_PILOT_4G_MMR):
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 12789], [0, 12789]]
      ret.steerActuatorDelay = 0.3
      ret.lateralTuning.pid.kf = 0.000035
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.115], [0.052]]

    else:
      ret.steerActuatorDelay = 0.15
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 3840], [0, 3840]]
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    # These cars use alternate user brake msg (0x1BE)
    if 0x1BE in fingerprint[CAN.pt] and candidate in HONDA_BOSCH:
      ret.flags |= HondaFlags.BOSCH_ALT_BRAKE.value

    if ret.flags & HondaFlags.BOSCH_ALT_BRAKE:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.ALT_BRAKE.value
    if candidate in HONDA_NIDEC_ALT_SCM_MESSAGES:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.NIDEC_ALT.value
    if ret.openpilotLongitudinalControl and candidate in HONDA_BOSCH:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.BOSCH_LONG.value
    if candidate in HONDA_BOSCH_RADARLESS:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.RADARLESS.value
    if candidate in HONDA_BOSCH_CANFD:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.BOSCH_CANFD.value
    if (ret.flags & HondaFlags.NIDEC) and (ret.flags & HondaFlags.HYBRID):
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.NIDEC_HYBRID.value
    if candidate == CAR.ACURA_RLX_HYBRID:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.RLX_STEER_BRIDGE.value

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter. Otherwise, add 0.5 mph margin to not
    # conflict with PCM acc
    if (candidate == CAR.HONDA_FIT_4G) and (not ret.openpilotLongitudinalControl):
      ret.autoResumeSng = False
    elif (ret.transmissionType == TransmissionType.manual) and (not ret.openpilotLongitudinalControl):
      ret.autoResumeSng = False
    else:
      ret.autoResumeSng = candidate in (HONDA_BOSCH | {CAR.HONDA_CIVIC, CAR.ACURA_MDX_3G, CAR.ACURA_RLX_HYBRID,
                                                       CAR.ACURA_TLX_1G, CAR.HONDA_CLARITY})
    if ret.autoResumeSng:
      ret.minEnableSpeed = -1.
    elif candidate == CAR.HONDA_ODYSSEY_TWN:
      ret.minEnableSpeed = 19. * CV.MPH_TO_MS
    elif candidate == CAR.HONDA_FIT_4G:
      ret.minEnableSpeed = 30. * CV.KPH_TO_MS
    else:
      ret.minEnableSpeed = 25.51 * CV.MPH_TO_MS

    ret.steerLimitTimer = 0.8
    ret.radarDelay = 0.1

    return ret

  @staticmethod
  def init(CP, can_recv, can_send, communication_control=None):
    if CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS):
      if communication_control is not None:
        # Re-enable the radar (deinit path)
        disable_ecu(can_recv, can_send, bus=CanBus(CP).pt, addr=0x18DAB0F1, com_cont_req=communication_control)
      elif CP.alphaLongitudinalAvailable and CP.carFingerprint in HONDA_BOSCH_CANFD:
        # CAN FD: only clear DTCs here; radar silencing is deferred to CarController until the comma
        # relay is open and AlphaLongitudinalEnabled is on. init() runs in ELM327 safety mode, so
        # silencing the radar from here raced the safety-mode switch and latched CRUISE_FAULT.
        #
        # Broadcast-clear stored DTCs every drive so radar comm-loss maturation is reset; clear the
        # radar's own stored DTCs before it is silenced. ELM327 mode allows the functional address.
        clear_all_dtcs(can_send, [CanBus(CP).pt, CanBus(CP).camera])
        clear_ecu_dtcs(can_recv, can_send, bus=CanBus(CP).pt, addr=0x18DAB0F1)
      # Bosch non-CAN FD: radar disable is also deferred in CarController (same stock-ACC handoff).

  @staticmethod
  def _reenable_bosch_radar(CP, can_recv, can_send) -> bool:
    """Hand longitudinal control back to the stock radar after it was silenced for alpha long.

    Raw ISO-TP single frames are used instead of IsoTpParallelQuery, whose flow-control frames
    hondaBosch safety blocks while onroad. Two things must hold for the handoff to be fault-free:

    * one UDS request at a time. The radar only answers one request of a back-to-back burst, so a
      blind burst of session/CommunicationControl frames left it silenced until its diagnostic
      session timed out (S3, ~5 s) and reset CommunicationControl on its own.
    * no ACC_CONTROL gap. CarController stops its own ACC_CONTROL stream the moment alpha long is
      off; if the radar is not transmitting yet the VSA latches BRAKE_ERROR and the radar
      ACC_PROBLEM/FCM_PROBLEM for the rest of the drive (stock ACC then refuses to engage). The
      disengaged ACC_CONTROL stream is bridged at 50 Hz here until the radar's own one is back.

    Returns True when the stock radar was seen transmitting ACC_CONTROL on the powertrain bus.
    """
    CAN = CanBus(CP)
    bus = CAN.pt
    packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])
    ext_session = CanData(BOSCH_RADAR_DIAG_ADDR, b'\x02\x10\x03\x00\x00\x00\x00\x00', bus)
    comm_enable = CanData(BOSCH_RADAR_DIAG_ADDR, b'\x03\x28\x80\x03\x00\x00\x00\x00', bus)

    radar_alive = False
    session_ok = False
    attempts = 0
    request_frame: int | None = None
    enable_frame: int | None = None
    idle_frames = 0

    for frame in range(BOSCH_RADAR_REENABLE_MAX_FRAMES):
      # pandad publishes CAN at 100 Hz, so each iteration is one ~10 ms frame
      can_packets = can_recv(wait_for_one=True)
      if len(can_packets) == 0:
        idle_frames += 1
        if idle_frames >= BOSCH_RADAR_REENABLE_IDLE_FRAMES:
          carlog.warning("Bosch radar re-enable: no CAN traffic, giving up")
          break
        continue
      idle_frames = 0

      for packet in can_packets:
        for msg in packet:
          # our own transmissions are echoed back with src = bus + 128
          if msg.src != bus:
            continue
          if msg.address == BOSCH_RADAR_ACC_CONTROL_ADDR:
            radar_alive = True
          elif msg.address == BOSCH_RADAR_DIAG_RESP_ADDR:
            if msg.dat[:3] == b'\x02\x50\x03':
              session_ok = True
            elif msg.dat[:2] == b'\x03\x7F':
              carlog.warning(f"Bosch radar re-enable: negative response {msg.dat.hex()}")
      if radar_alive:
        break

      # The radar transmits ACC_CONTROL every 20 ms: only once it has been silent for a few frames is
      # it known to be disabled. Until then neither bridge (would double up a live stream) nor poke it.
      if frame < BOSCH_RADAR_SILENT_FRAMES:
        continue

      if frame % 2 == 0:
        can_send([CanData(*m) for m in hondacan.create_acc_commands(packer, CAN, False, False, 0.0, 0.0, 0, CP, 0.0)])

      if request_frame is None:
        if attempts >= BOSCH_RADAR_REENABLE_ATTEMPTS:
          break
        attempts += 1
        session_ok = False
        can_send([ext_session])
        request_frame, enable_frame = frame, None
      elif enable_frame is None:
        # the radar answers the session request within ~20 ms; don't queue the next request behind it
        if session_ok or frame - request_frame >= BOSCH_RADAR_RESPONSE_FRAMES:
          can_send([comm_enable])
          enable_frame = frame
      elif frame - enable_frame >= BOSCH_RADAR_RESUME_FRAMES:
        request_frame = None

    if not radar_alive:
      carlog.error(f"Bosch radar re-enable: stock ACC_CONTROL not seen after {attempts} attempts")
    return radar_alive

  @staticmethod
  def deinit(CP, can_recv, can_send):
    if CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS):
      carlog.warning("re-enable Bosch radar (raw UDS)")
      CarInterface._reenable_bosch_radar(CP, can_recv, can_send)
      return
    communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, 0x80 | uds.CONTROL_TYPE.ENABLE_RX_ENABLE_TX,
                                   uds.MESSAGE_TYPE.NORMAL_AND_NETWORK_MANAGEMENT])
    CarInterface.init(CP, can_recv, can_send, communication_control)

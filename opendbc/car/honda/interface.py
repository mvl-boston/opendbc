#!/usr/bin/env python3
import numpy as np
from opendbc.car import get_safety_config, structs, uds
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.disable_ecu import disable_ecu, clear_all_dtcs, clear_ecu_dtcs
from opendbc.car.honda.hondacan import CanBus
from opendbc.car.honda.values import CarControllerParams, HondaFlags, CAR, HondaSafetyFlags, HONDA_BOSCH, HONDA_BOSCH_CANFD, HONDA_BOSCH_RADARLESS
from opendbc.car.honda.carcontroller import CarController
from opendbc.car.honda.carstate import CarState
from opendbc.car.honda.radar_interface import RadarInterface
from opendbc.car.interfaces import CarInterfaceBase

from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP, HondaSafetyFlagsSP

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  DRIVABLE_GEARS = (structs.CarState.GearShifter.sport, structs.CarState.GearShifter.brake,)

  @staticmethod
  def get_pid_accel_limits(CP, CP_SP, current_speed, cruise_speed):
    if CP.carFingerprint in HONDA_BOSCH:
      return CarControllerParams.BOSCH_ACCEL_MIN, CarControllerParams.BOSCH_ACCEL_MAX
    elif CP_SP.enableGasInterceptor:
      return CarControllerParams.NIDEC_ACCEL_MIN, CarControllerParams.NIDEC_ACCEL_MAX
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

    if ret.flags & HondaFlags.BOSCH:
      cfgs = [get_safety_config(structs.CarParams.SafetyModel.hondaBosch)]
      if ret.flags & HondaFlags.BOSCH_CANFD and CAN.pt >= 4:
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

    if candidate == CAR.HONDA_CRV_5G and 0x12f8bfa7 in fingerprint[CAN.radar]:
      ret.flags |= HondaFlags.HAS_BSM.value

    # Detect Bosch cars with new HUD msgs
    if any(0x33DA in f for f in fingerprint.values()):
      ret.flags |= HondaFlags.BOSCH_EXT_HUD.value

    if 0x184 in fingerprint[CAN.pt]:
      ret.flags |= HondaFlags.HYBRID.value

    if (ret.flags & HondaFlags.NIDEC) and (ret.flags & HondaFlags.HYBRID) and (0x223 in fingerprint[CAN.pt]):
      ret.flags |= HondaFlags.HYBRID_ALT_BRAKEHOLD.value

    if all(msg not in fingerprint[CAN.pt] for msg in (0x191, 0x1A3)):
      ret.transmissionType = TransmissionType.manual
    elif 0x191 in fingerprint[CAN.pt] and candidate != CAR.ACURA_RDX:
      # Traditional CVTs, gearshift position in GEARBOX_CVT
      ret.transmissionType = TransmissionType.cvt
    else:
      # Traditional autos, direct-drive EVs and eCVTs, gearshift position in GEARBOX_AUTO
      ret.transmissionType = TransmissionType.automatic

    ret.steerActuatorDelay = 0.1

    if ret.flags & HondaFlags.BOSCH:
      # longitudinal gas-only tuning for Bosch hondas is in carcontroller
      if ret.flags & HondaFlags.BOSCH_RADARLESS:
        ret.stopAccel = CarControllerParams.BOSCH_ACCEL_MIN  # stock uses -4.0 m/s^2 once stopped but limited by safety model
        ret.longitudinalActuatorDelay = 0.25 # s
      elif candidate in HONDA_BOSCH_CANFD:
        ret.longitudinalActuatorDelay = 0.05 # set to near zero, canfd seems to have stock feedforward correction
      else:
        ret.longitudinalActuatorDelay = 0.25 # s, per Bosch A log
    else:
      # Interceptor hardware (0x201 on PT bus) vs wire-gas PCM tuning; enableGasInterceptor is set
      # from the same fingerprint check in _get_params_sp for any Nidec platform.
      if 0x201 not in fingerprint[CAN.pt]:
        # ret.longitudinalTuning.kiBP = [0., 5., 35.]
        # ret.longitudinalTuning.kiV = [1.2, 0.8, 0.5]
        pass
      else:
        # The old [1.2, 0.8, 0.5] is the Nidec PCM speed-servo tune. The interceptor is a different
        # plant: the plan accel goes straight onto the pedal through a 0.2 pedal/s rise limit, and the
        # pedal -> accel response lags 0.5-1 s (longer across a DCT downshift). An integrator at 0.7-1.2
        # on that loop winds up through the whole lag: ACURA_ILX route 2752303cce1f0aba|00000000, every
        # tip-in ran the integral term to +0.5..+0.7 m/s2 on top of a 1.0-1.3 plan (25-70% of the peak
        # pedal), which is what pulled the DCT down two gears, and the overshoot that followed ran it to
        # -0.2..-0.3, which is what put the brake on against a -0.1..-0.2 plan. With ~1 s of effective
        # loop delay, ki * delay must stay well under 1; 0.3 keeps the term within +-0.2 in replay.
        ret.longitudinalTuning.kiBP = [0.]
        ret.longitudinalTuning.kiV = [0.3]

    # Disable control if EPS mod detected
    for fw in car_fw:
      if fw.ecu == "eps" and b"," in fw.fwVersion:
        ret.dashcamOnly = True

    if candidate == CAR.HONDA_CIVIC:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.HONDA_CIVIC_BOSCH, CAR.HONDA_CIVIC_BOSCH_DIESEL):
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.HONDA_CIVIC_2022, CAR.ACURA_INTEGRA):
      ret.steerActuatorDelay = 0.15
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.HONDA_PRELUDE_6G:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.HONDA_ACCORD:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.HONDA_ACCORD_11G:
      ret.steerActuatorDelay = 0.3
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.ACURA_ILX:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.HONDA_CRV, CAR.HONDA_CRV_EU, CAR.HONDA_CRV_SA):
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.wheelSpeedFactor = 1.025

    elif candidate == CAR.HONDA_CRV_5G:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.wheelSpeedFactor = 1.025

    elif candidate == CAR.HONDA_CRV_HYBRID:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.wheelSpeedFactor = 1.025

    elif candidate in (CAR.HONDA_CRV_6G):
      ret.steerActuatorDelay = 0.15
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.wheelSpeedFactor = 1.025

    elif candidate == CAR.HONDA_FIT:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.HONDA_FREED:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.HONDA_HRV, CAR.HONDA_HRV_3G):
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      if candidate == CAR.HONDA_HRV:
        ret.wheelSpeedFactor = 1.025

    elif candidate == CAR.ACURA_RDX:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.ACURA_RDX_3G:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      CarControllerParams.BOSCH_GAS_LOOKUP_V = [0, 2200]

    elif candidate == CAR.ACURA_RDX_3G_MMR:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      CarControllerParams.BOSCH_GAS_LOOKUP_V = [0, 2000]
      if not ret.openpilotLongitudinalControl:
        # When using stock ACC, the radar intercepts and filters steering commands the EPS would otherwise accept
        ret.minSteerSpeed = 70. * CV.KPH_TO_MS

    elif candidate == CAR.HONDA_ODYSSEY:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.HONDA_ODYSSEY_TWN:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.HONDA_PILOT, CAR.HONDA_PILOT_4G):
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.ACURA_MDX_4G_MMR:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.HONDA_RIDGELINE:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.HONDA_INSIGHT, CAR.HONDA_NBOX_2G):
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.HONDA_E, CAR.HONDA_E_ADVANCE):
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.HONDA_ODYSSEY_5G_MMR:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.steerActuatorDelay = 0.15
      CarControllerParams.BOSCH_GAS_LOOKUP_V = [0, 2000]
      if not ret.openpilotLongitudinalControl:
        # When using stock ACC, the radar intercepts and filters steering commands the EPS would otherwise accept
        ret.minSteerSpeed = 70. * CV.KPH_TO_MS

    elif candidate == CAR.ACURA_MDX_4G:
      ret.steerActuatorDelay = 0.3
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.ACURA_TLX_2G_MMR:
      ret.steerActuatorDelay = 0.15
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    # TODO-SP: remove when https://github.com/commaai/opendbc/pull/2687 is merged
    elif candidate in (
        CAR.HONDA_CLARITY,
        CAR.HONDA_ACCORD_9G,
        CAR.ACURA_MDX_3G,
        CAR.ACURA_TLX_1G,
        CAR.ACURA_RLX_HYBRID,
      ):
      # steerActuatorDelay and resume flags for these live in _get_params_sp
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.HONDA_FIT_4G:
      ret.steerActuatorDelay = 0.15
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.ACURA_ADX:
      ret.steerActuatorDelay = 0.15
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate in (CAR.HONDA_PASSPORT_4G, CAR.HONDA_PILOT_4G_MMR):
      ret.steerActuatorDelay = 0.3
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    else:
      ret.steerActuatorDelay = 0.15
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate == CAR.HONDA_PILOT_4G:
      CarControllerParams.BOSCH_GAS_LOOKUP_V = [0, 2200]

    # These cars use alternate user brake msg (0x1BE)
    if 0x1BE in fingerprint[CAN.pt] and ret.flags & HondaFlags.BOSCH:
      ret.flags |= HondaFlags.BOSCH_ALT_BRAKE.value

    if ret.flags & HondaFlags.BOSCH_ALT_BRAKE:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.ALT_BRAKE.value
    if ret.flags & HondaFlags.NIDEC_ALT_SCM_MESSAGES:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.NIDEC_ALT.value
    if ret.openpilotLongitudinalControl and ret.flags & HondaFlags.BOSCH:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.BOSCH_LONG.value
    if ret.flags & HondaFlags.BOSCH_RADARLESS:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.RADARLESS.value
    if candidate == CAR.ACURA_INTEGRA:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.NO_ENGINE_DATA_MSG.value
    if ret.flags & HondaFlags.BOSCH_CANFD:
      ret.safetyConfigs[-1].safetyParam |= HondaSafetyFlags.BOSCH_CANFD.value
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
      ret.autoResumeSng = bool(ret.flags & HondaFlags.BOSCH) or candidate == CAR.HONDA_CIVIC
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
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    CAN = CanBus(stock_cp, fingerprint)

    for fw in car_fw:
      if fw.ecu == "eps" and b"," in fw.fwVersion:
        ret.flags |= HondaFlagsSP.EPS_MODIFIED.value
        stock_cp.dashcamOnly = False

    if bool(stock_cp.flags & HondaFlags.NIDEC) and bool(stock_cp.flags & HondaFlags.HYBRID):
      ret.flags |= HondaFlagsSP.NIDEC_HYBRID.value
      ret.safetyParam |= HondaSafetyFlagsSP.NIDEC_HYBRID
      # some hybrids use a different brake hold
      if 0x223 in fingerprint[CAN.pt]:
        ret.flags |= HondaFlagsSP.HYBRID_ALT_BRAKEHOLD.value

    if 0x35E in fingerprint[CAN.pt]:
      ret.flags |= HondaFlagsSP.HAS_CAMERA_MESSAGES.value

    if candidate == CAR.HONDA_CIVIC:
      if ret.flags & HondaFlagsSP.EPS_MODIFIED:
        # stock request input values:     0x0000, 0x00DE, 0x014D, 0x01EF, 0x0290, 0x0377, 0x0454, 0x0610, 0x06EE
        # stock request output values:    0x0000, 0x0917, 0x0DC5, 0x1017, 0x119F, 0x140B, 0x1680, 0x1680, 0x1680
        # modified request output values: 0x0000, 0x0917, 0x0DC5, 0x1017, 0x119F, 0x140B, 0x1680, 0x2880, 0x3180
        # stock filter output values:     0x009F, 0x0108, 0x0108, 0x0108, 0x0108, 0x0108, 0x0108, 0x0108, 0x0108
        # modified filter output values:  0x009F, 0x0108, 0x0108, 0x0108, 0x0108, 0x0108, 0x0108, 0x0400, 0x0480
        # note: max request allowed is 4096, but request is capped at 3840 in firmware, so modifications result in 2x max
        CarInterfaceBase.configure_torque_tune(candidate, stock_cp.lateralTuning)

    elif candidate in (CAR.HONDA_CIVIC_BOSCH, CAR.HONDA_CIVIC_BOSCH_DIESEL):
      if ret.flags & HondaFlagsSP.EPS_MODIFIED:
        CarInterfaceBase.configure_torque_tune(candidate, stock_cp.lateralTuning)

    elif candidate == CAR.HONDA_CIVIC_2022:
      if ret.flags & HondaFlagsSP.EPS_MODIFIED:
        CarInterfaceBase.configure_torque_tune(candidate, stock_cp.lateralTuning)

    elif candidate == CAR.HONDA_ACCORD:
      if ret.flags & HondaFlagsSP.EPS_MODIFIED:
        CarInterfaceBase.configure_torque_tune(candidate, stock_cp.lateralTuning)

    elif candidate == CAR.HONDA_CRV_5G:
      if ret.flags & HondaFlagsSP.EPS_MODIFIED:
        # stock request input values:     0x0000, 0x00DB, 0x01BB, 0x0296, 0x0377, 0x0454, 0x0532, 0x0610, 0x067F
        # stock request output values:    0x0000, 0x0500, 0x0A15, 0x0E6D, 0x1100, 0x1200, 0x129A, 0x134D, 0x1400
        # modified request output values: 0x0000, 0x0500, 0x0A15, 0x0E6D, 0x1100, 0x1200, 0x1ACD, 0x239A, 0x2800
        CarInterfaceBase.configure_torque_tune(candidate, stock_cp.lateralTuning)

    elif candidate == CAR.HONDA_CLARITY:
      stock_cp.autoResumeSng = True
      stock_cp.minEnableSpeed = -1
      if ret.flags & HondaFlagsSP.EPS_MODIFIED:
        for fw in car_fw:
          if fw.ecu == "eps" and b"-" not in fw.fwVersion and b"," in fw.fwVersion:
            CarInterfaceBase.configure_torque_tune(candidate, stock_cp.lateralTuning)
          elif fw.ecu == "eps" and b"-" in fw.fwVersion and b"," in fw.fwVersion:
            CarInterfaceBase.configure_torque_tune(candidate, stock_cp.lateralTuning)
      else:
        CarInterfaceBase.configure_torque_tune(candidate, stock_cp.lateralTuning)

    elif candidate == CAR.ACURA_MDX_3G:
      stock_cp.autoResumeSng = True
      stock_cp.minEnableSpeed = -1
      # lagd runs the planner and the torque controller at steerActuatorDelay + 0.2 until it has 5 valid 60 s blocks
      # above 15 m/s, which this car's town drives never give it (routes 00000181 / 00000182: 20% calibrated the whole
      # drive, lateralDelay 0.55 in force). Its one estimated block reads 0.73 s; the desired-to-actual lateral accel
      # cross-correlation on routes 00000177 / 00000181 reads 0.64-0.67 s on the well-correlated blocks, and the steer
      # plant fits 0.15-0.25 s of dead time plus a 0.8-1.0 s first order. 0.5 puts the delay in force at 0.70 s.
      # Both exits the driver reported as overshoot (181 @ 21:15:33 ET, 182 @ 08:27:27 ET) are the loop ringing at a
      # ~4 s period against a saturated, under-compensated plant: the car 0.7 s behind a planner reversal of
      # ~3 m/s^2 in under 2 s, the request pinned the other way for 2-2.5 s, the car swinging past and back
      stock_cp.steerActuatorDelay = 0.5
      CarInterfaceBase.configure_torque_tune(candidate, stock_cp.lateralTuning)

    elif candidate == CAR.ACURA_TLX_1G:
      stock_cp.autoResumeSng = True
      stock_cp.minEnableSpeed = -1
      stock_cp.steerActuatorDelay = 0.3
      CarInterfaceBase.configure_torque_tune(candidate, stock_cp.lateralTuning)

    elif candidate == CAR.HONDA_ACCORD_9G:
      stock_cp.steerActuatorDelay = 0.3
      CarInterfaceBase.configure_torque_tune(candidate, stock_cp.lateralTuning)

    elif candidate == CAR.ACURA_RLX_HYBRID:
      # STEERING_CONTROL is bridged to the EPS on the steer bus by a pre-flashed red panda.
      stock_cp.autoResumeSng = True
      stock_cp.minEnableSpeed = -1
      stock_cp.steerActuatorDelay = 0.3
      CarInterfaceBase.configure_torque_tune(candidate, stock_cp.lateralTuning)

    if candidate in HONDA_BOSCH:
      pass
    else:
      ret.enableGasInterceptor = 0x201 in fingerprint[CAN.pt]
      stock_cp.pcmCruise = not ret.enableGasInterceptor

    if ret.enableGasInterceptor and candidate not in HONDA_BOSCH:
      ret.safetyParam |= HondaSafetyFlagsSP.GAS_INTERCEPTOR

    stock_cp.autoResumeSng = stock_cp.autoResumeSng or ret.enableGasInterceptor
    stock_cp.minEnableSpeed = -1. if ret.enableGasInterceptor else stock_cp.minEnableSpeed

    ret.intelligentCruiseButtonManagementAvailable = candidate in (HONDA_BOSCH - HONDA_BOSCH_CANFD) or \
                                                     (candidate in HONDA_BOSCH_CANFD and not is_release_sp)

    return ret

  @staticmethod
  def init(CP, CP_SP, can_recv, can_send, communication_control=None):
    if CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS) and CP.openpilotLongitudinalControl:
      if communication_control is None and CP.carFingerprint in HONDA_BOSCH_CANFD:
        # CAN FD: only clear DTCs here; the radar silencing itself is deferred to CarController until
        # the comma relay is confirmed open. init() runs while the panda is still in the ELM327 safety
        # mode, and silencing the radar from here raced the safety-mode switch: openpilot's replacement
        # ACC_CONTROL stream was blocked until the switch landed, and whenever that took longer than
        # ~110 ms the brake module (VSA) latched CRUISE_FAULT (accFaulted) for the entire drive.
        #
        # The brake module latches a radar lost-communication DTC when the radar goes silent for more
        # than ~0.1 s at cutover, and the DTC matures over trips (Honda two-trip detection): once it is
        # confirmed from a previous drive, the very next comm-loss detection trips
        # BRAKE_MODULE.CRUISE_FAULT ~0.16 s after the radar is silenced. Broadcast-clear stored DTCs on
        # all ECUs (powertrain and camera buses) every drive so the maturation counter is reset, and
        # clear the radar's own stored DTCs so codes accumulated while it was disabled don't re-fault a
        # later drive. Clearing must precede the radar silence because a DTC clear can take an ECU
        # several hundred ms to process.
        # NOTE: ELM327 safety mode allows the 29-bit functional diagnostic address on every bus, so the
        # broadcast needs no TX allowlist entry in the car safety mode.
        clear_all_dtcs(can_send, [CanBus(CP).pt, CanBus(CP).camera])
        clear_ecu_dtcs(can_recv, can_send, bus=CanBus(CP).pt, addr=0x18DAB0F1)
      else:
        # 0x80 silences response
        if communication_control is None:
          communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, 0x80 | uds.CONTROL_TYPE.DISABLE_RX_DISABLE_TX,
                                         uds.MESSAGE_TYPE.NORMAL_AND_NETWORK_MANAGEMENT])
        disable_ecu(can_recv, can_send, bus=CanBus(CP).pt, addr=0x18DAB0F1, com_cont_req=communication_control)

  @staticmethod
  def deinit(CP, can_recv, can_send):
    communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, 0x80 | uds.CONTROL_TYPE.ENABLE_RX_ENABLE_TX,
                                   uds.MESSAGE_TYPE.NORMAL_AND_NETWORK_MANAGEMENT])
    CarInterface.init(CP, can_recv, can_send, communication_control)

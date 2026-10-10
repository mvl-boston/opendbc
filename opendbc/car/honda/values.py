from dataclasses import dataclass, field
from enum import Enum, IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, structs, uds
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries, p16

Ecu = structs.CarParams.Ecu
VisualAlert = structs.CarControl.HUDControl.VisualAlert
GearShifter = structs.CarState.GearShifter


class CarControllerParams:
  # Allow small margin below -3.5 m/s^2 from ISO 15622:2018 since we
  # perform the closed loop control, and might need some
  # to apply some more braking if we're on a downhill slope.
  # Our controller should still keep the 2 second average above
  # -3.5 m/s^2 as per planner limits
  NIDEC_ACCEL_MIN = -4.0  # m/s^2
  NIDEC_ACCEL_MAX = 2.0  # m/s^2, dv_sat removes need for artificial limit

  NIDEC_ACCEL_LOOKUP_BP = [-1., 0., .6]
  NIDEC_ACCEL_LOOKUP_V = [-4.8, 0., 2.0]

  NIDEC_MAX_ACCEL_V = [0.5, 2.4, 1.4, 0.6]
  NIDEC_MAX_ACCEL_BP = [0.0, 4.0, 10., 20.]

  NIDEC_GAS_MAX = 198  # 0xc6
  NIDEC_BRAKE_MAX = 1024 // 4

  BOSCH_ACCEL_MIN = -3.5  # m/s^2
  BOSCH_ACCEL_MAX = 2.0  # m/s^2

  BOSCH_GAS_LOOKUP_BP = [0.0, 2.0]  # 2m/s^2
  BOSCH_GAS_LOOKUP_V = [0, 1600]

  STEER_STEP = 1  # 100 Hz
  STEER_DELTA_UP = 3  # min/max in 0.33s for all Honda
  STEER_DELTA_DOWN = 3
  STEER_GLOBAL_MIN_SPEED = 3 * CV.MPH_TO_MS

  def __init__(self, CP):
    self.STEER_MAX = CP.lateralParams.torqueBP[-1]
    # mirror of list (assuming first item is zero) for interp of signed request
    # values and verify that both arrays begin at zero
    assert CP.lateralParams.torqueBP[0] == 0
    assert CP.lateralParams.torqueV[0] == 0
    self.STEER_LOOKUP_BP = [v * -1 for v in CP.lateralParams.torqueBP][1:][::-1] + list(CP.lateralParams.torqueBP)
    self.STEER_LOOKUP_V = [v * -1 for v in CP.lateralParams.torqueV][1:][::-1] + list(CP.lateralParams.torqueV)


class HondaSafetyFlags(IntFlag):
  ALT_BRAKE = 1
  BOSCH_LONG = 2
  NIDEC_ALT = 4
  RADARLESS = 8
  BOSCH_CANFD = 16
  NIDEC_HYBRID = 32
  # RLX: a bridge panda relays the stock camera's LKAS_HUD from the steer bus onto the powertrain bus
  RLX_STEER_BRIDGE = 64
  # EU CR-V 6G: the ECU authoring STEERING_CONTROL is on the car side of the harness and is silenced over UDS
  VISION_CTRL = 128


class HondaFlags(IntFlag):
  # Detected flags
  # Bosch models with alternate set of LKAS_HUD messages
  BOSCH_EXT_HUD = 1
  BOSCH_ALT_BRAKE = 2

  # Static flags
  BOSCH = 4
  BOSCH_RADARLESS = 8

  NIDEC = 16
  NIDEC_ALT_PCM_ACCEL = 32
  NIDEC_ALT_SCM_MESSAGES = 64

  BOSCH_CANFD = 128

  HAS_ALL_DOOR_STATES = 256  # Some Hondas have all door states, others only driver door
  BOSCH_ALT_RADAR = 512
  # The ECU that authors STEERING_CONTROL (and the ACC messages) is not isolated by the comma harness, so
  # opening the relay does not take it off the bus. It is silenced over UDS instead (see vision_ctrl.py)
  VISION_CTRL = 1024
  HYBRID = 2048
  BOSCH_TJA_CONTROL = 4096
  LKAS_MINSPEED_CUTOFF = 8192
  HYBRID_ALT_BRAKEHOLD = 16384  # Some Nidec Hybrids use a different brakehold


# Car button codes
class CruiseButtons:
  RES_ACCEL = 4
  DECEL_SET = 3
  CANCEL = 2
  MAIN = 1


class CruiseSettings:
  DISTANCE = 3
  LKAS = 1


@dataclass
class HondaCarDocs(CarDocs):
  package: str = "Honda Sensing"

  def init_make(self, CP: structs.CarParams):
    if CP.flags & HondaFlags.BOSCH:
      if CP.flags & HondaFlags.BOSCH_CANFD:
        harness = CarHarness.bosch_c
      elif CP.flags & HondaFlags.BOSCH_RADARLESS:
        harness = CarHarness.bosch_b
      else:
        harness = CarHarness.bosch_a
    else:
      harness = CarHarness.nidec

    self.car_parts = CarParts.common([harness])

    if CP.alphaLongitudinalAvailable:
      self.footnotes.append(Footnote.HONDA_ALPHALONG)

class Footnote(Enum):
  CIVIC_DIESEL = CarFootnote(
    "2019 Honda Civic 1.6L Diesel Sedan does not have ALC below 12mph.",
    Column.FSR_STEERING)
  HONDA_ALPHALONG = CarFootnote(
    "Enabling longitudinal control (alpha) will disable all CMBS functionality, including AEB and FCW.",
    Column.LONGITUDINAL)


@dataclass
class HondaBoschPlatformConfig(PlatformConfig):
  def init(self):
    self.flags |= HondaFlags.BOSCH


@dataclass
class HondaBoschCANFDPlatformConfig(HondaBoschPlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'honda_common_canfd_generated', Bus.radar: 'honda_common_canfd_generated'})

  def init(self):
    super().init()
    self.flags |= HondaFlags.BOSCH_CANFD


# CAN FD body and radar look-alikes (LANE_PATH, HUD_OBJECTS, RADAR_LEAD, RADAR_REFERENCE) but the radarless-style
# ACC_CONTROL (0x1C8) and CRUISE_FAULT_STATUS instead of the radar's 0x1DF/0x1EF: what the vision controller
# authors on the EU CR-V and the MDX Type S (relay-open census of route ad9840558640c31d/00000009--b2e159e05d)
VISION_CTRL_DBC: DbcDict = {Bus.pt: 'honda_vision_ctrl_generated', Bus.radar: 'honda_vision_ctrl_generated'}


@dataclass
class HondaNidecPlatformConfig(PlatformConfig):
  def init(self):
    self.flags |= HondaFlags.NIDEC


def radar_dbc_dict(pt_dict):
  return {Bus.pt: pt_dict, Bus.radar: 'acura_ilx_2016_nidec'}


# Certain Hondas have an extra steering sensor at the bottom of the steering rack,
# which improves controls quality as it removes the steering column torsion from feedback.
# Tire stiffness factor fictitiously lower if it includes the steering column torsion effect.
# For modeling details, see p.198-200 in "The Science of Vehicle Dynamics (2014), M. Guiggiani"


class CAR(Platforms):
  # Bosch Cars
  HONDA_NBOX_2G = HondaBoschPlatformConfig(
    [
      HondaCarDocs("Honda N-Box 2018", "All", min_steer_speed=5.),
    ],
    CarSpecs(mass=890., wheelbase=2.520, steerRatio=18.64),
    {Bus.pt: 'acura_rdx_2020_can_generated'},
  )
  HONDA_ACCORD = HondaBoschPlatformConfig(
    [
      HondaCarDocs("Honda Accord 2018-22", "All", video="https://www.youtube.com/watch?v=mrUwlj3Mi58", min_steer_speed=3. * CV.MPH_TO_MS),
      HondaCarDocs("Honda Inspire 2018", "All", min_steer_speed=3. * CV.MPH_TO_MS),
      HondaCarDocs("Honda Accord Hybrid 2018-22", "All", min_steer_speed=3. * CV.MPH_TO_MS),
    ],
    # steerRatio: 11.82 is spec end-to-end
    CarSpecs(mass=3279 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=16.33, centerToFrontRatio=0.39, tireStiffnessFactor=0.8467),
    {Bus.pt: 'honda_civic_hatchback_ex_2017_can_generated'},
  )
  HONDA_ACCORD_11G = HondaBoschCANFDPlatformConfig(
    [
      HondaCarDocs("Honda Accord 2023-25", "All"),
      HondaCarDocs("Honda Accord Hybrid 2023-26", "All"),
  ],
    CarSpecs(mass=3477 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=16.7, centerToFrontRatio=0.39),
  )
  HONDA_CIVIC_BOSCH = HondaBoschPlatformConfig(
    [
      HondaCarDocs("Honda Civic 2019-21", "All", video="https://www.youtube.com/watch?v=4Iz1Mz5LGF8",
                   footnotes=[Footnote.CIVIC_DIESEL], min_steer_speed=2. * CV.MPH_TO_MS),
      HondaCarDocs("Honda Civic Hatchback 2017-18", min_steer_speed=12. * CV.MPH_TO_MS),
      HondaCarDocs("Honda Civic Hatchback 2019-21", "All", min_steer_speed=12. * CV.MPH_TO_MS),
    ],
    CarSpecs(mass=1326, wheelbase=2.7, steerRatio=15.38, centerToFrontRatio=0.4),  # steerRatio: 10.93 is end-to-end spec
    {Bus.pt: 'honda_civic_hatchback_ex_2017_can_generated'},
  )
  HONDA_CIVIC_BOSCH_DIESEL = HondaBoschPlatformConfig(
    [],  # don't show in docs
    HONDA_CIVIC_BOSCH.specs,
    {Bus.pt: 'honda_civic_hatchback_ex_2017_can_generated'},
  )
  HONDA_CIVIC_2022 = HondaBoschPlatformConfig(
    [
      HondaCarDocs("Honda Civic 2022-24", "All", video="https://youtu.be/ytiOT5lcp6Q"),
      HondaCarDocs("Honda Civic Hybrid 2025-26", "All"),
      HondaCarDocs("Honda Civic Hatchback 2022-24", "All", video="https://youtu.be/ytiOT5lcp6Q"),
      # TODO: Confirm 2025
      HondaCarDocs("Honda Civic Hatchback 2026", "All"),
      HondaCarDocs("Honda Civic Hatchback Hybrid (Europe only) 2023", "All"),
      # TODO: Confirm 2024
      HondaCarDocs("Honda Civic Hatchback Hybrid 2025-26", "All"),
    ],
    HONDA_CIVIC_BOSCH.specs,
    {Bus.pt: 'honda_bosch_radarless_generated'},
    flags=HondaFlags.BOSCH_RADARLESS
  )
  HONDA_CRV_5G = HondaBoschPlatformConfig(
    [HondaCarDocs("Honda CR-V 2017-22", min_steer_speed=15. * CV.MPH_TO_MS)],
    # steerRatio: 12.3 is spec end-to-end
    CarSpecs(mass=3410 * CV.LB_TO_KG, wheelbase=2.66, steerRatio=16.0, centerToFrontRatio=0.41, tireStiffnessFactor=0.677),
    {Bus.pt: 'honda_civic_hatchback_ex_2017_can_generated', Bus.body: 'honda_crv_ex_2017_body_generated'},
    flags=HondaFlags.LKAS_MINSPEED_CUTOFF
  )
  HONDA_CRV_6G = HondaBoschCANFDPlatformConfig(
    [
      HondaCarDocs("Honda CR-V 2023-26", "All"),
      HondaCarDocs("Honda CR-V Hybrid 2023-26", "All"),
    ],
    CarSpecs(mass=1703, wheelbase=2.7, steerRatio=16.2, centerToFrontRatio=0.42),
  )
  HONDA_CRV_6G_EU = HondaBoschCANFDPlatformConfig(
    # European CR-V e:HEV (3E7/3E8/3E9 part codes). CAN FD body like the US CR-V 6G, but there is no separate
    # radar (no 0x18DAB0F1 ECU, no ACC_CONTROL 0x1DF, empty radar bus): a single radar/vision controller
    # authors STEERING_CONTROL and the radarless-style ACC messages (0x1C8/0x1EF), and the comma harness at the
    # camera does not isolate it, so openpilot has to silence it over UDS before it can steer (vision_ctrl.py).
    # Don't show in docs until lateral control is proven on-car.
    [],
    HONDA_CRV_6G.specs,
    VISION_CTRL_DBC,
    flags=HondaFlags.VISION_CTRL,
  )
  HONDA_CRV_HYBRID = HondaBoschPlatformConfig(
    [HondaCarDocs("Honda CR-V Hybrid 2017-22", min_steer_speed=12. * CV.MPH_TO_MS)],
    # mass: mean of 4 models in kg, steerRatio: 12.3 is spec end-to-end
    CarSpecs(mass=1667, wheelbase=2.66, steerRatio=16, centerToFrontRatio=0.41, tireStiffnessFactor=0.677),
    {Bus.pt: 'honda_civic_hatchback_ex_2017_can_generated'},
  )
  HONDA_HRV_3G = HondaBoschPlatformConfig(
    [HondaCarDocs("Honda HR-V 2023-27", "All")],
    CarSpecs(mass=3125 * CV.LB_TO_KG, wheelbase=2.61, steerRatio=15.2, centerToFrontRatio=0.41, tireStiffnessFactor=0.5),
    {Bus.pt: 'honda_bosch_radarless_generated'},
    flags=HondaFlags.BOSCH_RADARLESS,
  )
  HONDA_CITY_7G = HondaBoschPlatformConfig(
    [HondaCarDocs("Honda City (Brazil only) 2023", "All")],
    CarSpecs(mass=3125 * CV.LB_TO_KG, wheelbase=2.6, steerRatio=19.0, centerToFrontRatio=0.41, minSteerSpeed=23. * CV.KPH_TO_MS),
    {Bus.pt: 'honda_bosch_radarless_generated'},
    flags=HondaFlags.BOSCH_RADARLESS | HondaFlags.LKAS_MINSPEED_CUTOFF
  )
  ACURA_RDX_3G = HondaBoschPlatformConfig(
    [HondaCarDocs("Acura RDX 2019-21", "All", min_steer_speed=3. * CV.MPH_TO_MS)],
    CarSpecs(mass=4068 * CV.LB_TO_KG, wheelbase=2.75, steerRatio=11.95, centerToFrontRatio=0.41, tireStiffnessFactor=0.677),  # as spec
    {Bus.pt: 'acura_rdx_2020_can_generated'},
  )
  ACURA_RDX_3G_MMR = HondaBoschPlatformConfig(
    [HondaCarDocs("Acura RDX 2022-24", "All", min_steer_speed=70. * CV.KPH_TO_MS)],
    CarSpecs(mass=4079 * CV.LB_TO_KG, wheelbase=2.75, centerToFrontRatio=0.41, steerRatio=16.2),
    {Bus.pt: 'acura_rdx_2020_can_generated'},
    flags=HondaFlags.BOSCH_ALT_RADAR,
  )
  HONDA_INSIGHT = HondaBoschPlatformConfig(
    [HondaCarDocs("Honda Insight 2019-22", "All", min_steer_speed=3. * CV.MPH_TO_MS)],
    CarSpecs(mass=2987 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=15.0, centerToFrontRatio=0.39, tireStiffnessFactor=0.82),  # as spec
    {Bus.pt: 'honda_insight_ex_2019_can_generated'},
  )
  HONDA_E = HondaBoschPlatformConfig(
    [HondaCarDocs("Honda e 2020", "All", min_steer_speed=3. * CV.MPH_TO_MS)],
    CarSpecs(mass=3338.8 * CV.LB_TO_KG, wheelbase=2.5, centerToFrontRatio=0.5, steerRatio=16.71, tireStiffnessFactor=0.82),
    {Bus.pt: 'acura_rdx_2020_can_generated'},
  )
  HONDA_E_ADVANCE = HondaBoschPlatformConfig(
    [],  # don't show in docs, base trim already in docs
    CarSpecs(mass=1527, wheelbase=2.5, centerToFrontRatio=0.5, steerRatio=16.71, tireStiffnessFactor=0.82),
    {Bus.pt: 'honda_e_advance_2020_can_generated'}, # 8 bit LKAS_HUD in Advance trim
  )
  HONDA_PILOT_4G = HondaBoschCANFDPlatformConfig(
    [HondaCarDocs("Honda Pilot 2023-25", "All")],
    CarSpecs(mass=4660 * CV.LB_TO_KG, wheelbase=2.89, centerToFrontRatio=0.442, steerRatio=17.5),
  )
  HONDA_PILOT_4G_MMR = HondaBoschCANFDPlatformConfig( # Mid-Model Refresh has more powerful EPS
    [HondaCarDocs("Honda Pilot 2026", "All")],
    CarSpecs(mass=4528 * CV.LB_TO_KG, wheelbase=2.89, centerToFrontRatio=0.442, steerRatio=17.5),
  )
  HONDA_PASSPORT_4G = HondaBoschCANFDPlatformConfig(
    [HondaCarDocs("Honda Passport 2026", "All")],
    CarSpecs(mass=4620 * CV.LB_TO_KG, wheelbase=2.89, centerToFrontRatio=0.442, steerRatio=18.5),
  )
  HONDA_PRELUDE_6G = HondaBoschPlatformConfig(
    [HondaCarDocs("Honda Prelude 2026", "All")],
    # Shares the Civic e:HEV platform and the radarless Bosch messaging. 63/37 front/rear weight distribution.
    # steerRatio from liveParameters, converged at 13.65 on a EU-market car (variable ratio rack, 14.9 on-center)
    CarSpecs(mass=1480, wheelbase=2.605, steerRatio=13.65, centerToFrontRatio=0.37),
    {Bus.pt: 'honda_bosch_radarless_generated'},
    flags=HondaFlags.BOSCH_RADARLESS,
  )
  ACURA_MDX_4G = HondaBoschPlatformConfig(
    [HondaCarDocs("Acura MDX 2022-24", "All")],
    CarSpecs(mass=4788 * CV.LB_TO_KG, wheelbase=2.89, steerRatio=15.8, centerToFrontRatio=0.428),  # as spec
    {Bus.pt: 'honda_common_canfd_generated'}, # not CANFD car but shares same dbc
    flags=HondaFlags.BOSCH_ALT_RADAR | HondaFlags.BOSCH_TJA_CONTROL,
  )
  # mid-model refresh
  ACURA_MDX_4G_MMR = HondaBoschCANFDPlatformConfig(
    [HondaCarDocs("Acura MDX 2025-26", "All except Type S")],
    CarSpecs(mass=4776 * CV.LB_TO_KG, wheelbase=2.89, centerToFrontRatio=0.428, steerRatio=16.7),
  )
  ACURA_MDX_4G_TYPE_S = HondaBoschCANFDPlatformConfig(
    # US MMR Type S (TYB part codes). CAN FD body like the 4G_MMR, but no Bosch radar on the powertrain bus
    # (no 0x18DAB0F1 ECU, no ACC_CONTROL 0x1DF, no radar tick references). Per the service wiring diagram the
    # driver-assist system is a central Radar Vision Unit (RVU) on AF-CAN A: the camera and all five radars
    # (front center, two front corner, two rear corner) each hang off it on a private CAN pair. The RVU authors
    # STEERING_CONTROL and the radarless-style ACC messages (0x1C8/0x1EF) on AF-CAN A, i.e. on the car side of
    # the camera harness (same architecture as the EU CR-V), so openpilot has to silence it over UDS before it
    # can steer (vision_ctrl.py). Confirmed on route ad9840558640c31d/00000009--b2e159e05d: the RVU answers at
    # 0x18DAB8F1, and CommunicationControl there stops its STEERING_CONTROL, ACC_CONTROL (0x1C8), ACC_HUD,
    # LKAS_HUD, LANE_PATH, HUD_OBJECTS and RADAR_LEAD within one frame, while RADAR_REFERENCE (0x3A1) and the
    # camera's own 0x1EF/0x35E/RADAR_LEAD2 keep going. The only car-side messages the relay isolates are those
    # three camera ones. The harness's second pair is the camera<->RVU private link, not a radar: it carries a
    # CAN FD stream (0xE6/0x334 at 100 Hz, 64-byte 0x5xx frames on a 60 ms cycle) that answered none of the
    # standard Honda diagnostic addresses (route ad9840558640c31d/00000001--d1808da632).
    # The RVU also authors a 50 Hz ACC_CONTROL companion (0x1C9) and constant status broadcasts at 25/10/1 Hz
    # (0x29B, 0x2E8, 0x1A45AA24) that all stop with it; with only STEERING_CONTROL/ACC_CONTROL/HUD look-alikes
    # replacing it (route ad9840558640c31d/0000000a--cffee2dde2, no panda blocks) the brake module still latched
    # CRUISE_FAULT 0.34 s after the switchover and the cluster raised transmission, lane change CMBS and front
    # cross traffic faults, so hondacan replaces those four as well and matches the stock idle ACC_CONTROL bytes.
    # With those (route ad9840558640c31d/0000000b--5cfa56b3e8) CRUISE_FAULT no longer latched, but the same three
    # cluster faults did, the transmission one 1.02 s after the RVU's last 50 Hz frames in every drive: the
    # switchover gap was 3 frames on the 50 Hz pair and the camera (behind the relay) had been seeing the RVU's
    # STEERING_CONTROL/ACC_CONTROL/0x1C9/ACC_HUD/LKAS_HUD through panda forwarding and lost all of them, and
    # 0x334 @ 100 Hz on the harness radar bus (bus 1) stopped with the RVU while 0xE6 kept going. Now the
    # switchover is detected after 2 missed frames, every look-alike goes out in the first silent frame with the
    # stock idle contents, the five camera-facing messages are mirrored onto the camera bus byte-identically, and
    # 0x334 is authored on bus 1.
    # Route ad9840558640c31d/0000000d--b1a7407153 (tick-exact cutover, stock checksums, stock intervals on every
    # replaced stream) still raised the same faults on the same clock: PCM GEARBOX_AUTO (0x1A3) gear nibble
    # blinking from +1.03 s, the 25 Hz 0x22C status zeroed at +1.57 s, RADAR_REFERENCE (0x3A1) and 0xF31AA57
    # dropping a bit at +2.14 s, nothing on buses 0/1/2 changing before them, and the camera's messages unchanged
    # for 15 s. On the dashcam drive 0000000f--e75f85e2d5 (stock RVU) the remembered faults cleared within seconds
    # of driving. The wiring diagram puts the RVU on AF-CAN B as well (the rear corner radars' network, not
    # reachable from the harness) besides AF-CAN A and the six private pairs, so the every-network
    # CommunicationControl (28 83 03) also took it off the networks that cannot be replaced: vision_ctrl now tries
    # the this-network form (28 01 F3, disableTx on AF-CAN A only, private links and AF-CAN B stay up) first and
    # falls back to the every-network one if STEERING_CONTROL survives it. The remaining content differences to
    # stock (0x1C9 gap table per distance bar, LANE_PATH/HUD_OBJECTS idle sentinel 2044, LKAS_STATE_CHANGE pulse,
    # counters restarting at 0) are gone too.
    # Don't show in docs until lateral control is proven on-car.
    [],
    CarSpecs(mass=4544 * CV.LB_TO_KG, wheelbase=2.89, centerToFrontRatio=0.428, steerRatio=16.7),
    VISION_CTRL_DBC,
    flags=HondaFlags.VISION_CTRL,
  )
  HONDA_ODYSSEY_5G_MMR = HondaBoschPlatformConfig(
    [HondaCarDocs("Honda Odyssey 2021-26", "All", min_steer_speed=70. * CV.KPH_TO_MS)],
    CarSpecs(mass=4590 * CV.LB_TO_KG, wheelbase=3.00, steerRatio=19.4, centerToFrontRatio=0.41),
    {Bus.pt: 'acura_rdx_2020_can_generated'},
    flags=HondaFlags.BOSCH_ALT_RADAR,
  )
  ACURA_TLX_2G = HondaBoschPlatformConfig(
    [HondaCarDocs("Acura TLX 2021-22", "All")],
    CarSpecs(mass=3982 * CV.LB_TO_KG, wheelbase=2.87, steerRatio=14.0, centerToFrontRatio=0.43),
    {Bus.pt: 'honda_civic_hatchback_ex_2017_can_generated'},
    flags=HondaFlags.BOSCH_ALT_RADAR,
  )
  # mid-model refresh
  ACURA_TLX_2G_MMR = HondaBoschCANFDPlatformConfig(
    [HondaCarDocs("Acura TLX 2025", "All")],
    CarSpecs(mass=3990 * CV.LB_TO_KG, wheelbase=2.87, centerToFrontRatio=0.43, steerRatio=13.7),
  )
  HONDA_FIT_4G = HondaBoschPlatformConfig(
    [
      HondaCarDocs("Honda Fit (Taiwan) 2021", "All"),
      # TODO: add 2022-2023 fingerprints
      HondaCarDocs("Honda Fit (Taiwan) 2024-25", "All"),
    ],
    CarSpecs(mass=1229, wheelbase=2.53, steerRatio=19.7, centerToFrontRatio=0.39, minSteerSpeed=23. * CV.KPH_TO_MS),
    {Bus.pt: 'honda_bosch_radarless_generated'},
    flags=HondaFlags.BOSCH_RADARLESS | HondaFlags.LKAS_MINSPEED_CUTOFF
  )
  ACURA_INTEGRA = HondaBoschPlatformConfig(
    [HondaCarDocs("Acura Integra 2023-25", "All")],
    CarSpecs(mass=3338.8 * CV.LB_TO_KG, wheelbase=2.5, centerToFrontRatio=0.5, steerRatio=15.5,),
    {Bus.pt: 'honda_bosch_radarless_generated'},
    flags=HondaFlags.BOSCH_RADARLESS,
  )
  ACURA_ADX = HondaBoschPlatformConfig(
    [HondaCarDocs("Acura ADX 2025", "All")],
    CarSpecs(mass=3578 * CV.LB_TO_KG, wheelbase=2.65, steerRatio=16.6, centerToFrontRatio=0.43),
    {Bus.pt: 'honda_bosch_radarless_generated'},
    flags=HondaFlags.BOSCH_RADARLESS
  )

  # Nidec Cars
  ACURA_ILX = HondaNidecPlatformConfig(
    [
      HondaCarDocs("Acura ILX 2016-18", "Technology Plus Package or AcuraWatch Plus", min_steer_speed=25. * CV.MPH_TO_MS),
      HondaCarDocs("Acura ILX 2019", "All", min_steer_speed=25. * CV.MPH_TO_MS),
    ],
    CarSpecs(mass=3095 * CV.LB_TO_KG, wheelbase=2.67, steerRatio=18.61, centerToFrontRatio=0.37, tireStiffnessFactor=0.72),  # 15.3 is spec end-to-end
    radar_dbc_dict('acura_ilx_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_ACCORD_9G = HondaNidecPlatformConfig(
    [], # don't add to cardocs since custom steering board
    CarSpecs(mass=3343 * CV.LB_TO_KG, wheelbase=2.78, steerRatio=17.5, centerToFrontRatio=0.37),  # as spec
    radar_dbc_dict('honda_accord_2017_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_CRV = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda CR-V 2015-16", "Touring Trim", min_steer_speed=12. * CV.MPH_TO_MS)],
    CarSpecs(mass=3572 * CV.LB_TO_KG, wheelbase=2.62, steerRatio=16.89, centerToFrontRatio=0.41, tireStiffnessFactor=0.444),  # as spec
    radar_dbc_dict('honda_crv_touring_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_CRV_EU = HondaNidecPlatformConfig(
    [],  # Euro version of CRV Touring, don't show in docs
    HONDA_CRV.specs,
    radar_dbc_dict('honda_crv_touring_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_CRV_SA = HondaNidecPlatformConfig(
    [],  # South Africa version of CRV Touring, don't show in docs
    HONDA_CRV.specs,
    radar_dbc_dict('acura_rdx_2018_can_generated'), # different gearbox message from USA CRV
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_FIT = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda Fit 2018-20", min_steer_speed=12. * CV.MPH_TO_MS)],
    CarSpecs(mass=2644 * CV.LB_TO_KG, wheelbase=2.53, steerRatio=13.06, centerToFrontRatio=0.39, tireStiffnessFactor=0.75),
    radar_dbc_dict('acura_ilx_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES,
  )
  HONDA_FREED = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda Freed 2020", min_steer_speed=12. * CV.MPH_TO_MS)],
    CarSpecs(mass=3086. * CV.LB_TO_KG, wheelbase=2.74, steerRatio=13.06, centerToFrontRatio=0.39, tireStiffnessFactor=0.75),  # mostly copied from FIT
    radar_dbc_dict('acura_ilx_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES,
  )
  HONDA_HRV = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda HR-V 2019-22", min_steer_speed=12. * CV.MPH_TO_MS)],
    HONDA_HRV_3G.specs,
    radar_dbc_dict('acura_ilx_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES,
  )
  HONDA_ODYSSEY = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda Odyssey 2018-20")],
    CarSpecs(mass=1900, wheelbase=3.0, steerRatio=14.35, centerToFrontRatio=0.41, tireStiffnessFactor=0.82),
    radar_dbc_dict('honda_odyssey_exl_2018_generated'),
    flags=HondaFlags.NIDEC_ALT_PCM_ACCEL | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_ODYSSEY_TWN = HondaNidecPlatformConfig(
    [
      HondaCarDocs("Honda Odyssey (Taiwan) 2018-19"),
      HondaCarDocs("Honda Odyssey (Singapore) 2021")
    ],
    CarSpecs(mass=1865, wheelbase=2.9, steerRatio=14.35, centerToFrontRatio=0.44, tireStiffnessFactor=0.82),
    radar_dbc_dict('honda_odyssey_twn_2018_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES,
  )
  ACURA_RDX = HondaNidecPlatformConfig(
    [HondaCarDocs("Acura RDX 2016-18", "AcuraWatch Plus or Advance Package", min_steer_speed=12. * CV.MPH_TO_MS)],
    CarSpecs(mass=3925 * CV.LB_TO_KG, wheelbase=2.68, steerRatio=15.0, centerToFrontRatio=0.38, tireStiffnessFactor=0.444),  # as spec
    radar_dbc_dict('acura_rdx_2018_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  ACURA_MDX_3G = HondaNidecPlatformConfig(
    [], # don't add to cardocs since custom steering board # TODO: find remaining fingerprints
    CarSpecs(mass=4215 * CV.LB_TO_KG, wheelbase=2.82, steerRatio=16.8, centerToFrontRatio=0.428),  # as spec, learned steerRatio
    radar_dbc_dict('acura_mdx_3g_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES,
  )
  ACURA_RLX_HYBRID = HondaNidecPlatformConfig(
    # 2017 RLX Sport Hybrid. Don't add to cardocs: the EPS is on a separate steer bus; a
    # pre-flashed red panda bridges steer messages onto the powertrain bus this code sees
    [],
    CarSpecs(mass=4359 * CV.LB_TO_KG, wheelbase=2.85, centerToFrontRatio=0.43, steerRatio=18.3),
    radar_dbc_dict('acura_rlx_2017_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_PILOT = HondaNidecPlatformConfig(
    [
      HondaCarDocs("Honda Pilot 2016-22", min_steer_speed=12. * CV.MPH_TO_MS),
      HondaCarDocs("Honda Passport 2019-25", "All", min_steer_speed=12. * CV.MPH_TO_MS),
    ],
    CarSpecs(mass=4278 * CV.LB_TO_KG, wheelbase=2.86, centerToFrontRatio=0.428, steerRatio=16.0, tireStiffnessFactor=0.444),  # as spec
    radar_dbc_dict('acura_ilx_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_RIDGELINE = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda Ridgeline 2017-26", min_steer_speed=12. * CV.MPH_TO_MS)],
    CarSpecs(mass=4515 * CV.LB_TO_KG, wheelbase=3.18, centerToFrontRatio=0.41, steerRatio=15.59, tireStiffnessFactor=0.444),  # as spec
    radar_dbc_dict('acura_ilx_2016_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_CIVIC = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda Civic 2016-18", min_steer_speed=12. * CV.MPH_TO_MS, video="https://youtu.be/-IkImTe1NYE")],
    CarSpecs(mass=1326, wheelbase=2.70, centerToFrontRatio=0.4, steerRatio=15.38),  # 10.93 is end-to-end spec
    radar_dbc_dict('honda_civic_touring_2016_can_generated'),
    flags=HondaFlags.HAS_ALL_DOOR_STATES
  )
  ACURA_TLX_1G = HondaNidecPlatformConfig(
    [], # don't add to cardocs since custom steering board
    CarSpecs(mass=3680 * CV.LB_TO_KG, wheelbase=2.78, steerRatio=17.0, centerToFrontRatio=0.40, tireStiffnessFactor=0.18),
    radar_dbc_dict('acura_mdx_3g_can_generated'),
    flags=HondaFlags.NIDEC_ALT_SCM_MESSAGES | HondaFlags.HAS_ALL_DOOR_STATES,
  )
  HONDA_CLARITY = HondaNidecPlatformConfig(
    [HondaCarDocs("Honda Clarity 2018-21", min_steer_speed=12. * CV.MPH_TO_MS)],
    CarSpecs(mass=1838, wheelbase=2.75, centerToFrontRatio=0.4, steerRatio=16.5),
    radar_dbc_dict('honda_clarity_hybrid_2018_can_generated'),
    flags=HondaFlags.HAS_ALL_DOOR_STATES,
  )


HONDA_NIDEC_ALT_PCM_ACCEL = CAR.with_flags(HondaFlags.NIDEC_ALT_PCM_ACCEL)
HONDA_NIDEC_ALT_SCM_MESSAGES = CAR.with_flags(HondaFlags.NIDEC_ALT_SCM_MESSAGES)
HONDA_BOSCH = CAR.with_flags(HondaFlags.BOSCH)
HONDA_BOSCH_RADARLESS = CAR.with_flags(HondaFlags.BOSCH_RADARLESS)
HONDA_BOSCH_CANFD = CAR.with_flags(HondaFlags.BOSCH_CANFD)
HONDA_BOSCH_ALT_RADAR = CAR.with_flags(HondaFlags.BOSCH_ALT_RADAR)
HONDA_BOSCH_TJA_CONTROL = CAR.with_flags(HondaFlags.BOSCH_TJA_CONTROL)
HONDA_LKAS_MINSPEED_CUTOFF = CAR.with_flags(HondaFlags.LKAS_MINSPEED_CUTOFF)
HONDA_BOSCH_VISION_CTRL = CAR.with_flags(HondaFlags.VISION_CTRL)


# Honda 29-bit physical diagnostic addressing: tester 0xF1 -> ECU 0xXX is 0x18DAXXF1, the ECU replies on 0x18DAF1XX
HONDA_DIAG_TX_BASE = 0x18DA00F1
HONDA_DIAG_RX_BASE = 0x18DAF100
HONDA_FWD_CAMERA_DIAG_ADDR = 0x18DAB5F1
# CAN gateway (EU CR-V field notes): never addressed by openpilot, not even with TesterPresent
HONDA_GATEWAY_DIAG_ADDR = 0x18DAEFF1

# Candidate diagnostic addresses of the radar/vision controller (EU CR-V, MDX Type S), in order of preference. Panda
# safety allowlists exactly these (payload-gated to the silence/restore handshake), so the controller search in
# CarController can only ever touch them. Known powertrain/chassis ECUs (EPS, VSA, SRS, PGM-FI, gateway, ...) are
# deliberately not candidates. CarInterface.init() scans the bus and moves the responding candidates to the front;
# CarController then verifies each one empirically (the stock STEERING_CONTROL must stop) before settling on it.
VISION_CTRL_CANDIDATE_ADDRS = [
  # 0xB8..0xBB: the four unknown ADAS ECUs that answered TesterPresent on the MDX Type S PT bus (route
  # ad9840558640c31d/00000008--276649690b, next to the camera and 0xB3); 0xB8 is also the ECU the Honda tester
  # polls on the EU CR-V ACC-CAN. One of them should be the Radar Vision Unit.
  0x18DAB8F1,
  0x18DAB9F1,
  0x18DABAF1,
  0x18DABBF1,
  0x18DAB3F1,  # secondary camera address seen on Bosch radarless cameras; answers on the MDX Type S too
  0x18DAB0F1,  # fwdRadar address on every other Bosch Honda
  0x18DA07F1,  # ECU 0x07, probed in the crveubackup experiments
  # fwdCamera: answers the scan on every car, but it is a sensor of the Radar Vision Unit, not the STEERING_CONTROL
  # author (EU CR-V: 0 stock STEERING_CONTROL frames on the camera bus in 399 relay-open segments). Kept as the last
  # resort only, and never promoted by the scan: silencing it blinds the controller for the length of the probe.
  HONDA_FWD_CAMERA_DIAG_ADDR,
]


DBC = CAR.create_dbc_map()


STEER_THRESHOLD = {
  # default is 1200, overrides go here
  CAR.ACURA_RDX: 400,
  CAR.HONDA_CRV_EU: 400,
  CAR.HONDA_ACCORD_11G: 600,
  CAR.HONDA_PILOT_4G: 600,
  CAR.HONDA_PILOT_4G_MMR: 600,
  CAR.HONDA_PASSPORT_4G: 600,
  CAR.ACURA_MDX_4G_MMR: 600,
  CAR.ACURA_MDX_4G_TYPE_S: 600,
  CAR.HONDA_CRV: 600,
  CAR.HONDA_CRV_6G: 600,
  CAR.HONDA_CRV_6G_EU: 600,
  CAR.HONDA_CITY_7G: 600,
  CAR.HONDA_NBOX_2G: 600,
  CAR.HONDA_PASSPORT_4G: 600,
  CAR.HONDA_ODYSSEY_5G_MMR: 600,
  CAR.HONDA_ACCORD_9G: 30,
  CAR.ACURA_MDX_3G: 400,
  CAR.ACURA_TLX_1G: 200,
  CAR.ACURA_RLX_HYBRID: 2400,
}


HONDA_ALT_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xF112)
HONDA_ALT_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(0xF112)


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # Currently used to fingerprint
    Request(
      [StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.UDS_VERSION_RESPONSE],
      bus=1,
    ),

    # Data collection requests:
    # Log manufacturer-specific identifier for current ECUs
    Request(
      [HONDA_ALT_VERSION_REQUEST],
      [HONDA_ALT_VERSION_RESPONSE],
      bus=1,
      logging=True,
    ),
    # Nidec PT bus
    Request(
      [StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.UDS_VERSION_RESPONSE],
      bus=0,
    ),
    # Bosch PT bus
    Request(
      [StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.UDS_VERSION_RESPONSE],
      bus=1,
      obd_multiplexing=False,
    ),
  ],
  # We lose these ECUs without the comma power on these cars.
  # Note that we still attempt to match with them when they are present
  # This is or'd with (ALL_ECUS - ESSENTIAL_ECUS) from fw_versions.py
  non_essential_ecus={
    Ecu.eps: [CAR.ACURA_RDX_3G, CAR.HONDA_ACCORD, CAR.HONDA_E, CAR.HONDA_E_ADVANCE, CAR.ACURA_MDX_4G,
              CAR.HONDA_CRV_SA, CAR.ACURA_MDX_3G, CAR.HONDA_ACCORD_9G, CAR.ACURA_RLX_HYBRID,
              *HONDA_BOSCH_ALT_RADAR,
              *HONDA_BOSCH_RADARLESS, *HONDA_BOSCH_CANFD],
    Ecu.vsa: [CAR.ACURA_RDX_3G, CAR.HONDA_ACCORD, CAR.HONDA_CIVIC, CAR.HONDA_CIVIC_BOSCH, CAR.HONDA_CRV_5G, CAR.HONDA_CRV_HYBRID, CAR.HONDA_E,
              CAR.HONDA_E_ADVANCE, CAR.HONDA_INSIGHT, CAR.HONDA_NBOX_2G, CAR.ACURA_MDX_4G,
              CAR.HONDA_ACCORD_9G, CAR.ACURA_RLX_HYBRID, CAR.ACURA_MDX_3G,
              *HONDA_BOSCH_ALT_RADAR, *HONDA_BOSCH_RADARLESS,
              *HONDA_BOSCH_CANFD],
  },
  extra_ecus=[
    (Ecu.combinationMeter, 0x18da60f1, None),
    (Ecu.programmedFuelInjection, 0x18da10f1, None),
    # The only other ECU on PT bus accessible by camera on radarless Civic
    # This is likely a manufacturer-specific sub-address implementation: the camera responds to this and 0x18dab0f1
    # Unclear what the part number refers to: 8S103 is 'Camera Set Mono', while 36160 is 'Camera Monocular - Honda'
    # TODO: add query back, camera does not support querying both in parallel and 0x18dab0f1 often fails to respond
    # (Ecu.unknown, 0x18DAB3F1, None),
  ],
)

from collections import namedtuple

from cereal import car
from openpilot.selfdrive.car import AngleRateLimit, CarSpecs, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarDocs
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = car.CarParams.Ecu

Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values', 'mux'])


class CAR(Platforms):
  TESLA_AP3_MODEL3 = PlatformConfig(
    [CarDocs("Tesla AP3 Model 3", "All")],
    CarSpecs(mass=1899., wheelbase=2.875, steerRatio=12.0),
    dbc_dict('tesla_model3_vehicle', None, chassis_dbc='tesla_model3_party')
  )
  TESLA_AP3_MODELY = PlatformConfig(
    [CarDocs("Tesla AP3 Model Y", "All")],
    CarSpecs(mass=2072., wheelbase=2.890, steerRatio=12.0),
    dbc_dict('tesla_model3_vehicle', None, chassis_dbc='tesla_model3_party')
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.eps],
      rx_offset=0x08,
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      rx_offset=0x10,
      bus=1,
      obd_multiplexing=False,
    ),
  ]
)


class CANBUS:
  party = 0
  vehicle = 1
  autopilot_party = 2


GEAR_MAP = {
  "DI_GEAR_INVALID": car.CarState.GearShifter.unknown,
  "DI_GEAR_P": car.CarState.GearShifter.park,
  "DI_GEAR_R": car.CarState.GearShifter.reverse,
  "DI_GEAR_N": car.CarState.GearShifter.neutral,
  "DI_GEAR_D": car.CarState.GearShifter.drive,
  "DI_GEAR_SNA": car.CarState.GearShifter.unknown,
}

BUTTONS = [
  # Button(car.CarState.ButtonEvent.Type.leftBlinker, "SCCM_leftStalk", "SCCM_turnIndicatorStalkStatus", [3, 4], None),
  # Button(car.CarState.ButtonEvent.Type.rightBlinker, "SCCM_leftStalk", "SCCM_turnIndicatorStalkStatus", [1, 2], None),
  Button(car.CarState.ButtonEvent.Type.accelCruise, "VCLEFT_switchStatus", "VCLEFT_swcRightScrollTicks", list(range(1, 10)), None),
  Button(car.CarState.ButtonEvent.Type.decelCruise, "VCLEFT_switchStatus", "VCLEFT_swcRightScrollTicks", list(range(-9, 0)), None),
  # Button(car.CarState.ButtonEvent.Type.cancel, "SCCM_rightStalk", "SCCM_rightStalkStatus", [1, 2], None),
  # Button(car.CarState.ButtonEvent.Type.resumeCruise, "SCCM_rightStalk", "SCCM_rightStalkStatus", [3, 4], None),
  Button(car.CarState.ButtonEvent.Type.altButton2, "UI_status2", "UI_activeTouchPoints", [3], None), # 3-finger touch on center display for MADS
  Button(car.CarState.ButtonEvent.Type.gapAdjustCruise, "VCLEFT_switchStatus", "VCLEFT_swcRightTiltRight", [2], 1),  # TODO: directional gap adjustment
]


class CarControllerParams:
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[7.0, 1.6, .3])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[7.0, 5.5, 0.8])
  JERK_LIMIT_MAX = 4.9
  JERK_LIMIT_MIN = -4.9
  ACCEL_TO_SPEED_MULTIPLIER = 3
  TORQUE_TO_ANGLE_MULTIPLIER_OUTER = 4  # Higher = easier to influence when manually steering more than OP
  TORQUE_TO_ANGLE_MULTIPLIER_INNER = 8  # Higher = easier to influence when manually steering less than OP
  TORQUE_TO_ANGLE_DEADZONE = 0.5  # This equates to hands-on level 1, so we don't allow override if not hands-on
  TORQUE_TO_ANGLE_CLIP = 10. # Steering (usually) disengages at 2.5 Nm, this limit exists only in case the EPAS gives bad data
  CONTINUED_OVERRIDE_ANGLE = 10.  # The angle difference between OP and user to continue overriding steering (prevents oscillation)

  def __init__(self, CP):
    pass


DBC = CAR.create_dbc_map()

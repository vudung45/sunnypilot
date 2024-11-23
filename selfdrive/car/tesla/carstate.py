import copy
from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.tesla.values import DBC, CANBUS, GEAR_MAP, BUTTONS
from openpilot.selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.button_states = {button.event_type: False for button in BUTTONS}
    self.can_define = CANDefine(DBC[CP.carFingerprint]['chassis'])

    # Needed by carcontroller
    self.hands_on_level = 0
    self.steer_warning = None
    self.acc_enabled = None
    self.sccm_right_stalk_counter = None
    self.das_control = None
    self.steering_override = False  # Set in CC because actuator info is needed to determine this.

  def update(self, cp, cp_cam, cp_adas):
    ret = car.CarState.new_message()

    # Vehicle speed
    ret.vEgoRaw = cp.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = (ret.vEgo < 0.1)

    # Gas pedal
    pedal_status = cp.vl["DI_systemStatus"]["DI_accelPedalPos"]
    ret.gas = pedal_status / 100.0
    ret.gasPressed = (pedal_status > 0)

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp.vl["IBST_status"]["IBST_driverBrakeApply"] == 2

    # Steering wheel
    epas_status = cp.vl["EPAS3S_sysStatus"]
    self.hands_on_level = epas_status["EPAS3S_handsOnLevel"]
    self.steer_warning = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacErrorCode"].get(int(epas_status["EPAS3S_eacErrorCode"]), None)
    ret.steeringAngleDeg = -epas_status["EPAS3S_internalSAS"]
    ret.steeringRateDeg = -cp_adas.vl["SCCM_steeringAngleSensor"]["SCCM_steeringAngleSpeed"]
    ret.steeringTorque = -epas_status["EPAS3S_torsionBarTorque"]

    ret.steeringPressed = (self.hands_on_level > 0 or self.steering_override or self.update_steering_pressed(abs(ret.steeringTorque) > 1.0, 5))  # hands_on_level has too much filtering
    eac_status = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacStatus"].get(int(epas_status["EPAS3S_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status in ["EAC_FAULT"]
    ret.steerFaultTemporary = self.steer_warning not in ["EAC_ERROR_IDLE", "EAC_ERROR_HANDS_ON"] and eac_status not in ["EAC_ACTIVE", "EAC_AVAILABLE"]

    # Cruise state
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(int(cp.vl["DI_state"]["DI_speedUnits"]), None)

    self.acc_enabled = (cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL"))
    ret.cruiseState.enabled = self.acc_enabled
    if speed_units == "KPH":
      ret.cruiseState.speed = cp.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS
    elif speed_units == "MPH":
      ret.cruiseState.speed = cp.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS
    ret.cruiseState.available = ((cruise_state == "STANDBY") or ret.cruiseState.enabled)
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special

    speed_limit = cp_cam.vl["DAS_status"]["DAS_fusedSpeedLimit"]
    ret.cruiseState.speedLimit = self._calculate_speed_limit(speed_limit, speed_units)

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_define.dv["DI_systemStatus"]["DI_gear"].get(int(cp.vl["DI_systemStatus"]["DI_gear"]), "DI_GEAR_INVALID")]

    button_events = []
    for button in BUTTONS:
      if button.mux is not None:  # HACK: handle muxed signals
        indexName = button.can_addr + "Index"
        index = cp_adas.vl[button.can_addr][indexName]
        if index != button.mux:
          continue
      state = (cp_adas.vl[button.can_addr][button.can_msg] in button.values)
      if self.button_states[button.event_type] != state:
        event = car.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
      self.button_states[button.event_type] = state
    self.button_events = button_events

    # Doors
    ret.doorOpen = (cp.vl["UI_warning"]["anyDoorOpen"] == 1)

    # Blinkers
    ret.leftBlinker = (cp_adas.vl["ID3F5VCFRONT_lighting"]["VCFRONT_indicatorLeftRequest"] != 0)
    ret.rightBlinker = (cp_adas.vl["ID3F5VCFRONT_lighting"]["VCFRONT_indicatorRightRequest"] != 0)

    # Seatbelt
    ret.seatbeltUnlatched = cp.vl["UI_warning"]["buckleStatus"] != 1

    # Blindspot
    if self.CP.enableBsm:
      ret.leftBlindspot = cp_cam.vl["DAS_status"]["DAS_blindSpotRearLeft"] != 0
      ret.rightBlindspot = cp_cam.vl["DAS_status"]["DAS_blindSpotRearRight"] != 0

    # AEB
    ret.stockAeb = (cp_cam.vl["DAS_control"]["DAS_aebEvent"] == 1)

    # Messages needed by carcontroller
    # self.sccm_right_stalk_counter = copy.copy(cp_adas.vl["SCCM_rightStalk"]["SCCM_rightStalkCounter"])
    self.das_control = copy.copy(cp_cam.vl["DAS_control"])

    return ret

  def _calculate_speed_limit(self, speed_limit, speed_unit):
    if speed_limit in [0, 155]:
      return 0
    if speed_unit == "KPH":
      return speed_limit * CV.KPH_TO_MS
    elif speed_unit == "MPH":
      return speed_limit * CV.MPH_TO_MS
    return 0

  @staticmethod
  def get_can_parser(CP):
    messages = [
      # sig_address, frequency
      ("ESP_B", 50),
      ("DI_systemStatus", 100),
      ("IBST_status", 25),
      ("DI_state", 10),
      ("EPAS3S_sysStatus", 100),
      ("UI_warning", 10)
    ]

    return CANParser(DBC[CP.carFingerprint]['chassis'], messages, CANBUS.party)

  @staticmethod
  def get_cam_can_parser(CP):
    messages = [
      ("DAS_control", 25),
      ("DAS_status", 2)
    ]

    return CANParser(DBC[CP.carFingerprint]['chassis'], messages, CANBUS.autopilot_party)

  @staticmethod
  def get_adas_can_parser(CP):  # Vehicle Can on Model 3
    messages = [
      ("VCLEFT_switchStatus", 20),
      # ("SCCM_leftStalk", 10),
      # ("SCCM_rightStalk", 10),
      ("SCCM_steeringAngleSensor", 100),
      ("DAS_bodyControls", 2),
      ("ID3F5VCFRONT_lighting", 10),
      ("UI_status2", 2),  # variable rate, minimum 2hz
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.vehicle)

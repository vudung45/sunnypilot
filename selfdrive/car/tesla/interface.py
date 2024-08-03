#!/usr/bin/env python3
from cereal import car
from panda import Panda
from openpilot.selfdrive.car.tesla.values import CAR
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase

ButtonType = car.CarState.ButtonEvent.Type


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.last_full_press = False
    self.last_gear = car.CarState.GearShifter.unknown

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "tesla"

    # There is no safe way to do steer blending with user torque,
    # so the steering behaves like autopilot. This is not
    # how openpilot should be, hence dashcamOnly
    # ret.dashcamOnly = True

    ret.steerControlType = car.CarParams.SteerControlType.angle

    ret.longitudinalActuatorDelay = 0.5 # s
    ret.radarUnavailable = True

    if candidate in [CAR.TESLA_AP3_MODEL3, CAR.TESLA_AP3_MODELY]:
      flags = Panda.FLAG_TESLA_MODEL3_Y
      flags |= Panda.FLAG_TESLA_LONG_CONTROL
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.tesla, flags)]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25
    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam, self.cp_adas)

    if self.enable_mads:
      for b in self.CS.button_events:
        if b.type == ButtonType.altButton1 and b.pressed:  # Rising edge from none to half
          self.last_full_press = False
          self.last_gear = ret.gearShifter
        elif b.type == ButtonType.altButton2 and b.pressed:  # Rising edge from half to full
          self.last_full_press = True
        elif b.type == ButtonType.altButton1 and not b.pressed:  # Falling edge from half to none
          if self.last_gear == car.CarState.GearShifter.drive and ret.gearShifter == car.CarState.GearShifter.drive:
            if self.CS.params_list.tesla_mads_acc_first:
              self.CS.accEnabled = True if not self.last_full_press or self.CS.params_list.tesla_mads_combo else self.CS.accEnabled
              self.CS.madsEnabled = True if self.last_full_press else self.CS.madsEnabled
            else:
              self.CS.madsEnabled = True if not self.last_full_press or self.CS.params_list.tesla_mads_combo else self.CS.madsEnabled
              self.CS.accEnabled = True if self.last_full_press else self.CS.accEnabled
        elif b.type == ButtonType.cancel:
          self.CS.madsEnabled = False
          self.CS.accEnabled = False
      self.CS.madsEnabled = False if self.CS.hands_on_level >= 3 else self.CS.madsEnabled
      self.CS.accEnabled = False if not ret.cruiseState.available else self.CS.accEnabled

    if self.get_sp_pedal_disengage(ret):
      self.CS.madsEnabled, self.CS.accEnabled = self.get_sp_cancel_cruise_state(self.CS.madsEnabled)
      ret.cruiseState.enabled = ret.cruiseState.enabled if not self.enable_mads else False if self.CP.pcmCruise else self.CS.accEnabled

    ret, self.CS = self.get_sp_common_state(ret, self.CS)

    ret.buttonEvents = self.CS.button_events

    events = self.create_common_events(ret, c, pcm_enable=False)

    events, ret = self.create_sp_events(self.CS, ret, events)

    ret.events = events.to_msg()

    return ret

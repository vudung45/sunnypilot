from openpilot.common.numpy_fast import clip
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_std_steer_angle_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.tesla.teslacan import TeslaCAN
from openpilot.selfdrive.car.tesla.values import DBC, CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    super().__init__(dbc_name, CP, VM)
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_name)
    self.pt_packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.tesla_can = TeslaCAN(self.packer, self.pt_packer)
    self.last_right_stalk_press = 0
    self.pcm_cancel_cmd = False # Must be latching because of frame rate
    self.acc_mismatch_start_nanos = None

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    self.pcm_cancel_cmd = CC.cruiseControl.cancel or self.pcm_cancel_cmd

    can_sends = []

    # Temp disable steering on a hands_on_fault, and allow for user override
    hands_on_fault = CS.hands_on_level >= 3
    lkas_enabled = CC.latActive and not hands_on_fault

    if self.frame % 2 == 0:
      if lkas_enabled:
        # Angular rate limit based on speed
        apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgo, CarControllerParams)

        # To not fault the EPS
        apply_angle = clip(apply_angle, CS.out.steeringAngleDeg - 20, CS.out.steeringAngleDeg + 20)
      else:
        apply_angle = CS.out.steeringAngleDeg

      self.apply_angle_last = apply_angle
      use_lka_mode = CS.params_list.enable_mads
      can_sends.append(self.tesla_can.create_steering_control(apply_angle, lkas_enabled, (self.frame // 2) % 16, use_lka_mode))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      acc_state = CS.das_control["DAS_accState"]
      target_accel = actuators.accel
      target_speed = max(CS.out.vEgo + (target_accel * CarControllerParams.ACCEL_TO_SPEED_MULTIPLIER), 0)
      max_accel = 0 if target_accel < 0 else target_accel
      min_accel = 0 if target_accel > 0 else target_accel

      counter = CS.das_control["DAS_controlCounter"]
      can_sends.append(self.tesla_can.create_longitudinal_commands(acc_state, target_speed, min_accel, max_accel, counter))

    if hands_on_fault and not CS.params_list.enable_mads:
      self.pcm_cancel_cmd = True

    # Cancel ACC if MADS is enabled and ACC is not supposed to be enabled
    if CS.params_list.enable_mads and not CS.accEnabled and CS.acc_enabled:
      # ACC mismatch must persist for a while before cancelling because of race condition
      if self.acc_mismatch_start_nanos is None:
        self.acc_mismatch_start_nanos = now_nanos
      elif now_nanos - self.acc_mismatch_start_nanos > 200e6: # 200ms
        self.pcm_cancel_cmd = True
    else:
      self.acc_mismatch_start_nanos = None

    # Unlatch cancel command only when ACC is actually disabled
    if not CS.acc_enabled:
      self.pcm_cancel_cmd = False

    # Send cancel request only if ACC is enabled
    if self.frame % 10 == 0 and self.pcm_cancel_cmd:
      counter = int(CS.sccm_right_stalk_counter)
      # Alternate between 1 and 0 every 10 frames, car needs time to detect falling edge in order to prevent shift to N
      value = 1 if self.last_right_stalk_press == 0 else 0
      can_sends.append(self.tesla_can.right_stalk_press((counter + 1) % 16 , value))
      self.last_right_stalk_press = value
    elif self.last_right_stalk_press != 0 and self.frame % 10 == 0:
      self.last_right_stalk_press = 0

    # TODO: HUD control

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends

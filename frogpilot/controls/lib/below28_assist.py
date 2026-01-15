#!/usr/bin/env python3

from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import (
  ButtonType,
  CRUISE_INTERVAL_SIGN,
  CRUISE_LONG_PRESS,
  CRUISE_NEAREST_FUNC,
  IMPERIAL_INCREMENT,
  V_CRUISE_MIN,
)

BELOW_28_MPH = 28
BELOW_28_KPH = round(BELOW_28_MPH * CV.MPH_TO_KPH, 1)

class Below28Assist:
  def __init__(self):
    self.active = False
    self.ui_set_speed_kph = 0.0
    self.cap_mps = 0.0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

  def _get_button_event(self, car_state):
    long_press = False
    button_type = None

    for button in car_state.buttonEvents:
      if button.type.raw in self.button_timers and not button.pressed:
        if self.button_timers[button.type.raw] > CRUISE_LONG_PRESS:
          return None, False
        button_type = button.type.raw
        break
    else:
      for button_type, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          long_press = True
          break
      else:
        button_type = None

    return button_type, long_press

  def _get_cruise_delta_interval(self, long_press, frogpilot_toggles):
    tap_delta = frogpilot_toggles.cruise_increase
    hold_delta = frogpilot_toggles.cruise_increase_long
    if frogpilot_toggles.reverse_cruise_increase:
      tap_delta, hold_delta = hold_delta, tap_delta

    return hold_delta if long_press else tap_delta

  def _step_speed(self, speed_kph, button_type, long_press, frogpilot_toggles):
    v_cruise_delta = 1.0 if frogpilot_toggles.is_metric else IMPERIAL_INCREMENT
    v_cruise_delta_interval = self._get_cruise_delta_interval(long_press, frogpilot_toggles)
    v_cruise_delta *= v_cruise_delta_interval

    if v_cruise_delta_interval % 5 == 0 and speed_kph % v_cruise_delta != 0:
      speed_kph = CRUISE_NEAREST_FUNC[button_type](speed_kph / v_cruise_delta) * v_cruise_delta
    else:
      speed_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    return speed_kph

  def _reset(self, v_cruise_kph):
    self.active = False
    self.ui_set_speed_kph = v_cruise_kph
    self.cap_mps = 0.0

  def update_button_timers(self, car_state, enabled):
    for button_type in self.button_timers:
      if self.button_timers[button_type] > 0:
        self.button_timers[button_type] += 1

    for button in car_state.buttonEvents:
      if button.type.raw in self.button_timers:
        self.button_timers[button.type.raw] = 1 if button.pressed else 0
        self.button_change_states[button.type.raw] = {
          "standstill": car_state.cruiseState.standstill,
          "enabled": enabled,
        }

  def update(self, car_state, enabled, long_control_active, v_cruise_kph, frogpilot_toggles, slc_confirmation_active):
    button_type, long_press = self._get_button_event(car_state)

    if slc_confirmation_active:
      button_type = None
      long_press = False

    if (not enabled or not long_control_active or not car_state.cruiseState.available or v_cruise_kph > BELOW_28_KPH):
      self._reset(v_cruise_kph)
      self.update_button_timers(car_state, enabled)
      return

    if not self.active:
      self.ui_set_speed_kph = min(v_cruise_kph, BELOW_28_KPH)

    if button_type is not None:
      cruise_standstill = self.button_change_states[button_type]["standstill"] or car_state.cruiseState.standstill
      if button_type == ButtonType.accelCruise and cruise_standstill:
        pass
      elif not self.button_change_states[button_type]["enabled"]:
        pass
      else:
        self.ui_set_speed_kph = self._step_speed(self.ui_set_speed_kph, button_type, long_press, frogpilot_toggles)

    self.ui_set_speed_kph = min(max(round(self.ui_set_speed_kph, 1), V_CRUISE_MIN), BELOW_28_KPH)
    self.active = True
    self.cap_mps = self.ui_set_speed_kph * CV.KPH_TO_MS

    self.update_button_timers(car_state, enabled)

#!/usr/bin/env python3
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.controls.lib.drive_helpers import (
  ButtonType,
  CRUISE_INTERVAL_SIGN,
  CRUISE_LONG_PRESS,
  CRUISE_NEAREST_FUNC,
  IMPERIAL_INCREMENT,
  V_CRUISE_MAX,
  V_CRUISE_MIN,
)

BELOW_28_MPH = 28
BELOW_28_KPH = BELOW_28_MPH * CV.MPH_TO_KPH


class Below28Assist:
  def __init__(self):
    self.active = False
    self.ui_set_speed_kph = 0.0
    self.cap_mps = 0.0
    self.last_below28_set_kph = 0.0

    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

  def reset(self):
    self.active = False
    self.ui_set_speed_kph = 0.0
    self.cap_mps = 0.0
    self.reset_button_timers()

  def reset_button_timers(self):
    for button_type in self.button_timers:
      self.button_timers[button_type] = 0
      self.button_change_states[button_type] = {"standstill": False, "enabled": False}

  def update(self, v_cruise_kph, v_ego, v_ego_diff, CS, enabled, speed_limit_changed, frogpilot_toggles):
    buttons_enabled = enabled or CS.cruiseState.enabled
    if not buttons_enabled:
      self.reset()
      self.update_button_timers(CS, buttons_enabled)
      return

    current_speed_kph = self.ui_set_speed_kph if self.active else v_cruise_kph
    updated_speed = self.update_set_speed(current_speed_kph, v_ego, v_ego_diff, CS, buttons_enabled, speed_limit_changed, frogpilot_toggles)

    if updated_speed is not None:
      self.ui_set_speed_kph = updated_speed
      self.active = self.ui_set_speed_kph < BELOW_28_KPH
      if self.active:
        self.last_below28_set_kph = self.ui_set_speed_kph
    elif self.active and self.ui_set_speed_kph >= BELOW_28_KPH:
      self.active = False

    if not self.active:
      self.ui_set_speed_kph = v_cruise_kph

    if self.active:
      self.cap_mps = max(self.ui_set_speed_kph * CV.KPH_TO_MS - v_ego_diff, 0.0)
    else:
      self.cap_mps = 0.0

    self.update_button_timers(CS, buttons_enabled)

  def update_set_speed(self, v_cruise_kph, v_ego, v_ego_diff, CS, buttons_enabled, speed_limit_changed, frogpilot_toggles):
    if not buttons_enabled:
      return None

    long_press = False
    button_type = None

    for b in CS.buttonEvents:
      if b.type.raw in (ButtonType.setCruise, ButtonType.resumeCruise) and not b.pressed:
        if speed_limit_changed:
          return None
        if b.type.raw == ButtonType.resumeCruise and self.last_below28_set_kph > 0:
          self.reset_button_timers()
          return self.last_below28_set_kph
        if b.type.raw == ButtonType.setCruise:
          v_ego_cluster = v_ego + v_ego_diff
          set_speed_kph = v_ego_cluster * CV.MS_TO_KPH
          self.reset_button_timers()
          return clip(round(set_speed_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return None
        button_type = b.type.raw
        break
    else:
      for k in self.button_timers.keys():
        if self.button_timers[k] and self.button_timers[k] % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return None

    if speed_limit_changed:
      return None

    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return None

    if not self.button_change_states[button_type]["enabled"]:
      return None

    v_cruise_delta = 1.0 if frogpilot_toggles.is_metric else IMPERIAL_INCREMENT

    tap_increment = frogpilot_toggles.cruise_increase
    hold_increment = frogpilot_toggles.cruise_increase_long
    if frogpilot_toggles.reverse_cruise_increase:
      tap_increment, hold_increment = hold_increment, tap_increment

    v_cruise_delta_interval = hold_increment if long_press else tap_increment
    v_cruise_delta *= v_cruise_delta_interval

    if v_cruise_delta_interval % 5 == 0 and v_cruise_kph % v_cruise_delta != 0:
      v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    v_cruise_kph = clip(round(v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

    return v_cruise_kph

  def update_button_timers(self, CS, buttons_enabled):
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": buttons_enabled}

#!/usr/bin/env python3
"""
Below28Assist: Enables cruise set speed below 28 mph for 2017 Toyota Corolla
with SDSU + Comma Pedal, using SLC cap injection for enforcement.

This file contains all business logic for the below-28 feature:
- State machine for tracking below-28 mode
- Step computation using existing OP stepping helpers
- Button event handling with proper dedupe
- Resume + set-current support when cruise on but OP not engaged
- Outputs for UI display (kph) and enforcement cap (m/s)
"""
import math
from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.controls.lib.drive_helpers import (
  IMPERIAL_INCREMENT, CRUISE_NEAREST_FUNC, CRUISE_INTERVAL_SIGN,
  V_CRUISE_MIN, V_CRUISE_MAX, CRUISE_LONG_PRESS
)

ButtonType = car.CarState.ButtonEvent.Type

# 28 mph in kph, snapped to imperial grid
# 28 mph * 1.609344 = 45.06 kph
FLOOR_MPH = 28
FLOOR_KPH = round(FLOOR_MPH * CV.MPH_TO_KPH / IMPERIAL_INCREMENT) * IMPERIAL_INCREMENT


class Below28Assist:
  """
  State machine for below-28 mph cruise control.

  Handles:
  - Transition across the 28 mph boundary (both directions)
  - Button press/release dedupe for responsive taps
  - Resume last set speed when cruise enabled
  - Set to current speed when pressing set/decel
  - Enforcement cap output for SLC injection
  """

  def __init__(self):
    # Core state
    self.active = False
    self.ui_set_speed_kph = 0.0

    # Last set speed for resume functionality
    self.last_set_speed_kph = 0.0
    self.last_set_valid = False

    # Button timers - mirrors VCruiseHelper exactly
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

    # Track previous frame state for edge detection
    self.prev_enabled = False
    self.prev_cruise_available = False

  def update(self, v_cruise_kph, v_ego, CS, enabled, is_metric, frogpilot_toggles):
    """
    Main update function called each frame.

    Args:
      v_cruise_kph: Current cruise set speed from VCruiseHelper (kph)
      v_ego: Current vehicle speed (m/s)
      CS: CarState message
      enabled: Whether openpilot is engaged (active control)
      is_metric: Whether display is in metric units
      frogpilot_toggles: FrogPilot toggle settings

    Returns:
      tuple: (below28_active, below28_ui_set_speed_kph, below28_cap_mps)
    """
    cruise_available = CS.cruiseState.available

    # Reset state when cruise becomes unavailable
    if not cruise_available:
      self._reset_state()
      self.prev_enabled = enabled
      self.prev_cruise_available = cruise_available
      return self.active, self.ui_set_speed_kph, 0.0

    # Get step settings from toggles
    step_size = frogpilot_toggles.cruise_increase if hasattr(frogpilot_toggles, 'cruise_increase') else 1
    step_size_long = frogpilot_toggles.cruise_increase_long if hasattr(frogpilot_toggles, 'cruise_increase_long') else 5

    # Always use imperial increment for stepping (matches VCruiseHelper behavior)
    v_cruise_delta = IMPERIAL_INCREMENT

    if enabled:
      # OpenPilot is engaged - handle normal stepping
      # Note: button processing happens BEFORE timer update (mirrors VCruiseHelper)
      self._handle_engaged_stepping(v_cruise_kph, v_cruise_delta, step_size, step_size_long, CS)
    else:
      # Cruise on but OP not engaged - handle resume/set-current
      self._handle_not_engaged(CS, v_ego, v_cruise_kph, v_cruise_delta)

    # Update button timers AFTER processing (mirrors VCruiseHelper order)
    self._update_button_timers(CS, enabled)

    # Store last set speed for resume (when valid)
    if self.ui_set_speed_kph > 0:
      self.last_set_speed_kph = self.ui_set_speed_kph
      self.last_set_valid = True

    # Track state transitions
    self.prev_enabled = enabled
    self.prev_cruise_available = cruise_available

    # Compute enforcement cap (m/s) when active
    below28_cap_mps = 0.0
    if self.active and self.ui_set_speed_kph < FLOOR_KPH:
      below28_cap_mps = self.ui_set_speed_kph * CV.KPH_TO_MS

    return self.active, self.ui_set_speed_kph, below28_cap_mps

  def _reset_state(self):
    """Reset state when cruise becomes unavailable."""
    self.active = False
    self.ui_set_speed_kph = 0.0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

  def _update_button_timers(self, CS, enabled):
    """
    Update button press timers - mirrors VCruiseHelper.update_button_timers exactly.
    """
    # Increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def _handle_engaged_stepping(self, v_cruise_kph, v_cruise_delta, step_size, step_size_long, CS):
    """
    Handle cruise speed stepping while OpenPilot is engaged.

    Mirrors VCruiseHelper._update_v_cruise_non_pcm logic but extends below 28.
    Only processes button events if we're currently below floor or about to go below.
    """
    long_press = False
    button_type = None

    # Check for button release (short press) - mirrors VCruiseHelper lines 87-92
    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          # End of long press - no action on release
          return
        button_type = b.type.raw
        break
    else:
      # Check for long press tick - mirrors VCruiseHelper lines 94-98
      for k in self.button_timers.keys():
        if self.button_timers[k] and self.button_timers[k] % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      # No button action - sync with upstream v_cruise if we're above floor
      if v_cruise_kph >= FLOOR_KPH:
        self.active = False
        self.ui_set_speed_kph = v_cruise_kph
      elif self.active:
        # Already active (below floor), keep current state
        pass
      else:
        # Not active and v_cruise is at/above floor - sync
        self.ui_set_speed_kph = v_cruise_kph
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed
    if not self.button_change_states[button_type]["enabled"]:
      return

    # Calculate step interval
    step_interval = step_size_long if long_press else step_size

    # Calculate delta
    delta = v_cruise_delta * step_interval * CRUISE_INTERVAL_SIGN[button_type]

    # Determine current set speed to use as base
    if self.active:
      current_speed = self.ui_set_speed_kph
    else:
      current_speed = v_cruise_kph

    # Calculate new speed with grid snapping (mirrors VCruiseHelper lines 118-121)
    step_delta = v_cruise_delta * step_interval
    if step_interval % 5 == 0 and current_speed % step_delta != 0:
      # Snap to grid for 5-step intervals
      new_speed = CRUISE_NEAREST_FUNC[button_type](current_speed / step_delta) * step_delta
    else:
      new_speed = current_speed + delta

    # Clip to valid range (but allow below FLOOR_KPH down to V_CRUISE_MIN)
    new_speed = clip(round(new_speed, 1), V_CRUISE_MIN, V_CRUISE_MAX)

    # Update state based on whether we're below floor
    self.ui_set_speed_kph = new_speed
    self.active = new_speed < FLOOR_KPH

  def _handle_not_engaged(self, CS, v_ego, v_cruise_kph, v_cruise_delta):
    """
    Handle cruise control when cruise is on but OpenPilot is not engaged.

    Supports:
    - UP (resume): Restore last set speed
    - DOWN (set): Set to current vehicle speed

    Note: This is called when cruise is available but OP not engaged,
    which means we're waiting for the user to press a button to engage.
    """
    for b in CS.buttonEvents:
      if not b.pressed:
        continue

      # Resume button pressed - restore last set speed
      if b.type in (ButtonType.accelCruise, ButtonType.resumeCruise):
        if self.last_set_valid and self.last_set_speed_kph > 0:
          self.ui_set_speed_kph = self.last_set_speed_kph
          self.active = self.ui_set_speed_kph < FLOOR_KPH
        elif v_cruise_kph > 0 and v_cruise_kph < 250:
          # Use upstream last speed if we don't have our own
          self.ui_set_speed_kph = v_cruise_kph
          self.active = self.ui_set_speed_kph < FLOOR_KPH
        return

      # Set/decel button pressed - set to current speed
      if b.type in (ButtonType.decelCruise, ButtonType.setCruise):
        # Get current speed from cluster if available, else vEgo
        v_ego_kph = (CS.vEgoCluster if CS.vEgoCluster > 0 else v_ego) * CV.MS_TO_KPH

        # Snap to imperial grid
        new_speed = round(v_ego_kph / v_cruise_delta) * v_cruise_delta
        new_speed = clip(round(new_speed, 1), V_CRUISE_MIN, V_CRUISE_MAX)

        self.ui_set_speed_kph = new_speed
        self.active = new_speed < FLOOR_KPH

        # Save as last set speed
        self.last_set_speed_kph = new_speed
        self.last_set_valid = True
        return

  def get_enforcement_cap_mps(self):
    """Get the enforcement cap in m/s, or 0 if not active."""
    if self.active and self.ui_set_speed_kph < FLOOR_KPH:
      return self.ui_set_speed_kph * CV.KPH_TO_MS
    return 0.0

#!/usr/bin/env python3
"""
Below-28 mph speed controller for Toyota PCM cruise (gas interceptor users).

Intercepts button presses at or below the PCM floor (28 mph) and routes them
to a controlled variable instead of the default car system. This variable is
enforced through the SLC path and displayed as the set speed in the UI.

That's all the logic: intercept buttons below floor → our variable → SLC → UI.
"""
from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.controls.lib.drive_helpers import CRUISE_NEAREST_FUNC, CRUISE_INTERVAL_SIGN

ButtonType = car.CarState.ButtonEvent.Type

# Toyota PCM cruise floor (28 mph)
FLOOR_MPH = 28


class ToyotaLowSpeedOverride:
  def __init__(self):
    self.active = False
    self.speed_mph = FLOOR_MPH
    self.target = 0  # in m/s, routed through SLC

    self._prev_cruise_enabled = False  # Track cruise activation for SET/RES handling
    self._saved_speed_mph = 0  # Saved speed for RES to resume below floor

  @staticmethod
  def _snap_speed(speed, delta, button_type):
    """Snap to nearest delta boundary using openpilot's CRUISE_NEAREST_FUNC, or step by delta if aligned."""
    if delta >= 5 and speed % delta != 0:
      return CRUISE_NEAREST_FUNC[button_type](speed / delta) * delta
    return speed + delta * CRUISE_INTERVAL_SIGN[button_type]

  def update(self, carState, controlsState, v_cruise_cluster_ms, frogpilot_toggles):
    if not frogpilot_toggles.toyota_low_speed_override:
      self.active = False
      self.target = 0
      return

    # Detect cruise activation (SET or RES to engage from off)
    cruise_just_activated = carState.cruiseState.enabled and not self._prev_cruise_enabled
    self._prev_cruise_enabled = carState.cruiseState.enabled

    # Deactivate on cruise disable or cancel — save speed for RES resume
    if not carState.cruiseState.enabled or not controlsState.enabled:
      if self.active:
        self._saved_speed_mph = self.speed_mph
      self.active = False
      self.speed_mph = FLOOR_MPH
      self.target = 0
      return

    if any(be.type == ButtonType.cancel and be.pressed for be in carState.buttonEvents):
      if self.active:
        self._saved_speed_mph = self.speed_mph
      self.active = False
      self.speed_mph = FLOOR_MPH
      self.target = 0
      return

    # Read button presses from openpilot's existing button event system
    decel_pressed = any(be.type == ButtonType.decelCruise and be.pressed for be in carState.buttonEvents)
    accel_pressed = any(be.type == ButtonType.accelCruise and be.pressed for be in carState.buttonEvents)

    v_cruise_mph = int(round(v_cruise_cluster_ms * CV.MS_TO_MPH))
    v_ego_mph = int(round(carState.vEgo * CV.MS_TO_MPH))
    delta = 1 if frogpilot_toggles.reverse_cruise_increase else 5

    # Handle cruise activation (SET or RES from off)
    if cruise_just_activated:
      if decel_pressed and v_ego_mph < FLOOR_MPH:
        # SET to activate below floor — set to current speed
        self.active = True
        self.speed_mph = max(1, v_ego_mph)
      elif accel_pressed and 0 < self._saved_speed_mph < FLOOR_MPH:
        # RES to resume below floor — restore previous override speed
        self.active = True
        self.speed_mph = self._saved_speed_mph
      # Clear saved speed on any activation (consumed or not relevant)
      self._saved_speed_mph = 0
    else:
      # Normal button presses while cruising — use openpilot's CRUISE_NEAREST_FUNC for boundary snapping
      if decel_pressed:
        if self.active:
          self.speed_mph = max(1, self._snap_speed(self.speed_mph, delta, ButtonType.decelCruise))
        elif v_cruise_mph <= FLOOR_MPH:
          # At floor — activate and snap down (28 snaps to 25 with delta=5)
          self.active = True
          self.speed_mph = max(1, self._snap_speed(FLOOR_MPH, delta, ButtonType.decelCruise))

      if accel_pressed and self.active:
        self.speed_mph = self._snap_speed(self.speed_mph, delta, ButtonType.accelCruise)

    # Hand back to PCM when our variable reaches the floor
    if self.speed_mph >= FLOOR_MPH:
      self.active = False
      self.speed_mph = FLOOR_MPH
      self._saved_speed_mph = 0  # Clear — user moved past floor, PCM takes over

    # Set target in m/s — this gets routed through SLC in frogpilot_vcruise.py
    self.target = self.speed_mph * CV.MPH_TO_MS if self.active else 0

#!/usr/bin/env python3
"""
Toyota Low-Speed Cruise Override for FrogPilot

This module implements below-28-mph cruise set-speed support for Toyota vehicles
with PCM cruise and pedal interceptor (SDSU). The PCM won't report set speeds
below 28 mph (~45 kph), so this module provides OpenPilot-owned set speed control
when operating below that floor.

Key behaviors:
- SET/- while below 28 mph: set speed becomes current vehicle speed (not 28 mph)
- RES/+ with previous below-28 speed: resumes to that stored speed
- Increment/decrement logic respects FrogPilot's reverse_cruise_increase toggle:
  - Normal: tap=1, hold=1 per interval
  - Reverse: tap=5, hold=5 per interval
- Resume-from-standstill does NOT trigger speed recalculation
- Seamless transition back to PCM control when set speed exceeds floor
- Previous below-28 speed is preserved across cruise disengagement for RES/+ resume

Author: FrogPilot
"""
import math

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip

# Constants
V_CRUISE_MIN = 8  # kph
V_CRUISE_MAX = 145  # kph
V_CRUISE_UNSET = 255
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)

# PCM cruise set speed floor for Toyota cars (28 mph in kph)
V_CRUISE_PCM_FLOOR = round(28 * CV.MPH_TO_KPH, 1)  # ~45 kph

# Frame count threshold for long press detection (50 frames @ 100Hz = 0.5 seconds)
CRUISE_LONG_PRESS = 50

ButtonType = car.CarState.ButtonEvent.Type


class ToyotaLowSpeedCruise:
  """
  Manages low-speed cruise override for Toyota vehicles with pedal interceptor.

  This class encapsulates all logic for setting and adjusting cruise speed below
  the PCM's 28 mph floor. The VCruiseHelper in drive_helpers.py maintains minimal
  state and delegates to this class for all low-speed cruise decisions.
  """

  # Mapping of button types to rounding functions for 5 mph/kph intervals
  CRUISE_NEAREST_FUNC = {
    ButtonType.accelCruise: math.ceil,
    ButtonType.decelCruise: math.floor,
  }

  # Mapping of button types to +/- increment direction
  CRUISE_INTERVAL_SIGN = {
    ButtonType.accelCruise: +1,
    ButtonType.decelCruise: -1,
  }

  def __init__(self):
    # Override state
    self.low_speed_override_active = False
    self.v_cruise_override_kph = V_CRUISE_UNSET

    # Track last known set speed for restore functionality
    self._last_v_cruise_below_floor = V_CRUISE_UNSET

  def reset(self):
    """Reset all override state (called when cruise becomes unavailable)."""
    self.low_speed_override_active = False
    self.v_cruise_override_kph = V_CRUISE_UNSET

  def compute_new_set_speed(
    self,
    v_ego_kph: float,
    pcm_v_cruise_kph: float,
    is_enabled: bool,
    is_metric: bool,
    button_type,  # ButtonType enum value or None
    long_press: bool,
    cruise_standstill: bool,
    button_was_enabled: bool,
    speed_limit_changed: bool,
    frogpilot_toggles,
  ) -> tuple[float, bool]:
    """
    Compute the new set speed for low-speed cruise override.

    Args:
      v_ego_kph: Current vehicle speed in kph
      pcm_v_cruise_kph: PCM-reported cruise set speed in kph
      is_enabled: Whether cruise control is currently enabled
      is_metric: Whether to use metric (kph) or imperial (mph) increments
      button_type: The button that was pressed (accelCruise, decelCruise, or None)
      long_press: Whether this is a long press event
      cruise_standstill: Whether we're in standstill state
      button_was_enabled: Whether cruise was enabled when button was first pressed
      speed_limit_changed: Whether speed limit just changed (don't adjust speed)
      frogpilot_toggles: FrogPilot toggle settings

    Returns:
      Tuple of (new_v_cruise_kph, override_active)
      - new_v_cruise_kph: The new set speed to use
      - override_active: Whether low-speed override mode is active
    """
    # Check if we should enter low-speed override mode
    # PCM must be at floor AND we're enabled AND not in standstill resume
    at_pcm_floor = pcm_v_cruise_kph <= V_CRUISE_PCM_FLOOR + 2  # Small tolerance for PCM rounding

    # Entry conditions for low-speed override:
    # 1. SET/- (decelCruise): Set to current vehicle speed (even if below 28 mph)
    # 2. RES/+ (accelCruise): Resume to previous below-28 speed (if we have one stored)
    entering_via_set = (
      not self.low_speed_override_active and
      is_enabled and
      button_type == ButtonType.decelCruise and
      at_pcm_floor and
      not cruise_standstill  # Don't enter on resume from standstill
    )

    entering_via_resume = (
      not self.low_speed_override_active and
      is_enabled and
      button_type == ButtonType.accelCruise and
      at_pcm_floor and
      not cruise_standstill and
      self._last_v_cruise_below_floor != V_CRUISE_UNSET and
      self._last_v_cruise_below_floor < V_CRUISE_PCM_FLOOR  # Only if previous was actually below floor
    )

    # Track if we just entered (to skip adjustment on entry press)
    just_entered = False

    if entering_via_set:
      self.low_speed_override_active = True
      just_entered = True
      # SET/-: Use current vehicle speed, not PCM floor value
      # When driver presses SET at 15 mph, they want 15 mph, not 28 mph
      initial_speed = min(v_ego_kph, pcm_v_cruise_kph)
      # Ensure we start below the floor (that's the whole point)
      initial_speed = min(initial_speed, V_CRUISE_PCM_FLOOR - 1)
      # Apply minimum
      self.v_cruise_override_kph = max(initial_speed, V_CRUISE_MIN)
      # Store for potential restore later
      self._last_v_cruise_below_floor = self.v_cruise_override_kph

    elif entering_via_resume:
      self.low_speed_override_active = True
      just_entered = True
      # RES/+: Resume to previous below-28 speed
      self.v_cruise_override_kph = self._last_v_cruise_below_floor

    if self.low_speed_override_active:
      # Process button presses while in low-speed override mode
      # Skip adjustment on the button press that triggered entry
      if is_enabled and button_type is not None and not speed_limit_changed and not just_entered:
        # Don't adjust speed when pressing resume to exit standstill
        if not (button_type == ButtonType.accelCruise and cruise_standstill):
          # Don't adjust if we enabled since button was pressed
          if button_was_enabled:
            self._apply_speed_adjustment(button_type, long_press, is_metric, frogpilot_toggles)

      # Check if we should exit low-speed override mode
      # Exit when: OP-owned set speed goes at or above the PCM floor
      if self.v_cruise_override_kph >= V_CRUISE_PCM_FLOOR:
        self.low_speed_override_active = False
        # Return our computed value (not PCM) for smooth transition
        # This prevents flicker from PCM having a different value due to
        # double button processing (PCM also processes buttons via CAN)
        return self.v_cruise_override_kph, False

      # Update last known below-floor speed
      self._last_v_cruise_below_floor = self.v_cruise_override_kph

      return self.v_cruise_override_kph, True

    # Not in override mode - return PCM value
    return pcm_v_cruise_kph, False

  def _apply_speed_adjustment(
    self,
    button_type,
    long_press: bool,
    is_metric: bool,
    frogpilot_toggles,
  ):
    """
    Apply speed increment/decrement based on button press.

    Toyota PCM cruise button behavior (mirrored here for OP-owned low-speed control):
    - Normal (reverse_cruise_increase=False): tap=1, hold=1 per interval (~0.5s)
    - Reverse (reverse_cruise_increase=True): tap=5, hold=5 per interval (~0.5s)

    The reverse toggle changes increment from 1 to 5 for both tap and hold.
    """
    # Base increment unit (1 kph for metric, ~1.6 kph for imperial)
    v_cruise_delta = 1.0 if is_metric else IMPERIAL_INCREMENT

    # Toyota PCM cruise button logic with reverse_cruise_increase toggle:
    # - reverse=False (normal): tap=1, hold=1
    # - reverse=True: tap=5, hold=5
    reverse_enabled = getattr(frogpilot_toggles, 'reverse_cruise_increase', False)
    v_cruise_delta_interval = 5 if reverse_enabled else 1

    v_cruise_delta = v_cruise_delta * v_cruise_delta_interval

    # Apply rounding for 5 mph/kph intervals (snap to grid on long press)
    if v_cruise_delta_interval % 5 == 0 and self.v_cruise_override_kph % v_cruise_delta != 0:
      self.v_cruise_override_kph = self.CRUISE_NEAREST_FUNC[button_type](
        self.v_cruise_override_kph / v_cruise_delta
      ) * v_cruise_delta
    else:
      self.v_cruise_override_kph += v_cruise_delta * self.CRUISE_INTERVAL_SIGN[button_type]

    # Apply long-press offset if configured (for non-PCM cruise, but check anyway)
    set_speed_offset = getattr(frogpilot_toggles, 'set_speed_offset', 0)
    if long_press and set_speed_offset != 0:
      v_cruise_offset = set_speed_offset * self.CRUISE_INTERVAL_SIGN[button_type]
      if v_cruise_offset < 0:
        v_cruise_offset = set_speed_offset - v_cruise_delta
      self.v_cruise_override_kph += v_cruise_offset

    # Clip to valid range
    self.v_cruise_override_kph = clip(round(self.v_cruise_override_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  def get_last_below_floor_speed(self) -> float:
    """Get the last known set speed that was below the PCM floor."""
    return self._last_v_cruise_below_floor

  @property
  def is_active(self) -> bool:
    """Whether low-speed override mode is currently active."""
    return self.low_speed_override_active


# Singleton instance for use by VCruiseHelper
_instance = None


def get_toyota_low_speed_cruise() -> ToyotaLowSpeedCruise:
  """Get the singleton ToyotaLowSpeedCruise instance."""
  global _instance
  if _instance is None:
    _instance = ToyotaLowSpeedCruise()
  return _instance

import math
import numpy as np

from cereal import car, log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY

# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

# PCM cruise set speed floor for Toyota cars (28 mph in kph)
# The PCM won't report set speeds below this, so we need OP-owned control below this threshold
V_CRUISE_PCM_FLOOR = round(28 * CV.MPH_TO_KPH, 1)  # ~45 kph

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0
# This is a turn radius smaller than most cars can achieve
MAX_CURVATURE = 0.2

# EU guidelines
MAX_LATERAL_JERK = 5.0
MAX_LATERAL_ACCEL_NO_ROLL = 3.0  # m/s^2
MAX_VEL_ERR = 5.0

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

    # Low-speed override mode for Toyota with pedal (gas interceptor)
    # When enabled, allows setting cruise speed below the PCM's 28 mph floor
    self.low_speed_override_active = False
    self._last_v_cruise_below_floor = V_CRUISE_UNSET  # For RES/+ resume to previous below-floor speed

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric, speed_limit_changed, frogpilot_toggles):
    self.v_cruise_kph_last = self.v_cruise_kph

    if CS.cruiseState.available:
      if not self.CP.pcmCruise:
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self._update_v_cruise_non_pcm(CS, enabled, is_metric, speed_limit_changed, frogpilot_toggles)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self.update_button_timers(CS, enabled)
      else:
        # PCM cruise mode - but check for low-speed override (Toyota with pedal)
        pcm_v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        pcm_v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH

        if CS.cruiseState.speed == 0:
          self.v_cruise_kph = V_CRUISE_UNSET
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
          self.low_speed_override_active = False
        elif getattr(frogpilot_toggles, 'toyota_low_speed_override', False):
          # Low-speed override enabled - handle OP-owned set speed below PCM floor
          self._update_v_cruise_low_speed_override(CS, enabled, is_metric, speed_limit_changed, frogpilot_toggles, pcm_v_cruise_kph, pcm_v_cruise_cluster_kph)
        else:
          # Normal PCM cruise behavior
          self.v_cruise_kph = pcm_v_cruise_kph
          self.v_cruise_cluster_kph = pcm_v_cruise_cluster_kph
          self.low_speed_override_active = False
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET
      self.low_speed_override_active = False

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric, speed_limit_changed, frogpilot_toggles):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k in self.button_timers.keys():
        if self.button_timers[k] and self.button_timers[k] % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # Don't adjust speed when pressing to confirm/deny speed limits
    if speed_limit_changed:
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    if not self.button_change_states[button_type]["enabled"]:
      return

    v_cruise_delta_interval = frogpilot_toggles.cruise_increase_long if long_press else frogpilot_toggles.cruise_increase
    v_cruise_delta = v_cruise_delta * v_cruise_delta_interval
    if v_cruise_delta_interval % 5 == 0 and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    v_cruise_offset = (frogpilot_toggles.set_speed_offset * CRUISE_INTERVAL_SIGN[button_type]) if long_press else 0
    if v_cruise_offset < 0:
      v_cruise_offset = frogpilot_toggles.set_speed_offset - v_cruise_delta
    self.v_cruise_kph += v_cruise_offset

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  def _update_v_cruise_low_speed_override(self, CS, enabled, is_metric, speed_limit_changed, frogpilot_toggles, pcm_v_cruise_kph, pcm_v_cruise_cluster_kph):
    """
    Handle low-speed cruise override for Toyota with pedal (gas interceptor).

    When PCM reports set speed at/near its floor (~28 mph), we take over and
    allow setting speeds below that floor.
    """
    v_ego_kph = CS.vEgo * CV.MS_TO_KPH
    at_pcm_floor = pcm_v_cruise_kph <= V_CRUISE_PCM_FLOOR + 5  # ~31 mph tolerance for entry

    # Detect button tap/hold (before updating timers)
    button_type = None
    long_press = False
    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          pass  # end of long press, no tap action
        else:
          button_type = b.type  # Use enum, not raw int
        break
    else:
      for k in self.button_timers.keys():
        if self.button_timers[k] and self.button_timers[k] % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    # Check for entry conditions
    just_entered = False
    if not self.low_speed_override_active and enabled and at_pcm_floor:
      cruise_standstill = self.button_change_states.get(button_type, {}).get("standstill", False) if button_type else False
      cruise_standstill = cruise_standstill or CS.cruiseState.standstill

      # Entry via SET/-: Set to current vehicle speed
      if button_type == ButtonType.decelCruise and not cruise_standstill:
        self.low_speed_override_active = True
        just_entered = True
        self.v_cruise_kph = max(min(v_ego_kph, V_CRUISE_PCM_FLOOR - 1), V_CRUISE_MIN)
        self._last_v_cruise_below_floor = self.v_cruise_kph

      # Entry via RES/+: Resume to previous below-floor speed
      elif (button_type == ButtonType.accelCruise and not cruise_standstill and
            self._last_v_cruise_below_floor != V_CRUISE_UNSET and
            self._last_v_cruise_below_floor < V_CRUISE_PCM_FLOOR):
        self.low_speed_override_active = True
        just_entered = True
        self.v_cruise_kph = self._last_v_cruise_below_floor

    # Handle active override mode
    if self.low_speed_override_active:
      # Exit if cruise disabled
      if not enabled:
        self.low_speed_override_active = False
        self.v_cruise_kph = pcm_v_cruise_kph
        self.v_cruise_cluster_kph = pcm_v_cruise_cluster_kph
        self.update_button_timers(CS, enabled)
        return

      # Handle button presses for speed adjustment (skip on entry frame)
      if button_type is not None and not speed_limit_changed and not just_entered:
        cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
        if not (button_type == ButtonType.accelCruise and cruise_standstill):
          if self.button_change_states[button_type]["enabled"]:
            # Toyota PCM: reverse_cruise_increase=False → 1 mph, True → 5 mph (both tap and hold)
            v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT
            v_cruise_delta_interval = 5 if getattr(frogpilot_toggles, 'reverse_cruise_increase', False) else 1
            v_cruise_delta = v_cruise_delta * v_cruise_delta_interval

            if v_cruise_delta_interval == 5 and self.v_cruise_kph % v_cruise_delta != 0:
              self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
            else:
              self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

            self.v_cruise_kph = clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

      self.update_button_timers(CS, enabled)

      # Check for exit: speed went at or above floor
      if self.v_cruise_kph >= V_CRUISE_PCM_FLOOR:
        self.low_speed_override_active = False
        # Keep our computed value for smooth transition (don't snap to PCM)

      # Store for potential resume
      if self.low_speed_override_active:
        self._last_v_cruise_below_floor = self.v_cruise_kph

      self.v_cruise_cluster_kph = self.v_cruise_kph
    else:
      # Normal PCM cruise behavior
      self.v_cruise_kph = pcm_v_cruise_kph
      self.v_cruise_cluster_kph = pcm_v_cruise_cluster_kph
      self.update_button_timers(CS, enabled)

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool, desired_speed_limit, frogpilot_toggles) -> None:
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode and not frogpilot_toggles.conditional_experimental_mode else V_CRUISE_INITIAL

    # 250kph or above probably means we never had a set speed
    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_kph_last < 250:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      if desired_speed_limit != 0 and frogpilot_toggles.set_speed_limit:
        self.v_cruise_kph = int(round(desired_speed_limit * CV.MS_TO_KPH))
      else:
        self.v_cruise_kph = int(round(clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph


def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error


def apply_center_deadzone(error, deadzone):
  if (error > - deadzone) and (error < deadzone):
    error = 0.
  return error


def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)

def clamp(val, min_val, max_val):
  clamped_val = float(np.clip(val, min_val, max_val))
  return clamped_val, clamped_val != val

def smooth_value(val, prev_val, tau, dt=DT_MDL):
  alpha = 1 - np.exp(-dt/tau) if tau > 0 else 1
  return alpha * val + (1 - alpha) * prev_val

def clip_curvature(v_ego, prev_curvature, new_curvature, roll) -> tuple[float, bool]:
  # This function respects ISO lateral jerk and acceleration limits + a max curvature
  v_ego = max(v_ego, MIN_SPEED)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego ** 2)  # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  new_curvature = np.clip(new_curvature,
                          prev_curvature - max_curvature_rate * DT_CTRL,
                          prev_curvature + max_curvature_rate * DT_CTRL)

  roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
  max_lat_accel = MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  min_lat_accel = -MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  new_curvature, limited_accel = clamp(new_curvature, min_lat_accel / v_ego ** 2, max_lat_accel / v_ego ** 2)

  new_curvature, limited_max_curv = clamp(new_curvature, -MAX_CURVATURE, MAX_CURVATURE)
  return float(new_curvature), limited_accel or limited_max_curv


def get_friction(lateral_accel_error: float, lateral_accel_deadzone: float, friction_threshold: float,
                 torque_params: car.CarParams.LateralTorqueTuning) -> float:
  # TODO torque params' friction should be in lataxel space, not torque space
  friction_interp = interp(
    apply_center_deadzone(lateral_accel_error, lateral_accel_deadzone),
    [-friction_threshold, friction_threshold],
    [-torque_params.friction * torque_params.latAccelFactor, torque_params.friction * torque_params.latAccelFactor]
  )
  return float(friction_interp)


def get_speed_error(modelV2: log.ModelDataV2, v_ego: float) -> float:
  # ToDo: Try relative error, and absolute speed
  if len(modelV2.temporalPose.trans):
    vel_err = clip(modelV2.temporalPose.trans[0] - v_ego, -MAX_VEL_ERR, MAX_VEL_ERR)
    return float(vel_err)
  return 0.0


def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL, vEgoStopping=0.05):
  if len(speeds) == len(t_idxs):
    v_now = speeds[0]
    a_now = accels[0]
    v_target = np.interp(action_t, t_idxs, speeds)
    a_target = 2 * (v_target - v_now) / (action_t) - a_now
    v_target_1sec = np.interp(action_t + 1.0, t_idxs, speeds)
  else:
    v_target = 0.0
    v_target_1sec = 0.0
    a_target = 0.0
  should_stop = (v_target < vEgoStopping and
                 v_target_1sec < vEgoStopping)
  return a_target, should_stop

def curv_from_psis(psi_target, psi_rate, vego, action_t):
  vego = np.clip(vego, MIN_SPEED, np.inf)
  curv_from_psi = psi_target / (vego * action_t)
  return 2*curv_from_psi - psi_rate / vego

def get_curvature_from_plan(yaws, yaw_rates, t_idxs, vego, action_t):
  psi_target = np.interp(action_t, t_idxs, yaws)
  psi_rate = yaw_rates[0]
  return curv_from_psis(psi_target, psi_rate, vego, action_t)

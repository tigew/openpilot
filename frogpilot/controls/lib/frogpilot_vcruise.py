#!/usr/bin/env python3
import math

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import COMFORT_BRAKE

from openpilot.frogpilot.common.frogpilot_variables import CRUISING_SPEED, PLANNER_TIME
from openpilot.frogpilot.controls.lib.curve_speed_controller import CurveSpeedController
from openpilot.frogpilot.controls.lib.speed_limit_controller import SpeedLimitController

BELOW28_FLOOR_MPH = 28

class FrogPilotVCruise:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.csc = CurveSpeedController(self)
    self.slc = SpeedLimitController()

    self.forcing_stop = False
    self.override_force_stop = False

    self.override_force_stop_timer = 0
    self.below28_active = False
    self.below28_target = 0
    self.below28_ui_set_speed_mph = BELOW28_FLOOR_MPH
    self.below28_resume = False
    self.prev_controls_enabled = False

  def _update_below28_assist(self, car_state, controls_state, v_cruise_cluster, speed_limit_changed, just_enabled, frogpilot_toggles):
    if not car_state.cruiseState.enabled or not controls_state.enabled:
      if self.below28_active:
        self.below28_resume = True
      self.below28_active = False
      return

    v_cruise_mph = int(round(v_cruise_cluster * CV.MS_TO_MPH))
    if just_enabled and self.below28_resume and self.below28_ui_set_speed_mph < BELOW28_FLOOR_MPH:
      self.below28_active = True
      self.below28_resume = False
      return

    if just_enabled and v_cruise_mph >= BELOW28_FLOOR_MPH:
      self.below28_active = False
      self.below28_ui_set_speed_mph = BELOW28_FLOOR_MPH
      return

    cancel_pressed = any(be.type == car.CarState.ButtonEvent.Type.cancel and be.pressed for be in car_state.buttonEvents)
    if cancel_pressed:
      self.below28_active = False

    if speed_limit_changed:
      return

    short_step_mph = 5 if frogpilot_toggles.reverse_cruise_increase else 1

    button_type = None
    for event in car_state.buttonEvents:
      if event.type in (car.CarState.ButtonEvent.Type.decelCruise, car.CarState.ButtonEvent.Type.accelCruise) and event.pressed:
        button_type = event.type
        break
      if event.type in (car.CarState.ButtonEvent.Type.decelCruise, car.CarState.ButtonEvent.Type.accelCruise) and not event.pressed:
        button_type = event.type
        break

    if button_type is None:
      return

    can_enter_below28 = button_type == car.CarState.ButtonEvent.Type.decelCruise and (
      v_cruise_mph <= BELOW28_FLOOR_MPH or (v_cruise_mph - short_step_mph) < BELOW28_FLOOR_MPH
    )
    if not self.below28_active and not can_enter_below28:
      return

    if not self.below28_active:
      self.below28_ui_set_speed_mph = v_cruise_mph

    step_mph = short_step_mph
    direction = 1 if button_type == car.CarState.ButtonEvent.Type.accelCruise else -1
    if step_mph % 5 == 0 and self.below28_ui_set_speed_mph % step_mph != 0:
      if direction > 0:
        self.below28_ui_set_speed_mph = int(math.ceil(self.below28_ui_set_speed_mph / step_mph) * step_mph)
      else:
        self.below28_ui_set_speed_mph = int(math.floor(self.below28_ui_set_speed_mph / step_mph) * step_mph)
    else:
      self.below28_ui_set_speed_mph += step_mph * direction

    self.below28_ui_set_speed_mph = max(1, self.below28_ui_set_speed_mph)

    if not self.below28_active and button_type == car.CarState.ButtonEvent.Type.decelCruise and self.below28_ui_set_speed_mph < BELOW28_FLOOR_MPH:
      self.below28_active = True
      self.below28_ui_set_speed_mph = max(1, min(self.below28_ui_set_speed_mph, BELOW28_FLOOR_MPH - 1))

    if self.below28_ui_set_speed_mph >= BELOW28_FLOOR_MPH:
      self.below28_active = False

  def update(self, gps_position, now, time_validated, v_cruise, v_ego, sm, frogpilot_toggles):
    controls_enabled = sm["controlsState"].enabled
    just_enabled = controls_enabled and not self.prev_controls_enabled
    force_stop = self.frogpilot_planner.cem.stop_light_detected and sm["controlsState"].enabled and frogpilot_toggles.force_stops
    force_stop &= self.frogpilot_planner.model_stopped
    force_stop &= self.override_force_stop_timer <= 0

    self.force_stop_timer = self.force_stop_timer + DT_MDL if force_stop else 0

    force_stop_enabled = self.force_stop_timer >= 1

    self.override_force_stop |= sm["carState"].gasPressed
    self.override_force_stop |= sm["frogpilotCarState"].accelPressed
    self.override_force_stop &= force_stop_enabled

    if self.override_force_stop:
      self.override_force_stop_timer = 10
    elif self.override_force_stop_timer > 0:
      self.override_force_stop_timer -= DT_MDL

    v_cruise_cluster = max(sm["controlsState"].vCruiseCluster * CV.KPH_TO_MS, v_cruise)
    v_cruise_diff = v_cruise_cluster - v_cruise

    v_ego_cluster = max(sm["carState"].vEgoCluster, v_ego)
    v_ego_diff = v_ego_cluster - v_ego

    # FrogsGoMoo's Curve Speed Controller
    if v_ego > CRUISING_SPEED and sm["controlsState"].enabled and self.frogpilot_planner.road_curvature_detected and frogpilot_toggles.curve_speed_controller:
      self.csc.update_target(v_ego)

      self.csc_controlling_speed = True

      self.csc_target = self.csc.target
    else:
      self.csc.log_data(v_ego, sm)

      self.csc_controlling_speed = False
      self.csc.target_set = False

      self.csc_target = v_cruise

    # Pfeiferj's Speed Limit Controller
    self.slc.frogpilot_toggles = frogpilot_toggles

    slc_active = frogpilot_toggles.show_speed_limits or frogpilot_toggles.speed_limit_controller
    if frogpilot_toggles.speed_limit_controller:
      self.slc.update_limits(sm["frogpilotCarState"].dashboardSpeedLimit, gps_position, sm["frogpilotNavigation"].navigationSpeedLimit, now, time_validated, v_cruise, v_ego, sm)
      self.slc.update_override(v_cruise, v_cruise_diff, v_ego, v_ego_diff, sm)

      self.slc_offset = self.slc.offset
      self.slc_target = self.slc.target
    elif frogpilot_toggles.show_speed_limits:
      self.slc.update_limits(sm["frogpilotCarState"].dashboardSpeedLimit, gps_position, sm["frogpilotNavigation"].navigationSpeedLimit, now, time_validated, v_cruise, v_ego, sm)

      self.slc_offset = 0
      self.slc_target = self.slc.target
    else:
      self.slc_offset = 0
      self.slc_target = 0

    self._update_below28_assist(
      sm["carState"],
      sm["controlsState"],
      v_cruise_cluster,
      self.slc.speed_limit_changed_timer > DT_MDL if slc_active else False,
      just_enabled,
      frogpilot_toggles,
    )
    if self.below28_active and self.below28_ui_set_speed_mph < BELOW28_FLOOR_MPH:
      self.below28_target = self.below28_ui_set_speed_mph * CV.MPH_TO_MS
    else:
      self.below28_target = 0

    if force_stop_enabled and not self.override_force_stop:
      self.forcing_stop |= not sm["carState"].standstill

      self.tracked_model_length = max(self.tracked_model_length - (v_ego * DT_MDL), 0)
      v_cruise = min((self.tracked_model_length // PLANNER_TIME), v_cruise)

    else:
      self.forcing_stop = False

      self.tracked_model_length = self.frogpilot_planner.model_length

      targets = [self.csc_target, v_cruise]
      if frogpilot_toggles.speed_limit_controller:
        targets.append(max(self.slc.overridden_speed, self.slc_target + self.slc_offset) - v_ego_diff)
      if self.below28_target > 0:
        targets.append(self.below28_target)

      v_cruise = min([target if target >= CRUISING_SPEED else v_cruise for target in targets])

    self.prev_controls_enabled = controls_enabled
    return v_cruise

#!/usr/bin/env python3
from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import COMFORT_BRAKE

from openpilot.frogpilot.common.frogpilot_variables import CRUISING_SPEED, PLANNER_TIME
from openpilot.frogpilot.controls.lib.curve_speed_controller import CurveSpeedController
from openpilot.frogpilot.controls.lib.speed_limit_controller import SpeedLimitController

# Toyota PCM cruise floor (28 mph) - the PCM won't report set speeds below this
SPEED_FLOOR_MPH = 28
SPEED_FLOOR_MS = SPEED_FLOOR_MPH * CV.MPH_TO_MS

class FrogPilotVCruise:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.csc = CurveSpeedController(self)
    self.slc = SpeedLimitController()

    self.forcing_stop = False
    self.override_force_stop = False

    self.override_force_stop_timer = 0

    # Speed floor validation state (below 28 mph)
    # When a speed adjustment would go below the floor, we reroute to a
    # controlled value and enforce it through the speed limit controller.
    self.speed_floor_active = False
    self.speed_floor_speed_mph = SPEED_FLOOR_MPH
    self.speed_floor_target = 0  # in m/s, fed to SLC for enforcement
    self.prev_v_cruise_mph = SPEED_FLOOR_MPH

  def _validate_speed_floor(self, carState, controlsState, v_cruise_cluster_ms, frogpilot_toggles):
    """
    Validate speed adjustments against the 28 mph floor for Toyota with pedal.
    If an adjustment would drop below the floor, reroute to a controlled value
    that gets enforced through the speed limit controller path.
    """
    if not frogpilot_toggles.toyota_low_speed_override:
      self.speed_floor_active = False
      self.speed_floor_target = 0
      return

    # Deactivate if cruise is disabled or cancel pressed
    if not carState.cruiseState.enabled or not controlsState.enabled:
      self.speed_floor_active = False
      self.speed_floor_speed_mph = SPEED_FLOOR_MPH

    cancel_pressed = any(be.type == car.CarState.ButtonEvent.Type.cancel and be.pressed for be in carState.buttonEvents)
    if cancel_pressed:
      self.speed_floor_active = False
      self.speed_floor_speed_mph = SPEED_FLOOR_MPH

    # Detect button presses (Toyota TSS2 sends single-frame pressed=True events)
    decel_pressed = any(be.type == car.CarState.ButtonEvent.Type.decelCruise and be.pressed for be in carState.buttonEvents)
    accel_pressed = any(be.type == car.CarState.ButtonEvent.Type.accelCruise and be.pressed for be in carState.buttonEvents)

    # Skip if speed limit is changing (let SLC handle it)
    if self.slc.speed_limit_changed_timer > DT_MDL:
      return

    v_cruise_mph = int(round(v_cruise_cluster_ms * CV.MS_TO_MPH))
    v_ego_mph = int(round(carState.vEgo * CV.MS_TO_MPH))

    # Increment: 1 mph when reverse_cruise_increase, else 5 mph (Toyota default)
    delta = 1 if frogpilot_toggles.reverse_cruise_increase else 5

    if decel_pressed:
      if self.speed_floor_active:
        # Already below floor - decrease the controlled value
        self.speed_floor_speed_mph = max(1, self.speed_floor_speed_mph - delta)
      elif v_cruise_mph == SPEED_FLOOR_MPH:
        # Adjustment would go below floor - reroute to controlled value
        self.speed_floor_active = True
        if self.prev_v_cruise_mph > SPEED_FLOOR_MPH:
          expected_speed = self.prev_v_cruise_mph - delta
        else:
          expected_speed = SPEED_FLOOR_MPH - delta
        self.speed_floor_speed_mph = max(1, min(v_ego_mph, expected_speed))

    if accel_pressed and self.speed_floor_active:
      self.speed_floor_speed_mph += delta

    # Exit floor control if speed reaches or exceeds floor
    if self.speed_floor_speed_mph >= SPEED_FLOOR_MPH:
      self.speed_floor_active = False
      self.speed_floor_speed_mph = SPEED_FLOOR_MPH

    # Track cruise speed for seamless transition through floor
    self.prev_v_cruise_mph = v_cruise_mph

    # Set the controlled target (in m/s) for SLC enforcement
    if self.speed_floor_active:
      self.speed_floor_target = self.speed_floor_speed_mph * CV.MPH_TO_MS
    else:
      self.speed_floor_target = 0

  def update(self, gps_position, now, time_validated, v_cruise, v_ego, sm, frogpilot_toggles):
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

    # Speed floor validation: check if the speed adjustment would go below 28 mph.
    # If so, reroute to a separate controlled value injected through the SLC enforcement path.
    self._validate_speed_floor(sm["carState"], sm["controlsState"], v_cruise_cluster, frogpilot_toggles)

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
      if self.speed_floor_active and self.speed_floor_target > 0:
        targets.append(self.speed_floor_target - v_ego_diff)

      v_cruise = min([target if target >= CRUISING_SPEED else v_cruise for target in targets])

    return v_cruise

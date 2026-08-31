import math
import numpy as np

from types import SimpleNamespace

from cereal import car
from opendbc.car.gm.values import CC_ONLY_CAR, GMFlags
from openpilot.common.constants import CV


# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

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

# FrogPilot "Low Set Speed": frames to wait after a SET- press ends before checking whether the PCM accepted it
LOW_SET_SPEED_SETTLE_FRAMES = 10
# PCM set speeds above this can't be the PCM's minimum, so a rejected-looking press there is ignored (display units)
LOW_SET_SPEED_MAX_FLOOR = {True: 50, False: 30}
LOW_SET_SPEED_LONG_STEP = 5
# largest gap (km/h) between cruiseState.speedCluster and cruiseState.speed at which the cluster is considered caught up with the PCM
LOW_SET_SPEED_AGREEMENT_KPH = 2.5


class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

    # OPGM variables
    self.gm_cc_only = self.CP.carFingerprint in CC_ONLY_CAR and self.CP.flags & GMFlags.CC_LONG.value

    # FrogPilot variables
    self.low_set_speed_active = False
    self.low_set_speed_target = 0  # display units (mph or km/h), integer
    self.low_set_speed_floor = 0  # cluster set speed (display units) the virtual target sits below
    self.low_set_speed_floor_pcm_kph = 0  # 33 Hz PCM set speed (integer km/h) at activation
    self.low_set_speed_metric = None
    self.low_set_speed_cluster_last = 0
    self.low_set_speed_pcm_kph_last = 0
    self.low_set_speed_decel_frames = 0  # consecutive frames with SET- held
    self.low_set_speed_episode_frames = 0  # length of the last SET- press awaiting evaluation
    self.low_set_speed_episode_cluster = 0  # cluster set speed (display units) before that press
    self.low_set_speed_episode_pcm_kph = 0  # 33 Hz PCM set speed (integer km/h) before that press
    self.low_set_speed_settle_frames = 0

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric, frogpilot_toggles):
    self.v_cruise_kph_last = self.v_cruise_kph

    if CS.cruiseState.available:
      if self.gm_cc_only or not self.CP.pcmCruise:
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self._update_v_cruise_non_pcm(CS, enabled, is_metric, frogpilot_toggles)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self.update_button_timers(CS, enabled)
      else:
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if CS.cruiseState.speed == 0:
          self.v_cruise_kph = V_CRUISE_UNSET
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          self.v_cruise_kph = -1
          self.v_cruise_cluster_kph = -1
        self._update_low_set_speed(CS, is_metric, frogpilot_toggles)
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET
      self._reset_low_set_speed()

  def _reset_low_set_speed(self):
    self.low_set_speed_active = False
    self.low_set_speed_target = 0
    self.low_set_speed_floor = 0
    self.low_set_speed_floor_pcm_kph = 0
    self.low_set_speed_metric = None
    self.low_set_speed_cluster_last = 0
    self.low_set_speed_pcm_kph_last = 0
    self.low_set_speed_decel_frames = 0
    self.low_set_speed_episode_frames = 0
    self.low_set_speed_episode_cluster = 0
    self.low_set_speed_episode_pcm_kph = 0
    self.low_set_speed_settle_frames = 0

  def _evaluate_low_set_speed_episode(self, CS, cluster_display, pcm_kph, is_metric):
    # The PCM didn't move on a SET- press and is already at a plausible minimum: it rejected the press, so step our own target down.
    # Two-signal rule: cruiseState.speedCluster is the cluster display (on Toyota a 1 Hz message that can lag or be stale from the
    # previous engagement) while cruiseState.speed is the PCM's own set speed (33 Hz, integer km/h). A press counts as accepted
    # if EITHER moved, and we only start a virtual target once the two agree, so a lagging cluster can never undercut the PCM.
    episode_frames = self.low_set_speed_episode_frames
    episode_cluster = self.low_set_speed_episode_cluster
    episode_pcm_kph = self.low_set_speed_episode_pcm_kph
    self.low_set_speed_episode_frames = 0
    self.low_set_speed_settle_frames = 0

    if cluster_display != episode_cluster or pcm_kph != episode_pcm_kph or cluster_display > LOW_SET_SPEED_MAX_FLOOR[is_metric]:
      return

    if not self.low_set_speed_active and abs(CS.cruiseState.speedCluster - CS.cruiseState.speed) * CV.MS_TO_KPH > LOW_SET_SPEED_AGREEMENT_KPH:
      return

    step = LOW_SET_SPEED_LONG_STEP if episode_frames >= CRUISE_LONG_PRESS else 1
    base = self.low_set_speed_target if self.low_set_speed_active else cluster_display
    self.low_set_speed_target = max(base - step, self._low_set_speed_min_display(is_metric))
    self.low_set_speed_floor = cluster_display
    self.low_set_speed_floor_pcm_kph = pcm_kph
    self.low_set_speed_active = True

  @staticmethod
  def _low_set_speed_min_display(is_metric):
    return math.ceil(V_CRUISE_MIN * (1.0 if is_metric else CV.KPH_TO_MPH))

  def _update_low_set_speed(self, CS, is_metric, frogpilot_toggles):
    # Virtual set speed below the PCM's minimum, stepped down with the stock SET- button once the PCM stops accepting presses
    if not (self.CP.openpilotLongitudinalControl and frogpilot_toggles.low_set_speed and CS.cruiseState.available
            and CS.cruiseState.enabled and CS.cruiseState.speed > 0 and CS.cruiseState.speedCluster > 0):
      self._reset_low_set_speed()
      return

    if self.low_set_speed_metric is not None and self.low_set_speed_metric != is_metric:
      self._reset_low_set_speed()
    self.low_set_speed_metric = is_metric

    cluster_display = int(round(CS.cruiseState.speedCluster * (CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH)))
    pcm_kph = int(round(CS.cruiseState.speed * CV.MS_TO_KPH))
    decel_now = any(b.type == ButtonType.decelCruise and b.pressed for b in CS.buttonEvents)
    accel_now = any(b.type == ButtonType.accelCruise and b.pressed for b in CS.buttonEvents)

    if self.low_set_speed_active and (accel_now or cluster_display != self.low_set_speed_floor or pcm_kph != self.low_set_speed_floor_pcm_kph):
      # driver pressed RES+ or either PCM set speed signal moved: hand control back to the PCM
      self._reset_low_set_speed()
      self.low_set_speed_metric = is_metric

    if decel_now:
      if self.low_set_speed_decel_frames == 0:
        if self.low_set_speed_episode_frames > 0:
          # a new press started before the previous one settled: judge the previous one now
          self._evaluate_low_set_speed_episode(CS, cluster_display, pcm_kph, is_metric)
        # the PCM may change on the same frame the press is first seen, so use the values from before the press
        self.low_set_speed_episode_cluster = self.low_set_speed_cluster_last if self.low_set_speed_cluster_last > 0 else cluster_display
        self.low_set_speed_episode_pcm_kph = self.low_set_speed_pcm_kph_last if self.low_set_speed_pcm_kph_last > 0 else pcm_kph
      self.low_set_speed_decel_frames += 1
    else:
      if self.low_set_speed_decel_frames > 0:
        self.low_set_speed_episode_frames = self.low_set_speed_decel_frames
        self.low_set_speed_settle_frames = LOW_SET_SPEED_SETTLE_FRAMES
        self.low_set_speed_decel_frames = 0
      elif self.low_set_speed_settle_frames > 0:
        self.low_set_speed_settle_frames -= 1
        if self.low_set_speed_settle_frames == 0:
          self._evaluate_low_set_speed_episode(CS, cluster_display, pcm_kph, is_metric)

    self.low_set_speed_cluster_last = cluster_display
    self.low_set_speed_pcm_kph_last = pcm_kph

    if self.low_set_speed_active:
      # never above either PCM signal, never below the lowest speed openpilot will hold
      self.low_set_speed_target = max(min(self.low_set_speed_target, cluster_display), self._low_set_speed_min_display(is_metric))
      v_cruise_kph = min(self.low_set_speed_target * (1.0 if is_metric else CV.MPH_TO_KPH), CS.cruiseState.speed * CV.MS_TO_KPH)
      self.v_cruise_kph = v_cruise_kph
      self.v_cruise_cluster_kph = v_cruise_kph

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric, frogpilot_toggles):
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
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
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

    # FrogPilot variables
    if long_press and frogpilot_toggles.set_speed_offset > 0:
      self.v_cruise_kph += frogpilot_toggles.set_speed_offset
      if button_type == ButtonType.decelCruise:
        self.v_cruise_kph -= v_cruise_delta

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

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

  def initialize_v_cruise(self, CS, experimental_mode: bool, resume_prev_button: bool, frogpilot_toggles: SimpleNamespace, slc_target: float = 0) -> None:
    # initializing is handled by the PCM
    if self.CP.pcmCruise and not self.gm_cc_only:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode and not frogpilot_toggles.conditional_experimental_mode else V_CRUISE_INITIAL

    if (any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents)
      and self.v_cruise_initialized or (self.gm_cc_only and resume_prev_button)):
      self.v_cruise_kph = self.v_cruise_kph_last
    elif frogpilot_toggles.set_speed_limit and slc_target > 0:
      self.v_cruise_kph = int(round(np.clip(slc_target * CV.MS_TO_KPH, V_CRUISE_MIN, V_CRUISE_MAX)))
    else:
      self.v_cruise_kph = int(round(np.clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph

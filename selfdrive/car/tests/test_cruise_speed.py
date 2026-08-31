import math
import pytest
import itertools
import numpy as np

from parameterized import parameterized_class
from types import SimpleNamespace
from cereal import log
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_MIN, V_CRUISE_MAX, V_CRUISE_INITIAL, V_CRUISE_UNSET, IMPERIAL_INCREMENT
from openpilot.selfdrive.car.cruise import CRUISE_LONG_PRESS, LOW_SET_SPEED_SETTLE_FRAMES
from cereal import car
from openpilot.common.constants import CV
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


def run_cruise_simulation(cruise, e2e, personality, t_end=20.):
  man = Maneuver(
    '',
    duration=t_end,
    initial_speed=max(cruise - 1., 0.0),
    lead_relevancy=True,
    initial_distance_lead=100,
    cruise_values=[cruise],
    prob_lead_values=[0.0],
    breakpoints=[0.],
    e2e=e2e,
    personality=personality,
  )
  valid, output = man.evaluate()
  assert valid
  return output[-1, 3]


@parameterized_class(("e2e", "personality", "speed"), itertools.product(
                      [True, False], # e2e
                      log.LongitudinalPersonality.schema.enumerants, # personality
                      [5,35])) # speed
class TestCruiseSpeed:
  def test_cruise_speed(self):
    print(f'Testing {self.speed} m/s')
    cruise_speed = float(self.speed)

    simulation_steady_state = run_cruise_simulation(cruise_speed, self.e2e, self.personality)
    assert simulation_steady_state == pytest.approx(cruise_speed, abs=.01), f'Did not reach {self.speed} m/s'


# TODO: test pcmCruise
@parameterized_class(('pcm_cruise',), [(False,)])
class TestVCruiseHelper:
  def setup_method(self):
    self.CP = car.CarParams(pcmCruise=self.pcm_cruise)
    self.v_cruise_helper = VCruiseHelper(self.CP)
    self.reset_cruise_speed_state()

  def reset_cruise_speed_state(self):
    # Two resets previous cruise speed
    for _ in range(2):
      self.v_cruise_helper.update_v_cruise(car.CarState(cruiseState={"available": False}), enabled=False, is_metric=False)

  def enable(self, v_ego, experimental_mode):
    # Simulates user pressing set with a current speed
    self.v_cruise_helper.initialize_v_cruise(car.CarState(vEgo=v_ego), experimental_mode, False)

  def test_adjust_speed(self):
    """
    Asserts speed changes on falling edges of buttons.
    """

    self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False)

    for btn in (ButtonType.accelCruise, ButtonType.decelCruise):
      for pressed in (True, False):
        CS = car.CarState(cruiseState={"available": True})
        CS.buttonEvents = [ButtonEvent(type=btn, pressed=pressed)]

        self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)
        assert pressed == (self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last)

  def test_rising_edge_enable(self):
    """
    Some car interfaces may enable on rising edge of a button,
    ensure we don't adjust speed if enabled changes mid-press.
    """

    # NOTE: enabled is always one frame behind the result from button press in controlsd
    for enabled, pressed in ((False, False),
                             (False, True),
                             (True, False)):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=pressed)]
      self.v_cruise_helper.update_v_cruise(CS, enabled=enabled, is_metric=False)
      if pressed:
        self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False)

      # Expected diff on enabling. Speed should not change on falling edge of pressed
      assert not pressed == self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last

  def test_resume_in_standstill(self):
    """
    Asserts we don't increment set speed if user presses resume/accel to exit cruise standstill.
    """

    self.enable(0, False)

    for standstill in (True, False):
      for pressed in (True, False):
        CS = car.CarState(cruiseState={"available": True, "standstill": standstill})
        CS.buttonEvents = [ButtonEvent(type=ButtonType.accelCruise, pressed=pressed)]
        self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)

        # speed should only update if not at standstill and button falling edge
        should_equal = standstill or pressed
        assert should_equal == (self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last)

  def test_set_gas_pressed(self):
    """
    Asserts pressing set while enabled with gas pressed sets
    the speed to the maximum of vEgo and current cruise speed.
    """

    for v_ego in np.linspace(0, 100, 101):
      self.reset_cruise_speed_state()
      self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False)

      # first decrement speed, then perform gas pressed logic
      expected_v_cruise_kph = self.v_cruise_helper.v_cruise_kph - IMPERIAL_INCREMENT
      expected_v_cruise_kph = max(expected_v_cruise_kph, v_ego * CV.MS_TO_KPH)  # clip to min of vEgo
      expected_v_cruise_kph = float(np.clip(round(expected_v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX))

      CS = car.CarState(vEgo=float(v_ego), gasPressed=True, cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=False)]
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)

      # TODO: fix skipping first run due to enabled on rising edge exception
      if v_ego == 0.0:
        continue
      assert expected_v_cruise_kph == self.v_cruise_helper.v_cruise_kph

  def test_initialize_v_cruise(self):
    """
    Asserts allowed cruise speeds on enabling with SET.
    """

    for experimental_mode in (True, False):
      for v_ego in np.linspace(0, 100, 101):
        self.reset_cruise_speed_state()
        assert not self.v_cruise_helper.v_cruise_initialized

        self.enable(float(v_ego), experimental_mode)
        assert V_CRUISE_INITIAL <= self.v_cruise_helper.v_cruise_kph <= V_CRUISE_MAX
        assert self.v_cruise_helper.v_cruise_initialized


class TestVCruiseHelperLowSetSpeed:
  """
  FrogPilot "Low Set Speed": a virtual set speed below the PCM's minimum on pcmCruise cars.
  """
  SETTLE = LOW_SET_SPEED_SETTLE_FRAMES + 5

  def setup_method(self):
    self.is_metric = False
    self.CP = car.CarParams(pcmCruise=True, openpilotLongitudinalControl=True)
    self.v_cruise_helper = VCruiseHelper(self.CP)
    self.frogpilot_toggles = SimpleNamespace(low_set_speed=True, reverse_cruise_increase=False, cruise_increase=1, cruise_increase_long=5, set_speed_offset=0)

  def display_to_ms(self, speed):
    return speed * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)

  def display_to_kph(self, speed):
    return speed * (1.0 if self.is_metric else CV.MPH_TO_KPH)

  def step_ms(self, speed_ms, cluster_ms, buttons=(), enabled=True, available=True, gas_pressed=False, v_ego_ms=0.0):
    # cruiseState.speed is the PCM's own set speed (33 Hz on Toyota), cruiseState.speedCluster the cluster display (1 Hz on Toyota)
    CS = car.CarState(cruiseState={"available": available, "enabled": enabled, "speed": speed_ms, "speedCluster": cluster_ms},
                      gasPressed=gas_pressed, vEgo=v_ego_ms)
    CS.buttonEvents = [ButtonEvent(type=btn, pressed=True) for btn in buttons]
    self.v_cruise_helper.update_v_cruise(CS, enabled=enabled, is_metric=self.is_metric, frogpilot_toggles=self.frogpilot_toggles)
    return CS

  def step(self, pcm_speed, buttons=(), enabled=True, available=True, cluster_speed=None, gas_pressed=False, v_ego=None):
    cluster_speed = pcm_speed if cluster_speed is None else cluster_speed
    v_ego = pcm_speed if v_ego is None else v_ego
    return self.step_ms(self.display_to_ms(pcm_speed), self.display_to_ms(cluster_speed), buttons=buttons, enabled=enabled, available=available,
                        gas_pressed=gas_pressed, v_ego_ms=self.display_to_ms(v_ego))

  def press_decel(self, pcm_speeds, cluster_speeds=None, gas_pressed=False, v_ego=None):
    # Toyota emits a pressed decelCruise event on every frame SET- is held and never a release event
    cluster_speeds = pcm_speeds if cluster_speeds is None else cluster_speeds
    for pcm_speed, cluster_speed in zip(pcm_speeds, cluster_speeds, strict=True):
      self.step(pcm_speed, buttons=(ButtonType.decelCruise,), cluster_speed=cluster_speed, gas_pressed=gas_pressed, v_ego=v_ego)

  def settle(self, pcm_speed, frames=None, cluster_speed=None):
    for _ in range(self.SETTLE if frames is None else frames):
      self.step(pcm_speed, cluster_speed=cluster_speed)

  def assert_virtual(self, target):
    assert self.v_cruise_helper.low_set_speed_active
    assert self.v_cruise_helper.v_cruise_kph == pytest.approx(self.display_to_kph(target))
    assert self.v_cruise_helper.v_cruise_cluster_kph == self.v_cruise_helper.v_cruise_kph

  def assert_pcm(self, pcm_speed, cluster_speed=None):
    # stock behaviour: v_cruise follows the PCM set speed, v_cruise_cluster the cluster display
    cluster_speed = pcm_speed if cluster_speed is None else cluster_speed
    assert not self.v_cruise_helper.low_set_speed_active
    assert self.v_cruise_helper.v_cruise_kph == pytest.approx(self.display_to_kph(pcm_speed))
    assert self.v_cruise_helper.v_cruise_cluster_kph == pytest.approx(self.display_to_kph(cluster_speed))

  def test_rejected_press_steps_down(self):
    self.settle(28)
    self.assert_pcm(28)

    # 20-frame SET- press that the PCM ignores (it's at its floor)
    self.press_decel([28] * 20)
    # nothing happens until the PCM has had time to answer
    assert not self.v_cruise_helper.low_set_speed_active
    self.settle(28)
    self.assert_virtual(27)

    # second press steps from the virtual target, not from the PCM value
    self.press_decel([28] * 20)
    self.settle(28)
    self.assert_virtual(26)

  def test_long_press_steps_five(self):
    self.settle(28)
    # a hold is judged mid-press, after CRUISE_LONG_PRESS frames, and snaps to the next multiple of 5 like the PCM does
    self.press_decel([28] * (CRUISE_LONG_PRESS - 1))
    assert not self.v_cruise_helper.low_set_speed_active
    self.press_decel([28])
    self.assert_virtual(25)

    # releasing after a hold does not add a tap step
    self.settle(28)
    self.assert_virtual(25)

    # a following short press still steps by one
    self.press_decel([28] * 5)
    self.settle(28)
    self.assert_virtual(24)

    # from a multiple of 5 a hold steps a full 5
    self.press_decel([28] * CRUISE_LONG_PRESS)
    self.settle(28)
    self.assert_virtual(20)

  def test_hold_repeats_while_held(self):
    self.settle(28)
    self.press_decel([28] * CRUISE_LONG_PRESS)
    self.assert_virtual(25)
    self.press_decel([28] * CRUISE_LONG_PRESS)
    self.assert_virtual(20)
    self.press_decel([28] * CRUISE_LONG_PRESS)
    self.assert_virtual(15)
    self.settle(28)
    self.assert_virtual(15)

  def test_hold_from_above_the_floor_is_left_to_the_pcm(self):
    # the PCM applies the tap step on the press itself, so a hold from 30 changes its set speed before we would judge it
    self.settle(30)
    self.press_decel([29] * (2 * CRUISE_LONG_PRESS))
    assert not self.v_cruise_helper.low_set_speed_active
    self.settle(29)
    self.assert_pcm(29)

  def test_reverse_cruise_increase_swaps_tap_and_hold(self):
    self.frogpilot_toggles.reverse_cruise_increase = True
    self.settle(28)
    # a tap now moves 5 (snapping to the next multiple of 5)
    self.press_decel([28] * 20)
    self.settle(28)
    self.assert_virtual(25)
    self.press_decel([28] * 20)
    self.settle(28)
    self.assert_virtual(20)

    # and a hold moves 1 per CRUISE_LONG_PRESS frames
    self.press_decel([28] * CRUISE_LONG_PRESS)
    self.assert_virtual(19)
    self.settle(28)
    self.assert_virtual(19)

  def test_gas_pressed_never_sets_below_current_speed(self):
    self.settle(28)
    self.press_decel([28] * 20)
    self.settle(28)
    self.assert_virtual(27)

    # foot on the gas at ~27 mph: SET- keeps the target at the current speed instead of 26
    self.press_decel([28] * 20, gas_pressed=True, v_ego=26.7)
    self.settle(28)
    self.assert_virtual(27)

    # slower than the would-be target: the normal step applies
    self.press_decel([28] * 20, gas_pressed=True, v_ego=24.2)
    self.settle(28)
    self.assert_virtual(26)

  def test_gas_pressed_above_the_floor_does_not_activate(self):
    self.settle(28)
    self.press_decel([28] * 20, gas_pressed=True, v_ego=32)
    self.settle(28)
    self.assert_pcm(28)

  def test_clipped_to_v_cruise_min(self):
    self.settle(28)
    for _ in range(10):
      self.press_decel([28] * CRUISE_LONG_PRESS)
      self.settle(28)
    self.assert_virtual(math.ceil(V_CRUISE_MIN * CV.KPH_TO_MPH))

  def test_accepted_press_does_nothing(self):
    self.settle(35)
    # PCM decrements a few frames into the press: normal PCM behaviour, no virtual target
    self.press_decel([35] * 3 + [34] * 17)
    self.settle(34)
    self.assert_pcm(34)

    # PCM decrements on the same frame the press is first seen
    self.press_decel([33] * 5)
    self.settle(33)
    self.assert_pcm(33)

    # PCM decrements only after the press ended
    self.press_decel([33] * 5)
    self.settle(33, frames=3)
    self.settle(32)
    self.assert_pcm(32)

  def test_rejected_press_above_sanity_bound_ignored(self):
    self.settle(45)
    self.press_decel([45] * 20)
    self.settle(45)
    self.assert_pcm(45)

  def test_rapid_presses_before_settle(self):
    self.settle(28)
    for _ in range(3):
      self.press_decel([28] * 5)
      self.settle(28, frames=2)
    self.settle(28)
    self.assert_virtual(25)

  def test_accel_press_hands_back_to_pcm(self):
    self.settle(28)
    self.press_decel([28] * 20)
    self.settle(28)
    self.assert_virtual(26 + 1)

    self.step(28, buttons=(ButtonType.accelCruise,))
    self.assert_pcm(28)
    self.settle(29)
    self.assert_pcm(29)

  def test_pcm_rising_hands_back_to_pcm(self):
    self.settle(28)
    self.press_decel([28] * 20)
    self.settle(28)
    self.assert_virtual(27)

    self.step(29)
    self.assert_pcm(29)
    self.settle(29)
    self.assert_pcm(29)

  def test_pcm_dropping_hands_back_to_pcm(self):
    self.settle(30)
    self.press_decel([30] * 20)
    self.settle(30)
    self.assert_virtual(29)

    self.step(28)
    self.assert_pcm(28)

  def test_disengage_clears(self):
    self.settle(28)
    self.press_decel([28] * 20)
    self.settle(28)
    self.assert_virtual(27)

    self.step(28, enabled=False)
    assert not self.v_cruise_helper.low_set_speed_active
    assert self.v_cruise_helper.v_cruise_kph == pytest.approx(28 * CV.MPH_TO_KPH)

    # re-enabling starts from the PCM value again; a pending press from before the disengage is forgotten
    self.settle(28)
    self.assert_pcm(28)

  def test_cruise_off_clears(self):
    self.settle(28)
    self.press_decel([28] * 20)
    self.settle(28)
    self.assert_virtual(27)

    self.step(0)
    assert not self.v_cruise_helper.low_set_speed_active
    assert self.v_cruise_helper.v_cruise_kph == V_CRUISE_UNSET

    self.step(28, available=False)
    assert not self.v_cruise_helper.low_set_speed_active
    assert self.v_cruise_helper.v_cruise_kph == V_CRUISE_UNSET

  def test_toggle_off_matches_stock(self):
    self.frogpilot_toggles.low_set_speed = False
    self.settle(28)
    self.press_decel([28] * CRUISE_LONG_PRESS)
    self.settle(28)
    self.assert_pcm(28)
    assert self.v_cruise_helper.low_set_speed_decel_frames == 0
    assert self.v_cruise_helper.low_set_speed_settle_frames == 0

  def test_no_openpilot_long_matches_stock(self):
    self.CP = car.CarParams(pcmCruise=True, openpilotLongitudinalControl=False)
    self.v_cruise_helper = VCruiseHelper(self.CP)
    self.settle(28)
    self.press_decel([28] * 20)
    self.settle(28)
    self.assert_pcm(28)

  def test_metric(self):
    self.is_metric = True
    self.settle(30)
    self.press_decel([30] * 20)
    self.settle(30)
    self.assert_virtual(29)
    assert self.v_cruise_helper.v_cruise_kph == pytest.approx(29.0)

    self.press_decel([30] * CRUISE_LONG_PRESS)
    self.settle(30)
    self.assert_virtual(25)

    # PCM floors above the metric sanity bound are ignored
    self.step(31)
    self.assert_pcm(31)
    self.settle(55)
    self.press_decel([55] * 20)
    self.settle(55)
    self.assert_pcm(55)

  def test_unit_change_clears(self):
    self.settle(28)
    self.press_decel([28] * 20)
    self.settle(28)
    self.assert_virtual(27)

    self.is_metric = True
    self.step(45)
    self.assert_pcm(45)

  def test_stale_cluster_at_engage(self):
    # PCM set speed is already 35 on engage while the 1 Hz cluster value still shows 28 from the previous engagement
    self.settle(0, cluster_speed=0)
    self.press_decel([35] * 5, cluster_speeds=[28] * 5)
    self.settle(35, cluster_speed=28)
    self.assert_pcm(35, cluster_speed=28)

    # even a long rejected-looking press with the two signals apart must not activate
    self.press_decel([35] * CRUISE_LONG_PRESS, cluster_speeds=[28] * CRUISE_LONG_PRESS)
    self.settle(35, cluster_speed=28)
    self.assert_pcm(35, cluster_speed=28)

  def test_long_press_with_lagging_cluster(self):
    # at 30 a hold steps the PCM to 29 straight away while the cluster keeps showing 30 for another second
    self.settle(30)
    self.press_decel([29] * (CRUISE_LONG_PRESS + 5), cluster_speeds=[30] * (CRUISE_LONG_PRESS + 5))
    self.settle(29, cluster_speed=30)
    self.assert_pcm(29, cluster_speed=30)
    self.settle(29, frames=100, cluster_speed=30)
    self.assert_pcm(29, cluster_speed=30)

    # once the cluster has caught up, a genuinely rejected press steps below the PCM
    self.settle(29)
    self.press_decel([29] * 20)
    self.settle(29)
    self.assert_virtual(28)

  def test_speed_cluster_zero_is_inert(self):
    # PCM set speed valid but the cluster value not seen yet: the feature stays off and v_cruise follows the PCM
    for _ in range(5):
      self.step_ms(self.display_to_ms(28), 0)
    for _ in range(20):
      self.step_ms(self.display_to_ms(28), 0, buttons=(ButtonType.decelCruise,))
    for _ in range(self.SETTLE):
      self.step_ms(self.display_to_ms(28), 0)
    assert not self.v_cruise_helper.low_set_speed_active
    assert self.v_cruise_helper.low_set_speed_decel_frames == 0
    assert self.v_cruise_helper.low_set_speed_settle_frames == 0
    assert self.v_cruise_helper.v_cruise_kph == pytest.approx(28 * CV.MPH_TO_KPH)

  def test_metric_car_imperial_openpilot_accepted_step(self):
    # car set speed in km/h (47 -> 46 accepted) with openpilot in mph: both round to 29 mph, but the km/h value moved
    for _ in range(5):
      self.step_ms(47 * CV.KPH_TO_MS, 47 * CV.KPH_TO_MS)
    for _ in range(5):
      self.step_ms(46 * CV.KPH_TO_MS, 46 * CV.KPH_TO_MS, buttons=(ButtonType.decelCruise,))
    for _ in range(self.SETTLE):
      self.step_ms(46 * CV.KPH_TO_MS, 46 * CV.KPH_TO_MS)
    assert not self.v_cruise_helper.low_set_speed_active
    assert self.v_cruise_helper.v_cruise_kph == pytest.approx(46.0)

  def test_integer_kph_pcm_value_at_floor(self):
    # imperial car: the PCM reports an integer km/h set speed (45 for 28 mph) while the cluster shows 28 mph
    pcm_ms = 45 * CV.KPH_TO_MS
    cluster_ms = self.display_to_ms(28)
    for _ in range(5):
      self.step_ms(pcm_ms, cluster_ms)
    assert self.v_cruise_helper.v_cruise_kph == pytest.approx(45.0)

    for target in (27, 26):
      for _ in range(20):
        self.step_ms(pcm_ms, cluster_ms, buttons=(ButtonType.decelCruise,))
      for _ in range(self.SETTLE):
        self.step_ms(pcm_ms, cluster_ms)
      self.assert_virtual(target)

    self.step_ms(pcm_ms, cluster_ms, buttons=(ButtonType.accelCruise,))
    assert not self.v_cruise_helper.low_set_speed_active
    assert self.v_cruise_helper.v_cruise_kph == pytest.approx(45.0)
    assert self.v_cruise_helper.v_cruise_cluster_kph == pytest.approx(28 * CV.MPH_TO_KPH)

  def test_pcm_kph_below_target_clamps(self):
    # imperial car with the PCM reporting 43 km/h for 28 mph: the virtual 27 mph (43.45 km/h) is capped at the PCM value
    pcm_ms = 43 * CV.KPH_TO_MS
    cluster_ms = self.display_to_ms(28)
    for _ in range(5):
      self.step_ms(pcm_ms, cluster_ms)
    for _ in range(20):
      self.step_ms(pcm_ms, cluster_ms, buttons=(ButtonType.decelCruise,))
    for _ in range(self.SETTLE):
      self.step_ms(pcm_ms, cluster_ms)
    assert self.v_cruise_helper.low_set_speed_active
    assert self.v_cruise_helper.low_set_speed_target == 27
    assert self.v_cruise_helper.v_cruise_kph == pytest.approx(43.0)
    assert self.v_cruise_helper.v_cruise_cluster_kph == self.v_cruise_helper.v_cruise_kph

    for _ in range(20):
      self.step_ms(pcm_ms, cluster_ms, buttons=(ButtonType.decelCruise,))
    for _ in range(self.SETTLE):
      self.step_ms(pcm_ms, cluster_ms)
    self.assert_virtual(26)

  def test_pcm_kph_moving_hands_back_to_pcm(self):
    # the 33 Hz PCM value moves while the 1 Hz cluster value hasn't caught up yet
    self.settle(28)
    self.press_decel([28] * 20)
    self.settle(28)
    self.assert_virtual(27)

    self.step(29, cluster_speed=28)
    assert not self.v_cruise_helper.low_set_speed_active
    assert self.v_cruise_helper.v_cruise_kph == pytest.approx(29 * CV.MPH_TO_KPH)
    self.settle(29)
    self.assert_pcm(29)

"""FrogPilot Toyota tests: smartDSU / DSU bypass overrides and comma pedal (gas interceptor) support."""
from types import SimpleNamespace

import pytest

# opendbc.car.interfaces imports openpilot.common.params, whose cython extension only exists after a scons build
pytest.importorskip("openpilot.common.params_pyx", reason="openpilot.common.params_pyx is not built")

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, crc8_pedal, gen_empty_fingerprint, structs
from opendbc.car.car_helpers import PEDAL_MSG, apply_frogpilot_car_overrides
from opendbc.car.toyota.carcontroller import CarController, MAX_INTERCEPTOR_GAS
from opendbc.car.toyota.carstate import CarState, GAS_INTERCEPTOR_THRESHOLD
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, DBC, MIN_ACC_SPEED, ToyotaFrogPilotFlags, ToyotaSafetyFlags

LongCtrlState = structs.CarControl.Actuators.LongControlState

SDSU_MSG = 0x2FF
GAS_COMMAND_MSG = 0x200

STOCK_LONG = ToyotaSafetyFlags.STOCK_LONGITUDINAL.value
GAS_INTERCEPTOR = ToyotaSafetyFlags.GAS_INTERCEPTOR.value


def make_toggles(**overrides) -> SimpleNamespace:
  toggles = {
    # read by CarInterfaceBase.get_params / get_frogpilot_params / apply_frogpilot_car_overrides
    "force_torque_controller": False,
    "nnff": False,
    "nnff_lite": False,
    "toyota_dsu_bypass": False,
    "disable_openpilot_long": False,
    # read by CarController.update
    "sng_hack": False,
    "frogsgomoo_tweak": False,
    "reverse_cruise_increase": False,
    "lock_doors": False,
    "unlock_doors": False,
  }
  toggles.update(overrides)
  return SimpleNamespace(**toggles)


def build_car_params(addrs: tuple[int, ...] = (), toggles: SimpleNamespace | None = None, candidate: str = CAR.TOYOTA_COROLLA):
  toggles = toggles or make_toggles()
  fingerprint = gen_empty_fingerprint()
  fingerprint[0].update(dict.fromkeys(addrs, 8))

  CP = CarInterface.get_params(candidate, fingerprint, [], alpha_long=False, is_release=False, docs=False, frogpilot_toggles=toggles)
  FPCP = CarInterface.get_frogpilot_params(candidate, fingerprint, [], CP, toggles)
  apply_frogpilot_car_overrides(CP, FPCP, fingerprint, toggles)
  return CP, FPCP


def safety_params(CP, FPCP) -> tuple[int, int]:
  return CP.safetyConfigs[0].safetyParam, FPCP.safetyConfigs[0].safetyParam


class TestToyotaFrogPilotCarParams:
  def test_stock_corolla_untouched(self):
    CP, FPCP = build_car_params()
    assert not CP.openpilotLongitudinalControl
    assert not CP.enableGasInterceptorDEPRECATED
    assert CP.minEnableSpeed == pytest.approx(MIN_ACC_SPEED)
    assert not FPCP.flags & (ToyotaFrogPilotFlags.SMART_DSU.value | ToyotaFrogPilotFlags.DSU_BYPASS.value)
    for param in safety_params(CP, FPCP):
      assert param & STOCK_LONG
      assert not param & GAS_INTERCEPTOR

  def test_pedal_without_long_control_is_ignored(self):
    # a pedal on the bus is only used while openpilot controls longitudinal
    CP, FPCP = build_car_params((PEDAL_MSG,))
    assert not CP.openpilotLongitudinalControl
    assert not CP.enableGasInterceptorDEPRECATED
    for param in safety_params(CP, FPCP):
      assert param & STOCK_LONG
      assert not param & GAS_INTERCEPTOR

  def test_sdsu_enables_long_control(self):
    CP, FPCP = build_car_params((SDSU_MSG,))
    assert FPCP.flags & ToyotaFrogPilotFlags.SMART_DSU.value
    assert CP.openpilotLongitudinalControl
    assert CP.minEnableSpeed == -1
    assert not CP.enableGasInterceptorDEPRECATED
    for param in safety_params(CP, FPCP):
      assert not param & STOCK_LONG
      assert not param & GAS_INTERCEPTOR

  def test_sdsu_with_pedal(self):
    CP, FPCP = build_car_params((SDSU_MSG, PEDAL_MSG))
    assert CP.openpilotLongitudinalControl
    assert CP.enableGasInterceptorDEPRECATED
    assert CP.minEnableSpeed == -1
    for param in safety_params(CP, FPCP):
      assert not param & STOCK_LONG
      assert param & GAS_INTERCEPTOR

  def test_dsu_bypass_with_pedal(self):
    CP, FPCP = build_car_params((PEDAL_MSG,), make_toggles(toyota_dsu_bypass=True))
    assert FPCP.flags & ToyotaFrogPilotFlags.DSU_BYPASS.value
    assert not FPCP.flags & ToyotaFrogPilotFlags.SMART_DSU.value
    assert CP.openpilotLongitudinalControl
    assert CP.enableGasInterceptorDEPRECATED
    assert CP.minEnableSpeed == -1
    for param in safety_params(CP, FPCP):
      assert not param & STOCK_LONG
      assert param & GAS_INTERCEPTOR

  def test_dsu_bypass_without_pedal_keeps_min_enable_speed(self):
    CP, FPCP = build_car_params((), make_toggles(toyota_dsu_bypass=True))
    assert CP.openpilotLongitudinalControl
    assert not CP.enableGasInterceptorDEPRECATED
    assert CP.minEnableSpeed == pytest.approx(MIN_ACC_SPEED)
    for param in safety_params(CP, FPCP):
      assert not param & STOCK_LONG

  def test_disable_openpilot_long_restores_stock(self):
    CP, FPCP = build_car_params((SDSU_MSG, PEDAL_MSG), make_toggles(disable_openpilot_long=True))
    assert not CP.openpilotLongitudinalControl
    assert FPCP.openpilotLongitudinalControlDisabled
    assert not CP.enableGasInterceptorDEPRECATED
    for param in safety_params(CP, FPCP):
      assert param & STOCK_LONG
      assert not param & GAS_INTERCEPTOR

  @pytest.mark.parametrize("addrs, toggles", [
    ((), make_toggles()),
    ((SDSU_MSG,), make_toggles()),
    ((SDSU_MSG, PEDAL_MSG), make_toggles()),
    ((PEDAL_MSG,), make_toggles(toyota_dsu_bypass=True)),
    ((SDSU_MSG, PEDAL_MSG), make_toggles(disable_openpilot_long=True)),
  ])
  def test_safety_params_stay_in_sync(self, addrs, toggles):
    # pandad ORs CP and FPCP params together and selfdrived compares the panda's param against FPCP's
    CP, FPCP = build_car_params(addrs, toggles)
    cp_param, fpcp_param = safety_params(CP, FPCP)
    assert cp_param == fpcp_param


class TestToyotaFrogPilotCarParamsOtherPlatforms:
  def test_tss2_ignores_pedal(self):
    # factory stop-and-go cars never drive a leftover pedal (matches the "Pedal Support: No" the UI shows for them)
    CP, FPCP = build_car_params((PEDAL_MSG,), candidate=CAR.TOYOTA_COROLLA_TSS2)
    assert CP.openpilotLongitudinalControl
    assert not FPCP.canUsePedal
    assert not CP.enableGasInterceptorDEPRECATED
    for param in safety_params(CP, FPCP):
      assert not param & GAS_INTERCEPTOR

  def test_secoc_ignores_pedal(self):
    CP, FPCP = build_car_params((PEDAL_MSG,), candidate=CAR.TOYOTA_RAV4_PRIME)
    assert not CP.enableGasInterceptorDEPRECATED
    for param in safety_params(CP, FPCP):
      assert not param & GAS_INTERCEPTOR

  def test_tss2_disable_openpilot_long_sets_stock_longitudinal(self):
    # the interface enabled openpilot long for TSS2, so the toggle must hand ACC_CONTROL back to the camera in the panda too
    CP, FPCP = build_car_params((), make_toggles(disable_openpilot_long=True), candidate=CAR.TOYOTA_COROLLA_TSS2)
    assert not CP.openpilotLongitudinalControl
    assert FPCP.openpilotLongitudinalControlDisabled
    for param in safety_params(CP, FPCP):
      assert param & STOCK_LONG


class TestToyotaGasInterceptorCarState:
  def test_gas_sensor_parser_registration(self):
    CP, _ = build_car_params((SDSU_MSG, PEDAL_MSG))
    assert PEDAL_MSG in CarState.get_can_parsers(CP)[Bus.pt].addresses

    CP, _ = build_car_params((SDSU_MSG,))
    assert PEDAL_MSG not in CarState.get_can_parsers(CP)[Bus.pt].addresses

  def test_gas_pressed_threshold_matches_panda(self):
    # opendbc/safety/modes/toyota.h TOYOTA_GAS_INTERCEPTOR_THRESHOLD
    assert GAS_INTERCEPTOR_THRESHOLD == 805

  @pytest.mark.parametrize("gas_counts, expected_pressed", [(0, False), (700, False), (805, False), (806, True), (1200, True)])
  def test_gas_comes_from_interceptor(self, gas_counts, expected_pressed):
    CP, FPCP = build_car_params((SDSU_MSG, PEDAL_MSG))
    CS = CarState(CP, FPCP)
    parsers = CarState.get_can_parsers(CP)
    packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])

    # the PCM reports the pedal as pressed (openpilot's own virtual press through the interceptor), which must be ignored
    frames = [
      packer.make_can_msg("GAS_SENSOR", 0, {"INTERCEPTOR_GAS": gas_counts, "INTERCEPTOR_GAS2": gas_counts}),
      packer.make_can_msg("PCM_CRUISE", 0, {"GAS_RELEASED": 0}),
    ]
    parsers[Bus.pt].update([(1, frames)])
    ret, _ = CS.update(parsers, make_toggles(cluster_offset=1.0))

    assert ret.gasPressed == expected_pressed

  def test_gas_pressed_from_pcm_without_interceptor(self):
    CP, FPCP = build_car_params((SDSU_MSG,))
    CS = CarState(CP, FPCP)
    parsers = CarState.get_can_parsers(CP)
    packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])

    for gas_released, expected_pressed in ((0, True), (1, False)):
      parsers[Bus.pt].update([(1, [packer.make_can_msg("PCM_CRUISE", 0, {"GAS_RELEASED": gas_released})])])
      ret, _ = CS.update(parsers, make_toggles(cluster_offset=1.0))
      assert ret.gasPressed == expected_pressed


def decode_gas_command(dat: bytes) -> dict[str, float]:
  parser = CANParser(DBC[CAR.TOYOTA_COROLLA][Bus.pt], [("GAS_COMMAND", 50)], 0)
  parser.update([(1, [(GAS_COMMAND_MSG, dat, 0)])])
  return dict(parser.vl["GAS_COMMAND"])


class TestToyotaGasInterceptorCarController:
  def setup_method(self):
    self.toggles = make_toggles()
    self.CP, self.FPCP = build_car_params((SDSU_MSG, PEDAL_MSG))
    assert self.CP.enableGasInterceptorDEPRECATED

  def make_controller(self, CP=None, FPCP=None):
    CP = CP or self.CP
    FPCP = FPCP or self.FPCP
    CS = CarState(CP, FPCP)
    CS.out.vEgo = 0.
    CS.out.vEgoRaw = 0.
    CS.out.standstill = True
    CS.out.cruiseState.enabled = True
    CS.pcm_acc_status = 8
    dbc_names = {Bus.pt: DBC[CP.carFingerprint][Bus.pt], Bus.cam: DBC[CP.carFingerprint][Bus.pt]}
    return CarController(dbc_names, CP), CS

  @staticmethod
  def make_car_control(long_active: bool, accel: float) -> structs.CarControl:
    CC = structs.CarControl()
    CC.enabled = long_active
    CC.latActive = False
    CC.longActive = long_active
    CC.actuators.accel = accel
    CC.actuators.longControlState = LongCtrlState.pid if long_active else LongCtrlState.off
    return CC.as_reader()

  @staticmethod
  def gas_commands(can_sends) -> list[bytes]:
    return [dat for addr, dat, bus in can_sends if addr == GAS_COMMAND_MSG and bus == 0]

  def test_active_sends_gas(self):
    controller, CS = self.make_controller()
    new_actuators, can_sends = controller.update(self.make_car_control(True, 1.0), CS, 0, self.toggles)

    cmds = self.gas_commands(can_sends)
    assert len(cmds) == 1
    dat = cmds[0]
    assert len(dat) == 6
    assert crc8_pedal(dat[:-1]) == dat[-1]

    values = decode_gas_command(dat)
    assert values["ENABLE"] == 1
    assert values["COUNTER_PEDAL"] == 0

    # Corolla at standstill: PEDAL_SCALE 0.3, offset -0.4 -> 0.3 * (1.0 - 0.4)
    expected_gas = 0.3 * (1.0 - 0.4)
    assert 0 < expected_gas < MAX_INTERCEPTOR_GAS
    assert new_actuators.gas == pytest.approx(expected_gas)
    assert values["GAS_COMMAND"] == pytest.approx(expected_gas * 255., abs=0.2)
    assert values["GAS_COMMAND2"] == pytest.approx(expected_gas * 255., abs=0.2)

    # pedal cars send the standstill request on entering a stop even without a stop timer
    assert controller.standstill_req

  def test_active_gas_is_clipped(self):
    controller, CS = self.make_controller()
    new_actuators, can_sends = controller.update(self.make_car_control(True, 10.0), CS, 0, self.toggles)
    assert new_actuators.gas == pytest.approx(MAX_INTERCEPTOR_GAS)
    values = decode_gas_command(self.gas_commands(can_sends)[0])
    assert values["GAS_COMMAND"] == pytest.approx(MAX_INTERCEPTOR_GAS * 255., abs=0.2)

  def test_inactive_sends_exact_zero(self):
    controller, CS = self.make_controller()
    new_actuators, can_sends = controller.update(self.make_car_control(False, 1.0), CS, 0, self.toggles)

    cmds = self.gas_commands(can_sends)
    assert len(cmds) == 1
    dat = cmds[0]
    assert crc8_pedal(dat[:-1]) == dat[-1]
    values = decode_gas_command(dat)
    assert values["ENABLE"] == 0
    # raw gas fields must be exactly zero so the interceptor does not rescale the pedal range
    assert dat[:4] == b"\x00\x00\x00\x00"
    assert new_actuators.gas == 0

  def test_braking_sends_zero_gas(self):
    controller, CS = self.make_controller()
    new_actuators, can_sends = controller.update(self.make_car_control(True, -1.0), CS, 0, self.toggles)
    dat = self.gas_commands(can_sends)[0]
    assert decode_gas_command(dat)["ENABLE"] == 0
    assert dat[:4] == b"\x00\x00\x00\x00"
    assert new_actuators.gas == 0

  def test_sent_every_other_frame_with_counter(self):
    controller, CS = self.make_controller()
    CC = self.make_car_control(True, 1.0)
    counters = []
    for frame in range(8):
      _, can_sends = controller.update(CC, CS, 0, self.toggles)
      cmds = self.gas_commands(can_sends)
      assert len(cmds) == (1 if frame % 2 == 0 else 0)
      if cmds:
        counters.append(decode_gas_command(cmds[0])["COUNTER_PEDAL"])
    assert counters == [0, 1, 2, 3]

  def test_no_gas_command_without_pedal(self):
    CP, FPCP = build_car_params((SDSU_MSG,))
    assert not CP.enableGasInterceptorDEPRECATED
    controller, CS = self.make_controller(CP, FPCP)
    for _ in range(4):
      new_actuators, can_sends = controller.update(self.make_car_control(True, 1.0), CS, 0, self.toggles)
      assert len(self.gas_commands(can_sends)) == 0
      assert new_actuators.gas == 0

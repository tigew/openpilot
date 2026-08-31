"""FrogPilot Toyota tests: smartDSU / DSU bypass longitudinal overrides."""
from types import SimpleNamespace

import pytest

# opendbc.car.interfaces imports openpilot.common.params, whose cython extension only exists after a scons build
pytest.importorskip("openpilot.common.params_pyx", reason="openpilot.common.params_pyx is not built")

from opendbc.car import gen_empty_fingerprint
from opendbc.car.car_helpers import apply_frogpilot_car_overrides
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, MIN_ACC_SPEED, ToyotaFrogPilotFlags, ToyotaSafetyFlags

SDSU_MSG = 0x2FF

STOCK_LONG = ToyotaSafetyFlags.STOCK_LONGITUDINAL.value


def make_toggles(**overrides) -> SimpleNamespace:
  toggles = {
    # read by CarInterfaceBase.get_params / get_frogpilot_params / apply_frogpilot_car_overrides
    "force_torque_controller": False,
    "nnff": False,
    "nnff_lite": False,
    "toyota_dsu_bypass": False,
    "disable_openpilot_long": False,
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
    assert CP.minEnableSpeed == pytest.approx(MIN_ACC_SPEED)
    assert not FPCP.flags & (ToyotaFrogPilotFlags.SMART_DSU.value | ToyotaFrogPilotFlags.DSU_BYPASS.value)
    for param in safety_params(CP, FPCP):
      assert param & STOCK_LONG

  def test_sdsu_enables_long_control(self):
    CP, FPCP = build_car_params((SDSU_MSG,))
    assert FPCP.flags & ToyotaFrogPilotFlags.SMART_DSU.value
    assert CP.openpilotLongitudinalControl
    assert CP.minEnableSpeed == -1
    for param in safety_params(CP, FPCP):
      assert not param & STOCK_LONG

  def test_dsu_bypass_enables_long_control(self):
    CP, FPCP = build_car_params((), make_toggles(toyota_dsu_bypass=True))
    assert FPCP.flags & ToyotaFrogPilotFlags.DSU_BYPASS.value
    assert not FPCP.flags & ToyotaFrogPilotFlags.SMART_DSU.value
    assert CP.openpilotLongitudinalControl
    assert CP.minEnableSpeed == pytest.approx(MIN_ACC_SPEED)
    for param in safety_params(CP, FPCP):
      assert not param & STOCK_LONG

  def test_disable_openpilot_long_restores_stock(self):
    CP, FPCP = build_car_params((SDSU_MSG,), make_toggles(disable_openpilot_long=True))
    assert not CP.openpilotLongitudinalControl
    assert FPCP.openpilotLongitudinalControlDisabled
    for param in safety_params(CP, FPCP):
      assert param & STOCK_LONG

  def test_tss2_disable_openpilot_long_sets_stock_longitudinal(self):
    # the interface enabled openpilot long for TSS2, so the toggle must hand ACC_CONTROL back to the camera in the panda too
    CP, FPCP = build_car_params((), make_toggles(disable_openpilot_long=True), candidate=CAR.TOYOTA_COROLLA_TSS2)
    assert not CP.openpilotLongitudinalControl
    assert FPCP.openpilotLongitudinalControlDisabled
    for param in safety_params(CP, FPCP):
      assert param & STOCK_LONG

  @pytest.mark.parametrize("addrs, toggles", [
    ((), make_toggles()),
    ((SDSU_MSG,), make_toggles()),
    ((), make_toggles(toyota_dsu_bypass=True)),
    ((SDSU_MSG,), make_toggles(disable_openpilot_long=True)),
  ])
  def test_safety_params_stay_in_sync(self, addrs, toggles):
    # pandad ORs CP and FPCP params together and selfdrived compares the panda's param against FPCP's
    CP, FPCP = build_car_params(addrs, toggles)
    cp_param, fpcp_param = safety_params(CP, FPCP)
    assert cp_param == fpcp_param

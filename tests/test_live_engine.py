import pytest
from conveyor_control.live_engine import LiveSimulationEngine


def test_live_engine_initialization_and_stepping():
    engine = LiveSimulationEngine(dt=0.01)
    assert not engine.is_running
    assert engine.target_speed == 1.8
    assert len(engine.packages) == 3

    sample = engine.step()
    assert sample["time_s"] == 0.01
    assert "actual_rpm" in sample
    assert "command" in sample
    assert "controller" in sample
    assert sample["controller"] == "pid"


def test_live_engine_controller_switching():
    engine = LiveSimulationEngine(dt=0.01)
    engine.set_controller("bang-bang")
    assert engine.active_controller_name == "bang-bang"

    sample = engine.step()
    assert sample["controller"] == "bang-bang"

    engine.set_controller("pwm")
    assert engine.active_controller_name == "pwm"
    sample2 = engine.step()
    assert sample2["controller"] == "pwm"


def test_live_engine_disturbance_and_load():
    engine = LiveSimulationEngine(dt=0.01)
    initial_load = engine.total_load_torque
    engine.set_manual_load(0.12)
    assert engine.total_load_torque > initial_load

    res = engine.inject_disturbance("heavy_box")
    assert res["type"] == "warning"
    assert len(engine.packages) == 4


def test_live_engine_telemetry_and_diagnostics():
    engine = LiveSimulationEngine(dt=0.01)
    for _ in range(10):
        engine.step()
    telem = engine.get_telemetry()
    assert "actual_speed_m_s" in telem
    assert "diagnostics" in telem
    assert isinstance(telem["diagnostics"], list)
    assert len(telem["packages"]) >= 3

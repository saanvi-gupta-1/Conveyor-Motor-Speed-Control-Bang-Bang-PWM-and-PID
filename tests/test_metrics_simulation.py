import math
import pytest

from conveyor_control.controllers import PIDController
from conveyor_control.metrics import calculate_metrics
from conveyor_control.simulation import SimulationConfig, run_simulation


def test_metrics_on_known_samples():
    rows = [{"time_s": 0., "error_m_s": 1., "target_m_s": 1., "speed_m_s": 0., "command": .5, "saturated": 0.},
            {"time_s": 1., "error_m_s": 0., "target_m_s": 1., "speed_m_s": 1., "command": .5, "saturated": 1.}]
    m = calculate_metrics(rows)
    assert m["IAE_m_s_s"] == pytest.approx(1)
    assert m["RMSE_m_s"] == pytest.approx(math.sqrt(.5))
    assert m["saturation_time_s"] == pytest.approx(0)


def test_zero_target_and_reproducibility():
    config = SimulationConfig(duration=.2, target_speed=0)
    a = run_simulation(PIDController(), config=config)
    b = run_simulation(PIDController(), config=config)
    assert a == b
    assert calculate_metrics(a)["overshoot_percent"] == 0
    metrics = calculate_metrics(a)
    assert metrics["peak_speed_m_s"] >= 0
    assert metrics["maximum_absolute_error_m_s"] == 0


def test_bad_scenario_and_settings():
    with pytest.raises(ValueError):
        run_simulation(PIDController(), "unknown")
    with pytest.raises(ValueError):
        run_simulation(PIDController(), config=SimulationConfig(dt=0))
    with pytest.raises(ValueError):
        run_simulation(PIDController(), config=SimulationConfig(pwm_frequency_hz=100))
    with pytest.raises(ValueError):
        run_simulation(PIDController(), config=SimulationConfig(load_torque=.4))


def test_load_drop_and_recovery_metrics_on_known_trace():
    rows = []
    errors = [0, 0, .10, .08, .01, .005, .005, .005]
    for i, error in enumerate(errors):
        rows.append({"time_s": i * .1, "error_m_s": error, "target_m_s": 1.0,
                     "speed_m_s": 1.0 - error, "command": .5, "duty": .5,
                     "saturated": 0., "load_torque_Nm": .0 if i < 2 else .1})
    metrics = calculate_metrics(rows)
    assert metrics["speed_drop_on_load_m_s"] == pytest.approx(.1)
    assert metrics["recovery_time_after_load_s"] == pytest.approx(.2)


def test_load_schedule_and_pwm_frequency_change_are_logged():
    from conveyor_control.controllers import PWMController
    config = SimulationConfig(duration=2, dt=.005)
    light = run_simulation(PIDController(), "light_load", config)
    heavy = run_simulation(PIDController(), "heavy_load", config)
    assert heavy[-1]["speed_m_s"] < light[-1]["speed_m_s"]
    cycle = run_simulation(PIDController(), "load_cycle", config)
    assert min(row["load_torque_Nm"] for row in cycle) == 0
    assert max(row["load_torque_Nm"] for row in cycle) > cycle[-1]["load_torque_Nm"]
    slow = run_simulation(PWMController(frequency_hz=10), config=config)
    fast = run_simulation(PWMController(frequency_hz=40), config=config)
    assert slow[10]["pwm_frequency_hz"] == 10
    assert fast[10]["pwm_frequency_hz"] == 40
    assert [row["pwm_on"] for row in slow] != [row["pwm_on"] for row in fast]


def test_pid_terms_and_bangbang_switch_count_are_logged():
    from conveyor_control.controllers import BangBangController
    pid_rows = run_simulation(PIDController(), config=SimulationConfig(duration=.1))
    for row in pid_rows:
        assert row["raw_command"] == pytest.approx(row["p_term"] + row["i_term"] + row["d_term"])
        assert "pid_integral_state" in row
    relay_rows = run_simulation(BangBangController(), "changing_reference", SimulationConfig(duration=1))
    assert relay_rows[-1]["switch_count"] > 0

import pytest

from conveyor_control.stability import analyze_stability


def test_default_plant_and_pid_linearization_are_stable():
    result = analyze_stability()
    assert result.plant_stable
    assert result.plant_pole_rad_s == pytest.approx(-.04 / .06)
    assert result.pid_closed_loop_stable
    assert all(p.real < 0 for p in result.closed_loop_poles_rad_s)


def test_invalid_negative_gains_rejected():
    with pytest.raises(ValueError):
        analyze_stability(kp=-1)

import pytest

from conveyor_control.controllers import BangBangController, PWMController, PIDController


def test_bang_bang_threshold_and_switching():
    c = BangBangController(deadband=.1, min_switch_interval=0)
    assert c.update(1, 0, .01, 0).command == 1
    assert c.update(0, 1, .01, .1).command == 0
    assert c.switch_count == 2


def test_pwm_bounds_and_actual_carrier():
    c = PWMController(frequency_hz=10)
    a = c.update(1, 0, .01, .01)
    b = c.update(1, .8, .01, .08)
    assert 0 <= a.duty <= 1 and 0 <= b.duty <= 1
    assert a.switched_on is True and b.switched_on is False
    assert c.update(1, 1, .01, .31).duty > 0
    saturated = c.update(10, 0, .01, .4)
    assert saturated.duty == 1 and saturated.saturated
    assert c.frequency_hz == 10


def test_pid_saturates_and_anti_windup():
    c = PIDController(kp=5, ki=2, kd=0)
    outputs = [c.update(10, 0, .01, i * .01) for i in range(100)]
    assert all(0 <= o.command <= 1 for o in outputs)
    assert abs(c.integral) < .02
    recovering = c.update(0, 1, .01, 1.1)
    assert 0 <= recovering.command <= 1


def test_invalid_pwm():
    with pytest.raises(ValueError):
        PWMController(frequency_hz=0)


def test_pwm_marks_actuator_limit_clipping_as_saturation():
    controller = PWMController(frequency_hz=10, limits=(.2, .8))
    output = controller.update(1, 0, .01, .01)
    assert output.command == pytest.approx(.8)
    assert output.duty == 1
    assert output.saturated is True

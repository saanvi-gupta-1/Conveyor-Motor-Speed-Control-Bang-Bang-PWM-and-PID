import pytest

from conveyor_control.model import DCMotor, MotorParameters


def test_increasing_input_increases_speed():
    motor = DCMotor()
    for _ in range(200):
        motor.step(.8, 0, .005)
    first = motor.omega
    for _ in range(200):
        motor.step(1, 0, .005)
    assert motor.omega > first


def test_load_reduces_speed():
    motor = DCMotor()
    for _ in range(1000):
        motor.step(.7, 0, .005)
    before = motor.omega
    for _ in range(300):
        motor.step(.7, .2, .005)
    assert motor.omega < before


def test_speed_constraints_and_zero():
    motor = DCMotor(MotorParameters(max_speed=8))
    for _ in range(2000):
        motor.step(1, 0, .005)
    assert motor.omega <= 8
    motor.step(0, 10, .005)
    assert motor.omega >= 0


def test_invalid_dt_rejected():
    with pytest.raises(ValueError):
        DCMotor().step(0, 0, 0)

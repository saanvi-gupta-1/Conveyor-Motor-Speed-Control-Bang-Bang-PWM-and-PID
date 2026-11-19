"""Common actuator controllers. Outputs use normalized [0, 1] actuator units."""

from dataclasses import dataclass
import math


@dataclass
class ControlOutput:
    raw: float
    command: float
    saturated: bool
    duty: float = 0.0
    switched_on: bool = False
    p_term: float = 0.0
    i_term: float = 0.0
    d_term: float = 0.0
    switch_count: int = 0
    integral_state: float = 0.0


class Controller:
    def reset(self) -> None:
        pass

    def update(self, target: float, measured: float, dt: float, time: float) -> ControlOutput:
        raise NotImplementedError


def _clamp(value: float, limits: tuple[float, float]) -> float:
    return min(limits[1], max(limits[0], value))


class BangBangController(Controller):
    def __init__(self, deadband: float = 0.12, limits: tuple[float, float] = (0.0, 1.0), min_switch_interval: float = 0.04) -> None:
        if deadband < 0 or limits[0] >= limits[1]:
            raise ValueError("Invalid deadband or actuator limits")
        self.deadband, self.limits, self.min_switch_interval = deadband, limits, min_switch_interval
        self.state = False
        self.last_switch = -math.inf
        self.switch_count = 0

    def reset(self) -> None:
        self.state, self.last_switch, self.switch_count = False, -math.inf, 0

    def update(self, target: float, measured: float, dt: float, time: float) -> ControlOutput:
        error = target - measured
        desired = True if error > self.deadband else False if error < -self.deadband else self.state
        if desired != self.state and time - self.last_switch >= self.min_switch_interval:
            self.state, self.last_switch = desired, time
            self.switch_count += 1
        raw = self.limits[1] if self.state else self.limits[0]
        return ControlOutput(raw, _clamp(raw, self.limits), False, float(self.state), self.state,
                             switch_count=self.switch_count)


class PWMController(Controller):
    """Proportional error-to-duty controller with an explicit carrier waveform."""

    def __init__(self, frequency_hz: float = 20.0, proportional_gain: float = 0.55,
                 feedforward_gain: float = 0.454545, limits: tuple[float, float] = (0.0, 1.0)) -> None:
        if frequency_hz <= 0 or proportional_gain < 0 or feedforward_gain < 0 or limits[0] >= limits[1]:
            raise ValueError("Invalid PWM parameters")
        self.frequency_hz, self.proportional_gain, self.feedforward_gain, self.limits = frequency_hz, proportional_gain, feedforward_gain, limits

    def update(self, target: float, measured: float, dt: float, time: float) -> ControlOutput:
        # Nominal plant feedforward supplies the steady-state torque; proportional
        # correction adjusts duty for tracking error without integral action.
        raw_duty = self.feedforward_gain * target + self.proportional_gain * (target - measured)
        duty = min(1.0, max(0.0, raw_duty))
        switched = (time * self.frequency_hz) % 1.0 < duty
        raw_command = self.limits[1] if switched else self.limits[0]
        command = _clamp(raw_command, self.limits)
        saturated = raw_duty != duty or raw_command != command
        return ControlOutput(raw_duty, command, saturated, duty, switched)


class PIDController(Controller):
    """Discrete PID with conditional integration anti-windup and derivative filtering."""

    def __init__(self, kp: float = 0.7, ki: float = 0.8, kd: float = 0.01,
                 limits: tuple[float, float] = (0.0, 1.0), derivative_tau: float = 0.025) -> None:
        if limits[0] >= limits[1] or min(kp, ki, kd, derivative_tau) < 0:
            raise ValueError("Invalid PID parameters")
        self.kp, self.ki, self.kd, self.limits, self.derivative_tau = kp, ki, kd, limits, derivative_tau
        self.reset()

    def reset(self) -> None:
        self.integral, self.previous_error, self.derivative = 0.0, None, 0.0

    def update(self, target: float, measured: float, dt: float, time: float) -> ControlOutput:
        if dt <= 0:
            raise ValueError("dt must be positive")
        error = target - measured
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        alpha = dt / (self.derivative_tau + dt) if self.derivative_tau else 1.0
        self.derivative += alpha * (derivative - self.derivative)
        proposed_integral = self.integral + error * dt
        raw = self.kp * error + self.ki * proposed_integral + self.kd * self.derivative
        command = _clamp(raw, self.limits)
        # Integrate only when unsaturated or when error drives the output back from saturation.
        if raw == command or (raw > self.limits[1] and error < 0) or (raw < self.limits[0] and error > 0):
            self.integral = proposed_integral
            raw = self.kp * error + self.ki * self.integral + self.kd * self.derivative
            command = _clamp(raw, self.limits)
        self.previous_error = error
        p_term = self.kp * error
        d_term = self.kd * self.derivative
        return ControlOutput(raw, command, command != raw, command,
                             p_term=p_term, i_term=raw - p_term - d_term,
                             d_term=d_term, integral_state=self.integral)

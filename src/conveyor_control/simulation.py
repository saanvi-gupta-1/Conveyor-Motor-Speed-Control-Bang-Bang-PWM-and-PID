"""Deterministic closed-loop simulation and experiment definitions."""

from dataclasses import dataclass, field
import random

from .controllers import Controller, ControlOutput
from .model import DCMotor, MotorParameters


@dataclass
class SimulationConfig:
    dt: float = 0.005
    duration: float = 12.0
    target_speed: float = 1.8  # m/s
    sensor_noise_std: float = 0.0
    seed: int = 7
    load_torque: float = 0.16
    pwm_frequency_hz: float = 20.0
    actuator_min: float = 0.0
    actuator_max: float = 1.0
    bangbang_deadband: float = .12
    motor: MotorParameters = field(default_factory=MotorParameters)

    def validate(self) -> None:
        if self.dt <= 0 or self.duration <= 0 or self.target_speed < 0 or self.sensor_noise_std < 0:
            raise ValueError("Invalid simulation settings")
        if not 0 <= self.load_torque <= self.motor.max_load_torque:
            raise ValueError("load_torque must fit within the configured motor load limit")
        if self.pwm_frequency_hz <= 0 or self.pwm_frequency_hz >= .5 / self.dt:
            raise ValueError("PWM frequency must be positive and below the sampled Nyquist frequency")
        if not 0 <= self.actuator_min < self.actuator_max <= 1:
            raise ValueError("Actuator limits must satisfy 0 <= min < max <= 1")
        if self.bangbang_deadband < 0:
            raise ValueError("Bang-Bang deadband cannot be negative")


def run_simulation(controller: Controller, scenario: str = "baseline", config: SimulationConfig | None = None) -> list[dict[str, float]]:
    cfg = config or SimulationConfig()
    cfg.validate()
    motor = DCMotor(cfg.motor)
    controller.reset()
    rng = random.Random(cfg.seed)
    rows: list[dict[str, float]] = []
    n = int(round(cfg.duration / cfg.dt))
    for i in range(n + 1):
        t = i * cfg.dt
        target = cfg.target_speed
        load = 0.0
        if scenario == "load_step":
            if t >= cfg.duration * 0.4:
                load = cfg.load_torque
        elif scenario == "load_cycle":
            load = 0.0 if t < cfg.duration * .35 else cfg.motor.max_load_torque * .8 if t < cfg.duration * .70 else cfg.motor.max_load_torque * .15
        elif scenario == "light_load":
            load = cfg.motor.max_load_torque * .17
        elif scenario == "medium_load":
            load = cfg.motor.max_load_torque * .53
        elif scenario == "heavy_load":
            load = cfg.motor.max_load_torque * .90
        elif scenario == "changing_reference":
            target = cfg.target_speed * (0.55 if t < cfg.duration * 0.35 else 1.0 if t < cfg.duration * 0.7 else 0.7)
        elif scenario == "saturation":
            target = cfg.motor.roller_radius * cfg.motor.max_speed * 1.15
        elif scenario != "baseline":
            raise ValueError(f"Unknown scenario: {scenario}")
        measured = max(0.0, motor.belt_speed + rng.gauss(0.0, cfg.sensor_noise_std))
        out: ControlOutput = controller.update(target, measured, cfg.dt, t)
        rows.append({"time_s": t, "controller": type(controller).__name__,
                     "target_m_s": target, "measured_speed_m_s": measured, "speed_m_s": motor.belt_speed,
                     "omega_rad_s": motor.omega, "error_m_s": target - motor.belt_speed,
                     "load_torque_Nm": load, "raw_command": out.raw, "command": out.command,
                     "saturated": float(out.saturated), "duty": out.duty,
                     "pwm_on": float(out.switched_on), "switch_count": float(out.switch_count),
                     "p_term": out.p_term, "i_term": out.i_term, "d_term": out.d_term,
                     "pid_integral_state": out.integral_state,
                     "pwm_frequency_hz": float(getattr(controller, "frequency_hz", 0.0)),
                     "scenario": scenario})
        if i < n:
            motor.step(out.command, load, cfg.dt)
    return rows

"""Reduced-order DC motor model using SI units and fixed-step integration."""

from dataclasses import dataclass


@dataclass
class MotorParameters:
    """Parameters for J*dω/dt = K*u - b*ω - τ_load."""

    inertia: float = 0.06  # kg m²
    torque_constant: float = 1.10  # N m / normalized input
    viscous_friction: float = 0.04  # N m s / rad
    roller_radius: float = 0.08  # m
    max_speed: float = 45.0  # rad/s
    max_load_torque: float = 0.30  # N m

    def validate(self) -> None:
        if min(self.inertia, self.torque_constant, self.roller_radius, self.max_speed) <= 0:
            raise ValueError("Inertia, torque constant, roller radius and max speed must be positive")
        if min(self.viscous_friction, self.max_load_torque) < 0:
            raise ValueError("Friction and load torque cannot be negative")


class DCMotor:
    """Forward-Euler model with torque disturbances and configurable speed limits."""

    def __init__(self, parameters: MotorParameters | None = None) -> None:
        self.parameters = parameters or MotorParameters()
        self.parameters.validate()
        self.omega = 0.0

    @property
    def belt_speed(self) -> float:
        return self.parameters.roller_radius * self.omega

    def reset(self, omega: float = 0.0) -> None:
        self.omega = min(self.parameters.max_speed, max(0.0, float(omega)))

    def step(self, command: float, load_torque: float, dt: float) -> float:
        if dt <= 0:
            raise ValueError("dt must be positive")
        p = self.parameters
        command = min(1.0, max(0.0, float(command)))
        load_torque = min(p.max_load_torque, max(0.0, float(load_torque)))
        acceleration = (p.torque_constant * command - p.viscous_friction * self.omega - load_torque) / p.inertia
        self.omega = min(p.max_speed, max(0.0, self.omega + acceleration * dt))
        return self.omega

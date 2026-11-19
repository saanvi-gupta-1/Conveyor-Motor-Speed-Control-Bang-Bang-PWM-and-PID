"""Linear stability checks for the reduced plant and ideal unsaturated PID loop."""

from dataclasses import dataclass
import math
from .model import MotorParameters


@dataclass(frozen=True)
class StabilityResult:
    plant_pole_rad_s: float
    closed_loop_poles_rad_s: tuple[complex, complex]
    plant_stable: bool
    pid_closed_loop_stable: bool
    assumptions: str = "Continuous ideal PID linearization; saturation, PWM switching and derivative filter omitted."


def analyze_stability(parameters: MotorParameters | None = None, kp: float = 0.7,
                      ki: float = 0.8, kd: float = 0.01) -> StabilityResult:
    """Check pole locations for G(s)=K/(Js+b) with an ideal PID controller."""
    p = parameters or MotorParameters()
    p.validate()
    if min(kp, ki, kd) < 0:
        raise ValueError("PID gains cannot be negative")
    plant_pole = -p.viscous_friction / p.inertia
    # Characteristic equation: (J+K*Kd)s² + (b+K*Kp)s + K*Ki = 0.
    a = p.inertia + p.torque_constant * kd
    b = p.viscous_friction + p.torque_constant * kp
    c = p.torque_constant * ki
    discriminant = b * b - 4 * a * c
    root = math.sqrt(abs(discriminant))
    if discriminant >= 0:
        poles = (complex((-b + root) / (2 * a), 0), complex((-b - root) / (2 * a), 0))
    else:
        poles = (complex(-b / (2 * a), root / (2 * a)), complex(-b / (2 * a), -root / (2 * a)))
    return StabilityResult(plant_pole, poles, plant_pole < 0, all(pole.real < 0 for pole in poles))

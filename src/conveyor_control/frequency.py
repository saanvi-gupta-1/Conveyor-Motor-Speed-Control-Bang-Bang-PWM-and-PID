"""Analytical frequency response of the linearized reduced-order motor plant."""

import math
import cmath
from .model import MotorParameters


def motor_frequency_response(parameters: MotorParameters | None = None, points: int = 160,
                             pid_gains: tuple[float, float, float] = (0.7, 0.8, 0.01)) -> list[dict[str, float]]:
    """Return plant and ideal unsaturated PID closed-loop Bode samples."""
    p = parameters or MotorParameters()
    p.validate()
    if points < 2:
        raise ValueError("At least two frequency samples are required")
    samples = []
    for i in range(points):
        omega = 10.0 ** (-2.0 + 5.0 * i / (points - 1))
        denominator = math.hypot(p.viscous_friction, p.inertia * omega)
        magnitude = p.torque_constant / denominator
        phase = -math.degrees(math.atan2(p.inertia * omega, p.viscous_friction))
        s = 1j * omega
        kp, ki, kd = pid_gains
        controller = kp + ki / s + kd * s
        plant = p.torque_constant / (p.inertia * s + p.viscous_friction)
        closed_loop = plant * controller / (1 + plant * controller)
        samples.append({"frequency_rad_s": omega,
                        "magnitude_rad_s_per_unit": magnitude,
                        "magnitude_db": 20.0 * math.log10(magnitude),
                        "phase_deg": phase,
                        "pid_closed_loop_magnitude_db": 20.0 * math.log10(abs(closed_loop)),
                        "pid_closed_loop_phase_deg": math.degrees(cmath.phase(closed_loop))})
    return samples

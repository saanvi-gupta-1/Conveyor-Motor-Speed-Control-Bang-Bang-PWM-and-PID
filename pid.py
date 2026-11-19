"""
controllers/pid.py
==============================================================================
Proportional-Integral-Derivative (PID) Controller with Anti-Windup.

Three Parallel Control Actions:
  1. Proportional (P): Reacts to CURRENT error  --> P = Kp * e
  2. Integral (I):     Reacts to PAST errors     --> I = Ki * integral(e * dt)
  3. Derivative (D):   Predicts FUTURE trend     --> D = Kd * de/dt (filtered)

Advanced Industrial Features:
  - Conditional Integrator Clamping (Anti-Windup):
    Prevents the integral term from growing uncontrollably when the voltage
    hits its physical saturation ceiling (±24V).
  - First-Order Derivative Low-Pass Filter (N-coefficient):
    Prevents high-frequency measurement derivative spikes from causing chatter.

Characteristics:
  Guarantees EXACT zero steady-state error even under heavy payload drag,
  damps oscillations, and provides fast, smooth disturbance rejection.
==============================================================================
"""

import numpy as np


class PIDController:
    name = "PID"

    def __init__(
        self,
        Kp: float = 55.0,
        Ki: float = 45.0,
        Kd: float = 0.5,
        N: float = 20.0,
        V_max: float = 24.0,
    ):
        """
        Parameters:
            Kp: Proportional gain.
            Ki: Integral gain (eliminates steady-state error).
            Kd: Derivative gain (damps oscillations and overshoot).
            N:  Derivative filter bandwidth limiter coefficient.
            V_max: Actuator saturation limit [Volts].
        """
        self.Kp = float(Kp)
        self.Ki = float(Ki)
        self.Kd = float(Kd)
        self.N = float(N)
        self.V_max = float(V_max)
        self.reset()

    def reset(self):
        """Reset internal accumulator states."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._filtered_deriv = 0.0
        self._first_step = True

    def set_gains(self, Kp=None, Ki=None, Kd=None):
        """Dynamically tune controller gains without wiping integrator."""
        if Kp is not None:
            self.Kp = float(Kp)
        if Ki is not None:
            self.Ki = float(Ki)
        if Kd is not None:
            self.Kd = float(Kd)

    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        """
        Compute control voltage using filtered PID law and anti-windup clamping.
        """
        error = setpoint - measurement

        if self._first_step:
            self._prev_error = error
            self._first_step = False

        # 1. Proportional Term
        P = self.Kp * error

        # 2. Derivative Term with 1st-Order Low-Pass Filter (Tustin / Euler)
        raw_deriv = (error - self._prev_error) / dt if dt > 1e-9 else 0.0
        alpha = self.N * dt
        self._filtered_deriv = (1.0 - alpha) * self._filtered_deriv + alpha * raw_deriv
        D = self.Kd * self._filtered_deriv

        # 3. Integral Candidate
        candidate_integral = self._integral + error * dt
        I = self.Ki * candidate_integral

        # Unconstrained controller demand
        raw_voltage = P + I + D

        # Physical actuator saturation (clipping to [0.0V, +V_max] for unidirectional conveyor)
        saturated_voltage = float(np.clip(raw_voltage, 0.0, self.V_max))

        # Anti-Windup Clamping Logic:
        # If output is saturated, only integrate if error acts to PULL the output
        # out of saturation. Stop integrating if error drives deeper into saturation.
        is_saturated = raw_voltage >= self.V_max or raw_voltage <= 0.0
        error_worsens_saturation = (raw_voltage >= self.V_max and error > 0) or (raw_voltage <= 0.0 and error < 0)

        if not (is_saturated and error_worsens_saturation):
            self._integral = candidate_integral

        # Clamp accumulator to prevent integrator runaway beyond physical voltage boundaries
        max_accum = self.V_max / max(1.0, self.Ki)
        self._integral = float(np.clip(self._integral, 0.0, max_accum))

        self._prev_error = error
        return saturated_voltage

    def get_info(self) -> str:
        return f"Kp={self.Kp:.1f}, Ki={self.Ki:.1f}, Kd={self.Kd:.2f}, Anti-Windup=ON"

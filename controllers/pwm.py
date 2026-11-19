"""
controllers/pwm.py
==============================================================================
Pulse-Width Modulation (PWM) Speed Controller.

Concept:
  PWM controls the average electric power delivered to a motor by modulating
  the duty cycle D in range [-1.0, +1.0] at a switching carrier frequency.
  
  Effective Voltage V_avg = D * V_max

  In industrial speed control without an integral accumulator:
    D = D_feedforward + Kp * (v_target - v_actual)
    where D_feedforward = v_target / v_max_linear

Characteristics:
  Smooth, continuous, vibration-free operation without violent chatter.
  However, because PWM lacks an integral accumulator (no 'I' term),
  it CANNOT eliminate steady-state error under heavy payload:
  when boxes are added, the motor experiences a permanent SPEED DROOP.
==============================================================================
"""

import numpy as np


class PWMController:
    name = "PWM"

    # Calibration constant: maximum linear speed at full 24V under no load
    V_MAX_LINEAR_SPEED = 3.48  # [m/s] at 24.0 V

    def __init__(self, V_max: float = 24.0, Kp: float = 0.60, carrier_hz: float = 1000.0):
        """
        Parameters:
            V_max: Power supply voltage limit [Volts].
            Kp: Proportional regulation gain mapping speed error to duty cycle adjustment.
            carrier_hz: PWM switching carrier frequency [Hz].
        """
        self.V_max = float(V_max)
        self.Kp = float(Kp)
        self.carrier_hz = float(carrier_hz)
        self._duty = 0.0

    def reset(self):
        """Reset internal duty cycle."""
        self._duty = 0.0

    def compute(self, setpoint: float, measurement: float, dt: float = None) -> float:
        """
        Calculate duty cycle D and return effective average armature voltage.
        
        Formula:
          D = D_nominal + Kp * (setpoint - measurement)
          V = D * V_max
        """
        error = setpoint - measurement

        # Feedforward duty cycle based on target setpoint
        d_nominal = setpoint / self.V_MAX_LINEAR_SPEED if self.V_MAX_LINEAR_SPEED > 0 else 0.0

        # Proportional duty cycle modulation
        d_total = d_nominal + self.Kp * error

        # Duty cycle saturation [0%, +100%] for unidirectional conveyor
        self._duty = float(np.clip(d_total, 0.0, 1.0))

        # Effective average voltage delivered to motor armature
        return self._duty * self.V_max

    @property
    def duty_percent(self) -> float:
        """Current duty cycle expressed as a percentage (-100% to +100%)."""
        return self._duty * 100.0

    def get_info(self) -> str:
        return f"V_max=±{self.V_max}V, Kp={self.Kp:.2f}, Carrier={self.carrier_hz}Hz"

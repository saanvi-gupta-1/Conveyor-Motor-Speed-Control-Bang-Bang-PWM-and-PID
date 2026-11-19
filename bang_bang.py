"""
controllers/bang_bang.py
==============================================================================
Bang-Bang (Binary On-Off / Relay with Hysteresis) Speed Controller.

Concept:
  Binary switching controller that supplies full power (+V_max) when the motor
  is below target speed, and shuts power OFF (0V) when the motor exceeds target speed.

  Hysteresis Deadband (±delta):
    Prevents instantaneous infinite-frequency switching (destructive contactor chatter).
    If error > +dead_band  ==> Apply +V_max (Power ON)
    If error < -dead_band  ==> Apply 0.0 V  (Power OFF / Coast with friction)
    If within [-dead_band, +dead_band] ==> Hold previous switching state

Characteristics:
  Fastest initial acceleration, simple implementation, but produces persistent
  limit-cycle oscillations (speed ripple) and high electrical current spikes.
==============================================================================
"""


class BangBangController:
    name = "Bang-Bang"

    def __init__(self, V_max: float = 24.0, dead_band: float = 0.03, bipolar: bool = False):
        """
        Parameters:
            V_max: Power supply voltage limit [Volts].
            dead_band: Half-width of hysteresis band [m/s].
            bipolar: If True, switches between -V_max and +V_max. If False (default for conveyors),
                     switches between 0.0V (OFF) and +V_max (ON).
        """
        self.V_max = float(V_max)
        self.dead_band = float(dead_band)
        self.bipolar = bool(bipolar)
        self._output = 0.0

    def reset(self):
        """Reset internal switching state."""
        self._output = 0.0

    def compute(self, setpoint: float, measurement: float, dt: float = None) -> float:
        """
        Calculate binary control voltage using hysteresis logic.
        """
        error = setpoint - measurement

        off_voltage = -self.V_max if self.bipolar else 0.0

        if error > self.dead_band:
            self._output = self.V_max
        elif error < -self.dead_band:
            self._output = off_voltage
        # Within dead-band: hold previous state to prevent chatter

        return self._output

    def get_info(self) -> str:
        mode_str = "±V_max" if self.bipolar else "0V / +V_max"
        return f"V_max={self.V_max}V, Deadband=±{self.dead_band:.3f} m/s ({mode_str})"

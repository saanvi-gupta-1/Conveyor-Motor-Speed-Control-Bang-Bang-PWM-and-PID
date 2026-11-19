"""
simulation/motor_model.py
==============================================================================
Mathematical Model of a Permanent Magnet DC Motor Driving a Conveyor Belt.

Electrical Equation:
  L * di/dt = V_in - R * i - Ke * omega

Mechanical Equation:
  J_eff * domega/dt = Kt * i - B * omega - T_load

Integration Method:
  4th-Order Runge-Kutta (RK4) for mathematical stability and precision.
==============================================================================
"""

import numpy as np


class DCMotorModel:
    """
    Simulates the electromechanical dynamics of a DC motor driving a conveyor.
    """

    # Physical parameters calibrated for an industrial conveyor drive:
    R = 1.0          # Armature coil resistance [Ohms]
    L = 0.04         # Armature coil inductance [Henries]
    Ke = 0.32        # Back-EMF voltage constant [V*s/rad]
    Kt = 0.32        # Motor torque constant [N*m/A]
    J_rotor = 0.015  # Motor rotor + drive pulley base inertia [kg*m^2]
    B = 0.008        # Bearing viscous friction coefficient [N*m*s/rad]
    r_pulley = 0.05  # Drive pulley radius [meters] (5 cm)
    V_max = 24.0     # Power supply voltage saturation limit [Volts]
    mu_belt = 0.12   # Friction coefficient of belt bed
    g = 9.81         # Gravitational acceleration [m/s^2]

    def __init__(self, dt: float = 1.0 / 240.0):
        self.dt = dt
        self.reset()

    def reset(self):
        """Reset motor state variables to rest (zero speed, zero current)."""
        self.i = 0.0            # Armature current [Amperes]
        self.omega = 0.0        # Rotor angular velocity [rad/s]
        self.T_external = 0.0   # External load disturbance torque [N*m]
        self.box_masses = []    # List of box masses currently on belt [kg]
        self.V_applied = 0.0    # Actual voltage applied after saturation [Volts]
        self.time = 0.0         # Elapsed simulation time [seconds]

    @property
    def belt_speed(self) -> float:
        """Linear velocity of the conveyor belt: v = omega * r [m/s]."""
        return self.omega * self.r_pulley

    @property
    def total_box_mass(self) -> float:
        """Total mass of all cargo boxes resting on the conveyor [kg]."""
        return sum(self.box_masses)

    @property
    def J_effective(self) -> float:
        """Effective reflected inertia: J_motor + (sum m_boxes) * r^2."""
        return self.J_rotor + self.total_box_mass * (self.r_pulley ** 2)

    @property
    def T_boxes(self) -> float:
        """Resistive friction torque created by cargo boxes on the belt [N*m]."""
        if abs(self.omega) < 1e-4:
            return 0.0
        direction = 1.0 if self.omega > 0 else -1.0
        return direction * self.mu_belt * self.total_box_mass * self.g * self.r_pulley

    @property
    def T_total_load(self) -> float:
        """Total opposing torque: external disturbance + cargo box friction [N*m]."""
        return self.T_external + self.T_boxes

    def set_external_load(self, torque_nm: float):
        """Apply an external resistance torque disturbance [N*m]."""
        self.T_external = max(0.0, float(torque_nm))

    def set_box_masses(self, masses: list[float]):
        """Update list of physical cargo boxes on the conveyor."""
        self.box_masses = [float(m) for m in masses]

    @property
    def linear_speed(self) -> float:
        """Alias for belt_speed [m/s]."""
        return self.belt_speed

    @property
    def armature_current(self) -> float:
        """Alias for motor current i [A]."""
        return self.i

    def step(self, V_command: float, T_load_override: float = None, J_eff_override: float = None, dt_override: float = None) -> dict:
        """
        Advance motor simulation by dt using 4th-order Runge-Kutta (RK4).

        Parameters:
            V_command: Voltage demanded by controller [Volts].
            T_load_override: Optional custom load torque override [N*m].
            J_eff_override: Optional custom effective inertia override [kg*m^2].
            dt_override: Optional custom timestep override [s].

        Returns:
            Dictionary containing telemetry snapshot.
        """
        # Actuator Saturation: Clamp voltage to physical power supply limit (0.0V to +24.0V)
        # Conveyor drives are unidirectional; reverse voltage is prevented by mechanical ratchet / drive diode
        V = float(np.clip(V_command, 0.0, self.V_max))
        self.V_applied = V

        dt = float(dt_override) if dt_override is not None else self.dt
        J_eff = float(J_eff_override) if J_eff_override is not None else self.J_effective
        T_load = float(T_load_override) if T_load_override is not None else self.T_total_load

        # System of Differential Equations:
        # di/dt    = (V - R*i - Ke*omega) / L
        # domega/dt = (Kt*i - B*omega - T_load) / J_eff
        def derivatives(curr_i, curr_w):
            di_dt = (V - self.R * curr_i - self.Ke * curr_w) / self.L
            dw_dt = (self.Kt * curr_i - self.B * curr_w - T_load) / J_eff
            return di_dt, dw_dt

        # RK4 Integration steps:
        # k1 = slope at beginning of step
        k1_i, k1_w = derivatives(self.i, self.omega)

        # k2 = slope at midpoint using k1
        k2_i, k2_w = derivatives(
            self.i + 0.5 * dt * k1_i,
            self.omega + 0.5 * dt * k1_w,
        )

        # k3 = slope at midpoint using k2
        k3_i, k3_w = derivatives(
            self.i + 0.5 * dt * k2_i,
            self.omega + 0.5 * dt * k2_w,
        )

        # k4 = slope at end of step using k3
        k4_i, k4_w = derivatives(
            self.i + dt * k3_i,
            self.omega + dt * k3_w,
        )

        # Weighted average update: (k1 + 2*k2 + 2*k3 + k4) / 6
        self.i += (dt / 6.0) * (k1_i + 2 * k2_i + 2 * k3_i + k4_i)
        self.omega += (dt / 6.0) * (k1_w + 2 * k2_w + 2 * k3_w + k4_w)

        # Unidirectional conveyor mechanical ratchet / clutch constraint (cannot rotate in reverse)
        if self.omega < 0.0:
            self.omega = 0.0
            if self.i < 0.0:
                self.i = 0.0

        self.time += dt

        return {
            "time": self.time,
            "belt_speed": self.belt_speed,
            "omega": self.omega,
            "current": self.i,
            "voltage": V,
            "saturated": abs(V_command) > self.V_max,
            "T_load": T_load,
            "total_box_mass": self.total_box_mass,
        }

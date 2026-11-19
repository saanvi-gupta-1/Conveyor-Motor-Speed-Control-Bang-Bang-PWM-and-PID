"""Real-time interactive DC motor conveyor simulation engine.

Couples motor dynamics, package transport physics, controller updates,
and telemetry streaming for the modern industrial control room.
"""

from collections import deque
from dataclasses import dataclass, field
import math
import random
import threading
import time
from typing import Any, Dict, List, Optional

from .controllers import Controller, BangBangController, PWMController, PIDController, ControlOutput
from .model import DCMotor, MotorParameters
from .experiments import make_controller, run_suite, run_pwm_frequency_sweep
from .simulation import SimulationConfig
from .metrics import calculate_metrics
from .results import interpretation


@dataclass
class Package:
    id: int
    x: float  # position along conveyor in meters [-1.5, 1.5]
    mass_kg: float
    width_m: float = 0.28
    color: str = "#f59e0b"
    label: str = "Standard"


class LiveSimulationEngine:
    """Thread-safe real-time closed-loop conveyor simulation."""

    def __init__(self, dt: float = 0.01, motor_params: Optional[MotorParameters] = None):
        self.lock = threading.RLock()
        self.dt = dt
        self.speed_multiplier = 1.0
        self.motor_params = motor_params or MotorParameters()
        self.motor = DCMotor(self.motor_params)

        # Operational state
        self.is_running = False
        self.sim_time = 0.0
        self.target_speed = 1.8  # m/s
        self.base_load_torque = 0.08  # Nm (belt tare friction + minimal load)
        self.manual_load_torque = 0.0  # Nm
        self.active_controller_name = "pid"

        # Controllers
        self.controllers: Dict[str, Controller] = {
            "bang-bang": BangBangController(deadband=0.12, limits=(0.0, 1.0)),
            "pwm": PWMController(frequency_hz=20.0, limits=(0.0, 1.0)),
            "pid": PIDController(kp=0.7, ki=0.8, kd=0.01, limits=(0.0, 1.0)),
        }
        self.active_controller: Controller = self.controllers["pid"]

        # Conveyor package logistics simulation
        self.packages: List[Package] = []
        self._next_pkg_id = 1
        self._init_packages()

        # Rolling history buffer (last 500 samples for live charts)
        self.history: deque = deque(maxlen=600)
        self.session_data: List[Dict[str, Any]] = []

        # Background thread control
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    def _init_packages(self) -> None:
        """Seed 3 initial packages spaced along the conveyor."""
        colors = ["#f59e0b", "#3b82f6", "#10b981"]
        masses = [1.5, 2.5, 1.8]
        labels = ["Electronics", "Mechanical Parts", "Raw Material"]
        self.packages = [
            Package(id=1, x=-1.0, mass_kg=masses[0], color=colors[0], label=labels[0]),
            Package(id=2, x=-0.2, mass_kg=masses[1], color=colors[1], label=labels[1]),
            Package(id=3, x=0.6, mass_kg=masses[2], color=colors[2], label=labels[2]),
        ]
        self._next_pkg_id = 4

    @property
    def total_load_torque(self) -> float:
        """Total load torque including base friction, package mass contribution, and manual disturbances."""
        pkg_torque = sum(p.mass_kg * 0.025 for p in self.packages if -1.4 <= p.x <= 1.4)
        return min(self.motor_params.max_load_torque, max(0.0, self.base_load_torque + pkg_torque + self.manual_load_torque))

    def step(self) -> Dict[str, Any]:
        """Perform one discrete numerical simulation step."""
        with self.lock:
            t = self.sim_time
            target = self.target_speed
            load = self.total_load_torque
            v_measured = self.motor.belt_speed

            # Run active controller
            out: ControlOutput = self.active_controller.update(target, v_measured, self.dt, t)

            # Step physical DC motor model
            self.motor.step(out.command, load, self.dt)
            self.sim_time += self.dt

            # Update package kinematics
            v_actual = self.motor.belt_speed
            dx = v_actual * self.dt
            for pkg in self.packages:
                pkg.x += dx
                # Recycle package when it falls off the exit roller (x > 1.4 m)
                if pkg.x > 1.45:
                    pkg.x = -1.45
                    pkg.mass_kg = round(random.uniform(1.0, 3.5), 1)

            rpm = self.motor.omega * (60.0 / (2.0 * math.pi))
            target_rpm = (target / self.motor_params.roller_radius) * (60.0 / (2.0 * math.pi))
            error_m_s = target - v_actual

            sample = {
                "time_s": round(self.sim_time, 4),
                "target_speed_m_s": round(target, 4),
                "actual_speed_m_s": round(v_actual, 4),
                "target_rpm": round(target_rpm, 1),
                "actual_rpm": round(rpm, 1),
                "omega_rad_s": round(self.motor.omega, 3),
                "error_m_s": round(error_m_s, 4),
                "raw_command": round(out.raw, 4),
                "command": round(out.command, 4),
                "saturated": bool(out.saturated),
                "duty": round(out.duty, 4),
                "pwm_on": bool(out.switched_on),
                "switch_count": int(out.switch_count),
                "load_torque_Nm": round(load, 4),
                "p_term": round(out.p_term, 4),
                "i_term": round(out.i_term, 4),
                "d_term": round(out.d_term, 4),
                "controller": self.active_controller_name,
            }

            self.history.append(sample)
            self.session_data.append(sample)
            return sample

    def start(self) -> None:
        with self.lock:
            if not self.is_running:
                self.is_running = True
                if self._thread is None or not self._thread.is_alive():
                    self._stop_event.clear()
                    self._thread = threading.Thread(target=self._run_loop, daemon=True)
                    self._thread.start()

    def pause(self) -> None:
        with self.lock:
            self.is_running = False

    def reset(self) -> None:
        with self.lock:
            self.is_running = False
            self.sim_time = 0.0
            self.motor.reset(0.0)
            self.active_controller.reset()
            self._init_packages()
            self.history.clear()
            self.session_data.clear()

    def set_controller(self, name: str) -> None:
        with self.lock:
            name = name.lower()
            if name in self.controllers:
                self.active_controller_name = name
                self.active_controller = self.controllers[name]
                self.active_controller.reset()

    def update_pid_params(self, kp: Optional[float] = None, ki: Optional[float] = None, kd: Optional[float] = None) -> None:
        with self.lock:
            pid = self.controllers["pid"]
            if isinstance(pid, PIDController):
                if kp is not None and kp >= 0:
                    pid.kp = kp
                if ki is not None and ki >= 0:
                    pid.ki = ki
                if kd is not None and kd >= 0:
                    pid.kd = kd

    def update_pwm_params(self, frequency_hz: Optional[float] = None) -> None:
        with self.lock:
            pwm = self.controllers["pwm"]
            if isinstance(pwm, PWMController):
                if frequency_hz is not None and frequency_hz > 0:
                    pwm.frequency_hz = frequency_hz

    def update_bangbang_params(self, deadband: Optional[float] = None) -> None:
        with self.lock:
            bb = self.controllers["bang-bang"]
            if isinstance(bb, BangBangController):
                if deadband is not None and deadband >= 0:
                    bb.deadband = deadband

    def set_target_speed(self, speed_m_s: float) -> None:
        with self.lock:
            self.target_speed = max(0.0, min(3.5, float(speed_m_s)))

    def set_manual_load(self, torque_Nm: float) -> None:
        with self.lock:
            self.manual_load_torque = max(0.0, min(0.25, float(torque_Nm)))

    def inject_disturbance(self, disturbance_type: str) -> Dict[str, Any]:
        """Trigger predefined physical disturbance events."""
        with self.lock:
            if disturbance_type == "heavy_box":
                # Drops a 4.5kg cargo package at intake
                self.packages.append(Package(id=self._next_pkg_id, x=-1.4, mass_kg=4.5, color="#ef4444", label="Heavy Cargo"))
                self._next_pkg_id += 1
                return {"message": "Heavy box added (+4.5 kg load disturbance)", "type": "warning"}
            elif disturbance_type == "speed_step":
                self.target_speed = 2.4 if self.target_speed < 2.0 else 1.2
                return {"message": f"Target speed stepped to {self.target_speed:.2f} m/s", "type": "info"}
            elif disturbance_type == "torque_pulse":
                self.manual_load_torque = 0.18
                # Auto-release torque pulse after 2 seconds
                def _release():
                    time.sleep(2.0)
                    with self.lock:
                        self.manual_load_torque = 0.0
                threading.Thread(target=_release, daemon=True).start()
                return {"message": "Torque disturbance pulse (+0.18 Nm for 2.0s) injected", "type": "warning"}
            elif disturbance_type == "clear_load":
                self.manual_load_torque = 0.0
                self._init_packages()
                return {"message": "All cargo cleared to nominal tare weight", "type": "info"}
            return {"message": "Unknown disturbance type", "type": "error"}

    def get_telemetry(self) -> Dict[str, Any]:
        with self.lock:
            v_actual = self.motor.belt_speed
            target = self.target_speed
            error = target - v_actual
            rpm = self.motor.omega * (60.0 / (2.0 * math.pi))
            target_rpm = (target / self.motor_params.roller_radius) * (60.0 / (2.0 * math.pi))
            load = self.total_load_torque

            # Determine rule-based system diagnostic insights
            diagnostics = self._compute_diagnostics(error, v_actual, target, load)

            return {
                "is_running": self.is_running,
                "sim_time": round(self.sim_time, 2),
                "target_speed_m_s": round(target, 3),
                "actual_speed_m_s": round(v_actual, 3),
                "target_rpm": round(target_rpm, 1),
                "actual_rpm": round(rpm, 1),
                "error_m_s": round(error, 3),
                "load_torque_Nm": round(load, 3),
                "controller": self.active_controller_name,
                "diagnostics": diagnostics,
                "packages": [{"id": p.id, "x": round(p.x, 3), "mass_kg": p.mass_kg, "color": p.color, "label": p.label} for p in self.packages],
                "history_chunk": list(self.history)[-50:] if self.history else [],
                "params": {
                    "kp": getattr(self.controllers["pid"], "kp", 0.7),
                    "ki": getattr(self.controllers["pid"], "ki", 0.8),
                    "kd": getattr(self.controllers["pid"], "kd", 0.01),
                    "pwm_freq": getattr(self.controllers["pwm"], "frequency_hz", 20.0),
                    "bangbang_deadband": getattr(self.controllers["bang-bang"], "deadband", 0.12),
                }
            }

    def _compute_diagnostics(self, error: float, v: float, target: float, load: float) -> List[Dict[str, str]]:
        """Real-time rule-based diagnostics derived from actual physical state."""
        insights = []
        if abs(error) <= 0.04 * max(target, 0.5):
            insights.append({"level": "nominal", "title": "Nominal Tracking", "desc": f"Belt speed is tightly tracking setpoint within 4% tolerance."})
        elif error > 0.15:
            insights.append({"level": "warning", "title": "Tracking Undershoot", "desc": f"Speed is {abs(error):.2f} m/s below setpoint. Motor accelerating."})
        elif error < -0.15:
            insights.append({"level": "warning", "title": "Speed Overshoot", "desc": f"Conveyor speed exceeded target by {abs(error):.2f} m/s."})

        if load > 0.22:
            insights.append({"level": "critical", "title": "Heavy Load Warning", "desc": f"Conveyor load ({load:.2f} Nm) is near motor torque capacity."})

        if self.active_controller_name == "bang-bang":
            insights.append({"level": "info", "title": "Relay Mode Active", "desc": "Bang-Bang switching creates an orbitally stable limit cycle."})
        elif self.active_controller_name == "pwm":
            insights.append({"level": "info", "title": "Carrier Modulation", "desc": "Discrete pulse switching drives motor. Mechanical inertia filters speed."})
        elif self.active_controller_name == "pid":
            pid = self.controllers["pid"]
            if hasattr(pid, "integral") and abs(pid.integral) > 0.5:
                insights.append({"level": "info", "title": "Integral Active", "desc": f"Anti-windup PID integral term is actively rejecting steady load ({load:.2f} Nm)."})

        return insights

    def _run_loop(self) -> None:
        """Precision simulation loop running at real-time clock pacing."""
        while not self._stop_event.is_set():
            if self.is_running:
                t0 = time.perf_counter()
                self.step()
                elapsed = time.perf_counter() - t0
                target_sleep = (self.dt / max(0.1, self.speed_multiplier)) - elapsed
                if target_sleep > 0.001:
                    time.sleep(target_sleep)
            else:
                time.sleep(0.05)

    def stop_server(self) -> None:
        self.pause()
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

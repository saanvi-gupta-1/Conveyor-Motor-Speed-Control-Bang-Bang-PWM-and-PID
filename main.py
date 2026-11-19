"""
main.py
==============================================================================
ACADEMIC PROJECT P15:
CONVEYOR MOTOR SPEED CONTROL: BANG-BANG, PWM AND PID
==============================================================================
Main Entry Point integrating:
  1. Permanent Magnet DC Motor state-space model (RK4 ODE solver)
  2. Conveyor belt kinematics and dynamic cargo box payload reflection
  3. Bang-Bang, PWM, and PID control algorithms with anti-windup clamping
  4. Real-time 3D PyBullet physics visualization with GUI sliders & buttons
  5. Interactive beginner-friendly Command-Line Dashboard (CLI)
  6. Scientific evaluation metrics and multi-variable comparison plots
==============================================================================
"""

import sys
import os
import time
import threading
from typing import Dict, Any

# Ensure project root is on Python module search path
PROJECT_ROOT = os.path.abspath(os.path.dirname(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import pybullet as pb

from simulation.motor_model import DCMotorModel
from simulation.conveyor import ConveyorBeltSystem
from simulation.conveyor_scene import ConveyorScene3D
from controllers import BangBangController, PWMController, PIDController
from evaluation.metrics import compute_controller_metrics, format_metrics_table
from evaluation.plots import plot_live_telemetry, plot_comparison_benchmark, plot_session_controllers_comparison
from dashboard.cli_menu import print_banner, print_status_bar, print_menu, handle_cli_choice


class SimulationManager:
    """
    Coordinates motor physics, conveyor mechanics, controllers,
    3D visualization, and user telemetry.
    """

    def __init__(self, enable_gui: bool = True):
        self.enable_gui = enable_gui
        self.running = True
        self.paused = False
        self.ready = False

        # Physical Subsystems
        self.motor = DCMotorModel()
        self.conveyor = ConveyorBeltSystem()
        self.scene: ConveyorScene3D = None

        # Control System
        self.target_speed = 1.0  # [m/s]
        self.active_controller_name = "pid"
        self.pid_gains = {"kp": 55.0, "ki": 45.0, "kd": 0.50}

        self.controllers = {
            "bang": BangBangController(V_max=24.0, dead_band=0.03, bipolar=False),
            "pwm": PWMController(V_max=24.0, Kp=0.60),
            "pid": PIDController(
                Kp=self.pid_gains["kp"],
                Ki=self.pid_gains["ki"],
                Kd=self.pid_gains["kd"],
                V_max=24.0,
            ),
        }

        # Telemetry State
        self.sim_time = 0.0
        self.sim_dt = 1.0 / 240.0
        self.last_voltage = 0.0
        self.is_saturated = False

        # Live history buffer (max 3600 points = ~15 seconds)
        self.history = {
            "time": [],
            "speed": [],
            "target_speed": [],
            "voltage": [],
            "current": [],
            "load_torque": [],
            "controller": [],
        }

    def set_controller(self, name: str):
        cname = name.lower()
        if cname in self.controllers:
            self.active_controller_name = cname
            self.controllers[cname].reset()

    def set_pid_gains(self, kp: float, ki: float, kd: float):
        self.pid_gains = {"kp": kp, "ki": ki, "kd": kd}
        self.controllers["pid"].set_gains(kp, ki, kd)

    def inject_box(self, preset_name: str):
        """Add a cargo box with real-world rotational momentum impact shock."""
        box = self.conveyor.add_preset_box(preset_name)
        if box:
            # Physical Momentum Shock: Accelerating stationary mass m (v=0)
            # to current belt speed extracts rotational kinetic energy from the motor rotor:
            # omega_after = omega_before * (J_rotor / (J_rotor + m * r^2))
            j_box = box.mass * (self.conveyor.PULLEY_RAD ** 2)
            momentum_factor = self.motor.J_rotor / (self.motor.J_rotor + j_box)
            speed_before = self.motor.linear_speed
            self.motor.omega *= momentum_factor
            speed_after = self.motor.linear_speed
            t_drop = box.mass * self.conveyor.GRAVITY * self.conveyor.PULLEY_RAD * 0.65
            print(f"\n  >>> [LOAD DISTURBANCE INJECTED] {box.name} dropped onto conveyor!")
            print(f"      -> Impact Speed Dip: {speed_before:.2f} m/s -> {speed_after:.2f} m/s (-{speed_before - speed_after:.2f} m/s dip)")
            print(f"      -> Dynamic Slip Drag: {t_drop:.2f} N*m | Total Payload Mass: {self.conveyor.total_mass:.1f} kg")
        return box

    def reset(self):
        """Reset motor, belt, and default boxes."""
        self.motor.reset()
        self.conveyor.reset()
        for c in self.controllers.values():
            c.reset()
        self.last_voltage = 0.0
        self.is_saturated = False
        for k in self.history:
            self.history[k].clear()

    def step(self):
        """Execute one simulation cycle (240 Hz)."""
        if self.scene is None or not self.scene.is_connected():
            return

        dt = self.sim_dt

        # 1. Read interactive GUI inputs (Sliders & Buttons in PyBullet)
        gui_events = self.scene.read_gui_inputs()
        if gui_events:
            if "speed" in gui_events:
                self.target_speed = gui_events["speed"]
                print(f"  >>> [Speed Setpoint Slider] Target Speed -> {self.target_speed:.2f} m/s (Range: 0.0 to 3.0 m/s)")
            if "ctrl" in gui_events:
                self.set_controller(gui_events["ctrl"])
                ctrl_mode_names = {"pid": "PID (Tuned Anti-Windup)", "pwm": "PWM (Proportional Duty Cycle)", "bang": "Bang-Bang (On/Off Hysteresis)"}
                print(f"  >>> [Controller Mode Slider] Switched to -> {ctrl_mode_names.get(self.active_controller_name, self.active_controller_name.upper())}")
            if "kp" in gui_events or "ki" in gui_events or "kd" in gui_events:
                kp = gui_events.get("kp", self.pid_gains["kp"])
                ki = gui_events.get("ki", self.pid_gains["ki"])
                kd = gui_events.get("kd", self.pid_gains["kd"])
                self.set_pid_gains(kp, ki, kd)
                if "pwm" in self.controllers:
                    self.controllers["pwm"].Kp = kp * 0.015
                print(f"  >>> [PID Gains Slider] Kp={kp:.1f}, Ki={ki:.1f}, Kd={kd:.2f}")
            if gui_events.get("add_light"):
                self.inject_box("light")
            if gui_events.get("add_medium"):
                self.inject_box("medium")
            if gui_events.get("add_heavy"):
                self.inject_box("heavy")
            if gui_events.get("reset"):
                self.reset()
                print("  >>> [Reset] Conveyor, motor, and cargo boxes reset to default.")
            if gui_events.get("plot"):
                threading.Thread(target=self.generate_telemetry_plot, daemon=True).start()
            if gui_events.get("compare_plot"):
                threading.Thread(target=self.run_benchmark_comparison, daemon=True).start()

        # 2. Advance physics if not paused
        if not self.paused:
            current_speed = self.motor.linear_speed
            ctrl = self.controllers[self.active_controller_name]

            # Compute control voltage
            u_v = ctrl.compute(self.target_speed, current_speed, dt)

            # Actuator saturation constraint: Unidirectional forward conveyor drive [0V to +24V]
            v_sat = max(0.0, min(24.0, u_v))
            self.is_saturated = v_sat >= 23.95
            self.last_voltage = v_sat

            # Compute effective reflected payload
            j_eff = self.motor.J_rotor + self.conveyor.payload_reflected_inertia
            t_load = self.conveyor.payload_friction_torque

            # Step DC motor state-space ODEs via RK4
            self.motor.step(v_sat, t_load, j_eff, dt)

            # Advance physical conveyor belt and cargo boxes
            discharged_boxes = self.conveyor.update(self.motor.linear_speed, dt)
            for d_box in discharged_boxes:
                print(f"  >>> [Discharge Arc] '{d_box.name}' launched off roller -> Collected in Hopper Bin!")
            self.sim_time += dt

            # Record telemetry
            self.history["time"].append(self.sim_time)
            self.history["speed"].append(self.motor.linear_speed)
            self.history["target_speed"].append(self.target_speed)
            self.history["voltage"].append(v_sat)
            self.history["current"].append(self.motor.armature_current)
            self.history["load_torque"].append(t_load)
            self.history["controller"].append(self.active_controller_name)

            # Keep telemetry bounded to prevent memory growth
            if len(self.history["time"]) > 3600:
                for k in self.history:
                    self.history[k].pop(0)

        # 3. Synchronize 3D rigid bodies and HUD
        self.scene.sync_boxes(self.conveyor.boxes)
        self.scene.update(
            belt_speed=self.motor.linear_speed,
            dt=dt,
            target_speed=self.target_speed,
            ctrl_name=self.active_controller_name,
            voltage=self.last_voltage,
            load_nm=self.conveyor.payload_friction_torque,
            is_saturated=self.is_saturated,
            current_a=self.motor.armature_current,
            box_count=self.conveyor.active_box_count,
            total_mass=self.conveyor.total_mass,
            in_bin_count=self.conveyor.in_bin_count,
        )

        try:
            pb.stepSimulation()
        except Exception:
            self.running = False

    def generate_telemetry_plot(self):
        """Generate and display session telemetry plot."""
        try:
            if len(self.history["time"]) < 50:
                print("\n  [!] Not enough telemetry data yet to plot. Let the simulation run for a few seconds.")
                return

            # Snapshot copy to prevent list modification race conditions
            snapshot = {k: list(v) for k, v in self.history.items()}

            out_path = os.path.join(PROJECT_ROOT, "conveyor_telemetry.jpg")
            saved_file = plot_live_telemetry(snapshot, save_path=out_path)
            if saved_file:
                print(f"\n  [✓] Telemetry plot saved to: {saved_file}")
                try:
                    if sys.platform == "win32":
                        os.startfile(saved_file)
                except Exception:
                    pass
        except Exception as e:
            print(f"\n  [!] Could not generate telemetry plot: {e}")

    def run_benchmark_comparison(self):
        """
        Execute an automated, rigorous comparative test of Bang-Bang, PWM, and PID
        under identical operating conditions and cargo disturbances.
        """
        print("\n" + "=" * 70)
        print("  AUTOMATED BENCHMARK: BANG-BANG vs PWM vs PID SPEED CONTROL")
        print("=" * 70)
        print("  Test Cycle Description:")
        print("    - 0.0s to 4.0s : Step command from 0.0 to 1.2 m/s (Empty belt)")
        print("    - 4.0s         : Disturbance 1 -> Medium Cargo Box (3.0 kg dropped)")
        print("    - 8.0s         : Disturbance 2 -> Heavy Cargo Box (8.0 kg dropped)")
        print("    - 12.0s        : Benchmark evaluation complete")
        print("-" * 70)

        results = {}
        metrics_summary = {}

        controllers_to_test = [
            ("bang", BangBangController(V_max=24.0, dead_band=0.03, bipolar=False)),
            ("pwm",  PWMController(V_max=24.0, Kp=0.60)),
            ("pid",  PIDController(Kp=50.0, Ki=40.0, Kd=0.50, V_max=24.0)),
        ]

        dt = 1.0 / 240.0
        t_total = 12.0
        n_steps = int(t_total / dt)
        target_v = 1.20

        for cname, ctrl in controllers_to_test:
            print(f"  Testing Controller: [{cname.upper()}] ...", end="", flush=True)

            m = DCMotorModel()
            conv = ConveyorBeltSystem()
            conv.BELT_LENGTH = 50.0  # Maintain disturbance payload on belt across full benchmark window
            conv.clear_all_boxes()  # Start empty for pure step response test
            ctrl.reset()

            res = {
                "time": [],
                "speed": [],
                "target_speed": [],
                "voltage": [],
                "current": [],
                "load_torque": [],
            }

            for step_i in range(n_steps):
                t_now = step_i * dt

                # Injected disturbances
                if step_i == int(4.0 / dt):
                    conv.add_preset_box("medium")  # +3.0 kg
                elif step_i == int(8.0 / dt):
                    conv.add_preset_box("heavy")   # +8.0 kg

                v_act = m.linear_speed
                v_ctrl = ctrl.compute(target_v, v_act, dt)
                v_sat = max(0.0, min(24.0, v_ctrl))

                j_eff = m.J_rotor + conv.payload_reflected_inertia
                t_load = conv.payload_friction_torque

                m.step(v_sat, t_load, j_eff, dt)
                conv.update(m.linear_speed, dt)

                res["time"].append(t_now)
                res["speed"].append(m.linear_speed)
                res["target_speed"].append(target_v)
                res["voltage"].append(v_sat)
                res["current"].append(m.armature_current)
                res["load_torque"].append(t_load)

            results[cname] = res
            metrics_summary[cname] = compute_controller_metrics(
                res["time"],
                res["speed"],
                target_speed=target_v,
                t_step_start=0.0,
                t_disturb=4.0,
            )
            print(" Done.")

        print("\n" + "=" * 70)
        print("  QUANTITATIVE PERFORMANCE COMPARISON TABLE")
        print("=" * 70)
        table_str = format_metrics_table(metrics_summary)
        print(table_str)
        print("=" * 70)

        # Plot and save comparison figures
        out_path = os.path.join(PROJECT_ROOT, "comparison_benchmark.jpg")
        saved_fig = plot_comparison_benchmark(results, save_path=out_path)

        out_session_path = os.path.join(PROJECT_ROOT, "session_3_controllers_comparison.jpg")
        saved_session_fig = plot_session_controllers_comparison(results, metrics=metrics_summary, save_path=out_session_path)

        print(f"\n  [✓] Benchmark comparison plot saved to: {saved_fig}")
        print(f"  [✓] 3-Controller session scorecard plot saved to: {saved_session_fig}")
        try:
            if sys.platform == "win32":
                os.startfile(saved_session_fig)
        except Exception:
            pass


def simulation_thread_worker(manager: SimulationManager):
    """Background worker driving PyBullet 3D physics and rendering at 240 Hz."""
    try:
        manager.scene = ConveyorScene3D(enable_gui=manager.enable_gui)
        manager.ready = True

        sim_dt = manager.sim_dt
        while manager.running and manager.scene.is_connected():
            manager.step()
            time.sleep(sim_dt)
    except Exception as e:
        pass
    finally:
        if manager.scene is not None:
            manager.scene.close()


def main():
    print_banner()
    print("  Initializing DC Motor, Conveyor Mechanics & PyBullet 3D Engine...")

    manager = SimulationManager(enable_gui=True)

    # Launch PyBullet physics in background thread
    sim_thread = threading.Thread(target=simulation_thread_worker, args=(manager,), daemon=True)
    sim_thread.start()

    # Wait for 3D scene to initialize
    timeout = 10.0
    t0 = time.time()
    while not manager.ready and time.time() - t0 < timeout:
        time.sleep(0.1)

    if not manager.ready:
        print("  [X] Failed to initialize PyBullet 3D environment.")
        return

    print("  [✓] PyBullet 3D Conveyor Simulation active on screen.")
    print("  [i] Interactive Controls Available:")
    print("      - Right Panel Sliders: Target Speed, Controller, Kp, Ki, Kd")
    print("      - Right Panel Buttons: Add Light/Med/Heavy Box, Reset, Plots")
    print("      - Terminal Commands  : Numbered Menu below\n")

    print_status_bar(manager)
    print_menu()

    # Main thread CLI loop
    try:
        while manager.running:
            if manager.scene and not manager.scene.is_connected():
                print("\n  [i] 3D Visualization window was closed.")
                break

            try:
                choice = input("\nconveyor> ").strip()
                if not choice:
                    continue
                keep_running = handle_cli_choice(choice, manager)
                if not keep_running:
                    break
            except (EOFError, KeyboardInterrupt):
                print("\n  [!] Exiting...")
                break

    finally:
        manager.running = False
        time.sleep(0.2)
        print("\n  [✓] Conveyor Motor Speed Control session completed successfully.\n")


if __name__ == "__main__":
    main()

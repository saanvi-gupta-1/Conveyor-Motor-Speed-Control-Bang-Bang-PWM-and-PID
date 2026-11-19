"""
dashboard/cli_menu.py
==============================================================================
Interactive Beginner-Friendly Command-Line Interface (CLI) Dashboard.

Provides:
  - Real-time simulation status display
  - One-touch controller switching (Bang-Bang, PWM, PID)
  - Speed setpoint and PID gain adjustments
  - Cargo box disturbance injection (Light 0.5kg, Medium 2kg, Heavy 6kg)
  - Automated 3-controller benchmark execution
  - Real-time telemetry plotting
==============================================================================
"""

import sys
import time
from typing import Any

try:
    from colorama import init, Fore, Style
    init(autoreset=True)
    HAS_COLOR = True
except Exception:
    HAS_COLOR = False

    class _DummyColor:
        def __getattr__(self, name):
            return ""

    Fore = _DummyColor()
    Style = _DummyColor()


def print_banner():
    banner = f"""
{Fore.CYAN}================================================================================
          CONVEYOR MOTOR SPEED CONTROL: BANG-BANG, PWM & PID (P15)
================================================================================{Style.RESET_ALL}"""
    print(banner)


def print_status_bar(sim_state: Any):
    """Display real-time state of the simulation, motor, and payload."""
    cname = sim_state.active_controller_name.upper()
    v_tgt = sim_state.target_speed
    v_act = sim_state.motor.linear_speed
    v_err = v_tgt - v_act
    volt = sim_state.last_voltage
    curr = sim_state.motor.armature_current
    t_load = sim_state.conveyor.payload_friction_torque
    n_boxes = sim_state.conveyor.active_box_count
    total_m = sim_state.conveyor.total_mass
    status_str = "PAUSED" if sim_state.paused else "RUNNING"

    color_err = Fore.GREEN if abs(v_err) < 0.05 else Fore.YELLOW if abs(v_err) < 0.15 else Fore.RED
    color_ctrl = Fore.GREEN if cname == "PID" else Fore.YELLOW if cname == "PWM" else Fore.MAGENTA

    print(f"\n{Fore.WHITE}--- [ SIMULATION STATUS : {Fore.CYAN}{status_str}{Fore.WHITE} ] ----------------------------------------------------")
    print(f"  Controller   : {color_ctrl}[{cname}]{Style.RESET_ALL}       Target Speed : {Fore.CYAN}{v_tgt:.3f} m/s{Style.RESET_ALL}")
    print(f"  Actual Speed : {color_err}{v_act:+6.3f} m/s{Style.RESET_ALL}    Speed Error  : {color_err}{v_err:+6.3f} m/s{Style.RESET_ALL}")
    print(f"  Armature V   : {Fore.YELLOW}{volt:+6.1f} V{Style.RESET_ALL}      Current      : {Fore.YELLOW}{curr:+6.2f} A{Style.RESET_ALL}")
    print(f"  Load Torque  : {Fore.MAGENTA}{t_load:4.2f} N*m{Style.RESET_ALL}     Cargo Boxes  : {Fore.CYAN}{n_boxes} active ({total_m:.1f} kg total){Style.RESET_ALL}")
    print(f"--------------------------------------------------------------------------------{Style.RESET_ALL}")


def print_menu():
    print(f"""{Fore.WHITE}Available Commands:
  {Fore.GREEN}[1]{Style.RESET_ALL} Run PID Controller (Tuned with Anti-Windup Clamping)
  {Fore.YELLOW}[2]{Style.RESET_ALL} Run PWM Controller (Proportional Duty-Cycle Modulation)
  {Fore.MAGENTA}[3]{Style.RESET_ALL} Run Bang-Bang Controller (Binary ON/OFF with Hysteresis)
  {Fore.CYAN}[4]{Style.RESET_ALL} Set Target Conveyor Speed (0.1 to 3.0 m/s)
  {Fore.CYAN}[5]{Style.RESET_ALL} Add Cargo Box Disturbance (Light 0.5kg, Medium 2kg, Heavy 6kg)
  {Fore.CYAN}[6]{Style.RESET_ALL} Remove Cargo Box / Clear All Boxes
  {Fore.CYAN}[7]{Style.RESET_ALL} Tune PID Gains (Kp, Ki, Kd)
  {Fore.CYAN}[8]{Style.RESET_ALL} Run Automated 3-Controller Benchmark Comparison (Plot & Table)
  {Fore.CYAN}[C]{Style.RESET_ALL} Plot 3-Controller Session Scorecard & Comparison Figures
  {Fore.CYAN}[9]{Style.RESET_ALL} View Live Telemetry Graph (Matplotlib 4-Panel Dashboard)
  {Fore.WHITE}[P]{Style.RESET_ALL} Pause / Resume Simulation
  {Fore.WHITE}[R]{Style.RESET_ALL} Reset Conveyor & Motor to Default State
  {Fore.WHITE}[H]{Style.RESET_ALL} Show Current Real-Time Telemetry Status
  {Fore.RED}[Q]{Style.RESET_ALL} Quit Simulation
""")


def handle_cli_choice(choice: str, sim_state: Any) -> bool:
    """
    Process user input and update simulation state accordingly.
    Returns False if user requests exit, True otherwise.
    """
    cmd = choice.strip().lower()

    if cmd == "q":
        print(f"\n{Fore.RED}[!] Shutting down simulation...{Style.RESET_ALL}")
        sim_state.running = False
        return False

    elif cmd == "1":
        sim_state.set_controller("pid")
        print(f"{Fore.GREEN}[✓] Switched to PID Controller.{Style.RESET_ALL}")

    elif cmd == "2":
        sim_state.set_controller("pwm")
        print(f"{Fore.YELLOW}[✓] Switched to PWM Controller.{Style.RESET_ALL}")

    elif cmd == "3":
        sim_state.set_controller("bang")
        print(f"{Fore.MAGENTA}[✓] Switched to Bang-Bang Controller.{Style.RESET_ALL}")

    elif cmd == "4":
        try:
            val_str = input(f"{Fore.CYAN}Enter target belt speed [0.1 to 3.0 m/s]: {Style.RESET_ALL}")
            val = float(val_str)
            if 0.1 <= val <= 3.0:
                sim_state.target_speed = val
                print(f"{Fore.GREEN}[✓] Target speed set to {val:.3f} m/s.{Style.RESET_ALL}")
            else:
                print(f"{Fore.RED}[X] Speed out of safe range (0.1 - 3.0 m/s).{Style.RESET_ALL}")
        except ValueError:
            print(f"{Fore.RED}[X] Invalid numeric value.{Style.RESET_ALL}")

    elif cmd == "5":
        print("\nSelect cargo box weight:")
        print("  1. Light Box (1.0 kg)")
        print("  2. Medium Box (3.0 kg)")
        print("  3. Heavy Box (8.0 kg) - High Disturbance")
        sub = input("Choose box type [1-3]: ").strip()
        if sub == "1":
            box = sim_state.inject_box("light")
        elif sub == "2":
            box = sim_state.inject_box("medium")
        elif sub == "3":
            box = sim_state.inject_box("heavy")
        else:
            print(f"{Fore.RED}[X] Invalid selection.{Style.RESET_ALL}")

    elif cmd == "6":
        print("\nBox Removal Options:")
        print("  1. Remove Single Box")
        print("  2. Clear All Boxes (Empty Conveyor)")
        sub = input("Choose option [1-2]: ").strip()
        if sub == "1":
            removed = sim_state.conveyor.remove_box()
            if removed:
                print(f"{Fore.YELLOW}[✓] Removed {removed.name} ({removed.mass} kg). Remaining: {sim_state.conveyor.active_box_count}{Style.RESET_ALL}")
            else:
                print(f"{Fore.RED}[!] Conveyor is already empty.{Style.RESET_ALL}")
        elif sub == "2":
            sim_state.conveyor.clear_all_boxes()
            print(f"{Fore.YELLOW}[✓] Cleared all boxes. Conveyor is now empty.{Style.RESET_ALL}")

    elif cmd == "7":
        print(f"\nCurrent PID Gains: Kp={sim_state.pid_gains['kp']}, Ki={sim_state.pid_gains['ki']}, Kd={sim_state.pid_gains['kd']}")
        try:
            kp_str = input(f"Enter Kp [{sim_state.pid_gains['kp']}]: ").strip()
            ki_str = input(f"Enter Ki [{sim_state.pid_gains['ki']}]: ").strip()
            kd_str = input(f"Enter Kd [{sim_state.pid_gains['kd']}]: ").strip()

            kp = float(kp_str) if kp_str else sim_state.pid_gains['kp']
            ki = float(ki_str) if ki_str else sim_state.pid_gains['ki']
            kd = float(kd_str) if kd_str else sim_state.pid_gains['kd']

            sim_state.set_pid_gains(kp, ki, kd)
            print(f"{Fore.GREEN}[✓] Updated PID Gains: Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.3f}{Style.RESET_ALL}")
        except ValueError:
            print(f"{Fore.RED}[X] Invalid numbers entered.{Style.RESET_ALL}")

    elif cmd in ("8", "c"):
        print(f"\n{Fore.CYAN}[i] Starting automated 3-controller comparison benchmark & scorecard...{Style.RESET_ALL}")
        sim_state.run_benchmark_comparison()

    elif cmd == "9":
        print(f"\n{Fore.CYAN}[i] Generating live session telemetry plot...{Style.RESET_ALL}")
        sim_state.generate_telemetry_plot()

    elif cmd == "p":
        sim_state.paused = not sim_state.paused
        state_str = "PAUSED" if sim_state.paused else "RESUMED"
        print(f"{Fore.YELLOW}[✓] Simulation {state_str}.{Style.RESET_ALL}")

    elif cmd == "r":
        sim_state.reset()
        print(f"{Fore.GREEN}[✓] Simulation, motor, and cargo boxes reset to default.{Style.RESET_ALL}")

    elif cmd == "h":
        print_status_bar(sim_state)

    else:
        print(f"{Fore.RED}[X] Unknown command '{choice}'. Type 1-9, P, R, H, or Q.{Style.RESET_ALL}")

    return True

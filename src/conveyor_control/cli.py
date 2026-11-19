"""Command-line experiment dashboard."""

import argparse
from pathlib import Path
import sys

from .experiments import make_controller, run_suite, run_pwm_frequency_sweep
from .simulation import SimulationConfig, run_simulation
from .metrics import calculate_metrics
from .results import (save_experiment, plot_scenario, save_suite_summary, save_report,
                      save_frequency_response, save_spectrum, plot_spectrum_comparison,
                      plot_pwm_waveform, save_pwm_sweep, plot_metrics_comparison,
                      plot_load_robustness, interpretation)
from .frequency import motor_frequency_response


def _show(rows, metrics, label):
    try:
        from rich.console import Console
        from rich.table import Table
        table = Table(title=f"{label} results")
        table.add_column("Metric")
        table.add_column("Value", justify="right")
        for key, value in metrics.items():
            table.add_row(key, f"{value:.5g}" if value == value else "not reached")
        Console().print(table)
    except ImportError:
        print(label, metrics)


def main(argv: list[str] | None = None) -> int:
    if argv is None:
        argv = sys.argv[1:]
    if not argv:
        try:
            from rich.console import Console
            from rich.panel import Panel
            from rich.table import Table
            console = Console()
            menu = Table.grid(padding=(0, 2))
            menu.add_column(style="bold cyan", justify="right")
            menu.add_column()
            menu.add_row("1", "Run one controller and scenario")
            menu.add_row("2", "Compare all controllers across all scenarios")
            menu.add_row("3", "Generate the frequency response")
            menu.add_row("4", "Test several PWM carrier frequencies")
            menu.add_row("5", "Run a PyBullet visual replay")
            menu.add_row("6", "Exit")
            console.print(Panel(menu, title="[bold]P15 - Conveyor Motor Speed Control[/bold]",
                                subtitle="Bang-Bang | PWM | PID", border_style="cyan", expand=False))
        except ImportError:
            print("P15 - Conveyor Motor Speed Control")
            print("1) Run  2) Compare  3) Frequency response  4) PWM frequency test  5) PyBullet replay  6) Exit")
        choice = input("Select [1-6]: ").strip()
        if choice == "2":
            argv = ["--compare"]
        elif choice == "1":
            selected = input("Controller [bang-bang/pwm/pid] (pid): ").strip().lower() or "pid"
            scenario = input("Scenario [baseline/load_step/load_cycle/light_load/medium_load/heavy_load/saturation/changing_reference] (baseline): ").strip() or "baseline"
            argv = ["--controller", selected, "--scenario", scenario]
            if selected == "pwm":
                argv.extend(["--pwm-frequency", input("PWM frequency Hz (20): ").strip() or "20"])
            if selected == "bang-bang":
                argv.extend(["--deadband", input("Speed deadband m/s (.12): ").strip() or ".12"])
            if selected == "pid":
                for name, default in (("kp", ".7"), ("ki", ".8"), ("kd", ".01")):
                    argv.extend([f"--{name}", input(f"{name.upper()} ({default}): ").strip() or default])
            if scenario == "load_step":
                argv.extend(["--load-torque", input("Load torque N m (.16): ").strip() or ".16"])
        elif choice == "3":
            argv = ["--frequency"]
        elif choice == "4":
            argv = ["--pwm-sweep"]
        elif choice == "5":
            selected = input("Controller [bang-bang/pwm/pid] (pid): ").strip().lower() or "pid"
            scenario = input("Scenario [baseline/load_step/load_cycle/light_load/medium_load/heavy_load/saturation/changing_reference] (baseline): ").strip() or "baseline"
            argv = ["--controller", selected, "--scenario", scenario, "--visualize"]
        elif choice == "6":
            return 0
        else:
            print("Choose a menu option from 1 to 6.", file=sys.stderr)
            return 2
    parser = argparse.ArgumentParser(description="Conveyor motor speed control experiments")
    parser.add_argument("--controller", choices=["bang-bang", "pwm", "pid"], default="pid")
    parser.add_argument("--scenario", choices=["baseline", "load_step", "load_cycle", "light_load", "medium_load", "heavy_load", "saturation", "changing_reference"], default="baseline")
    parser.add_argument("--compare", action="store_true", help="run all controllers and scenarios")
    parser.add_argument("--pwm-sweep", action="store_true", help="compare speed ripple at several PWM carrier rates")
    parser.add_argument("--frequency", action="store_true", help="generate analytical plant frequency response")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--dt", type=float, default=.005)
    parser.add_argument("--target", type=float, default=1.8, help="belt target speed in m/s")
    parser.add_argument("--load-torque", type=float, default=.16, help="load-step torque in N m")
    parser.add_argument("--pwm-frequency", type=float, default=20.0, help="PWM carrier frequency in Hz")
    parser.add_argument("--deadband", type=float, default=.12, help="Bang-Bang speed deadband in m/s")
    parser.add_argument("--actuator-min", type=float, default=0.0, help="normalized actuator lower limit")
    parser.add_argument("--actuator-max", type=float, default=1.0, help="normalized actuator upper limit")
    parser.add_argument("--kp", type=float, default=.7)
    parser.add_argument("--ki", type=float, default=.8)
    parser.add_argument("--kd", type=float, default=.01)
    parser.add_argument("--visualize", action="store_true", help="replay run in PyBullet GUI")
    parser.add_argument("--results", type=Path, default=Path("results"))
    args = parser.parse_args(argv)
    try:
        cfg = SimulationConfig(dt=args.dt, duration=args.duration, target_speed=args.target,
                               load_torque=args.load_torque, pwm_frequency_hz=args.pwm_frequency,
                               actuator_min=args.actuator_min, actuator_max=args.actuator_max,
                               bangbang_deadband=args.deadband)
        if args.frequency:
            path = save_frequency_response(motor_frequency_response(cfg.motor), args.results)
            print(f"Saved analytical plant Bode plot to {path}")
            return 0
        if args.pwm_sweep:
            sweep = run_pwm_frequency_sweep(cfg)
            save_pwm_sweep(sweep, args.results)
            for frequency, item in sweep.items():
                save_experiment(f"pwm_frequency_{frequency:g}hz", "pwm", item["rows"], item["metrics"], args.results)
                save_spectrum(f"pwm_{frequency:g}hz", "pwm", item["spectrum"], args.results)
            print("PWM frequency sweep (actual simulated data):")
            for frequency, item in sweep.items():
                print(f"  {frequency:g} Hz: speed ripple RMS={item['metrics']['speed_ripple_rms_m_s']:.5g} m/s, "
                      f"dominant={item['metrics']['dominant_ripple_frequency_hz']:.4g} Hz")
            print(f"Saved sweep graph and table to {args.results.resolve()}")
            return 0
        if args.compare:
            suite = run_suite(cfg)
            for scenario, controllers in suite.items():
                for name, result in controllers.items():
                    save_experiment(scenario, name, result["rows"], result["metrics"], args.results)
                    if scenario == "baseline":
                        save_spectrum(scenario, name, result["spectrum"], args.results)
                path = plot_scenario(scenario, controllers, args.results)
                print(f"\n{scenario}: {path}")
                print(interpretation(controllers))
            save_suite_summary(suite, args.results)
            sweep = run_pwm_frequency_sweep(cfg)
            save_report(suite, cfg, args.results, pwm_sweep=sweep)
            bode_path = save_frequency_response(motor_frequency_response(cfg.motor), args.results)
            spectrum_path = plot_spectrum_comparison(suite["baseline"], root=args.results)
            metrics_path = plot_metrics_comparison(suite, args.results)
            robustness_path = plot_load_robustness(suite, args.results)
            plot_pwm_waveform(suite["baseline"]["pwm"]["rows"], args.results)
            save_pwm_sweep(sweep, args.results)
            for frequency, item in sweep.items():
                save_experiment(f"pwm_frequency_{frequency:g}hz", "pwm", item["rows"], item["metrics"], args.results)
                save_spectrum(f"pwm_{frequency:g}hz", "pwm", item["spectrum"], args.results)
            print(f"\nPlant frequency response: {bode_path}")
            print(f"Speed ripple spectrum: {spectrum_path}")
            print(f"Metrics chart: {metrics_path}; load robustness chart: {robustness_path}")
            print(f"PWM frequency sweep: {args.results / 'plots' / 'pwm_frequency_sweep.png'}")
            print(f"\nSaved data and plots to {args.results.resolve()}")
        else:
            controller = make_controller(args.controller, args.kp, args.ki, args.kd,
                                         pwm_frequency_hz=args.pwm_frequency,
                                         limits=(args.actuator_min, args.actuator_max),
                                         deadband=args.deadband)
            rows = run_simulation(controller, args.scenario, cfg)
            metrics = calculate_metrics(rows)
            save_experiment(args.scenario, args.controller, rows, metrics, args.results)
            _show(rows, metrics, f"{args.scenario} / {args.controller}")
            if args.visualize:
                from .visualization import replay_pybullet
                replay_pybullet(rows)
            if args.controller == "pwm":
                plot_pwm_waveform(rows, args.results)
            print(f"Saved CSV and metrics to {args.results.resolve()}")
    except (ValueError, RuntimeError, OSError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

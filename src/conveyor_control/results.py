"""CSV, plot, summary and evidence-based interpretation output."""

import csv
import json
import math
from pathlib import Path
from typing import Any


def save_experiment(name: str, controller: str, rows: list[dict[str, float]], metrics: dict[str, float], root: Path = Path("results")) -> None:
    csv_dir, plot_dir = root / "csv", root / "plots"
    csv_dir.mkdir(parents=True, exist_ok=True)
    plot_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{name}_{controller}"
    with (csv_dir / f"{stem}.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with (root / f"{stem}_metrics.json").open("w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, allow_nan=True)


def plot_scenario(name: str, results: dict[str, dict[str, Any]], root: Path = Path("results")) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plot_dir = root / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(6, 1, figsize=(10, 15), sharex=True, constrained_layout=True)
    fig.suptitle(f"{name.replace('_', ' ').title()} — controller comparison", fontsize=14, fontweight="bold")
    for controller, result in results.items():
        rows = result["rows"]
        t = [r["time_s"] for r in rows]
        axes[0].plot(t, [r["speed_m_s"] for r in rows], label=controller)
        axes[1].plot(t, [r["error_m_s"] for r in rows], label=controller, alpha=.85)
        axes[2].plot(t, [r["raw_command"] for r in rows], linestyle="--", alpha=.55, label=f"{controller} raw")
        axes[2].plot(t, [r["command"] for r in rows], label=f"{controller} applied", alpha=.9)
        axes[3].plot(t, [r["duty"] for r in rows], label=controller, alpha=.85)
        axes[4].step(t, [r["saturated"] for r in rows], where="post", label=f"{controller} saturated", alpha=.8)
    first = next(iter(results.values()))["rows"]
    axes[0].plot([r["time_s"] for r in first], [r["target_m_s"] for r in first], "k--", label="target")
    axes[0].set_ylabel("Belt speed (m/s)")
    axes[1].set_ylabel("Speed error (m/s)")
    axes[2].set_ylabel("Raw / applied command")
    axes[3].set_ylabel("Duty / command fraction")
    axes[4].set_ylabel("Saturated (0/1)")
    axes[4].set_ylim(-.05, 1.15)
    axes[4].legend(loc="upper left", ncol=3)
    axes[5].step([r["time_s"] for r in first], [r.get("payload_count", 0) for r in first],
                 where="post", color="#79539a", label="active boxes")
    axes[5].set_ylabel("Payload boxes")
    axes[5].set_ylim(-.1, 3.5)
    axes[5].set_yticks(range(4))
    axes[5].set_xlabel("Time (s)")
    axes[5].legend(loc="upper left")
    load_axis = axes[5].twinx()
    load_axis.plot([r["time_s"] for r in first], [r["load_torque_Nm"] for r in first], "k--", label="load torque")
    load_axis.set_ylabel("Load torque (N·m)")
    load_axis.legend(loc="upper right")
    axes[0].legend(ncol=4)
    for ax in axes:
        ax.grid(True, alpha=.25)
    path = plot_dir / f"{name}_comparison.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def save_spectrum(name: str, controller: str, spectrum: dict[str, Any], root: Path = Path("results")) -> None:
    csv_dir = root / "csv"
    csv_dir.mkdir(parents=True, exist_ok=True)
    with (csv_dir / f"{name}_{controller}_spectrum.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["frequency_hz", "amplitude_m_s"])
        writer.writeheader()
        writer.writerows(spectrum["rows"])


def plot_spectrum_comparison(results: dict[str, dict[str, Any]], name: str = "baseline",
                             root: Path = Path("results")) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plot_dir = root / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(9, 5), constrained_layout=True)
    for controller, result in results.items():
        spectrum = result["spectrum"]["rows"]
        ax.semilogy([r["frequency_hz"] for r in spectrum[1:]],
                    [max(r["amplitude_m_s"], 1e-12) for r in spectrum[1:]], label=controller)
    ax.set_title(f"{name.replace('_', ' ').title()} steady-tail speed spectrum")
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Speed amplitude (m/s)")
    ax.grid(True, which="both", alpha=.3)
    ax.legend()
    path = plot_dir / "frequency_spectrum.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def plot_pwm_waveform(rows: list[dict[str, float | str]], root: Path = Path("results"), window_s: float = .5) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plot_dir = root / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    end = float(rows[-1]["time_s"])
    start = max(float(rows[0]["time_s"]), end - window_s)
    selected = [row for row in rows if float(row["time_s"]) >= start]
    fig, ax = plt.subplots(figsize=(10, 4), constrained_layout=True)
    times = [float(row["time_s"]) for row in selected]
    ax.step(times, [float(row["pwm_on"]) for row in selected], where="post", label="PWM ON/OFF")
    ax.plot(times, [float(row["duty"]) for row in selected], label="duty command", linewidth=1.5)
    frequency = float(selected[0].get("pwm_frequency_hz", 0.0))
    ax.set_title(f"PWM carrier waveform ({frequency:g} Hz), final {end-start:.2f} s")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Normalized actuator")
    ax.set_ylim(-.08, 1.08)
    ax.grid(True, alpha=.3)
    ax.legend()
    path = plot_dir / "pwm_waveform.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def save_pwm_sweep(sweep: dict[float, dict[str, Any]], root: Path = Path("results")) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    csv_dir, plot_dir = root / "csv", root / "plots"
    csv_dir.mkdir(parents=True, exist_ok=True)
    plot_dir.mkdir(parents=True, exist_ok=True)
    records = [{"pwm_frequency_hz": frequency, **item["metrics"]} for frequency, item in sweep.items()]
    with (csv_dir / "pwm_frequency_sweep.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    fig, ax = plt.subplots(figsize=(8, 5), constrained_layout=True)
    ax.plot([r["pwm_frequency_hz"] for r in records], [r["speed_ripple_peak_to_peak_m_s"] for r in records], "o-")
    ax.set_title("PWM carrier frequency vs belt-speed ripple")
    ax.set_xlabel("PWM frequency (Hz)")
    ax.set_ylabel("Peak-to-peak speed ripple (m/s)")
    ax.grid(True, alpha=.3)
    fig.savefig(plot_dir / "pwm_frequency_sweep.png", dpi=160)
    plt.close(fig)


def plot_metrics_comparison(suite: dict[str, dict[str, Any]], root: Path = Path("results")) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    controllers = list(suite["baseline"])
    measures = (("RMSE_m_s", "RMSE (m/s)"), ("IAE_m_s_s", "IAE (m/s·s)"),
                ("rise_time_s", "Rise time (s)"), ("settling_time_s", "Settling (s)"),
                ("control_effort", "Control effort"), ("saturation_percent", "Saturation (%)"))
    fig, axes = plt.subplots(2, 3, figsize=(12, 7), constrained_layout=True)
    for ax, (key, title) in zip(axes.flat, measures):
        vals = [float(suite["baseline"][name]["metrics"][key]) for name in controllers]
        heights = [0.0 if not math.isfinite(value) else value for value in vals]
        bars = ax.bar(controllers, heights, color=["#4778a8", "#e69545", "#4d9a6a"])
        pad = max((max(heights) * .03), .02)
        for bar, value in zip(bars, vals):
            if not math.isfinite(value):
                ax.text(bar.get_x() + bar.get_width() / 2, pad, "n/a", ha="center", va="bottom", fontsize=8)
        ax.set_title(title)
        ax.grid(axis="y", alpha=.25)
        ax.tick_params(axis="x", rotation=15)
    fig.suptitle("Baseline performance metrics", fontsize=14, fontweight="bold")
    path = root / "plots" / "metrics_comparison.png"
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def plot_load_robustness(suite: dict[str, dict[str, Any]], root: Path = Path("results")) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    scenarios = ["light_load", "medium_load", "heavy_load"]
    controllers = list(suite[scenarios[0]])
    fig, ax = plt.subplots(figsize=(8, 5), constrained_layout=True)
    loads = [float(suite[scenario][controllers[0]]["rows"][0]["load_torque_Nm"]) for scenario in scenarios]
    for controller in controllers:
        ax.plot(loads, [suite[s][controller]["metrics"]["MAE_m_s"] for s in scenarios],
                marker="o", label=controller)
    ax.set_title("Tracking error under increasing steady load")
    ax.set_xlabel("Load torque (N·m)")
    ax.set_ylabel("Mean absolute error (m/s)")
    ax.grid(True, alpha=.3)
    ax.legend()
    path = root / "plots" / "load_robustness.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def interpretation(results: dict[str, dict[str, Any]]) -> str:
    """Describe observed metrics without encoding a preferred controller ranking."""
    lines = ["Evidence-based comparison (computed from these run files):"]
    for label, metric, fmt in (("MAE", "MAE_m_s", ".4f"), ("RMSE", "RMSE_m_s", ".4f"),
                               ("rise time s", "rise_time_s", ".3f"),
                               ("overshoot %", "overshoot_percent", ".2f"),
                               ("settling s", "settling_time_s", ".3f"),
                               ("load recovery s", "recovery_time_after_load_s", ".3f"),
                               ("effort", "control_effort", ".4f"),
                               ("saturation %", "saturation_percent", ".2f"),
                               ("speed ripple p-p m/s", "speed_ripple_peak_to_peak_m_s", ".5f"),
                               ("dominant ripple Hz", "dominant_ripple_frequency_hz", ".3f")):
        vals = [(name, float(result["metrics"][metric])) for name, result in results.items()]
        finite = [(name, value) for name, value in vals if math.isfinite(value)]
        if not finite:
            lines.append(f"- {label}: no finite result")
            continue
        best = min(finite, key=lambda pair: pair[1])
        formatted = ", ".join(f"{name}={'n/a' if not math.isfinite(value) else format(value, fmt)}" for name, value in vals)
        lines.append(f"- {label}: {formatted}; lowest observed: {best[0]} ({best[1]:{fmt}})")
    return "\n".join(lines)


def save_suite_summary(suite: dict[str, dict[str, Any]], root: Path = Path("results")) -> None:
    root.mkdir(parents=True, exist_ok=True)
    records = []
    for scenario, controllers in suite.items():
        for name, result in controllers.items():
            records.append({"scenario": scenario, "controller": name, **result["metrics"]})
    if records:
        with (root / "metrics.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)
    text = []
    for scenario, controllers in suite.items():
        text.extend([f"## {scenario}", interpretation(controllers), ""])
    (root / "interpretation.md").write_text("\n".join(text), encoding="utf-8")


def save_report(suite: dict[str, dict[str, Any]], config, root: Path = Path("results"),
                pwm_sweep: dict[float, dict[str, Any]] | None = None) -> None:
    """Write report-ready settings and measured metric tables from this suite."""
    root.mkdir(parents=True, exist_ok=True)
    lines = ["# Simulation results report", "", "Generated from actual simulation samples.", "",
             "## Settings", "", "| Parameter | Value |", "|---|---:|",
             f"| Time step | {config.dt:g} s |", f"| Duration | {config.duration:g} s |",
             f"| Nominal target | {config.target_speed:g} m/s |", f"| Torque per payload box | {config.load_torque:g} N m |",
             f"| PWM carrier | {config.pwm_frequency_hz:g} Hz |", f"| Random seed | {config.seed} |",
             f"| Actuator range | [{config.actuator_min:g}, {config.actuator_max:g}] |",
             f"| Bang-Bang deadband | {config.bangbang_deadband:g} m/s |",
             f"| Inertia | {config.motor.inertia:g} kg m² |", f"| Torque coefficient | {config.motor.torque_constant:g} N m/unit |",
             f"| Viscous friction | {config.motor.viscous_friction:g} N m s/rad |",
             f"| Roller radius | {config.motor.roller_radius:g} m |", "", "## Metrics by scenario", ""]
    metric_names = list(next(iter(next(iter(suite.values())).values()))["metrics"])
    descriptions = {"baseline": "Constant nominal target with two active payload boxes.",
                    "load_step": "Increases from one to three active boxes at 40% of the run.",
                    "load_cycle": "Increases from one to three boxes at 35%, then removes one at 70%.",
                    "light_load": "One active payload box.",
                    "medium_load": "Two active payload boxes.",
                    "heavy_load": "Three active payload boxes.",
                    "saturation": "Requests 115% of the configured maximum belt speed.",
                    "changing_reference": "Target steps through 55%, 100%, then 70% of nominal."}
    for scenario, controllers in suite.items():
        lines.extend([f"### {scenario}", "", descriptions.get(scenario, "See experiment configuration."), "",
                      "| Controller | " + " | ".join(metric_names) + " |",
                      "|---|" + "---:|" * len(metric_names)])
        for controller, result in controllers.items():
            values = ["n/a" if not math.isfinite(float(result["metrics"][key])) else f"{result['metrics'][key]:.5g}" for key in metric_names]
            lines.append(f"| {controller} | " + " | ".join(values) + " |")
        lines.extend(["", f"Graph: `plots/{scenario}_comparison.png`", ""])
    from .stability import analyze_stability
    stability = analyze_stability(config.motor)
    lines.extend(["## Linear stability check", "",
                  f"The reduced plant pole is {stability.plant_pole_rad_s:.5g} rad/s (stable: {stability.plant_stable}).",
                  "The ideal continuous PID linearization has poles " + ", ".join(f"{pole.real:.5g}{pole.imag:+.5g}j" for pole in stability.closed_loop_poles_rad_s) +
                  f" rad/s (stable: {stability.pid_closed_loop_stable}). " + stability.assumptions,
                  "", "## Frequency response", "", "The plant and ideal unsaturated PID closed-loop Bode traces are in `plots/frequency_response.png`; samples are in `csv/frequency_response.csv`. Baseline speed FFT is in `plots/frequency_spectrum.png`; PWM carrier test is in `plots/pwm_frequency_sweep.png`."])
    if pwm_sweep:
        lines.extend(["", "## PWM carrier-frequency sweep", "",
                      "| Carrier (Hz) | Speed ripple RMS (m/s) | Speed ripple p-p (m/s) | Dominant ripple (Hz) |",
                      "|---:|---:|---:|---:|"])
        for frequency, item in pwm_sweep.items():
            m = item["metrics"]
            lines.append(f"| {frequency:g} | {m['speed_ripple_rms_m_s']:.6g} | {m['speed_ripple_peak_to_peak_m_s']:.6g} | {m['dominant_ripple_frequency_hz']:.5g} |")
        best = min(pwm_sweep, key=lambda f: pwm_sweep[f]["metrics"]["speed_ripple_peak_to_peak_m_s"])
        lines.append(f"Lowest measured peak-to-peak ripple in this sweep: {best:g} Hz.")
    lines.extend(["", "## Interpretation", "", (root / "interpretation.md").read_text(encoding="utf-8"), "",
                  "## Limits", "", "The plant is reduced-order. PyBullet is a synchronized visualization, not coupled co-simulation. Frequency response describes the linearized plant and is not a nonlinear closed-loop stability proof."])
    (root / "report.md").write_text("\n".join(lines), encoding="utf-8")


def save_frequency_response(samples: list[dict[str, float]], root: Path = Path("results")) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    csv_dir, plot_dir = root / "csv", root / "plots"
    csv_dir.mkdir(parents=True, exist_ok=True)
    plot_dir.mkdir(parents=True, exist_ok=True)
    with (csv_dir / "frequency_response.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(samples[0]))
        writer.writeheader()
        writer.writerows(samples)
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True, constrained_layout=True)
    frequency = [row["frequency_rad_s"] for row in samples]
    axes[0].semilogx(frequency, [row["magnitude_db"] for row in samples], label="motor plant")
    axes[0].semilogx(frequency, [row["pid_closed_loop_magnitude_db"] for row in samples], label="ideal PID closed loop")
    axes[1].semilogx(frequency, [row["phase_deg"] for row in samples], label="motor plant")
    axes[1].semilogx(frequency, [row["pid_closed_loop_phase_deg"] for row in samples], label="ideal PID closed loop")
    axes[0].set_ylabel("Magnitude (dB)")
    axes[1].set_ylabel("Phase (degrees)")
    axes[1].set_xlabel("Angular frequency (rad/s)")
    for ax in axes:
        ax.grid(True, which="both", alpha=.3)
        ax.legend()
    path = plot_dir / "frequency_response.png"
    fig.suptitle("Motor plant and ideal PID frequency response", fontsize=14, fontweight="bold")
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path

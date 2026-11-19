"""
evaluation/plots.py
==============================================================================
High-Quality Scientific Matplotlib Visualization for Conveyor Motor Control.

Features:
  - Telemetry Dashboard: 4-panel multi-variable live analysis
  - Comparison Benchmark: Side-by-side Bang-Bang vs PWM vs PID evaluation
  - Clean academic styling, dual units, disturbance markers, saturation limits
==============================================================================
"""

import os
import matplotlib
# Force headless Agg backend to ensure 100% thread safety without Tkinter GUI conflicts
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from typing import Dict, Any, Optional

plt.rcParams["font.sans-serif"] = "DejaVu Sans"
plt.rcParams["axes.edgecolor"] = "#cccccc"
plt.rcParams["axes.linewidth"] = 0.8


def _safe_savefig(fig, save_path: str, dpi: int = 160) -> str:
    """
    Save figure safely as high-quality JPG (quality=95) with solid white background,
    retrying with alternative names if locked by another application.
    Also produces a companion PNG for full compatibility.
    """
    directory = os.path.dirname(save_path) or "."
    base_name = os.path.basename(save_path)
    stem, ext = os.path.splitext(base_name)

    # Force solid white facecolor so JPEG has no dark/transparent artifacts
    facecolor = "#ffffff"
    try:
        fig.patch.set_facecolor(facecolor)
    except Exception:
        pass

    jpg_target = os.path.join(directory, f"{stem}.jpg")
    png_target = os.path.join(directory, f"{stem}.png")

    saved_jpg = ""
    # 1. Save as high-quality JPG
    for attempt in range(5):
        t = jpg_target if attempt == 0 else os.path.join(directory, f"{stem}_{attempt}.jpg")
        try:
            fig.savefig(t, dpi=dpi, bbox_inches="tight", facecolor=facecolor, edgecolor="none", pil_kwargs={"quality": 95})
            saved_jpg = os.path.abspath(t)
            break
        except (OSError, PermissionError):
            continue

    # 2. Also save PNG copy for backup
    for attempt in range(5):
        t = png_target if attempt == 0 else os.path.join(directory, f"{stem}_{attempt}.png")
        try:
            fig.savefig(t, dpi=dpi, bbox_inches="tight", facecolor=facecolor, edgecolor="none")
            break
        except (OSError, PermissionError):
            continue

    if saved_jpg:
        return saved_jpg
    return os.path.abspath(png_target)


def plot_live_telemetry(history: Dict[str, list], save_path: Optional[str] = None) -> str:
    """
    Generate a 4-panel telemetry dashboard from simulation session history.
    """
    t = np.array(history.get("time", []))
    v_act = np.array(history.get("speed", []))
    v_tgt = np.array(history.get("target_speed", []))
    volts = np.array(history.get("voltage", []))
    currs = np.array(history.get("current", []))
    loads = np.array(history.get("load_torque", []))
    cname = history.get("controller", ["PID"])[-1] if history.get("controller") else "PID"

    if len(t) < 2:
        return ""

    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    fig.patch.set_facecolor("#f8f9fa")

    # 1. Belt Speed
    ax = axes[0]
    ax.plot(t, v_act, color="#0066cc", lw=1.8, label="Actual Belt Speed v(t)")
    ax.plot(t, v_tgt, color="#d9534f", lw=1.5, ls="--", label="Target Speed v_ref")
    ax.set_ylabel("Speed (m/s)", fontweight="bold")
    ax.set_title(f"Conveyor Motor Telemetry Dashboard — Controller: {cname.upper()}", fontsize=13, fontweight="bold", pad=10)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="upper right", framealpha=0.9)

    # 2. Motor Voltage
    ax = axes[1]
    ax.plot(t, volts, color="#2e7d32", lw=1.6, label="Applied Voltage V(t)")
    ax.axhline(24.0, color="#b71c1c", ls=":", lw=1.2, label="+24V Saturation Limit")
    ax.axhline(-24.0, color="#b71c1c", ls=":", lw=1.2, label="-24V Saturation Limit")
    ax.set_ylabel("Voltage (V)", fontweight="bold")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="upper right", framealpha=0.9)

    # 3. Armature Current
    ax = axes[2]
    ax.plot(t, currs, color="#e65100", lw=1.4, label="Armature Current i_a(t)")
    ax.set_ylabel("Current (A)", fontweight="bold")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="upper right", framealpha=0.9)

    # 4. Load Torque
    ax = axes[3]
    ax.plot(t, loads, color="#6a1b9a", lw=1.6, label="Payload Friction Torque T_load(t)")
    ax.set_ylabel("Torque (N·m)", fontweight="bold")
    ax.set_xlabel("Time (seconds)", fontweight="bold")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="upper right", framealpha=0.9)

    plt.tight_layout()

    if save_path is None:
        save_path = os.path.abspath("conveyor_telemetry.jpg")
    final_path = _safe_savefig(fig, save_path, dpi=160)
    plt.close(fig)
    return final_path


def plot_comparison_benchmark(results: Dict[str, Dict[str, Any]], save_path: Optional[str] = None) -> str:
    """
    Plot side-by-side benchmark comparison for Bang-Bang, PWM, and PID.
    """
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.patch.set_facecolor("#ffffff")

    colors = {
        "bang": "#d32f2f",      # Red
        "pwm":  "#f57c00",      # Orange
        "pid":  "#1976d2",      # Blue
    }
    labels = {
        "bang": "Bang-Bang (Hysteresis)",
        "pwm":  "PWM (Duty Cycle Average)",
        "pid":  "PID (Tuned with Anti-Windup)",
    }

    t = None
    v_ref = None

    # Subplot 1: Speed Response
    ax_spd = axes[0]
    for key in ["bang", "pwm", "pid"]:
        if key in results:
            data = results[key]
            t = np.array(data["time"])
            v_ref = np.array(data["target_speed"])
            ax_spd.plot(t, data["speed"], color=colors[key], lw=1.8, label=labels[key])

    if v_ref is not None and t is not None:
        ax_spd.plot(t, v_ref, "k--", lw=1.5, label="Target Speed (1.2 m/s)")
        # Disturbance markers
        ax_spd.axvline(4.0, color="#666666", ls=":", lw=1.2)
        ax_spd.text(4.05, 0.45, "+3.0 kg Med Box Drop", color="#000000", fontsize=9, fontweight="bold")
        ax_spd.axvline(8.0, color="#666666", ls=":", lw=1.2)
        ax_spd.text(8.05, 0.45, "+8.0 kg Heavy Box Drop", color="#000000", fontsize=9, fontweight="bold")

    ax_spd.set_ylabel("Belt Speed (m/s)", fontsize=11, fontweight="bold")
    ax_spd.set_title("Conveyor Motor Speed Control Benchmark — Bang-Bang vs PWM vs PID", fontsize=13, fontweight="bold", pad=10)
    ax_spd.grid(True, linestyle=":", alpha=0.6)
    ax_spd.legend(loc="upper right", framealpha=0.92, fontsize=9.5)

    # Subplot 2: Applied Voltage
    ax_v = axes[1]
    for key in ["bang", "pwm", "pid"]:
        if key in results:
            data = results[key]
            ax_v.plot(t, data["voltage"], color=colors[key], lw=1.4, alpha=0.85, label=labels[key])
    ax_v.axhline(24.0, color="#990000", ls="--", lw=1.0, label="+24V Saturation Limit")
    ax_v.axhline(-24.0, color="#990000", ls="--", lw=1.0)
    ax_v.set_ylabel("Motor Voltage (V)", fontsize=11, fontweight="bold")
    ax_v.grid(True, linestyle=":", alpha=0.6)
    ax_v.legend(loc="upper right", framealpha=0.92, fontsize=9.5)

    # Subplot 3: Speed Tracking Error
    ax_err = axes[2]
    for key in ["bang", "pwm", "pid"]:
        if key in results:
            data = results[key]
            err = np.array(data["target_speed"]) - np.array(data["speed"])
            ax_err.plot(t, err, color=colors[key], lw=1.5, label=f"Error {labels[key]}")
    ax_err.axhline(0.0, color="black", lw=0.9)
    ax_err.set_ylabel("Speed Error e(t) [m/s]", fontsize=11, fontweight="bold")
    ax_err.set_xlabel("Time (seconds)", fontsize=11, fontweight="bold")
    ax_err.grid(True, linestyle=":", alpha=0.6)
    ax_err.legend(loc="upper right", framealpha=0.92, fontsize=9.5)

    plt.tight_layout()

    if save_path is None:
        save_path = os.path.abspath("comparison_benchmark.jpg")
    final_path = _safe_savefig(fig, save_path, dpi=160)
    plt.close(fig)
    return final_path


def plot_session_controllers_comparison(
    results: Dict[str, Dict[str, Any]],
    metrics: Optional[Dict[str, Dict[str, Any]]] = None,
    save_path: Optional[str] = None,
) -> str:
    """
    Generate a 4-panel multi-controller comparison plot evaluating Bang-Bang,
    PWM, and PID under identical step commands and cargo disturbances in one session.

    Panels:
      1. Speed Response v(t) with Setpoint & Disturbance Annotations
      2. Armature Voltage V(t) with Actuator Saturation Ceilings
      3. Speed Tracking Error e(t) = v_ref - v_act
      4. Quantitative Performance Scorecard (Grouped Bar Chart)
    """
    fig = plt.figure(figsize=(13, 11), facecolor="#ffffff")
    gs = fig.add_gridspec(4, 1, height_ratios=[1.25, 0.95, 0.95, 1.15], hspace=0.36)

    colors = {
        "bang": "#d32f2f",  # Crimson Red
        "pwm":  "#e65100",  # Vivid Orange
        "pid":  "#0d47a1",  # Deep Royal Blue
    }
    labels = {
        "bang": "Bang-Bang (Relay On/Off)",
        "pwm":  "PWM (Duty-Cycle Average)",
        "pid":  "PID (Tuned Anti-Windup)",
    }

    t = None
    v_ref = None

    # Panel 1: Speed Response
    ax1 = fig.add_subplot(gs[0])
    for key in ["bang", "pwm", "pid"]:
        if key in results:
            d = results[key]
            t = np.array(d["time"])
            v_ref = np.array(d["target_speed"])
            ax1.plot(t, d["speed"], color=colors[key], lw=1.8, label=labels[key])

    if v_ref is not None and t is not None:
        ax1.plot(t, v_ref, "k--", lw=1.6, label="Target Speed (1.20 m/s)")
        ax1.axvline(4.0, color="#555555", ls=":", lw=1.2)
        ax1.text(4.08, 0.38, "Disturbance 1: +3.0 kg Med Box", color="#000000", fontsize=9, fontweight="bold")
        ax1.axvline(8.0, color="#555555", ls=":", lw=1.2)
        ax1.text(8.08, 0.38, "Disturbance 2: +8.0 kg Heavy Box", color="#000000", fontsize=9, fontweight="bold")

    ax1.set_ylabel("Belt Speed (m/s)", fontsize=11, fontweight="bold")
    ax1.set_title("Multi-Controller Session Benchmark — Bang-Bang vs PWM vs PID Speed Control", fontsize=13, fontweight="bold", pad=10)
    ax1.grid(True, linestyle=":", alpha=0.6)
    ax1.legend(loc="upper right", framealpha=0.92, fontsize=9.5)

    # Panel 2: Armature Voltage
    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    for key in ["bang", "pwm", "pid"]:
        if key in results:
            d = results[key]
            ax2.plot(t, d["voltage"], color=colors[key], lw=1.3, alpha=0.85, label=labels[key])
    ax2.axhline(24.0, color="#b71c1c", ls="--", lw=1.1, label="+24V Actuator Limit")
    ax2.axhline(0.0, color="#424242", ls=":", lw=0.9)
    ax2.set_ylabel("Armature Voltage (V)", fontsize=11, fontweight="bold")
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.legend(loc="upper right", framealpha=0.92, fontsize=9.5)

    # Panel 3: Speed Tracking Error
    ax3 = fig.add_subplot(gs[2], sharex=ax1)
    for key in ["bang", "pwm", "pid"]:
        if key in results:
            d = results[key]
            err = np.array(d["target_speed"]) - np.array(d["speed"])
            ax3.plot(t, err, color=colors[key], lw=1.4, label=f"Error {labels[key]}")
    ax3.axhline(0.0, color="black", lw=1.0)
    ax3.set_ylabel("Error e(t) [m/s]", fontsize=11, fontweight="bold")
    ax3.set_xlabel("Time (seconds)", fontsize=11, fontweight="bold")
    ax3.grid(True, linestyle=":", alpha=0.6)
    ax3.legend(loc="upper right", framealpha=0.92, fontsize=9.5)

    # Panel 4: Quantitative Engineering Performance Scorecard
    ax4 = fig.add_subplot(gs[3])
    if metrics:
        categories = ["Overshoot\n(% / 2)", "SS Droop\n(cm/s)", "Speed Ripple\n(cm/s)", "Recovery Time\n(seconds)"]
        x = np.arange(len(categories))
        width = 0.25

        ctrl_keys = ["bang", "pwm", "pid"]
        for idx, key in enumerate(ctrl_keys):
            if key in metrics:
                m = metrics[key]
                # Scaled values for intuitive visual comparison on one axis
                rec_t = m.get("recovery_time", m.get("disturbance_recovery_time", 0.0))
                vals = [
                    m.get("overshoot_pct", 0.0) / 2.0,
                    abs(m.get("steady_state_error", 0.0)) * 100.0,
                    m.get("speed_ripple", 0.0) * 100.0,
                    min(8.0, rec_t),
                ]
                rects = ax4.bar(x + (idx - 1) * width, vals, width, label=labels[key], color=colors[key], alpha=0.88, edgecolor="#333333")
                for rect in rects:
                    h = rect.get_height()
                    if h > 0.05:
                        ax4.annotate(f"{h:.1f}",
                                     xy=(rect.get_x() + rect.get_width() / 2, h),
                                     xytext=(0, 3), textcoords="offset points",
                                     ha="center", va="bottom", fontsize=8, fontweight="bold")

        ax4.set_xticks(x)
        ax4.set_xticklabels(categories, fontsize=10, fontweight="bold")
        ax4.set_ylabel("Comparison Score", fontsize=10, fontweight="bold")
        ax4.set_title("Engineering Performance Metrics Scorecard (Lower is Better)", fontsize=11, fontweight="bold")
        ax4.grid(True, linestyle=":", alpha=0.5, axis="y")
        ax4.legend(loc="upper right", framealpha=0.92, fontsize=9)
    else:
        ax4.text(0.5, 0.5, "Benchmark Metrics Recorded in Session", ha="center", va="center", fontsize=11)

    if save_path is None:
        save_path = os.path.abspath("session_3_controllers_comparison.jpg")
    final_path = _safe_savefig(fig, save_path, dpi=160)
    plt.close(fig)
    return final_path

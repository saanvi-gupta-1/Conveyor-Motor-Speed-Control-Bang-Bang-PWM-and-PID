"""
evaluation/metrics.py
==============================================================================
Performance Evaluation Metrics for DC Motor Conveyor Speed Control.

Calculates key control systems metrics:
  - Rise Time (10% to 90% of setpoint) [s]
  - Settling Time (within +/-5% tolerance band) [s]
  - Percentage Overshoot (% OS) [%]
  - Steady-State Error (e_ss) [m/s]
  - Root Mean Square Error (RMSE) [m/s]
  - Speed Ripple / Chatter (Peak-to-peak steady state oscillation) [m/s]
  - Disturbance Speed Dip (Drop when payload box drops) [m/s]
  - Disturbance Recovery Time [s]
==============================================================================
"""

import numpy as np
from typing import Dict, Any


def compute_controller_metrics(
    time_series: np.ndarray,
    speed_series: np.ndarray,
    target_speed: float,
    t_step_start: float = 0.0,
    t_disturb: float = 4.0,
) -> Dict[str, Any]:
    """
    Compute standard control engineering performance metrics.

    Parameters:
      time_series: Array of timestamps [s]
      speed_series: Array of belt speeds [m/s]
      target_speed: Commanded setpoint speed [m/s]
      t_step_start: Time when initial step commanded [s]
      t_disturb: Time when first major payload disturbance occurred [s]

    Returns:
      Dictionary containing all calculated metrics.
    """
    time_arr = np.asarray(time_series, dtype=float)
    spd_arr = np.asarray(speed_series, dtype=float)

    if len(time_arr) < 10 or target_speed <= 0.0:
        return {
            "rise_time": 0.0,
            "settling_time": 0.0,
            "overshoot_pct": 0.0,
            "steady_state_error": 0.0,
            "rmse": 0.0,
            "speed_ripple": 0.0,
            "disturbance_dip": 0.0,
            "recovery_time": 0.0,
        }

    # 1. Step Response Window (from t_step_start to t_disturb)
    step_mask = (time_arr >= t_step_start) & (time_arr < t_disturb)
    t_step = time_arr[step_mask]
    v_step = spd_arr[step_mask]

    # Rise Time: Time from 10% to 90% of setpoint
    v10 = 0.10 * target_speed
    v90 = 0.90 * target_speed
    idx10 = np.where(v_step >= v10)[0]
    idx90 = np.where(v_step >= v90)[0]
    if len(idx10) > 0 and len(idx90) > 0 and idx90[0] >= idx10[0]:
        rise_time = float(t_step[idx90[0]] - t_step[idx10[0]])
    else:
        rise_time = float(t_step[-1] - t_step[0]) if len(t_step) > 0 else 0.0

    # Overshoot %
    max_v = float(np.max(v_step)) if len(v_step) > 0 else 0.0
    overshoot_pct = max(0.0, ((max_v - target_speed) / target_speed) * 100.0)

    # Settling Time (5% band around setpoint)
    band_lower = 0.95 * target_speed
    band_upper = 1.05 * target_speed
    out_of_band = np.where((v_step < band_lower) | (v_step > band_upper))[0]
    if len(out_of_band) == 0:
        settling_time = 0.0
    elif out_of_band[-1] + 1 < len(v_step):
        settling_time = float(t_step[out_of_band[-1] + 1] - t_step_start)
    else:
        # Never settled inside 5% band
        settling_time = float(t_disturb - t_step_start)

    # Steady-State Window (Last 1.0s before disturbance)
    ss_mask = (t_step >= (t_disturb - 1.0)) & (t_step < t_disturb)
    if np.sum(ss_mask) > 5:
        v_ss = v_step[ss_mask]
        mean_ss = float(np.mean(v_ss))
        steady_state_error = float(abs(target_speed - mean_ss))
        speed_ripple = float(np.max(v_ss) - np.min(v_ss))
    else:
        steady_state_error = float(abs(target_speed - v_step[-1])) if len(v_step) > 0 else 0.0
        speed_ripple = 0.0

    # Overall RMSE across the step window
    rmse = float(np.sqrt(np.mean((target_speed - v_step) ** 2))) if len(v_step) > 0 else 0.0

    # 2. Disturbance Analysis (From t_disturb onwards)
    disturb_mask = (time_arr >= t_disturb)
    if np.sum(disturb_mask) > 10:
        t_dist = time_arr[disturb_mask]
        v_dist = spd_arr[disturb_mask]
        min_v_dist = float(np.min(v_dist[:min(len(v_dist), int(1.5 / (time_arr[1] - time_arr[0])))]))
        disturbance_dip = max(0.0, float(target_speed - min_v_dist))

        # Recovery time: time until it re-enters 5% band after dip
        rec_candidates = np.where((t_dist > t_disturb + 0.1) & (v_dist >= band_lower) & (v_dist <= band_upper))[0]
        if len(rec_candidates) > 0:
            recovery_time = float(t_dist[rec_candidates[0]] - t_disturb)
        else:
            recovery_time = float(t_dist[-1] - t_disturb)
    else:
        disturbance_dip = 0.0
        recovery_time = 0.0

    return {
        "rise_time": round(rise_time, 3),
        "settling_time": round(settling_time, 3),
        "overshoot_pct": round(overshoot_pct, 2),
        "steady_state_error": round(steady_state_error, 4),
        "rmse": round(rmse, 4),
        "speed_ripple": round(speed_ripple, 4),
        "disturbance_dip": round(disturbance_dip, 4),
        "recovery_time": round(recovery_time, 3),
    }


def format_metrics_table(results_dict: Dict[str, Dict[str, Any]]) -> str:
    """Format comparison metrics into a clean ASCII table."""
    headers = [
        "Metric",
        "Bang-Bang",
        "PWM",
        "PID (Tuned)",
    ]
    rows = [
        ("Rise Time (s)", "rise_time", "{:.3f}"),
        ("Settling Time (s)", "settling_time", "{:.3f}"),
        ("Overshoot (%)", "overshoot_pct", "{:.1f}%"),
        ("Steady-State Error (m/s)", "steady_state_error", "{:.4f}"),
        ("RMS Error (m/s)", "rmse", "{:.4f}"),
        ("Speed Ripple / Chatter (m/s)", "speed_ripple", "{:.4f}"),
        ("Disturbance Speed Dip (m/s)", "disturbance_dip", "{:.4f}"),
        ("Disturbance Recovery (s)", "recovery_time", "{:.3f}"),
    ]

    col_w = [30, 16, 16, 16]
    line = "+" + "+".join(["-" * w for w in col_w]) + "+"
    hdr_str = "|" + "|".join([h.center(col_w[i]) for i, h in enumerate(headers)]) + "|"

    output = [line, hdr_str, line]

    for label, key, fmt in rows:
        row_vals = [label]
        for cname in ["bang", "pwm", "pid"]:
            if cname in results_dict and key in results_dict[cname]:
                val = results_dict[cname][key]
                row_vals.append(fmt.format(val))
            else:
                row_vals.append("N/A")
        output.append("|" + "|".join([row_vals[i].center(col_w[i]) for i in range(4)]) + "|")

    output.append(line)
    return "\n".join(output)

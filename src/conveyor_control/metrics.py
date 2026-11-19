"""Time-domain performance metrics computed from saved simulation samples."""

import math
from statistics import mean, pstdev
from typing import Sequence


def calculate_metrics(rows: Sequence[dict[str, float | str]], settling_band: float = 0.02) -> dict[str, float]:
    if len(rows) < 2:
        raise ValueError("At least two samples are required")
    dt = float(rows[1]["time_s"]) - float(rows[0]["time_s"])
    if dt <= 0 or settling_band <= 0:
        raise ValueError("Sample interval and settling band must be positive")
    times = [float(r["time_s"]) for r in rows]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("Sample times must be strictly increasing")
    errors = [float(r["error_m_s"]) for r in rows]
    targets = [float(r["target_m_s"]) for r in rows]
    speeds = [float(r["speed_m_s"]) for r in rows]
    intervals = rows[:-1]
    final_target = targets[-1]
    band = max(settling_band * max(abs(final_target), 1e-9), 1e-4)

    settling = math.nan
    for i, row in enumerate(rows):
        if all(abs(float(r["error_m_s"])) <= band for r in rows[i:]):
            settling = times[i]
            break

    start_target = next((target for target in targets if target > 0), 0.0)
    rise_time = math.nan
    if start_target > 0:
        t10 = next((times[i] for i, speed in enumerate(speeds) if speed >= .1 * start_target), math.nan)
        t90 = next((times[i] for i, speed in enumerate(speeds) if speed >= .9 * start_target), math.nan)
        if math.isfinite(t10) and math.isfinite(t90) and t90 >= t10:
            rise_time = t90 - t10

    tail = speeds[max(0, int(len(speeds) * .8)):]
    duty_values = [float(r.get("duty", r.get("command", 0.0))) for r in rows]
    tail_duty = duty_values[max(0, int(len(duty_values) * .8)):]
    saturation_time = sum(float(r.get("saturated", 0.0)) for r in intervals) * dt
    saturation_fraction = saturation_time / max(times[-1] - times[0], dt)

    load_event = next((i for i in range(1, len(rows))
                       if float(rows[i].get("load_torque_Nm", 0.0)) > float(rows[i - 1].get("load_torque_Nm", 0.0)) + 1e-12), None)
    speed_drop = recovery_time = math.nan
    if load_event is not None:
        before_speed = speeds[load_event - 1]
        post_stop = min(len(rows), load_event + max(2, int(.5 / dt)))
        speed_drop = max(0.0, before_speed - min(speeds[load_event:post_stop]))
        recovery_count = max(2, int(.25 / dt))
        excursion = next((i for i in range(load_event, len(rows)) if abs(errors[i]) > band), None)
        if excursion is None:
            recovery_time = 0.0
        for i in range(excursion if excursion is not None else len(rows), len(rows) - recovery_count):
            if all(abs(float(rows[j]["error_m_s"])) <= band for j in range(i, i + recovery_count)):
                recovery_time = times[i] - times[load_event]
                break

    duration = max(times[-1] - times[0], dt)
    return {
        "IAE_m_s_s": sum(abs(float(r["error_m_s"])) for r in intervals) * dt,
        "ISE_m2_s": sum(float(r["error_m_s"]) ** 2 for r in intervals) * dt,
        "MAE_m_s": mean(abs(error) for error in errors),
        "RMSE_m_s": math.sqrt(mean(error * error for error in errors)),
        "maximum_absolute_error_m_s": max(abs(error) for error in errors),
        "steady_state_error_m_s": mean(errors[max(0, int(len(errors) * .9)):]),
        "rise_time_s": rise_time,
        "settling_time_s": settling,
        "overshoot_percent": max(0.0, (max(speeds) - max(targets)) / max(targets) * 100) if max(targets) > 0 else 0.0,
        "peak_speed_m_s": max(speeds),
        "speed_drop_on_load_m_s": speed_drop,
        "recovery_time_after_load_s": recovery_time,
        "speed_ripple_rms_m_s": pstdev(tail),
        "speed_ripple_peak_to_peak_m_s": max(tail) - min(tail),
        "control_effort": sum(float(r["command"]) ** 2 for r in intervals) * dt,
        "mean_pwm_duty": mean(duty_values),
        "pwm_duty_ripple_rms": pstdev(tail_duty),
        "bang_bang_switch_count": max(float(r.get("switch_count", 0.0)) for r in rows),
        "saturation_time_s": saturation_time,
        "saturation_fraction": saturation_fraction,
        "saturation_percent": saturation_fraction * 100.0,
    }

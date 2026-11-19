# Experiment Plan and Evaluation Protocols

This document details the reproducible scenarios executed across Bang-Bang, PWM, and PID controllers under identical motor plant parameters ($J=0.06\text{ kg}\cdot\text{m}^2, K=1.10\text{ N}\cdot\text{m}, b=0.04\text{ N}\cdot\text{m}\cdot\text{s/rad}, r=0.08\text{ m}$).

---

## Experiment Matrix

| Scenario Name | Setpoint $v_{\text{target}}$ | Disturbance Schedule $\tau_{\text{load}}(t)$ | Primary Phenomenon Evaluated |
| :--- | :--- | :--- | :--- |
| **1. Baseline Speed Tracking** | $1.80\text{ m/s}$ (Constant) | Two boxes: $0.12\text{ N}\cdot\text{m}$ | Loaded step response, rise time, settling time, overshoot, and steady-state ripple. |
| **2. Load Step Disturbance** | $1.80\text{ m/s}$ (Constant) | $t < 4.8\text{s}$: one box, $0.06\text{ N}\cdot\text{m}$<br>$t \ge 4.8\text{s}$: three boxes, $0.18\text{ N}\cdot\text{m}$ | Add payload during operation; measure speed drop $\Delta v$ and recovery time $t_{\text{rec}}$. |
| **3. Load Increase/Decrease Cycle** | $1.80\text{ m/s}$ (Constant) | $t < 4.2\text{s}$: one box, $0.06\text{ N}\cdot\text{m}$<br>$4.2\text{s} \le t < 8.4\text{s}$: three boxes, $0.18\text{ N}\cdot\text{m}$<br>$t \ge 8.4\text{s}$: two boxes, $0.12\text{ N}\cdot\text{m}$ | Dynamic parcel accumulation and unloading. |
| **4. Robustness under Constant Load** | $1.80\text{ m/s}$ (Constant) | Light: one box, $0.06\text{ N}\cdot\text{m}$<br>Medium: two boxes, $0.12\text{ N}\cdot\text{m}$<br>Heavy: three boxes, $0.18\text{ N}\cdot\text{m}$ | Compare tracking and control effort across payload counts. |
| **5. Infeasible Actuator Saturation** | $4.14\text{ m/s}$ ($115\%$ of physical max) | Two boxes, $0.12\text{ N}\cdot\text{m}$ | Actuator clamping ($u=1.0$), integral anti-windup, and saturation duration. |
| **6. Changing Speed Reference** | $0 \to 0.99\text{ m/s} \to 1.80\text{ m/s} \to 1.26\text{ m/s}$ | Two boxes, $0.12\text{ N}\cdot\text{m}$ | Multi-step tracking, velocity transitions, acceleration/deceleration. |
| **7. Multi-Rate PWM Frequency Sweep** | $1.80\text{ m/s}$ (Constant) | Two boxes, $0.12\text{ N}\cdot\text{m}$ | Compare carrier rates ($5, 10, 20, 40\text{ Hz}$) against measured belt-speed ripple. |

Batch simulations map each active box to $0.06\text{ N}\cdot\text{m}$ by default, configurable with `--load-torque`. The total load is capped at the motor limit. The same payload schedule is applied to all three controllers; CSV rows record both payload count and resulting torque. PyBullet replays the logged box count and motor speed. It is a synchronized visual twin: its contact forces do not feed back into the Python motor model.

---

## Standard Simulation Settings
- **Timestep:** $\Delta t = 0.005\text{ s}$ ($200\text{ Hz}$) in batch suite; $\Delta t = 0.01\text{ s}$ ($100\text{ Hz}$) in live web engine.
- **Duration:** $12.0\text{ s}$ (Batch comparison suite); $8.0\text{ s}$ (Web battle suite).
- **Actuator Limits:** $u_{\text{min}} = 0.0$, $u_{\text{max}} = 1.0$ (Single-quadrant unipolar motor drive).
- **Default Controller Gains:**
  - **Bang-Bang:** Deadband $\Delta = 0.12\text{ m/s}$, Dwell time $\tau_{\text{dwell}} = 0.04\text{ s}$.
  - **PWM:** Carrier $f_{\text{pwm}} = 20.0\text{ Hz}$, $K_p = 0.55$, $K_{\text{ff}} = 0.4545$.
  - **PID:** $K_p = 0.70$, $K_i = 0.80$, $K_d = 0.010$, Filter $\tau_d = 0.025\text{ s}$.

---

## Empirical Findings Summary

1. **PID Control:**
   - **Strengths:** In the current generated results, the PID has the lowest baseline MAE ($0.1791\text{ m/s}$) and peak-to-peak speed ripple ($0.000263\text{ m/s}$). It recovers from the default one-to-three-box load step in $2.105\text{ s}$.
   - **Trade-off:** Requires anti-windup clamping to prevent large overshoot on high load desaturation.

2. **PWM Control:**
   - **Strengths:** Explicit carrier-frequency switching. In the current sweep, peak-to-peak ripple drops from $0.0321\text{ m/s}$ at $5\text{ Hz}$ to $0.00734\text{ m/s}$ at $20\text{ Hz}$; the $40\text{ Hz}$ sample is $0.00864\text{ m/s}$, so the trend is not strictly monotonic.
   - **Trade-off:** Slight steady-state offset under heavy load due to lack of integral accumulation.

3. **Bang-Bang Control:**
   - **Strengths:** Simple relay control and fast rise time (tied at $2.48\text{ s}$ in the baseline run).
   - **Trade-off:** It has the largest baseline peak-to-peak speed ripple in the current results ($0.166\text{ m/s}$).

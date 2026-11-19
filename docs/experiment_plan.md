# Experiment Plan and Evaluation Protocols

This document details the reproducible scenarios executed across Bang-Bang, PWM, and PID controllers under identical motor plant parameters ($J=0.06\text{ kg}\cdot\text{m}^2, K=1.10\text{ N}\cdot\text{m}, b=0.04\text{ N}\cdot\text{m}\cdot\text{s/rad}, r=0.08\text{ m}$).

---

## Experiment Matrix

| Scenario Name | Setpoint $v_{\text{target}}$ | Disturbance Schedule $\tau_{\text{load}}(t)$ | Primary Phenomenon Evaluated |
| :--- | :--- | :--- | :--- |
| **1. Baseline Speed Tracking** | $1.80\text{ m/s}$ (Constant) | $\tau_{\text{load}} = 0.0\text{ N}\cdot\text{m}$ | Unloaded step response, rise time, settling time, overshoot, steady-state ripple. |
| **2. Load Step Disturbance** | $1.80\text{ m/s}$ (Constant) | $t < 4.8\text{s}: 0.0\text{ N}\cdot\text{m}$<br>$t \ge 4.8\text{s}: 0.16\text{ N}\cdot\text{m}$ | Sudden mass increase (e.g., heavy box dropped onto belt), speed drop $\Delta v$, recovery time $t_{\text{rec}}$. |
| **3. Load Increase/Decrease Cycle** | $1.80\text{ m/s}$ (Constant) | $t < 4.2\text{s}: 0.0\text{ N}\cdot\text{m}$<br>$4.2\text{s} \le t < 8.4\text{s}: 0.24\text{ N}\cdot\text{m}$ ($80\%$ max)<br>$t \ge 8.4\text{s}: 0.045\text{ N}\cdot\text{m}$ | Dynamic loading cycle representing conveyor parcel accumulation and discharge. |
| **4. Robustness under Constant Load** | $1.80\text{ m/s}$ (Constant) | Three steady levels:<br>Light: $0.051\text{ N}\cdot\text{m}$<br>Medium: $0.159\text{ N}\cdot\text{m}$<br>Heavy: $0.270\text{ N}\cdot\text{m}$ | Sensitivity of tracking accuracy and control effort across conveyor operational regimes. |
| **5. Infeasible Actuator Saturation** | $4.14\text{ m/s}$ ($115\%$ of physical max) | $\tau_{\text{load}} = 0.0\text{ N}\cdot\text{m}$ | Actuator clamping ($u=1.0$), integral anti-windup prevention, saturation duration. |
| **6. Changing Speed Reference** | $0 \to 0.99\text{ m/s} \to 1.80\text{ m/s} \to 1.26\text{ m/s}$ | $\tau_{\text{load}} = 0.0\text{ N}\cdot\text{m}$ | Multi-step tracking, velocity transitions, bidirectional acceleration/deceleration. |
| **7. Multi-Rate PWM Frequency Sweep** | $1.80\text{ m/s}$ (Constant) | $\tau_{\text{load}} = 0.0\text{ N}\cdot\text{m}$ | Carrier rate comparison ($5\text{ Hz}, 10\text{ Hz}, 20\text{ Hz}, 40\text{ Hz}$) vs speed ripple RMS. |

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
   - **Strengths:** Zero steady-state error ($7.3 \times 10^{-5}\text{ m/s}$), lowest speed ripple ($0.00054\text{ m/s}$ peak-to-peak), full recovery after load step ($t_{\text{rec}} = 2.54\text{ s}$).
   - **Trade-off:** Requires anti-windup clamping to prevent large overshoot on high load desaturation.

2. **PWM Control:**
   - **Strengths:** Controlled carrier-frequency switching without linear amplifier power dissipation; speed ripple decreases inversely with carrier frequency ($20\text{ Hz} \implies 0.017\text{ m/s}$ ripple).
   - **Trade-off:** Slight steady-state offset under heavy load due to lack of integral accumulation.

3. **Bang-Bang Control:**
   - **Strengths:** Fastest initial rise time ($1.87\text{ s}$); minimal computational complexity.
   - **Trade-off:** Induces characteristic limit-cycle oscillation ($0.245\text{ m/s}$ peak-to-peak ripple); frequent actuator switching.

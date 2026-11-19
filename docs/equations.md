# Mathematical Formulations and Control Equations

## 1. Electromechanical DC Motor Dynamics

### Full Second-Order System:
The complete electrical and mechanical state equations of a DC motor are:
$$L \frac{di(t)}{dt} = V(t) - R i(t) - K_e \omega(t)$$
$$J \frac{d\omega(t)}{dt} = K_t i(t) - b \omega(t) - \tau_{\text{load}}(t)$$

### Implemented Reduced-Order Model:
In industrial conveyor drives where the electrical time constant $\tau_e = L/R \ll \tau_m = J/b$, armature current reaches steady-state almost instantaneously ($i \approx (V - K_e \omega)/R$). Substituting into the torque balance yields the reduced-order mechanical differential equation:
$$J \frac{d\omega(t)}{dt} = K u(t) - b \omega(t) - \tau_{\text{load}}(t)$$

Where:
- $u(t) \in [0.0, 1.0]$: Normalized input effort (armature voltage fraction $V / V_{\text{max}}$).
- $J = 0.06\text{ kg}\cdot\text{m}^2$: Lumped moment of inertia (rotor, shaft, drive roller, and effective conveyor belt inertia).
- $K = 1.10\text{ N}\cdot\text{m}$: Motor torque scaling constant ($K_t V_{\text{max}} / R$).
- $b = 0.04\text{ N}\cdot\text{m}\cdot\text{s/rad}$: Viscous friction damping coefficient.
- $\tau_{\text{load}}(t)$: External disturbance and cargo resistance torque ($\text{N}\cdot\text{m}$).
- $\omega(t)$: Motor rotor angular velocity ($\text{rad/s}$).

### Linear Belt Kinematics:
The conveyor linear velocity $v(t)$ is directly coupled via drive roller radius $r = 0.08\text{ m}$:
$$v(t) = r \cdot \omega(t)$$
$$\text{RPM}(t) = \omega(t) \cdot \frac{60}{2\pi}$$

---

## 2. Discrete Numerical Integration

Using fixed-step Forward Euler integration with simulation timestep $\Delta t = 0.005\text{ s}$ (or $\Delta t = 0.01\text{ s}$ in live engine):
$$\alpha_k = \frac{K u_k - b \omega_k - \tau_{\text{load}, k}}{J}$$
$$\omega_{k+1} = \min\left(\omega_{\text{max}}, \max\left(0.0, \omega_k + \alpha_k \Delta t\right)\right)$$
$$v_{k+1} = r \cdot \omega_{k+1}$$

---

## 3. Control Algorithms

### A. Bang-Bang Controller (Two-Position Relay with Deadband)
Given error $e_k = v_{\text{target}, k} - v_{\text{measured}, k}$:
$$u_{\text{raw}, k} = \begin{cases} 
u_{\text{max}} & e_k > \Delta \\
u_{\text{min}} & e_k < -\Delta \\
u_{k-1} & |e_k| \le \Delta
\end{cases}$$
Subject to minimum switching dwell interval $\tau_{\text{dwell}} = 0.04\text{ s}$:
$$\text{State changes only if } (t_k - t_{\text{last\_switch}}) \ge \tau_{\text{dwell}}$$

### B. Pulse-Width Modulation (PWM) Controller
Proportional error-to-duty mapping:
$$D_{\text{raw}, k} = K_{\text{ff}} v_{\text{target}, k} + K_p e_k$$
$$D_k = \min\left(1.0, \max\left(0.0, D_{\text{raw}, k}\right)\right)$$
Discrete carrier wave test at carrier frequency $f_{\text{pwm}}$ (default $20\text{ Hz}$):
$$s_k = \left(t_k \cdot f_{\text{pwm}} \pmod{1.0}\right) < D_k$$
$$u_k = \begin{cases} u_{\text{max}} & s_k \text{ is True} \\ u_{\text{min}} & s_k \text{ is False} \end{cases}$$

### C. Discrete PID Controller with Anti-Windup
Proportional error: $P_k = K_p e_k$.  
Derivative with low-pass filter ($\tau_d = 0.025\text{ s}$):
$$\beta = \frac{\Delta t}{\tau_d + \Delta t}$$
$$d_k = \frac{e_k - e_{k-1}}{\Delta t}$$
$$D_k = D_{k-1} + \beta (d_k - D_{k-1})$$
$$D_{\text{term}, k} = K_d D_k$$

Conditional Integration Anti-Windup:
$$I_{\text{prop}} = I_{k-1} + e_k \Delta t$$
$$u_{\text{tentative}} = P_k + K_i I_{\text{prop}} + D_{\text{term}, k}$$
$$u_k = \min\left(u_{\text{max}}, \max\left(u_{\text{min}}, u_{\text{tentative}}\right)\right)$$
$$\text{If } (u_{\text{tentative}} == u_k) \lor (u_{\text{tentative}} > u_{\text{max}} \land e_k < 0) \lor (u_{\text{tentative}} < u_{\text{min}} \land e_k > 0):$$
$$I_k = I_{\text{prop}}$$
$$\text{Else: } I_k = I_{k-1} \quad (\text{Integrator Clamped})$$

---

## 4. Frequency-Domain Formulations

1. **Open-Loop Plant Transfer Function:**
   $$G_p(s) = \frac{\Omega(s)}{U(s)} = \frac{K}{J s + b} = \frac{1.10}{0.06 s + 0.04} = \frac{18.33}{s + 0.667}$$
   Linear conveyor speed transfer function:
   $$G_v(s) = \frac{V(s)}{U(s)} = r \cdot G_p(s) = \frac{1.467}{s + 0.667}$$
   Pole: $s = -0.667\text{ rad/s}$ ($f_c = 0.106\text{ Hz}$).

2. **Discrete FFT Speed Ripple Spectrum:**
   Sampled over steady-state window ($N$ samples):
   $$w[n] = 0.5 \left(1 - \cos\left(\frac{2\pi n}{N-1}\right)\right) \quad (\text{Hann Window})$$
   $$X[k] = \sum_{n=0}^{N-1} (v[n] - \bar{v}) w[n] e^{-j 2\pi k n / N}$$

---

## 5. Quantitative Performance Metrics

- **Mean Absolute Error (MAE):** $\text{MAE} = \frac{1}{N} \sum_{k=1}^N |e_k|$
- **Root Mean Square Error (RMSE):** $\text{RMSE} = \sqrt{\frac{1}{N} \sum_{k=1}^N e_k^2}$
- **Integral Absolute Error (IAE):** $\text{IAE} = \sum_{k=1}^{N-1} |e_k| \Delta t$
- **Integral Squared Error (ISE):** $\text{ISE} = \sum_{k=1}^{N-1} e_k^2 \Delta t$
- **Rise Time ($t_r$):** Duration between 10% and 90% of target speed.
- **Settling Time ($t_s$):** First time sample after which all subsequent speed values stay within 2% of setpoint ($|v_k - v_{\text{target}}| \le 0.02 v_{\text{target}}$).
- **Overshoot Percentage:** $\text{OS}\% = \max\left(0.0, \frac{\max(v) - v_{\text{target}}}{v_{\text{target}}} \times 100\right)$
- **Control Effort:** $\int_0^T u(t)^2 dt \approx \sum_{k=1}^{N-1} u_k^2 \Delta t$
- **Speed Ripple RMS:** Population standard deviation of the last 20% of steady-state speed samples.

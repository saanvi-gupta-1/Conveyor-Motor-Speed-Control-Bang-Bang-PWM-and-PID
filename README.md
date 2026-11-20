# Conveyor Motor Speed Control: Bang-Bang, PWM and PID

[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue.svg?logo=python)](https://www.python.org/)
[![Physics Engine: PyBullet](https://img.shields.io/badge/Physics%20Engine-PyBullet%203D-orange.svg)](https://pybullet.org/)
[![GUI: Real--Time Sliders](https://img.shields.io/badge/GUI-Interactive%20Parameters-green.svg)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-purple.svg)](LICENSE)
[![Academic Project](https://img.shields.io/badge/Academic%20Project-P15-red.svg)]()

> **An end-to-end electromechanical systems modeling and control engineering project.**  
> Investigating permanent magnet DC motor speed regulation on an industrial conveyor belt under dynamic cargo payload variations and actuator voltage saturation ($\pm 24\,\text{V}$).

---

## 📸 3D Simulation & Performance Dashboard

<p align="center">
  <img src="assets/conveyor_3d_simulation.jpg" width="85%" alt="3D PyBullet Industrial Conveyor Simulation" />
</p>

<p align="center">
  <em>Figure 1: PyBullet 3D Real-Time Simulation with Industrial Conveyor Bed, Blue DC Motor, Safety Yellow Rails, Animated Slats, and Green Collection Hopper.</em>
</p>

---

## 👥 Team Members & Contribution Matrix (Team of 4)

This project was developed by a team of 4 engineers, with modular separation of responsibilities across physical modeling, control algorithms, 3D visualization, and quantitative data analytics:

| Team Member | Engineering Role | Primary Owned Modules | Key Responsibilities & Contributions |
| :--- | :--- | :--- | :--- |
| **Member 1** *(Lead)* | **Physical Systems & Electromechanical Modeling** | `simulation/motor_model.py`<br>`simulation/conveyor.py`<br>`test_motor_standalone.py` | • Derived electromechanical state-space differential equations.<br>• Implemented 4th-Order Runge-Kutta (RK4) numerical ODE solver.<br>• Modeled payload friction torque ($T_{\text{load}} = \mu \sum m_i g r$) and reflected moment of inertia ($J_{\text{eff}} = J_{\text{rotor}} + \sum m_i r^2$).<br>• Solved inelastic momentum impact when boxes land on belt. |
| **Member 2** | **Control Systems & Algorithm Architecture** | `controllers/bang_bang.py`<br>`controllers/pwm.py`<br>`controllers/pid.py`<br>`test_controllers_standalone.py` | • Formulated unipolar Bang-Bang relay with hysteresis band ($\pm 0.03\,\text{m/s}$).<br>• Designed PWM proportional duty-cycle modulation and modeled open-loop speed droop.<br>• Implemented full PID controller with filtered derivative ($N=10$).<br>• Engineered conditional **Anti-Windup Integrator Clamping** for actuator ceiling ($0\text{ to }24\,\text{V}$). |
| **Member 3** | **3D Simulation, PyBullet Rigging & Interactive UI** | `simulation/conveyor_scene.py`<br>`dashboard/cli_menu.py`<br>`main.py`<br>`Run_Conveyor_Project.bat` | • Built full 3D PyBullet CAD environment (frame, rollers, rails, motor, slats, collection hopper).<br>• Programmed real-time parameter sliders (Target Speed, Mode, Kp, Ki, Kd) and debounced event buttons.<br>• Developed dual-interface Colorama CLI menu and live terminal status telemetry.<br>• Authored one-click Windows execution batch script (`Run_Conveyor_Project.bat`). |
| **Member 4** | **Quantitative Evaluation, Benchmarking & Analytics** | `evaluation/metrics.py`<br>`evaluation/plots.py`<br>`test_load_disturbances.py` | • Coded standard control metrics algorithms: $t_r$, $t_s$, $\%OS$, $e_{\text{ss}}$, $\text{RMSE}$, ripple, $t_{\text{rec}}$.<br>• Automated 12-second side-by-side benchmark test cycle under identical disturbance injections.<br>• Built multi-panel publication-grade Matplotlib dashboards saved directly as high-resolution `.jpg`.<br>• Designed 4-dimension engineering scorecard bar chart comparing controllers. |

---

## 🎯 Executive Summary & Objectives

In automated distribution warehouses and industrial production lines, conveyor belts must maintain an exact target linear velocity $v_{\text{target}}$ regardless of package mass variations. Placing cargo boxes on the belt introduces sudden external load torque surges and increases rotational inertia.

This project investigates and benchmarks **three fundamental control strategies**:
1. **Bang-Bang (Binary ON/OFF Relay with Hysteresis)**: Maximum actuator wear, high overshoot, high-frequency chattering.
2. **Pulse-Width Modulation (PWM Proportional Duty Cycle)**: Smooth actuation, zero chattering, but exhibits unavoidable **steady-state speed droop** under heavy cargo payload.
3. **Proportional-Integral-Derivative (PID with Anti-Windup Clamping)**: Fast transient rise time, zero steady-state error, rapid disturbance rejection, and smooth actuator voltage without integrator saturation.

---

## 📐 Mathematical Formulation

### 1. DC Motor Electromechanical State-Space Equations
The permanent magnet DC motor driving the conveyor pulley is governed by coupled electrical and mechanical differential equations:

$$\frac{di(t)}{dt} = \frac{V(t) - R \cdot i(t) - K_e \cdot \omega(t)}{L}$$

$$\frac{d\omega(t)}{dt} = \frac{K_t \cdot i(t) - B \cdot \omega(t) - T_{\text{load}}(t)}{J_{\text{eff}}}$$

The linear conveyor belt speed $v(t)$ is directly proportional to drive pulley radius $r_{\text{pulley}} = 0.05\,\text{m}$:

$$v(t) = \omega(t) \cdot r_{\text{pulley}}$$

### 2. Motor & Conveyor Parameter Specifications

| Parameter | Symbol | Value | Physical Significance |
| :--- | :--- | :--- | :--- |
| **Armature Resistance** | $R$ | $1.00\,\Omega$ | Motor winding coil resistance |
| **Armature Inductance** | $L$ | $5.00\,\text{mH}$ | Armature magnetic field lag constant |
| **Back-EMF Constant** | $K_e$ | $0.32\,\text{V}\cdot\text{s/rad}$ | Counter-electromotive force coefficient |
| **Torque Constant** | $K_t$ | $0.32\,\text{N}\cdot\text{m/A}$ | Armature current to mechanical torque ratio |
| **Rotor Inertia** | $J_{\text{rotor}}$ | $0.015\,\text{kg}\cdot\text{m}^2$ | Motor armature & drive pulley base rotational inertia |
| **Viscous Damping** | $B$ | $0.008\,\text{N}\cdot\text{m}\cdot\text{s/rad}$ | Bearing and shaft mechanical friction |
| **Drive Pulley Radius** | $r_{\text{pulley}}$ | $0.05\,\text{m}$ | Converts angular speed $\omega$ to linear belt speed $v$ |
| **Actuator Voltage Limit** | $V_{\text{max}}$ | $+24.0\,\text{V}$ | Maximum unipolar power supply ceiling |
| **Conveyor Belt Length** | $L_{\text{belt}}$ | $3.80\,\text{m}$ | Usable physical transport bed |
| **Coulomb Friction Coeff.** | $\mu$ | $0.35$ | Sliding friction between cargo boxes and belt bed |

### 3. Cargo Payload Disturbance Reflection
When cargo boxes of masses $m_i$ are placed on the conveyor belt:
1. **Frictional Load Torque**:
   $$T_{\text{load}}(t) = \mu \cdot \left(\sum_{i=1}^{N} m_i\right) \cdot g \cdot r_{\text{pulley}}$$
   - *Light Box (1.0 kg)* $\rightarrow T_{\text{load}} = 0.17\,\text{N}\cdot\text{m}$
   - *Medium Box (3.5 kg)* $\rightarrow T_{\text{load}} = 0.60\,\text{N}\cdot\text{m}$
   - *Heavy Box (8.5 kg)* $\rightarrow T_{\text{load}} = 1.46\,\text{N}\cdot\text{m}$

2. **Reflected Moment of Inertia**:
   $$J_{\text{eff}} = J_{\text{rotor}} + \left(\sum_{i=1}^{N} m_i\right) \cdot r_{\text{pulley}}^2$$

---

## 📊 Benchmark Results & Scorecard Comparison

All three controllers were evaluated through an automated 12.0-second benchmark cycle with identical operating setpoints and cargo disturbance drops:
* **$0.0\,\text{s} - 4.0\,\text{s}$**: Step response from $0.0$ to $1.20\,\text{m/s}$ (Empty conveyor belt).
* **$4.0\,\text{s}$**: Disturbance 1 $\rightarrow$ **Medium Cargo Box (3.5 kg)** dropped onto the belt.
* **$8.0\,\text{s}$**: Disturbance 2 $\rightarrow$ **Heavy Cargo Box (8.5 kg)** dropped onto the belt.

<p align="center">
  <img src="assets/session_3_controllers_comparison.jpg" width="95%" alt="Multi-Controller Benchmark and Scorecard" />
</p>

<p align="center">
  <em>Figure 2: Comprehensive 4-Panel Multi-Controller Comparison: (1) Belt Speed $v(t)$, (2) Armature Voltage $V(t)$, (3) Speed Tracking Error $e(t)$, and (4) Quantitative Scorecard Bar Chart.</em>
</p>

### Quantitative Performance Metrics Table

| Metric | Bang-Bang Controller | PWM Controller | PID Controller (Tuned) | Engineering Analysis |
| :--- | :---: | :---: | :---: | :--- |
| **Rise Time ($t_r$)** | **0.058 s** | 0.071 s | 0.062 s | Bang-Bang hits 24V instantly; PID reaches setpoint nearly as fast. |
| **Settling Time ($t_s$)** | 4.000 s *(Never settles)* | **0.221 s** | 1.321 s | Bang-Bang stays in continuous limit cycle oscillation. |
| **Percentage Overshoot (%OS)** | 29.6% | 14.8% | **3.5%** | PID's derivative term dampens initial overshoot effectively. |
| **Steady-State Error ($e_{\text{ss}}$)** | 0.0445 m/s | 0.3021 m/s *(Under 8.5kg)* | **0.0000 m/s** | PID integrator drives steady-state error completely to zero. |
| **RMS Error (RMSE)** | 0.1550 m/s | 0.1210 m/s | **0.0615 m/s** | PID achieves the lowest cumulative root-mean-square tracking error. |
| **Speed Ripple / Chatter** | 0.2421 m/s *(Severe)* | **0.0000 m/s** | 0.0092 m/s | Bang-Bang chatters at 240 Hz; PWM and PID are smooth. |
| **Disturbance Speed Dip** | 0.0963 m/s | 0.0975 m/s | **0.0559 m/s** | PID resists load surge with minimum speed drop. |
| **Disturbance Recovery Time** | 0.146 s | $\infty$ *(Permanent Droop)* | **0.104 s** | PWM droops permanently; PID recovers to 1.20 m/s within 0.10 s. |

<p align="center">
  <img src="assets/comparison_benchmark.jpg" width="90%" alt="Side by Side Benchmark Comparison" />
</p>

<p align="center">
  <em>Figure 3: Side-by-Side Step Response and Disturbance Rejection Benchmark ($v(t)$, $V(t)$, and $e(t)$).</em>
</p>

---

## 🗂️ Repository Architecture

```
conveyor_motor_control/
├── assets/                               # Professional documentation images for GitHub
│   ├── conveyor_3d_simulation.jpg        # 3D PyBullet simulation screenshot
│   ├── session_3_controllers_comparison.jpg # 4-panel scorecard comparison figure
│   ├── comparison_benchmark.jpg          # 3-panel step & disturbance figure
│   └── conveyor_telemetry.jpg            # Live session multi-variable telemetry plot
├── controllers/                          # Modular controller algorithms (Member 2)
│   ├── __init__.py                       # Package exports
│   ├── bang_bang.py                      # Unipolar relay controller with hysteresis
│   ├── pwm.py                            # PWM duty-cycle proportional controller
│   └── pid.py                            # PID controller with anti-windup clamping
├── simulation/                           # Physics and 3D visual engine (Members 1 & 3)
│   ├── __init__.py                       # Package exports
│   ├── motor_model.py                    # DC motor state-space RK4 ODE solver
│   ├── conveyor.py                       # Conveyor belt physical kinematics & boxes
│   └── conveyor_scene.py                 # PyBullet 3D CAD environment & GUI controls
├── evaluation/                           # Numerical metrics & plotting (Member 4)
│   ├── __init__.py                       # Package exports
│   ├── metrics.py                        # Control engineering metrics calculations
│   └── plots.py                          # High-resolution scientific Matplotlib plots (.jpg)
├── dashboard/                            # Terminal User Interface (Member 3)
│   ├── __init__.py                       # Package exports
│   └── cli_menu.py                       # Colorama interactive CLI menu & status bar
├── test_motor_standalone.py              # Standalone verification for motor ODEs
├── test_controllers_standalone.py        # Standalone verification for controllers
├── test_load_disturbances.py             # Standalone verification for cargo disturbances
├── test_pybullet_standalone.py           # Standalone verification for PyBullet 3D
├── main.py                               # Central orchestrator integrating all modules
├── Run_Conveyor_Project.bat              # One-click Windows runner
├── requirements.txt                      # Project Python dependencies
├── .gitignore                            # Comprehensive git exclusion rules
└── README.md                             # Professional academic repository documentation
```

---

## 🚀 Installation & Quick Start

### 1. Prerequisites
* **Operating System**: Windows 10/11, Ubuntu 20.04+, or macOS
* **Python**: Version 3.10, 3.11, 3.12, 3.13, or 3.14

### 2. Step-by-Step Setup

```bash
# 1. Clone the repository
git clone https://github.com/<your-username>/conveyor-motor-speed-control.git
cd conveyor-motor-speed-control

# 2. Create and activate a virtual environment
python -m venv venv

# Windows (Command Prompt / PowerShell):
venv\Scripts\activate

# Linux / macOS:
source venv/bin/activate

# 3. Install dependencies
pip install -r requirements.txt
```

### 3. Launching the Simulation
* **On Windows**: Simply double-click **`Run_Conveyor_Project.bat`**
* **Via Terminal**:
  ```bash
  python main.py
  ```


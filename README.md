# Conveyor Motor Speed Control: Bang-Bang, PWM and PID

A comprehensive, academic control-systems simulation and digital twin of an industrial DC motor driving a conveyor belt. The project models electromechanical plant dynamics, implements and compares three distinct control strategies (**Bang-Bang**, **PWM**, and **PID**), and evaluates performance under dynamic load disturbances and actuator saturation constraints. 

The system features both an **interactive SCADA Web Control Room Dashboard** with live telemetry, dynamic cargo physics, real-time charts, and procedural audio synthesis, alongside a **PyBullet 3D Digital Twin** featuring an actual revolute motor/roller joint and moving parcels.

---

## Table of Contents
1. [Objectives and Features](#objectives-and-features)
2. [System Architecture](#system-architecture)
3. [Mathematical Motor Model](#mathematical-motor-model)
4. [Controller Implementations](#controller-implementations)
5. [Presentation Interfaces](#presentation-interfaces)
   - [SCADA Web Control Room](#1-scada-web-control-room-dashboard-recommended)
   - [PyBullet 3D Digital Twin](#2-pybullet-3d-digital-twin)
   - [Rich Terminal CLI Dashboard](#3-rich-terminal-command-line-dashboard)
6. [Experiment Scenarios](#experiment-scenarios)
7. [Performance Metrics & Analysis](#performance-metrics--analysis)
8. [Installation & Windows Quick Start](#installation--windows-quick-start)
9. [Running the Project](#running-the-project)
10. [Automated Testing](#automated-testing)
11. [Project Directory Structure](#project-directory-structure)
12. [Assumptions and Limitations](#assumptions-and-limitations)
13. [Technologies Used](#technologies-used)

---

## Objectives and Features

- **Electromechanical Plant Modeling:** Reduced-order DC motor model ($J \dot{\omega} = K u - b \omega - \tau_{\text{load}}$) integrated via fixed-step numerical integration.
- **Fair Controller Benchmarking:** Bang-Bang, discrete PWM, and discrete PID operate on the identical motor plant under identical load profiles.
- **Actuator Constraints & Saturation:** Clamped control command $[0, 1]$, raw vs. applied logging, and dedicated saturation testing ($115\%$ setpoint).
- **Integral Anti-Windup:** Conditional integration halts integrator accumulation during saturation, preventing runaway and overshoot.
- **Dynamic Load Disturbances:** Step increases, multi-stage loading cycles, and physical cargo parcel masses ($1.0 - 4.5\text{ kg}$).
- **Time & Frequency Domain Analysis:** 20 quantitative metrics (IAE, ISE, RMSE, Settling, Rise, Ripple), analytical Bode plots, and Hann-windowed FFT speed ripple spectra.
- **Dual Visual Digital Twins:**
  1. *SCADA Web Twin:* High-fps HTML5 Canvas conveyor with spinning rollers, belt chevrons, moving packages, live multi-trace charts, controller battle mode, and Web Audio API synthesizer.
  2. *PyBullet 3D Twin:* Physics-based scene with a driven roller revolute joint (`p.JOINT_REVOLUTE`), cylindrical motor housing, conveyor deck, and payload boxes.
- **Automated Results & Reports:** 14 Matplotlib plots, 37 CSV logs, 28 JSON metric files, master `metrics.csv`, and empirical data-driven interpretations.

---

## System Architecture

The simulation is architected as a modular hybrid system: Python computes the core differential equations and control laws, while the visual layers serve as synchronized digital twins.

```text
USER INPUT (Web Control Room / Sliders / CLI / Batch Scripts)
    │
    ▼
TARGET SPEED (v_target)
    │
    ▼
CONTROLLER (Bang-Bang / Carrier PWM / Discrete PID)
    │
    ▼
ACTUATOR SATURATION (Clamped u in [u_min, 1.0])
    │
    ▼
DC MOTOR MODEL (J dω/dt = Ku - bω - Tau_load) <─── LOAD TORQUE / DISTURBANCES
    │                                                   (Tare + Cargo + Manual)
    ▼
CONVEYOR SPEED (v = r * ω)
    │
    ├─────────────────────────────┬─────────────────────────────┐
    ▼                             ▼                             ▼
FEEDBACK SENSOR          SCADA WEB TWIN (2D)           PYBULLET 3D TWIN
(v_meas = max(0, v+η))   - Moving Cargo Packages       - Driven Revolute Roller
    │                    - Rotating Roller Spokes      - Motor Housing
    └─► CONTROLLER LOOP  - Live Multi-Trace Charts     - Moving Rigid Boxes
```

See [docs/architecture.md](file:///d:/GitHub/Conveyor-Motor-Speed-Control-Bang-Bang-PWM-and-PID/docs/architecture.md) for full block diagram and module specifications.

---

## Mathematical Motor Model

Industrial conveyor drives operate with an electrical time constant $\tau_e = L/R \ll \tau_m = J/b$. Armature current reaches quasi-steady-state almost instantaneously, yielding the reduced-order torque balance equation:

$$J \frac{d\omega(t)}{dt} = K u(t) - b \omega(t) - \tau_{\text{load}}(t)$$

$$v(t) = r \cdot \omega(t)$$

$$\text{RPM}(t) = \omega(t) \cdot \frac{60}{2\pi}$$

### Model Parameters (SI Units)
- **Moment of Inertia ($J$):** $0.06\text{ kg}\cdot\text{m}^2$ (Lumped rotor, shaft, rollers, and belt).
- **Torque Constant ($K$):** $1.10\text{ N}\cdot\text{m}$ (Per normalized input effort).
- **Viscous Friction ($b$):** $0.04\text{ N}\cdot\text{m}\cdot\text{s/rad}$.
- **Drive Roller Radius ($r$):** $0.08\text{ m}$.
- **Speed Limit ($\omega_{\text{max}}$):** $45.0\text{ rad/s}$ ($v_{\text{max}} = 3.6\text{ m/s}$, $429.7\text{ RPM}$).
- **Maximum Load Torque ($\tau_{\text{max}}$):** $0.30\text{ N}\cdot\text{m}$.
- **Timestep ($\Delta t$):** $0.005\text{ s}$ ($200\text{ Hz}$) for batch runs; $0.01\text{ s}$ ($100\text{ Hz}$) for live engine.

See [docs/equations.md](file:///d:/GitHub/Conveyor-Motor-Speed-Control-Bang-Bang-PWM-and-PID/docs/equations.md) for full mathematical derivations.

---

## Controller Implementations

All three controllers share a standardized interface (`update(target, measured, dt, time) -> ControlOutput`):

### 1. Bang-Bang Controller (Relay Actuation)
Two-position threshold controller with deadband $\Delta = 0.12\text{ m/s}$ and minimum dwell time $\tau_{\text{dwell}} = 0.04\text{ s}$ to prevent high-frequency chatter:
$$u(t) = \begin{cases} u_{\text{max}} & e(t) > \Delta \\ u_{\text{min}} & e(t) < -\Delta \\ u(t^-) & |e(t)| \le \Delta \end{cases}$$
- **Characteristics:** Fast rise time ($1.87\text{ s}$), zero modulation complexity, but induces an orbitally stable limit cycle with steady-state speed ripple ($0.245\text{ m/s}$ p-p).

### 2. PWM Controller (Carrier-Based Switched Actuation)
Proportional error-to-duty mapping driving an explicit square-wave carrier oscillator ($f_{\text{pwm}} = 20\text{ Hz}$):
$$D(t) = \text{sat}(K_{\text{ff}} v_{\text{target}} + K_p e(t))$$
$$u_{\text{PWM}}(t) = \begin{cases} 1.0 & (t \cdot f_{\text{pwm}} \pmod 1) < D(t) \\ 0.0 & \text{otherwise} \end{cases}$$
- **Characteristics:** True switching actuation (not an averaged-voltage approximation). Mechanical rotor inertia filters the pulse train into smooth velocity; speed ripple decreases inversely with carrier frequency.

### 3. Discrete PID Controller (Continuous Linear Actuation)
Discrete PID with low-pass filtered derivative ($\tau_d = 0.025\text{ s}$) and conditional integration anti-windup:
$$u_{\text{raw}}(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de_f(t)}{dt}$$
- **Anti-Windup Logic:** Halts integration when the actuator is saturated ($u_{\text{raw}} \notin [0, 1]$) and the error points deeper into the limit.
- **Default Gains:** $K_p = 0.70$, $K_i = 0.80$, $K_d = 0.010$.
- **Characteristics:** Zero steady-state error ($7.3 \times 10^{-5}\text{ m/s}$), lowest speed ripple ($0.00054\text{ m/s}$ p-p), and complete disturbance recovery ($t_{\text{rec}} = 2.54\text{ s}$).

---

## Presentation Interfaces

The project provides three synchronized presentation interfaces:

### 1. SCADA Web Control Room Dashboard (Recommended)
A modern industrial automation control room accessible locally at `http://127.0.0.1:8050`:
- **Canvas Conveyor Twin:** 2D interactive conveyor with spinning roller spokes, belt chevrons, moving and recycling cargo parcels ($1.0 - 4.5\text{ kg}$), and DC motor casing with live HUD.
- **Telemetry Readouts:** Digital gauges for actual/target RPM, linear speed ($v$), speed error ($e$), motor command ($u$), and load torque.
- **Master Controls & Sliders:** Play, Pause, Step, Reset, Target Speed slider ($0.2 - 3.5\text{ m/s}$), and Dynamic Load slider ($0.0 - 0.22\text{ Nm}$).
- **Interactive Tuning Station:** Dynamic parameter knobs for PID ($K_p, K_i, K_d$), PWM carrier frequency, and Bang-Bang deadband.
- **Disturbance Injection:** One-click buttons to drop heavy cargo (+4.5 kg), trigger torque pulses (+0.18 Nm for 2s), step setpoints, or clear cargo.
- **Live Multi-Trace Charts:** High-fps Canvas telemetry charts for Speed Tracking, Actuator Output ($u$ vs duty), and Error/Disturbance.
- **Controller Battle Mode:** Side-by-side comparative tournament evaluating Bang-Bang, PWM, and PID under identical conditions with automatic ranking and empirical strength/weakness analysis.
- **Predefined Experiment Lab:** Automated execution of standard scenarios with live tabular metrics.
- **Rule-Based Diagnostics:** Continuous assessment of tracking fidelity, actuator limits, and load stress.
- **Procedural Audio Synthesizer:** Web Audio API motor hum dynamically tracking RPM ($\propto \omega$), relay clicks on switching, and overload alerts (muted by default with a visible toggle).

### 2. PyBullet 3D Digital Twin
A standalone 3D OpenGL visualization of the conveyor system:
- **Revolute Motor Joint:** Driven roller operates with an actual PyBullet revolute joint (`p.JOINT_REVOLUTE`) synchronized with motor $\omega$.
- **Conveyor Assembly:** Conveyor deck, idler roller, cylindrical DC motor housing, and ground plane.
- **Payload Boxes:** Multi-body boxes of different masses and colors translate along the deck at linear belt velocity $v = r \cdot \omega$.
- **Real-Time Text HUD:** Displays motor synchronization state and instantaneous load torque.

### 3. Rich Terminal Command-Line Dashboard
A formatted terminal interface with styled tables, panels, and status indicators:
```powershell
python -m conveyor_control.cli
```

---

## Experiment Scenarios

The suite includes 7 repeatable scenarios defined in [docs/experiment_plan.md](file:///d:/GitHub/Conveyor-Motor-Speed-Control-Bang-Bang-PWM-and-PID/docs/experiment_plan.md):

1. **`baseline`:** Step response from 0 to $1.80\text{ m/s}$ under tare load. Evaluates rise time, overshoot, settling, and ripple.
2. **`load_step`:** Sudden load increase ($+0.16\text{ N}\cdot\text{m}$) applied at 40% of runtime. Evaluates speed drop $\Delta v$ and recovery time $t_{\text{rec}}$.
3. **`load_cycle`:** Multi-stage loading cycle ($0 \to 80\% \to 15\%$ rated torque). Evaluates dynamic parcel accumulation and discharge.
4. **`light_load`, `medium_load`, `heavy_load`:** Three steady load levels ($0.051, 0.159, 0.270\text{ N}\cdot\text{m}$). Evaluates steady-state load robustness.
5. **`saturation`:** Commands $4.14\text{ m/s}$ ($115\%$ of physical maximum). Demonstrates actuator clamping to $1.0$ and verifies anti-windup clamping.
6. **`changing_reference`:** Multi-step profile ($0.99 \to 1.80 \to 1.26\text{ m/s}$). Evaluates acceleration and deceleration tracking.
7. **`pwm-sweep`:** Compares PWM carrier rates ($5, 10, 20, 40\text{ Hz}$). Measures harmonic speed ripple attenuation.

---

## Performance Metrics & Analysis

### Time-Domain Metrics
Calculated programmatically across simulation data in `src/conveyor_control/metrics.py`:
- **Tracking:** MAE, RMSE, Maximum Absolute Error, Steady-State Error, IAE, ISE.
- **Transient:** Rise Time ($10\% \to 90\%$), Settling Time ($2\%$ band), Overshoot Percentage, Peak Speed.
- **Disturbance:** Speed Drop on Load Step ($\Delta v_{\text{drop}}$), Recovery Time ($t_{\text{rec}}$).
- **Actuator:** Control Effort ($\int u^2 dt$), Saturation Duration, Saturation Percentage.
- **Ripple & Switching:** Speed Ripple RMS, Peak-to-Peak Ripple, Bang-Bang Switch Count, Mean PWM Duty.

### Frequency-Domain & Stability Analysis
- **Plant Transfer Function:** First-order low-pass system $G(s) = \frac{18.33}{s + 0.667}$ with corner frequency $\omega_c = 0.667\text{ rad/s}$ ($0.106\text{ Hz}$).
- **Bode Diagram:** Generated to `results/plots/frequency_response.png`.
- **FFT Ripple Spectrum:** Hann-windowed FFT on steady-state velocity identifies dominant switching ripple harmonics (`results/plots/frequency_spectrum.png`).
- **Stability:** Open-loop pole at $s = -0.667$ lies strictly in the Left Half Plane ($\implies$ asymptotically stable). Closed-loop stability verified via 3rd-order Routh-Hurwitz criterion.

---

## Installation & Windows Quick Start

### Prerequisites
- Windows 10/11
- Python 3.10, 3.11, 3.12, or 3.14

### Automatic Setup (1-Click)
Double-click `run_project.bat`. On its first run, it automatically creates `.venv`, upgrades pip, installs required packages, and opens the main menu.

### Manual Setup via PowerShell
```powershell
# Clone or navigate to the repository
cd d:\GitHub\Conveyor-Motor-Speed-Control-Bang-Bang-PWM-and-PID

# Create and activate virtual environment
python -m venv .venv
.\.venv\Scripts\Activate.ps1

# Install package and dependencies
python -m pip install --upgrade pip
python -m pip install -e ".[dev,visual]"
```

---

## Running the Project

### 1. Launch SCADA Web Control Room (Recommended)
Double-click:
```text
run_dashboard.bat
```
*Or run via terminal:*
```powershell
.\.venv\Scripts\python.exe run_dashboard.py
```
Open **`http://127.0.0.1:8050`** in your browser. Press `Ctrl + C` in the terminal to stop.

### 2. Launch PyBullet 3D Digital Twin Visualizer
```powershell
.\.venv\Scripts\python.exe -m conveyor_control.cli --controller pid --scenario load_cycle --visualize
```

### 3. Run Full Comparative Experiment Suite
Executes all 3 controllers across all 8 scenarios, generating all plots, CSVs, and markdown reports:
```powershell
.\.venv\Scripts\python.exe -m conveyor_control.cli --compare
```

### 4. Interactive Menu
```powershell
run_project.bat
```

---

## Automated Testing

The repository includes a comprehensive automated test suite with **29 passing tests**:
```powershell
.\.venv\Scripts\python.exe -m pytest tests/ -v
```

### Test Coverage Breakdown:
- `tests/test_model.py`: Motor ODE integration, torque limits, viscous friction, speed bounds.
- `tests/test_controllers.py`: Bang-Bang deadband/chatter, PWM carrier oscillator, PID anti-windup clamping.
- `tests/test_metrics_simulation.py`: Programmatic metric calculation, reproducibility, zero targets.
- `tests/test_frequency.py`: Analytical transfer function and Bode evaluation.
- `tests/test_spectrum.py`: FFT spectral peak detection and Nyquist bounds.
- `tests/test_stability.py`: Open-loop and closed-loop pole locations.
- `tests/test_visualization.py`: PyBullet revolute roller joint and moving cargo payload.
- `tests/test_live_engine.py`: Real-time simulation engine, package kinematics, live disturbances.
- `tests/test_server.py`: REST API endpoints, telemetry streaming, and automated battle runner.

---

## Project Directory Structure

```text
Conveyor-Motor-Speed-Control-Bang-Bang-PWM-and-PID/
├── main.py                         # Application CLI entrypoint
├── run_dashboard.py                # Web dashboard server launcher
├── run_dashboard.bat               # 1-Click batch launcher for SCADA Control Room
├── run_project.bat                 # Multi-choice interactive batch launcher
├── verify_pybullet.py              # PyBullet connection verification script
├── fix_pybullet_windows.bat        # Windows Smart App Control helper
├── pyproject.toml                  # Project packaging configuration
├── requirements.txt                # Package dependencies
├── README.md                       # Master documentation
├── src/conveyor_control/           # Main source package
│   ├── model.py                    # Electromechanical DC motor ODE model
│   ├── controllers.py              # Bang-Bang, PWM, and PID controllers
│   ├── simulation.py               # Deterministic scenario simulation engine
│   ├── live_engine.py              # Real-time interactive simulation engine
│   ├── server.py                   # Multi-threaded HTTP/REST API server
│   ├── cli.py                      # Rich terminal command-line dashboard
│   ├── experiments.py              # Batch comparison orchestrator
│   ├── metrics.py                  # 20 time-domain performance metrics
│   ├── frequency.py                # Analytical Bode response generator
│   ├── spectrum.py                 # Discrete FFT speed ripple analyzer
│   ├── stability.py                # Stability and pole analyzer
│   ├── results.py                  # Matplotlib plotting and CSV export
│   ├── visualization.py            # PyBullet 3D visual twin
│   └── web/                        # Web SCADA assets
│       └── index.html              # Modern dark SCADA dashboard UI
├── tests/                          # 29 automated pytest test suites
├── docs/                           # Technical documentation
│   ├── architecture.md             # System architecture and block diagrams
│   ├── equations.md                # Mathematical equations and discrete ODEs
│   └── experiment_plan.md          # Experiment scenarios and evaluation protocols
└── results/                        # Generated output data
    ├── report.md                   # Experiment summary report
    ├── interpretation.md           # Empirical comparative analysis
    ├── metrics.csv                 # Master comparative metrics table
    ├── csv/                        # 37 raw simulation telemetry CSV files
    └── plots/                      # 14 high-resolution comparison plots
```

---

## Assumptions and Limitations

- **Model Assumptions:** Armature inductance ($L$) is neglected under the assumption that electrical dynamics are fast relative to mechanical inertia ($\tau_e \ll \tau_m$). Gearing, belt elasticity, slip, and thermal variations are modeled via lumped viscous damping ($b$) and external load torque ($\tau_{\text{load}}$).
- **Coupling Distinction:** Python owns the physical motor dynamics and control laws. PyBullet and the Canvas Web Twin operate as synchronized visual twins receiving motor speed ($v = r\omega$) and updating roller joints and parcel positions accordingly.
- **Audio Synthesis:** Sound effects are generated procedurally via the browser's Web Audio API and are muted by default. Audio processing runs in an isolated browser thread and does not impact simulation numerical solver accuracy.

---

## Technologies Used

- **Programming Language:** Python 3.10+ (tested on Python 3.14)
- **Physics & 3D Simulation:** PyBullet 3.2.7
- **Numerical Computation:** NumPy
- **Data Analysis & Export:** Pandas, CSV, JSON
- **Data Visualization & Plotting:** Matplotlib
- **Terminal UI & Styling:** Rich
- **Frontend Dashboard:** HTML5, CSS3 (SCADA Dark Theme), JavaScript (Canvas 2D, Web Audio API, Fetch REST API)
- **Backend Server:** Python `http.server.ThreadingHTTPServer` (Multi-threaded, zero external web dependencies)
- **Automated Testing:** Pytest

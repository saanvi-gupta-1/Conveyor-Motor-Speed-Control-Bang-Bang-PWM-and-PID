# System Architecture: Conveyor Motor Speed Control (P15)

## 1. Overview
The project is architected as a modular, high-fidelity hybrid simulation where mathematical electromechanical dynamics and control algorithms are calculated in Python, while visualization is delivered via both an interactive SCADA Web Twin and a PyBullet 3D Digital Twin.

```text
                                  +-----------------------+
                                  |      User Input       |
                                  | (Web SCADA / CLI / API)
                                  +-----------+-----------+
                                              |
                                              v
                                  +-----------------------+
                                  |     Target Speed      |
                                  |       v_target        |
                                  +-----------+-----------+
                                              |
                                              v
+-----------------------------------------------------------------------------------------+
|                                  CONTROLLER LAYER                                       |
|  +--------------------+        +--------------------+        +-----------------------+  |
|  |     Bang-Bang      |   OR   |    Discrete PWM    |   OR   |     Discrete PID      |  |
|  |  Relay + Deadband  |        |  Carrier Modulated |        | Conditional Anti-Windup|  |
|  +---------+----------+        +---------+----------+        +-----------+-----------+  |
+------------|-----------------------------|-------------------------------|--------------+
             |                             |                               |
             +-----------------------------+-------------------------------+
                                           |
                                           v
                             +---------------------------+
                             |    Actuator Saturation    |
                             |  Clamping u in [u_min, 1] |
                             +-------------+-------------+
                                           |
                                           | Voltage/Effort u(t)
                                           v
                             +---------------------------+
                             |      DC Motor Model       | <--- Load Torque Tau_load(t)
                             | J dω/dt = Ku - bω - Tau_L |      (Base + Cargo + Manual)
                             +-------------+-------------+
                                           |
                                           | Motor Speed ω(t)
                                           v
                             +---------------------------+
                             |      Conveyor Speed       |
                             |         v = r * ω         |
                             +-------------+-------------+
                                           |
                    +----------------------+----------------------+
                    |                                             |
                    v (Feedback Sensor)                           v (Visual Digital Twins)
        +-----------------------+                  +------------------------------------+
        |   Measured Velocity   |                  | PyBullet 3D Twin & SCADA Web Twin  |
        |  v_meas = max(0, v+η) |                  | - Driven Roller Revolute Joint     |
        +-----------+-----------+                  | - Moving Multi-body Payload Boxes  |
                    |                              | - Real-time Synchronized HUD       |
                    +----------------------------> +------------------------------------+
```

## 2. Core Modules & Responsibilities

| Module | Location | Responsibility |
| :--- | :--- | :--- |
| **DC Motor Plant** | `src/conveyor_control/model.py` | Reduced-order electromechanical ODE solver ($J\dot{\omega} = Ku - b\omega - \tau$) with fixed-step Euler integration. |
| **Controllers** | `src/conveyor_control/controllers.py` | Bang-Bang, Carrier PWM, and PID with anti-windup clamping and derivative filter. |
| **Simulation Engine** | `src/conveyor_control/simulation.py` | Fixed-step deterministic scenario execution and logging. |
| **Live Interactive Engine** | `src/conveyor_control/live_engine.py` | Thread-safe real-time simulator coupling motor dynamics, dynamic package transport physics, and telemetry. |
| **Web SCADA Server** | `src/conveyor_control/server.py` | Multi-threaded HTTP/REST server streaming telemetry and handling real-time control actions. |
| **Web SCADA Dashboard** | `src/conveyor_control/web/index.html` | Industrial automation control room with canvas conveyor visualizer, multi-trace charts, controller battle, and Web Audio API synthesizer. |
| **PyBullet 3D Twin** | `src/conveyor_control/visualization.py` | Bullet physics client with conveyor deck, idler roller, driven roller revolute joint (`p.JOINT_REVOLUTE`), and moving payload boxes. |
| **Time-Domain Metrics** | `src/conveyor_control/metrics.py` | Programmatic evaluation of 20 quantitative performance indicators (MAE, RMSE, Settling, Rise, Ripple). |
| **Frequency Analysis** | `src/conveyor_control/frequency.py` | Analytical transfer function $G(s)$ and Bode plot generation. |
| **FFT Spectral Ripple** | `src/conveyor_control/spectrum.py` | Hann-windowed discrete Fourier transform to quantify speed ripple harmonics. |
| **Stability Analysis** | `src/conveyor_control/stability.py` | Open-loop pole locations, closed-loop Routh-Hurwitz criterion, and limit cycle stability. |
| **CLI Dashboard** | `src/conveyor_control/cli.py` | Rich terminal dashboard with styled tables and interactive menus. |
| **Experiment Suite** | `src/conveyor_control/experiments.py` | Batch scenario orchestration and automated comparison suite. |
| **Results & Report** | `src/conveyor_control/results.py` | Matplotlib graph rendering, CSV exports, summary tables, and empirical data-driven interpretations. |

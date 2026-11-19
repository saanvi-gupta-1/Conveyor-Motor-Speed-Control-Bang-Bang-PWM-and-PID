# Conveyor Motor Speed Control: Bang-Bang, PWM and PID

A reproducible, command-line simulation of a DC motor driving a conveyor. The project compares relay control, carrier-based pulse-width modulation and discrete PID under reference changes, load disturbances and actuator constraints. CSV logs, computed metrics, comparison plots and an automatically generated interpretation support analysis and presentation.

## Objectives and features

- Simulate conveyor belt speed from a reduced-order DC motor model using fixed-step integration.
- Compare Bang-Bang, PWM and PID with identical plant and scenario settings.
- Exercise light, medium and heavy loads, a load increase/decrease cycle, changing setpoint and an infeasible saturation target.
- Log raw controller request, bounded actuator command, saturation, PWM duty/switching, Bang-Bang switch count and PID terms.
- Calculate tracking, overshoot, settling, effort and saturation metrics from samples.
- Generate comparison plots, load robustness charts, PWM carrier-frequency tests, FFT ripple spectra, CSV files and evidence-based interpretation.
- Optionally replay the recorded speed in a PyBullet conveyor scene with a revolute roller joint and moving boxes.
- Tune PID gains from command-line options; use an interactive menu when launched without arguments.

## Architecture

The motor and controller are calculated in Python. The observed belt speed is `v = rω`. PyBullet is an optional visual replay: its roller joint is synchronized from the simulated angular speed and boxes move according to belt speed. It is not a physics-coupled electrical motor simulation and does not generate the experiment data.

```text
Target → Controller → Saturated actuator → Motor + load → Conveyor speed
   ↑                                                   │
   └────────────────── Sensor feedback ───────────────┘
                                                       └→ CSV / metrics / plots / PyBullet replay
```

The implementation details, equations, assumptions and experiment profiles are summarized below and in the generated report under `results/`.

## Motor model

The implemented reduced-order equation is `J dω/dt = K u − bω − τ_load`; `v = rω`. The default parameters are `J=0.06 kg·m²`, `K=1.10 N·m` per normalized input, `b=0.04 N·m·s/rad`, roller radius `r=0.08 m`, speed limit `45 rad/s`, and load limit `0.30 N·m`. These are illustrative model assumptions, not measurements of a specific conveyor. Forward Euler runs at a default 5 ms step. Input, speed and load are bounded.

## Controllers

- **Bang-Bang:** switches between actuator bounds outside a configurable speed-error deadband, with a minimum switching interval. Relay control is simple but can produce speed ripple.
- **PWM:** proportional error-to-duty mapping produces a duty in [0, 1]. An actual square-wave carrier drives the model; inspect `pwm_on` for switching and `duty` for its envelope. This is switched actuation rather than an averaged-voltage approximation.
- **PID:** discrete proportional, integral and filtered derivative terms. Conditional integration blocks windup when saturation and error point farther into the limit. Gains are configurable with `--kp`, `--ki`, and `--kd`.

The actuator uses normalized command limits [0,1]. `raw_command` and `command` are both retained. Controller saturation time measures intervals where the raw request was clamped; relay/PWM laws are intrinsically bounded.

## Install on Windows

Use Python 3.10 or newer. To set up the complete project on Windows, double-click `run_project.bat` from the repository folder. It finds an available Python installation, creates `.venv` once, and installs the dashboard, test and PyBullet packages. If you prefer manual setup in PowerShell:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".[dev,visual]"
```

If PowerShell blocks activation, call `.venv\Scripts\python.exe` directly. Core numerical use needs Matplotlib and Rich. PyBullet is optional for the visualization; on Windows, if pip needs to build it from source, install Microsoft C++ Build Tools first. Pytest is installed by the `dev` extra.

## Run

```powershell
python -m conveyor_control.cli
python -m conveyor_control.cli --controller pid --scenario baseline
python -m conveyor_control.cli --controller pid --scenario load_step --kp 0.7 --ki 0.8 --kd 0.01 --load-torque 0.2
python -m conveyor_control.cli --controller pwm --scenario saturation --pwm-frequency 40
python -m conveyor_control.cli --pwm-sweep
python -m conveyor_control.cli --compare
python -m conveyor_control.cli --controller pid --visualize
```

The first command opens a menu. Choose **5** to run a controller in the PyBullet visual replay; choose **6** to exit. The comparison runs all three controllers over baseline, load-step, load-cycle, light/medium/heavy load, saturation and changing-reference scenarios. `--duration`, `--dt`, `--target`, `--load-torque`, `--pwm-frequency`, and `--results` configure the run. The visualization needs a desktop session with GUI support.

## Interface

The front end is the Rich-based Python command-line dashboard, which is what the assignment asks for. It has a styled menu and formatted metric tables. There is no web page or browser dashboard. PyBullet is the separate 3D visual scene; it is not a replacement for the control dashboard.

## Experiments and output

1. `baseline`: nominal constant speed target.
2. `load_step`: adds the configured load torque at 40% of run time.
3. `load_cycle`: increases load, then decreases it during operation.
4. `light_load`, `medium_load`, `heavy_load`: three reproducible steady load levels.
5. `saturation`: requests 115% of the configured maximum belt speed.
6. `changing_reference`: requests 55%, 100%, then 70% of nominal target.

The comparison writes `results/csv/<scenario>_<controller>.csv`, per-run JSON metrics, FFT data, a PWM frequency sweep, comparison and robustness plots, aggregate `results/metrics.csv`, `results/interpretation.md`, report-ready `results/report.md`, and analytical plant/PID Bode data. Each scenario CSV is raw per-step simulation output. Use `--frequency` to generate only the Bode response, or `--pwm-sweep` to run the carrier-frequency experiment separately.

Metrics: IAE, ISE, MAE, RMSE, maximum absolute and steady-state error, rise and settling time, overshoot, peak speed, load-step speed drop/recovery, speed ripple, duty, switching count, control effort and saturation duration/percentage. Settling is the first point after which all remaining samples stay inside 2% of the final target. FFT analysis removes a linear trend from the final 25% of uniformly sampled speed data, applies a Hann window, and calculates a one-sided spectrum; its Nyquist frequency comes from the logged sample interval. The Bode plot compares the linear plant and an ideal continuous PID closed-loop model; the report also gives their pole locations. Those checks do not prove stability for the switched/saturated nonlinear implementation.

## Assignment alignment and report support

- Time-domain analysis: speed, error, raw/applied command and saturation/load traces plus tracking and disturbance-recovery metrics are in `results/`.
- Frequency-domain analysis: analytical Bode response, measured FFT ripple spectra, and a multi-rate PWM frequency test are generated in `results/plots/` and `results/csv/`.
- Constraints and actuation: inspect raw/applied command and PWM duty/switching columns in scenario CSVs; `saturation` demonstrates an infeasible speed request.
- Stability: the model has stable open-loop pole `−b/J`; bounded simulation traces show the tested responses but do not prove nonlinear closed-loop stability.
- Faculty demo checklist: run each controller, compare all four scenarios, show CSV/plots/report, then demonstrate the synchronized PyBullet roller if PyBullet is installed.
- Project contribution: the standard controllers are not novel. The contribution is the reproducible same-plant harness, visible PWM carrier, disturbance/saturation runs and automatic comparison report.
- Presentation: organize a maximum of 17 slides around the problem, architecture, model, controllers, constraints, experiment method, baseline/load/saturation results, frequency response, limits and conclusion.

The frequency response includes the linearized plant transfer function `Ω(s)/U(s)=K/(Js+b)` and an ideal, unsaturated continuous PID closed-loop response; it is not robustness analysis of the switched, saturated controllers. A formal nonlinear stability proof is not included. The genuine engineering contribution is the reproducible comparison harness, explicit PWM switching evidence, combined disturbance and saturation experiments, and generated interpretations—not the standard control laws themselves.

## Exact first-time run

The easiest way is to double-click `run_project.bat` in the repository folder. On its first run it creates `.venv`, installs the dashboard and PyBullet dependencies, and opens the menu. Later runs open the menu directly. You can also pass CLI options to it, such as `run_project.bat --compare`.

If you prefer PowerShell, open it in this repository folder and run these commands one at a time:

```powershell
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".[dev,visual]"
python -m conveyor_control.cli
```

At the menu, choose `1`, then `pid`, then `baseline`. To run the full comparison directly, use:

```powershell
python -m conveyor_control.cli --compare
```

To see one complete example without the menu:

```powershell
python -m conveyor_control.cli --controller pid --scenario load_step
```

The full comparison creates CSVs, metrics, plots, interpretations and a report under `results/`. To open the PyBullet visual scene, install the `visual` extra above (or `python -m pip install pybullet`), then add `--visualize` to a single-run command. If pip asks to build PyBullet from source, Microsoft C++ Build Tools are required. On this computer, Windows Application Control blocks PyBullet's native extension, so the GUI cannot currently start here. That Windows policy must allow the extension before the 3D replay can run; the numerical simulation and comparisons do not require PyBullet.

## Tests

```powershell
python -m pytest
```

Tests cover motor acceleration/load/limits, controller switching and duty, PID saturation and anti-windup, metrics, repeatability, zero target and invalid inputs.

## Limitations and future work

The plant omits armature inductance/current, back-EMF electrical dynamics, gearing, belt elasticity, slip, thermal effects and a physical sensor model beyond optional seeded Gaussian noise. PyBullet uses a kinematically synchronized joint and kinematic boxes; its rigid-body state is not coupled back to the Python plant. Formal nonlinear closed-loop stability analysis, recorded PyBullet screenshots and measured hardware validation remain future work.

## Suggested live demo

1. Run `--compare` and show generated metric/interpretation files.
2. Compare baseline speed, command and duty traces, highlighting the actual PWM pulse field.
3. Run the load-step comparison and point out the disturbance timestamp in the CSV.
4. Run the saturation scenario and explain the raw versus applied command.
5. Launch one PID run with `--visualize` to show the synchronized roller and moving boxes.

## Project structure

```text
src/conveyor_control/  model, controllers, simulation, metrics, results, CLI, visualization
tests/                 unit and behavior tests
results/               generated CSVs, metrics, interpretations, report and plots
```

## Git commands

After reviewing the generated results, commit locally with:

```powershell
git add README.md .gitignore pyproject.toml requirements.txt main.py run_project.bat src tests results
git commit -m "Build conveyor motor control simulation"
git push origin main
```

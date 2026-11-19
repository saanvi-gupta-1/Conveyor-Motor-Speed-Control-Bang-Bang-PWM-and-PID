"""Controller constructors and reproducible experiment-suite runner."""

from .controllers import BangBangController, PWMController, PIDController
from .simulation import SimulationConfig, run_simulation
from .metrics import calculate_metrics
from .spectrum import analyze_speed_spectrum


def make_controller(name: str, kp: float = 0.7, ki: float = 0.8, kd: float = 0.01,
                    pwm_frequency_hz: float = 20.0, limits: tuple[float, float] = (0.0, 1.0),
                    deadband: float = .12):
    constructors = {"bang-bang": lambda: BangBangController(deadband=deadband, limits=limits),
                    "pwm": lambda: PWMController(frequency_hz=pwm_frequency_hz, limits=limits),
                    "pid": lambda: PIDController(kp=kp, ki=ki, kd=kd, limits=limits),
                    "bang-bang-custom": lambda: BangBangController(deadband=deadband, limits=limits),
                    "pwm-custom": lambda: PWMController(frequency_hz=pwm_frequency_hz, limits=limits)}
    try:
        return constructors[name.lower()]()
    except KeyError as exc:
        raise ValueError(f"Unknown controller: {name}") from exc


def run_suite(config: SimulationConfig | None = None) -> dict[str, dict[str, object]]:
    output: dict[str, dict[str, object]] = {}
    for scenario in ("baseline", "load_step", "load_cycle", "light_load", "medium_load", "heavy_load", "saturation", "changing_reference"):
        output[scenario] = {}
        for name in ("bang-bang", "pwm", "pid"):
            cfg = config or SimulationConfig()
            controller = make_controller(name, pwm_frequency_hz=cfg.pwm_frequency_hz,
                                         limits=(cfg.actuator_min, cfg.actuator_max),
                                         deadband=cfg.bangbang_deadband)
            rows = run_simulation(controller, scenario, config)
            metrics = calculate_metrics(rows)
            spectrum = analyze_speed_spectrum(rows)
            metrics.update({"dominant_ripple_frequency_hz": spectrum["dominant_frequency_hz"],
                            "dominant_ripple_amplitude_m_s": spectrum["dominant_amplitude_m_s"]})
            output[scenario][name] = {"rows": rows, "metrics": metrics, "spectrum": spectrum}
    return output


def run_pwm_frequency_sweep(config: SimulationConfig | None = None,
                            frequencies: tuple[float, ...] = (5.0, 10.0, 20.0, 40.0)) -> dict[float, dict[str, object]]:
    """Compare PWM carrier rates under the same nominal reference and plant."""
    cfg = config or SimulationConfig()
    output: dict[float, dict[str, object]] = {}
    for frequency in frequencies:
        if frequency >= .5 / cfg.dt:
            continue
        controller = make_controller("pwm-custom", pwm_frequency_hz=frequency,
                                     limits=(cfg.actuator_min, cfg.actuator_max),
                                     deadband=cfg.bangbang_deadband)
        rows = run_simulation(controller, "baseline", cfg)
        metrics = calculate_metrics(rows)
        spectrum = analyze_speed_spectrum(rows)
        metrics.update({"dominant_ripple_frequency_hz": spectrum["dominant_frequency_hz"],
                        "dominant_ripple_amplitude_m_s": spectrum["dominant_amplitude_m_s"]})
        output[frequency] = {"rows": rows, "metrics": metrics, "spectrum": spectrum}
    if len(output) < 2:
        raise ValueError("PWM frequency sweep requires at least two frequencies below Nyquist")
    return output

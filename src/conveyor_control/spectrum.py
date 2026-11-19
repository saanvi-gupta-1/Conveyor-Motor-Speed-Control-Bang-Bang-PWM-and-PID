"""FFT analysis of steady-tail belt-speed ripple."""

import numpy as np


def analyze_speed_spectrum(rows: list[dict[str, float | str]], tail_fraction: float = .25):
    """Return one-sided Hann-windowed amplitude spectrum for the final trace segment."""
    if len(rows) < 16 or not 0 < tail_fraction <= 1:
        raise ValueError("FFT needs at least 16 samples and a tail fraction in (0, 1]")
    times = np.asarray([float(row["time_s"]) for row in rows], dtype=float)
    speeds = np.asarray([float(row["speed_m_s"]) for row in rows], dtype=float)
    dt = float(np.mean(np.diff(times)))
    if dt <= 0 or not np.allclose(np.diff(times), dt, rtol=1e-6, atol=1e-10):
        raise ValueError("FFT samples must be uniformly spaced")
    start = max(0, int(len(speeds) * (1 - tail_fraction)))
    values = speeds[start:]
    sample_axis = np.arange(len(values), dtype=float)
    trend = np.polyval(np.polyfit(sample_axis, values, 1), sample_axis)
    values = values - trend
    window = np.hanning(len(values))
    spectrum = np.fft.rfft(values * window)
    frequencies = np.fft.rfftfreq(len(values), d=dt)
    amplitudes = np.abs(spectrum) * (2.0 / max(window.sum(), 1e-15))
    if len(amplitudes) > 1:
        amplitudes[-1] *= .5 if len(values) % 2 == 0 else 1.0
    ripple_rms = float(np.std(values))
    dominant_index = int(np.argmax(amplitudes[1:]) + 1) if len(amplitudes) > 1 else 0
    dominant_frequency = float(frequencies[dominant_index]) if ripple_rms >= 1e-9 else float("nan")
    return {
        "rows": [{"frequency_hz": float(f), "amplitude_m_s": float(a)} for f, a in zip(frequencies, amplitudes)],
        "dominant_frequency_hz": dominant_frequency,
        "dominant_amplitude_m_s": float(amplitudes[dominant_index]) if ripple_rms >= 1e-9 else 0.0,
        "segment_start_s": float(times[start]),
        "segment_duration_s": float((len(values) - 1) * dt),
        "sampling_interval_s": dt,
        "nyquist_frequency_hz": .5 / dt,
        "speed_ripple_rms_m_s": ripple_rms,
        "speed_ripple_peak_to_peak_m_s": float(np.ptp(values)),
    }

import math
import pytest

from conveyor_control.spectrum import analyze_speed_spectrum


def test_fft_recovers_sampled_sine_frequency_and_nyquist():
    dt = .005
    frequency = 20.0
    rows = [{"time_s": i * dt, "speed_m_s": 1 + .02 * math.sin(2 * math.pi * frequency * i * dt)}
            for i in range(2000)]
    result = analyze_speed_spectrum(rows, tail_fraction=.5)
    assert result["dominant_frequency_hz"] == pytest.approx(frequency, abs=.1)
    assert result["sampling_interval_s"] == pytest.approx(dt)
    assert result["nyquist_frequency_hz"] == pytest.approx(100)

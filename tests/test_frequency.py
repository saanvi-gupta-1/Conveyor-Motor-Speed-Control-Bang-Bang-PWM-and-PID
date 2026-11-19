import math

from conveyor_control.frequency import motor_frequency_response
from conveyor_control.model import MotorParameters


def test_analytical_motor_frequency_response():
    p = MotorParameters()
    rows = motor_frequency_response(p, points=20)
    assert len(rows) == 20
    assert rows[0]["magnitude_rad_s_per_unit"] > rows[-1]["magnitude_rad_s_per_unit"]
    assert -90 < rows[-1]["phase_deg"] < 0
    assert rows[0]["magnitude_db"] == 20 * math.log10(rows[0]["magnitude_rad_s_per_unit"])
    assert math.isfinite(rows[0]["pid_closed_loop_magnitude_db"])

from .bang_bang import BangBangController
from .pwm import PWMController
from .pid import PIDController


def get_controller(name: str):
    """Factory helper to instantiate a controller by name."""
    clean = name.strip().lower()
    if clean in ("bang", "bangbang", "bang-bang", "1"):
        return BangBangController()
    elif clean in ("pwm", "2"):
        return PWMController()
    elif clean in ("pid", "3"):
        return PIDController()
    else:
        raise ValueError(f"Unknown controller '{name}'. Choose: 'bang', 'pwm', or 'pid'")

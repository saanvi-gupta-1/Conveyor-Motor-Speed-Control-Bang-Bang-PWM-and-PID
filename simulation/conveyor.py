"""
simulation/conveyor.py
==============================================================================
Conveyor Belt Physical Representation & Dynamic Cargo Box Payload Tracking.

Key Physics:
  - Belt kinematics: x(t) = x_prev + v_belt * dt
  - Box states: "on_belt" -> "falling" (chute deflector arc) -> "in_bin" (resting in collection hopper)
  - Dynamic kinetic slip friction during initial drop phase (mu = 0.65 for 1.0s)
    settling to steady transport friction (mu = 0.30)
  - Reflected moment of inertia: J_reflected = (sum of active box masses) * r_pulley^2
  - Reliable discharge into end collection hopper bin regardless of belt speed
==============================================================================
"""

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class CargoBox:
    """Represents an individual physical box on the conveyor belt or in the collection bin."""
    id: int
    name: str
    mass: float          # Mass in kilograms [kg]
    x_pos: float         # Linear position along belt X [meters]
    y_pos: float = 0.0   # Lateral position Y [meters]
    z_pos: float = 0.695 # Vertical height Z [meters] (rests on belt)
    vx: float = 0.0      # Velocity X [m/s]
    vz: float = 0.0      # Velocity Z [m/s]
    state: str = "on_belt"  # "on_belt" -> "falling" -> "in_bin"
    age: float = 0.0     # Time elapsed since spawn [s]
    bin_time: float = 0.0 # Time resting in bin [s]
    color_rgb: tuple = (0.9, 0.5, 0.2)


class ConveyorBeltSystem:
    """
    Manages conveyor geometry, physical box tracking, payload reflections,
    and end-of-belt discharge projectile physics into the collection hopper.
    """
    BELT_LENGTH = 3.80   # Conveyor length [meters] (generous transit viewing distance)
    BELT_WIDTH  = 0.55   # Conveyor width [meters]
    PULLEY_RAD  = 0.05   # Drive pulley radius [meters] (5 cm)
    GRAVITY     = 9.81   # Gravity [m/s^2]
    MAX_BOXES   = 6      # Maximum active boxes on belt simultaneously

    # Standard cargo box presets with distinct, tangible disturbance loads
    PRESETS = {
        "light":  {"name": "Light Box (1.0kg)", "mass": 1.0, "color": (0.95, 0.82, 0.18)},
        "medium": {"name": "Med Box (3.5kg)",   "mass": 3.5, "color": (0.18, 0.82, 0.32)},
        "heavy":  {"name": "Heavy Box (8.5kg)",  "mass": 8.5, "color": (0.92, 0.22, 0.18)},
    }

    def __init__(self):
        self.boxes: List[CargoBox] = []
        self._next_id = 1
        self.belt_travel_distance = 0.0
        self.reset()

    def reset(self):
        """Reset conveyor to nominal 1 medium cargo box spaced on the belt."""
        self.boxes.clear()
        self.belt_travel_distance = 0.0
        self._next_id = 1

        # Spawn 1 initial box that will travel along belt and fall into bin
        self.add_box("Box-Green (3.5kg)", 3.5, init_x=1.20, color=(0.18, 0.82, 0.32))

    def add_box(self, name: str, mass: float, init_x: float = None, color: tuple = (0.9, 0.5, 0.2)) -> CargoBox:
        """Add a cargo box onto the conveyor belt."""
        if init_x is None:
            spawn_x = 0.28
            for b in self.boxes:
                if b.state == "on_belt" and abs(b.x_pos - spawn_x) < 0.40:
                    spawn_x = max(spawn_x, b.x_pos + 0.40)
            init_x = min(spawn_x, self.BELT_LENGTH - 0.60)

        box = CargoBox(
            id=self._next_id,
            name=str(name),
            mass=float(mass),
            x_pos=float(init_x),
            y_pos=0.0,
            z_pos=0.695,
            color_rgb=color,
        )
        self._next_id += 1
        self.boxes.append(box)
        return box

    def add_preset_box(self, preset_name: str) -> Optional[CargoBox]:
        """Convenience helper to spawn a Light, Medium, or Heavy box at entrance."""
        key = preset_name.strip().lower()
        if key in self.PRESETS:
            spec = self.PRESETS[key]
            return self.add_box(spec["name"], spec["mass"], init_x=None, color=spec["color"])
        return None

    def remove_box(self) -> Optional[CargoBox]:
        """Remove the most recently added box."""
        if self.boxes:
            return self.boxes.pop()
        return None

    def clear_all_boxes(self):
        """Remove all boxes from the system."""
        self.boxes.clear()

    @property
    def active_belt_boxes(self) -> List[CargoBox]:
        """Boxes currently riding the belt (exerting load on the motor)."""
        return [b for b in self.boxes if b.state == "on_belt"]

    @property
    def in_bin_boxes(self) -> List[CargoBox]:
        """Boxes currently resting inside the collection hopper bin."""
        return [b for b in self.boxes if b.state == "in_bin"]

    @property
    def total_mass(self) -> float:
        """Total mass of active boxes currently resting on belt [kg]."""
        return sum(b.mass for b in self.active_belt_boxes)

    @property
    def active_box_count(self) -> int:
        return len(self.active_belt_boxes)

    @property
    def in_bin_count(self) -> int:
        return len(self.in_bin_boxes)

    FRICTION_COEFF = 0.35  # Constant physical coulomb friction coefficient on conveyor bed

    @property
    def payload_friction_torque(self) -> float:
        """
        Calculates constant physical friction drag torque on the drive pulley.
        T_load = mu * (sum m_i) * g * r_pulley
        - Empty belt: 0.00 N*m
        - Light (1.0kg): 0.17 N*m
        - Medium (3.5kg): 0.60 N*m
        - Heavy (8.5kg): 1.46 N*m
        """
        return sum(self.FRICTION_COEFF * b.mass * self.GRAVITY * self.PULLEY_RAD for b in self.active_belt_boxes)

    @property
    def payload_reflected_inertia(self) -> float:
        """Reflected moment of inertia of active boxes on belt [kg*m^2]."""
        return self.total_mass * (self.PULLEY_RAD ** 2)

    def update(self, belt_speed: float, dt: float) -> list:
        """
        Advance conveyor belt surface and cargo boxes according to belt speed.
        Boxes reaching the discharge end fall off the conveyor in a deflector chute arc
        and land safely inside the collection hopper bin.

        Returns:
            List of CargoBox objects that rolled off the roller during this step.
        """
        displacement = belt_speed * dt
        self.belt_travel_distance += displacement

        discharged_events = []
        surviving_boxes = []
        bin_count = sum(1 for b in self.boxes if b.state == "in_bin")

        for box in self.boxes:
            box.age += dt

            # 1. State: Riding on top of the conveyor belt (forward towards collection hopper)
            if box.state == "on_belt":
                box.x_pos = max(0.15, box.x_pos + max(0.0, displacement))
                box.vx = max(0.0, belt_speed)
                box.z_pos = 0.695
                # When box reaches end of conveyor (roller edge), it discharges!
                if box.x_pos >= (self.BELT_LENGTH - 0.05):
                    box.state = "falling"
                    # Forward velocity directed into collection hopper (capped to prevent overshoot)
                    box.vx = min(1.80, max(0.60, belt_speed))
                    box.vz = 0.0
                    discharged_events.append(box)
                surviving_boxes.append(box)

            # 2. State: Falling in air / deflecting into the collection hopper
            elif box.state == "falling":
                box.vz += self.GRAVITY * dt
                box.z_pos -= box.vz * dt
                box.x_pos += box.vx * dt

                # Deflector backstop wall catches fast boxes at X = 4.45m
                if box.x_pos >= 4.45:
                    box.x_pos = 4.45
                    box.vx = 0.0

                # Landing on the floor of the collection bin (Z = 0.145m)
                if box.z_pos <= 0.145:
                    box.state = "in_bin"
                    box.z_pos = 0.145
                    box.vx = 0.0
                    box.vz = 0.0
                    # Place cleanly within hopper floor bounds
                    box.x_pos = min(4.40, max(3.95, box.x_pos))
                surviving_boxes.append(box)

            # 3. State: Resting safely inside the collection bin
            elif box.state == "in_bin":
                box.bin_time += dt
                # Keep resting in bin unless too many boxes accumulate (>5) or older than 15s
                if bin_count > 5 or box.bin_time > 15.0:
                    bin_count -= 1
                else:
                    surviving_boxes.append(box)

        self.boxes = surviving_boxes
        return discharged_events

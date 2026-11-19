"""
simulation/conveyor_scene.py
==============================================================================
PyBullet 3D Conveyor Simulation with Interactive GUI Sliders & Action Buttons.

Visual & Physical Elements:
  - Checkered industrial floor
  - Conveyor bed with safety yellow side guide rails
  - Steel head and tail pulleys (rollers)
  - Blue electric DC drive motor mounted on head roller with coupler
  - End collection hopper bin (green base with deflector walls)
  - High-contrast animated belt slats moving in real time with belt velocity
  - Physical cargo boxes synchronized with ConveyorBeltSystem (belt -> falling -> bin)
  - 3D physical scoreboard billboard panel behind conveyor
  - Right-hand "Params" panel matching academic reference layout:
      * Target Speed slider (0 to 3 m/s)
      * Controller selection slider (0:PID, 1:PWM, 2:Bang-Bang)
      * Kp, Ki, Kd Gain sliders
      * Add Light (1.0kg), Add Medium (3.5kg), Add Heavy (8.5kg) buttons
      * Reset Conveyor and Plot Telemetry buttons
  - Safe disconnect protection (handles window close gracefully)
==============================================================================
"""

import math
import random
import pybullet as pb
import pybullet_data


class ConveyorScene3D:
    # Physical dimensions
    BELT_LENGTH = 3.80   # Length [m] (generous transit viewing distance)
    BELT_WIDTH  = 0.55   # Width [m]
    BELT_HEIGHT = 0.62   # Height from floor [m]
    BELT_THICK  = 0.02   # Half-thickness [m]
    PULLEY_RAD  = 0.075  # Roller radius [m]
    PULLEY_LEN  = 0.60   # Roller length [m]
    RAIL_HEIGHT = 0.08   # Side rail height [m]
    BOX_HALF    = 0.075  # Box half-size [m]
    N_SLATS     = 18     # Animated slats count

    # Color Palette
    COLOR_BELT     = [0.18, 0.18, 0.20, 1.0]
    COLOR_SLAT     = [0.85, 0.85, 0.90, 0.80]
    COLOR_FRAME    = [0.12, 0.12, 0.14, 1.0]
    COLOR_RAIL     = [0.88, 0.65, 0.05, 1.0]  # Safety yellow
    COLOR_ROLLER   = [0.75, 0.78, 0.82, 1.0]  # Polished steel
    COLOR_MOTOR    = [0.12, 0.38, 0.85, 1.0]  # Industrial blue
    COLOR_SHAFT    = [0.60, 0.62, 0.68, 1.0]  # Steel shaft coupling
    COLOR_BIN_BASE = [0.15, 0.55, 0.25, 1.0]  # Green collector base
    COLOR_BIN_WALL = [0.12, 0.45, 0.22, 1.0]  # Green collector walls
    COLOR_CHUTE    = [0.55, 0.58, 0.62, 1.0]  # Metal guide chute
    COLOR_BOARD_BG = [0.07, 0.09, 0.13, 0.98] # Sleek dark matte panel
    COLOR_BOARD_TRIM = [0.30, 0.35, 0.45, 1.0] # Metallic trim

    def __init__(self, enable_gui: bool = True):
        self.enable_gui = enable_gui
        self.client_id = None
        self.belt_id = None
        self.slat_ids = []
        self.slat_xs = []
        self.box_body_map = {}  # Maps CargoBox.id -> PyBullet body ID
        self.hud_ids = {}
        self.slat_spacing = self.BELT_LENGTH / self.N_SLATS

        # GUI Parameter handles
        self.param_speed = None
        self.param_ctrl  = None
        self.param_kp    = None
        self.param_ki    = None
        self.param_kd    = None
        self.btn_light   = None
        self.btn_medium  = None
        self.btn_heavy   = None
        self.btn_reset   = None
        self.btn_plot    = None
        self.btn_compare = None

        # Debounce counters
        self.last_btn_light   = 0
        self.last_btn_medium  = 0
        self.last_btn_heavy   = 0
        self.last_btn_reset   = 0
        self.last_btn_plot    = 0
        self.last_btn_compare = 0

        # Cached values & smoothed display filters to eliminate jitter blur
        self.cached_speed = 1.0
        self.cached_ctrl_idx = 0
        self.cached_kp = 50.0
        self.cached_ki = 40.0
        self.cached_kd = 0.5
        self._hud_tick = 0
        self.disp_speed = 0.0
        self.disp_volt  = 0.0
        self.disp_curr  = 0.0
        self.disp_load  = 0.0

        self._setup_physics()
        self._build_conveyor()
        self._build_hud()
        if self.enable_gui:
            self._setup_gui_controls()

    def is_connected(self) -> bool:
        """Safe check to verify physics server is still active."""
        try:
            return bool(pb.isConnected())
        except Exception:
            return False

    def _setup_physics(self):
        mode = pb.GUI if self.enable_gui else pb.DIRECT
        self.client_id = pb.connect(mode)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0, 0, -9.81)
        pb.setTimeStep(1.0 / 240.0)

        # Checkered floor
        floor = pb.loadURDF("plane.urdf")
        pb.changeDynamics(floor, -1, lateralFriction=0.6, restitution=0.1)

        if self.enable_gui:
            pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 1)
            pb.configureDebugVisualizer(pb.COV_ENABLE_SHADOWS, 1)
            # Frame camera from left to right: Start (X=0) on Left, Discharge/Hopper on Right
            pb.resetDebugVisualizerCamera(
                cameraDistance=4.7,
                cameraYaw=-20.0,
                cameraPitch=-24.0,
                cameraTargetPosition=[2.05, 0.05, 0.55],
            )

    def _setup_gui_controls(self):
        """Create interactive sliders and action buttons with clear, unclipped labels."""
        self.param_speed = pb.addUserDebugParameter("Target Spd (0-3 m/s)", 0.0, 3.0, 1.0)
        self.param_ctrl  = pb.addUserDebugParameter("Mode (0:PID 1:PWM 2:BB)", 0.0, 2.0, 0.0)
        self.param_kp    = pb.addUserDebugParameter("Kp Gain (0-100)", 0.0, 100.0, 50.0)
        self.param_ki    = pb.addUserDebugParameter("Ki Gain (0-100)", 0.0, 100.0, 40.0)
        self.param_kd    = pb.addUserDebugParameter("Kd Gain (0-5)", 0.0, 5.0, 0.5)

        self.btn_light   = pb.addUserDebugParameter("+ Light Box (1kg)", 1, 0, 0)
        self.btn_medium  = pb.addUserDebugParameter("+ Med Box (3.5kg)", 1, 0, 0)
        self.btn_heavy   = pb.addUserDebugParameter("+ Heavy Box (8.5kg)", 1, 0, 0)
        self.btn_reset   = pb.addUserDebugParameter("Reset Conveyor", 1, 0, 0)
        self.btn_plot    = pb.addUserDebugParameter("Plot Telemetry", 1, 0, 0)
        self.btn_compare = pb.addUserDebugParameter("Plot 3-Ctrl Compare", 1, 0, 0)

    def read_gui_inputs(self) -> dict:
        """Poll PyBullet sliders and buttons with debouncing."""
        if not self.enable_gui or self.param_speed is None or not self.is_connected():
            return {}

        events = {}
        try:
            # Sliders
            spd = round(float(pb.readUserDebugParameter(self.param_speed)), 3)
            if abs(spd - self.cached_speed) >= 0.005:
                self.cached_speed = spd
                events["speed"] = spd

            ctrl_raw = int(round(float(pb.readUserDebugParameter(self.param_ctrl))))
            ctrl_idx = max(0, min(2, ctrl_raw))
            if ctrl_idx != self.cached_ctrl_idx:
                self.cached_ctrl_idx = ctrl_idx
                ctrl_map = {0: "pid", 1: "pwm", 2: "bang"}
                events["ctrl"] = ctrl_map[ctrl_idx]

            kp = round(float(pb.readUserDebugParameter(self.param_kp)), 2)
            if abs(kp - self.cached_kp) >= 0.1:
                self.cached_kp = kp
                events["kp"] = kp

            ki = round(float(pb.readUserDebugParameter(self.param_ki)), 2)
            if abs(ki - self.cached_ki) >= 0.05:
                self.cached_ki = ki
                events["ki"] = ki

            kd = round(float(pb.readUserDebugParameter(self.param_kd)), 3)
            if abs(kd - self.cached_kd) >= 0.01:
                self.cached_kd = kd
                events["kd"] = kd

            # Buttons
            c_light = int(pb.readUserDebugParameter(self.btn_light))
            if c_light > self.last_btn_light:
                self.last_btn_light = c_light
                events["add_light"] = True

            c_med = int(pb.readUserDebugParameter(self.btn_medium))
            if c_med > self.last_btn_medium:
                self.last_btn_medium = c_med
                events["add_medium"] = True

            c_hvy = int(pb.readUserDebugParameter(self.btn_heavy))
            if c_hvy > self.last_btn_heavy:
                self.last_btn_heavy = c_hvy
                events["add_heavy"] = True

            c_rst = int(pb.readUserDebugParameter(self.btn_reset))
            if c_rst > self.last_btn_reset:
                self.last_btn_reset = c_rst
                events["reset"] = True

            c_plt = int(pb.readUserDebugParameter(self.btn_plot))
            if c_plt > self.last_btn_plot:
                self.last_btn_plot = c_plt
                events["plot"] = True

            c_cmp = int(pb.readUserDebugParameter(self.btn_compare))
            if c_cmp > self.last_btn_compare:
                self.last_btn_compare = c_cmp
                events["compare_plot"] = True

        except Exception:
            pass

        return events

    def _make_box(self, half_extents, rgba, pos, mass=0.0):
        c = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=half_extents)
        v = pb.createVisualShape(pb.GEOM_BOX, halfExtents=half_extents, rgbaColor=rgba)
        return pb.createMultiBody(baseMass=mass, baseCollisionShapeIndex=c, baseVisualShapeIndex=v, basePosition=pos)

    def _make_cylinder(self, radius, length, rgba, pos, orn):
        c = pb.createCollisionShape(pb.GEOM_CYLINDER, radius=radius, height=length)
        v = pb.createVisualShape(pb.GEOM_CYLINDER, radius=radius, length=length, rgbaColor=rgba)
        return pb.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=c, baseVisualShapeIndex=v, basePosition=pos, baseOrientation=orn)

    def _build_conveyor(self):
        bh = self.BELT_HEIGHT
        cx = self.BELT_LENGTH / 2.0

        # Conveyor Bed
        self.belt_id = self._make_box(
            [self.BELT_LENGTH / 2.0, self.BELT_WIDTH / 2.0, self.BELT_THICK],
            self.COLOR_BELT,
            [cx, 0.0, bh - self.BELT_THICK],
        )

        # Rollers (Silver steel cylinders)
        q_roller = pb.getQuaternionFromEuler([math.pi / 2.0, 0, 0])
        roller_z = bh - self.BELT_THICK
        self._make_cylinder(self.PULLEY_RAD, self.PULLEY_LEN, self.COLOR_ROLLER, [self.PULLEY_RAD, 0.0, roller_z], q_roller)
        self._make_cylinder(self.PULLEY_RAD, self.PULLEY_LEN, self.COLOR_ROLLER, [self.BELT_LENGTH - self.PULLEY_RAD, 0.0, roller_z], q_roller)

        # Frame Legs (supporting posts to floor)
        leg_half_w = 0.04
        leg_h = (bh - 2 * self.BELT_THICK)
        for lx in [0.20, cx, self.BELT_LENGTH - 0.20]:
            for ly in [-(self.BELT_WIDTH / 2.0 - 0.04), (self.BELT_WIDTH / 2.0 - 0.04)]:
                self._make_box([leg_half_w, leg_half_w, leg_h / 2.0], self.COLOR_FRAME, [lx, ly, leg_h / 2.0])

        # Safety Yellow Side Guide Rails
        rail_th = 0.018
        rail_h = self.RAIL_HEIGHT / 2.0
        for ry in [-(self.BELT_WIDTH / 2.0 + rail_th), (self.BELT_WIDTH / 2.0 + rail_th)]:
            self._make_box([self.BELT_LENGTH / 2.0, rail_th, rail_h], self.COLOR_RAIL, [cx, ry, bh + rail_h])

        # Blue Electric DC Drive Motor mounted on side of head drive roller shaft
        q_motor = pb.getQuaternionFromEuler([math.pi / 2.0, 0, 0])
        motor_x = self.BELT_LENGTH - self.PULLEY_RAD
        motor_y = -(self.BELT_WIDTH / 2.0 + 0.16)
        # Drive shaft coupler
        self._make_cylinder(0.035, 0.08, self.COLOR_SHAFT, [motor_x, -(self.BELT_WIDTH / 2.0 + 0.04), roller_z], q_motor)
        # Motor Stator Body
        self._make_cylinder(0.095, 0.22, self.COLOR_MOTOR, [motor_x, motor_y, roller_z], q_motor)
        # Motor terminal box
        self._make_box([0.06, 0.06, 0.05], [0.10, 0.30, 0.70, 1.0], [motor_x, motor_y, roller_z + 0.10])

        # End Collection Hopper Bin (Positioned directly under and past discharge roller)
        bin_cx = self.BELT_LENGTH + 0.40  # 4.20m
        bin_half_x = 0.38                 # Spans 3.82m to 4.58m
        bin_half_y = 0.35                 # Spans -0.35m to +0.35m
        bin_floor_z = 0.04
        bin_wall_h = 0.20

        # Bin Floor Base (Industrial Green)
        self._make_box([bin_half_x, bin_half_y, 0.03], self.COLOR_BIN_BASE, [bin_cx, 0.0, bin_floor_z])

        # Tall Deflector Backstop Wall at far end (catches fast moving boxes!)
        backstop_h = 0.24
        self._make_box([0.02, bin_half_y, backstop_h], self.COLOR_BIN_WALL, [bin_cx + bin_half_x, 0.0, bin_floor_z + backstop_h])

        # Side Walls (Left and Right)
        for wy in [-bin_half_y, bin_half_y]:
            self._make_box([bin_half_x, 0.02, bin_wall_h], self.COLOR_BIN_WALL, [bin_cx, wy, bin_floor_z + bin_wall_h])

        # Sloped Discharge Chute Plate (Guides boxes off roller into the hopper)
        self._make_box([0.08, bin_half_y - 0.02, 0.015], self.COLOR_CHUTE, [self.BELT_LENGTH + 0.05, 0.0, bh - 0.18])

        # Animated Belt Slats
        for i in range(self.N_SLATS):
            x_pos = i * self.slat_spacing
            slat = self._make_box([0.02, self.BELT_WIDTH / 2.0 - 0.02, 0.002], self.COLOR_SLAT, [x_pos, 0.0, bh + 0.002])
            self.slat_ids.append(slat)
            self.slat_xs.append(x_pos)

    def _build_hud(self):
        """
        Build floating status text in pure BLACK color directly against the 3D scene.
        No blackboard or background obstacles are created.
        """
        if not self.enable_gui:
            return

        text_x = 0.70
        text_y = 1.05
        base_z = 1.15
        ts = 0.58
        black = [0.0, 0.0, 0.0]

        self.hud_ids["title"] = pb.addUserDebugText(
            "=== CONVEYOR MOTOR SPEED CONTROL (P15) ===",
            [text_x, text_y, base_z + 0.30],
            textColorRGB=black,
            textSize=ts,
        )
        self.hud_ids["ctrl"] = pb.addUserDebugText(
            "MODE: [ PID CONTROLLER ]   (Anti-Windup Active | 0:PID  1:PWM  2:BB)",
            [text_x, text_y, base_z + 0.15],
            textColorRGB=black,
            textSize=ts,
        )
        self.hud_ids["speed"] = pb.addUserDebugText(
            "TARGET:  1.00 m/s     ACTUAL:  1.00 m/s     ERR: +0.00 m/s",
            [text_x, text_y, base_z + 0.00],
            textColorRGB=black,
            textSize=ts,
        )
        self.hud_ids["telemetry"] = pb.addUserDebugText(
            "VOLTAGE: +13.2 V      CURRENT:  1.52 A      LOAD:  0.17 N*m",
            [text_x, text_y, base_z - 0.15],
            textColorRGB=black,
            textSize=ts,
        )
        self.hud_ids["cargo"] = pb.addUserDebugText(
            "CARGO: 0 on Belt (0.0 kg)   |   IN HOPPER: 0   |   STATUS: NORMAL",
            [text_x, text_y, base_z - 0.30],
            textColorRGB=black,
            textSize=ts,
        )

    def sync_boxes(self, physical_boxes: list):
        """
        Synchronize 3D rigid bodies with the CargoBox objects in ConveyorBeltSystem.
        Creates new 3D bodies when boxes are added, updates positions, and removes old ones.
        """
        if not self.is_connected():
            return

        active_ids = {b.id for b in physical_boxes}
        # Remove 3D bodies that are no longer in physical system
        for box_id in list(self.box_body_map.keys()):
            if box_id not in active_ids:
                pb_body_id = self.box_body_map.pop(box_id)
                try:
                    pb.removeBody(pb_body_id)
                except Exception:
                    pass

        # Update or create 3D bodies
        for b in physical_boxes:
            if b.id not in self.box_body_map:
                rgba = [b.color_rgb[0], b.color_rgb[1], b.color_rgb[2], 1.0]
                bid = self._make_box(
                    [self.BOX_HALF, self.BOX_HALF, self.BOX_HALF],
                    rgba,
                    [b.x_pos, b.y_pos, b.z_pos],
                    mass=0.0,  # Kinematic so position updates are 100% smooth without gravity jitter
                )
                self.box_body_map[b.id] = bid
            else:
                bid = self.box_body_map[b.id]
                try:
                    pb.resetBasePositionAndOrientation(
                        bid,
                        [b.x_pos, b.y_pos, b.z_pos],
                        [0, 0, 0, 1],
                    )
                except Exception:
                    pass

    def update(
        self,
        belt_speed: float,
        dt: float,
        target_speed: float,
        ctrl_name: str,
        voltage: float,
        load_nm: float,
        is_saturated: bool,
        current_a: float = 0.0,
        box_count: int = 0,
        total_mass: float = 0.0,
        in_bin_count: int = 0,
    ):
        """Update slat animations and HUD scoreboard telemetry."""
        if not self.is_connected():
            return

        # Animate slats
        for i in range(len(self.slat_ids)):
            new_x = self.slat_xs[i] + belt_speed * dt
            if new_x > (self.BELT_LENGTH - 0.05):
                new_x = 0.05 + (new_x - (self.BELT_LENGTH - 0.05))
            elif new_x < 0.05:
                new_x = (self.BELT_LENGTH - 0.05) - (0.05 - new_x)
            self.slat_xs[i] = new_x
            try:
                pb.resetBasePositionAndOrientation(
                    self.slat_ids[i],
                    [new_x, 0.0, self.BELT_HEIGHT + 0.002],
                    [0, 0, 0, 1],
                )
            except Exception:
                pass

        # Smooth display values with first-order filter to eliminate jitter/dancing blur
        self.disp_speed = 0.85 * self.disp_speed + 0.15 * belt_speed
        self.disp_volt  = 0.85 * self.disp_volt  + 0.15 * voltage
        self.disp_curr  = 0.85 * self.disp_curr  + 0.15 * current_a
        self.disp_load  = 0.85 * self.disp_load  + 0.15 * load_nm

        # Update floating status text in solid black at ~8 Hz (every 30 simulation steps at 240 Hz)
        self._hud_tick += 1
        if self.enable_gui and "title" in self.hud_ids and (self._hud_tick % 30 == 0):
            text_x = 0.70
            text_y = 1.05
            base_z = 1.15
            ts = 0.58
            black = [0.0, 0.0, 0.0]
            err = target_speed - self.disp_speed

            if ctrl_name.lower() == "pid":
                ctrl_text = "MODE: [ PID CONTROLLER ]   (Tuned Anti-Windup | 0:PID  1:PWM  2:BB)"
            elif ctrl_name.lower() == "pwm":
                ctrl_text = "MODE: [ PWM CONTROLLER ]   (Open-Loop Droop | 0:PID  1:PWM  2:BB)"
            else:
                ctrl_text = "MODE: [ BANG-BANG ]        (Relay On/Off Chatter | 0:PID  1:PWM  2:BB)"

            sat_str = "STATUS: SATURATED (+24V MAX)" if is_saturated else "STATUS: NORMAL (<=24V Limit)"

            try:
                pb.addUserDebugText(
                    ctrl_text,
                    [text_x, text_y, base_z + 0.15],
                    textColorRGB=black,
                    textSize=ts,
                    replaceItemUniqueId=self.hud_ids["ctrl"],
                )
                pb.addUserDebugText(
                    f"TARGET:  {target_speed:4.2f} m/s    ACTUAL:  {self.disp_speed:4.2f} m/s    ERR: {err:+5.2f} m/s",
                    [text_x, text_y, base_z + 0.00],
                    textColorRGB=black,
                    textSize=ts,
                    replaceItemUniqueId=self.hud_ids["speed"],
                )
                if "telemetry" in self.hud_ids:
                    pb.addUserDebugText(
                        f"VOLTAGE: {self.disp_volt:5.1f} V    CURRENT: {self.disp_curr:5.2f} A    LOAD: {self.disp_load:4.2f} N*m",
                        [text_x, text_y, base_z - 0.15],
                        textColorRGB=black,
                        textSize=ts,
                        replaceItemUniqueId=self.hud_ids["telemetry"],
                    )
                if "cargo" in self.hud_ids:
                    pb.addUserDebugText(
                        f"CARGO: {box_count} on Belt ({total_mass:4.1f} kg)   |   HOPPER: {in_bin_count}   |   {sat_str}",
                        [text_x, text_y, base_z - 0.30],
                        textColorRGB=black,
                        textSize=ts,
                        replaceItemUniqueId=self.hud_ids["cargo"],
                    )
            except Exception:
                pass

    def close(self):
        if self.client_id is not None:
            try:
                pb.disconnect(self.client_id)
            except Exception:
                pass
            self.client_id = None

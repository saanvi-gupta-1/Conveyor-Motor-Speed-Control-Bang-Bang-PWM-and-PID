"""PyBullet conveyor replay. Python owns control dynamics; Bullet is the visual twin."""

import math
import time


def create_conveyor_scene(p, pybullet_data, set_camera: bool = True) -> dict[str, object]:
    """Create the deck, driven roller revolute joint, motor housing and payload boxes."""
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    deck_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1.7, .38, .08])
    deck_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[1.7, .38, .08], rgbaColor=[.20, .23, .27, 1])
    deck = p.createMultiBody(0, deck_collision, deck_visual, [0, 0, .55])

    roller_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=.09, height=.82)
    roller_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=.09, length=.82, rgbaColor=[.62, .66, .70, 1])
    base_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.01, .01, .01])
    driven_roller = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=base_collision,
        basePosition=[-1.50, 0, .55], linkMasses=[.5], linkCollisionShapeIndices=[roller_collision],
        linkVisualShapeIndices=[roller_visual], linkPositions=[[0, 0, 0]],
        linkOrientations=[[math.sqrt(.5), 0, 0, math.sqrt(.5)]],
        linkInertialFramePositions=[[0, 0, 0]], linkInertialFrameOrientations=[[0, 0, 0, 1]],
        linkParentIndices=[0], linkJointTypes=[p.JOINT_REVOLUTE], linkJointAxis=[[0, 1, 0]])
    static_rollers = []
    for x in (1.50,):
        static_rollers.append(p.createMultiBody(0, roller_collision, roller_visual,
            [x, 0, .55], [math.sqrt(.5), 0, 0, math.sqrt(.5)]))

    motor_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=.16, length=.34, rgbaColor=[.18, .42, .72, 1])
    motor_housing = p.createMultiBody(0, -1, motor_visual, [1.88, 0, .57],
        [math.sqrt(.5), 0, 0, math.sqrt(.5)])

    box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.17, .18, .14])
    box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[.17, .18, .14], rgbaColor=[.90, .53, .17, 1])
    boxes = [p.createMultiBody(.8, box_collision, box_visual, [-1.05 - i * .72, 0, .80]) for i in range(3)]
    load_label = -1
    if set_camera:
        p.resetDebugVisualizerCamera(4.2, 35, -22, [0, 0, .55])
        p.addUserDebugText("Revolute roller synchronized to Python motor; boxes follow belt speed",
                           [0, -.63, 1.25], textColorRGB=[.1, .1, .1], textSize=1.2)
        load_label = p.addUserDebugText("Model load torque: 0.00 N m", [0, -.63, 1.13],
                                        textColorRGB=[.75, .18, .12], textSize=1.1)
    return {"deck": deck, "driven_roller": driven_roller, "static_rollers": static_rollers,
            "motor_housing": motor_housing, "boxes": boxes, "angle": 0.0,
            "load_label": load_label}


def advance_conveyor_scene(p, scene: dict[str, object], row: dict[str, float], dt: float,
                           speed_scale: float = 1.0) -> None:
    """Apply one numerical sample to the roller joint and kinematic belt payloads."""
    omega = row["omega_rad_s"]
    angle = float(scene["angle"]) - omega * dt
    scene["angle"] = angle
    roller = int(scene["driven_roller"])
    p.resetJointState(roller, 0, angle, targetVelocity=-omega)
    if int(scene.get("load_label", -1)) >= 0:
        p.addUserDebugText(f"Python model load torque: {row.get('load_torque_Nm', 0.0):.2f} N m",
                           [0, -.63, 1.13], textColorRGB=[.75, .18, .12], textSize=1.1,
                           replaceItemUniqueId=int(scene["load_label"]))
    for body in scene["boxes"]:
        position, _ = p.getBasePositionAndOrientation(body)
        x = position[0] + row["speed_m_s"] * dt * speed_scale
        if x > 1.35:
            x = -1.35
        p.resetBasePositionAndOrientation(body, [x, 0, .80], [0, 0, 0, 1])
        p.resetBaseVelocity(body, [row["speed_m_s"] * speed_scale, 0, 0], [0, 0, 0])
    p.stepSimulation()


def replay_pybullet(rows: list[dict[str, float]], speed_scale: float = 1.0) -> None:
    """Play a completed simulation in a PyBullet GUI until the user closes it."""
    if len(rows) < 2:
        raise ValueError("At least two simulation rows are required for visualization")
    try:
        import pybullet as p
        import pybullet_data
    except ImportError as exc:
        raise RuntimeError(f"PyBullet could not be imported ({exc}). Install it and allow its native extension to run to use the visual scene.") from exc
    cid = p.connect(p.GUI)
    if cid < 0:
        raise RuntimeError("PyBullet GUI could not be opened")
    try:
        p.setRealTimeSimulation(0)
        scene = create_conveyor_scene(p, pybullet_data)
        dt = rows[1]["time_s"] - rows[0]["time_s"]
        if dt <= 0:
            raise ValueError("Simulation timestamps must be strictly increasing")
        p.setTimeStep(dt)
        for row in rows:
            advance_conveyor_scene(p, scene, row, dt, speed_scale)
            time.sleep(dt)
        while p.isConnected(cid):
            p.stepSimulation(physicsClientId=cid)
            time.sleep(1 / 240)
    finally:
        if p.isConnected(cid):
            p.disconnect(cid)

import pytest

pybullet = pytest.importorskip("pybullet", exc_type=ImportError)
pybullet_data = pytest.importorskip("pybullet_data", exc_type=ImportError)

from conveyor_control.visualization import create_conveyor_scene, advance_conveyor_scene


def test_pybullet_scene_has_revolute_roller_and_moving_payload():
    client = pybullet.connect(pybullet.DIRECT)
    try:
        scene = create_conveyor_scene(pybullet, pybullet_data, set_camera=False)
        driven = scene["driven_roller"]
        assert pybullet.getNumJoints(driven) == 1
        assert pybullet.getJointInfo(driven, 0)[2] == pybullet.JOINT_REVOLUTE
        boxes = scene["boxes"]
        before, _ = pybullet.getBasePositionAndOrientation(boxes[0])
        row = {"omega_rad_s": 5.0, "speed_m_s": .4, "load_torque_Nm": .16}
        advance_conveyor_scene(pybullet, scene, row, .01)
        after, _ = pybullet.getBasePositionAndOrientation(boxes[0])
        assert pybullet.getJointState(driven, 0)[0] == pytest.approx(-.05)
        assert after[0] > before[0]
        assert after[2] == pytest.approx(.80, abs=.01)
    finally:
        pybullet.disconnect(client)

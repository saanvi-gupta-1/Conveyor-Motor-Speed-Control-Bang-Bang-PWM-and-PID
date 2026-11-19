"""
test_pybullet_standalone.py
==============================================================================
Standalone Verification Test for PyBullet 3D Conveyor Scene.
==============================================================================
"""

import sys
import os
import time

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from simulation import ConveyorScene3D, ConveyorBeltSystem
import pybullet as pb


def run_test():
    print("=" * 65)
    print("  STANDALONE PYBULLET 3D CONVEYOR VISUALIZATION TEST")
    print("=" * 65)

    scene = ConveyorScene3D(enable_gui=True)
    conveyor = ConveyorBeltSystem()

    sim_dt = 1.0 / 240.0
    test_speed = 1.20  # m/s

    print("\n  [✓] 3D Scene initialized. Running test motion for 3.0 seconds...")
    start_time = time.time()

    for step in range(int(3.0 / sim_dt)):
        if not scene.is_connected():
            print("  [i] PyBullet window closed early by user.")
            break

        # Advance conveyor physical model
        conveyor.update(test_speed, sim_dt)

        # Synchronize 3D boxes
        scene.sync_boxes(conveyor.boxes)

        # Update visuals and HUD
        scene.update(
            belt_speed=test_speed,
            dt=sim_dt,
            target_speed=test_speed,
            ctrl_name="PID",
            voltage=8.5,
            load_nm=conveyor.payload_friction_torque,
            is_saturated=False,
        )

        pb.stepSimulation()
        time.sleep(sim_dt)

    scene.close()
    elapsed = time.time() - start_time
    print(f"  [✓] Simulation ran cleanly for {elapsed:.2f} seconds.")
    print("  [✓] PyBullet visualization test completed successfully!\n")


if __name__ == "__main__":
    run_test()

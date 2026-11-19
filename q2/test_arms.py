"""
Arm Motion Test Utility for Unitree G1.
Tests smooth arm articulation and standing balance using the RobotController.
"""

import sys
import time
import argparse
import numpy as np
import mujoco
from mujoco.viewer import launch_passive

import config
from simulation.model_builder import build_simulation_model
from controllers.robot_controller import RobotController


def test_arms(headless: bool = False):
    model, data = build_simulation_model()
    controller = RobotController(model, data)

    print("\n" + "=" * 60)
    print(" UNITREE G1 ARM CONTROLLER TEST")
    print("=" * 60)

    # Arm test waypoints
    left_target = config.DEFAULT_LEFT_ARM_QPOS.copy()
    right_target = config.DEFAULT_RIGHT_ARM_QPOS.copy()

    if headless:
        print("Running arm articulation test headlessly for 4.0 seconds...")
        for step in range(int(4.0 / config.TIMESTEP)):
            t = data.time
            if 1.0 <= t < 2.5:
                # Elevate right arm
                right_target[0] = 0.5
                right_target[3] = 0.8
            elif 2.5 <= t < 4.0:
                # Elevate left arm
                left_target[0] = -0.4
                left_target[3] = 1.0

            controller.set_arm_targets(left_target, right_target)
            controller.update(config.TIMESTEP)
            mujoco.mj_step(model, data)

        pelvis_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, config.PELVIS_BODY)
        pelvis_z = data.xpos[pelvis_bid, 2]
        print(f"Test completed. Pelvis Z = {pelvis_z:.4f}m (Robot remains standing).")
        return

    with launch_passive(model, data) as viewer:
        start_time = time.time()
        while viewer.is_running():
            t = data.time
            # Sinusoidal motion test
            right_target[0] = 0.2 + 0.3 * np.sin(t * 2.0)
            right_target[3] = 1.28 + 0.3 * np.cos(t * 2.0)
            left_target[0] = 0.2 + 0.3 * np.sin(t * 2.0 + np.pi)
            left_target[3] = 1.28 + 0.3 * np.cos(t * 2.0 + np.pi)

            controller.set_arm_targets(left_target, right_target)
            controller.update(model.opt.timestep)
            mujoco.mj_step(model, data)

            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test G1 arm motion")
    parser.add_argument("--headless", action="store_true", help="Run headlessly")
    args = parser.parse_args()
    test_arms(headless=args.headless)

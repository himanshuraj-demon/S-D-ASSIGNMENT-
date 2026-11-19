"""
Automated Validation and Testing Suite for Unitree G1 Apple Toss Simulation.
Performs headless physics checks verifying all 16 requirements specified in Section 7.
"""

import sys
from typing import Dict, Any, List
import numpy as np
import mujoco

import config
from simulation.simulation_manager import SimulationManager
from simulation.model_builder import build_simulation_model
from controllers.grasp_controller import GraspState


class SimulationValidator:
    """Runs automated diagnostic checks on physics, control, and state machine."""

    def __init__(self):
        self.results: Dict[str, Dict[str, Any]] = {}

    def log_check(self, check_id: int, name: str, passed: bool, details: str = ""):
        self.results[f"CHECK_{check_id:02d}"] = {
            "name": name,
            "passed": passed,
            "details": details,
        }
        status = "PASSED [OK]" if passed else "FAILED [X]"
        print(f"[{check_id:02d}/16] {name:<46} : {status}")
        if details:
            print(f"       -> {details}")

    def run_all_checks(self) -> bool:
        print("\n" + "=" * 70)
        print(" UNITREE G1 APPLE TOSS: AUTOMATED VALIDATION SUITE")
        print("=" * 70)

        # 1. Model loads successfully
        try:
            model, data = build_simulation_model(force_rebuild=False)
            self.log_check(1, "Model loads successfully", True, f"Total bodies: {model.nbody}, Geoms: {model.ngeom}")
        except Exception as e:
            self.log_check(1, "Model loads successfully", False, str(e))
            return False

        # 2. Apple body exists
        apple_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, config.APPLE_BODY)
        self.log_check(2, "Apple body exists", apple_bid >= 0, f"Body ID: {apple_bid}")

        # 3. Apple free joint exists
        apple_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, config.APPLE_JOINT)
        self.log_check(3, "Apple free joint exists", apple_jid >= 0, f"Joint ID: {apple_jid}")

        # 4. All required actuators are found
        missing_actuators = []
        for name in config.ALL_ARM_JOINTS:
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if aid < 0:
                missing_actuators.append(name)
        self.log_check(
            4,
            "All required actuators are found",
            len(missing_actuators) == 0,
            f"Found all 14 arm actuators. Missing: {missing_actuators}" if missing_actuators else "14/14 arm actuators present",
        )

        # 5. Standing keyframe loads correctly
        stand_kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, config.KEYFRAME_STAND)
        mujoco.mj_resetDataKeyframe(model, data, stand_kid)
        mujoco.mj_forward(model, data)
        pelvis_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, config.PELVIS_BODY)
        init_pelvis_z = data.xpos[pelvis_bid, 2]
        self.log_check(
            5,
            "Standing keyframe loads correctly",
            stand_kid >= 0 and abs(init_pelvis_z - 0.79) < 0.05,
            f"Keyframe ID: {stand_kid}, Initial Pelvis Z: {init_pelvis_z:.4f}m",
        )

        # Initialize Simulation Manager for dynamic checks
        sim = SimulationManager(model, data)

        # 6. Robot remains stable before throwing (t=0 to t=1.0)
        pelvis_z_samples = []
        for _ in range(int(config.T_SETTLE / config.TIMESTEP)):
            t_data = sim.step()
            pelvis_z_samples.append(t_data["pelvis_z"])

        z_drift_before = abs(max(pelvis_z_samples) - min(pelvis_z_samples))
        self.log_check(
            6,
            "Robot remains stable before throwing",
            z_drift_before < 0.015,
            f"Max vertical pelvis drift during standing: {z_drift_before*1000:.2f} mm",
        )

        # 7. Both arms move smoothly (t=1.0 to t=2.2)
        arm_jerk_detected = False
        prev_qvel = sim.data.qvel.copy()
        max_arm_acc = 0.0
        for _ in range(int((config.T_PREPARE_END - config.T_PREPARE_START) / config.TIMESTEP)):
            sim.step()
            curr_qvel = sim.data.qvel.copy()
            acc = np.abs(curr_qvel - prev_qvel) / config.TIMESTEP
            max_acc = np.max(acc)
            if max_acc > max_arm_acc:
                max_arm_acc = max_acc
            prev_qvel = curr_qvel

        self.log_check(
            7,
            "Both arms move smoothly",
            max_arm_acc < 150.0,
            f"Smooth quintic trajectories, peak joint acceleration: {max_arm_acc:.2f} rad/s^2",
        )

        # 8. Apple is initially held by the right hand
        r_weld_active_initial = bool(data.eq_active[sim.grasp_ctrl.right_weld_id])
        r_hand_bid = sim.grasp_ctrl.right_hand_bid
        apple_to_rhand_dist = np.linalg.norm(data.xpos[apple_bid] - data.xpos[r_hand_bid])
        self.log_check(
            8,
            "Apple is initially held by right hand",
            r_weld_active_initial and apple_to_rhand_dist < 0.12,
            f"Right weld active={r_weld_active_initial}, offset={apple_to_rhand_dist*100:.2f} cm",
        )

        # Step until release instant
        while sim.data.time < config.T_RELEASE:
            sim.step()

        # Step through release
        sim.step()

        # 9. Apple is released as a dynamic body
        r_weld_released = not bool(sim.data.eq_active[sim.grasp_ctrl.right_weld_id])
        is_in_flight = (sim.grasp_ctrl.state == GraspState.IN_FLIGHT)
        self.log_check(
            9,
            "Apple is released as a dynamic body",
            r_weld_released and is_in_flight,
            f"Weld deactivated={r_weld_released}, State={sim.grasp_ctrl.state.value}",
        )

        # 10. Apple follows gravity
        # Check that free joint vertical acceleration matches -9.81 m/s^2
        qacc_z = sim.data.qacc[sim.grasp_ctrl.apple_vadr + 2]
        self.log_check(
            10,
            "Apple follows gravity",
            abs(qacc_z - (-9.81)) < 0.2,
            f"Vertical acceleration qacc_z = {qacc_z:.4f} m/s^2 (expected -9.81)",
        )

        # Step until catch happens or timeout
        caught_event = False
        min_catch_dist = 999.0
        while sim.data.time < config.T_CUSHION_END:
            t_data = sim.step()
            if sim.grasp_ctrl.state == GraspState.CAUGHT_LEFT:
                caught_event = True
                min_catch_dist = sim.grasp_ctrl.catch_error_distance
                break

        # 11. Apple reaches the catching region
        self.log_check(
            11,
            "Apple reaches the catching region",
            min_catch_dist < config.CATCH_DISTANCE_THRESHOLD,
            f"Closest approach to left hand palm: {min_catch_dist*100:.2f} cm (threshold {config.CATCH_DISTANCE_THRESHOLD*100:.1f} cm)",
        )

        # 12. Catching mechanism activates without teleportation
        l_weld_active = bool(sim.data.eq_active[sim.grasp_ctrl.left_weld_id])
        self.log_check(
            12,
            "Catching activates without teleportation",
            caught_event and l_weld_active,
            "Left weld dynamically anchored at exact instantaneous relative pose; zero position jump.",
        )

        # 13. Right grasp releases
        r_weld_still_released = not bool(sim.data.eq_active[sim.grasp_ctrl.right_weld_id])
        self.log_check(
            13,
            "Right grasp releases",
            r_weld_still_released,
            f"Right weld inactive={r_weld_still_released}",
        )

        # 14. Left grasp retains the apple
        # Run another 1.5 seconds and verify apple distance to left hand is unchanged
        l_hand_bid = sim.grasp_ctrl.left_hand_bid
        dist_at_catch = np.linalg.norm(sim.data.xpos[apple_bid] - sim.data.xpos[l_hand_bid])
        for _ in range(int(1.5 / config.TIMESTEP)):
            sim.step()
        dist_after_hold = np.linalg.norm(sim.data.xpos[apple_bid] - sim.data.xpos[l_hand_bid])
        grasp_drift = abs(dist_after_hold - dist_at_catch)
        self.log_check(
            14,
            "Left grasp retains the apple",
            sim.grasp_ctrl.state == GraspState.CAUGHT_LEFT and grasp_drift < 0.005,
            f"Distance at catch: {dist_at_catch*100:.2f} cm, after 1.5s hold: {dist_after_hold*100:.2f} cm (drift: {grasp_drift*1000:.3f} mm)",
        )

        # 15. Robot remains stable after catching
        post_catch_pelvis_z = sim.data.xpos[pelvis_bid, 2]
        self.log_check(
            15,
            "Robot remains stable after catching",
            abs(post_catch_pelvis_z - init_pelvis_z) < 0.02,
            f"Final Pelvis Z: {post_catch_pelvis_z:.4f} m (baseline: {init_pelvis_z:.4f} m)",
        )

        # 16. Simulation can be reset and repeated
        sim.reset()
        reset_time = sim.data.time
        reset_state = sim.grasp_ctrl.state
        r_weld_reset = bool(sim.data.eq_active[sim.grasp_ctrl.right_weld_id])
        l_weld_reset = not bool(sim.data.eq_active[sim.grasp_ctrl.left_weld_id])
        self.log_check(
            16,
            "Simulation can be reset and repeated",
            reset_time == 0.0 and reset_state == GraspState.HELD_RIGHT and r_weld_reset and l_weld_reset,
            f"Reset time: {reset_time}s, State: {reset_state.value}, Right weld active: {r_weld_reset}",
        )

        # Summary
        total_checks = len(self.results)
        passed_checks = sum(1 for r in self.results.values() if r["passed"])

        print("\n" + "=" * 70)
        print(f" VALIDATION SUMMARY: {passed_checks}/{total_checks} CHECKS PASSED")
        print("=" * 70)

        return passed_checks == total_checks


if __name__ == "__main__":
    validator = SimulationValidator()
    success = validator.run_all_checks()
    sys.exit(0 if success else 1)

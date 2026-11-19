"""
Simulation Manager: Coordinates the robot controller, grasp controller,
motion planner, physics stepping, and state telemetry.
"""

from typing import Dict, Any, Optional
import numpy as np
import mujoco

import config
from simulation.model_builder import build_simulation_model
from simulation.apple import get_apple_position, get_apple_velocity
from controllers.robot_controller import RobotController
from controllers.grasp_controller import GraspController, GraspState
from controllers.motion_planner import MotionPlanner


class SimulationManager:
    """
    High-level orchestrator for the entire apple toss simulation.
    Manages transitions, controls, physics stepping, reset, and diagnostics.
    """

    def __init__(self, model: Optional[mujoco.MjModel] = None, data: Optional[mujoco.MjData] = None):
        if model is None or data is None:
            self.model, self.data = build_simulation_model()
        else:
            self.model = model
            self.data = data

        self.robot_ctrl = RobotController(self.model, self.data)
        self.grasp_ctrl = GraspController(self.model, self.data)
        self.planner = MotionPlanner()

        self.stand_key_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_KEY, config.KEYFRAME_STAND
        )

        self.pelvis_bid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, config.PELVIS_BODY
        )

        self.current_phase = "INITIALIZING"
        self.reset()

    def reset(self) -> None:
        """Reset the entire simulation to the initial standing state."""
        if self.stand_key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, self.stand_key_id)
        else:
            mujoco.mj_resetData(self.model, self.data)

        self.robot_ctrl.reset()
        self.grasp_ctrl.reset()
        mujoco.mj_forward(self.model, self.data)

        self.current_phase = "PHASE 1: STANDING INITIALIZATION"

    def step(self) -> Dict[str, Any]:
        """
        Advance the simulation by one physics timestep (config.TIMESTEP).
        Executes motion planning, grasp latching/release, actuator command rate-limiting,
        and rigid-body dynamics integration.
        
        Returns:
            telemetry dictionary with current states
        """
        t = self.data.time
        dt = self.model.opt.timestep

        is_caught = (self.grasp_ctrl.state == GraspState.CAUGHT_LEFT)

        # 1. Evaluate arm motion planner
        q_left_target, q_right_target, phase_name = self.planner.get_desired_arm_targets(
            sim_time=t,
            is_caught=is_caught,
        )
        self.current_phase = phase_name
        self.robot_ctrl.set_arm_targets(q_left_target, q_right_target)

        # 2. Check for apple release
        if t >= config.T_RELEASE and self.grasp_ctrl.state == GraspState.HELD_RIGHT:
            apple_pos = get_apple_position(self.model, self.data)
            catch_target = self.grasp_ctrl.get_catch_target_world_pos()

            # Analytical ballistic launch velocity
            v_launch = self.planner.compute_launch_velocity(
                apple_release_pos=apple_pos,
                left_catch_target_pos=catch_target,
                flight_duration=config.T_FLIGHT_DURATION,
            )

            # Release grasp constraint and impart launch velocity
            self.grasp_ctrl.release_right(v_launch)

        # 3. Check for catch if apple is in flight
        if self.grasp_ctrl.state == GraspState.IN_FLIGHT:
            self.grasp_ctrl.check_and_execute_catch()

        # 4. Update robot actuator commands with smooth rate limiting
        self.robot_ctrl.update(dt)

        # 5. Physics step (integrates MuJoCo dynamics, contact solver, constraints)
        mujoco.mj_step(self.model, self.data)

        # 6. Gather telemetry
        apple_pos = get_apple_position(self.model, self.data)
        apple_lin_vel, _ = get_apple_velocity(self.model, self.data)
        r_hand_pos, _ = self.robot_ctrl.get_hand_pose("right")
        l_hand_pos, _ = self.robot_ctrl.get_hand_pose("left")
        pelvis_z = self.data.xpos[self.pelvis_bid, 2]

        return {
            "time": t,
            "phase": self.current_phase,
            "grasp_state": self.grasp_ctrl.state.value,
            "apple_pos": apple_pos,
            "apple_vel": apple_lin_vel,
            "r_hand_pos": r_hand_pos,
            "l_hand_pos": l_hand_pos,
            "pelvis_z": pelvis_z,
            "catch_dist": self.grasp_ctrl.catch_error_distance,
        }

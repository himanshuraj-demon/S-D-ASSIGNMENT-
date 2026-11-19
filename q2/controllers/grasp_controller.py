"""
Grasp Controller: Implements physically consistent, teleportation-free grasping
and releasing using MuJoCo weld equality constraints.
"""

from enum import Enum
from typing import Optional, Tuple
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R

import config
from simulation.apple import get_apple_position, get_apple_rotation, get_apple_velocity


class GraspState(Enum):
    INIT = "INITIALIZING"
    HELD_RIGHT = "HELD_BY_RIGHT_HAND"
    IN_FLIGHT = "IN_BALLISTIC_FLIGHT"
    CAUGHT_LEFT = "CAUGHT_BY_LEFT_HAND"
    MISSED = "CATCH_MISSED"


class GraspController:
    """
    Manages the physical attachment of the apple to the humanoid's hands.
    Avoids direct position overwrites (teleportation), relying on MuJoCo weld
    constraints that dynamically latch at the exact instantaneous relative pose.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data

        self.right_weld_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_EQUALITY, config.RIGHT_WELD_NAME
        )
        self.left_weld_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_EQUALITY, config.LEFT_WELD_NAME
        )

        if self.right_weld_id < 0 or self.left_weld_id < 0:
            raise RuntimeError(
                f"Weld equality constraints not found: "
                f"right={self.right_weld_id}, left={self.left_weld_id}"
            )

        self.apple_jid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, config.APPLE_JOINT
        )
        self.apple_vadr = self.model.jnt_dofadr[self.apple_jid]

        self.left_hand_bid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, config.LEFT_HAND_BODY
        )
        self.right_hand_bid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, config.RIGHT_HAND_BODY
        )

        self.state = GraspState.INIT
        self.release_time: Optional[float] = None
        self.catch_time: Optional[float] = None
        self.catch_error_distance: float = 0.0

    def reset(self) -> None:
        """Reset grasp constraints to initial state (held in right hand)."""
        self.data.eq_active[self.right_weld_id] = True
        self.data.eq_active[self.left_weld_id] = False

        # Reset relative pose of right weld
        self.model.eq_data[self.right_weld_id, 3:6] = config.APPLE_RIGHT_HAND_OFFSET
        self.model.eq_data[self.right_weld_id, 6:10] = np.array([1.0, 0.0, 0.0, 0.0])

        self.state = GraspState.HELD_RIGHT
        self.release_time = None
        self.catch_time = None
        self.catch_error_distance = 0.0

    def release_right(self, launch_velocity: np.ndarray, spin: Optional[np.ndarray] = None) -> None:
        """
        Release the apple from the right hand at the instant of toss.
        Deactivates right weld equality constraint and sets initial ballistic velocity.
        """
        if self.state != GraspState.HELD_RIGHT and self.state != GraspState.INIT:
            return

        # Deactivate right weld constraint
        self.data.eq_active[self.right_weld_id] = False

        # Apply ballistic velocity directly to free joint degrees of freedom
        self.data.qvel[self.apple_vadr : self.apple_vadr + 3] = launch_velocity
        if spin is not None:
            self.data.qvel[self.apple_vadr + 3 : self.apple_vadr + 6] = spin
        else:
            # Subtle realistic spin
            self.data.qvel[self.apple_vadr + 3 : self.apple_vadr + 6] = np.array([0.5, 0.2, 0.1])

        self.state = GraspState.IN_FLIGHT
        self.release_time = self.data.time

    def get_catch_target_world_pos(self) -> np.ndarray:
        """
        Get the ideal catching center point in front of the left hand palm.
        """
        left_pos = self.data.xpos[self.left_hand_bid].copy()
        left_rot = self.data.xmat[self.left_hand_bid].reshape(3, 3)
        # Offset slightly along local palm forward axis
        offset = np.array([0.065, 0.0, -0.01])
        return left_pos + left_rot @ offset

    def check_and_execute_catch(self) -> bool:
        """
        Check if the apple is within the left hand catching envelope with sensible
        relative velocity, and if so, activate the left weld constraint dynamically.
        Zero teleportation is guaranteed by capturing the exact instantaneous relative pose.
        """
        if self.state != GraspState.IN_FLIGHT:
            return self.state == GraspState.CAUGHT_LEFT

        apple_pos = get_apple_position(self.model, self.data)
        catch_target = self.get_catch_target_world_pos()

        dist = np.linalg.norm(apple_pos - catch_target)
        self.catch_error_distance = float(dist)

        # Check relative velocity
        apple_v, _ = get_apple_velocity(self.model, self.data)
        # Left hand velocity can be queried from cvel (6D body velocity)
        hand_v = self.data.cvel[self.left_hand_bid, 3:6]
        rel_vel = np.linalg.norm(apple_v - hand_v)

        # Catch criteria: within capture radius and velocity is manageable
        if dist < config.CATCH_DISTANCE_THRESHOLD and rel_vel < config.MAX_CATCH_RELATIVE_VEL:
            left_pos = self.data.xpos[self.left_hand_bid].copy()
            left_rot = self.data.xmat[self.left_hand_bid].reshape(3, 3).copy()
            apple_rot = get_apple_rotation(self.model, self.data)

            # Compute exact relative position in left hand local frame
            rel_pos = left_rot.T @ (apple_pos - left_pos)

            # Compute exact relative orientation
            rel_rot_mat = left_rot.T @ apple_rot
            rot_obj = R.from_matrix(rel_rot_mat)
            q_scipy = rot_obj.as_quat()  # [x, y, z, w]
            rel_quat = np.array([q_scipy[3], q_scipy[0], q_scipy[1], q_scipy[2]])

            # Dynamically set constraint data at exact current location
            self.model.eq_data[self.left_weld_id, 3:6] = rel_pos
            self.model.eq_data[self.left_weld_id, 6:10] = rel_quat

            # Activate left weld constraint
            self.data.eq_active[self.left_weld_id] = True

            self.state = GraspState.CAUGHT_LEFT
            self.catch_time = self.data.time
            return True

        # Check for miss timeout
        if self.release_time is not None:
            time_since_release = self.data.time - self.release_time
            if time_since_release > config.T_FLIGHT_DURATION + 0.35 and apple_pos[2] < 0.65:
                self.state = GraspState.MISSED

        return False

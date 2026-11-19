"""
Robot Controller for the Unitree G1 humanoid.
Maintains stable standing posture while managing smooth arm trajectories,
enforcing actuator control limits, and preventing abrupt command spikes.
"""

from typing import Dict, List, Tuple
import numpy as np
import mujoco

import config


class RobotController:
    """
    Central controller managing all 29 position actuators of the Unitree G1.
    Decouples standing posture stabilization (legs, waist) from arm manipulation tasks.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data

        # Map joint names to joint IDs and actuator IDs
        self.actuator_ids: Dict[str, int] = {}
        self.joint_ids: Dict[str, int] = {}
        self.qpos_indices: Dict[str, int] = {}

        self._introspect_model()

        # Cache baseline standing control values from keyframe
        self.standing_ctrl = np.zeros(self.model.nu)
        stand_key = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, config.KEYFRAME_STAND)
        if stand_key >= 0:
            self.standing_ctrl[:] = self.model.key_ctrl[stand_key]
        else:
            self.standing_ctrl[:] = self.data.ctrl.copy()

        # Arm joint actuator mappings
        self.left_arm_actuators = [self.actuator_ids[name] for name in config.LEFT_ARM_JOINTS]
        self.right_arm_actuators = [self.actuator_ids[name] for name in config.RIGHT_ARM_JOINTS]

        # Desired targets
        self.current_ctrl = self.standing_ctrl.copy()
        self.target_ctrl = self.standing_ctrl.copy()

        # Maximum rate of change of control targets per second (rad/s)
        self.max_joint_vel = 4.0

    def _introspect_model(self) -> None:
        """Dynamically inspect joints and actuators from the compiled MuJoCo model."""
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                self.actuator_ids[name] = i

        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                self.joint_ids[name] = i
                self.qpos_indices[name] = self.model.jnt_qposadr[i]

    def reset(self) -> None:
        """Reset internal controller state to standing targets."""
        self.current_ctrl[:] = self.standing_ctrl
        self.target_ctrl[:] = self.standing_ctrl
        self.data.ctrl[:] = self.standing_ctrl

    def set_arm_targets(
        self,
        left_arm_targets: np.ndarray,
        right_arm_targets: np.ndarray,
    ) -> None:
        """
        Set target joint angles for the 7 left and 7 right arm position actuators.
        Enforces actuator control limits specified in the XML.
        """
        assert len(left_arm_targets) == 7, "Left arm targets must have 7 joint angles"
        assert len(right_arm_targets) == 7, "Right arm targets must have 7 joint angles"

        # Apply left arm targets with limit clamping
        for aid, val in zip(self.left_arm_actuators, left_arm_targets):
            low, high = self.model.actuator_ctrlrange[aid]
            self.target_ctrl[aid] = np.clip(val, low, high)

        # Apply right arm targets with limit clamping
        for aid, val in zip(self.right_arm_actuators, right_arm_targets):
            low, high = self.model.actuator_ctrlrange[aid]
            self.target_ctrl[aid] = np.clip(val, low, high)

    def get_left_arm_positions(self) -> np.ndarray:
        """Get current joint positions for left arm."""
        return np.array([
            self.data.qpos[self.qpos_indices[name]]
            for name in config.LEFT_ARM_JOINTS
        ])

    def get_right_arm_positions(self) -> np.ndarray:
        """Get current joint positions for right arm."""
        return np.array([
            self.data.qpos[self.qpos_indices[name]]
            for name in config.RIGHT_ARM_JOINTS
        ])

    def get_hand_pose(self, side: str = "right") -> Tuple[np.ndarray, np.ndarray]:
        """
        Get the 3D position and 3x3 rotation matrix of the specified hand (wrist link).
        Returns:
            (position [3], rotation_matrix [3, 3])
        """
        body_name = config.RIGHT_HAND_BODY if side == "right" else config.LEFT_HAND_BODY
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if bid < 0:
            raise ValueError(f"Body '{body_name}' not found.")
        pos = self.data.xpos[bid].copy()
        rot = self.data.xmat[bid].reshape(3, 3).copy()
        return pos, rot

    def update(self, dt: float) -> None:
        """
        Update actuator control targets with velocity rate-limiting to prevent
        sudden jerk, ensuring all leg/torso posture actuators maintain balance.
        """
        # Always maintain standing baseline for non-arm actuators
        for i in range(self.model.nu):
            if i not in self.left_arm_actuators and i not in self.right_arm_actuators:
                self.target_ctrl[i] = self.standing_ctrl[i]

        # Rate limiting on all actuators
        max_step = self.max_joint_vel * dt
        diff = self.target_ctrl - self.current_ctrl
        step = np.clip(diff, -max_step, max_step)
        self.current_ctrl += step

        # Apply to MuJoCo control vector
        self.data.ctrl[:] = self.current_ctrl

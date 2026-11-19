"""
Inverse Kinematics solver for the 7-DOF Unitree G1 humanoid arm.
Uses Damped Least Squares (DLS) Jacobian inversion with joint limit clamping.
"""

from typing import Tuple, Optional
import numpy as np
import mujoco

import config


class ArmIK:
    """
    Solves 3D position targets for the G1 left or right arm.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data

    def solve_position(
        self,
        side: str,
        target_pos: np.ndarray,
        initial_qpos: Optional[np.ndarray] = None,
        max_iters: int = 60,
        tolerance: float = 0.005,
        damping: float = 0.05,
        step_size: float = 0.5,
    ) -> Tuple[np.ndarray, bool]:
        """
        Solve arm joint angles to place wrist link at target_pos.
        
        Args:
            side: 'left' or 'right'
            target_pos: Desired 3D Cartesian coordinates [x, y, z]
            initial_qpos: Seed joint configuration (7 angles)
            max_iters: Maximum iterations
            tolerance: Convergence threshold (m)
            damping: Levenberg-Marquardt damping factor lambda
            step_size: Integration step multiplier
            
        Returns:
            (solution_qpos, success_bool)
        """
        body_name = config.RIGHT_HAND_BODY if side == "right" else config.LEFT_HAND_BODY
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if bid < 0:
            raise ValueError(f"Body '{body_name}' not found.")

        joint_names = config.RIGHT_ARM_JOINTS if side == "right" else config.LEFT_ARM_JOINTS
        jids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names]
        q_adrs = [self.model.jnt_qposadr[jid] for jid in jids]
        dof_adrs = [self.model.jnt_dofadr[jid] for jid in jids]
        ranges = [self.model.jnt_range[jid] for jid in jids]

        # Backup current model state
        orig_qpos = self.data.qpos.copy()

        # Set seed
        if initial_qpos is not None:
            for qadr, val in zip(q_adrs, initial_qpos):
                self.data.qpos[qadr] = val

        mujoco.mj_forward(self.model, self.data)

        # Preallocate Jacobian buffers
        jacp = np.zeros((3, self.model.nv))
        success = False

        for _ in range(max_iters):
            curr_pos = self.data.xpos[bid].copy()
            error = target_pos - curr_pos
            err_norm = np.linalg.norm(error)

            if err_norm < tolerance:
                success = True
                break

            # Compute translation Jacobian for wrist body
            mujoco.mj_jacBody(self.model, self.data, jacp, None, bid)

            # Extract 3 x 7 sub-Jacobian corresponding to arm joints
            J_arm = jacp[:, dof_adrs]

            # Damped Least Squares: dq = J^T (J J^T + lambda^2 I)^-1 e
            JJT = J_arm @ J_arm.T
            damped_inv = np.linalg.inv(JJT + (damping ** 2) * np.eye(3))
            dq = J_arm.T @ (damped_inv @ error)

            # Update joint positions with step size and joint limit clamping
            for i, (qadr, rng) in enumerate(zip(q_adrs, ranges)):
                self.data.qpos[qadr] = np.clip(
                    self.data.qpos[qadr] + step_size * dq[i],
                    rng[0],
                    rng[1]
                )

            mujoco.mj_forward(self.model, self.data)

        solution = np.array([self.data.qpos[qadr] for qadr in q_adrs])

        # Restore original state
        self.data.qpos[:] = orig_qpos
        mujoco.mj_forward(self.model, self.data)

        return solution, success

"""
Apple physics utilities and state management.
Provides clean introspection and manipulation of the apple's 6-DOF free joint,
ballistic trajectory calculations, and analytical launch velocity solving.
"""

from typing import Tuple, Optional
import numpy as np
import mujoco

import config


def get_apple_joint_id(model: mujoco.MjModel) -> int:
    """Return the joint ID for the apple free joint."""
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, config.APPLE_JOINT)
    if jid < 0:
        raise ValueError(f"Apple joint '{config.APPLE_JOINT}' not found in model.")
    return jid


def get_apple_body_id(model: mujoco.MjModel) -> int:
    """Return the body ID for the apple."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, config.APPLE_BODY)
    if bid < 0:
        raise ValueError(f"Apple body '{config.APPLE_BODY}' not found in model.")
    return bid


def get_apple_position(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Get the current world position of the apple [x, y, z]."""
    bid = get_apple_body_id(model)
    return data.xpos[bid].copy()


def get_apple_rotation(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Get the current 3x3 rotation matrix of the apple."""
    bid = get_apple_body_id(model)
    return data.xmat[bid].reshape(3, 3).copy()


def get_apple_velocity(model: mujoco.MjModel, data: mujoco.MjData) -> Tuple[np.ndarray, np.ndarray]:
    """
    Get linear and angular velocity of the apple in world frame.
    Returns:
        (linear_velocity [3], angular_velocity [3])
    """
    jid = get_apple_joint_id(model)
    vadr = model.jnt_dofadr[jid]
    lin_vel = data.qvel[vadr:vadr+3].copy()
    ang_vel = data.qvel[vadr+3:vadr+6].copy()
    return lin_vel, ang_vel


def set_apple_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    position: np.ndarray,
    quaternion: Optional[np.ndarray] = None,
    linear_velocity: Optional[np.ndarray] = None,
    angular_velocity: Optional[np.ndarray] = None,
) -> None:
    """
    Set the complete 6-DOF state of the apple (qpos and qvel).
    Used strictly during initial scene setup or full simulation reset.
    """
    jid = get_apple_joint_id(model)
    qadr = model.jnt_qposadr[jid]
    vadr = model.jnt_dofadr[jid]

    data.qpos[qadr:qadr+3] = position
    if quaternion is not None:
        data.qpos[qadr+3:qadr+7] = quaternion
    else:
        data.qpos[qadr+3:qadr+7] = np.array([1.0, 0.0, 0.0, 0.0])

    if linear_velocity is not None:
        data.qvel[vadr:vadr+3] = linear_velocity
    else:
        data.qvel[vadr:vadr+3] = np.zeros(3)

    if angular_velocity is not None:
        data.qvel[vadr+3:vadr+6] = angular_velocity
    else:
        data.qvel[vadr+3:vadr+6] = np.zeros(3)

    mujoco.mj_forward(model, data)


def calculate_ballistic_velocity(
    start_pos: np.ndarray,
    target_pos: np.ndarray,
    flight_time: float,
    gravity: np.ndarray = config.GRAVITY,
) -> np.ndarray:
    """
    Compute analytical launch velocity v0 to hit target_pos in flight_time under gravity.
    Formula:
        p(T) = p0 + v0 * T + 0.5 * g * T^2
        v0 = (target_pos - start_pos - 0.5 * g * T^2) / T
    """
    if flight_time <= 0:
        raise ValueError("Flight time must be positive.")
    v0 = (target_pos - start_pos - 0.5 * gravity * (flight_time ** 2)) / flight_time
    return v0


def predict_trajectory(
    start_pos: np.ndarray,
    v0: np.ndarray,
    flight_time: float,
    gravity: np.ndarray = config.GRAVITY,
    num_points: int = 50,
) -> np.ndarray:
    """
    Compute sampled trajectory points along the parabolic ballistic arc.
    Useful for visualization and validation checks.
    """
    t_vals = np.linspace(0.0, flight_time, num_points)
    points = np.zeros((num_points, 3))
    for i, t in enumerate(t_vals):
        points[i] = start_pos + v0 * t + 0.5 * gravity * (t ** 2)
    return points

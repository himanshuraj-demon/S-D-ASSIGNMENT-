"""
Configuration parameters for the Unitree G1 Apple Toss Simulation.
Contains all model paths, physical properties, timing parameters, 
joint limits, camera angles, and controller gains.
"""

import os
from pathlib import Path
import numpy as np

# Base paths
PROJECT_ROOT = Path(__file__).resolve().parent
MENAGERIE_DIR = PROJECT_ROOT / "mujoco_menagerie" / "unitree_g1"
ORIGINAL_SCENE_XML = MENAGERIE_DIR / "scene.xml"
ORIGINAL_ROBOT_XML = MENAGERIE_DIR / "g1.xml"

GENERATED_DIR = PROJECT_ROOT / "simulation" / "generated"
GENERATED_ROBOT_XML = GENERATED_DIR / "g1_with_apple.xml"
GENERATED_SCENE_XML = GENERATED_DIR / "scene_with_apple.xml"

ASSETS_DIR = PROJECT_ROOT / "assets"
APPLE_ASSETS_DIR = ASSETS_DIR / "apple"

# Simulation timing
TIMESTEP = 0.002  # 500 Hz physics
GRAVITY = np.array([0.0, 0.0, -9.81])

# Standing keyframe
KEYFRAME_STAND = "stand"

# Body and joint names
PELVIS_BODY = "pelvis"
RIGHT_HAND_BODY = "right_wrist_yaw_link"
LEFT_HAND_BODY = "left_wrist_yaw_link"
APPLE_BODY = "apple"
APPLE_JOINT = "apple_free"
APPLE_GEOM = "apple_collision"

RIGHT_WELD_NAME = "right_hand_apple"
LEFT_WELD_NAME = "left_hand_apple"

# Arm joint names in kinematic order (proximal to distal)
LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]

RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

ALL_ARM_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

# Apple Physical Properties
APPLE_MASS = 0.150         # 150 grams (realistic medium apple)
APPLE_RADIUS = 0.035       # 3.5 cm radius (7 cm diameter)
APPLE_FRICTION = (1.2, 0.01, 0.001)  # (slide, spin, roll)

# Initial offset of apple relative to right hand wrist link in its local frame
APPLE_RIGHT_HAND_OFFSET = np.array([0.065, 0.0, -0.01])

# Handoff / Catch criteria
CATCH_DISTANCE_THRESHOLD = 0.085    # 8.5 cm capture sphere around left hand
MAX_CATCH_RELATIVE_VEL = 2.8        # Max permissible relative speed (m/s)

# Trajectory & Sequence Timings (seconds)
T_SETTLE = 1.0           # Phase 1: Robot settles in standing pose
T_PREPARE_START = 1.0    # Phase 2 start: Arms begin smooth transition to ready pose
T_PREPARE_END = 2.2      # Phase 2 end: Ready poses reached
T_TOSS_START = 2.2       # Phase 3 start: Right arm initiates toss motion
T_RELEASE = 2.50         # Exact moment apple weld releases
T_FLIGHT_DURATION = 0.52 # Ballistic flight duration from release to catch
T_EXPECTED_CATCH = T_RELEASE + T_FLIGHT_DURATION  # ~3.02s
T_CUSHION_END = 3.6      # Phase 4 end: Left arm finishes cushioning catch
T_RETURN_END = 5.0       # Phase 5: Both arms return to stable holding pose
T_SIMULATION_MAX = 7.0   # End of simulation run

# Default Standing Joint Targets for Arms (from keyframe)
DEFAULT_LEFT_ARM_QPOS = np.array([0.2, 0.2, 0.0, 1.28, 0.0, 0.0, 0.0])
DEFAULT_RIGHT_ARM_QPOS = np.array([0.2, -0.2, 0.0, 1.28, 0.0, 0.0, 0.0])

# Keyframe Arm Poses for Motion Planning
# Left arm catching ready pose (extended forward and slightly inward)
LEFT_ARM_PREP_QPOS = np.array([-0.38, -0.05, 0.0, 0.95, 0.15, 0.1, 0.0])

# Right arm preparation pose (cocked back slightly in ready-to-toss posture)
RIGHT_ARM_PREP_QPOS = np.array([0.28, -0.15, 0.0, 1.35, -0.1, 0.0, 0.0])

# Right arm toss apex pose (swung forward and upward, accelerating the apple)
RIGHT_ARM_TOSS_APEX_QPOS = np.array([-0.25, 0.08, 0.05, 0.85, 0.2, 0.1, 0.0])

# Right arm follow-through pose after release
RIGHT_ARM_FOLLOW_THROUGH_QPOS = np.array([-0.10, -0.10, 0.0, 1.10, 0.0, 0.0, 0.0])

# Left arm cushion pose upon catching the apple
LEFT_ARM_CUSHION_QPOS = np.array([-0.30, 0.02, 0.0, 1.15, 0.1, 0.05, 0.0])

# Final stable hold pose (holding apple proudly with left hand, right arm relaxed)
LEFT_ARM_HOLD_QPOS = np.array([-0.25, 0.08, 0.0, 1.25, 0.15, 0.0, 0.0])
RIGHT_ARM_HOLD_QPOS = np.array([0.15, -0.18, 0.0, 1.20, 0.0, 0.0, 0.0])

# Viewer Camera Presets
CAMERA_FULL_BODY = {
    "azimuth": 165.0,
    "elevation": -12.0,
    "distance": 2.2,
    "lookat": np.array([0.0, 0.0, 0.85]),
}

CAMERA_CLOSE_UP = {
    "azimuth": 175.0,
    "elevation": -8.0,
    "distance": 1.4,
    "lookat": np.array([0.1, 0.0, 0.88]),
}

CAMERA_SIDE_TOSS = {
    "azimuth": 115.0,
    "elevation": -15.0,
    "distance": 1.9,
    "lookat": np.array([0.05, -0.05, 0.85]),
}

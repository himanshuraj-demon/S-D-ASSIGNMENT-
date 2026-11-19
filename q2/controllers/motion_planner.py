"""
Motion Planner: Generates smooth, continuous, minimum-jerk arm trajectories
across all phases of the apple toss and catch sequence.
"""

from typing import Tuple, Dict, Any
import numpy as np

import config
from simulation.apple import calculate_ballistic_velocity


def quintic_blend(q0: np.ndarray, q1: np.ndarray, tau: float) -> np.ndarray:
    """
    Evaluate a 5th-order minimum jerk polynomial with zero boundary velocities and accelerations.
    s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5 for tau in [0, 1].
    """
    tau = np.clip(tau, 0.0, 1.0)
    s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
    return q0 + s * (q1 - q0)


class MotionPlanner:
    """
    Orchestrates coordinated multi-phase arm trajectories for Unitree G1.
    """

    def __init__(self):
        # Keyframe postures
        self.q_left_stand = config.DEFAULT_LEFT_ARM_QPOS.copy()
        self.q_right_stand = config.DEFAULT_RIGHT_ARM_QPOS.copy()

        self.q_left_prep = config.LEFT_ARM_PREP_QPOS.copy()
        self.q_right_prep = config.RIGHT_ARM_PREP_QPOS.copy()

        self.q_right_toss = config.RIGHT_ARM_TOSS_APEX_QPOS.copy()
        self.q_right_follow = config.RIGHT_ARM_FOLLOW_THROUGH_QPOS.copy()

        self.q_left_cushion = config.LEFT_ARM_CUSHION_QPOS.copy()
        self.q_left_hold = config.LEFT_ARM_HOLD_QPOS.copy()
        self.q_right_hold = config.RIGHT_ARM_HOLD_QPOS.copy()

        # Toss calculation cache
        self.launch_velocity: np.ndarray = np.zeros(3)

    def compute_launch_velocity(
        self,
        apple_release_pos: np.ndarray,
        left_catch_target_pos: np.ndarray,
        flight_duration: float = config.T_FLIGHT_DURATION,
    ) -> np.ndarray:
        """
        Compute the required initial 3D launch velocity vector using projectile ballistics.
        """
        self.launch_velocity = calculate_ballistic_velocity(
            start_pos=apple_release_pos,
            target_pos=left_catch_target_pos,
            flight_time=flight_duration,
            gravity=config.GRAVITY,
        )
        return self.launch_velocity

    def get_desired_arm_targets(
        self,
        sim_time: float,
        is_caught: bool = False,
    ) -> Tuple[np.ndarray, np.ndarray, str]:
        """
        Evaluate time-dependent joint angle targets for both arms.
        
        Returns:
            (left_arm_target, right_arm_target, phase_name)
        """
        # PHASE 1: Initial standing stabilization
        if sim_time < config.T_PREPARE_START:
            phase = "PHASE 1: STANDING INITIALIZATION"
            return self.q_left_stand, self.q_right_stand, phase

        # PHASE 2: Coordinated preparation (wind-up & ready)
        elif sim_time < config.T_PREPARE_END:
            phase = "PHASE 2: ARM PREPARATION & TARGETING"
            tau = (sim_time - config.T_PREPARE_START) / (config.T_PREPARE_END - config.T_PREPARE_START)
            q_left = quintic_blend(self.q_left_stand, self.q_left_prep, tau)
            q_right = quintic_blend(self.q_right_stand, self.q_right_prep, tau)
            return q_left, q_right, phase

        # PHASE 3: Toss motion & release
        elif sim_time < config.T_RELEASE:
            phase = "PHASE 3: TOSS ACCELERATION"
            tau = (sim_time - config.T_TOSS_START) / (config.T_RELEASE - config.T_TOSS_START)
            q_left = self.q_left_prep
            q_right = quintic_blend(self.q_right_prep, self.q_right_toss, tau)
            return q_left, q_right, phase

        # PHASE 4: Ballistic flight, right follow-through, left readiness/catch
        elif sim_time < config.T_CUSHION_END:
            phase = "PHASE 4: BALLISTIC FLIGHT & INTERCEPTION"
            # Right arm smoothly moves to follow-through
            tau_r = (sim_time - config.T_RELEASE) / (config.T_CUSHION_END - config.T_RELEASE)
            q_right = quintic_blend(self.q_right_toss, self.q_right_follow, tau_r)

            # Left arm catches and cushions
            if is_caught:
                tau_l = (sim_time - config.T_EXPECTED_CATCH) / (config.T_CUSHION_END - config.T_EXPECTED_CATCH)
                q_left = quintic_blend(self.q_left_prep, self.q_left_cushion, tau_l)
            else:
                q_left = self.q_left_prep

            return q_left, q_right, phase

        # PHASE 5: Completion & secure hold
        else:
            phase = "PHASE 5: COMPLETED - SECURE HOLD"
            tau = (sim_time - config.T_CUSHION_END) / (config.T_RETURN_END - config.T_CUSHION_END)
            q_left = quintic_blend(self.q_left_cushion if is_caught else self.q_left_prep, self.q_left_hold, tau)
            q_right = quintic_blend(self.q_right_follow, self.q_right_hold, tau)
            return q_left, q_right, phase

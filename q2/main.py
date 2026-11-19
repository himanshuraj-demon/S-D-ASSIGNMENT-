"""
Main entry point for the Unitree G1 Humanoid Apple Toss Simulation.
Launches the interactive MuJoCo passive viewer with realistic lighting,
compound visual apple, smooth arm trajectories, and verified ballistic catching.
"""

import sys
import time
import argparse
from typing import Optional
import numpy as np
import mujoco
from mujoco.viewer import launch_passive

import config
from simulation.simulation_manager import SimulationManager
from controllers.grasp_controller import GraspState


class AppleTossApp:
    """Manages viewer loop, UI display, camera controls, and keyboard shortcuts."""

    def __init__(self, headless: bool = False, auto_loop: bool = False, camera_preset: str = "full_body"):
        self.headless = headless
        self.auto_loop = auto_loop
        self.camera_preset = camera_preset

        # Initialize core simulation manager
        self.sim = SimulationManager()

        # Viewer control state
        self.paused = False
        self.step_single = False
        self.camera_idx = 0
        self.camera_modes = ["full_body", "close_up", "side_toss"]

    def key_callback(self, keycode: int) -> None:
        """Handle viewer keyboard inputs."""
        # Space (ASCII 32): Toggle Pause / Resume
        if keycode == 32:
            self.paused = not self.paused
            state_str = "PAUSED" if self.paused else "RUNNING"
            print(f"\n[USER INPUT] Simulation {state_str}")

        # 'R' or 'r' (ASCII 82 or 114): Reset simulation
        elif keycode in (ord("r"), ord("R")):
            self.sim.reset()
            print("\n[USER INPUT] Simulation RESET to initial standing pose.")

        # 'C' or 'c' (ASCII 67 or 99): Cycle Camera presets
        elif keycode in (ord("c"), ord("C")):
            self.camera_idx = (self.camera_idx + 1) % len(self.camera_modes)
            mode = self.camera_modes[self.camera_idx]
            print(f"\n[USER INPUT] Camera switched to: {mode.upper()}")

        # Tab (ASCII 9) or 'S' (ord('s')): Step single frame when paused
        elif keycode in (9, ord("s"), ord("S")):
            self.step_single = True

    def apply_camera(self, viewer) -> None:
        """Apply active camera viewpoint to viewer."""
        mode = self.camera_modes[self.camera_idx]
        preset = config.CAMERA_FULL_BODY
        if mode == "close_up":
            preset = config.CAMERA_CLOSE_UP
        elif mode == "side_toss":
            preset = config.CAMERA_SIDE_TOSS

        viewer.cam.azimuth = preset["azimuth"]
        viewer.cam.elevation = preset["elevation"]
        viewer.cam.distance = preset["distance"]
        viewer.cam.lookat[:] = preset["lookat"]

    def print_hud(self, telemetry: dict) -> None:
        """Render a clean in-terminal status overlay."""
        t = telemetry["time"]
        phase = telemetry["phase"]
        grasp = telemetry["grasp_state"]
        dist_cm = telemetry["catch_dist"] * 100.0

        if grasp == GraspState.CAUGHT_LEFT.value:
            status_indicator = "SUCCESS [TRANSFER COMPLETED]"
        elif grasp == GraspState.IN_FLIGHT.value:
            status_indicator = "IN FLIGHT [BALLISTIC TRAJECTORY]"
        elif grasp == GraspState.HELD_RIGHT.value:
            status_indicator = "HELD [RIGHT HAND PREPARATION]"
        else:
            status_indicator = grasp

        sys.stdout.write(
            f"\r[t={t:5.2f}s] [{phase[:28]:<28}] [{status_indicator:<28}] Catch Dist: {dist_cm:4.1f}cm"
        )
        sys.stdout.flush()

    def run_headless(self, max_time: float = config.T_SIMULATION_MAX) -> None:
        """Run physics simulation without a graphical viewer."""
        print("\n" + "=" * 70)
        print(" RUNNING HEADLESS UNITREE G1 APPLE TOSS SIMULATION")
        print("=" * 70)

        dt = self.sim.model.opt.timestep
        total_steps = int(max_time / dt)

        for step in range(total_steps):
            telemetry = self.sim.step()
            if step % 250 == 0 or step == total_steps - 1:
                self.print_hud(telemetry)

        print(f"\nSimulation finished at t={self.sim.data.time:.2f}s.")
        print(f"Final Grasp State: {self.sim.grasp_ctrl.state.value}")

    def run_viewer(self) -> None:
        """Launch interactive passive viewer with real-time sync and HUD."""
        print("\n" + "=" * 70)
        print(" UNITREE G1 HUMANOID: APPLE TOSS & CATCH SIMULATION")
        print("=" * 70)
        print(" Controls:")
        print("   [SPACE]     : Pause / Resume simulation")
        print("   [R]         : Reset simulation to standing state")
        print("   [C]         : Cycle camera viewpoints (Full Body / Close-up / Side)")
        print("   [S / TAB]   : Advance single step when paused")
        print("=" * 70 + "\n")

        with launch_passive(self.sim.model, self.sim.data, key_callback=self.key_callback) as viewer:
            # Set initial camera view
            self.apply_camera(viewer)

            last_hud_update = 0.0
            last_cam_idx = self.camera_idx

            while viewer.is_running():
                step_start = time.time()

                # Update camera if switched
                if self.camera_idx != last_cam_idx:
                    self.apply_camera(viewer)
                    last_cam_idx = self.camera_idx

                # Physics step if running or single-stepping
                if not self.paused or self.step_single:
                    telemetry = self.sim.step()
                    self.step_single = False

                    # Update terminal HUD periodically (10 Hz)
                    wall_now = time.time()
                    if wall_now - last_hud_update > 0.1:
                        self.print_hud(telemetry)
                        last_hud_update = wall_now

                    # Auto loop check
                    if self.auto_loop and self.sim.data.time >= config.T_SIMULATION_MAX:
                        time.sleep(1.0)
                        self.sim.reset()
                        print("\n[AUTO-LOOP] Simulation reset for next iteration.")

                # Synchronize visual viewer
                viewer.sync()

                # Maintain realistic real-time simulation pacing
                elapsed = time.time() - step_start
                sleep_duration = self.sim.model.opt.timestep - elapsed
                if sleep_duration > 0:
                    time.sleep(sleep_duration)

        print(f"\nSimulation closed. Final State: {self.sim.grasp_ctrl.state.value}")


def main():
    parser = argparse.ArgumentParser(description="Unitree G1 Apple Toss Simulation")
    parser.add_argument("--headless", action="store_true", help="Run without graphical viewer")
    parser.add_argument("--auto-loop", action="store_true", help="Automatically reset and loop simulation")
    parser.add_argument(
        "--camera",
        choices=["full_body", "close_up", "side_toss"],
        default="full_body",
        help="Initial camera viewpoint",
    )
    args = parser.parse_args()

    app = AppleTossApp(headless=args.headless, auto_loop=args.auto_loop, camera_preset=args.camera)

    if args.headless:
        app.run_headless()
    else:
        app.run_viewer()


if __name__ == "__main__":
    main()

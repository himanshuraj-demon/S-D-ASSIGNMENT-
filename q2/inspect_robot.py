"""
Robot and Scene Model Inspection Utility for Unitree G1.
Reports joint, actuator, body, and constraint properties of both the baseline
Menagerie model and the generated apple toss simulation model.
"""

import sys
from pathlib import Path
import mujoco

import config
from simulation.model_builder import build_simulation_model


def inspect_model(model: mujoco.MjModel, data: mujoco.MjData, title: str) -> None:
    print("\n" + "=" * 70)
    print(f" {title.upper()}")
    print("=" * 70)
    print(f"Total Joints:     {model.njnt}")
    print(f"Total Actuators:  {model.nu}")
    print(f"Total Bodies:     {model.nbody}")
    print(f"Total Geoms:      {model.ngeom}")
    print(f"Equality Welds:   {model.neq}")
    print(f"Keyframes:        {model.nkey}")

    print("\n--- ARM ACTUATORS & RANGES ---")
    for name in config.ALL_ARM_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if jid < 0 or aid < 0:
            print(f"  {name:30}: NOT FOUND")
            continue

        qadr = model.jnt_qposadr[jid]
        current_val = data.qpos[qadr]
        ctrl_range = model.actuator_ctrlrange[aid]
        jnt_range = model.jnt_range[jid]

        print(
            f"  {name:28} | joint_id={jid:2} | act_id={aid:2} | "
            f"qpos={current_val:6.3f} | ctrl_range=[{ctrl_range[0]:6.3f}, {ctrl_range[1]:6.3f}]"
        )

    print("\n--- HAND & APPLE BODY POSITIONS (STANDING) ---")
    tracked_bodies = [config.PELVIS_BODY, config.RIGHT_HAND_BODY, config.LEFT_HAND_BODY]
    apple_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, config.APPLE_BODY)
    if apple_bid >= 0:
        tracked_bodies.append(config.APPLE_BODY)

    for bname in tracked_bodies:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bname)
        if bid >= 0:
            pos = data.xpos[bid]
            print(f"  {bname:25} pos = [{pos[0]:8.4f}, {pos[1]:8.4f}, {pos[2]:8.4f}]")

    if model.neq > 0:
        print("\n--- EQUALITY CONSTRAINTS ---")
        for i in range(model.neq):
            eq_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
            b1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.eq_obj1id[i])
            b2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.eq_obj2id[i])
            active = bool(data.eq_active[i])
            print(f"  Constraint #{i}: '{eq_name}' connecting '{b1}' <-> '{b2}' | active={active}")


def main():
    # Inspect generated apple toss model
    model, data = build_simulation_model(force_rebuild=False)
    inspect_model(model, data, "Unitree G1 Apple Toss Simulation Model")


if __name__ == "__main__":
    main()

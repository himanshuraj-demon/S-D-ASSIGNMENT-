"""
Model Builder: Generates the customized Unitree G1 model with apple asset,
equality constraints for physically consistent grasping, and enhanced visual scene.
Leaves original Menagerie assets completely untouched.
"""

import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Tuple
import mujoco
import numpy as np

import config


def build_apple_xml_elements(parent_worldbody: ET.Element, apple_pos: np.ndarray) -> None:
    """Add the physical apple body and its compound geoms into the worldbody."""
    apple_body = ET.SubElement(
        parent_worldbody,
        "body",
        name=config.APPLE_BODY,
        pos=f"{apple_pos[0]:.6f} {apple_pos[1]:.6f} {apple_pos[2]:.6f}"
    )

    # Free joint for 6-DOF dynamic ballistic motion
    ET.SubElement(apple_body, "freejoint", name=config.APPLE_JOINT)

    # Physical collision sphere (group 3 for physical collisions)
    ET.SubElement(
        apple_body,
        "geom",
        name=config.APPLE_GEOM,
        type="sphere",
        size=str(config.APPLE_RADIUS),
        mass=str(config.APPLE_MASS),
        friction="1.2 0.01 0.001",
        contype="1",
        conaffinity="1",
        group="3",
        priority="1"
    )

    # High quality visual geoms (group 2, contype 0, conaffinity 0)
    ET.SubElement(
        apple_body,
        "geom",
        name="apple_body_vis",
        type="sphere",
        size=str(config.APPLE_RADIUS),
        material="apple_skin",
        contype="0",
        conaffinity="0",
        group="2"
    )

    ET.SubElement(
        apple_body,
        "geom",
        name="apple_top_vis",
        type="sphere",
        pos="0 0 0.012",
        size="0.028",
        material="apple_skin",
        contype="0",
        conaffinity="0",
        group="2"
    )

    ET.SubElement(
        apple_body,
        "geom",
        name="apple_stem_vis",
        type="cylinder",
        pos="0 0 0.040",
        size="0.0022 0.010",
        quat="0.98 0.15 0 0.1",
        material="apple_stem",
        contype="0",
        conaffinity="0",
        group="2"
    )

    ET.SubElement(
        apple_body,
        "geom",
        name="apple_leaf_vis",
        type="ellipsoid",
        pos="0.008 0.004 0.046",
        size="0.013 0.006 0.002",
        quat="0.88 0.25 0.38 -0.15",
        material="apple_leaf",
        contype="0",
        conaffinity="0",
        group="2"
    )


def generate_g1_with_apple_xml(out_path: Path) -> None:
    """Generate the modified G1 XML model including the apple and weld equality constraints."""
    tree = ET.parse(config.ORIGINAL_ROBOT_XML)
    root = tree.getroot()

    # Direct meshdir relative to the generated folder
    compiler = root.find("compiler")
    if compiler is not None:
        rel_meshdir = os.path.relpath(config.MENAGERIE_DIR / "assets", out_path.parent)
        compiler.set("meshdir", rel_meshdir.replace("\\", "/"))

    # Add apple materials to asset section
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")

    ET.SubElement(
        asset,
        "texture",
        name="apple_red_tex",
        type="2d",
        builtin="gradient",
        rgb1="0.95 0.12 0.08",
        rgb2="0.65 0.04 0.02",
        width="256",
        height="256"
    )
    ET.SubElement(
        asset,
        "material",
        name="apple_skin",
        texture="apple_red_tex",
        specular="0.8",
        shininess="0.7",
        reflectance="0.15"
    )
    ET.SubElement(
        asset,
        "material",
        name="apple_stem",
        rgba="0.32 0.18 0.08 1",
        specular="0.2",
        shininess="0.1"
    )
    ET.SubElement(
        asset,
        "material",
        name="apple_leaf",
        rgba="0.18 0.75 0.18 1",
        specular="0.5",
        shininess="0.5"
    )

    # Initial apple world position matching right wrist in standing pose
    # Computed from FK of right_wrist_yaw_link with APPLE_RIGHT_HAND_OFFSET
    init_apple_pos = np.array([-0.020985, -0.234876, 0.656580])

    # Add apple to worldbody
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("No worldbody element found in robot XML.")
    build_apple_xml_elements(worldbody, init_apple_pos)

    # Update standing keyframe to include the apple's 7 coordinates
    keyframe = root.find("keyframe")
    if keyframe is not None:
        key = keyframe.find("key")
        if key is not None and key.get("name") == config.KEYFRAME_STAND:
            orig_qpos = key.get("qpos")
            apple_qpos_str = f" {init_apple_pos[0]:.6f} {init_apple_pos[1]:.6f} {init_apple_pos[2]:.6f} 1 0 0 0"
            key.set("qpos", orig_qpos + apple_qpos_str)

    # Add equality constraints for physical weld grasping
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")

    # Initial weld holding apple to right hand (active)
    relpose_r = f"{config.APPLE_RIGHT_HAND_OFFSET[0]} {config.APPLE_RIGHT_HAND_OFFSET[1]} {config.APPLE_RIGHT_HAND_OFFSET[2]} 1 0 0 0"
    ET.SubElement(
        equality,
        "weld",
        name=config.RIGHT_WELD_NAME,
        body1=config.RIGHT_HAND_BODY,
        body2=config.APPLE_BODY,
        active="true",
        relpose=relpose_r,
        solref="0.005 1",
        solimp="0.99 0.999 0.001"
    )

    # Weld for catching apple in left hand (inactive initially)
    ET.SubElement(
        equality,
        "weld",
        name=config.LEFT_WELD_NAME,
        body1=config.LEFT_HAND_BODY,
        body2=config.APPLE_BODY,
        active="false",
        solref="0.005 1",
        solimp="0.99 0.999 0.001"
    )

    # Contact exclusions: exclude contact between apple and robot arm links/torso
    # to prevent mesh-clipping repulsive impulses during release and catch
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    
    arm_bodies = [
        "right_shoulder_pitch_link", "right_shoulder_roll_link", "right_shoulder_yaw_link",
        "right_elbow_link", "right_wrist_roll_link", "right_wrist_pitch_link", "right_wrist_yaw_link",
        "left_shoulder_pitch_link", "left_shoulder_roll_link", "left_shoulder_yaw_link",
        "left_elbow_link", "left_wrist_roll_link", "left_wrist_pitch_link", "left_wrist_yaw_link",
        "torso_link"
    ]
    for bname in arm_bodies:
        ET.SubElement(contact, "exclude", body1=bname, body2=config.APPLE_BODY)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(out_path, encoding="utf-8", xml_declaration=True)


def generate_scene_with_apple_xml(out_path: Path, robot_xml_path: Path) -> None:
    """Generate the complete scene XML with floor, lighting, cameras, and robot inclusion."""
    rel_robot_file = os.path.relpath(robot_xml_path, out_path.parent).replace("\\", "/")

    scene_xml = f"""<mujoco model="unitree_g1_apple_toss_scene">
  <include file="{rel_robot_file}"/>

  <visual>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.25 0.25 0.25" specular="0.9 0.9 0.9"/>
    <rgba haze="0.15 0.22 0.32 1"/>
    <global azimuth="150" elevation="-20"/>
    <quality shadowsize="4096"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.35 0.55 0.75" rgb2="0.08 0.12 0.18" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.22 0.26 0.30" rgb2="0.14 0.17 0.21"
      markrgb="0.75 0.80 0.85" width="400" height="400"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="6 6" reflectance="0.25"/>
  </asset>

  <worldbody>
    <!-- Primary studio directional lighting with shadows -->
    <light pos="1.5 -1.0 3.8" dir="-0.3 0.2 -1.0" directional="true" castshadow="true" diffuse="0.85 0.85 0.85" specular="0.3 0.3 0.3"/>
    
    <!-- Soft fill light from opposite angle -->
    <light pos="-1.5 1.5 3.0" dir="0.4 -0.4 -1.0" directional="true" castshadow="false" diffuse="0.35 0.35 0.40"/>
    
    <!-- Floor plane -->
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane" friction="1.0 0.01 0.001"/>

    <!-- Academic Demonstration Cameras -->
    <!-- Full-body perspective camera -->
    <camera name="demo_full_body" pos="2.1 -0.4 1.15" xyaxes="0.22 0.975 0 -0.16 0.035 0.985"/>
    
    <!-- Close-up toss and catch action camera -->
    <camera name="demo_action_closeup" pos="1.45 -0.25 0.95" xyaxes="0.18 0.98 0 -0.18 0.035 0.98"/>
    
    <!-- Front-facing symmetrical camera -->
    <camera name="demo_front" pos="2.1 0.0 1.0" xyaxes="0 1 0 -0.12 0 0.99"/>
  </worldbody>
</mujoco>
"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(scene_xml)


def build_simulation_model(force_rebuild: bool = False) -> Tuple[mujoco.MjModel, mujoco.MjData]:
    """
    Ensure generated XML files exist, compile the MuJoCo model,
    initialize to the standing keyframe, and return (model, data).
    """
    if force_rebuild or not config.GENERATED_ROBOT_XML.exists():
        generate_g1_with_apple_xml(config.GENERATED_ROBOT_XML)

    if force_rebuild or not config.GENERATED_SCENE_XML.exists():
        generate_scene_with_apple_xml(config.GENERATED_SCENE_XML, config.GENERATED_ROBOT_XML)

    model = mujoco.MjModel.from_xml_path(str(config.GENERATED_SCENE_XML))
    data = mujoco.MjData(model)

    # Initialize to standing keyframe
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, config.KEYFRAME_STAND)
    if stand_id < 0:
        raise RuntimeError(f"Standing keyframe '{config.KEYFRAME_STAND}' not found in model.")

    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)

    return model, data

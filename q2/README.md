# Unitree G1 Humanoid Apple Toss Simulation

A physically consistent, high-fidelity robotics simulation in **MuJoCo** where a **Unitree G1 humanoid robot** tosses an apple from its right hand and catches it with its left hand, maintaining full standing posture and balance throughout.

---

## 1. Assignment Objective

The primary objective of this project is to implement a realistic, reliable, and repeatable apple toss task on the 29-DOF Unitree G1 humanoid robot using MuJoCo physics:
1. **Phase 1 – Initial State**: The robot stands stable on a ground plane, holding a realistic red apple near its right wrist.
2. **Phase 2 – Preparation**: Both arms move in a smooth, minimum-jerk coordinated motion into throwing and catching postures.
3. **Phase 3 – Toss**: The right arm swings forward and upward, accelerating the apple along an upward arc, releasing it at the optimal apex.
4. **Phase 4 – Ballistic Flight**: The apple becomes an independent 6-DOF dynamic rigid body governed purely by gravity ($\vec{g} = [0, 0, -9.81]\text{ m/s}^2$).
5. **Phase 5 – Catch & Stabilization**: The left hand intercepts the apple in its receiving zone, dynamically activating a MuJoCo weld constraint at the exact relative pose without teleportation, followed by gentle cushioning into a stable holding posture.

---

## 2. Simulation Architecture

The codebase is organized into modular components adhering to professional robotics engineering standards:

```text
q2/
│
├── config.py                       # Global parameters (paths, timings, gains, thresholds)
├── main.py                         # Primary application entrypoint & interactive viewer
├── validate_simulation.py          # Automated 16-point physics & control test suite
├── inspect_robot.py                # Model introspection utility
├── test_arms.py                    # Arm motion & standing stability tester
├── apple_toss.py                   # Legacy entrypoint (wraps main.py)
├── requirements.txt                # Python package dependencies
├── README.md                       # Complete technical documentation
│
├── controllers/                    # Robot control algorithms
│   ├── __init__.py
│   ├── robot_controller.py         # 29-actuator rate-limited PD/position controller
│   ├── motion_planner.py           # Quintic minimum-jerk trajectory generator
│   ├── grasp_controller.py         # Teleportation-free MuJoCo weld equality manager
│   └── inverse_kinematics.py       # Damped Least Squares (DLS) Jacobian IK solver
│
├── simulation/                     # Simulation environment & asset builders
│   ├── __init__.py
│   ├── model_builder.py            # Generates isolated scene XML with lights & camera
│   ├── apple.py                    # Apple 6-DOF state utilities & ballistic solvers
│   └── simulation_manager.py       # High-level state machine & physics coordinator
│
├── assets/                         # Static simulation assets
│   └── apple/
│       ├── apple_visual.xml        # Visual apple asset snippet
│       └── README.md               # Apple asset documentation
│
└── mujoco_menagerie/               # Original DeepMind Menagerie models (untouched)
    └── unitree_g1/
        ├── assets/                 # Robot meshes and textures
        ├── g1.xml                  # 29-DOF Unitree G1 robot model
        └── scene.xml               # Default Menagerie scene
```

---

## 3. Installation & Virtual Environment Setup (Windows)

### Prerequisites
- Windows 10 or 11 (64-bit)
- Python 3.10+ (tested on Python 3.14)
- PowerShell or Windows Command Prompt

### Step-by-Step Setup
Open PowerShell in the project directory (`q2/`):

```powershell
# 1. Create a Python virtual environment (if not already present)
python -m venv .venv

# 2. Activate the virtual environment
.\.venv\Scripts\Activate.ps1
# (If execution policies block activation, run: Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass)

# 3. Upgrade pip
python -m pip install --upgrade pip

# 4. Install required dependencies
pip install -r requirements.txt
```

---

## 4. How to Launch the Simulation

### Primary Launch Command (Interactive GUI Viewer)
```powershell
python main.py
```

### Viewer Keyboard Shortcuts
- **`[SPACE]`**: Pause / Resume simulation.
- **`[R]`**: Reset simulation to initial standing pose at $t = 0$.
- **`[C]`**: Cycle camera viewpoints (Full Body View $\leftrightarrow$ Action Close-Up $\leftrightarrow$ Side View).
- **`[S]` or `[TAB]`**: Advance a single physics step while paused.

### Additional Command-Line Options
```powershell
# Run continuously with automatic looping/reset (ideal for video recording)
python main.py --auto-loop

# Start with a dramatic action close-up camera
python main.py --camera close_up

# Run headlessly in terminal (no window opened)
python main.py --headless
```

---

## 5. Automated Validation & Testing

Run the comprehensive 16-point test suite:
```powershell
python validate_simulation.py
```

This verifies:
1. XML compilation & model integrity.
2. Apple 6-DOF free joint & body definitions.
3. Actuator availability for all 14 arm joints.
4. Standing keyframe loading & posture stability.
5. Standing vertical stability (pelvis drift $< 2\text{ mm}$).
6. Arm trajectory smoothness without jerk spikes.
7. Initial right-hand grasp attachment.
8. Dynamic ballistic release under unconstrained 6-DOF physics.
9. Pure gravitational acceleration ($\vec{a}_z = -9.81\text{ m/s}^2$).
10. Interception within left hand capture radius.
11. Teleportation-free weld constraint engagement.
12. Complete right-hand grasp release.
13. Retentive capture with zero post-catch drift ($< 3\text{ mm}$).
14. Post-catch humanoid balance retention.
15. Full reset and repeat reliability.

---

## 6. How the G1 Humanoid Robot Model Works

- **Base Kinematics**: 29 actuated joints (12 leg joints, 3 waist joints, 14 arm joints) plus a 6-DOF unactuated floating base at the pelvis (35 total generalized coordinates $qpos$).
- **Actuation**: All 29 actuators are MuJoCo `<position>` actuators with high stiffness ($k_p = 500\text{ N}\cdot\text{m/rad}$, critical damping ratio $\zeta = 1.0$) and joint torque limits:
  - Leg actuators: up to $139\text{ N}\cdot\text{m}$.
  - Waist actuators: up to $88\text{ N}\cdot\text{m}$.
  - Shoulder / Elbow actuators: up to $25\text{ N}\cdot\text{m}$.
  - Wrist actuators: up to $5\text{ N}\cdot\text{m}$.
- **Standing Stability**: Baseline posture commands for all lower body joints (`left_hip_*`, `right_hip_*`, `left_knee_*`, `waist_*`, etc.) are continuously preserved from the Menagerie `stand` keyframe. Actuator rate limiting ($\Delta \theta_{\max} = 4.0\text{ rad/s}$) guarantees smooth transitions without generating ground reaction torque spikes that could cause tipping.

---

## 7. How the Apple is Represented

- **Mass & Inertia**: Realistic 150-gram mass ($m = 0.150\text{ kg}$) with a radius of $3.5\text{ cm}$ ($70\text{ mm}$ diameter).
- **Free Joint**: A MuJoCo `<freejoint name="apple_free"/>` grants 6 independent degrees of freedom ($x, y, z$ translation and unit quaternion orientation $q_w, q_x, q_y, q_z$).
- **Collision Separation**:
  - The physical collision body is a spherical geom (`apple_collision`, group 3) with realistic friction parameters ($1.2$ sliding, $0.01$ torsional, $0.001$ rolling).
  - High-fidelity visual components (glossy red apple body, upper crown taper, brown woody stem, fresh green leaf) are placed in group 2 with `contype="0"` and `conaffinity="0"`, preventing visual geometry from distorting contact physics.
  - Contact exclusions between the apple and the robot's arm links prevent mesh-clipping repulsive force spikes when releasing or catching.

---

## 8. Physical Grasping & Release (Zero Teleportation)

The legacy prototype used manual coordinate overwrites (`data.qpos = target`), which constitutes unphysical teleportation. This simulation replaces that with **MuJoCo Weld Equality Constraints**:

### Initial Grasp (Right Hand)
In the generated XML, an equality constraint connects the right wrist link (`right_wrist_yaw_link`) to the apple:
```xml
<weld name="right_hand_apple" body1="right_wrist_yaw_link" body2="apple"
      active="true" relpose="0.065 0 -0.01 1 0 0 0" solref="0.005 1" solimp="0.99 0.999 0.001"/>
```
The apple is initialized with matching coordinates, ensuring zero initial strain or impulse.

### Ballistic Release
At $t = T_{\text{RELEASE}} = 2.50\text{s}$:
1. The right weld constraint is deactivated: `data.eq_active[right_weld_id] = False`.
2. The calculated analytical launch velocity $\vec{v}_0$ is assigned to the apple's generalized velocity degrees of freedom: `data.qvel[apple_dof : apple_dof + 3] = v0`.
3. The apple enters pure ballistic free-flight governed by MuJoCo's symplectic Euler/implicit-fast physics integrator.

### Teleportation-Free Catch (Left Hand)
When the apple enters the left hand's capture sphere ($r < 8.5\text{ cm}$) with acceptable relative velocity:
1. The instantaneous relative position and orientation are computed:
   $$\vec{p}_{\text{rel}} = \mathbf{R}_{\text{left}}^T (\vec{p}_{\text{apple}} - \vec{p}_{\text{left}})$$
   $$\mathbf{R}_{\text{rel}} = \mathbf{R}_{\text{left}}^T \mathbf{R}_{\text{apple}} \longrightarrow \vec{q}_{\text{rel}}$$
2. The left weld constraint's relative pose parameters are updated dynamically:
   `model.eq_data[left_weld_id, 3:6] = rel_pos`
   `model.eq_data[left_weld_id, 6:10] = rel_quat`
3. The left weld is engaged: `data.eq_active[left_weld_id] = True`.

Because the constraint is initialized to the exact current relative transform, **no displacement or velocity discontinuity occurs**.

---

## 9. Mathematical & Physical Equations

### 1. Minimum-Jerk Trajectory Interpolation
To avoid sudden acceleration spikes, arm trajectories transition between waypoints using a 5th-order polynomial blend:
$$s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5, \quad \tau = \frac{t - t_0}{t_1 - t_0} \in [0, 1]$$
Properties:
$$s(0) = 0, \quad s(1) = 1, \quad \dot{s}(0) = \dot{s}(1) = 0, \quad \ddot{s}(0) = \ddot{s}(1) = 0$$

### 2. Projectile Ballistics (Launch Velocity Solver)
Given the launch position $\vec{p}_0$, the target catch position $\vec{p}_{\text{catch}}$, and the flight duration $T_{\text{flight}} = 0.52\text{s}$:
$$\vec{p}(T) = \vec{p}_0 + \vec{v}_0 T + \frac{1}{2} \vec{g} T^2 = \vec{p}_{\text{catch}}$$
Solving for the required initial velocity vector $\vec{v}_0$:
$$\vec{v}_0 = \frac{\vec{p}_{\text{catch}} - \vec{p}_0 - \frac{1}{2}\vec{g} T^2}{T}$$
Apex of the ballistic arc occurs at:
$$t_{\text{apex}} = \frac{v_{0, z}}{|g_z|} \approx 0.27\text{s}$$
$$z_{\text{apex}} = z_0 + \frac{v_{0, z}^2}{2 |g_z|} \approx 1.05\text{m}$$

### 3. Inverse Kinematics (Damped Least Squares)
When solving Cartesian wrist targets for the 7-DOF arm, the Damped Least Squares (Levenberg-Marquardt) update rule is employed:
$$\Delta \vec{\theta} = \mathbf{J}^T (\mathbf{J} \mathbf{J}^T + \lambda^2 \mathbf{I})^{-1} \vec{e}$$
where $\mathbf{J} \in \mathbb{R}^{3 \times 7}$ is the position Jacobian, $\vec{e} = \vec{p}_{\text{target}} - \vec{p}_{\text{wrist}}$, and $\lambda = 0.05$ prevents kinematic singularities.

---

## 10. Known Simplifications & Technical Limitations

1. **G1 Hand Geometry**: The baseline Unitree G1 Menagerie model (`g1.xml`) terminates at the wrist pitch/yaw links without multi-articulated finger tendons. As sanctioned in prompt Section 3.C, dynamic weld equality constraints represent the contact grasp forces.
2. **Ground Contact**: Ground reaction uses MuJoCo's pyramid friction solver with default menagerie foot spheres. High-speed running or jumping is outside the scope of standing balance.
3. **Aerodynamic Drag**: Given the low Reynolds number and short flight distance ($< 0.5\text{m}$), air resistance is negligible and omitted.

---

## 11. Troubleshooting

| Issue | Cause | Solution |
| :--- | :--- | :--- |
| `FileNotFoundError` for Menagerie assets | Working directory mismatch | Run scripts from the project root `q2/` |
| Robot tilts or tips over | Incompatible leg actuator commands | Never overwrite leg actuators; use `RobotController` |
| Apple flies erratically upon release | Arm link collision mesh penetration | Ensure `model_builder.py` contact exclusions are active |
| Viewer window does not open | Headless server or missing OpenGL display | Run with `--headless` flag or verify graphics drivers |

---

## 12. Recording Demonstration Guide (2–3 Minutes)

Follow this sequence for the YouTube demonstration:

1. **Introduction & Robot Setup (0:00 - 0:30)**:
   - Launch simulation: `python main.py`
   - Point out the full Unitree G1 humanoid standing stably on the studio plane.
   - Show the red apple with leaf and stem held near the right wrist.
2. **Preparation & Throwing Wind-Up (0:30 - 1:00)**:
   - Hit `[SPACE]` to start.
   - Highlight the smooth quintic transition as the right arm winds up and the left arm opens into the ready catch posture.
3. **Ballistic Toss & Arc (1:00 - 1:40)**:
   - Switch camera view (`[C]`) to the close-up action view.
   - Show the right arm swing up and release the apple.
   - Point out the parabolic arc governed by gravity.
4. **Catch & Completion (1:40 - 2:20)**:
   - Show the left hand intercepting the apple at chest level.
   - Observe the smooth cushioning motion as the left arm secures the apple.
   - Point to the in-terminal HUD confirming `SUCCESS [TRANSFER COMPLETED]`.
5. **Repeatability (2:20 - 2:40)**:
   - Press `[R]` to reset and demonstrate repeatability.

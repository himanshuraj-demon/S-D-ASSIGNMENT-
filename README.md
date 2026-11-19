# Robotics & Physics Simulation Assignment

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![MuJoCo](https://img.shields.io/badge/physics-MuJoCo%203.1+-orange.svg)](https://mujoco.org/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

This repository contains the complete implementation and technical documentation for both questions of the assignment:
- **Question 1 (`q1/`)**: Interactive Newtonian 2-Body Orbital Simulator in 2D with real-time conservation tracking and interactive GUI widgets.
- **Question 2 (`q2/`)**: Unitree G1 Humanoid Apple Toss & Catch Simulation in **MuJoCo** with minimum-jerk arm trajectory control, teleportation-free dynamic weld constraints, ballistic flight integration, and standing stability preservation.

---

## Repository Structure

```text
.
├── .gitignore                      # Ignore cache, .venv, logs, and compiled binaries
├── .gitmodules                     # Git submodule configuration for MuJoCo Menagerie
├── README.md                       # Main repository overview and instructions
├── q1/                             # Question 1: Newtonian 2-Body Simulator
│   ├── physics.py                  # Analytical equations of motion, ODE solver & invariants
│   ├── main.py                     # Matplotlib animation GUI, sliders, pause/reset
│   ├── requirements.txt            # Python dependencies for Q1
│   └── README.md                   # Technical documentation for Q1
└── q2/                             # Question 2: Unitree G1 Humanoid Apple Toss
    ├── main.py                     # MuJoCo interactive 3D simulation viewer
    ├── validate_simulation.py      # Automated 16-point physics & stability test suite
    ├── inspect_robot.py            # Robot kinematic introspection tool
    ├── test_arms.py                # Stand-alone arm trajectory & balance tester
    ├── apple_toss.py               # Application wrapper
    ├── config.py                   # Central simulation hyperparameters & paths
    ├── requirements.txt            # Python dependencies for Q2
    ├── README.md                   # Complete robotics technical documentation for Q2
    ├── controllers/                # PD position controller, IK solver, grasp manager
    ├── simulation/                 # XML model assembler, scene builder, ballistic coordinator
    ├── assets/                     # Custom visual assets (apple geom, stem, leaf)
    └── mujoco_menagerie/           # DeepMind MuJoCo Menagerie models (Unitree G1)
```

---

## Getting Started

### 1. Clone the Repository
Clone the repository along with its submodules:
```bash
git clone --recurse-submodules https://github.com/himanshuraj-demon/S-D-ASSIGNMENT-.git
cd S-D-ASSIGNMENT-
```

If already cloned without `--recurse-submodules`, initialize submodules with:
```bash
git submodule update --init --recursive
```

---

## Question 1: Interactive Newtonian 2-Body Orbital Simulator

### Overview
A 2D gravitational simulator simulating the mutual gravitational attraction between two bodies ($m_1, m_2$) under Newton's Law of Universal Gravitation ($F = G \frac{m_1 m_2}{r^2}$).
- Solved using SciPy's high-precision adaptive Runge-Kutta ODE integrator (`solve_ivp`, `rtol=1e-9`, `atol=1e-11`).
- Real-time live HUD monitoring total mechanical energy ($E = K + U$) and angular momentum ($L_z$).
- Interactive Matplotlib UI with parameter sliders (`m1`, `m2`, `separation`, `velocity`) and control buttons (`Pause`, `Reset`).

### Running Q1
```powershell
cd q1
pip install -r requirements.txt
python main.py
```

*For in-depth mathematical formulations and details, see [q1/README.md](q1/README.md).*

---

## Question 2: Unitree G1 Humanoid Apple Toss & Catch Simulation

### Overview
A physics-consistent robotics simulation in **MuJoCo** featuring the 29-DOF Unitree G1 humanoid robot:
1. **Initial Posture**: Humanoid stands stably on a ground plane holding a 150g apple at the right wrist.
2. **Preparation Phase**: Coordinated minimum-jerk arm motion using 5th-order polynomial interpolation ($\Delta \theta_{\max} = 4.0\text{ rad/s}$).
3. **Ballistic Toss**: Right arm swings upward and releases the apple at $t = 2.50\text{s}$, giving it an analytically computed ballistic velocity vector $\vec{v}_0$.
4. **Free Flight**: Apple flies on an unconstrained 6-DOF parabolic arc governed purely by gravity ($\vec{g} = [0, 0, -9.81]\text{ m/s}^2$).
5. **Teleportation-Free Catch**: Left hand intercepts the apple in mid-air, activating a dynamic MuJoCo weld equality constraint at the exact relative transform with zero positional discontinuity.
6. **Balance Preservation**: Lower body joints remain locked to the validated `stand` keyframe, ensuring standing pelvis drift $< 2\text{ mm}$ throughout the entire routine.

### Running Q2
```powershell
cd q2

# Install dependencies (virtual environment recommended)
pip install -r requirements.txt

# Run interactive 3D simulation
python main.py

# Run automated 16-point validation suite
python validate_simulation.py
```

### Viewer Shortcuts (Q2)
- `[SPACE]`: Pause / Resume simulation.
- `[R]`: Reset simulation to initial standing pose at $t = 0$.
- `[C]`: Cycle camera views (Full Body $\leftrightarrow$ Action Close-Up $\leftrightarrow$ Side View).
- `[S]` / `[TAB]`: Step simulation one frame forward while paused.

*For detailed kinematics, actuator gains, ballistic equations, and verification results, see [q2/README.md](q2/README.md).*

---

## Dependencies & Requirements

Both questions provide self-contained `requirements.txt` files:

- **Q1 (`q1/requirements.txt`)**:
  - `numpy>=1.24.0`
  - `scipy>=1.10.0`
  - `matplotlib>=3.7.0`

- **Q2 (`q2/requirements.txt`)**:
  - `mujoco>=3.1.0`
  - `numpy>=1.24.0`
  - `scipy>=1.10.0`
  - `matplotlib>=3.7.0`

---

## License
Distributed under the MIT License. See [LICENSE](q2/mujoco_menagerie/LICENSE) for MuJoCo Menagerie model terms.

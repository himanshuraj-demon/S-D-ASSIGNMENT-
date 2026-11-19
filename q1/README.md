# Question 1: Interactive Newtonian 2-Body Orbital Simulator

A high-precision, interactive 2D orbital simulation of a two-body gravitational system implemented in Python using **NumPy**, **SciPy**, and **Matplotlib**.

---

## 1. Overview & Physics Formulation

The simulation models the mutual gravitational interaction between two celestial bodies of masses $m_1$ and $m_2$ in a 2D Cartesian plane under Newton's Law of Universal Gravitation:

$$\vec{F}_{12} = G \frac{m_1 m_2}{r^3} (\vec{r}_2 - \vec{r}_1)$$

where:
- $G = 1.0$ (normalized gravitational constant)
- $\vec{r}_1 = (x_1, y_1)$ and $\vec{r}_2 = (x_2, y_2)$ are the position vectors of the bodies.
- $r = \|\vec{r}_2 - \vec{r}_1\| = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}$ is the Euclidean distance between them.
- A softening condition ($r < 10^{-8}$) prevents numerical singularities at close encounters.

### Equations of Motion (State Space)
The 8-dimensional state vector is defined as:
$$\vec{S} = [x_1, y_1, x_2, y_2, v_{x1}, v_{y1}, v_{x2}, v_{y2}]^T$$

The first-order system of differential equations is:
$$\dot{\vec{S}} = [v_{x1}, v_{y1}, v_{x2}, v_{y2}, a_{x1}, a_{y1}, a_{x2}, a_{y2}]^T$$

where:
$$\vec{a}_1 = \frac{G m_2}{r^3}(\vec{r}_2 - \vec{r}_1), \quad \vec{a}_2 = -\frac{G m_1}{r^3}(\vec{r}_2 - \vec{r}_1)$$

### Conservation Laws
The simulation tracks and displays two fundamental physical invariants:
1. **Total Mechanical Energy**:
   $$E = K + U = \frac{1}{2}m_1 v_1^2 + \frac{1}{2}m_2 v_2^2 - \frac{G m_1 m_2}{r}$$
2. **Total Angular Momentum** (about the coordinate origin):
   $$L_z = m_1 (x_1 v_{y1} - y_1 v_{x1}) + m_2 (x_2 v_{y2} - y_2 v_{x2})$$

---

## 2. Code Structure

```text
q1/
├── physics.py           # Core ODE solver, equations of motion, and physical invariants
├── main.py              # Interactive Matplotlib GUI with sliders, buttons, and animations
├── requirements.txt     # Python package dependencies
└── README.md            # Detailed documentation
```

### Module Descriptions
- **`physics.py`**:
  - `derivatives(t, state, m1, m2)`: Evaluates $\dot{\vec{S}}$ for the numerical integrator.
  - `simulate(m1, m2, separation, v1, v2, duration)`: Integrates the equations of motion using SciPy's adaptive Runge-Kutta solver (`solve_ivp`, `rtol=1e-9`, `atol=1e-11`) over 2,000 evaluation steps.
  - `calculate_energy(m1, m2, state)`: Computes total mechanical energy.
  - `calculate_angular_momentum(m1, m2, state)`: Computes total angular momentum.
- **`main.py`**:
  - Sets up the Matplotlib animation canvas, body markers, and fading orbital trails.
  - Implements interactive widgets:
    - **`m1` slider**: Adjust mass of Body 1 ($0.5$ to $15.0$).
    - **`m2` slider**: Adjust mass of Body 2 ($0.5$ to $15.0$).
    - **`separation` slider**: Initial orbital distance ($2.0$ to $8.0$).
    - **`velocity` slider**: Initial orbital speed scale ($0.05$ to $1.5$).
    - **Pause / Resume button**: Toggles real-time animation.
    - **Reset button**: Restores default orbital parameters.
  - Real-time diagnostic telemetry HUD updating live energy and angular momentum.

---

## 3. Installation & Setup

### Prerequisites
- Python 3.10+ (tested on Python 3.14)
- Virtual environment recommended

### Quick Start
```powershell
# Navigate into Question 1 directory
cd q1

# (Optional) Create and activate a virtual environment
python -m venv .venv
.\.venv\Scripts\Activate.ps1   # On Windows PowerShell
# source .venv/bin/activate    # On Linux / macOS

# Install required dependencies
pip install -r requirements.txt

# Run the interactive simulator
python main.py
```

---

## 4. Controls & Usage

| Control | Type | Description |
| :--- | :--- | :--- |
| **`m1` Slider** | Interactive Slider | Adjusts mass of Body 1 (Royal Blue). Re-computes orbit on change. |
| **`m2` Slider** | Interactive Slider | Adjusts mass of Body 2 (Crimson). Re-computes orbit on change. |
| **`separation` Slider** | Interactive Slider | Modifies initial distance between the bodies along the X-axis. |
| **`velocity` Slider** | Interactive Slider | Scales initial circular orbital speed $v_{\text{rel}} = \sqrt{(m_1 + m_2)/d}$. |
| **Pause / Resume** | Push Button | Freezes / unfreezes the current animation loop. |
| **Reset** | Push Button | Resets all sliders to factory default settings. |

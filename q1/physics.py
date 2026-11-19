
import numpy as np
from scipy.integrate import solve_ivp

G = 1.0  # Normalized gravitational constant


def derivatives(t, state, m1, m2):
    """
    State:
    [x1, y1, x2, y2, vx1, vy1, vx2, vy2]
    """

    x1, y1, x2, y2, vx1, vy1, vx2, vy2 = state

    dx = x2 - x1
    dy = y2 - y1

    r2 = dx**2 + dy**2
    r = np.sqrt(r2)

    if r < 1e-8:
        r = 1e-8

    # Gravitational acceleration
    a1x = G * m2 * dx / r**3
    a1y = G * m2 * dy / r**3

    a2x = -G * m1 * dx / r**3
    a2y = -G * m1 * dy / r**3

    return [
        vx1, vy1,
        vx2, vy2,
        a1x, a1y,
        a2x, a2y
    ]


def simulate(m1, m2, separation, v1, v2, duration=30):
    """
    Simulate the two-body system.
    """

    # Initial positions around the centre of mass
    x1 = -m2 * separation / (m1 + m2)
    x2 = m1 * separation / (m1 + m2)

    y1 = 0.0
    y2 = 0.0

    vx1, vy1 = v1
    vx2, vy2 = v2

    initial_state = [
        x1, y1, x2, y2,
        vx1, vy1, vx2, vy2
    ]

    t_eval = np.linspace(0, duration, 2000)

    solution = solve_ivp(
        derivatives,
        [0, duration],
        initial_state,
        args=(m1, m2),
        t_eval=t_eval,
        rtol=1e-9,
        atol=1e-11
    )

    return solution.t, solution.y


def calculate_energy(m1, m2, state):
    """
    Calculate total mechanical energy.
    """

    x1, y1, x2, y2, vx1, vy1, vx2, vy2 = state

    r = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    kinetic = (
        0.5 * m1 * (vx1**2 + vy1**2)
        + 0.5 * m2 * (vx2**2 + vy2**2)
    )

    potential = -G * m1 * m2 / r

    return kinetic + potential


def calculate_angular_momentum(m1, m2, state):
    x1, y1, x2, y2, vx1, vy1, vx2, vy2 = state

    L1 = m1 * (x1 * vy1 - y1 * vx1)
    L2 = m2 * (x2 * vy2 - y2 * vx2)

    return L1 + L2

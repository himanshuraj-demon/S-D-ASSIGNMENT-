
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.widgets import Slider, Button
from matplotlib.animation import FuncAnimation

from physics import (
    simulate,
    calculate_energy,
    calculate_angular_momentum
)


# -------------------------------
# Default parameters
# -------------------------------

params = {
    "m1": 5.0,
    "m2": 5.0,
    "separation": 4.0,
    "velocity": 0.5,
    "duration": 30
}

paused = False
frame_index = 0

t = None
states = None


def generate_simulation():
    global t, states, frame_index

    m1 = params["m1"]
    m2 = params["m2"]
    d = params["separation"]
    v = params["velocity"]

    # Initial circular-orbit velocities.
    # Relative speed for a circular orbit:
    v_rel = np.sqrt((m1 + m2) / d)

    v1 = (0, -m2 / (m1 + m2) * v_rel)
    v2 = (0, m1 / (m1 + m2) * v_rel)

    t, states = simulate(
        m1,
        m2,
        d,
        v1,
        v2,
        params["duration"]
    )

    frame_index = 0


# -------------------------------
# Initial simulation
# -------------------------------

generate_simulation()


# -------------------------------
# Plot layout
# -------------------------------

fig = plt.figure(figsize=(12, 8))
fig.suptitle("Interactive Newtonian 2-Body Simulator")

ax = fig.add_axes([0.08, 0.32, 0.60, 0.60])

ax.set_xlim(-8, 8)
ax.set_ylim(-8, 8)
ax.set_aspect("equal")
ax.set_xlabel("X position")
ax.set_ylabel("Y position")
ax.grid(True)

ax.set_title("Orbital Motion")

body1, = ax.plot([], [], "o", color="royalblue",
                 markersize=12, label="Body 1")

body2, = ax.plot([], [], "o", color="crimson",
                 markersize=12, label="Body 2")

trail1, = ax.plot([], [], "-", color="royalblue",
                  alpha=0.6)

trail2, = ax.plot([], [], "-", color="crimson",
                  alpha=0.6)

ax.legend(loc="upper right")

info = fig.text(0.73, 0.75, "", fontsize=10)


# -------------------------------
# Sliders
# -------------------------------

slider_data = [
    ("m1", 0.5, 15, 0.05),
    ("m2", 0.5, 15, 0.05),
    ("separation", 2, 8, 0.05),
    ("velocity", 0.05, 1.5, 0.01)
]

sliders = {}

for i, (name, minimum, maximum, step) in enumerate(slider_data):

    slider_ax = fig.add_axes(
        [0.12, 0.23 - i * 0.045, 0.45, 0.025]
    )

    sliders[name] = Slider(
        slider_ax,
        name,
        minimum,
        maximum,
        valinit=params[name],
        valstep=step
    )


# -------------------------------
# Buttons
# -------------------------------

pause_ax = fig.add_axes([0.72, 0.48, 0.12, 0.05])
reset_ax = fig.add_axes([0.86, 0.48, 0.12, 0.05])

pause_button = Button(pause_ax, "Pause")
reset_button = Button(reset_ax, "Reset")


def pause(event):
    global paused
    paused = not paused
    pause_button.label.set_text(
        "Resume" if paused else "Pause"
    )


def reset(event):
    for name in params:
        if name in sliders:
            sliders[name].reset()

    generate_simulation()


pause_button.on_clicked(pause)
reset_button.on_clicked(reset)


def slider_changed(value):
    for name in sliders:
        params[name] = sliders[name].val

    generate_simulation()


for slider in sliders.values():
    slider.on_changed(slider_changed)


# -------------------------------
# Animation
# -------------------------------

def update(frame):

    global frame_index

    if paused:
        return body1, body2, trail1, trail2

    if frame_index >= len(t):
        frame_index = 0

    i = frame_index

    x1 = states[0, i]
    y1 = states[1, i]

    x2 = states[2, i]
    y2 = states[3, i]

    body1.set_data([x1], [y1])
    body2.set_data([x2], [y2])

    trail1.set_data(states[0, :i+1], states[1, :i+1])
    trail2.set_data(states[2, :i+1], states[3, :i+1])

    energy = calculate_energy(
        params["m1"],
        params["m2"],
        states[:, i]
    )

    angular = calculate_angular_momentum(
        params["m1"],
        params["m2"],
        states[:, i]
    )

    info.set_text(
        f"Time: {t[i]:.2f}\n"
        f"Energy: {energy:.5f}\n"
        f"Angular Momentum: {angular:.5f}\n"
        f"Mass 1: {params['m1']:.2f}\n"
        f"Mass 2: {params['m2']:.2f}"
    )

    frame_index += 1

    return body1, body2, trail1, trail2


ani = FuncAnimation(
    fig,
    update,
    interval=20,
    blit=False,
    cache_frame_data=False
)

plt.show()

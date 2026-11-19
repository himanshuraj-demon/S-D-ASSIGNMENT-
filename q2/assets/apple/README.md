# Apple Physical and Visual Asset

This asset defines the dynamic apple used in the Unitree G1 humanoid toss simulation.

## Physical Specifications
- **Mass**: 150 grams ($0.150 \text{ kg}$) — standard realistic medium Honeycrisp / Gala apple.
- **Diameter**: 70 mm ($r = 0.035 \text{ m}$).
- **Joint**: 6-DOF free joint (`apple_free`) enabling unconstrained ballistic physics, momentum transfer, and collision dynamics.
- **Collision Geometry**: Spherical collision envelope (`apple_collision`, group 3) with contact friction parameters:
  - Slide friction: $1.2$
  - Torsional friction: $0.01$
  - Rolling friction: $0.001$

## Visual Design
- **Body Skin**: High-gloss gradient material (`apple_skin`) providing a vibrant, recognizable red appearance with specular highlight reflections.
- **Crown Taper**: Secondary upper sphere producing an organic tapered apple profile.
- **Stem**: Angled dark wood brown capsule ($2.2 \text{ mm}$ diameter).
- **Leaf**: Fresh green curved ellipsoid ($13 \times 6 \times 2 \text{ mm}$) oriented adjacent to the stem base.
- All visual elements belong to `group="2"` with `contype="0"` and `conaffinity="0"`, guaranteeing zero interference with physical contact solvers while delivering rich visual fidelity for demonstration recordings.

# 6DOF Rigid Body Rocket Flight Dynamics Simulator

## Mission

Build a 6 Degree of Freedom (6DOF) rigid body rocket flight dynamics simulator from the ground up, learning the underlying physics incrementally at each stage.

## Goal

A simulator that takes real rocket parameters as inputs and simulates altitude, downrange distance, velocity, attitude, and flight phases from ignition through impact. I want to proceed with each DOF, up to the point of 3DOF there will be no user input, this is for ease of develop as there are many variables up to that point forward.  

---

## What is a Rigid Body?

The rocket is treated as a perfectly rigid object — every part moves together, no structural deformation, no fuel slosh. This is the standard assumption in all practical 6DOF flight simulators and makes the equations of motion tractable.

---

## Degrees of Freedom Roadmap

| DOF | What it adds | Input Method |
|---|---|---|
| 1DOF | Vertical translation — altitude | User prompts |
| 2DOF | Horizontal translation — downrange distance | User prompts |
| 3DOF | Pitch rotation — rocket tilting | CLI prompts |
| 4DOF | Yaw — side to side pointing | CLI prompts (RK4) |
| 5DOF | Roll — spinning along centerline | CLI prompts (RK4) |
| 6DOF | Full free flight — all translations + all rotations | CLI prompts (RK4) |

---

## 1DOF — Vertical Rocket Flight

### Physics Model

A rocket launching vertically, burning fuel, coasting to apogee, and falling back to impact. One axis only — altitude.

### Forces

| Force | Equation | Notes |
|---|---|---|
| Gravity (force)| `F = mass * -9.81` | Shrinks as fuel burns |
| Thrust | `T = mass_flow_rate * Ve` | Active only while fuel remains |
| Drag | `F = -0.5 * rho * v² * Cd * A` | Always opposes velocity direction |

### Integrator — Euler Method

```
mass    = dryMass + fuelMass
thrust  = mass_flow_rate * Ve
v²      = velocity * velocity
netForce = 0
netForce += mass * gravity
netForce += -0.5 * rho * v² * Cd * A
if (fuelMass > 0): netForce += thrust
acceleration = netForce / mass
velocity += acceleration * dt
position += velocity * dt
if (fuelMass > 0): fuelMass -= mass_flow_rate * dt
```

### Flight Phases

1. **Powered ascent** — thrust overcomes gravity, acceleration increases as fuel burns and mass drops
2. **Burnout** — thrust cuts off, net force flips downward
3. **Unpowered ascent** — coasting upward, gravity and drag both decelerating the rocket
4. **Apogee** — velocity = 0, peak altitude
5. **Descent** — gravity pulls down, drag now pushes up, terminal velocity reached when they balance
6. **Impact** — position returns to 0

### User Inputs

| Variable | Description | Unit |
|---|---|---|
| `dryMass` | Vehicle mass with no fuel | kg |
| `fuelMass` | Propellant mass at launch | kg |
| `mass_flow_rate` | Engine fuel consumption rate | kg/s |
| `Ve` | Exhaust velocity — engine dependent | m/s |
| `Cd` | Drag coefficient — shape dependent | dimensionless |
| `A` | Cross-sectional reference area | m² |
| `position` | Starting altitude | m |
| `total_time` | Simulation duration | s |

`dt` is hardcoded at 0.01 seconds.

### Exhaust Velocity Reference

| Engine Type | Ve (m/s) |
|---|---|
| Solid rocket (APCP) | ~2,500 |
| Kerosene + LOX (Falcon 9) | ~3,050 |
| Hydrogen + LOX (Space Shuttle) | ~4,400 |
| Hydrazine (monopropellant) | ~2,200 |

### Key Physics Principles

- **Newton's Second Law**: `F = ma` — acceleration is derived from net force divided by mass
- **Variable mass**: As fuel burns each timestep, mass decreases, so gravitational force decreases and acceleration increases — this is why rockets accelerate harder near burnout
- **Thrust-to-weight ratio**: Rocket only lifts off if `thrust > mass * 9.81`. Below 1.0 the rocket never leaves the pad
- **Drag**: Proportional to velocity squared — doubles in speed means four times the drag
- **Euler integration**: Simple, sufficient for 1DOF. Higher DOF may require more accurate integration methods

---

## 6DOF — Full Rigid-Body Flight

The final stage: all three translations and all three rotations, integrated together from a single source of truth — an attitude **quaternion** plus the **body angular rates** `p, q, r`. The old per-axis Euler angles (`theta`, `phi`) are demoted to derived display outputs.

### Frames & Conventions

- **World frame**: `x` = horizontal, `y` = up (gravity along `-y`), `z` = depth.
- **Body frame**: `+x` = nose (roll / longitudinal axis), `+y` / `+z` = transverse axes.
- **Attitude**: unit quaternion `(qw, qx, qy, qz)` maps body → world (Hamilton convention). Initialized from the launch angles so the nose starts pointing along `(sinθcosφ, cosθ, sinθsinφ)`.
- **Body rates**: `p` = roll (about x), `q` = pitch (about y), `r` = yaw (about z).

### Forces (summed in the world frame)

| Force | Model | Notes |
|---|---|---|
| Gravity | `mass * -9.81` along world `-y` | shrinks as fuel burns |
| Thrust | `mass_flow_rate * Ve` along body `+x`, rotated to world by the quaternion | active while fuel remains |
| Drag | `-0.5 * rho * v * Cd * A * velocity` | opposes the velocity vector |
| Normal / side | `Cn_alpha * q_bar * A * (AoA, sideslip)` in the body transverse plane, rotated to world | restoring aero force |

Aerodynamic angles come from the **body-frame velocity** `(u, v, w) = worldToBody(q, V)`:
`AoA = atan2(w, u)`, `sideslip = atan2(v, u)`.

### Moments (about the body axes)

Computed as **arm × force** (`M = d × F_aero`) with `d = ((CG-CP)·L_ref, 0, 0)`. This automatically produces the correct **opposite signs** for pitch vs yaw — the classic `Cm_α < 0` but `Cn_β > 0` static-stability asymmetry, which arises because `d(AoA)/dt = +q` while `d(beta)/dt = -r`.

- **Pitch**: `M_y = +arm·N_pitch − damp·q`
- **Yaw**:   `M_z = −arm·N_yaw  − damp·r`
- **Roll**:  `M_x = Cn_alpha·q_bar·A·L_ref·fin_cant − damp·p`  (canted-fin forcing vs aero damping)

### Inertia

- Transverse (pitch = yaw): `I_yy = mass·((1/12)·L_ref² + (1/4)·radius²)` — solid cylinder
- Axial (roll): `I_x = 0.5·mass·radius²`

### Rotational equations of motion (Euler's equations, axisymmetric)

```
ṗ = M_x / I_x
q̇ = (M_y + (I_t - I_x)·r·p) / I_t
ṙ = (M_z + (I_x - I_t)·p·q) / I_t
```

### Attitude kinematics

```
q̇ = 0.5 · q ⊗ (0, p, q, r)     # quaternion derivative from body rates
q ← q / |q|                     # renormalize every step (kills RK4 drift)
```

### Integrator — RK4

Fourth-order Runge–Kutta over the full state vector (positions, velocities, quaternion, body rates, fuel). `dt = 0.002 s` — small enough to resolve the gyroscopic **nutation** of a fast-spinning body; at `0.01 s` a rolling rocket's nutation outruns the step and the solution diverges to NaN. Output prints every 5th step (~0.01 s cadence).

### Atmosphere

Air density follows the **International Standard Atmosphere**: the standard troposphere lapse rate up to 11 km and an isothermal lower stratosphere to 20 km — replacing the fixed sea-level value used in earlier stages.

### Additional Inputs (beyond the 1DOF set)

| Variable | Description | Unit |
|---|---|---|
| `theta_deg` | Launch angle from vertical (0 = straight up) | deg |
| `phi_deg` | Launch azimuth / compass heading | deg |
| `CP` | Center of pressure (fraction of L_ref) | – |
| `CG` | Center of gravity (fraction of L_ref) | – |
| `L_ref` | Reference length | m |
| `C_mq` | Aerodynamic damping coefficient | – |
| `Cn_alpha` | Normal-force slope | – |
| `radius` | Body radius (sets axial inertia) | m |
| `fin_cant` | Canted-fin roll angle (0 = no roll) | deg |

### Validation

The 6DOF model is checked against three independent closed-form benchmarks (each exercising a different part of the physics):

| Test | Physics | Formula | Error |
|---|---|---|---|
| Tsiolkovsky burnout | translation + mass loss | `v = Ve·ln(m₀/mf) − g·t_b` | 0.005% |
| Steady-state roll | roll forcing vs damping | `p_ss = 2·Cn_α·V·δ / (L_ref·C_mq)` | 0.007% |
| Pitch short-period | static stability + inertia | `ω_n = √(Cn_α·q_bar·A·L_ref·abs(CG-CP) / I_t)` | 0.020% |

### Key Physics Principles

- **Quaternions over Euler angles**: no gimbal lock, cheap to renormalize, and the single source of attitude truth.
- **Body vs world frames**: forces and aerodynamics are natural in the body frame; integration happens in the world frame — the quaternion bridges the two.
- **Static stability**: CP behind CG produces a restoring moment; the pitch/yaw sign asymmetry is a consequence of the axis definitions and is handled automatically by `M = d × F`.
- **Stiffness sets the timestep**: the fastest mode (spinning-body nutation), not the motion you actually want to watch, dictates `dt`.

---

## Physics Scope

This simulator covers flight dynamics only. The following are out of scope:

- **Propulsion analysis** — chamber pressure, nozzle design, isentropic expansion. Thrust is taken as a computed input from `mass_flow_rate * Ve`
- **Structural analysis** — the rigid body assumption means no flex, no slosh
- **Standard atmosphere** — air density `rho` is fixed at 1.225 kg/m³ (sea level) for the early DOF stages. In 6DOF this is replaced by the International Standard Atmosphere (density varying with altitude)

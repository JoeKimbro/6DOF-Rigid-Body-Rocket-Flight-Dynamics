# 6DOF Rigid Body Rocket Flight Dynamics Simulator

## Mission

Build a 6 Degree of Freedom (6DOF) rigid body rocket flight dynamics simulator from the ground up, learning the underlying physics incrementally at each stage.

## Goal

A simulator that takes real rocket parameters as inputs and simulates altitude, downrange distance, velocity, attitude, and flight phases from ignition through impact. I want to proceed with each DOF, up to the point of 3DOF there will be no user input, this is for ease of develop as there are many variables up to that point forward.  

---

## Interactive Web Dashboard

The 6DOF physics is also compiled to **WebAssembly** and driven from a browser
dashboard: enter rocket parameters, hit **Launch**, and watch the flight compute
and plot — a 3D trajectory with the rocket's actual quaternion attitude plus 2D
graphs of altitude, ground track, speed, angle of attack/sideslip, attitude, and
body rates. It runs entirely client-side (no server) and is hosted on GitHub
Pages. See [`web/README.md`](web/README.md) for the architecture and build steps.

```bash
./web/build.sh && cd web && python3 -m http.server 8000   # then open localhost:8000
```

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

Additional cross-checks run against the **compiled WebAssembly core** (the exact
build the dashboard uses), each compared to a closed-form or published reference:

| Check | Reference | Result |
|---|---|---|
| Energy conservation in unpowered coast (drag off) | `E = g·y + ½v²` constant | drift < 0.001% |
| Ballistic apogee (drag off) | `apogee = E/g` | exact (< 0.001%) |
| Free-fall acceleration (no thrust/drag/aero) | `a = −9.81 m/s²` | exact |
| Quaternion stays unit-norm over a full flight | `‖q‖ = 1` | error < 6e-7 |
| ISA air density vs **U.S. Standard Atmosphere 1976** | published table | < 0.25% to 11 km, < 1% to 20 km |
| Attitude faithfulness — does the rendered nose track the velocity? | stable rocket weathercocks (AoA ≈ 0) | max nose-vs-velocity angle 0.8° |

The last check is what makes the **3D visualization trustworthy**: the view applies
the same body→world quaternion the physics integrates, and a stable rocket holds its
nose along the flight path — exactly as a real one does. Behavioral checks also pass:
a statically **unstable** rocket (CP ahead of CG) tumbles to high angle of attack and
crashes early, while the stable configuration flies a clean arc.

### Key Physics Principles

- **Quaternions over Euler angles**: no gimbal lock, cheap to renormalize, and the single source of attitude truth.
- **Body vs world frames**: forces and aerodynamics are natural in the body frame; integration happens in the world frame — the quaternion bridges the two.
- **Static stability**: CP behind CG produces a restoring moment; the pitch/yaw sign asymmetry is a consequence of the axis definitions and is handled automatically by `M = d × F`.
- **Stiffness sets the timestep**: the fastest mode (spinning-body nutation), not the motion you actually want to watch, dictates `dt`.

---

## Assumptions & Limitations

This is a **flight-dynamics** simulator built to learn the physics, not a
production trajectory tool. It is **exact against the closed-form benchmarks above**,
but it models the vehicle and environment with deliberate simplifications. Knowing
these is essential to interpreting results correctly.

### Modeling assumptions

| Area | Assumption | Consequence |
|---|---|---|
| **Structure** | Perfectly rigid body — no flex, bending, or fuel slosh | Standard 6DOF assumption; fine for rigid airframes |
| **Drag** | A single **constant `Cd`** | No transonic/supersonic drag rise — the largest error source at high Mach |
| **Aerodynamics** | **Linear** normal force `N = Cn_α·q̄·A·α`, constant `Cn_α` | Accurate at small angle of attack; **overshoots past ~15–20°** and is unphysical near a full flip (real curve behaves like `sin α`) |
| **Propulsion** | Thrust = `mass_flow_rate · Ve`, constant while fuel remains, instant cutoff | No thrust curve, no ramp/tail-off, no pressure-thrust term, no throttle or TVC |
| **Mass properties** | Solid-cylinder inertia; **`CG` and `CP` are fixed fractions of `L_ref`** | Static margin does **not** migrate as fuel burns, unlike a real vehicle |
| **Gravity** | Constant `g = 9.81 m/s²`, flat non-rotating Earth | No altitude variation of `g`, no Earth curvature, no Coriolis — only valid for sub-orbital, local-range flights |
| **Atmosphere** | ISA density to 20 km, clamped above; **no wind** | No gusts, shear, or weathercocking-into-wind; density held constant above 20 km |
| **Flight** | Single stage, no recovery, no active control or guidance | No staging, parachutes, or closed-loop steering |

### Accuracy & numerical envelope

- **Roll is numerically stiff.** Axial inertia `I_x = ½·m·r²` is tiny, so a fin cant
  above **~0.5°** spins the body fast enough that its gyroscopic nutation outruns the
  explicit RK4 step and the solution diverges. The core detects this and stops cleanly
  (the dashboard flags it as a divergence) rather than emitting NaN.
- **High-angle-of-attack / tumbling flight is qualitatively right but quantitatively
  off** — an unstable rocket correctly tumbles and crashes early, but the linear aero
  model exaggerates forces once past ~90° AoA.
- **Do not expect digit-matching against full tools** (OpenRocket, RASAero II). Because
  drag here is a constant `Cd` rather than `Cd(Mach)`, apogee for a fast (transonic+)
  rocket will differ. Even OpenRocket and RASAero routinely disagree by ~10% on apogee,
  and both miss real flights by several percent. The right standard for this project is
  **exact against closed-form physics** and **correct in trend and behavior** against
  the full tools — both of which it meets.
- **Cross-checking yourself:** model the same rocket in
  [OpenRocket](https://openrocket.info/) (free, open source), and for the cleanest
  apples-to-apples comparison force its drag to a constant `Cd` matching yours and
  launch straight up — that removes the `Cd(Mach)` difference. Center-of-pressure /
  static-stability behavior can be checked against the **Barrowman equations**.

### Explicitly out of scope

- **Propulsion internals** — chamber pressure, nozzle/isentropic expansion (thrust is an input via `mass_flow_rate · Ve`)
- **Structural analysis** — loads, flex, slosh (rigid-body assumption)
- **Aeroheating, mass ablation, plume effects**
- **6DOF wind/turbulence, Earth rotation, and gravity variation**

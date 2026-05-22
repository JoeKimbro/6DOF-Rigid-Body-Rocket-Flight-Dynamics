# 6DOF Rigid Body Rocket Flight Dynamics Simulator

## Mission

Build a 6 Degree of Freedom (6DOF) rigid body rocket flight dynamics simulator from the ground up, learning the underlying physics incrementally at each stage.

## Goal

A simulator that takes real rocket parameters as inputs and accurately predicts altitude, downrange distance, velocity, attitude, and flight phases from ignition through impact. I want to proceed with each DOF, up to the point of 3DOF there will be no user input, this is for ease of develop as there are many variables up to that point forward.  

---

## What is a Rigid Body?

The rocket is treated as a perfectly rigid object — every part moves together, no structural deformation, no fuel slosh. This is the standard assumption in all practical 6DOF flight simulators and makes the equations of motion tractable.

---

## Degrees of Freedom Roadmap

| DOF | What it adds | Input Method |
|---|---|---|
| 1DOF | Vertical translation — altitude | User prompts |
| 2DOF | Horizontal translation — downrange distance | User prompts |
| 3DOF | Pitch rotation — rocket tilting | Config file |
| 4DOF | Yaw — side to side pointing | Config file |
| 5DOF | Roll — spinning along centerline | Config file |
| 6DOF | Full free flight — all translations + all rotations | Config file |

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

## Physics Scope

This simulator covers flight dynamics only. The following are out of scope:

- **Propulsion analysis** — chamber pressure, nozzle design, isentropic expansion. Thrust is taken as a computed input from `mass_flow_rate * Ve`
- **Structural analysis** — the rigid body assumption means no flex, no slosh
- **Standard atmosphere** — air density `rho` is fixed at 1.225 kg/m³ (sea level) for 1DOF. Variable density by altitude will be added in later DOF stages

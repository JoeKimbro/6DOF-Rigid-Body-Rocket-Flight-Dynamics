# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

```bash
# Compile (currently wired to 1DOF)
g++ -std=c++17 main.cpp 1DOF/1DOF.cpp 1DOF/Integrator1DOF.cpp -I. -o sim

# Run
./sim
```

When advancing to 2DOF, the compile command must include `2DOF/2DOF.cpp 2DOF/Integrator2DOF.cpp` and `main.cpp` must be updated to instantiate `DOF2` instead of `DOF1`.

## Architecture

The project is structured as one self-contained module per DOF stage. Each stage builds on the shared state in `StateVariables/` but is otherwise independent.

**State is centralized in `RigidBody`** (defined in `StateVariables/RigidBody.h`). It composes:
- `LinearState vertical` / `LinearState horizontal` — position, velocity, acceleration, netForce per axis
- `MassProperties props` — dryMass, fuelMass, mass, mass_flow_rate
- `PropulsionProps propul` — thrust, Ve, Cd, A
- `double theta` — pitch angle from vertical (radians), 0 = pointing straight up

**Each DOF stage has two classes:**
- `DOFn` — owns the simulation loop, user input, and console output (pattern from `DOF1::run`)
- `DOFnIntegrator` — owns the physics: force summation, integration, state update (`Integrator::step` / `DOF2Integrator::stepDOF2`)

**Integration method:** Forward Euler throughout. `dt = 0.01` s, hardcoded in the integrator class.

**Sign convention:** gravity = −9.81 m/s². Positive vertical is up. Drag opposes velocity (negative coefficient multiplied into velocity components). Thrust decomposes as `thrust * cos(theta)` vertical, `thrust * sin(theta)` horizontal.

## DOF Roadmap

| DOF | Branch | Input method | Status |
|-----|--------|-------------|--------|
| 1DOF | main | User prompts (stdin) | Complete |
| 2DOF | 2DOF | User prompts (stdin) | Physics written, sim loop pending |
| 3DOF–6DOF | future | Config file | Not started |

## Role of This Repository

This is a learning project. The user is building the simulator incrementally while learning flight dynamics from scratch. Claude's role is **teacher only** — explain physics, identify bugs conceptually, ask Socratic questions. Do not write code for the user. See the system prompt for full constraints.

---
name: project-3dof-state
description: Physics implementation state of the 3DOF module as of 2026-06-05, including what is working, what has bugs, and what is genuinely missing
metadata:
  type: project
---

As of 2026-06-05 (branch: 3DOFwindows, commit: "feat: 3DOF complete"):

**What the 3DOF actually models:**
- 2 translational DOF: vertical (y) and horizontal (x) position/velocity/acceleration
- 1 rotational DOF: pitch angle theta, integrated from moment balance (theta_ddot → omega → theta)
- This is genuine planar 3DOF — theta is a dynamic state, not a prescribed input

**What is implemented in Integrator3DOF.cpp:**
- Full translational physics: gravity, drag (v_total from END of previous step), thrust via cos/sin(theta)
- AoA: `body.rotation.AoA = body.theta - atan2(body.horizontal.velocity, body.vertical.velocity)`
- Dynamic pressure: `q_bar = 0.5 * rho * v_total^2`
- Normal force: `N = Cn_alpha * q_bar * A * AoA`, applied as +N*cos(theta) horizontal, -N*sin(theta) vertical
- Pitching moment: `M = Cn_alpha * q_bar * A * L_ref * AoA * (CP - CG)` — arithmetic, correctly signed
- Damping moment: `M += 0.25 * rho * v_total * A * L_ref^2 * C_mq * omega`
- I_yy: `(1/12) * mass * L_ref^2` (thin rod, variable-mass per step)
- Angular integration: `omega += theta_ddot * dt`, `theta += omega * dt`
- rho is constant (1.225 kg/m^3) — no ISA atmosphere model

**Known issues NOT yet surfaced to user:**
1. The moment arm bug (CP < CG boolean) was FIXED — now uses arithmetic (CP - CG). Correctly signed.
2. v_total lag: drag and q_bar use v_total from end of previous step (step 1 → v_total=0, so drag=0, N=0, M=0 on first tick). Pedagogically worth noting.
3. The damping moment coefficient factor 0.25 is non-standard — typically C_mq is applied with q_bar * L_ref^2 / (2*v), user has not been challenged on this derivation.
4. Normal force application direction: +N*cos(theta) horizontal, -N*sin(theta) vertical. User has not been asked to derive whether these signs are correct for a body-frame normal force projected to inertial axes.
5. theta is initialized to the user's launch angle — meaning the rotational integrator starts at a non-zero pitch and the moment balance immediately acts. User has not been asked whether this initial condition is physically consistent with zero omega at t=0.
6. Constant rho (no ISA model) is a known simplification not yet flagged.

**Conceptual state of the 3DOF:**
- The user has genuine 3DOF: x-translation, y-translation, pitch rotation
- theta is a dynamic state integrated from physics
- The pitching moment chain is: AoA → N → M → theta_ddot → omega → theta
- This is physically correct in structure; some coefficient and sign questions remain unresolved

**Why:** Tracking this prevents re-explaining already-implemented physics and helps focus teaching on real remaining gaps.
**How to apply:** When user asks about readiness for 4DOF, surface issues 3–5 above. The most pedagogically important question for the 4DOF transition is: what does a 4th DOF even mean for a rocket in 3D space?

See also: [[user-profile]]

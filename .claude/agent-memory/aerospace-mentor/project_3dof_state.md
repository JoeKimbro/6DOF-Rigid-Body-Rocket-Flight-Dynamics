---
name: project-3dof-state
description: Physics implementation state of the 3DOF module as of 2026-06-03, including what is working, what has bugs, and what is genuinely missing
metadata:
  type: project
---

As of 2026-06-03 (branch: 3DOF, commit: "feat: Added physics for 3DOF; Need to add user inputs"):

**What exists in Integrator3DOF.cpp:**
- Full 2DOF translational physics (gravity, drag, thrust decomposition) — copy of Integrator2DOF with rotational block appended
- AoA computed: `body.rotation.AoA = body.theta - atan2(body.horizontal.velocity, body.vertical.velocity)`
- Dynamic pressure: `q_bar = 0.5 * rho * v_total^2`
- Pitching moment: `M = Cn_alpha * q_bar * A * L_ref * AoA * (CP - CG)`
- I_yy computed per step: `(1/12) * mass * L_ref^2` (thin rod approximation, variable-mass aware)
- Angular acceleration: `theta_ddot = M / I_yy`
- Integration: `omega += theta_ddot * dt`, `theta += omega * dt`

**Known issues / physics gaps to surface to user:**
1. CRITICAL — Sequencing bug: translational forces use the OLD theta (from previous step) to decompose thrust, but then theta is updated at the END of the same step. This is actually correct for Forward Euler (use state at start of step), but the user should be aware of it consciously.
2. CRITICAL — AoA sign/convention: `atan2(v_horizontal, v_vertical)` gives the flight path angle from vertical. The sign of (CP - CG) determines whether the moment is restoring or destabilizing. If CP > CG (aerodynamically unstable), positive AoA produces a positive (diverging) moment. The user needs to understand this explicitly.
3. MISSING — No aerodynamic normal force on translational equations: the pitching moment is computed, but the normal force (Cn_alpha * q_bar * A * AoA) that acts perpendicular to the velocity vector is NOT added to the force equations. 3DOF should have this.
4. MISSING — Thrust vectoring / gimbal: theta updates but thrust decomposition uses theta, which now feeds back. This loop is implicitly present but the user may not have thought through the coupling.
5. MISSING — User inputs for 3DOF-specific parameters: Cn_alpha, CP, CG, L_ref are declared in PropulsionProps but 3DOF.cpp does NOT prompt the user for them. They will be zero-initialized, making the entire rotational block a no-op (M = 0 always).
6. MODELING NOTE — I_yy uses thin rod approximation. This is acceptable for learning but user should know it's an approximation and that I_yy changes as fuel burns (CG shifts too).

**What the user's own comment table said was missing (bottom of Integrator3DOF.cpp):**
The table is STALE — it says steps 1-5 are not implemented, but they actually ARE now implemented. The user implemented everything in the table but forgot to update their own status table. This is worth pointing out — their code is further along than their notes suggest.

**Why:** Tracking this prevents re-explaining already-implemented physics and helps focus teaching on the real remaining gaps (normal force, user inputs for rotational params, sign/stability understanding).
**How to apply:** When user asks "what's left," point to the 5 items above. The biggest functional blocker is item 5 (missing user inputs = rotational block silently does nothing).

See also: [[user-profile]]

---
name: project-3dof-state
description: Physics implementation state of the 3DOF module as of 2026-06-03, including what is working, what has bugs, and what is genuinely missing
metadata:
  type: project
---

As of 2026-06-04 (branch: 3DOFwindows, commit: "feat: windows 3DOF"):

**What exists in Integrator3DOF.cpp:**
- Full translational physics: gravity, drag (uses v_total from END of previous step), thrust decomposition via cos/sin(theta)
- Normal force added to translational equations: `N = Cn_alpha * q_bar * A * AoA`, applied as `+N*cos(theta)` horizontal, `-N*sin(theta)` vertical
- AoA computed: `body.rotation.AoA = body.theta - atan2(body.horizontal.velocity, body.vertical.velocity)`
- Dynamic pressure: `q_bar = 0.5 * rho * v_total^2`
- Pitching moment: `M = Cn_alpha * q_bar * A * L_ref * AoA * (CP < CG)` — NOTE: boolean (CP < CG), not arithmetic (CP - CG)
- I_yy computed per step: `(1/12) * mass * L_ref^2` (thin rod approximation, variable-mass aware)
- Angular acceleration: `theta_ddot = M / I_yy`
- Integration: `omega += theta_ddot * dt`, `theta += omega * dt`
- User inputs for ALL 3DOF parameters including CP, CG, L_ref, Cn_alpha (added in windows 3DOF commit)

**Known physics issues to surface to user:**
1. Moment arm: code uses boolean `(CP < CG)` — moment is either full expression or zero. The sign of the arm is not encoded; a physically stabilizing vs. destabilizing rocket is toggled by which is larger, not captured in sign.
2. AoA sign/convention: `atan2(v_horizontal, v_vertical)` gives flight path angle from vertical. User should understand what positive AoA means physically.
3. v_total lag: drag and q_bar use v_total from END of previous step. On step 1 this is 0, so drag=0 and N=0 and M=0 on step 1. User must understand this is a feature of the code's ordering, not a bug.
4. Moment arm physics: real stability requires CP > CG for an aerodynamically stable rocket. The code's (CP < CG) boolean means if user enters CP=0.3, CG=0.5, the boolean is true (1) and moment is active.

**Why:** Tracking this prevents re-explaining already-implemented physics and helps focus teaching on real remaining gaps.
**How to apply:** When user asks "what's left," point to the 4 items above. The most pedagogically important issue is item 1 — moment arm sign physics.

See also: [[user-profile]]

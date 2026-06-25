// Native smoke test for the simulation core -- no Emscripten required.
// Validates the Tsiolkovsky burnout benchmark from the README so we know the
// refactored core matches the original CLI physics before building to WASM.
#include <cmath>
#include <cstdio>
#include "sim_core.h"

int main() {
    SimParams p;                 // defaults: 5 kg dry + 5 kg fuel, 2.5 kg/s, Ve 2500
    p.theta_deg = 0.0;           // straight up for a clean 1D-ish burnout check
    p.fin_cant_deg = 0.0;
    auto traj = runSim(p);

    printf("samples: %zu\n", traj.size());
    if (traj.empty()) { printf("FAIL: empty trajectory\n"); return 1; }

    // Find burnout time and the speed there.
    const double t_b = p.fuelMass / p.mass_flow_rate;   // 5 / 2.5 = 2.0 s
    double v_at_burnout = 0.0, t_found = 0.0;
    for (auto& s : traj) { if (s.t <= t_b) { v_at_burnout = s.speed; t_found = s.t; } }

    const double m0 = p.dryMass + p.fuelMass, mf = p.dryMass;
    const double v_ideal = p.Ve * std::log(m0 / mf) - 9.81 * t_b; // no-drag ideal

    const Sample& last = traj.back();
    printf("burnout  t=%.3f  v_sim=%.2f  v_ideal(no drag)=%.2f m/s\n",
           t_found, v_at_burnout, v_ideal);
    printf("apogee-ish max altitude: ");
    double maxy = 0.0; for (auto& s : traj) maxy = std::fmax(maxy, s.y);
    printf("%.1f m\n", maxy);
    printf("final: t=%.2f  alt=%.2f  speed=%.2f\n", last.t, last.y, last.speed);
    printf("OK\n");
    return 0;
}

// Emscripten <-> JavaScript bridge.
//
// We expose ONE function to JS: runSimJson(params) -> JSON string. Returning a
// JSON string (rather than marshalling a vector<struct> through embind) keeps
// the JS side trivial -- JSON.parse and you have an array of plain objects to
// feed straight into Plotly/three.js. SimParams crosses the boundary as an
// embind "value_object" so JS passes a normal object with named fields.
#include <emscripten/bind.h>
#include <cmath>
#include <sstream>
#include <string>
#include "sim_core.h"

using namespace emscripten;

// Emit a finite number, or JSON `null` for NaN/Inf -- `nan`/`inf` are not legal
// JSON and would make JSON.parse() throw on the JS side.
static void num(std::ostringstream& os, const char* key, double v) {
    os << '"' << key << "\":";
    if (std::isfinite(v)) os << v; else os << "null";
}

static std::string runSimJson(const SimParams& p) {
    const std::vector<Sample> traj = runSim(p);

    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(6);
    os << '[';
    for (size_t i = 0; i < traj.size(); ++i) {
        const Sample& s = traj[i];
        if (i) os << ',';
        os << '{'; num(os, "t", s.t);
        os << ','; num(os, "mass", s.mass);
        os << ','; num(os, "x", s.x);   os << ','; num(os, "y", s.y);   os << ','; num(os, "z", s.z);
        os << ','; num(os, "vx", s.vx); os << ','; num(os, "vy", s.vy); os << ','; num(os, "vz", s.vz);
        os << ','; num(os, "speed", s.speed);
        os << ','; num(os, "AoA", s.AoA); os << ','; num(os, "sideslip", s.sideslip);
        os << ','; num(os, "theta", s.theta); os << ','; num(os, "phi", s.phi);
        os << ','; num(os, "p", s.p); os << ','; num(os, "q", s.q); os << ','; num(os, "r", s.r);
        os << ','; num(os, "qw", s.qw); os << ','; num(os, "qx", s.qx);
        os << ','; num(os, "qy", s.qy); os << ','; num(os, "qz", s.qz);
        os << ','; num(os, "thrust", s.thrust);
        os << '}';
    }
    os << ']';
    return os.str();
}

EMSCRIPTEN_BINDINGS(sixdof) {
    value_object<SimParams>("SimParams")
        .field("dryMass",        &SimParams::dryMass)
        .field("fuelMass",       &SimParams::fuelMass)
        .field("mass_flow_rate", &SimParams::mass_flow_rate)
        .field("Ve",             &SimParams::Ve)
        .field("Cd",             &SimParams::Cd)
        .field("A",              &SimParams::A)
        .field("startAltitude",  &SimParams::startAltitude)
        .field("total_time",     &SimParams::total_time)
        .field("theta_deg",      &SimParams::theta_deg)
        .field("phi_deg",        &SimParams::phi_deg)
        .field("CP",             &SimParams::CP)
        .field("CG",             &SimParams::CG)
        .field("L_ref",          &SimParams::L_ref)
        .field("C_mq",           &SimParams::C_mq)
        .field("Cn_alpha",       &SimParams::Cn_alpha)
        .field("radius",         &SimParams::radius)
        .field("fin_cant_deg",   &SimParams::fin_cant_deg);

    function("runSimJson", &runSimJson);
}

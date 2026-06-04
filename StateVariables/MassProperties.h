#pragma once

  struct MassProperties {
      double mass          = 1.0;
      double dryMass       = 0.0;
      double fuelMass      = 0.0;
      double mass_flow_rate = 0.0;
      double I_yy = 0.0; // moment of inertia; turns pitching moment into angular acceleration via α_pitch = M / I_yy
      double N = 0.0; // normal force magnitude

  }; 
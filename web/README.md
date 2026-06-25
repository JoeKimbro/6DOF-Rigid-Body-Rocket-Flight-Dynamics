# Web Dashboard (WebAssembly)

An interactive, browser-based front-end for the 6DOF simulator. You enter rocket
parameters, hit **Launch**, and the flight is computed and plotted entirely in
your browser — no server. It's hostable as static files on GitHub Pages.

## How it works

The project's own C++ physics is compiled to **WebAssembly** with Emscripten and
runs client-side, so there is no second copy of the math to keep in sync.

```
6DOF/6DOF_RK4.cpp      the real RK4 physics (unchanged)
        │
web/sim_core.{h,cpp}   runSim(SimParams) -> vector<Sample>   (NO std::cin/cout)
        │
web/bindings.cpp       embind: runSimJson(params) -> JSON string
        │  emcc
web/sim.js + sim.wasm   loaded by the page
        │
web/index.html/app.js   form -> runSimJson -> Plotly (2D) + three.js (3D)
```

The key refactor is `sim_core`: the original `6DOF/6DOF.cpp` welds the physics to
`std::cin`/`std::cout`, which can't be reused from a browser. `sim_core` takes a
plain `SimParams` struct and returns the whole trajectory — the same core also
compiles natively (see `test_native.cpp`).

## Build locally

Requires the [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html):

```bash
git clone https://github.com/emscripten-core/emsdk.git ~/emsdk
cd ~/emsdk && ./emsdk install latest && ./emsdk activate latest
```

Then from the repo root:

```bash
./web/build.sh          # produces web/sim.js + web/sim.wasm
cd web && python3 -m http.server 8000
# open http://localhost:8000
```

(You must serve over http — opening `index.html` as a `file://` URL won't let the
browser fetch the `.wasm`.)

## Native sanity check (no Emscripten)

```bash
g++ -std=c++17 -I. -Iweb web/test_native.cpp web/sim_core.cpp 6DOF/6DOF_RK4.cpp -o /tmp/t && /tmp/t
```

## Deploy

`.github/workflows/deploy.yml` builds the WASM in CI and publishes `web/` to
GitHub Pages on every push to `main`. Enable it once under
**Settings → Pages → Build and deployment → Source: GitHub Actions**.

## Note on roll

The roll channel is numerically stiff (small axial inertia `I_x = ½·m·r²`), so a
fin cant above ~0.5° can outrun the explicit RK4 step and diverge — the same
timestep limit documented in the main README. The core detects divergence and
stops cleanly, so the dashboard always gets a finite trajectory; it never crashes
or emits `NaN`. The presets stay in the validated translation + pitch/yaw regime.

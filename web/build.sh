#!/usr/bin/env bash
# Build the 6DOF physics core to WebAssembly.
#
# Output: web/sim.js + web/sim.wasm  (the .js is a loader that fetches the
# .wasm and exposes the embind API on the global `SixDOF` module factory).
#
# Requires the Emscripten SDK on PATH. If emcc isn't found we try to source a
# local emsdk install (the path the README's setup uses).
set -euo pipefail
cd "$(dirname "$0")/.."   # repo root, so -I. resolves StateVariables/ and 6DOF/

if ! command -v emcc >/dev/null 2>&1; then
    if [ -f "$HOME/emsdk/emsdk_env.sh" ]; then
        # shellcheck disable=SC1091
        source "$HOME/emsdk/emsdk_env.sh"
    else
        echo "error: emcc not found and ~/emsdk/emsdk_env.sh missing." >&2
        echo "Install: git clone https://github.com/emscripten-core/emsdk && cd emsdk && ./emsdk install latest && ./emsdk activate latest" >&2
        exit 1
    fi
fi

emcc \
    -std=c++17 -O3 -I. \
    web/sim_core.cpp web/bindings.cpp 6DOF/6DOF_RK4.cpp \
    -o web/sim.js \
    -s MODULARIZE=1 \
    -s EXPORT_NAME=SixDOF \
    -s EXPORT_ES6=0 \
    -s ENVIRONMENT=web \
    -s ALLOW_MEMORY_GROWTH=1 \
    -s SINGLE_FILE=0 \
    --bind

echo "Built web/sim.js + web/sim.wasm"

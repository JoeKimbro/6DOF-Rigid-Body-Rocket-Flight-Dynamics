/* 6DOF dashboard front-end.
 *
 * Flow:  load WASM module -> read params from the form -> Module.runSimJson()
 * -> parse the JSON trajectory -> draw 2D plots (Plotly) + 3D flight (three.js).
 * Everything runs client-side, so this whole page is static and Pages-hostable.
 */
"use strict";

const FIELDS = [
  "dryMass","fuelMass","mass_flow_rate","Ve","Cd","A","startAltitude",
  "total_time","theta_deg","phi_deg","CP","CG","L_ref","C_mq","Cn_alpha",
  "radius","fin_cant_deg",
];

// All presets keep fin_cant = 0: the roll channel is stiff (tiny axial inertia)
// and an aggressive cant outruns the RK4 step -- see the README timestep note.
// The sim still stops cleanly if you dial cant up, but the defaults stay in the
// validated translation + pitch/yaw regime.
const PRESETS = {
  sounding: { dryMass:5, fuelMass:5, mass_flow_rate:2.5, Ve:2500, Cd:0.5, A:0.0079,
    startAltitude:0, total_time:100, theta_deg:5, phi_deg:90, CP:0.6, CG:0.5,
    L_ref:2, C_mq:5, Cn_alpha:10, radius:0.05, fin_cant_deg:0 },
  lofted: { dryMass:8, fuelMass:12, mass_flow_rate:4, Ve:3050, Cd:0.45, A:0.0079,
    startAltitude:0, total_time:160, theta_deg:35, phi_deg:90, CP:0.62, CG:0.5,
    L_ref:2.5, C_mq:6, Cn_alpha:12, radius:0.05, fin_cant_deg:0 },
  maxrange: { dryMass:8, fuelMass:12, mass_flow_rate:5, Ve:3050, Cd:0.4, A:0.0079,
    startAltitude:0, total_time:200, theta_deg:45, phi_deg:90, CP:0.62, CG:0.5,
    L_ref:2.5, C_mq:6, Cn_alpha:12, radius:0.05, fin_cant_deg:0 },
};

let Module = null;     // the loaded WASM module
let traj = [];         // current trajectory (array of sample objects)
let three = null;      // 3D scene handles

// ----------------------------------------------------------------------------
// Boot: load the form defaults, then the WASM module.
// ----------------------------------------------------------------------------
function applyPreset(name) {
  const p = PRESETS[name] || PRESETS.sounding;
  for (const f of FIELDS) document.getElementById(f).value = p[f];
}

document.getElementById("preset").addEventListener("change", e => applyPreset(e.target.value));
applyPreset("sounding");

const statusEl = document.getElementById("status");
const launchBtn = document.getElementById("launch");

SixDOF().then(mod => {
  Module = mod;
  launchBtn.disabled = false;
  launchBtn.textContent = "▲ Launch";
  statusEl.textContent = "Physics core loaded.";
  initThree();
  runFlight();   // show a flight immediately
});

launchBtn.addEventListener("click", runFlight);

// ----------------------------------------------------------------------------
// Run one flight.
// ----------------------------------------------------------------------------
function readParams() {
  const p = {};
  for (const f of FIELDS) p[f] = parseFloat(document.getElementById(f).value) || 0;
  return p;
}

// Check the inputs before running. Returns { errors, warnings }:
//  - errors   : block the run entirely (physically impossible / nonsensical).
//  - warnings : let the run proceed but flag a risky / surprising configuration.
function validateParams(p) {
  const errors = [], warnings = [];
  const g = 9.81;

  // Every field must be a finite number (readParams already coerces, but a raw
  // NaN from a blank required box should still be caught explicitly).
  for (const f of FIELDS) {
    if (!Number.isFinite(p[f])) errors.push(`"${f}" is not a valid number.`);
  }

  // Quantities that must be strictly positive for the physics to make sense.
  if (p.dryMass <= 0)     errors.push("Dry mass must be greater than 0.");
  if (p.fuelMass < 0)     errors.push("Fuel mass can't be negative.");
  if (p.mass_flow_rate < 0) errors.push("Burn rate can't be negative.");
  if (p.Ve < 0)           errors.push("Exhaust velocity can't be negative.");
  if (p.A <= 0)           errors.push("Reference area A must be greater than 0.");
  if (p.radius <= 0)      errors.push("Radius must be greater than 0 (it sets the roll inertia).");
  if (p.L_ref <= 0)       errors.push("Reference length L_ref must be greater than 0.");
  if (p.total_time <= 0)  errors.push("Sim duration must be greater than 0.");

  // Thrust-to-weight: a rocket sitting on the pad (startAltitude ~ 0) can only
  // lift off if thrust exceeds weight. There's no launch rail in the model, so
  // TWR < 1 just sinks the rocket below y=0 on the first step -> a non-flight.
  const m0 = p.dryMass + p.fuelMass;
  const thrust = p.mass_flow_rate * p.Ve;
  if (m0 > 0 && p.startAltitude <= 0) {
    const twr = thrust / (m0 * g);
    if (thrust <= m0 * g) {
      errors.push(
        `Thrust-to-weight ratio is ${twr.toFixed(2)} (must exceed 1 to lift off). ` +
        `Thrust ${(thrust/1e3).toFixed(0)} kN can't raise ${(m0*g/1e3).toFixed(0)} kN of weight — ` +
        `raise burn rate or Ve, or reduce mass.`);
    } else if (twr < 1.2) {
      warnings.push(`Thrust-to-weight is only ${twr.toFixed(2)} — the rocket will climb slowly off the pad.`);
    }
  }

  // Static stability: CP must sit behind CG (larger fraction) or the rocket tumbles.
  if (p.CP < p.CG) {
    warnings.push(
      `Center of pressure (CP=${p.CP}) is ahead of center of gravity (CG=${p.CG}): ` +
      `the rocket is statically unstable and will tumble.`);
  }

  // Roll stiffness: a fin cant beyond ~0.5 deg can outrun the RK4 step.
  if (Math.abs(p.fin_cant_deg) > 0.5) {
    warnings.push(
      `Fin cant ${p.fin_cant_deg}° is above ~0.5° — the roll mode may diverge ` +
      `(the sim stops cleanly if it does).`);
  }
  return { errors, warnings };
}

function runFlight() {
  if (!Module) return;
  const params = readParams();
  const outcome = document.getElementById("outcome");

  // Gate the run on input validity. Hard errors block it outright.
  const { errors, warnings } = validateParams(params);
  if (errors.length) {
    outcome.hidden = false;
    outcome.className = "outcome bad";
    outcome.textContent = "Can't run — fix these inputs:\n" + errors.map(e => "• " + e).join("\n");
    statusEl.textContent = "Invalid inputs.";
    return;
  }

  const t0 = performance.now();
  let json;
  try {
    json = Module.runSimJson(params);
  } catch (err) {
    statusEl.textContent = "Sim error: " + err;
    return;
  }
  traj = JSON.parse(json);
  const ms = (performance.now() - t0).toFixed(0);
  statusEl.textContent = `Computed ${traj.length} samples in ${ms} ms.`;

  renderSummary(warnings);
  renderPlots();
  rebuildTrajectory();
}

// ----------------------------------------------------------------------------
// Summary stat cards.
// ----------------------------------------------------------------------------
function renderSummary(warnings = []) {
  const last = traj[traj.length - 1];

  // Apogee + the time it occurs (not just the max value).
  let apIdx = 0;
  for (let i = 1; i < traj.length; i++) if (traj[i].y > traj[apIdx].y) apIdx = i;
  const apogee = traj[apIdx].y, apogeeT = traj[apIdx].t;

  const vmax = Math.max(...traj.map(s => s.speed));
  const range = Math.hypot(last.x, last.z);
  const maxRoll = Math.max(...traj.map(s => Math.abs(s.p)));

  // Classify how the flight ENDED so every label is honest about what it shows.
  // - landed:   last sample is at the ground (y ~ 0) -> "Flight time"/"Impact speed" are real.
  // - truncated: ran out the sim window while still airborne -> these are cutoff values.
  // - diverged:  stopped early without landing or hitting the window (numerical blow-up).
  const totalTime = parseFloat(document.getElementById("total_time").value) || 0;
  const landed    = last.y <= 1.0;
  const truncated = !landed && last.t >= totalTime - 0.05;
  const diverged  = !landed && !truncated;

  const timeLabel  = landed ? "Flight time"  : "Time simulated";
  const speedLabel = landed ? "Impact speed" : "Speed at cutoff";
  const rangeLabel = landed ? "Downrange"    : "Downrange so far";

  const cards = [
    ["Apogee", apogee.toFixed(0), "m"],
    ["Time to apogee", apogeeT.toFixed(1), "s"],
    ["Max speed", vmax.toFixed(0), "m/s"],
    [timeLabel, last.t.toFixed(1), "s"],
    [rangeLabel, range.toFixed(0), "m"],
    [speedLabel, last.speed.toFixed(0), "m/s"],
    ["Peak roll rate", maxRoll.toFixed(1), "rad/s"],
  ];
  document.getElementById("summary").innerHTML = cards.map(
    ([k, v, u]) => `<div class="stat"><div class="k">${k}</div><div class="v">${v} <span class="u">${u}</span></div></div>`
  ).join("");

  // Honest banner about the outcome of this run.
  const outcome = document.getElementById("outcome");
  outcome.hidden = false;
  let cls, msg;
  if (landed) {
    cls = "ok";
    msg = `Rocket landed at t = ${last.t.toFixed(1)} s, ${range.toFixed(0)} m downrange, hitting at ${last.speed.toFixed(0)} m/s.`;
  } else if (truncated) {
    cls = "warn";
    msg = `⚠ Flight truncated — the rocket is still ${last.y.toFixed(0)} m up when the ${totalTime.toFixed(0)} s window ends. ` +
      `"Flight time" and "impact speed" are not final yet; raise Sim duration to fly it down to impact.`;
  } else { // diverged
    cls = "bad";
    msg = `⚠ Simulation stopped early at t = ${last.t.toFixed(1)} s due to numerical divergence ` +
      `(usually too aggressive a fin cant). The trajectory beyond this point is not physical.`;
  }

  // Prepend any input warnings; a warning never downgrades a "bad" outcome.
  if (warnings.length) {
    msg = warnings.map(w => "⚠ " + w).join("\n") + "\n" + msg;
    if (cls === "ok") cls = "warn";
  }
  outcome.className = "outcome " + cls;
  outcome.textContent = msg;
}

// ----------------------------------------------------------------------------
// 2D Plotly plots.
// ----------------------------------------------------------------------------
const PLOT_BG = "#161b22", GRID = "#2a3340", INK = "#e6edf3", MUTED = "#8b949e";
function layout(title, xt, yt) {
  return {
    title: { text: title, font: { size: 13, color: INK } },
    paper_bgcolor: PLOT_BG, plot_bgcolor: PLOT_BG,
    font: { color: MUTED, size: 11 },
    margin: { l: 52, r: 14, t: 30, b: 40 },
    xaxis: { title: xt, gridcolor: GRID, zerolinecolor: GRID },
    yaxis: { title: yt, gridcolor: GRID, zerolinecolor: GRID },
    showlegend: false,
  };
}
const CFG = { displayModeBar: false, responsive: true };
const line = (x, y, color) => ({ x, y, mode: "lines", line: { color, width: 2 } });

function renderPlots() {
  const t = traj.map(s => s.t);
  Plotly.react("plot_alt", [line(t, traj.map(s => s.y), "#ff7b3d")],
    layout("Altitude vs time", "t (s)", "altitude (m)"), CFG);
  Plotly.react("plot_ground",
    [{ x: traj.map(s => s.x), y: traj.map(s => s.z), mode: "lines",
       line: { color: "#4aa3ff", width: 2 } }],
    layout("Ground track (top-down)", "East x (m)", "depth z (m)"), CFG);
  Plotly.react("plot_speed", [line(t, traj.map(s => s.speed), "#3ddc97")],
    layout("Speed vs time", "t (s)", "speed (m/s)"), CFG);
  Plotly.react("plot_aero", [
      { ...line(t, traj.map(s => s.AoA), "#ff7b3d"), name: "AoA", showlegend: true },
      { ...line(t, traj.map(s => s.sideslip), "#4aa3ff"), name: "sideslip", showlegend: true },
    ], { ...layout("Angle of attack & sideslip", "t (s)", "deg"), showlegend: true,
         legend: { font: { color: MUTED } } }, CFG);
  Plotly.react("plot_attitude", [
      { ...line(t, traj.map(s => s.theta), "#ff7b3d"), name: "θ (from vert)", showlegend: true },
      { ...line(t, traj.map(s => s.phi), "#4aa3ff"), name: "φ (azimuth)", showlegend: true },
    ], { ...layout("Attitude angles", "t (s)", "deg"), showlegend: true,
         legend: { font: { color: MUTED } } }, CFG);
  Plotly.react("plot_rates", [
      { ...line(t, traj.map(s => s.p), "#ff7b3d"), name: "p roll", showlegend: true },
      { ...line(t, traj.map(s => s.q), "#3ddc97"), name: "q pitch", showlegend: true },
      { ...line(t, traj.map(s => s.r), "#4aa3ff"), name: "r yaw", showlegend: true },
    ], { ...layout("Body angular rates", "t (s)", "rad/s"), showlegend: true,
         legend: { font: { color: MUTED } } }, CFG);
}

// ----------------------------------------------------------------------------
// 3D scene (three.js r128). World axes: x = East, y = up, z = depth.
// ----------------------------------------------------------------------------
function initThree() {
  const el = document.getElementById("view3d");
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x06080c);

  const camera = new THREE.PerspectiveCamera(50, el.clientWidth / el.clientHeight, 0.1, 1e7);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(el.clientWidth, el.clientHeight);
  el.appendChild(renderer.domElement);

  scene.add(new THREE.AmbientLight(0xffffff, 0.7));
  const sun = new THREE.DirectionalLight(0xffffff, 0.8);
  sun.position.set(1, 2, 1);
  scene.add(sun);

  // Rocket: cone nose + cylinder body, built pointing along local +x (the body
  // nose axis), so applying the body->world quaternion aims it correctly.
  const rocket = new THREE.Group();
  const mat = new THREE.MeshStandardMaterial({ color: 0xff7b3d, metalness: 0.3, roughness: 0.5 });
  const body = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, 4, 16), mat);
  const nose = new THREE.Mesh(new THREE.ConeGeometry(1, 2, 16), mat);
  nose.position.y = 3;
  const inner = new THREE.Group();
  inner.add(body); inner.add(nose);
  inner.rotation.z = -Math.PI / 2;   // local +y model -> local +x (nose forward)
  rocket.add(inner);
  scene.add(rocket);

  const trailGeo = new THREE.BufferGeometry();
  const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0x4aa3ff }));
  scene.add(trail);

  const grid = new THREE.GridHelper(10, 20, 0x30506f, 0x1c2a3a);
  scene.add(grid);

  three = { scene, camera, renderer, rocket, inner, trail, grid, el,
            target: new THREE.Vector3(), dist: 10, yaw: 0.7, pitch: 0.4, rocketScale: 1 };

  setupOrbit();
  window.addEventListener("resize", onResize);
  animate();
}

function onResize() {
  if (!three) return;
  const { el, camera, renderer } = three;
  camera.aspect = el.clientWidth / el.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(el.clientWidth, el.clientHeight);
}

// Minimal orbit controls (drag to rotate, wheel to zoom) to avoid an extra dep.
function setupOrbit() {
  const dom = three.renderer.domElement;
  let dragging = false, px = 0, py = 0;
  dom.addEventListener("mousedown", e => { dragging = true; px = e.clientX; py = e.clientY; });
  window.addEventListener("mouseup", () => dragging = false);
  window.addEventListener("mousemove", e => {
    if (!dragging) return;
    three.yaw -= (e.clientX - px) * 0.01;
    three.pitch = Math.max(-1.4, Math.min(1.4, three.pitch + (e.clientY - py) * 0.01));
    px = e.clientX; py = e.clientY;
  });
  dom.addEventListener("wheel", e => {
    e.preventDefault();
    three.dist *= (1 + Math.sign(e.deltaY) * 0.12);
  }, { passive: false });
}

function rebuildTrajectory() {
  if (!three || traj.length === 0) return;

  // Fit the scene to the flight's bounding box.
  const xs = traj.map(s => s.x), ys = traj.map(s => s.y), zs = traj.map(s => s.z);
  const minX = Math.min(...xs), maxX = Math.max(...xs);
  const minY = Math.min(...ys), maxY = Math.max(...ys);
  const minZ = Math.min(...zs), maxZ = Math.max(...zs);
  const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2, cz = (minZ + maxZ) / 2;
  const extent = Math.max(maxX - minX, maxY - minY, maxZ - minZ, 1);

  three.target.set(cx, cy, cz);
  three.dist = extent * 1.6;
  three.rocketScale = extent * 0.02;

  const pts = traj.map(s => new THREE.Vector3(s.x, s.y, s.z));
  three.trail.geometry.setFromPoints(pts);

  // Ground grid sized to the flight, sitting at y = 0.
  three.scene.remove(three.grid);
  three.grid = new THREE.GridHelper(extent * 2, 24, 0x30506f, 0x1c2a3a);
  three.grid.position.set(cx, 0, cz);
  three.scene.add(three.grid);

  // Reset scrubber.
  const slider = document.getElementById("time");
  slider.max = traj.length - 1;
  slider.value = traj.length - 1;
  slider.disabled = false;
  updateRocket(traj.length - 1);
}

function updateRocket(i) {
  if (!three || !traj[i]) return;
  const s = traj[i];
  three.rocket.position.set(s.x, s.y, s.z);
  three.rocket.quaternion.set(s.qx, s.qy, s.qz, s.qw); // three uses (x,y,z,w)
  three.inner.scale.setScalar(three.rocketScale);
  document.getElementById("timelabel").textContent = `t = ${s.t.toFixed(2)} s`;
}

function animate() {
  requestAnimationFrame(animate);
  if (!three) return;
  const { camera, target, dist, yaw, pitch } = three;
  camera.position.set(
    target.x + dist * Math.cos(pitch) * Math.sin(yaw),
    target.y + dist * Math.sin(pitch),
    target.z + dist * Math.cos(pitch) * Math.cos(yaw),
  );
  camera.lookAt(target);
  three.renderer.render(three.scene, camera);
}

// ----------------------------------------------------------------------------
// Time scrubber + playback.
// ----------------------------------------------------------------------------
const slider = document.getElementById("time");
slider.addEventListener("input", () => { stopPlay(); updateRocket(+slider.value); });

let playId = null;
const playBtn = document.getElementById("play");
function stopPlay() { if (playId) { cancelAnimationFrame(playId); playId = null; playBtn.innerHTML = "▶ Play"; } }
playBtn.addEventListener("click", () => {
  if (playId) { stopPlay(); return; }
  playBtn.innerHTML = "❚❚ Pause";
  let i = (+slider.value >= traj.length - 1) ? 0 : +slider.value;
  const tick = () => {
    i += 3;                          // ~30 ms of flight per frame
    if (i >= traj.length) { stopPlay(); return; }
    slider.value = i;
    updateRocket(i);
    playId = requestAnimationFrame(tick);
  };
  playId = requestAnimationFrame(tick);
});

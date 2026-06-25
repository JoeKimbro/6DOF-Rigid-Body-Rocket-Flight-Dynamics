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
    startAltitude:0, total_time:80, theta_deg:5, phi_deg:90, CP:0.6, CG:0.5,
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

function runFlight() {
  if (!Module) return;
  const params = readParams();
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

  renderSummary();
  renderPlots();
  rebuildTrajectory();
}

// ----------------------------------------------------------------------------
// Summary stat cards.
// ----------------------------------------------------------------------------
function renderSummary() {
  const apogee = Math.max(...traj.map(s => s.y));
  const vmax = Math.max(...traj.map(s => s.speed));
  const last = traj[traj.length - 1];
  const range = Math.hypot(last.x, last.z);
  const flightTime = last.t;
  const maxRoll = Math.max(...traj.map(s => Math.abs(s.p)));

  const cards = [
    ["Apogee", apogee.toFixed(0), "m"],
    ["Max speed", vmax.toFixed(0), "m/s"],
    ["Flight time", flightTime.toFixed(1), "s"],
    ["Downrange", range.toFixed(0), "m"],
    ["Peak roll rate", maxRoll.toFixed(1), "rad/s"],
    ["Impact speed", last.speed.toFixed(0), "m/s"],
  ];
  document.getElementById("summary").innerHTML = cards.map(
    ([k, v, u]) => `<div class="stat"><div class="k">${k}</div><div class="v">${v} <span class="u">${u}</span></div></div>`
  ).join("");
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

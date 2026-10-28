"""The IK simulator's page: one HTML document, no external files."""

IK_PAGE_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Ring IK simulator</title>
<style>
:root {
  --bg: #f6f7f9; --panel: #ffffff; --ink: #1c2330; --muted: #5d6878; --line: #d9dee6; --grid: #e8ebf0;
  --decoded: #2a78d6; --error: #eb6834; --goal: #8a56c2; --x: #d9443a; --y: #2f9e5b; --z: #2a78d6; --cell: 28,35,48;
  --m-ring: #2a78d6; --m-ring_all: #7fb2ea; --m-cascade: #1c2330; --m-dls: #eb6834; --m-transpose: #1baf7a; --m-ccd: #b5892a;
}
@media (prefers-color-scheme: dark) {
  :root { --bg: #12151b; --panel: #1a1f28; --ink: #e8ecf2; --muted: #9aa5b5; --line: #2f3744; --grid: #252c37;
          --decoded: #5aa2f0; --error: #f08a5d; --goal: #b58ae6; --x: #f06a60; --y: #53c183; --z: #5aa2f0; --cell: 232,236,242;
          --m-ring: #5aa2f0; --m-ring_all: #9cc6f5; --m-cascade: #e8ecf2; --m-dls: #f08a5d; --m-transpose: #53c183; --m-ccd: #d6ad4f; }
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--ink); font: 14px/1.4 system-ui, -apple-system, "Segoe UI", sans-serif; }
header { display: flex; flex-wrap: wrap; gap: 12px 20px; align-items: baseline; padding: 14px 16px 6px; }
h1 { font-size: 17px; margin: 0; font-weight: 650; }
header .status { color: var(--muted); font-variant-numeric: tabular-nums; }
main { display: grid; grid-template-columns: minmax(0, 1.1fr) minmax(0, 1fr); gap: 12px; padding: 8px 16px 20px; }
@media (max-width: 900px) { main { grid-template-columns: minmax(0, 1fr); } }
section { background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 12px 14px; min-width: 0; }
section.wide { grid-column: 1 / -1; }
h2 { font-size: 13px; margin: 0 0 8px; font-weight: 650; color: var(--muted); }
canvas { display: block; width: 100%; }
#view { height: 400px; cursor: grab; touch-action: none; }
.big { display: flex; flex-wrap: wrap; gap: 6px 24px; align-items: baseline; margin-top: 8px; }
.big b { font-size: 26px; font-variant-numeric: tabular-nums; }
.big span { color: var(--muted); }
.legend { display: flex; flex-wrap: wrap; gap: 4px 16px; color: var(--muted); font-size: 12px; margin-top: 6px; }
.swatch { display: inline-block; width: 22px; height: 0; border-top: 2px solid var(--ink); vertical-align: middle; margin-right: 5px; }
.swatch.dash { border-top-style: dashed; } .swatch.dot { border-top-style: dotted; border-top-width: 3px; }
.row { display: flex; flex-wrap: wrap; gap: 8px 14px; align-items: center; margin-bottom: 10px; }
button, select { font: inherit; color: var(--ink); background: var(--panel); border: 1px solid var(--line); border-radius: 6px; padding: 5px 11px; cursor: pointer; }
button.on { background: var(--decoded); border-color: var(--decoded); color: #fff; }
button:disabled, select:disabled { opacity: 0.5; cursor: default; }
button:focus-visible, select:focus-visible, input:focus-visible { outline: 2px solid var(--decoded); outline-offset: 2px; }
.joint { display: grid; grid-template-columns: 96px minmax(0, 1fr) 118px; gap: 8px; align-items: center; margin: 3px 0; }
.joint label { color: var(--muted); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.joint output { font-variant-numeric: tabular-nums; text-align: right; color: var(--muted); }
input[type=range] { width: 100%; accent-color: var(--decoded); }
.chart { height: 150px; margin-bottom: 6px; }
.chart-title { display: flex; justify-content: space-between; color: var(--muted); font-size: 12px; font-variant-numeric: tabular-nums; }
.note { color: var(--muted); font-size: 12px; margin: 6px 0 0; }
.share { display: grid; grid-template-columns: 96px minmax(0, 1fr) 170px; gap: 8px; align-items: center; margin: 3px 0; }
.share label { color: var(--muted); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.share output { font-variant-numeric: tabular-nums; text-align: right; color: var(--muted); }
.bar { position: relative; height: 12px; background: var(--grid); border-radius: 3px; }
.bar i { position: absolute; top: 0; bottom: 0; background: var(--goal); border-radius: 3px; }
.bar u { position: absolute; top: -2px; bottom: -2px; left: 50%; width: 1px; background: var(--muted); }
table { border-collapse: collapse; width: 100%; font-size: 12.5px; font-variant-numeric: tabular-nums; }
th, td { text-align: right; padding: 3px 8px; border-bottom: 1px solid var(--grid); }
th:first-child, td:first-child { text-align: left; }
th { color: var(--muted); font-weight: 600; }
td .key { display: inline-block; width: 14px; height: 3px; vertical-align: middle; margin-right: 6px; border-radius: 2px; }
#gzbox { position: relative; background: var(--grid); border-radius: 6px; min-height: 240px; display: flex; align-items: center; justify-content: center; overflow: hidden; }
#gz { display: block; width: 100%; }
#gznote { color: var(--muted); padding: 16px; text-align: center; font-size: 13px; }
[hidden] { display: none !important; }
</style>
</head>
<body>
<header>
  <h1>Inverse kinematics on ring attractors</h1>
  <label>Chain <select id="chain"></select></label>
  <span class="status" id="status">connecting…</span>
</header>
<main>
  <section>
    <h2>The arm and the goal. Drag a ring of the goal to turn it about that axis; drag inside it to turn it freely</h2>
    <canvas id="view"></canvas>
    <div class="legend">
      <span><i class="swatch"></i>true frame of the hand</span><span><i class="swatch dash"></i>read from the network</span>
      <span><i class="swatch dot"></i>goal, with its three rings</span>
      <span style="color:var(--x)">x</span><span style="color:var(--y)">y</span><span style="color:var(--z)">z</span>
      <span>drag elsewhere to turn the view</span>
    </div>
    <div class="big">
      <div><b id="gerr">–</b> <span>from the goal (true orientation)</span></div>
      <div><b id="left">–</b> <span>of the turn no joint takes</span></div>
      <div><b id="err">–</b> <span>error of the network's readout</span></div>
    </div>
  </section>
  <section>
    <h2>Gazebo</h2>
    <div id="gzbox"><img id="gz" alt="Camera picture of the TIAGo in Gazebo" hidden><div id="gznote"></div></div>
    <p class="note" id="gzline"></p>
  </section>
  <section>
    <h2>Method, robot and goal</h2>
    <div class="row">
      <label>Method <select id="method"></select></label>
      <label>Robot <select id="robot"></select></label>
    </div>
    <div class="row">
      <button id="rgoal">Random goal</button><button id="replay">Same start and goal, next method</button><button id="stop">Stop here</button>
    </div>
    <div class="joint"><label for="g0">goal lift</label><input id="g0" type="range" min="-3.1416" max="3.1416" step="0.01" value="0"><output id="go0"></output></div>
    <div class="joint"><label for="g1">goal pitch</label><input id="g1" type="range" min="-1.5708" max="1.5708" step="0.01" value="0"><output id="go1"></output></div>
    <div class="joint"><label for="g2">goal yaw</label><input id="g2" type="range" min="-3.1416" max="3.1416" step="0.01" value="0"><output id="go2"></output></div>
    <p class="note" id="methodnote"></p>
    <h2 style="margin-top:12px">Joint angles (moving a slider takes the arm over by hand; the goal then stays on the hand)</h2>
    <div class="row"><label for="speed">Speed by hand</label>
      <input id="speed" type="range" min="0.1" max="3" step="0.05" value="1" style="width:160px"><output id="speedv"></output></div>
    <div id="joints"></div>
  </section>
  <section>
    <h2>Distance from the goal, one line per run (the last six)</h2>
    <canvas class="chart" id="runs" style="height:190px"></canvas>
    <table id="runtable"><thead><tr><th>Run</th><th>Robot</th><th>Turn</th><th>Within 5° after</th><th>Now / final</th><th>Path / shortest</th></tr></thead><tbody></tbody></table>
    <p class="note">A run starts with a random goal, with a drag that is let go, or with "same start and goal, next method", which first puts the arm back where the last run started.</p>
  </section>
  <section>
    <h2>Each joint's share of the turn (bar: share, up to the largest it takes; number: lead of its goal ring)</h2>
    <div id="shares"></div>
    <p class="note" id="sharenote"></p>
    <div class="chart-title" style="margin-top:10px"><span>lift, pitch, yaw: goal on the goal rings (purple) and true (dark)</span><span id="v0"></span></div>
    <canvas class="chart" id="lpy" style="height:130px"></canvas>
  </section>
  <section>
    <h2>Spikes of the last 50 ms: each joint's state ring and goal ring, the output rings, the three goal rings</h2>
    <canvas id="rings"></canvas>
    <p class="note">One row per ring, one cell per neuron; darker is more spikes. The tick under a state ring is where the joint angle was written; under an output ring the true angle; under a goal ring the goal that is written on it.</p>
  </section>
</main>
<script>
"use strict";
const $ = id => document.getElementById(id);
const css = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const deg = r => r * 180 / Math.PI;
let cfg = null, snap = null, hist = [], cam = {az: -0.9, el: 0.35}, dirty = true, sliders = [], lastSent = 0;
let goalSliders = [], shareRows = [], goalHeld = 0, goalSent = 0, dragGoal = null, gzFrames = -1, gzLoading = false, labels = {};

function post(path, body) {
  return fetch(path, {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body || {})});
}
function fit(canvas) {
  const r = canvas.getBoundingClientRect(), d = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(r.width * d) || canvas.height !== Math.round(r.height * d)) {
    canvas.width = Math.round(r.width * d); canvas.height = Math.round(r.height * d);
  }
  const g = canvas.getContext("2d"); g.setTransform(d, 0, 0, d, 0, 0); g.clearRect(0, 0, r.width, r.height);
  return [g, r.width, r.height];
}

function buildControls() {
  const sel = $("chain"); sel.innerHTML = "";
  for (const name of cfg.chains) { const o = document.createElement("option"); o.value = o.textContent = name; sel.appendChild(o); }
  sel.value = cfg.chain;
  const ms = $("method"); ms.innerHTML = ""; labels = {};
  for (const m of cfg.methods) { const o = document.createElement("option"); o.value = m.name; o.textContent = m.label; ms.appendChild(o); labels[m.name] = m.label; }
  const rs = $("robot"); rs.innerHTML = "";
  const labels3 = {light: "Light robot (no dynamics)", gazebo: "TIAGo in Gazebo", kangaroo: "Kangaroo in MuJoCo"};
  for (const name of Object.keys(labels3)) {
    const o = document.createElement("option"); o.value = name;
    o.textContent = labels3[name];
    o.disabled = name !== "light" && !(cfg.robot_chains[name] || []).includes(cfg.chain);
    rs.appendChild(o);
  }
  const box = $("joints"); box.innerHTML = ""; sliders = [];
  cfg.joints.forEach((j, k) => {
    const row = document.createElement("div"); row.className = "joint";
    const label = document.createElement("label"); label.textContent = j.name + " (" + j.axis + ")"; label.htmlFor = "q" + k;
    const input = document.createElement("input"); input.type = "range"; input.id = "q" + k;
    input.min = j.lower; input.max = j.upper; input.step = 0.005; input.value = 0;
    const out = document.createElement("output");
    input.addEventListener("input", sendTargets);
    row.append(label, input, out); box.appendChild(row); sliders.push({input, out, held: 0});
  });
  const sh = $("shares"); sh.innerHTML = ""; shareRows = [];
  cfg.joints.forEach(j => {
    const row = document.createElement("div"); row.className = "share";
    const label = document.createElement("label"); label.textContent = j.name;
    const bar = document.createElement("div"); bar.className = "bar"; const fill = document.createElement("i"); bar.append(fill, document.createElement("u"));
    const out = document.createElement("output");
    row.append(label, bar, out); sh.appendChild(row); shareRows.push({fill, out});
  });
  goalSliders = [0, 1, 2].map(a => $("g" + a));
  hist = []; snap = null; centre = null; gzFrames = -1;
}
function sendTargets() {
  const now = performance.now(); sliders.forEach(s => s.held = now);
  if (now - lastSent < 40) return; lastSent = now;
  post("/api/target", {q: sliders.map(s => parseFloat(s.input.value))});
}
function sendGoalSliders(commit) {
  const now = performance.now(); goalHeld = now;
  if (!commit && now - goalSent < 80) return; goalSent = now;
  post("/api/goal", {lpy: goalSliders.map(g => parseFloat(g.value)), commit: !!commit});
}
[0, 1, 2].forEach(a => { $("g" + a).addEventListener("input", () => sendGoalSliders(false)); $("g" + a).addEventListener("change", () => sendGoalSliders(true)); });
$("rgoal").onclick = () => post("/api/random_goal");
$("replay").onclick = () => post("/api/replay");
$("stop").onclick = () => post("/api/stop");
$("speed").oninput = e => post("/api/speed", {speed: parseFloat(e.target.value)});
$("method").onchange = e => { $("status").textContent = "switching method…"; post("/api/method", {method: e.target.value}); };
$("robot").onchange = e => post("/api/robot", {robot: e.target.value});
$("chain").onchange = e => { $("status").textContent = "building " + e.target.value + "…"; post("/api/chain", {chain: e.target.value}); };

function onTick(s) {
  if (hist.length && s.t <= hist[hist.length - 1].t) hist = hist.filter(p => p.t < s.t);
  snap = s; hist.push({t: s.t, tr: s.lpy_true, goal: s.ik.written});
  while (hist.length && s.t - hist[0].t > 30) hist.shift();
  dirty = true;
}
function connect() {
  const es = new EventSource("/events");
  es.addEventListener("config", e => { cfg = JSON.parse(e.data); buildControls(); });
  es.addEventListener("tick", e => onTick(JSON.parse(e.data)));
  es.onerror = () => { $("status").textContent = "connection lost; retrying…"; };
}
// "?poll" asks for the state ten times a second instead of keeping a stream open; "?poll&stop=N" stops
// after N answers, so that a headless browser can take a picture of a page that has come to rest.
async function poll(stop) {
  cfg = await (await fetch("/config")).json(); buildControls();
  let left = stop || Infinity, last = -1;
  const timer = setInterval(async () => {
    try {
      const s = await (await fetch("/state")).json();
      if (s.chain && s.chain !== cfg.chain) { cfg = await (await fetch("/config")).json(); buildControls(); }
      if (s.t !== undefined && s.t !== last) { last = s.t; onTick(s); }
    } catch (e) { $("status").textContent = "connection lost; retrying…"; }
    if (--left <= 0) clearInterval(timer);
  }, 100);
}

// ---- small vector helpers (matrices are 9 numbers, row by row) ----
const col = (R, a) => [R[a], R[3 + a], R[6 + a]];
const cross = (u, v) => [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
const dot = (u, v) => u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
function rotAbout(axis, angle) {
  const n = Math.hypot(axis[0], axis[1], axis[2]) || 1, x = axis[0] / n, y = axis[1] / n, z = axis[2] / n, c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
  return [t * x * x + c, t * x * y - s * z, t * x * z + s * y, t * x * y + s * z, t * y * y + c, t * y * z - s * x, t * x * z - s * y, t * y * z + s * x, t * z * z + c];
}
function matmul(A, B) {
  const C = new Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) C[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
  return C;
}

// ---- 3D view ----
let centre = null, viewScale = 1, goalLen = 0.1, ringPaths = [];
function camAxes() {      // screen right, screen up, and the direction toward the viewer, in world coordinates
  const ca = Math.cos(cam.az), sa = Math.sin(cam.az), ce = Math.cos(cam.el), se = Math.sin(cam.el);
  const right = [ca, -sa, 0], up = [se * sa, se * ca, ce];
  return [right, up, cross(right, up)];
}
function project(p, w, h, scale) {
  const [right, up] = camAxes(), d = [p[0] - centre[0], p[1] - centre[1], p[2] - centre[2]];
  return [w / 2 + scale * dot(d, right), h / 2 - scale * dot(d, up)];
}
function line(g, a, b, color, width, dash) {
  g.beginPath(); g.setLineDash(dash || []); g.strokeStyle = color; g.lineWidth = width; g.lineCap = "round";
  g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke(); g.setLineDash([]);
}
function goalNow() { return dragGoal ? dragGoal.matrix : snap.ik.goal_matrix; }
function drawView() {
  const [g, w, h] = fit($("view"));
  if (!snap || !cfg) return;
  const pts = snap.points, n = cfg.joints.length;
  const mid = [0, 1, 2].map(a => (Math.min(...pts.map(p => p[a])) + Math.max(...pts.map(p => p[a]))) / 2);
  centre = centre ? centre.map((c, a) => c + 0.05 * (mid[a] - c)) : mid;
  const reach = Math.max(cfg.reach, 0.2), scale = Math.min(w, h) * 0.7 / reach, P = p => project(p, w, h, scale);
  viewScale = scale;
  const ink = css("--ink"), muted = css("--muted"), axes = [css("--x"), css("--y"), css("--z")];
  const f = reach * 0.35; line(g, P([-f, 0, 0]), P([f, 0, 0]), css("--line"), 1); line(g, P([0, -f, 0]), P([0, f, 0]), css("--line"), 1);
  for (let k = 0; k + 1 < pts.length; k++) line(g, P(pts[k]), P(pts[k + 1]), muted, 5);
  for (let k = 0; k < pts.length; k++) { const q = P(pts[k]); g.beginPath(); g.fillStyle = ink; g.arc(q[0], q[1], k === pts.length - 1 ? 4.5 : 3.5, 0, 7); g.fill(); }
  const len = reach * 0.11;
  for (let k = 0; k <= n; k++) {
    const o = pts[k + 1], big = k === n, L = big ? len * 1.6 : len * 0.8;
    for (const [R, dash, width] of [[snap.true[k], null, big ? 3 : 1.5], [snap.decoded[k], [5, 4], big ? 2 : 1]]) {
      for (let a = 0; a < 3; a++) {
        const c = col(R, a), s = L * (dash ? 1.15 : 1);
        line(g, P(o), P([o[0] + s * c[0], o[1] + s * c[1], o[2] + s * c[2]]), axes[a], width, dash);
      }
    }
  }
  // the goal at the hand: three dotted axes and a ring around each of them
  const o = pts[n + 1], G = goalNow(), L = len * 2.4; goalLen = L; ringPaths = [];
  for (let a = 0; a < 3; a++) {
    const c = col(G, a), u = col(G, (a + 1) % 3), v = col(G, (a + 2) % 3);
    line(g, P(o), P([o[0] + L * c[0], o[1] + L * c[1], o[2] + L * c[2]]), axes[a], 3, [1, 6]);
    const path = [];
    for (let i = 0; i <= 48; i++) {
      const t = i / 48 * 2 * Math.PI, cu = Math.cos(t) * L, sv = Math.sin(t) * L;
      path.push(P([o[0] + cu * u[0] + sv * v[0], o[1] + cu * u[1] + sv * v[1], o[2] + cu * u[2] + sv * v[2]]));
    }
    ringPaths.push(path);
    g.beginPath(); g.strokeStyle = axes[a]; g.globalAlpha = dragGoal && dragGoal.axis === a ? 1 : 0.55; g.lineWidth = dragGoal && dragGoal.axis === a ? 3 : 1.6;
    path.forEach((q, i) => i ? g.lineTo(q[0], q[1]) : g.moveTo(q[0], q[1])); g.stroke(); g.globalAlpha = 1;
  }
  ringPaths.centre = P(o);
}
function nearestRing(x, y) {      // which goal ring is under the pointer, or -1 (inside the goal: free turn), or null
  if (!ringPaths.length) return null;
  let best = null, bestD = 9;
  ringPaths.forEach((path, a) => { for (const q of path) { const d = Math.hypot(q[0] - x, q[1] - y); if (d < bestD) { bestD = d; best = a; } } });
  if (best !== null) return best;
  return Math.hypot(ringPaths.centre[0] - x, ringPaths.centre[1] - y) < goalLen * viewScale ? -1 : null;
}
(function () {
  const v = $("view"); let drag = null;
  const at = e => { const r = v.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; };
  v.addEventListener("pointerdown", e => {
    const [x, y] = at(e), hit = snap ? nearestRing(x, y) : null;
    v.setPointerCapture(e.pointerId);
    if (hit === null) { drag = [e.clientX, e.clientY, cam.az, cam.el]; v.style.cursor = "grabbing"; return; }
    dragGoal = {axis: hit, matrix: snap.ik.goal_matrix.slice(), last: [x, y], sent: 0};
    goalHeld = performance.now(); dirty = true;
  });
  v.addEventListener("pointermove", e => {
    if (dragGoal) {
      const [x, y] = at(e), c = ringPaths.centre, [right, up, toward] = camAxes();
      let turn;
      if (dragGoal.axis >= 0) {      // about one axis of the goal: by the angle the pointer went round the centre
        const a0 = Math.atan2(-(dragGoal.last[1] - c[1]), dragGoal.last[0] - c[0]), a1 = Math.atan2(-(y - c[1]), x - c[0]);
        let d = a1 - a0; if (d > Math.PI) d -= 2 * Math.PI; if (d < -Math.PI) d += 2 * Math.PI;
        const axis = col(dragGoal.matrix, dragGoal.axis);
        turn = rotAbout(axis, d * (dot(axis, toward) >= 0 ? 1 : -1));
      } else {                       // freely: sideways about the screen's up, up and down about its right
        const dx = x - dragGoal.last[0], dy = y - dragGoal.last[1];
        turn = matmul(rotAbout(up, dx * 0.012), rotAbout(right, dy * 0.012));
      }
      dragGoal.matrix = matmul(turn, dragGoal.matrix); dragGoal.last = [x, y]; goalHeld = performance.now(); dirty = true;
      const now = performance.now();
      if (now - dragGoal.sent > 80) { dragGoal.sent = now; post("/api/goal", {matrix: dragGoal.matrix, commit: false}); }
      return;
    }
    if (!drag) { const [x, y] = at(e), hit = snap ? nearestRing(x, y) : null; v.style.cursor = hit === null ? "grab" : hit < 0 ? "move" : "crosshair"; return; }
    cam.az = drag[2] + (e.clientX - drag[0]) * 0.01; cam.el = Math.max(-1.5, Math.min(1.5, drag[3] + (e.clientY - drag[1]) * 0.01)); dirty = true;
  });
  const end = () => {
    if (dragGoal) { post("/api/goal", {matrix: dragGoal.matrix, commit: true}); const m = dragGoal.matrix; dragGoal = null; setTimeout(() => { dirty = true; }, 300); if (snap) snap.ik.goal_matrix = m; }
    drag = null; v.style.cursor = "grab"; dirty = true;
  };
  v.addEventListener("pointerup", end); v.addEventListener("pointercancel", end);
})();

// ---- charts ----
function axesFrame(g, w, h, lo, hi, ticks, xmax, xlabel) {
  const x0 = 34, x1 = w - 6, y0 = 8, y1 = h - 20;
  const X = t => x0 + t / xmax * (x1 - x0), Y = v => y1 - (v - lo) / (hi - lo) * (y1 - y0);
  g.font = "11px system-ui, sans-serif"; g.fillStyle = css("--muted"); g.textBaseline = "middle"; g.textAlign = "right";
  for (const v of ticks) { g.strokeStyle = css("--grid"); g.lineWidth = 1; g.beginPath(); g.moveTo(x0, Y(v)); g.lineTo(x1, Y(v)); g.stroke(); g.fillText(String(v), x0 - 5, Y(v)); }
  g.textAlign = "center"; g.textBaseline = "top";
  for (let t = 0; t <= xmax + 1e-9; t += xmax / 4) g.fillText(t.toFixed(0) + (t === 0 ? " s" : ""), X(t), y1 + 4);
  if (xlabel) { g.textAlign = "right"; g.fillText(xlabel, x1, y1 + 4); }
  return [X, Y];
}
function drawRuns() {
  const [g, w, h] = fit($("runs"));
  if (!snap) return;
  const runs = snap.runs || [];
  let xmax = 10, top = 30;
  for (const r of runs) { if (r.t.length) xmax = Math.max(xmax, r.t[r.t.length - 1]); top = Math.max(top, ...r.error); }
  xmax = Math.ceil(xmax / 4) * 4; top = Math.ceil(top / 30) * 30;
  const [X, Y] = axesFrame(g, w, h, 0, top, [0, top / 2, top], xmax);
  g.strokeStyle = css("--muted"); g.setLineDash([4, 4]); g.lineWidth = 1; g.beginPath(); g.moveTo(X(0), Y(5)); g.lineTo(X(xmax), Y(5)); g.stroke(); g.setLineDash([]);
  for (const r of runs) {
    g.strokeStyle = css("--m-" + r.method); g.lineWidth = r.live ? 2.6 : 1.8; g.globalAlpha = r.live ? 1 : 0.8; g.lineJoin = "round";
    g.setLineDash(r.robot !== "light" ? [6, 3] : []); g.beginPath();
    r.t.forEach((t, i) => i ? g.lineTo(X(t), Y(r.error[i])) : g.moveTo(X(t), Y(r.error[i]))); g.stroke(); g.setLineDash([]); g.globalAlpha = 1;
  }
  const body = $("runtable").tBodies[0]; body.innerHTML = "";
  for (const r of runs.slice().reverse()) {
    const tr = document.createElement("tr");
    const cells = ["", ({gazebo: "Gazebo", kangaroo: "Kangaroo"})[r.robot] || "light", r.start_error.toFixed(0) + "°", r.reached_s === null ? "not yet" : r.reached_s.toFixed(1) + " s",
                   r.final_error === null ? "–" : r.final_error.toFixed(1) + "°", r.path_ratio === null ? "–" : r.path_ratio.toFixed(2)];
    cells.forEach((text, i) => { const td = document.createElement("td"); td.textContent = text; tr.appendChild(td); });
    const key = document.createElement("span"); key.className = "key"; key.style.background = css("--m-" + r.method);
    tr.firstChild.append(key, document.createTextNode("#" + r.id + " " + (labels[r.method] || r.method) + (r.live ? " (running)" : "")));
    body.appendChild(tr);
  }
}
function drawLpy() {
  const [g, w, h] = fit($("lpy"));
  if (!hist.length) return;
  const x0 = 34, x1 = w - 4, y0 = 8, y1 = h - 8, now = hist[hist.length - 1].t;
  const X = t => x1 - (now - t) / 30 * (x1 - x0), Y = v => y1 - (v + 180) / 360 * (y1 - y0);
  g.font = "11px system-ui, sans-serif"; g.fillStyle = css("--muted"); g.textBaseline = "middle"; g.textAlign = "right";
  for (const v of [-180, 0, 180]) { g.strokeStyle = css("--grid"); g.lineWidth = 1; g.beginPath(); g.moveTo(x0, Y(v)); g.lineTo(x1, Y(v)); g.stroke(); g.fillText(String(v), x0 - 5, Y(v)); }
  const names = ["lift", "pitch", "yaw"], dashes = [[], [6, 3], [2, 3]];
  for (let a = 0; a < 3; a++) for (const [key, color, width] of [["goal", css("--goal"), 1.5], ["tr", css("--ink"), 1.8]]) {
    g.strokeStyle = color; g.lineWidth = width; g.setLineDash(dashes[a]); g.beginPath(); let prev = null;
    for (const p of hist) { const raw = p[key] ? p[key][a] : null; if (raw === null || raw === undefined) { prev = null; continue; } const v = deg(raw);
      if (prev === null || Math.abs(v - prev) > 180) g.moveTo(X(p.t), Y(v)); else g.lineTo(X(p.t), Y(v)); prev = v; }
    g.stroke();
  }
  g.setLineDash([]);
  if (snap) $("v0").textContent = names.map((nm, a) => nm + " " + deg(snap.lpy_true[a]).toFixed(0) + "° → " + (snap.ik.goal[a] === null ? "–" : deg(snap.ik.goal[a]).toFixed(0) + "°")).join("   ") + "   (solid, dashed, dotted)";
}

// ---- rings ----
function drawRings() {
  if (!snap || !cfg) return;
  const canvas = $("rings"), rowH = 16, gap = 8, labelW = 78, ik = snap.ik;
  const short = name => name.replace("_joint", "");
  const rows = [];
  snap.rings.forEach((counts, k) => {
    rows.push({label: short(cfg.joints[k].name), counts, mark: snap.written[k], color: css("--ink"), tight: true});
    rows.push({label: "  its goal", counts: ik.joint_goal_rings[k], mark: null, color: css("--goal"), goal: true});
  });
  ["lift", "pitch", "yaw"].forEach((name, a) => {
    const n = snap.out_rings[a].length;
    rows.push({label: name, counts: snap.out_rings[a], mark: ((snap.lpy_true[a] / (2 * Math.PI)) % 1 + 1) % 1 * n, color: css("--decoded"), space: a === 0});
  });
  ["lift", "pitch", "yaw"].forEach((name, a) => {
    const n = ik.goal_rings[a].length, w = ik.written[a];
    rows.push({label: "goal " + name, counts: ik.goal_rings[a], mark: w === null ? null : ((w / (2 * Math.PI)) % 1 + 1) % 1 * n, color: css("--goal"), goal: true, space: a === 0});
  });
  let total = 16; rows.forEach(r => { total += rowH + (r.tight ? 2 : gap) + (r.space ? 10 : 0); });
  canvas.style.height = total + "px";
  const [g, w] = fit(canvas);
  const rgb = css("--cell"), muted = css("--muted");
  g.font = "11px system-ui, sans-serif"; g.textBaseline = "middle";
  let y = 4;
  rows.forEach(r => {
    if (r.space) y += 10;
    const n = r.counts.length, cw = (w - labelW) / n, top = Math.max(3, ...r.counts);
    g.fillStyle = r.goal ? css("--goal") : muted; g.textAlign = "left"; g.fillText(r.label, 0, y + rowH / 2);
    g.strokeStyle = css("--line"); g.lineWidth = 1; g.strokeRect(labelW - 0.5, y - 0.5, w - labelW, rowH + 1);
    for (let i = 0; i < n; i++) if (r.counts[i] > 0) { g.fillStyle = "rgba(" + rgb + "," + (0.18 + 0.82 * r.counts[i] / top) + ")"; g.fillRect(labelW + i * cw, y, Math.max(cw, 1), rowH); }
    if (r.mark !== null && r.mark !== undefined) { g.fillStyle = r.color; const x = labelW + (r.mark + 0.5) * cw; g.fillRect(x - 1, y + rowH + 1, 2, r.tight ? 2 : 5); }
    y += rowH + (r.tight ? 2 : gap);
  });
}

// ---- Gazebo panel ----
function drawGazebo() {
  if (!snap || !cfg) return;
  const img = $("gz"), note = $("gznote"), gzb = snap.gazebo;
  if (snap.robot === "kangaroo") {
    img.hidden = true; note.hidden = false; note.textContent = "MuJoCo shows the robot in its own window";
    $("gzline").textContent = "MuJoCo time " + gzb.sim_time.toFixed(2) + " s, stepped " + cfg.tick_ms + " ms per tick of the network · largest distance of a joint from its target " + Math.max(...gzb.lag.map(v => Math.abs(deg(v)))).toFixed(1) + "°"; return;
  }
  if (!cfg.robots.includes("gazebo")) {
    img.hidden = true; note.hidden = false;
    note.textContent = "Gazebo is not running beside this simulator. Start it with docker/run.bash gazebo up (see the README), then start the simulator inside it; the light robot works without.";
    $("gzline").textContent = ""; return;
  }
  if (!gzb) {
    img.hidden = true; note.hidden = false;
    note.textContent = (cfg.robot_chains.gazebo || []).includes(cfg.chain) ? "Gazebo is running. Choose \"TIAGo in Gazebo\" as the robot to move its arm." : "Gazebo has the TIAGo arm only: choose one of the tiago chains.";
    $("gzline").textContent = ""; return;
  }
  if (gzb.frames > 0 && gzb.frames !== gzFrames && !gzLoading) {
    gzLoading = true; gzFrames = gzb.frames;
    const next = new Image();
    next.onload = () => { img.src = next.src; img.hidden = false; note.hidden = true; gzLoading = false; };
    next.onerror = () => { gzLoading = false; };
    next.src = "/gazebo.jpg?f=" + gzb.frames;
  }
  if (gzb.frames === 0) {
    img.hidden = true; note.hidden = false;
    note.textContent = "Gazebo moves the arm, but its camera gives no picture (it needs the container started with --gpu). The arm on the left is drawn from Gazebo's joint angles.";
  }
  const lag = Math.max(...gzb.lag.map(v => Math.abs(deg(v))));
  $("gzline").textContent = "Gazebo time " + gzb.sim_time.toFixed(2) + " s, stepped 50 ms per tick of the network · largest distance of a joint from its target " + lag.toFixed(1) + "°";
}

const METHOD_NOTES = {
  ring: "Ring IK: the spiking network compares goal and decoded orientation, hands each joint its share down the arm, and each joint follows its goal ring.",
  ring_all: "Ring IK without the cascade: every joint gets the whole comparison (the network is rebuilt for this).",
  cascade: "The cascade computed exactly from the true joint angles, no neurons. The network still runs and shows its rings.",
  dls: "Damped least squares, the usual method: the smallest joint motion that makes the turn. Exact joint angles, no neurons.",
  transpose: "Jacobian transpose: every joint moves by the part of the turn about its axis. Exact joint angles, no neurons.",
  ccd: "Cyclic coordinate descent: joint after joint takes the angle that brings the hand closest to the goal. Exact joint angles, no neurons."
};
function drawText() {
  if (!snap || !cfg) return;
  const k = snap.ik, now = performance.now();
  $("gerr").textContent = k.goal_error.toFixed(1) + "°";
  $("left").textContent = k.left === null ? "–" : deg(k.left).toFixed(1) + "°";
  $("err").textContent = snap.error.toFixed(1) + "°";
  const modes = {ik: "turning to the goal", hold: "at rest", manual: "moved by hand", placing: "going back to the start pose"};
  $("status").textContent = cfg.chain + (cfg.stub ? " · stand-in network (no spikes)" : " · " + cfg.neurons.toLocaleString() + " neurons") +
    " · " + (modes[snap.mode] || snap.mode) + " · " + snap.rate.toFixed(2) + "× real time · t = " + snap.t.toFixed(1) + " s";
  if (document.activeElement !== $("method")) $("method").value = snap.method;
  if (document.activeElement !== $("robot")) $("robot").value = snap.robot;
  $("methodnote").textContent = METHOD_NOTES[snap.method] || "";
  $("sharenote").textContent = snap.method === "ring" || snap.method === "ring_all" ? "Shares as the network holds them." : "Shares as the chosen method computes them; the lead is still that of the network's goal rings.";
  $("speedv").textContent = snap.speed.toFixed(2) + " rad/s"; if (document.activeElement !== $("speed")) $("speed").value = snap.speed;
  $("replay").disabled = !(snap.runs && snap.runs.length);
  goalSliders.forEach((g, a) => { if (now - goalHeld > 800 && k.goal[a] !== null) g.value = k.goal[a]; $("go" + a).textContent = k.goal[a] === null ? "" : deg(k.goal[a]).toFixed(0) + "°"; });
  shareRows.forEach((r, j) => {
    const share = k.shares[j] || 0, part = Math.max(-1, Math.min(1, share / cfg.caps[j])) * 50;
    r.fill.style.left = (part >= 0 ? 50 : 50 + part) + "%"; r.fill.style.width = Math.abs(part) + "%";
    r.out.textContent = deg(share).toFixed(1) + "°, lead " + (k.lead[j] === null ? "–" : k.lead[j].toFixed(1)) + " idx";
  });
  sliders.forEach((s, j) => {
    if (now - s.held > 600) s.input.value = snap.mode === "manual" ? snap.target[j] : snap.q[j];
    s.out.textContent = deg(snap.q[j]).toFixed(1) + "°";
  });
}
function frame() {
  try {
    if (dirty) { dirty = false; drawView(); drawRuns(); drawLpy(); drawRings(); drawGazebo(); drawText(); }
  } finally { requestAnimationFrame(frame); }
}
window.addEventListener("resize", () => { dirty = true; });
window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => { dirty = true; });
const params = new URLSearchParams(location.search);
if (params.has("poll")) poll(parseInt(params.get("stop") || "0", 10)); else connect();
frame();
</script>
</body>
</html>
"""

__all__ = ["IK_PAGE_HTML"]

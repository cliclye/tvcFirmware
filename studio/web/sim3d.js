"use strict";
/* 3D flight simulation page. The physics and the flight software run in C (flight_sim3d.c);
 * this file draws the result with a small canvas 3D renderer (no external libraries).
 * Uses el, api, action, notice, lineChart, cfg, vehicle, saveState, appState from the other scripts. */

const SIM_STATES = ["IDLE", "ARMED", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED", "FAULT"];
const STATE_COLORS = ["#9a98a8", "#3aa776", "#e0661b", "#6a4ce5", "#6a4ce5", "#288471", "#288471", "#c0392b"];
let sim = null, simResult = null, simCol = {}, play = {t: 0, playing: false, speed: 1, last: 0};
let cam = {yaw: -0.9, pitch: 0.35, dist: 60, follow: true, trueScale: false};

/* ---- rocket mesh in body coordinates (metres, origin at the liftoff CG, +z toward the nose) ---- */
function noseRadius(shape, x, Ln, R) {
  const t = Math.max(0, Math.min(1, x / Ln));
  if (shape === "conical") return R * t;
  if (shape === "parabolic") return R * (2 * t - t * t);
  if (shape === "elliptical") return R * Math.sqrt(Math.max(0, 1 - (1 - t) ** 2));
  const rho = (R * R + Ln * Ln) / (2 * R), xx = Ln - x;
  return Math.sqrt(Math.max(0, rho * rho - xx * xx)) + R - rho;
}
/* Motor axis for gimbal angles (rad): rotate +z about X by the A angle, then about Y by B,
 * the same sense the simulator uses for the thrust vector. Drawn exaggerated so 5 deg is visible. */
function gimbalAxis(ga, gb, exaggerate = 4) {
  const a = ga * exaggerate, b = gb * exaggerate;
  return [-Math.cos(a) * Math.sin(b), Math.sin(a), Math.cos(a) * Math.cos(b)];
}
function revolve(profile, segments, color, transform = v => v) {
  const out = [];
  for (let i = 1; i < profile.length; i++) {
    const [z0, r0] = profile[i - 1], [z1, r1] = profile[i];
    for (let s = 0; s < segments; s++) {
      const a0 = 2 * Math.PI * s / segments, a1 = 2 * Math.PI * (s + 1) / segments;
      const pts = [[r0 * Math.cos(a0), r0 * Math.sin(a0), z0], [r1 * Math.cos(a0), r1 * Math.sin(a0), z1],
                   [r1 * Math.cos(a1), r1 * Math.sin(a1), z1], [r0 * Math.cos(a1), r0 * Math.sin(a1), z0]].map(transform);
      out.push({pts, color: typeof color === "function" ? color(i, s) : color});
    }
  }
  return out;
}
function rocketMesh(model, ga, gb) {
  const v = vehicle, b = v.body, cg = model.cg_wet_mm, R = v.diameter_mm / 2, Ln = b.nose_length_mm;
  const z = mm => (cg - mm) / 1000;
  const nose = [];
  for (let i = 0; i <= 14; i++) { const x = Ln * i / 14; nose.push([z(x), noseRadius(b.nose_shape, x, Ln, R) / 1000]); }
  const tail = Ln + b.length_mm;
  const body = [[z(Ln), R / 1000], [z(tail), R / 1000], [z(tail), (R - 1.5) / 1000]];
  const mr = v.motor.diameter_mm / 2000, zp = z(model.pivot_mm);
  const axis = gimbalAxis(ga, gb);
  /* rotate motor points about the pivot so +z maps onto the gimbal axis */
  const ex = unit3(cross3([0, 1, 0], axis)), ey = cross3(axis, ex);
  const motorT = p => add3(add3(add3([0, 0, zp], ex, p[0]), ey, p[1]), axis, p[2] - zp);
  const motor = [[z(model.motor_top_mm), mr], [z(model.nozzle_mm), mr], [z(model.nozzle_mm), mr * 0.55]];
  return [
    ...revolve(nose, 20, [210, 75, 75]),
    ...revolve(body, 20, (i, s) => s % 10 < 5 ? [106, 76, 229] : [92, 64, 205]),
    ...revolve(motor, 14, [120, 120, 132], motorT),
  ];
}

const SIM_GROUPS = [
  ["Aerodynamics", [["cn_alpha", "Nose normal-force slope CNα · /rad (Barrowman 2)", .1]]],
  ["Conditions", [["wind_m_s", "Wind · m/s", .5], ["gust_m_s", "Gusts up to · m/s", .5], ["wind_dir_deg", "Wind from · °", 5],
    ["pad_tilt_deg", "Pad lean · °", .5], ["misalign_deg", "Motor misalignment · °", .1], ["gyro_bias_dps", "Gyro bias · °/s", .1],
    ["noise", "Sensor noise × (1 = BMI088)", .1], ["seed", "Random seed", 1]]],
  ["Launch", [["tower_m", "Tower / guide length · m", .1]]],
];

function simField(key, label, step) {
  const input = el("input", {type: "number", step, "aria-label": label});
  input.value = sim[key];
  input.oninput = () => { sim[key] = input.value === "" ? NaN : Number(input.value); simChanged(); };
  return el("label", {class: "field"}, el("span", {text: label}), input);
}
let simSaveTimer = null;
function simChanged() {
  clearTimeout(simSaveTimer);
  simSaveTimer = setTimeout(() => saveState({sim}).catch(e => notice(e.message)), 600);
}
function renderSimInputs() {
  const wrap = $("sim-inputs"); wrap.replaceChildren();
  for (const [title, fields] of SIM_GROUPS) {
    wrap.append(el("h3", {class: "sub", text: title}), el("div", {class: "field-grid"}, ...fields.map(f => simField(...f))));
  }
  const profile = el("select", {"aria-label": "Thrust curve"});
  [[1, "Ignition spike (Estes-like)"], [0, "Flat average"]].forEach(([v, n]) => { const o = el("option", {text: n}); o.value = v; profile.append(o); });
  profile.value = String(sim.thrust_profile);
  profile.onchange = () => { sim.thrust_profile = Number(profile.value); simChanged(); };
  const reverse = el("input", {type: "checkbox"}); reverse.checked = sim.reverse_servo_a === 1;
  reverse.onchange = () => { sim.reverse_servo_a = reverse.checked ? 1 : 0; simChanged(); };
  wrap.append(el("label", {class: "field"}, el("span", {text: "Thrust curve"}), profile),
    el("h3", {class: "sub", text: "Fault injection"}),
    el("label", {class: "check"}, reverse, " Servo A installed backwards (what a failed tilt test would mean)"));
}
function simRequest() {
  if (!cfg || !vehicle) throw new Error("Rocket setup is not loaded yet.");
  return {config: cfg, vehicle, sim};
}

/* ------------------------------------------------------------------ run */
/* Shared by the 3D sim and TVC gimbal pages. Starts playback at T-1.5 s. */
async function runSimFlight() {
  $("sim-run").disabled = true; $("sim-run").textContent = "Simulating…";
  try {
    simResult = await api("sim3d/run", simRequest());
    simCol = Object.fromEntries(simResult.columns.map((c, i) => [c, i]));
    $("sim-empty").hidden = true; $("sim-results").hidden = false;
    renderSimSummary(); drawSimCharts();
    const r = simResult.rows;
    play.t = Math.max(r[0][0], simResult.ignition_s - 1.5); play.playing = true; play.last = performance.now();
    cam.dist = cam.follow ? 2.5 : Math.max(30, Math.min(400, simResult.summary.apogee_m * 0.9));
    $("sim-slider").min = r[0][0]; $("sim-slider").max = r[r.length - 1][0]; $("sim-slider").step = 0.02;
  } finally { $("sim-run").disabled = false; $("sim-run").textContent = "Run flight ↗"; }
}
action("sim-run", runSimFlight);
action("sim-batch", async () => {
  $("sim-batch").disabled = true; $("sim-batch").textContent = "Flying 40…";
  try { renderBatch(await api("sim3d/batch", {...simRequest(), count: 40})); }
  finally { $("sim-batch").disabled = false; $("sim-batch").textContent = "40 random flights"; }
});
action("sim-reset", async () => { sim = (await api("sim3d/defaults", {})).sim; renderSimInputs(); simChanged(); });

function renderSimSummary() {
  const s = simResult.summary, ig = simResult.ignition_s;
  const tile = (label, value, cls = "") => el("div", {class: cls}, el("span", {text: label}), el("strong", {text: value}));
  $("sim-metrics").replaceChildren(
    tile("APOGEE", `${s.apogee_m.toFixed(0)} m`), tile("TIME TO APOGEE", `${s.apogee_t.toFixed(1)} s`),
    tile("MAX SPEED", `${s.max_speed.toFixed(0)} m/s`), tile("WORST BOOST TILT", `${s.max_boost_tilt.toFixed(1)}°`, s.tvc_fault ? "bad" : ""),
    tile("PARACHUTE", s.chute_t < 0 ? "never" : `${s.chute_t.toFixed(1)} s · ${s.chute_speed.toFixed(0)} m/s`, s.chute_t < 0 ? "bad" : ""),
    tile("LANDING", s.landing_speed ? `${s.landing_speed.toFixed(1)} m/s` : "—", s.landing_speed > 9 ? "bad" : ""),
    tile("DRIFT", `${s.drift_m.toFixed(0)} m`), tile("ARMED AFTER", s.armed_t < 0 ? "never" : `${s.armed_t.toFixed(1)} s`, s.armed_t < 0 ? "bad" : ""));
  const m = simResult.model;
  $("sim-model").replaceChildren(tile("ROCKET", `${m.length_mm.toFixed(0)} mm · ${m.mass_wet_g.toFixed(0)} g`),
    tile("CG / CP", `${m.cg_wet_mm.toFixed(1)} / ${m.cp_mm.toFixed(1)} mm`), tile("STABILITY", `${m.stability_cal.toFixed(2)} cal`),
    tile("PIVOT → CG", `${m.gimbal_arm_mm.toFixed(1)} mm`), tile("GIMBAL @ ±250 µs", `${(Math.asin(Math.min(1, m.linkage_ratio * Math.sin(250 * m.servo_deg_per_us * Math.PI / 180))) * 180 / Math.PI).toFixed(2)}°`));
  $("sim-verdict").replaceChildren(...simResult.verdict.map(v => el("div", {class: `check-line ${v.level}`, text: v.text})),
    el("div", {class: "check-line info", text: `Ignition at ${ig.toFixed(1)} s (after the ${(cfg.ARM_HOLD_MS / 1000).toFixed(1)} s arming hold). Times above are from ignition. This is a model: it does not include servo backlash, flex, or real aerodynamics, so prove the hardware on the bench.`}));
}
function simEvents() {
  const r = simResult.rows, out = []; let prev = -1;
  r.forEach(row => { const st = row[simCol.state]; if (st !== prev) { out.push({t: row[0], event: SIM_STATES[st] || st}); prev = st; } });
  const chute = r.find(row => row[simCol.chute]); if (chute) out.push({t: chute[0], event: "CHUTE"});
  return out.sort((a, b) => a.t - b.t);
}
function drawSimCharts() {
  if (!simResult) return;
  const ig = simResult.ignition_s;
  const rows = simResult.rows.filter(r => r[0] >= ig - 1);
  const t = rows.map(r => r[0]), c = name => rows.map(r => r[simCol[name]]);
  const charts = [["Altitude (m) · speed (m/s)", [c("z"), c("speed")], ["altitude", "speed"]],
    ["Tilt (°): true vs the board's estimate", [c("tilt_x"), c("tilt_y"), c("est_tilt_x")], ["X true", "Y true", "X estimate"]],
    ["Gimbal (°)", [c("gimbal_a"), c("gimbal_b")], ["A", "B"]]];
  const wrap = $("sim-charts"); wrap.replaceChildren();
  const events = simEvents();
  charts.forEach(([title, series, names]) => {
    const canvas = el("canvas", {class: "log-chart", role: "img", "aria-label": title});
    wrap.append(el("article", {class: "card chart-card"}, el("div", {class: "card-heading"}, el("h2", {text: title}),
      el("div", {class: "legend"}, ...names.map((n, i) => el("span", {style: `color:${COLORS[i]}`, text: `● ${n}`})))), canvas));
    requestAnimationFrame(() => lineChart(canvas, t, series, events));
  });
}
function renderBatch(result) {
  const box = $("sim-batch-result"); box.hidden = false;
  const pct = Math.round(100 * result.success / result.count);
  const failures = result.runs.filter(r => !r.success);
  const canvas = el("canvas", {class: "log-chart", role: "img", "aria-label": "Wind versus tilt"});
  box.replaceChildren(el("div", {class: "card-heading"}, el("div", {}, el("span", {class: "eyebrow", text: "MONTE CARLO"}),
      el("h2", {text: `${result.success} / ${result.count} flights OK (${pct}%)`})),
      el("span", {class: pct === 100 ? "chip good" : "chip", text: pct === 100 ? "robust in this envelope" : `${failures.length} failed`})),
    el("p", {class: "help", text: "Each flight randomises wind speed and direction, gusts, motor misalignment, pad lean and sensor noise around your settings. OK means TVC held, the parachute opened, and it landed under 9 m/s."}),
    canvas,
    el("p", {class: "help", text: failures.length ? `Failures: ${failures.slice(0, 6).map(f => `wind ${f.wind} m/s, misalign ${f.misalign}°, lean ${f.pad_tilt}° → ${f.fault ? "TVC gave up" : `landed ${f.landing.toFixed(1)} m/s`}`).join("; ")}` : "No failures."}));
  requestAnimationFrame(() => scatter(canvas, result.runs));
}
function scatter(canvas, runs) {
  const rect = canvas.getBoundingClientRect(); if (!rect.width) return;
  const k = window.devicePixelRatio || 1; canvas.width = rect.width * k; canvas.height = rect.height * k;
  const ctx = canvas.getContext("2d"); ctx.scale(k, k);
  const left = 46, top = 10, w = rect.width - 60, h = rect.height - 34;
  const maxW = Math.max(1, ...runs.map(r => r.gust)), maxT = Math.max(10, ...runs.map(r => r.tilt));
  ctx.strokeStyle = "#ebebef"; ctx.fillStyle = "#90909b"; ctx.font = "10px -apple-system, sans-serif";
  for (let i = 0; i <= 4; i++) { const y = top + i * h / 4; ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(left + w, y); ctx.stroke(); ctx.fillText(`${(maxT - i * maxT / 4).toFixed(0)}°`, 6, y + 3); }
  ctx.fillText("worst boost tilt", left + 4, top + 10); ctx.fillText("gust speed (m/s) →", left + w - 100, top + h + 22);
  for (let i = 0; i <= 4; i++) ctx.fillText((i * maxW / 4).toFixed(1), left + i * w / 4 - 6, top + h + 12);
  runs.forEach(r => { ctx.fillStyle = r.success ? "#288471" : "#c0392b"; ctx.beginPath();
    ctx.arc(left + r.gust / maxW * w, top + h - r.tilt / maxT * h, 4, 0, 2 * Math.PI); ctx.fill(); });
}

/* ------------------------------------------------------------------ 3D view */
function rowAt(t) {
  const r = simResult.rows; let lo = 0, hi = r.length - 1;
  while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (r[mid][0] <= t) lo = mid; else hi = mid; }
  return r[lo];
}
function qrot(q, v) {   /* body -> world */
  const [w, x, y, z] = q, [vx, vy, vz] = v;
  const tx = 2 * (y * vz - z * vy), ty = 2 * (z * vx - x * vz), tz = 2 * (x * vy - y * vx);
  return [vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx)];
}
const add3 = (a, b, s = 1) => [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s];
const sub3 = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross3 = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const unit3 = a => { const n = Math.hypot(...a) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };

function drawScene() {
  const canvas = $("sim-canvas"), rect = canvas.getBoundingClientRect();
  if (!rect.width || $("sim3d").hidden) return;
  const k = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(rect.width * k)) { canvas.width = rect.width * k; canvas.height = rect.height * k; }
  const ctx = canvas.getContext("2d"); ctx.setTransform(k, 0, 0, k, 0, 0);
  const W = rect.width, H = rect.height;
  const sky = ctx.createLinearGradient(0, 0, 0, H); sky.addColorStop(0, "#eef0fb"); sky.addColorStop(1, "#f9f8f4");
  ctx.fillStyle = sky; ctx.fillRect(0, 0, W, H);
  if (!simResult) return;

  const row = rowAt(play.t), C = simCol;
  const pos = [row[C.x], row[C.y], row[C.z]], q = [row[C.qw], row[C.qx], row[C.qy], row[C.qz]];
  const rows = simResult.rows;
  const xs = rows.map(r => r[C.x]), ys = rows.map(r => r[C.y]), zs = rows.map(r => r[C.z]);
  const center = [(Math.min(...xs) + Math.max(...xs)) / 2, (Math.min(...ys) + Math.max(...ys)) / 2, Math.max(...zs) / 2];
  const target = cam.follow ? pos : center;
  const eye = add3(target, [Math.cos(cam.pitch) * Math.cos(cam.yaw), Math.cos(cam.pitch) * Math.sin(cam.yaw), Math.sin(cam.pitch)], cam.dist);
  const f = unit3(sub3(target, eye)), r = unit3(cross3(f, [0, 0, 1])), u = cross3(r, f);
  const F = H / (2 * Math.tan(0.45));
  const project = p => { const d = sub3(p, eye), z = dot3(d, f); if (z < 0.05) return null;
    return [W / 2 + dot3(d, r) / z * F, H / 2 - dot3(d, u) / z * F, z]; };
  const line = (a, b, color, width = 1) => { const A = project(a), B = project(b); if (!A || !B) return;
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath(); ctx.moveTo(A[0], A[1]); ctx.lineTo(B[0], B[1]); ctx.stroke(); };

  /* ground grid around pad and landing point */
  const span = Math.max(40, Math.ceil(Math.max(...xs.map(Math.abs), ...ys.map(Math.abs), cam.dist * 0.6) / 10) * 10 + 20);
  const step = span > 200 ? 50 : span > 80 ? 20 : 10;
  for (let g = -span; g <= span; g += step) {
    line([g, -span, 0], [g, span, 0], g === 0 ? "#cfcbe0" : "#e3e1ea");
    line([-span, g, 0], [span, g, 0], g === 0 ? "#cfcbe0" : "#e3e1ea");
  }
  /* pad and wind arrow */
  line([0, 0, 0], [0, 0, 1.2], "#272735", 3);
  const wd = (sim.wind_dir_deg || 0) * Math.PI / 180, wl = Math.min(span * 0.3, 4 + sim.wind_m_s * 3);
  const w0 = [-span * 0.7, -span * 0.7, 0.1], w1 = add3(w0, [Math.cos(wd), Math.sin(wd), 0], wl);
  line(w0, w1, "#5f9bd4", 2); const tip = project(w1); if (tip) { ctx.fillStyle = "#5f9bd4"; ctx.fillText(`wind ${sim.wind_m_s} m/s`, tip[0] + 4, tip[1]); }

  /* trajectory: flown part solid, rest faint */
  for (let i = 1; i < rows.length; i++) {
    const a = rows[i - 1], b = rows[i];
    if (b[C.z] <= 0 && a[C.z] <= 0) continue;
    const done = b[0] <= play.t;
    line([a[C.x], a[C.y], a[C.z]], [b[C.x], b[C.y], b[C.z]], done ? STATE_COLORS[b[C.state]] || "#999" : "#d6d3e3", done ? 2 : 1);
  }
  /* shadow + drop line */
  line([pos[0], pos[1], 0], pos, "#c9c4dc", 1);
  const sh = project([pos[0], pos[1], 0]); if (sh) { ctx.fillStyle = "#0002"; ctx.beginPath(); ctx.ellipse(sh[0], sh[1], 6, 3, 0, 0, 2 * Math.PI); ctx.fill(); }

  /* rocket: true geometry from the rocket model (mm), enlarged only when the camera is far away */
  const model = simResult.model, L = model.length_mm / 1000;
  const scale = cam.trueScale ? 1 : Math.max(1, cam.dist * 0.05 / L);
  const ga = row[C.gimbal_a] * Math.PI / 180, gb = row[C.gimbal_b] * Math.PI / 180;
  const toWorld = v => add3(pos, qrot(q, [v[0] * scale, v[1] * scale, v[2] * scale]));
  const light = unit3([0.4, -0.6, 0.7]);
  const faces = [];
  for (const part of rocketMesh(model, ga, gb)) {
    const w = part.pts.map(toWorld), P = w.map(project);
    if (P.some(p => !p)) continue;
    const n = unit3(cross3(sub3(w[1], w[0]), sub3(w[2], w[0])));
    const shade = 0.55 + 0.45 * Math.abs(dot3(n, light));
    faces.push({P, depth: P.reduce((a, p) => a + p[2], 0) / P.length, color: part.color, shade});
  }
  faces.sort((a, b) => b.depth - a.depth);
  for (const fc of faces) {
    const [r0, g0, b0] = fc.color;
    ctx.fillStyle = `rgb(${r0 * fc.shade | 0},${g0 * fc.shade | 0},${b0 * fc.shade | 0})`;
    ctx.beginPath(); ctx.moveTo(fc.P[0][0], fc.P[0][1]); fc.P.slice(1).forEach(p => ctx.lineTo(p[0], p[1])); ctx.closePath(); ctx.fill();
  }
  const zN = (model.cg_wet_mm - model.nozzle_mm) / 1000, zP = (model.cg_wet_mm - model.pivot_mm) / 1000;
  const thrust = row[C.thrust];
  if (thrust > 0.05) {
    const axis = gimbalAxis(ga, gb);
    const nozzle = toWorld(add3([0, 0, zP], axis, zN - zP));
    const dirW = qrot(q, axis);
    const plumeLen = (0.15 + thrust / 40) * scale;
    line(nozzle, add3(nozzle, dirW, -plumeLen), "#f6a13a", Math.max(3, 10 / (1 + cam.dist / 6)));
    line(nozzle, add3(nozzle, dirW, -plumeLen * 0.45), "#ffe08a", 3);
  }
  const tipP = toWorld([0, 0, model.cg_wet_mm / 1000]);
  if (row[C.chute]) {
    const top = add3(tipP, [0, 0, 1], L * scale * 1.6), rad = (vehicle ? vehicle.chute_diameter_cm / 200 : 0.25) * scale;
    const T = project(top);
    if (T) { const pr = rad / T[2] * F; ctx.fillStyle = "#e9542ccc"; ctx.beginPath(); ctx.ellipse(T[0], T[1], pr, pr * 0.55, 0, Math.PI, 0); ctx.fill();
      const Pp = project(tipP); if (Pp) { ctx.strokeStyle = "#88889a"; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(T[0] - pr, T[1]); ctx.lineTo(Pp[0], Pp[1]); ctx.lineTo(T[0] + pr, T[1]); ctx.stroke(); } }
  }
  /* metric scale bar at the rocket's distance */
  const here = project(pos);
  if (here) { const px = F / here[2] * scale; const bar = px > 40 ? 0.1 : px > 4 ? 1 : 10;
    ctx.strokeStyle = "#555"; ctx.fillStyle = "#555"; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(W - 30 - bar * px, H - 26); ctx.lineTo(W - 30, H - 26); ctx.stroke();
    ctx.font = "10px -apple-system, sans-serif"; ctx.fillText(bar < 1 ? "10 cm" : `${bar} m`, W - 30 - bar * px, H - 32); }

  /* HUD */
  const ig = simResult.ignition_s, st = SIM_STATES[row[C.state]] || "?";
  ctx.fillStyle = "#ffffffd9"; ctx.fillRect(12, 12, 235, 132); ctx.strokeStyle = "#e3e1ea"; ctx.strokeRect(12, 12, 235, 132);
  ctx.fillStyle = STATE_COLORS[row[C.state]] || "#333"; ctx.font = "600 15px -apple-system, sans-serif";
  ctx.fillText(row[C.chute] && st !== "LANDED" ? `${st} · CHUTE` : st, 24, 36);
  ctx.fillStyle = "#272735"; ctx.font = "12px -apple-system, sans-serif";
  const tt = row[0] - ig;
  [[`T${tt < 0 ? "−" : "+"}${Math.abs(tt).toFixed(2)} s`, `${row[C.armed] ? "armed" : "not armed"}`],
   [`alt ${row[C.z].toFixed(1)} m`, `speed ${row[C.speed].toFixed(1)} m/s`],
   [`tilt X ${row[C.tilt_x].toFixed(1)}°`, `Y ${row[C.tilt_y].toFixed(1)}°`],
   [`gimbal A ${row[C.gimbal_a].toFixed(2)}°`, `B ${row[C.gimbal_b].toFixed(2)}°`],
   [`thrust ${row[C.thrust].toFixed(1)} N`, row[C.tvc] ? "TVC active" : "TVC off"]]
    .forEach(([a, b], i) => { ctx.fillText(a, 24, 58 + i * 18); ctx.fillText(b, 140, 58 + i * 18); });
  ctx.fillStyle = "#90909b"; ctx.font = "10px -apple-system, sans-serif";
  ctx.fillText(`drag to orbit · scroll to zoom · rocket ${scale > 1.05 ? `enlarged ${scale.toFixed(0)}×` : "true scale"} · motor gimbal shown ×4`, 12, H - 10);

  $("sim-slider").value = play.t;
  $("sim-time").textContent = `T${tt < 0 ? "−" : "+"}${Math.abs(tt).toFixed(1)} s`;
}
function tick(now) {
  if (simResult && play.playing && !$("sim3d").hidden) {
    const rows = simResult.rows, end = rows[rows.length - 1][0];
    play.t = Math.min(end, play.t + (now - play.last) / 1000 * play.speed);
    if (play.t >= end) play.playing = false;
  }
  $("sim-play").textContent = play.playing ? "Pause" : "Play";
  play.last = now;
  drawScene();
  requestAnimationFrame(tick);
}
$("sim-play").onclick = () => {
  if (!simResult) return;
  const rows = simResult.rows;
  if (!play.playing && play.t >= rows[rows.length - 1][0]) play.t = Math.max(rows[0][0], simResult.ignition_s - 1.5);
  play.playing = !play.playing; $("sim-play").textContent = play.playing ? "Pause" : "Play";
};
$("sim-speed").onchange = () => { play.speed = Number($("sim-speed").value); };
$("sim-slider").oninput = () => { play.t = Number($("sim-slider").value); play.playing = false; $("sim-play").textContent = "Play"; };
$("sim-follow").onchange = () => { cam.follow = $("sim-follow").checked; if (cam.follow) cam.dist = Math.min(cam.dist, 3); };
$("sim-true-scale").onchange = () => { cam.trueScale = $("sim-true-scale").checked; };
(() => {
  const canvas = $("sim-canvas"); let drag = null;
  canvas.addEventListener("pointerdown", e => { drag = [e.clientX, e.clientY, cam.yaw, cam.pitch]; canvas.setPointerCapture(e.pointerId); });
  canvas.addEventListener("pointermove", e => { if (!drag) return;
    cam.yaw = drag[2] - (e.clientX - drag[0]) * 0.008; cam.pitch = Math.max(0.03, Math.min(1.5, drag[3] + (e.clientY - drag[1]) * 0.006)); });
  canvas.addEventListener("pointerup", () => { drag = null; });
  canvas.addEventListener("wheel", e => { e.preventDefault(); cam.dist = Math.max(3, Math.min(2000, cam.dist * Math.exp(e.deltaY * 0.0012))); }, {passive: false});
})();

/* ------------------------------------------------------------------ start-up */
const previousTabHook = window.onTabShown;
window.onTabShown = id => {
  if (previousTabHook) previousTabHook(id);
  if (id === "sim3d") drawSimCharts();
};
(async () => {
  await window.studioReady;
  try {
    const stored = (await api("rocket/load", {})).state || {};
    const defaults = (await api("sim3d/defaults", {})).sim;
    sim = {...defaults, ...(stored.sim || {})};
    for (const key of Object.keys(sim)) if (!(key in defaults)) delete sim[key];
  } catch (error) { notice(error.message); return; }
  renderSimInputs();
  requestAnimationFrame(tick);
})();

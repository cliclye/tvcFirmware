"use strict";
/* TVC gimbal page: the two servos, horns, pushrods and gimbal rings, in millimetres, driven by
 * manual sliders, the power-up self-test sweep, a 3D-sim flight, or the live board (bench image).
 * Kinematics match flight_sim3d.c: horn = (pulse - centre) * deg/us * direction,
 * gimbal = asin(horn_mm / lever_mm * sin(horn)), clamped at the mechanical stop.
 * Uses el, api, action, notice, cfg, vehicle, vehicleChanged, rocketModel, liveStatus,
 * simResult, simCol, play and the vector helpers from the other scripts. */

const SERVO_BOX = [22.8, 12.2, 22.5];   /* MG90D body, mm (without the mounting tabs) */
let gim = {source: "sim", pulse: [1500, 1500], sweep: null, yaw: -0.65, pitch: 0.42, dist: 230, liveTimer: null};

function gimbalGeom() {
  const g = vehicle.gimbal, mo = vehicle.motor, b = vehicle.body;
  return {horn: g.horn_mm, lever: g.lever_mm, dpu: g.servo_deg_per_us, speed: 60 / g.servo_s_per_60, stop: g.stop_deg,
    ratio: g.horn_mm / g.lever_mm, motorR: mo.diameter_mm / 2, motorTop: mo.length_mm - b.pivot_from_nozzle_mm,
    motorBottom: -b.pivot_from_nozzle_mm, bodyR: vehicle.diameter_mm / 2};
}
function servoInfo(i, pulse) {
  const s = ["A", "B"][i], G = gimbalGeom();
  const center = cfg[`SERVO_${s}_CENTER_US`], dir = cfg[`SERVO_${s}_DIRECTION`];
  const horn = (pulse - center) * G.dpu * dir;
  const raw = Math.asin(Math.max(-1, Math.min(1, G.ratio * Math.sin(horn * Math.PI / 180)))) * 180 / Math.PI;
  return {pulse, horn, gimbal: Math.max(-G.stop, Math.min(G.stop, raw)), atStop: Math.abs(raw) >= G.stop - 1e-6};
}

/* ------------------------------------------------------------------ inputs and readouts */
function gField(key, label, step) {
  const input = el("input", {type: "number", step, "aria-label": label});
  input.value = vehicle.gimbal[key];
  input.oninput = () => { vehicle.gimbal[key] = Number(input.value); vehicleChanged(); renderGimbalTable(); };
  return el("label", {class: "field"}, el("span", {text: label}), input);
}
function renderGimbalInputs() {
  if (!vehicle || !cfg) return;
  $("gim-geometry").replaceChildren(gField("horn_mm", "Servo horn radius · mm (shaft → pushrod hole)", .5),
    gField("lever_mm", "Gimbal lever radius · mm (pivot axis → pushrod hole)", .5),
    gField("servo_deg_per_us", "Servo travel · °/µs (MG90D ≈ 0.09)", .005),
    gField("servo_s_per_60", "Servo speed · s per 60° (MG90D ≈ 0.10)", .01),
    gField("stop_deg", "Gimbal mechanical stop · °", .5));
  const sliders = $("gim-sliders"); sliders.replaceChildren();
  ["A", "B"].forEach((s, i) => {
    const slider = el("input", {type: "range", min: cfg[`SERVO_${s}_MIN_US`], max: cfg[`SERVO_${s}_MAX_US`], step: 1, "aria-label": `Servo ${s} pulse`});
    slider.value = gim.pulse[i];
    slider.oninput = () => { setGimbalSource("manual"); gim.sweep = null; gim.pulse[i] = Number(slider.value); };
    sliders.append(el("label", {class: "field"}, el("span", {text: `Servo ${s} · header ${cfg[`SERVO_${s}_CHANNEL`]} · ${cfg[`SERVO_${s}_MIN_US`]}–${cfg[`SERVO_${s}_MAX_US`]} µs`}), slider));
  });
  renderGimbalTable();
}
function renderGimbalTable() {
  if (!vehicle || !cfg) return;
  const G = gimbalGeom(), rows = [];
  for (const [i, s] of [[0, "A"], [1, "B"]]) {
    const lo = servoInfo(i, cfg[`SERVO_${s}_MIN_US`]), hi = servoInfo(i, cfg[`SERVO_${s}_MAX_US`]);
    rows.push(el("tr", {}, el("td", {text: `Servo ${s}`}),
      el("td", {text: `${lo.horn.toFixed(1)}° / ${hi.horn.toFixed(1)}°`}),
      el("td", {class: lo.atStop || hi.atStop ? "bad" : "", text: `${lo.gimbal.toFixed(2)}° / ${hi.gimbal.toFixed(2)}°${lo.atStop || hi.atStop ? "  hits stop" : ""}`}),
      el("td", {text: `${(Math.abs(hi.horn - lo.horn) / G.speed * 1000).toFixed(0)} ms`})));
  }
  $("gim-table").replaceChildren(el("thead", {}, el("tr", {}, ...["", "Horn at MIN / MAX", "Gimbal at MIN / MAX", "Full sweep time"].map(t => el("th", {text: t})))),
    el("tbody", {}, ...rows));
  const linear = G.ratio * 250 * G.dpu, real = servoInfo(0, cfg.SERVO_A_CENTER_US + 250).gimbal;
  $("gim-note").textContent = `Linkage ratio ${G.ratio.toFixed(3)} (horn ${G.horn} mm ÷ lever ${G.lever} mm). The linkage is slightly non-linear: 250 µs gives ${Math.abs(real).toFixed(2)}° of gimbal, against ${Math.abs(linear).toFixed(2)}° if it were linear. The 3D sim uses exactly this geometry. Measure the real horn and lever holes with calipers.`;
}

/* ------------------------------------------------------------------ sources */
function setGimbalSource(source) {
  gim.source = source; $("gim-source").value = source;
  $("gim-manual").hidden = source !== "manual";
  if (source !== "sim") play.playing = false;   /* the flight clock only drives "flight playback" */
  if (source === "live") pollGimbalLive();
}
$("gim-source").onchange = () => { gim.sweep = null; setGimbalSource($("gim-source").value); };
$("gim-sweep").onclick = () => {
  /* the firmware's power-up self test: A centre->max->min->centre, then B, 300 ms each step */
  setGimbalSource("manual");
  const steps = [];
  for (const [i, s] of [[0, "A"], [1, "B"]])
    for (const k of ["CENTER", "MAX", "MIN", "CENTER"]) steps.push([i, cfg[`SERVO_${s}_${k}_US`]]);
  gim.sweep = {steps, start: performance.now()};
};
$("gim-center").onclick = () => { gim.sweep = null; gim.pulse = [cfg.SERVO_A_CENTER_US, cfg.SERVO_B_CENTER_US]; renderGimbalInputs(); };
async function pollGimbalLive() {
  clearTimeout(gim.liveTimer);
  if (gim.source !== "live" || $("sim3d").hidden) return;
  try { liveStatus = await api("monitor/status", {}); } catch (error) { notice(error.message); return; }
  gim.liveTimer = setTimeout(pollGimbalLive, 150);
}
function currentState(now) {
  /* returns per servo {pulse, horn, gimbal, atStop}, plus a label */
  if (gim.source === "sim" && !simResult) {
    return {servos: [servoInfo(0, cfg.SERVO_A_CENTER_US), servoInfo(1, cfg.SERVO_B_CENTER_US)], label: "Flight playback: press Play to fly"};
  }
  if (gim.source === "sim" && simResult) {
    const row = rowAt(play.t);
    const make = (i, s) => { const info = servoInfo(i, row[simCol[`pulse_${s}`]]);
      return {...info, horn: row[simCol[`horn_${s}`]], gimbal: row[simCol[`gimbal_${s}`]]}; };
    const tt = row[0] - simResult.ignition_s;
    return {servos: [make(0, "a"), make(1, "b")], label: `3D sim · T${tt < 0 ? "−" : "+"}${Math.abs(tt).toFixed(2)} s · ${SIM_STATES[row[simCol.state]]}`};
  }
  if (gim.source === "live") {
    const bench = liveStatus && liveStatus.latest && liveStatus.latest.bench;
    if (!bench) return {servos: [servoInfo(0, cfg.SERVO_A_CENTER_US), servoInfo(1, cfg.SERVO_B_CENTER_US)], label: "Live board: no bench data (flash the bench image, connect the USB-UART on Live board)"};
    return {servos: [servoInfo(0, bench.pulse_a), servoInfo(1, bench.pulse_b)], label: `Live board · tilt X ${bench.tilt_x.toFixed(1)}° Y ${bench.tilt_y.toFixed(1)}°`};
  }
  if (gim.sweep) {
    const k = Math.floor((now - gim.sweep.start) / 300);
    if (k >= gim.sweep.steps.length) gim.sweep = null;
    else { const [i, us] = gim.sweep.steps[k]; gim.pulse = [cfg.SERVO_A_CENTER_US, cfg.SERVO_B_CENTER_US]; gim.pulse[i] = us; }
  }
  return {servos: [servoInfo(0, gim.pulse[0]), servoInfo(1, gim.pulse[1])], label: gim.sweep ? "Power-up self test (as the firmware does it)" : "Manual"};
}

/* ------------------------------------------------------------------ 3D mechanism view (mm, origin at the gimbal pivot) */
const rotX = (v, a) => [v[0], v[1] * Math.cos(a) - v[2] * Math.sin(a), v[1] * Math.sin(a) + v[2] * Math.cos(a)];
const rotY = (v, a) => [v[0] * Math.cos(a) + v[2] * Math.sin(a), v[1], -v[0] * Math.sin(a) + v[2] * Math.cos(a)];
function drawGimbal(now) {
  const canvas = $("gim-canvas"); if (!canvas || $("sim3d").hidden || !vehicle || !cfg) return;
  const rect = canvas.getBoundingClientRect(); if (!rect.width) return;
  const k = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(rect.width * k)) { canvas.width = rect.width * k; canvas.height = rect.height * k; }
  const ctx = canvas.getContext("2d"); ctx.setTransform(k, 0, 0, k, 0, 0);
  const W = rect.width, H = rect.height;
  ctx.fillStyle = "#f7f6fb"; ctx.fillRect(0, 0, W, H);
  const G = gimbalGeom(), state = currentState(now), [sa, sb] = state.servos;
  const ga = sa.gimbal * Math.PI / 180, gb = sb.gimbal * Math.PI / 180;
  /* ring hierarchy: outer ring turns about body X (servo A), inner ring about the outer ring's Y (servo B).
   * Motor axis = Rx(-A) Ry(-B) z = (-sin B, sin A cos B, cos A cos B): the simulator's thrust direction. */
  const outer = v => rotX(v, -ga), inner = v => outer(rotY(v, -gb));
  const target = [0, 0, (G.motorTop + G.motorBottom) / 2 * 0.4];
  const eye = add3(target, [Math.cos(gim.pitch) * Math.cos(gim.yaw), Math.cos(gim.pitch) * Math.sin(gim.yaw), Math.sin(gim.pitch)], gim.dist);
  const f = unit3(sub3(target, eye)), r = unit3(cross3(f, [0, 0, 1])), u = cross3(r, f), F = H / (2 * Math.tan(0.4));
  const P = p => { const d = sub3(p, eye), z = dot3(d, f); return z < 1 ? null : [W / 2 + dot3(d, r) / z * F, H / 2 - dot3(d, u) / z * F, z]; };
  const faces = [];
  const light = unit3([0.3, -0.5, 0.8]);
  const quad = (pts, color) => { const pp = pts.map(P); if (pp.some(p => !p)) return;
    const n = unit3(cross3(sub3(pts[1], pts[0]), sub3(pts[2], pts[0])));
    faces.push({pp, depth: pp.reduce((a, p) => a + p[2], 0) / pp.length, color, shade: 0.55 + 0.45 * Math.abs(dot3(n, light))}); };
  const lines = [];
  const seg = (a, b, color, width) => { const A = P(a), B = P(b); if (A && B) lines.push({A, B, color, width, depth: (A[2] + B[2]) / 2}); };
  const cylinder = (r0, z0, z1, n, color, xf) => { for (let s = 0; s < n; s++) { const a0 = 2 * Math.PI * s / n, a1 = 2 * Math.PI * (s + 1) / n;
    quad([[r0 * Math.cos(a0), r0 * Math.sin(a0), z0], [r0 * Math.cos(a0), r0 * Math.sin(a0), z1], [r0 * Math.cos(a1), r0 * Math.sin(a1), z1], [r0 * Math.cos(a1), r0 * Math.sin(a1), z0]].map(xf), color); } };
  const ring = (radius, halfH, color, xf) => cylinder(radius, -halfH, halfH, 28, color, xf);
  const box = (c, size, color) => { const [x, y, z] = c, [a, b, h] = size.map(v => v / 2);
    const v = [[x - a, y - b, z - h], [x + a, y - b, z - h], [x + a, y + b, z - h], [x - a, y + b, z - h], [x - a, y - b, z + h], [x + a, y - b, z + h], [x + a, y + b, z + h], [x - a, y + b, z + h]];
    [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]].forEach(q => quad(q.map(i => v[i]), color)); };

  /* airframe section around the mount (fixed), drawn as a translucent wire */
  for (let s = 0; s < 36; s++) { const a0 = 2 * Math.PI * s / 36, a1 = 2 * Math.PI * (s + 1) / 36;
    for (const z of [G.motorBottom, G.motorTop * 0.6]) seg([G.bodyR * Math.cos(a0), G.bodyR * Math.sin(a0), z], [G.bodyR * Math.cos(a1), G.bodyR * Math.sin(a1), z], "#cfcbe0", 1); }
  /* motor, inner ring (both axes), outer ring (servo A only) */
  cylinder(G.motorR, G.motorBottom, G.motorTop, 20, [125, 125, 138, 0.45], inner);   /* see-through so both linkages stay visible */
  cylinder(G.motorR * 0.55, G.motorBottom - 6, G.motorBottom, 14, [70, 70, 80], inner);
  ring(G.motorR + 3, 4, [232, 120, 52], inner);
  ring(G.motorR + 10, 4, [106, 76, 229], outer);
  /* pivot pins: outer ring on the airframe along X, inner ring on the outer ring along Y */
  seg([-(G.motorR + 18), 0, 0], [G.motorR + 18, 0, 0], "#272735", 3);
  seg(outer([0, -(G.motorR + 12), 0]), outer([0, G.motorR + 12, 0]), "#272735", 3);

  /* servos sit beside the motor, above the pivot; A drives the outer ring through a lever along +Y,
   * B the inner ring through a lever along +X; pushrods run down to the lever holes */
  const drop = Math.min(G.motorTop - 6, 34);
  const servo = (i, info, leverDirection, ringXf, color) => {
    const ax = leverDirection === "y" ? [0, 1, 0] : [1, 0, 0];
    const leverHole = ringXf(leverDirection === "y" ? [0, G.lever, 0] : [G.lever, 0, 0]);
    const shaft = add3([0, 0, drop], ax, G.lever - G.horn);
    /* the horn tip must move the same way as the lever hole: A's hole drops for +A, B's rises for +B */
    const h = info.horn * Math.PI / 180;
    const hornTip = add3(add3(shaft, ax, G.horn * Math.cos(h)), [0, 0, 1], (i === 0 ? -1 : 1) * G.horn * Math.sin(h));
    const boxCenter = add3(add3(shaft, ax, SERVO_BOX[0] / 2 - 6), [0, 0, -SERVO_BOX[2] / 2 + 3]);
    box(boxCenter, leverDirection === "y" ? [SERVO_BOX[1], SERVO_BOX[0], SERVO_BOX[2]] : [SERVO_BOX[0], SERVO_BOX[1], SERVO_BOX[2]], color);
    seg(shaft, hornTip, "#f0f0f4", 5); seg(shaft, hornTip, "#272735", 1);
    seg(hornTip, leverHole, "#3a3a46", 2);
    const label = P(add3(boxCenter, [0, 0, -SERVO_BOX[2] / 2 - 6]));
    return {label, name: `Servo ${["A", "B"][i]}`};
  };
  const labels = [servo(0, sa, "y", outer, [70, 90, 160]), servo(1, sb, "x", inner, [60, 140, 110])];

  faces.sort((a, b) => b.depth - a.depth);
  const all = [...faces.map(x => ({kind: "f", ...x})), ...lines.map(x => ({kind: "l", ...x}))].sort((a, b) => b.depth - a.depth);
  for (const it of all) {
    if (it.kind === "f") { const [r0, g0, b0, a = 1] = it.color; ctx.fillStyle = `rgba(${r0 * it.shade | 0},${g0 * it.shade | 0},${b0 * it.shade | 0},${a})`;
      ctx.beginPath(); ctx.moveTo(it.pp[0][0], it.pp[0][1]); it.pp.slice(1).forEach(p => ctx.lineTo(p[0], p[1])); ctx.closePath(); ctx.fill(); }
    else { ctx.strokeStyle = it.color; ctx.lineWidth = it.width; ctx.beginPath(); ctx.moveTo(it.A[0], it.A[1]); ctx.lineTo(it.B[0], it.B[1]); ctx.stroke(); }
  }
  ctx.font = "600 11px -apple-system, sans-serif"; ctx.fillStyle = "#272735";
  labels.forEach(l => { if (l.label) ctx.fillText(l.name, l.label[0] - 20, l.label[1]); });

  /* scale bar: 10 mm at the pivot */
  const o = P([0, 0, 0]);
  if (o) { const px = F / o[2] * 10; ctx.strokeStyle = "#555"; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(W - 24 - px, H - 22); ctx.lineTo(W - 24, H - 22); ctx.stroke();
    ctx.fillStyle = "#555"; ctx.font = "10px -apple-system, sans-serif"; ctx.fillText("10 mm", W - 24 - px, H - 28); }
  ctx.fillStyle = "#90909b"; ctx.fillText("true scale, mm · drag to orbit · scroll to zoom", 12, H - 10);

  /* readouts */
  ctx.fillStyle = "#ffffffe0"; ctx.fillRect(12, 12, 300, 92); ctx.strokeStyle = "#e3e1ea"; ctx.lineWidth = 1; ctx.strokeRect(12, 12, 300, 92);
  ctx.fillStyle = "#272735"; ctx.font = "600 12px -apple-system, sans-serif"; ctx.fillText(state.label, 22, 32);
  ctx.font = "12px -apple-system, sans-serif";
  state.servos.forEach((sv, i) => {
    ctx.fillStyle = sv.atStop ? "#c0392b" : "#272735";
    ctx.fillText(`Servo ${["A", "B"][i]}  ${Math.round(sv.pulse)} µs   horn ${sv.horn.toFixed(1)}°   gimbal ${sv.gimbal.toFixed(2)}°${sv.atStop ? "  STOP" : ""}`, 22, 56 + i * 20);
  });
  const axis = inner([0, 0, 1]);
  ctx.fillStyle = "#90909b"; ctx.fillText(`motor axis tilt ${(Math.acos(Math.max(-1, Math.min(1, axis[2]))) * 180 / Math.PI).toFixed(2)}°`, 22, 96);
  if (G.lever <= G.motorR + 12) { ctx.fillStyle = "#c0392b"; ctx.fillText(`Lever ${G.lever} mm is inside the outer ring (≈${(G.motorR + 12).toFixed(0)} mm): make it longer`, 12, H - 40); }
}

/* ------------------------------------------------------------------ camera, start-up
 * Drawn from the 3D sim page's animation loop (one clock for both views). */
window.drawGimbal = drawGimbal;
(() => {
  const canvas = $("gim-canvas"); let drag = null;
  canvas.addEventListener("pointerdown", e => { drag = [e.clientX, e.clientY, gim.yaw, gim.pitch]; canvas.setPointerCapture(e.pointerId); });
  canvas.addEventListener("pointermove", e => { if (!drag) return;
    gim.yaw = drag[2] - (e.clientX - drag[0]) * 0.008; gim.pitch = Math.max(-1.4, Math.min(1.4, drag[3] + (e.clientY - drag[1]) * 0.006)); });
  canvas.addEventListener("pointerup", () => { drag = null; });
  canvas.addEventListener("wheel", e => { e.preventDefault(); gim.dist = Math.max(60, Math.min(800, gim.dist * Math.exp(e.deltaY * 0.0012))); }, {passive: false});
})();
const previousGimbalHook = window.onTabShown;
window.onTabShown = id => {
  if (previousGimbalHook) previousGimbalHook(id);
  if (id === "sim3d") { renderGimbalInputs(); if (gim.source === "live") pollGimbalLive(); }
};
(async () => {
  await window.studioReady;
  const wait = () => new Promise(r => setTimeout(r, 100));
  for (let i = 0; i < 100 && (!cfg || !vehicle); i++) await wait();
  if (!cfg || !vehicle) return;
  gim.pulse = [cfg.SERVO_A_CENTER_US, cfg.SERVO_B_CENTER_US];
  renderGimbalInputs();
})();

"use strict";
/* Rocket setup, build & flash, live board, launch checklist and flight logs.
 * Shares $, api, action, notice, download, textNode, grid, profile, render and showTab with app.js. */

let cfg = null, savedCfg = null, buildInfo = null, appState = {}, vehicle = null;
let liveTimer = null, liveStatus = null, logData = null, validateTimer = null, rocketModel = null, modelTimer = null;
const COLORS = ["#7655e8", "#288471", "#c07a1a"];
const AXES = ["POS_X", "NEG_X", "POS_Y", "NEG_Y", "POS_Z", "NEG_Z"];

/* ------------------------------------------------------------------ helpers */
function el(tag, attrs = {}, ...children) {
  const node = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") node.className = v; else if (k === "text") node.textContent = v;
    else if (k === "style") node.style.cssText = v;   /* CSSOM: allowed by the page's CSP, unlike a style attribute */
    else if (k.startsWith("on")) node.addEventListener(k.slice(2), v); else node.setAttribute(k, v);
  }
  node.append(...children.filter(c => c !== null && c !== undefined));
  return node;
}
function fmtTime(seconds) { return new Date(seconds * 1000).toLocaleString(); }
function same(a, b) { return JSON.stringify(a) === JSON.stringify(b); }
function confirmModal(title, body, okText = "Continue") {
  return new Promise(resolve => {
    $("modal-title").textContent = title; $("modal-body").replaceChildren(...body);
    $("modal-ok").textContent = okText; $("modal").hidden = false;
    const done = value => { $("modal").hidden = true; $("modal-ok").onclick = $("modal-cancel").onclick = null; resolve(value); };
    $("modal-ok").onclick = () => done(true); $("modal-cancel").onclick = () => done(false);
  });
}
async function saveState(update) { appState = (await api("state", update)).state; }

/* ------------------------------------------------------------------ rocket setup */
const CONTROL_FIELDS = [["KP", "P · per rad", 0, 100, .05], ["KI", "I · per rad·s", 0, 100, .01], ["KD", "D · rad/s", 0, 100, .01],
  ["INTEGRATOR_LIMIT", "Integrator limit", 0, 1, .01], ["OUTPUT_RATE_LIMIT", "Slew · travel/s", .01, 100, .5],
  ["DERIVATIVE_CUTOFF", "D filter · Hz", 0, 100, 1], ["MAX_TILT_DEG", "Give-up tilt · °", 5, 45, 1]];
const LAUNCH_FIELDS = [["LAUNCH_ACCEL_G", "Launch at · g", 1.2, 10, .1], ["LAUNCH_CONFIRM_MS", "Confirm for · ms", 10, 500, 5],
  ["TVC_ENABLE_DELAY_MS", "TVC starts after · ms", 0, 1000, 10], ["ARM_HOLD_MS", "Still for · ms to arm", 1000, 30000, 500],
  ["ARM_MAX_TILT_DEG", "Arm within · ° of vertical", 1, 30, 1], ["DISARM_TILT_DEG", "Disarm past · °", 2, 60, 1]];
const RECOVERY_FIELDS = [["CHUTE_CLOSED_US", "Closed pulse · µs", 1000, 2000, 10], ["CHUTE_OPEN_US", "Open pulse · µs", 1000, 2000, 10],
  ["CHUTE_BACKUP_MS", "Backup after launch · ms", 1000, 30000, 100]];

function cfgField(key, label, min, max, step) {
  const input = el("input", {type: "number", min, max, step, "aria-label": label});
  input.value = cfg[key];
  input.oninput = () => { cfg[key] = input.value === "" ? NaN : Number(input.value); cfgChanged(); };
  return el("label", {class: "field"}, el("span", {text: label}), input);
}
function cfgSelect(key, label, options) {
  const select = el("select", {"aria-label": label});
  options.forEach(([value, name]) => { const o = el("option", {text: name}); o.value = value; select.append(o); });
  select.value = String(cfg[key]);
  select.onchange = () => { cfg[key] = typeof cfg[key] === "number" ? Number(select.value) : select.value; cfgChanged(); };
  return el("label", {class: "field"}, el("span", {text: label}), select);
}
function renderRocket() {
  if (!cfg) return;
  const mount = $("cfg-mount"); mount.replaceChildren();
  const top = grid();
  top.append(cfgSelect("NOSE_AXIS", "Nose axis (sensor)", AXES.map(a => [a, a.replace("_", " ")])),
             cfgSelect("SERVO_RATE_HZ", "Servo frame rate", [[50, "50 Hz (any servo)"], [100, "100 Hz"], [125, "125 Hz"], [200, "200 Hz (digital)"], [250, "250 Hz (digital)"]]));
  const led = el("input", {type: "checkbox"}); led.checked = cfg.LED_ACTIVE_LOW === 1;
  led.onchange = () => { cfg.LED_ACTIVE_LOW = led.checked ? 1 : 0; cfgChanged(); };
  const bright = el("input", {type: "range", min: 10, max: 100, step: 10, "aria-label": "Status LED brightness"});
  bright.value = cfg.LED_BRIGHTNESS_PCT;
  const brightLabel = el("span", {text: `Status LED brightness · ${cfg.LED_BRIGHTNESS_PCT} %`});
  bright.oninput = () => { cfg.LED_BRIGHTNESS_PCT = Number(bright.value); brightLabel.textContent = `Status LED brightness · ${bright.value} %`; cfgChanged(); };
  mount.append(top, el("label", {class: "check"}, led, " Status LED is active-low (power-up shows red → green → blue one at a time; untick if you see the opposite)"),
    el("label", {class: "field led-brightness"}, brightLabel, bright));
  for (const s of ["A", "B"]) {
    const block = el("div", {class: "servo-block"}, el("h3", {text: `Servo ${s} · tilts about body ${s === "A" ? "X" : "Y"}`}));
    const g = grid();
    g.append(cfgSelect(`SERVO_${s}_CHANNEL`, "Header", [[1, "1 (PB0)"], [2, "2 (PB1)"], [3, "3 (PB8)"], [4, "4 (PB9)"]]),
      cfgField(`SERVO_${s}_MIN_US`, "Min · µs", 1000, 2000, 5), cfgField(`SERVO_${s}_CENTER_US`, "Centre · µs", 1000, 2000, 5),
      cfgField(`SERVO_${s}_MAX_US`, "Max · µs", 1000, 2000, 5), cfgSelect(`SERVO_${s}_DIRECTION`, "Direction", [[1, "Normal (+1)"], [-1, "Reversed (−1)"]]));
    block.append(g); mount.append(block);
  }
  $("cfg-control").replaceChildren(...CONTROL_FIELDS.map(f => cfgField(...f)));
  const launchFields = LAUNCH_FIELDS.map(f => cfgField(...f));
  const sd = el("input", {type: "checkbox"}); sd.checked = cfg.REQUIRE_SD === 1;
  sd.onchange = () => { cfg.REQUIRE_SD = sd.checked ? 1 : 0; cfgChanged(); };
  $("cfg-launch").replaceChildren(...launchFields, el("label", {class: "check"}, sd, " Refuse to arm without an SD card"));
  $("cfg-recovery").replaceChildren(cfgSelect("CHUTE_CHANNEL", "Parachute servo header", [[0, "None (motor ejection)"], [1, "1"], [2, "2"], [3, "3"], [4, "4"]]),
    ...RECOVERY_FIELDS.map(f => cfgField(...f)));
  renderVehicle();
  updateRocketStatus();
}
function cfgChanged() {
  updateRocketStatus(); renderVehicleResults();
  clearTimeout(validateTimer);
  validateTimer = setTimeout(async () => {
    try { await api("rocket/validate", {config: cfg}); setRocketBanner(null); }
    catch (error) { setRocketBanner(error.message); }
  }, 250);
}
function setRocketBanner(error) {
  const banner = $("rocket-banner");
  if (error) { banner.className = "banner bad"; banner.replaceChildren(el("strong", {text: "Not valid yet"}), el("span", {text: error})); return; }
  banner.className = "banner";
  banner.replaceChildren(el("strong", {text: "Firmware folder"}), el("span", {text: appState.project || ""}));
}
function updateRocketStatus() {
  const dirty = cfg && !same(cfg, savedCfg);
  $("rocket-dirty").hidden = !dirty; $("rocket-save").disabled = !dirty;
  const box = $("rocket-build");
  if (!buildInfo) { box.textContent = "—"; return; }
  const images = buildInfo.images;
  if (dirty) { box.textContent = "You have unsaved changes. Save, then build."; box.className = "assessment"; }
  else if (!images.flight || !images.bench) { box.textContent = "Not built yet."; box.className = "assessment"; }
  else if (images.flight.stale || images.bench.stale) { box.textContent = "Saved setup is newer than the firmware. Build again."; box.className = "assessment"; }
  else { box.textContent = `Firmware is up to date (built ${fmtTime(images.flight.built)}).`; box.className = "assessment good"; }
  updateGo();
}
async function loadRocket() {
  try {
    const result = await api("rocket/load", {});
    cfg = result.config; savedCfg = structuredClone(cfg); buildInfo = result.build; appState = result.state || {};
    appState.project = result.project;
    const upgraded = await api("rocket/model", {vehicle: appState.vehicle || {}});
    vehicle = upgraded.vehicle; rocketModel = upgraded.model;
    setRocketBanner(null); renderRocket(); syncTuningFromRocket(); renderImages(); renderChecklists();
  } catch (error) {
    $("rocket-banner").className = "banner bad";
    $("rocket-banner").replaceChildren(el("strong", {text: "Setup unavailable"}), el("span", {text: error.message}));
  }
}
action("rocket-save", async () => {
  const result = await api("rocket/save", {config: cfg});
  cfg = result.config; savedCfg = structuredClone(cfg); buildInfo = result.build;
  renderRocket(); renderImages(); syncTuningFromRocket();
  notice("Saved to Core/Inc/vehicle_config.h. Build the firmware to use it.");
});
action("rocket-revert", async () => { cfg = structuredClone(savedCfg); renderRocket(); setRocketBanner(null); });

/* Tuning lab <-> rocket setup */
function syncTuningFromRocket() {
  if (!profile || !savedCfg) return;
  profile.axes.forEach(axis => Object.assign(axis, {kp: cfg.KP, ki: cfg.KI, kd: cfg.KD, integrator_limit: cfg.INTEGRATOR_LIMIT,
    output_limit: 1, output_rate_limit: cfg.OUTPUT_RATE_LIMIT, derivative_cutoff_hz: cfg.DERIVATIVE_CUTOFF}));
  ["A", "B"].forEach((s, i) => Object.assign(profile.servos[i], {minimum_us: cfg[`SERVO_${s}_MIN_US`], center_us: cfg[`SERVO_${s}_CENTER_US`],
    maximum_us: cfg[`SERVO_${s}_MAX_US`], channel: cfg[`SERVO_${s}_CHANNEL`], direction: cfg[`SERVO_${s}_DIRECTION`]}));
  profile.maximum_tilt_deg = cfg.MAX_TILT_DEG;
  if (vehicle) { profile.plant.thrust_n = Math.min(1000, Math.max(.1, vehicle.motor.avg_thrust_n)); }
  render();
}
action("tune-to-rocket", async () => {
  if (!cfg) throw new Error("Rocket setup is not loaded.");
  const [x, y] = profile.axes;
  const differ = ["kp", "ki", "kd"].some(k => Math.abs(x[k] - y[k]) > 1e-9);
  if (differ && !await confirmModal("Axes differ", [el("p", {text: "The firmware uses one set of gains for both axes. Axis X's gains will be used."})])) return;
  Object.assign(cfg, {KP: x.kp, KI: x.ki, KD: x.kd, INTEGRATOR_LIMIT: x.integrator_limit, OUTPUT_RATE_LIMIT: x.output_rate_limit,
    DERIVATIVE_CUTOFF: x.derivative_cutoff_hz, MAX_TILT_DEG: profile.maximum_tilt_deg});
  ["A", "B"].forEach((s, i) => Object.assign(cfg, {[`SERVO_${s}_MIN_US`]: profile.servos[i].minimum_us, [`SERVO_${s}_CENTER_US`]: profile.servos[i].center_us,
    [`SERVO_${s}_MAX_US`]: profile.servos[i].maximum_us, [`SERVO_${s}_CHANNEL`]: profile.servos[i].channel, [`SERVO_${s}_DIRECTION`]: profile.servos[i].direction}));
  renderRocket(); cfgChanged(); showTab("rocket");
  notice("Copied into Rocket setup. Review it, then Save to firmware.");
});

/* ------------------------------------------------------------------ vehicle calculator */
let vehicleSaveTimer = null;
function vehicleChanged() {
  renderVehicleResults();
  clearTimeout(modelTimer);
  modelTimer = setTimeout(async () => {
    try { rocketModel = (await api("rocket/model", {vehicle})).model; $("model-stats").classList.remove("stale"); renderModel(); }
    catch (error) { $("model-stats").replaceChildren(el("div", {class: "check-line bad", text: error.message})); }
  }, 250);
  clearTimeout(vehicleSaveTimer);
  vehicleSaveTimer = setTimeout(() => saveState({vehicle}).catch(e => notice(e.message)), 600);
}
function vField(object, key, label, step = 1) {
  const input = el("input", {type: key === "name" ? "text" : "number", step, "aria-label": label});
  input.value = object[key];
  input.oninput = () => { object[key] = key === "name" ? input.value : Number(input.value); vehicleChanged(); };
  return el("label", {class: "field"}, el("span", {text: label}), input);
}
function renderVehicle() {
  if (!vehicle) return;
  const m = vehicle.motor;
  const curve = Array.isArray(m.curve) ? m.curve : null;
  const impulse = curve ? curve.reduce((acc, [t, f], i) => acc + 0.5 * (f + (i ? curve[i - 1][1] : 0)) * (t - (i ? curve[i - 1][0] : 0)), 0) : null;
  const engInput = el("input", {type: "file", accept: ".eng,.txt", hidden: ""});
  engInput.onchange = async () => {
    try { const file = engInput.files[0]; if (!file) return;
      const info = await api("motor/parse-eng", {text: await file.text()});
      Object.assign(m, {name: info.name, avg_thrust_n: info.avg_thrust_n, burn_s: info.burn_s, total_mass_g: info.total_mass_g,
        prop_mass_g: info.prop_mass_g, diameter_mm: info.diameter_mm, length_mm: info.length_mm, curve: info.curve});
      renderVehicle(); vehicleChanged(); notice(`Loaded ${info.name}: ${info.impulse_ns} N·s, ${info.curve.length} points.`);
    } catch (error) { notice(error.message); } };
  $("vehicle-curve").replaceChildren(el("span", {class: "help", text: curve ? `Thrust curve: ${curve.length} measured points, ${impulse.toFixed(1)} N·s, peak ${Math.max(...curve.map(p => p[1])).toFixed(1)} N.` : "No measured curve: the sim uses a generic shape."}),
    el("button", {class: "small", text: "Open .eng…", onclick: () => engInput.click()}), engInput);
  $("vehicle-motor").replaceChildren(vField(m, "name", "Motor"), vField(m, "avg_thrust_n", "Avg thrust · N", .5), vField(m, "burn_s", "Burn time · s", .1),
    vField(m, "total_mass_g", "Motor mass · g"), vField(m, "prop_mass_g", "Propellant · g"), vField(vehicle, "diameter_mm", "Body diameter · mm"),
    vField(vehicle, "cd", "Body drag coeff.", .05), vField(vehicle, "chute_diameter_cm", "Parachute · cm"), vField(vehicle, "chute_cd", "Chute drag coeff.", .05));
  const b = vehicle.body, shape = el("select", {"aria-label": "Nose cone shape"});
  ["ogive", "conical", "parabolic", "elliptical"].forEach(n => { const o = el("option", {text: n}); o.value = n; shape.append(o); });
  shape.value = b.nose_shape; shape.onchange = () => { b.nose_shape = shape.value; vehicleChanged(); };
  $("vehicle-body").replaceChildren(vField(vehicle, "diameter_mm", "Body outer diameter", .1), vField(b, "length_mm", "Body tube length", 1),
    vField(b, "nose_length_mm", "Nose cone length", 1), el("label", {class: "field"}, el("span", {text: "Nose cone shape"}), shape),
    vField(m, "diameter_mm", "Motor diameter", .1), vField(m, "length_mm", "Motor length", .5),
    vField(b, "motor_overhang_mm", "Motor sticks out of tube", .5), vField(b, "pivot_from_nozzle_mm", "Gimbal pivot above nozzle", .5));
  const parts = $("vehicle-parts"); parts.replaceChildren();
  vehicle.parts.forEach((part, i) => {
    const name = el("input", {type: "text", "aria-label": "Part"}); name.value = part[0];
    name.oninput = () => { part[0] = name.value; vehicleChanged(); };
    const num = (index, label, step) => { const input = el("input", {type: "number", step, min: 0, "aria-label": `${part[0]} ${label}`});
      input.value = part[index]; input.oninput = () => { part[index] = Number(input.value); vehicleChanged(); }; return input; };
    parts.append(el("div", {class: "part-row"}, name, num(1, "grams", 1), num(2, "position mm", 1), num(3, "length mm", 1),
      el("button", {class: "quiet", "aria-label": "Remove", text: "×", onclick: () => { vehicle.parts.splice(i, 1); renderVehicle(); vehicleChanged(); }})));
  });
  renderVehicleResults();
  renderModel();
}
function renderModel() {
  const m = rocketModel; if (!m) return;
  const tile = (label, value, cls = "") => el("div", {class: cls}, el("span", {text: label}), el("strong", {text: value}));
  $("model-stats").replaceChildren(
    tile("LENGTH", `${m.length_mm.toFixed(0)} mm`), tile("LIFTOFF MASS", `${m.mass_wet_g.toFixed(0)} g`),
    tile("CG (WET → DRY)", `${m.cg_wet_mm.toFixed(1)} → ${m.cg_dry_mm.toFixed(1)} mm`), tile("CP (BARROWMAN)", `${m.cp_mm.toFixed(1)} mm`),
    tile("STABILITY", `${m.stability_cal.toFixed(2)} cal`, m.stability_cal < -4.5 ? "bad" : ""),
    tile("PIVOT → CG", `${m.gimbal_arm_mm.toFixed(1)} mm`), tile("CG → CP", `${m.cp_ahead_mm.toFixed(1)} mm`),
    tile("PITCH INERTIA", `${(m.inertia_wet * 1e4).toFixed(1)} kg·cm²`));
  drawSideView();
}
function drawSideView() {
  const canvas = $("side-view"), m = rocketModel; if (!canvas || !m || !vehicle) return;
  const rect = canvas.getBoundingClientRect(); if (!rect.width) return;
  const k = window.devicePixelRatio || 1; canvas.width = rect.width * k; canvas.height = rect.height * k;
  const ctx = canvas.getContext("2d"); ctx.scale(k, k);
  const W = rect.width, H = rect.height, pad = 24, D = vehicle.diameter_mm, b = vehicle.body, mo = vehicle.motor;
  const s = Math.min((W - 2 * pad) / m.length_mm, (H - 70) / (D * 1.6)), cy = 30 + D * s * 0.8;
  const X = mm => pad + mm * s, R = D / 2 * s, Ln = b.nose_length_mm;
  const noseR = x => { const t = x / Ln;
    if (b.nose_shape === "conical") return R * t;
    if (b.nose_shape === "parabolic") return R * (2 * t - t * t);
    if (b.nose_shape === "elliptical") return R * Math.sqrt(Math.max(0, 1 - (1 - t) ** 2));
    const rho = (R * R + (Ln * s) ** 2) / (2 * R), xx = (Ln - x) * s; return Math.sqrt(Math.max(0, rho * rho - xx * xx)) + R - rho; };
  ctx.font = pal().font(10); ctx.lineWidth = 1;
  /* body outline */
  ctx.beginPath(); ctx.moveTo(X(0), cy);
  for (let x = 0; x <= Ln; x += Ln / 40) ctx.lineTo(X(x), cy - noseR(x));
  ctx.lineTo(X(Ln + b.length_mm), cy - R); ctx.lineTo(X(Ln + b.length_mm), cy + R); ctx.lineTo(X(Ln), cy + R);
  for (let x = Ln; x >= 0; x -= Ln / 40) ctx.lineTo(X(x), cy + noseR(x));
  ctx.closePath(); ctx.fillStyle = pal().fill; ctx.fill(); ctx.strokeStyle = pal().purple; ctx.stroke();
  /* parts */
  vehicle.parts.forEach((p, i) => { const x0 = X(p[2] - p[3] / 2), w = Math.max(2, p[3] * s), h = R * 1.2;
    ctx.fillStyle = `hsla(${(i * 47) % 360},55%,60%,0.35)`; ctx.fillRect(x0, cy - h / 2, w, h); });
  /* motor */
  const mr = mo.diameter_mm / 2 * s;
  ctx.fillStyle = "#8a8a99"; ctx.fillRect(X(m.motor_top_mm), cy - mr, mo.length_mm * s, 2 * mr);
  /* markers */
  const marker = (mm, color, label, below) => { const x = X(mm); ctx.strokeStyle = color; ctx.fillStyle = color; ctx.beginPath();
    ctx.moveTo(x, cy - R - 8); ctx.lineTo(x, cy + R + 8); ctx.stroke(); ctx.beginPath(); ctx.arc(x, cy, 4, 0, 2 * Math.PI); ctx.fill();
    ctx.fillText(label, x - 14, below ? cy + R + 20 : cy - R - 12); };
  marker(m.cp_mm, "#c0392b", `CP ${m.cp_mm.toFixed(0)}`, false);
  marker(m.cg_wet_mm, "#1f6fb2", `CG ${m.cg_wet_mm.toFixed(0)}`, true);
  marker(m.pivot_mm, "#e0661b", `pivot ${m.pivot_mm.toFixed(0)}`, false);
  /* scale */
  ctx.strokeStyle = pal().muted; ctx.fillStyle = pal().muted; const y = H - 14;
  ctx.beginPath(); ctx.moveTo(X(0), y); ctx.lineTo(X(m.length_mm), y); ctx.stroke();
  for (let mm = 0; mm <= m.length_mm; mm += 100) { ctx.beginPath(); ctx.moveTo(X(mm), y - 4); ctx.lineTo(X(mm), y + 2); ctx.stroke(); ctx.fillText(`${mm}`, X(mm) - 8, y - 6); }
  ctx.fillText(`${m.length_mm.toFixed(0)} mm`, X(m.length_mm) - 40, y + 12);
}
$("part-add").onclick = () => { vehicle.parts.push(["New part", 0, 300, 20]); renderVehicle(); vehicleChanged(); };
$("use-backup").onclick = () => {
  const e = estimate(); if (!e || !isFinite(e.apogee_t)) { notice("No apogee estimate yet."); return; }
  cfg.CHUTE_BACKUP_MS = Math.round((e.apogee_t + 0.5) * 10) * 100;
  renderRocket(); cfgChanged(); notice(`Parachute backup set to ${cfg.CHUTE_BACKUP_MS} ms. Save to firmware to keep it.`);
};
function estimate() {
  if (!vehicle) return null;
  const m = vehicle.motor, g = 9.80665, rho = 1.225;
  const dry = vehicle.parts.reduce((s, p) => s + (Number(p[1]) || 0), 0) / 1000;
  const liftoff = dry + m.total_mass_g / 1000, burnout = liftoff - m.prop_mass_g / 1000;
  const area = Math.PI * (vehicle.diameter_mm / 2000) ** 2;
  if (!(liftoff > 0 && m.avg_thrust_n > 0 && m.burn_s > 0 && area > 0)) return null;
  let t = 0, v = 0, h = 0, vmax = 0, burnout_v = 0; const dt = 0.002;
  while (t < 60) {
    const burning = t < m.burn_s, mass = burning ? liftoff - (m.prop_mass_g / 1000) * t / m.burn_s : burnout;
    const drag = 0.5 * rho * vehicle.cd * area * v * Math.abs(v);
    const a = ((burning ? m.avg_thrust_n : 0) - drag) / mass - g;
    if (t > 0.05 && h <= 0 && a <= 0 && v <= 0 && burning) return {liftoff, burnout, tw: m.avg_thrust_n / (liftoff * g), no_liftoff: true};
    v += a * dt; h = Math.max(0, h + v * dt); t += dt; vmax = Math.max(vmax, v);
    if (burning) burnout_v = v;
    if (!burning && v <= 0) break;
  }
  const chuteArea = Math.PI * (vehicle.chute_diameter_cm / 200) ** 2;
  const descent = Math.sqrt(2 * burnout * g / (rho * vehicle.chute_cd * chuteArea));
  return {liftoff, burnout, tw: m.avg_thrust_n / (liftoff * g), apogee_m: h, apogee_t: t, vmax, burnout_v, descent};
}
function renderVehicleResults() {
  const e = estimate(); const box = $("vehicle-results"), checks = $("vehicle-checks");
  if (!e) { box.replaceChildren(); checks.replaceChildren(); return; }
  const tile = (label, value) => el("div", {}, el("span", {text: label}), el("strong", {text: value}));
  box.replaceChildren(tile("LIFTOFF MASS", `${Math.round(e.liftoff * 1000)} g`), tile("AVG THRUST/WEIGHT", e.tw.toFixed(2)),
    tile("APOGEE (EST.)", e.no_liftoff ? "—" : `${Math.round(e.apogee_m)} m @ ${e.apogee_t.toFixed(1)} s`),
    tile("MAX SPEED", e.no_liftoff ? "—" : `${e.vmax.toFixed(0)} m/s`), tile("DESCENT", e.descent ? `${e.descent.toFixed(1)} m/s` : "—"));
  const items = [];
  const add = (level, text) => items.push(el("div", {class: `check-line ${level}`, text}));
  if (e.no_liftoff) add("bad", "Thrust is below the weight: it will not lift off.");
  else if (e.tw < 2.5) add("bad", `Thrust/weight ${e.tw.toFixed(2)} is too low for TVC (aim for 3 or more): there's little control authority at low speed. Make it lighter.`);
  else if (e.tw < 3) add("warn", `Thrust/weight ${e.tw.toFixed(2)} is marginal. Lighter is better.`);
  else add("good", `Thrust/weight ${e.tw.toFixed(2)}: OK.`);
  if (cfg && e.tw < cfg.LAUNCH_ACCEL_G + 0.3) add("warn", `Average acceleration ${e.tw.toFixed(2)} g is close to the launch threshold (${cfg.LAUNCH_ACCEL_G} g). Launch then depends on the motor's initial thrust spike. Check the thrust curve, or lower the threshold.`);
  if (e.liftoff > 0.4) add("warn", "Over 400 g on an E12 class motor.");
  if (!e.no_liftoff && cfg && cfg.CHUTE_CHANNEL) {
    const ideal = (e.apogee_t + 0.5) * 1000;
    if (Math.abs(cfg.CHUTE_BACKUP_MS - ideal) > 1000) add("warn", `Parachute backup (${cfg.CHUTE_BACKUP_MS} ms) is far from apogee + 0.5 s (≈ ${Math.round(ideal / 100) * 100} ms).`);
    else add("good", "Parachute backup matches the apogee estimate.");
  }
  if (e.descent > 8) add("bad", `Descent ${e.descent.toFixed(1)} m/s is too fast: use a bigger parachute.`);
  else if (e.descent > 6) add("warn", `Descent ${e.descent.toFixed(1)} m/s: a little fast for servos and the board.`);
  else if (e.descent) add("good", `Descent ${e.descent.toFixed(1)} m/s: OK.`);
  add("info", "Estimate: constant average thrust, vertical flight, no wind. Use OpenRocket for real numbers.");
  checks.replaceChildren(...items);
}

/* ------------------------------------------------------------------ build & flash */
const IMAGE_NAMES = {bench: "Bench image", flight: "Flight image", sensortest: "Sensor test image"};
const SENSOR_TEST_STEPS = [el("strong", {text: "Now run the sensor test:"}),
  el("ol", {class: "steps"}, el("li", {text: "Put a microSD card in the board (it records every reading)."}),
    el("li", {text: "Lay the board flat and still. Unplug USB-C, wait 2 s, plug it back in."}),
    el("li", {text: "LEDs red → green → blue, one beep, then 3 s of fast green blinking while it tests. Don't touch it."}),
    el("li", {text: "Two short beeps + steady green = every sensor passed. Three long beeps + red blinking = a failure: 1 blink IMU, 2 barometer, 3 SD card, 4 power."}),
    el("li", {text: "Leave it running as long as you like (blue blinks while recording, ~9 min of space), then read the card in Flight logs for every value."})),
  el("p", {class: "help", text: "Servos stay centred and nothing arms. With a USB-UART on TX2/RX2 the Live board page shows the values live."})];
function renderImages() {
  const info = buildInfo; if (!info) return;
  const list = $("images"); list.replaceChildren();
  for (const key of ["bench", "flight", "sensortest"]) {
    const image = info.images[key];
    list.append(el("div", {class: `image-row ${image && !image.stale ? "" : "stale"}`},
      el("strong", {text: IMAGE_NAMES[key]}),
      el("span", {text: image ? `${(image.size / 1024).toFixed(1)} KB · ${fmtTime(image.built)}${image.stale ? " · OUT OF DATE" : ""}` : "not built"}),
      el("code", {text: image ? image.sha256.slice(0, 16) + "…" : ""})));
  }
  const dirty = cfg && !same(cfg, savedCfg);
  const stale = ["bench", "flight"].some(k => !info.images[k] || info.images[k].stale);
  $("build-status").textContent = dirty ? "Rocket setup has unsaved changes: save first." : stale ? "Build needed." : "Up to date with your saved setup.";
  $("build-status").className = dirty || stale ? "assessment" : "assessment good";
  const flashed = appState.flashed;
  $("flash-status").textContent = flashed ? `Last flashed: ${flashed.image} image (${flashed.sha256.slice(0, 12)}…) on ${fmtTime(flashed.time)}` : "Nothing flashed from Studio yet.";
  updateRocketStatus();
}
action("build", async () => {
  if (cfg && !same(cfg, savedCfg)) throw new Error("Save the rocket setup first.");
  $("build").disabled = true; $("build").textContent = "Building… (about 30 s)"; $("build-log").textContent = "Building…";
  try {
    const result = await api("build/run", {});
    buildInfo = result; $("build-log").textContent = result.log;
    renderImages();
    notice(result.ok ? "Build finished." : "Build failed. Open the build log.");
  } finally { $("build").disabled = false; $("build").innerHTML = "Build all images <span>↗</span>"; }
});
let dfuDevices = [];
action("dfu-refresh", async () => {
  $("dfu-status").textContent = "Looking…";
  const result = await api("dfu/list", {});
  dfuDevices = result.devices || [];
  $("dfu-status").textContent = result.message + (dfuDevices.length ? ` (${dfuDevices.map(d => d.serial).join(", ")})` : "");
  $("dfu-status").className = dfuDevices.length === 1 ? "assessment good" : "assessment";
});
async function flashImage(key) {
  if (!buildInfo || !buildInfo.images[key]) throw new Error("Build the firmware first.");
  const image = buildInfo.images[key];
  if (image.stale) throw new Error("This image is older than your saved setup. Build again.");
  const body = [el("p", {text: `${IMAGE_NAMES[key]} · ${(image.size / 1024).toFixed(1)} KB`}), el("p", {class: "mono", text: image.sha256})];
  if (key === "flight") {
    const missing = sectionMissing("bench");
    if (missing.length) body.push(el("p", {class: "warn-text", text: `The bench checklist isn't finished (${missing.length} item${missing.length > 1 ? "s" : ""} open). The flight image arms itself on the pad.`}));
    body.push(el("p", {text: "Flight image: once it's armed on the pad, the gimbal will move. Keep fingers clear of the linkage."}));
  }
  body.push(el("p", {class: "help", text: "Board in DFU mode, battery disconnected. Flashing takes about 10 s and verifies every byte."}));
  if (!await confirmModal(`Flash the ${key} image?`, body, "Flash")) return;
  if (!dfuDevices.length) await (async () => { const r = await api("dfu/list", {}); dfuDevices = r.devices || []; })();
  if (dfuDevices.length === 0) throw new Error("No board in DFU mode. Hold BOOT, tap RST, release BOOT, then try again.");
  if (dfuDevices.length > 1) throw new Error("More than one DFU device is connected. Unplug the others.");
  $("flash-status").textContent = "Flashing… do not unplug."; $("flash-status").className = "assessment";
  try {
    const result = await api("dfu/flash", {image: key, sha256: image.sha256, serial: dfuDevices[0].serial});
    $("flash-log").textContent = result.log;
    appState.flashed = {image: key, sha256: image.sha256, time: Date.now() / 1000};
    if (key === "bench") toggleCheck("b-flash", true);
    renderImages(); renderChecklists();
    $("flash-status").textContent = `Flashed and verified: ${key} image. Now unplug USB and the battery to power-cycle.`;
    $("flash-status").className = "assessment good";
    if (key === "sensortest") await confirmModal("Sensor test installed", SENSOR_TEST_STEPS, "OK");
  } catch (error) { $("flash-log").textContent = error.message; $("flash-status").textContent = "Flashing failed. See the log; the board stays in DFU."; throw error; }
}
action("flash-bench", () => flashImage("bench"));
action("flash-flight", () => flashImage("flight"));
action("flash-sensortest", () => flashImage("sensortest"));

/* ------------------------------------------------------------------ live board */
function showLivePorts(ports, current) {
  const select = $("live-port"), old = current || select.value; select.replaceChildren();
  if (!ports.length) select.append(el("option", {value: "", text: "No USB serial adapter found"}));
  ports.forEach(p => select.append(el("option", {value: p, text: p})));
  if (ports.includes(old)) select.value = old;
}
action("live-refresh", async () => showLivePorts((await api("monitor/status", {})).ports));
action("live-connect", async () => { liveStatus = await api("monitor/connect", {port: $("live-port").value}); renderLive(); pollLive(); });
action("live-disconnect", async () => { clearTimeout(liveTimer); liveStatus = await api("monitor/disconnect", {}); renderLive(); });
async function pollLive() {
  clearTimeout(liveTimer);
  try { liveStatus = await api("monitor/status", {}); renderLive(); }
  catch (error) { notice(error.message); return; }
  if (liveStatus.connected) liveTimer = setTimeout(pollLive, 250);
}
const SENSOR_CHECK_TEXT = {accel_id: "IMU accelerometer answers", gyro_id: "IMU gyroscope answers", baro_id: "Barometer answers",
  accel_1g: "Accelerometer reads 1 g at rest", gyro_still: "Gyroscope quiet at rest", imu_alive: "IMU values change (not stuck)",
  imu_rate: "IMU read every time", pressure: "Pressure 60–110 kPa", baro_temp: "Barometer temperature sane", humidity: "Humidity sane and changing",
  baro_rate: "Barometer read every time", accel_temp: "IMU die temperature sane", sd: "SD card recording, nothing lost", supply: "3.3 V rail 3.0–3.6 V"};
function checkList(rows) {
  return rows.map(([text, pass, note]) => el("li", {class: pass ? "pass" : "fail"}, el("b", {text: pass ? "PASS" : "FAIL"}), el("span", {text}),
    note ? el("span", {class: "help-inline", text: note}) : ""));
}
function renderSensorTest(s) {
  const v = s.latest.sensors, q = s.latest.sensor_checks, age = s.age.sensors;
  $("live-sensors").hidden = !v && !q;
  if (!v && !q) return;
  $("sensor-age").textContent = age == null ? "no data" : age > 1.5 ? `STALE ${age.toFixed(0)} s` : "live";
  $("sensor-age").className = age != null && age <= 1.5 ? "chip good" : "chip";
  const fmt = (x, d) => x.toFixed(d);
  const tiles = v ? [["Accel X / Y / Z", `${v.accel_g.map(x => fmt(x, 3)).join(" / ")} g`], ["|accel|", `${fmt(Math.hypot(...v.accel_g), 3)} g`],
    ["Gyro X / Y / Z", `${v.gyro_dps.map(x => fmt(x, 2)).join(" / ")} °/s`], ["IMU temperature", `${fmt(v.imu_temp_c, 1)} °C`],
    ["Pressure", `${fmt(v.pressure_pa / 100, 2)} hPa`], ["Baro temperature", `${fmt(v.baro_temp_c, 2)} °C`], ["Humidity", `${fmt(v.humidity_pct, 1)} %`],
    ["3.3 V rail", `${fmt(v.vdda_v, 3)} V`], ["Battery pin (PA4)", `${fmt(v.batt_pin_v, 3)} V`], ["MCU temperature", `${fmt(v.mcu_temp_c, 1)} °C`]] : [];
  if (q) tiles.push(["IMU samples / errors", `${q.imu_samples} / ${q.imu_errors}`], ["Baro samples / errors", `${q.baro_samples} / ${q.baro_errors}`],
    ["SD", q.sd ? `session ${q.session} · ${q.records} records` : "NO CARD"], ["SD dropped / errors", `${q.dropped} / ${q.write_errors}`]);
  $("sensor-values").replaceChildren(...tiles.map(([label, value]) => { const t = textNode("div", label); t.append(textNode("strong", value)); return t; }));
  if (q) {
    const keys = Object.keys(SENSOR_CHECK_TEXT), failed = keys.filter(k => !q.checks[k]);
    $("sensor-headline").textContent = failed.length ? `${failed.length} check${failed.length > 1 ? "s" : ""} failing` : "Every sensor passes";
    $("sensor-checks").replaceChildren(...checkList(keys.map(k => [SENSOR_CHECK_TEXT[k], q.checks[k], q.boot[k] === q.checks[k] ? "" : q.boot[k] ? "(passed at power-up)" : "(failed at power-up)"])));
  } else $("sensor-headline").textContent = "Waiting for the 1 Hz check line…";
}
function renderLive() {
  const s = liveStatus || {connected: false, latest: {}, age: {}, lines: []};
  $("live-connect").disabled = s.connected; $("live-disconnect").disabled = !s.connected;
  $("connection-badge").textContent = s.connected ? `BOARD · ${s.latest.status ? s.latest.status.state : "LISTENING"}` : "BOARD · NOT CONNECTED";
  $("live-state").textContent = s.connected ? `Listening on ${s.port}.` : "Not connected. Read-only: Studio never sends anything to the board.";
  if (s.ports) showLivePorts(s.ports, s.port);
  const st = s.latest.status, age = s.age.status;
  $("live-age").textContent = age == null ? "no data" : age > 2.5 ? `STALE ${age.toFixed(0)} s` : "live";
  $("live-age").className = age != null && age <= 2.5 ? "chip good" : "chip";
  const tiles = $("live-tiles"); tiles.replaceChildren();
  if (st) {
    $("live-headline").textContent = st.recovery ? "SAFE MODE (recovery boot)" : st.state === "ARMED" ? "ARMED: hands off" : st.state;
    const ok = v => v ? "OK" : "FAIL";
    [["State", st.state], ["Armed", st.armed ? "YES" : "no"], ["IMU", ok(st.imu)], ["Gyro cal", ok(st.gyro_cal)], ["Barometer", ok(st.baro)],
     ["SD card", ok(st.sd)], ["Session", st.session], ["Records", st.records], ["Dropped", st.dropped], ["Write errors", st.write_errors],
     ["Parachute", st.chute ? "OPEN" : "closed"], ["Recovery boot", st.recovery ? "YES" : "no"]]
      .forEach(([label, value]) => { const t = textNode("div", label); t.append(textNode("strong", String(value)));
        if (["FAIL", "YES"].includes(String(value)) && label !== "Armed") t.className = "bad"; tiles.append(t); });
  } else $("live-headline").textContent = s.connected ? "Waiting for the 1 Hz status line…" : "—";
  renderSensorTest(s);
  const bench = s.latest.bench, flight = s.latest.flight;
  let tx = null, ty = null;
  if (bench && s.age.bench < 1.5) { tx = bench.tilt_x; ty = bench.tilt_y; $("live-mode").textContent = "bench image"; }
  else if (flight && s.age.flight < 1.5) { tx = flight.roll * 57.2958; ty = flight.pitch * 57.2958; $("live-mode").textContent = "flight image"; }
  else $("live-mode").textContent = "—";
  drawBubble(tx, ty);
  const servos = $("live-servos"); servos.replaceChildren();
  if (bench && cfg) [["A", bench.pulse_a], ["B", bench.pulse_b]].forEach(([n, us]) => {
    const lo = cfg[`SERVO_${n}_MIN_US`], hi = cfg[`SERVO_${n}_MAX_US`], c = cfg[`SERVO_${n}_CENTER_US`];
    const pct = Math.max(0, Math.min(100, (us - 1000) / 10));
    const bar = el("div", {class: "servo-bar"}, el("i", {style: `left:${(lo - 1000) / 10}%;width:${(hi - lo) / 10}%`}), el("b", {style: `left:${pct}%`}), el("u", {style: `left:${(c - 1000) / 10}%`}));
    servos.append(el("div", {class: "servo-meter"}, el("span", {text: `Servo ${n} · ${us} µs`}), bar));
  });
  const suggestion = s.suggested_nose_axis;
  if (bench) {
    const raw = bench.raw.map(v => v.toFixed(2)).join(" / ");
    if (suggestion && cfg && suggestion === cfg.NOSE_AXIS) { $("nose-suggest").textContent = `Nose axis ${suggestion} is set correctly (raw accel ${raw} g).`; $("nose-suggest").className = "assessment good"; $("nose-apply").hidden = true; }
    else if (suggestion) { $("nose-suggest").textContent = `With the rocket nose-up, the nose axis is ${suggestion} (raw accel ${raw} g). Your setup says ${cfg ? cfg.NOSE_AXIS : "?"}.`; $("nose-suggest").className = "assessment"; $("nose-apply").hidden = false; $("nose-apply").textContent = `Use ${suggestion}`; }
    else { $("nose-suggest").textContent = `Raw accel ${raw} g. Stand the rocket nose-up and keep it still to detect the nose axis.`; $("nose-suggest").className = "assessment"; $("nose-apply").hidden = true; }
  }
  $("live-lines").textContent = (s.lines || []).join("\n") || "—";
}
$("nose-apply").onclick = () => {
  const suggestion = liveStatus && liveStatus.suggested_nose_axis; if (!suggestion || !cfg) return;
  cfg.NOSE_AXIS = suggestion; renderRocket(); cfgChanged(); showTab("rocket");
  notice(`Nose axis set to ${suggestion}. Save, build and reflash the bench image, then do the tilt test.`);
};
function drawBubble(tx, ty) {
  const canvas = $("bubble"), ctx = canvas.getContext("2d"), size = 260, c = size / 2, r = 110, range = 30;
  const scale = window.devicePixelRatio || 1; canvas.width = size * scale; canvas.height = size * scale; canvas.style.width = canvas.style.height = size + "px";
  ctx.scale(scale, scale); ctx.clearRect(0, 0, size, size);
  ctx.strokeStyle = pal().line; ctx.lineWidth = 1;
  [10, 20, 30].forEach(d => { ctx.beginPath(); ctx.arc(c, c, r * d / range, 0, 2 * Math.PI); ctx.stroke(); });
  ctx.beginPath(); ctx.moveTo(c - r, c); ctx.lineTo(c + r, c); ctx.moveTo(c, c - r); ctx.lineTo(c, c + r); ctx.stroke();
  const limit = cfg ? cfg.MAX_TILT_DEG : 25;
  ctx.strokeStyle = pal().badLine; ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.arc(c, c, r * Math.min(limit, range) / range, 0, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = pal().muted; ctx.font = pal().font(10); ctx.fillText("about X →", size - 62, c - 6); ctx.fillText("about Y ↑", c + 6, 14);
  [10, 20, 30].forEach(d => ctx.fillText(`${d}°`, c + 3, c - r * d / range + 11));
  if (tx == null) return;
  const x = c + Math.max(-1, Math.min(1, tx / range)) * r, y = c - Math.max(-1, Math.min(1, ty / range)) * r;
  ctx.fillStyle = Math.hypot(tx, ty) > limit ? "#c0392b" : pal().purple;
  ctx.beginPath(); ctx.arc(x, y, 9, 0, 2 * Math.PI); ctx.fill();
  ctx.fillStyle = pal().text; ctx.fillText(`X ${tx.toFixed(1)}°  Y ${ty.toFixed(1)}°`, 8, size - 8);
}

/* ------------------------------------------------------------------ launch checklist */
const CHECKLIST = [
  {id: "bench", title: "1 · Bench (bench image)", items: [
    ["b-flash", "Bench image flashed; board power-cycled"],
    ["b-led", "Power-up LED test shows red, then green, then blue (otherwise flip the LED setting)"],
    ["b-sweep", "Power-up sweep: servo A moves, then B, then the parachute servo. Nothing binds at the ends"],
    ["b-nose", "Nose axis correct: green LED solid (and Live board agrees) with the rocket nose-up"],
    ["b-tilt-a", "Tilt test, servo A: when the nose leans, the nozzle swings to the opposite side"],
    ["b-tilt-b", "Tilt test, servo B: same for the other two directions"],
    ["b-limit", "Past the give-up tilt (≈25°) the gimbal centres"],
    ["b-power", "Power test: 30 s of loaded servos on the LiPo with no reboot (the sweep didn't re-run); regulator not too hot"],
    ["b-sd", "SD log read back in Flight logs shows the tilt test"],
    ["b-chute", "Parachute release opens and closes reliably, and the parachute really comes out"]]},
  {id: "prep", title: "2 · Flight prep", items: [
    ["p-gains", "Gains checked in the Tuning lab with measured inertia and thrust arm"],
    ["p-backup", "Parachute backup time set from the apogee estimate"],
    ["p-flight", "Flight image built from the saved setup and flashed; board power-cycled"],
    ["p-weight", "Weighed: liftoff mass with motor under ~400 g"],
    ["p-cg", "CG checked; gimbal centred and free"],
    ["p-battery", "2S LiPo fully charged (≥ 8.2 V on a meter)"],
    ["p-sd", "SD card in the board"],
    ["p-motor", "E12-0 (no ejection charge) fitted and retained in the gimbal"],
    ["p-chute", "Parachute packed with heat protection; release closed"]]},
  {id: "pad", title: "3 · At the pad", items: [
    ["l-wind", "Low wind (calm for a first flight); clear field; spectators back"],
    ["l-stand", "Rocket vertical on the stand and free to leave it"],
    ["l-igniter", "Igniter fitted; launch controller safe (key out)"],
    ["l-power", "Battery connected: beep and gimbal sweep seen"],
    ["l-armed", "Hands off: green LED solid + chirp every second (ARMED), no red LED"],
    ["l-launch", "Controller armed, countdown, launch"]]},
  {id: "after", title: "4 · After the flight", items: [
    ["a-recover", "Rocket recovered; battery disconnected"],
    ["a-log", "Session read in Flight logs"]]},
];
const PACKING = ["Rocket + gimbal", "E12-0 motors (+ spares)", "Igniters (+ spares)", "Launch controller + its battery", "Launch stand",
  "Charged 2S LiPo + spare", "LiPo safe bag", "Laptop with EasyTVC Studio", "USB-C data cable", "USB-UART adapter + jumper wires",
  "SD card + card reader", "Parachute, wadding / Nomex", "Spare servo + pushrods", "Small screwdrivers / hex keys", "Tape, zip ties, CA glue",
  "Multimeter", "Fire extinguisher or water", "First-aid kit", "Phone / camera for video"];
function checks() { return appState.checklist || {}; }
function toggleCheck(id, value) {
  const c = {...checks(), [id]: value}; appState.checklist = c;
  saveState({checklist: c}).catch(e => notice(e.message)); renderChecklists();
}
function sectionMissing(id) { const sec = CHECKLIST.find(s => s.id === id); return sec.items.filter(([k]) => !checks()[k]); }
function firmwareReady() {
  const reasons = [];
  if (cfg && !same(cfg, savedCfg)) reasons.push("Rocket setup has unsaved changes.");
  const flight = buildInfo && buildInfo.images.flight;
  if (!flight) reasons.push("Flight image not built.");
  else if (flight.stale) reasons.push("Flight image is older than the saved setup.");
  const flashed = appState.flashed;
  if (!flashed || flashed.image !== "flight") reasons.push("Flight image not flashed from Studio.");
  else if (flight && flashed.sha256 !== flight.sha256) reasons.push("The flashed flight image isn't the current build.");
  return reasons;
}
function updateGo() {
  if (!cfg) return;
  const reasons = firmwareReady();
  for (const id of ["bench", "prep"]) { const m = sectionMissing(id); if (m.length) reasons.push(`${CHECKLIST.find(s => s.id === id).title}: ${m.length} open`); }
  const pad = sectionMissing("pad").filter(([k]) => k !== "l-launch");
  if (pad.length) reasons.push(`3 · At the pad: ${pad.length} open`);
  const go = reasons.length === 0;
  const banner = $("go-banner");
  banner.className = `go-banner ${go ? "go" : "nogo"}`;
  banner.replaceChildren(el("div", {class: "go-light"}), el("div", {}, el("strong", {text: go ? "GO FOR LAUNCH" : "NO-GO"}),
    el("p", {text: go ? "Every check is ticked and the flashed flight image is your current setup. Fly safe." : reasons.join(" · ")})));
  const side = $("sidebar-go"); side.className = `sidebar-go ${go ? "go" : ""}`;
  side.querySelector("strong").textContent = go ? "GO" : "NO-GO";
  side.querySelector("p").textContent = go ? "Ready to launch." : `${reasons.length} item${reasons.length === 1 ? "" : "s"} open`;
}
function renderChecklists() {
  const wrap = $("checklists"); wrap.replaceChildren();
  for (const section of CHECKLIST) {
    const done = section.items.filter(([k]) => checks()[k]).length;
    const card = el("article", {class: "card"}, el("div", {class: "card-heading"}, el("h2", {text: section.title}), el("span", {class: done === section.items.length ? "chip good" : "chip", text: `${done}/${section.items.length}`})));
    section.items.forEach(([key, text]) => {
      const box = el("input", {type: "checkbox"}); box.checked = !!checks()[key];
      box.onchange = () => toggleCheck(key, box.checked);
      card.append(el("label", {class: "check-item"}, box, el("span", {text})));
    });
    wrap.append(card);
  }
  const packing = appState.packing || {};
  const card = el("article", {class: "card"}, el("div", {class: "card-heading"}, el("h2", {text: "Launch-day kit"}),
    el("button", {class: "quiet", text: "Clear", onclick: () => { appState.packing = {}; saveState({packing: {}}); renderChecklists(); }})));
  PACKING.forEach(item => {
    const box = el("input", {type: "checkbox"}); box.checked = !!packing[item];
    box.onchange = () => { appState.packing = {...(appState.packing || {}), [item]: box.checked}; saveState({packing: appState.packing}).catch(e => notice(e.message)); };
    card.append(el("label", {class: "check-item"}, box, el("span", {text: item})));
  });
  wrap.append(card);
  updateGo();
}
action("check-reset-flight", async () => {
  const c = {...checks()}; for (const id of ["prep", "pad", "after"]) CHECKLIST.find(s => s.id === id).items.forEach(([k]) => delete c[k]);
  appState.checklist = c; await saveState({checklist: c}); renderChecklists();
});
action("check-reset-all", async () => {
  if (!await confirmModal("Clear every check?", [el("p", {text: "This includes the bench checks. Redo them if anything on the rocket changed."})], "Clear")) return;
  appState.checklist = {}; await saveState({checklist: {}}); renderChecklists();
});

/* ------------------------------------------------------------------ flight logs */
async function refreshDisks() {
  const result = await api("logs/disks", {});
  const select = $("disk"); select.replaceChildren();
  if (!result.disks.length) select.append(el("option", {value: "", text: "No card reader / SD card found"}));
  result.disks.forEach(d => select.append(el("option", {value: d.id, text: `${d.name} · ${d.size_gb} GB (${d.id})`})));
}
action("disk-refresh", refreshDisks);
action("sd-read", async () => {
  const disk = $("disk").value; if (!disk) throw new Error("Insert the SD card and press Refresh.");
  const n = $("session-number").value;
  $("sd-read").disabled = true; $("sd-read").textContent = "Reading… (macOS may ask for your password)";
  try { showLog(await api("logs/read-sd", {disk, session: n === "" ? null : Number(n)})); }
  finally { $("sd-read").disabled = false; $("sd-read").textContent = "Read from SD card"; }
});
action("log-import", () => $("log-file").click());
$("log-file").onchange = async () => {
  try {
    const file = $("log-file").files[0]; if (!file) return;
    const response = await fetch("/api/logs/upload", {method: "POST", body: await file.arrayBuffer(),
      headers: {Authorization: `Bearer ${token || ""}`, "Content-Type": "application/octet-stream"}});
    const result = await response.json(); if (!response.ok) throw new Error(result.error);
    result.source = file.name; showLog(result);
  } catch (error) { notice(error.message); } finally { $("log-file").value = ""; }
};
function showSensorLog(result) {
  logData = result; $("log-result").hidden = false; $("log-range").hidden = true; $("log-result").classList.add("sensor-mode");
  const r = result.report, tile = (label, value, cls = "") => el("div", {class: cls}, el("span", {text: label}), el("strong", {text: value}));
  const ch = r.channels, mean = (k, d, unit) => ch[k] ? `${ch[k].mean.toFixed(d)}${unit}` : "—";
  $("log-summary").replaceChildren(tile("SENSOR TEST", r.passed ? "PASSED" : "FAILED", r.passed ? "" : "bad"), tile("SESSION", String(result.session)),
    tile("DURATION", `${r.duration_s.toFixed(1)} s`), tile("IMU RATE", `${r.rates_hz.imu.toFixed(0)} Hz`), tile("BARO RATE", `${r.rates_hz.baro.toFixed(0)} Hz`),
    tile("RECORDS", String(r.counts.records)), tile("GAPS", String(r.counts.gaps), r.counts.gaps ? "bad" : ""),
    tile("SD LOST", r.sd ? String(r.sd.dropped + r.sd.write_errors) : "—", r.sd && r.sd.dropped + r.sd.write_errors ? "bad" : ""));
  $("log-title").textContent = `Sensor test · session ${result.session}`;
  $("log-title").previousElementSibling.textContent = "RESULTS";
  $("log-header").textContent = `${result.header}  ·  ${result.source}`;
  const stat = (k, d, unit) => ch[k] ? `${ch[k].mean.toFixed(d)} ± ${ch[k].sd.toFixed(d + 1)} ${unit} (min ${ch[k].min.toFixed(d)}, max ${ch[k].max.toFixed(d)})` : "no data";
  const ids = r.ids ? `accel 0x${r.ids.accel_id.toString(16).toUpperCase()}, gyro 0x${r.ids.gyro_id.toString(16).toUpperCase()}, baro 0x${r.ids.baro_id.toString(16).toUpperCase()}` : "—";
  $("log-events").replaceChildren(...checkList(r.checks.map(c => [c.text, c.boot, c.now === c.boot ? "" : c.now ? "(passes now)" : "(failing at the end)"])),
    ...[["Chip IDs", ids], ["Accel X", stat("accel_x_g", 3, "g")], ["Accel Y", stat("accel_y_g", 3, "g")], ["Accel Z", stat("accel_z_g", 3, "g")],
      ["Gyro X", stat("gyro_x_dps", 2, "°/s")], ["Gyro Y", stat("gyro_y_dps", 2, "°/s")], ["Gyro Z", stat("gyro_z_dps", 2, "°/s")],
      ["IMU temperature", stat("imu_temp_c", 1, "°C")], ["IMU read time", stat("read_us", 0, "µs")], ["Pressure", stat("pressure_pa", 0, "Pa")],
      ["Baro temperature", stat("baro_temp_c", 2, "°C")], ["Humidity", stat("humidity_pct", 1, "%")], ["3.3 V rail", stat("vdda_v", 3, "V")],
      ["Battery pin (PA4)", stat("batt_pin_v", 3, "V")], ["MCU temperature", stat("mcu_temp_c", 1, "°C")], ["Loop time (max)", stat("max_loop_us", 0, "µs")]]
      .map(([label, value]) => el("li", {}, el("time", {text: label}), el("span", {class: "mono", text: value}))));
  $("log-events").className = "events sensor-checks-log";
  $("log-sessions").replaceChildren();
  if (result.available_sessions && result.available_sessions.length > 1) {
    $("log-sessions").append(el("span", {class: "help", text: "Other sessions on this card: "}));
    result.available_sessions.slice().reverse().forEach(n => $("log-sessions").append(el("button", {class: n === result.session ? "small selected" : "small quiet", text: String(n),
      onclick: () => { $("session-number").value = n; $("sd-read").click(); }})));
  }
  sensorView.init(result);
}

/* ---- sensor-test charts: one per sensor, hover for exact values, drag to zoom (full-rate data
 * is fetched for the zoomed window), double-click or Reset to see the whole session ---- */
const SENSOR_CHARTS = [
  {title: "Accelerometer", unit: "g", lines: [["imu", "accel_x_g", "X"], ["imu", "accel_y_g", "Y"], ["imu", "accel_z_g", "Z"], ["imu", "accel_mag", "|a|"]], digits: 3},
  {title: "Gyroscope", unit: "°/s", lines: [["imu", "gyro_x_dps", "X"], ["imu", "gyro_y_dps", "Y"], ["imu", "gyro_z_dps", "Z"]], digits: 2},
  {title: "Pressure", unit: "hPa", lines: [["baro", "pressure_hpa", "pressure"]], digits: 2},
  {title: "Altitude change (from pressure)", unit: "m", lines: [["baro", "altitude_m", "height"]], digits: 2},
  {title: "Temperatures", unit: "°C", lines: [["baro", "baro_temp_c", "barometer"], ["imu", "imu_temp_c", "IMU"], ["adc", "mcu_temp_c", "MCU"]], digits: 2},
  {title: "Humidity", unit: "%", lines: [["baro", "humidity_pct", "humidity"]], digits: 1},
  {title: "Power", unit: "V", lines: [["adc", "vdda_v", "3.3 V rail"], ["adc", "batt_pin_v", "battery pin"]], digits: 3},
  {title: "Timing", unit: "µs", lines: [["imu", "read_us", "IMU read"], ["adc", "max_loop_us", "slowest loop"]], digits: 0},
];
function derive(series, p0) {
  const imu = series.imu, baro = series.baro;
  imu.accel_mag = imu.t.map((_, i) => Math.hypot(imu.accel_x_g[i], imu.accel_y_g[i], imu.accel_z_g[i]));
  baro.pressure_hpa = baro.pressure_pa.map(p => p / 100);
  baro.altitude_m = baro.pressure_pa.map(p => 44330 * (1 - Math.pow(p / p0, 0.190295)));
  return series;
}
function nearest(t, x) { let lo = 0, hi = t.length - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (t[m] < x) lo = m; else hi = m; } return Math.abs(t[lo] - x) <= Math.abs(t[hi] - x) ? lo : hi; }
const sensorColor = n => [...pal().series, pal().term ? "#5ac8fa" : "#3b82f6"][n % 4];
const sensorView = {
  result: null, overview: null, detail: null, full: [0, 1], view: [0, 1], hover: null, drag: null, cards: [], fetchTimer: null, seq: 0,
  init(result) {
    this.result = result;
    const S = result.series, p0 = S.baro.pressure_pa.length ? S.baro.pressure_pa.slice(0, 25).reduce((a, b) => a + b, 0) / Math.min(25, S.baro.pressure_pa.length) : 101325;
    this.p0 = p0; this.overview = derive(S, p0); this.detail = null;
    const ts = ["imu", "baro", "adc"].flatMap(g => S[g].t.length ? [S[g].t[0], S[g].t[S[g].t.length - 1]] : []);
    this.full = [Math.min(...ts), Math.max(...ts)]; this.view = [...this.full];
    const wrap = $("log-charts"); wrap.replaceChildren();
    const bar = el("div", {class: "sensor-zoom-bar"}, el("span", {id: "sensor-range", class: "mono"}),
      el("span", {class: "help-inline", text: "Hover to read values · drag across a chart to zoom in · double-click to zoom out"}),
      el("button", {class: "small", text: "Reset zoom", onclick: () => this.setView(this.full)}));
    wrap.append(bar);
    const grid = el("div", {class: "sensor-chart-grid"}); wrap.append(grid);
    this.cards = SENSOR_CHARTS.map(spec => {
      const base = el("canvas", {class: "sensor-chart", role: "img", "aria-label": `${spec.title} chart`});
      const over = el("canvas", {class: "sensor-chart overlay"});
      const legend = el("div", {class: "legend sensor-legend"});
      const card = el("article", {class: "card chart-card sensor-card"}, el("div", {class: "card-heading"}, el("h2", {text: `${spec.title} (${spec.unit})`}), legend),
        el("div", {class: "sensor-plot"}, base, over));
      grid.append(card);
      const c = {spec, base, over, legend, layout: null};
      over.addEventListener("mousemove", e => this.onMove(c, e));
      over.addEventListener("mouseleave", () => { this.hover = null; if (!this.drag) this.drawOverlays(); });
      over.addEventListener("mousedown", e => { this.drag = {card: c, x0: this.timeAt(c, e)}; });
      over.addEventListener("dblclick", () => this.setView(this.full));
      return c;
    });
    if (!this.mouseUpBound) { window.addEventListener("mouseup", e => this.onUp(e)); this.mouseUpBound = true; }
    requestAnimationFrame(() => this.drawAll());
  },
  data() { const d = this.detail; return d && d.range[0] <= this.view[0] && d.range[1] >= this.view[1] ? d.series : this.overview; },
  setView(range) {
    const span = Math.max(0.005, range[1] - range[0]);
    this.view = [Math.max(this.full[0], range[0]), Math.min(this.full[1], range[0] + span)];
    this.drawAll();
    clearTimeout(this.fetchTimer);
    const zoomed = this.view[1] - this.view[0] < (this.full[1] - this.full[0]) * 0.98;
    if (!zoomed) { this.detail = null; return; }
    const seq = ++this.seq, want = [...this.view];
    this.fetchTimer = setTimeout(async () => {
      try {
        const S = await api("logs/sensor-window", {id: this.result.data_id, t0: want[0], t1: want[1], points: 6000});
        if (seq !== this.seq) return;
        this.detail = {range: want, series: derive(S, this.p0), step: S.imu.step}; this.drawAll();
      } catch (error) { notice(error.message); }
    }, 120);
  },
  timeAt(c, e) { const r = c.over.getBoundingClientRect(), L = c.layout; return L.t0 + Math.max(0, Math.min(1, (e.clientX - r.left - L.left) / L.width)) * (L.t1 - L.t0); },
  onMove(c, e) { if (!c.layout) return; this.hover = this.timeAt(c, e); this.drawOverlays(); },
  onUp(e) {
    const d = this.drag; this.drag = null; if (!d) return;
    const t = this.timeAt(d.card, e), lo = Math.min(d.x0, t), hi = Math.max(d.x0, t);
    if (hi - lo > (this.view[1] - this.view[0]) * 0.01) this.setView([lo, hi]); else this.drawOverlays();
  },
  drawAll() {
    if (!this.cards.length || !this.cards[0].base.isConnected) return;
    const S = this.data(), [t0, t1] = this.view, span = t1 - t0;
    const detail = this.detail && S === this.detail.series;
    $("sensor-range").textContent = `${t0.toFixed(3)} – ${t1.toFixed(3)} s  (${span < 1 ? (span * 1000).toFixed(0) + " ms" : span.toFixed(1) + " s"})` +
      (detail ? (this.detail.step === 1 ? " · every sample" : ` · every ${this.detail.step}th sample`) : " · overview");
    this.cards.forEach(c => this.drawChart(c, S));
    this.drawOverlays();
  },
  drawChart(c, S) {
    const canvas = c.base, rect = canvas.getBoundingClientRect(); if (!rect.width) return;
    const k = window.devicePixelRatio || 1; canvas.width = rect.width * k; canvas.height = rect.height * k;
    const ctx = canvas.getContext("2d"); ctx.scale(k, k);
    const left = 56, top = 8, width = rect.width - 66, height = rect.height - 30, [t0, t1] = this.view;
    const lines = c.spec.lines.map(([g, key, name]) => ({t: S[g].t, v: S[g][key], name})).filter(l => l.v && l.v.length);
    let lo = Infinity, hi = -Infinity;
    lines.forEach(l => { for (let i = 0; i < l.t.length; i++) if (l.t[i] >= t0 && l.t[i] <= t1) { lo = Math.min(lo, l.v[i]); hi = Math.max(hi, l.v[i]); } });
    if (!isFinite(lo)) { lo = 0; hi = 1; }
    if (hi - lo < 1e-9) { lo -= 0.5 * Math.max(1e-3, Math.abs(lo) * 1e-3); hi = lo + Math.max(1e-3, Math.abs(lo) * 2e-3); }
    const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
    c.layout = {left, top, width, height, t0, t1, lo, hi, lines};
    const X = t => left + (t - t0) / (t1 - t0) * width, Y = v => top + (hi - v) / (hi - lo) * height;
    ctx.font = pal().font(10); ctx.fillStyle = pal().muted; ctx.strokeStyle = pal().grid; ctx.lineWidth = 1;
    const dec = Math.max(0, Math.min(4, Math.ceil(-Math.log10((hi - lo) / 4)) + 1));
    for (let i = 0; i <= 4; i++) { const v = hi - i * (hi - lo) / 4, y = Y(v); ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(left + width, y); ctx.stroke(); ctx.fillText(v.toFixed(dec), 2, y + 3); }
    const tdec = t1 - t0 < 0.5 ? 3 : t1 - t0 < 5 ? 2 : 1;
    for (let i = 0; i <= 5; i++) { const v = t0 + i * (t1 - t0) / 5; ctx.fillText(`${v.toFixed(tdec)}s`, X(v) - 12, top + height + 18); }
    ctx.save(); ctx.beginPath(); ctx.rect(left, top, width, height); ctx.clip();
    lines.forEach((l, n) => {
      /* min/max per pixel column, so noise and spikes survive any amount of zoom-out */
      ctx.strokeStyle = sensorColor(n); ctx.lineWidth = 1.3; ctx.beginPath();
      let col = null, mn = 0, mx = 0, started = false;
      const flush = () => { if (col === null) return; const x = left + col + 0.5; if (!started) { ctx.moveTo(x, Y(mn)); started = true; } else ctx.lineTo(x, Y(mn)); if (mx !== mn) ctx.lineTo(x, Y(mx)); };
      for (let i = 0; i < l.t.length; i++) {
        if (l.t[i] < t0 - (t1 - t0) * 0.01 || l.t[i] > t1 + (t1 - t0) * 0.01) continue;
        const cx = Math.round(X(l.t[i]) - left);
        if (cx !== col) { flush(); col = cx; mn = mx = l.v[i]; } else { mn = Math.min(mn, l.v[i]); mx = Math.max(mx, l.v[i]); }
      }
      flush(); ctx.stroke();
      if (t1 - t0 < 0.25) { ctx.fillStyle = sensorColor(n); for (let i = 0; i < l.t.length; i++) if (l.t[i] >= t0 && l.t[i] <= t1) ctx.fillRect(X(l.t[i]) - 1.5, Y(l.v[i]) - 1.5, 3, 3); }
    });
    ctx.restore();
  },
  drawOverlays() {
    this.cards.forEach(c => {
      const canvas = c.over, rect = canvas.getBoundingClientRect(), L = c.layout; if (!rect.width || !L) return;
      const k = window.devicePixelRatio || 1; canvas.width = rect.width * k; canvas.height = rect.height * k;
      const ctx = canvas.getContext("2d"); ctx.scale(k, k);
      const X = t => L.left + (t - L.t0) / (L.t1 - L.t0) * L.width, Y = v => L.top + (L.hi - v) / (L.hi - L.lo) * L.height;
      if (this.drag && this.hover != null) {
        const a = X(this.drag.x0), b = X(this.hover);
        ctx.fillStyle = "rgba(128,128,255,0.18)"; ctx.fillRect(Math.min(a, b), L.top, Math.abs(b - a), L.height);
      }
      const at = this.hover;
      c.legend.replaceChildren(...L.lines.map((l, n) => {
        let text = `● ${l.name}`;
        if (at != null && l.t.length) { const i = nearest(l.t, at); text += ` ${l.v[i].toFixed(c.spec.digits)}`; }
        return el("span", {style: `color:${sensorColor(n)}`, text});
      }));
      if (at == null) return;
      ctx.strokeStyle = pal().muted; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(X(at), L.top); ctx.lineTo(X(at), L.top + L.height); ctx.stroke(); ctx.setLineDash([]);
      L.lines.forEach((l, n) => { if (!l.t.length) return; const i = nearest(l.t, at); ctx.fillStyle = sensorColor(n); ctx.beginPath(); ctx.arc(X(l.t[i]), Y(l.v[i]), 3.5, 0, 2 * Math.PI); ctx.fill(); });
      ctx.fillStyle = pal().text; ctx.font = pal().font(10); ctx.fillText(`${at.toFixed(3)} s`, Math.min(X(at) + 5, L.left + L.width - 50), L.top + 11);
    });
  },
};
function showLog(result) {
  if (result.kind === "sensortest") return showSensorLog(result);
  $("log-range").hidden = false; $("log-events").className = "events"; $("log-result").classList.remove("sensor-mode"); $("log-title").previousElementSibling.textContent = "TIMELINE";
  logData = result; $("log-result").hidden = false;
  const s = result.summary, tile = (label, value, cls = "") => { const t = el("div", {class: cls}, el("span", {text: label}), el("strong", {text: value})); return t; };
  $("log-summary").replaceChildren(tile("SESSION", String(result.session)), tile("RECORDS", String(result.count)), tile("DURATION", `${s.duration_s.toFixed(1)} s`),
    tile("FLEW", s.flew ? "yes" : "no"), tile("MAX ALTITUDE", `${s.max_alt_m.toFixed(1)} m`),
    tile("WORST BOOST TILT", s.worst_boost_tilt_deg == null ? "—" : `${s.worst_boost_tilt_deg.toFixed(1)}°`, s.worst_boost_tilt_deg > 15 ? "bad" : ""),
    tile("PARACHUTE", s.chute_open_s == null ? "not opened" : `${s.chute_open_s.toFixed(2)} s`), tile("CONTROLLER FAULT", s.fault_s == null ? "none" : `${s.fault_s.toFixed(2)} s`, s.fault_s == null ? "" : "bad"));
  $("log-title").textContent = `Session ${result.session}`;
  $("log-header").textContent = `${result.header}  ·  ${result.source}`;
  $("log-events").replaceChildren(...result.events.map(e => el("li", {class: /FAULT/.test(e.event) ? "bad" : ""}, el("time", {text: `${e.t.toFixed(2)} s`}), el("span", {text: e.event}))));
  const sessions = $("log-sessions"); sessions.replaceChildren();
  if (result.available_sessions && result.available_sessions.length > 1) {
    sessions.append(el("span", {class: "help", text: "Other sessions on this card: "}));
    result.available_sessions.slice().reverse().forEach(n => sessions.append(el("button", {class: n === result.session ? "small selected" : "small quiet", text: String(n),
      onclick: () => { $("session-number").value = n; $("sd-read").click(); }})));
  }
  drawLogCharts();
}
$("log-range").onchange = () => { if (logData && logData.kind !== "sensortest") drawLogCharts(); };
function logWindow() {
  const cols = logData.columns, t = cols.indexOf("t_s"), st = cols.indexOf("state"), rows = logData.rows;
  if ($("log-range").value === "all" || !rows.length) return rows;
  const first = rows.findIndex(r => r[st] !== "IDLE");
  if (first < 0) return rows;
  const boost = rows.findIndex(r => r[st] === "BOOST");
  const start = (boost >= 0 ? rows[boost][t] : rows[first][t]) - 2, end = start + 40;
  return rows.filter(r => r[t] >= start && r[t] <= end);
}
function drawLogCharts() {
  if (!logData) return;
  if (logData.kind === "sensortest") return sensorView.drawAll();
  const cols = logData.columns, rows = logWindow(), col = name => cols.indexOf(name);
  const charts = [["Tilt (°)", ["tilt_x_deg", "tilt_y_deg"], ["X", "Y"]], ["TVC servos (µs)", ["servo_a_us", "servo_b_us"], ["A", "B"]],
    ["Rate (°/s)", ["gyro_x_dps", "gyro_y_dps", "gyro_z_dps"], ["X", "Y", "Z"]], ["Acceleration (g)", ["accel_x_g", "accel_y_g", "accel_z_g"], ["X", "Y", "Z (nose)"]],
    ["Altitude (m) · vertical speed (m/s)", ["baro_alt_m", "vspeed_m_s"], ["altitude", "v speed"]]];
  const wrap = $("log-charts"); wrap.replaceChildren();
  const events = logData.events;
  charts.forEach(([title, keys, names]) => {
    const canvas = el("canvas", {class: "log-chart", role: "img", "aria-label": title});
    const legend = el("div", {class: "legend"}, ...names.map((n, i) => el("span", {style: `color:${pal().series[i]}`, text: `● ${n}`})));
    wrap.append(el("article", {class: "card chart-card"}, el("div", {class: "card-heading"}, el("h2", {text: title}), legend), canvas));
    requestAnimationFrame(() => lineChart(canvas, rows.map(r => r[col("t_s")]), keys.map(k => rows.map(r => r[col(k)])), events));
  });
}
function lineChart(canvas, t, series, events) {
  const rect = canvas.getBoundingClientRect(); if (!rect.width || !t.length) return;
  const scale = window.devicePixelRatio || 1; canvas.width = rect.width * scale; canvas.height = rect.height * scale;
  const ctx = canvas.getContext("2d"); ctx.scale(scale, scale);
  const left = 46, top = 8, width = rect.width - 56, height = rect.height - 30;
  const t0 = t[0], t1 = Math.max(t[t.length - 1], t0 + 0.1);
  let lo = Math.min(...series.flat()), hi = Math.max(...series.flat());
  if (hi - lo < 1e-6) { lo -= 1; hi += 1; } const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
  const X = v => left + (v - t0) / (t1 - t0) * width, Y = v => top + (hi - v) / (hi - lo) * height;
  ctx.font = pal().font(10); ctx.fillStyle = pal().muted; ctx.strokeStyle = pal().grid; ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) { const v = hi - i * (hi - lo) / 4, y = Y(v); ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(left + width, y); ctx.stroke(); ctx.fillText(v.toFixed(Math.abs(hi - lo) < 10 ? 1 : 0), 2, y + 3); }
  for (let i = 0; i <= 5; i++) { const v = t0 + i * (t1 - t0) / 5; ctx.fillText(`${v.toFixed(1)}s`, X(v) - 10, top + height + 18); }
  let lastX = -1e9, row = 0;
  events.forEach(e => { if (e.t < t0 || e.t > t1) return; ctx.strokeStyle = /FAULT/.test(e.event) ? "#c0392b" : pal().line2; ctx.setLineDash([3, 3]);
    const x = X(e.t); row = x - lastX < 80 ? row + 1 : 0; lastX = x;
    ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, top + height); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = pal().muted; ctx.fillText(e.event, x + 3, top + 10 + row * 11); });
  series.forEach((values, i) => { ctx.strokeStyle = pal().series[i]; ctx.lineWidth = 1.6; ctx.beginPath();
    const step = Math.max(1, Math.floor(values.length / (width * 2)));
    for (let j = 0; j < values.length; j += step) { const x = X(t[j]), y = Y(values[j]); if (j) ctx.lineTo(x, y); else ctx.moveTo(x, y); }
    ctx.stroke(); });
}
action("log-csv", async () => {
  if (!logData) throw new Error("Read a session first.");
  if (logData.kind === "sensortest") {
    const out = await api("logs/sensor-csv", {id: logData.data_id});
    download(out.name, "text/csv", out.csv);
    return notice(`Exported ${out.rows.toLocaleString()} readings to ${out.name} (Downloads).`);
  }
  download(`easytvc_session_${logData.session}.csv`, "text/csv", logData.columns.join(",") + "\n" + logData.rows.map(r => r.join(",")).join("\n"));
});

/* ------------------------------------------------------------------ start-up */
window.onTabShown = id => {
  if (id === "live") { api("monitor/status", {}).then(s => { liveStatus = s; renderLive(); if (s.connected) pollLive(); }).catch(e => notice(e.message)); }
  if (id === "logs") refreshDisks().catch(e => notice(e.message));
  if (id === "flash") api("build/status", {}).then(b => { buildInfo = b; renderImages(); }).catch(e => notice(e.message));
  if (id === "logs" && logData) drawLogCharts();
};
window.addEventListener("resize", () => { if (logData && !$("logs").hidden) drawLogCharts(); if (!$("rocket").hidden) drawSideView(); });
(async () => { await window.studioReady; await loadRocket(); drawBubble(null, null); })();

/* ------------------------------------------------------------------ one-click install
 * Save the setup (and the tune) -> build both images -> wait for the board in DFU -> flash with
 * read-back verification. The firmware has no USB serial yet, so the board has to be put into
 * DFU by hand (BOOT + RST); everything else is automatic. */
let installCancel = false;
function installDialog(title) {
  $("modal-title").textContent = title;
  $("modal").hidden = false;
  const steps = [["save", "Save your setup to vehicle_config.h"], ["build", "Build the firmware"],
                 ["find", "Find the board in DFU mode"], ["flash", "Install and verify every byte"]];
  const list = el("ol", {class: "install-steps"}, ...steps.map(([id, text]) => el("li", {id: `install-${id}`, class: "pending"}, el("span", {text}))));
  const detail = el("div", {id: "install-detail", class: "help"});
  $("modal-body").replaceChildren(list, detail);
  $("modal-ok").hidden = true; $("modal-cancel").textContent = "Cancel";
  $("modal-cancel").onclick = () => { installCancel = true; };
  return {
    step(id, state, text) { const li = $(`install-${id}`); li.className = state; if (text !== undefined) li.querySelector("span").textContent = text; },
    detail(nodes) { detail.replaceChildren(...[].concat(nodes)); },
    finish(ok) { $("modal-cancel").textContent = ok ? "Done" : "Close"; $("modal-cancel").onclick = () => { $("modal").hidden = true; $("modal-ok").hidden = false; }; },
  };
}
async function installOnBoard(source) {
  if (!cfg) throw new Error("Rocket setup is not loaded yet.");
  if (source === "tune") {
    /* copy the Tuning lab's gains into the setup, like "Use in rocket setup" */
    const x = profile.axes[0];
    Object.assign(cfg, {KP: x.kp, KI: x.ki, KD: x.kd, INTEGRATOR_LIMIT: x.integrator_limit, OUTPUT_RATE_LIMIT: x.output_rate_limit,
      DERIVATIVE_CUTOFF: x.derivative_cutoff_hz, MAX_TILT_DEG: profile.maximum_tilt_deg});
    renderRocket();
  }
  await api("rocket/validate", {config: cfg});

  /* choose the image */
  const benchOpen = sectionMissing("bench").length;
  const choice = el("div", {class: "image-choice"});
  const option = (value, title, text, checked) => {
    const radio = el("input", {type: "radio", name: "install-image", value}); radio.checked = checked;
    return el("label", {class: "choice"}, radio, el("div", {}, el("strong", {text: title}), el("p", {text})));
  };
  choice.append(option("sensortest", "Sensor test", "Checks every sensor (IMU, barometer, temperatures, SD card, power) and records every reading to the SD card. Never arms; servos stay still.", source === "sensortest"),
    option("bench", "Bench image (recommended for testing)", "Never arms. The gimbal follows the board by hand, so you can check the servos.", source !== "sensortest"),
    option("flight", "Flight image", benchOpen ? `Arms itself on the pad. ${benchOpen} bench check${benchOpen > 1 ? "s are" : " is"} still open.` : "Arms itself on the pad and runs TVC during boost.", false));
  const tune = el("p", {class: "help", text: `Installs: KP ${cfg.KP} · KI ${cfg.KI} · KD ${cfg.KD} · servo A ${cfg.SERVO_A_MIN_US}/${cfg.SERVO_A_CENTER_US}/${cfg.SERVO_A_MAX_US} µs · servo B ${cfg.SERVO_B_MIN_US}/${cfg.SERVO_B_CENTER_US}/${cfg.SERVO_B_MAX_US} µs · nose ${cfg.NOSE_AXIS} · LED ${cfg.LED_BRIGHTNESS_PCT} %`});
  if (!await confirmModal("Install on board", [choice, tune, el("p", {class: "help", text: "Plug the board in by USB-C. If it isn't in DFU mode yet, you'll be told how."})], "Install")) return;
  const image = document.querySelector('input[name="install-image"]:checked').value;
  if (image === "flight" && benchOpen && !await confirmModal("Bench checks are not done", [el("p", {class: "warn-text", text: "The flight image arms itself when it stands still and upright. Finish the bench checks first unless you know what you're doing."})], "Install flight image anyway")) return;

  installCancel = false;
  const ui = installDialog(`Installing the ${IMAGE_NAMES[image].toLowerCase()}`);
  try {
    ui.step("save", "running");
    if (!same(cfg, savedCfg)) {
      const saved = await api("rocket/save", {config: cfg});
      cfg = saved.config; savedCfg = structuredClone(cfg); buildInfo = saved.build; renderRocket(); syncTuningFromRocket();
      ui.step("save", "done", "Setup saved to vehicle_config.h");
    } else ui.step("save", "done", "Setup already saved");

    ui.step("build", "running", "Building the firmware (about 20 s)…");
    const built = await api("build/run", {});
    buildInfo = built; renderImages();
    if (!built.ok) { ui.step("build", "failed", "Build failed"); ui.detail(el("pre", {class: "log", text: built.log.slice(-3000)})); ui.finish(false); return; }
    const target = built.images[image];
    ui.step("build", "done", `Built ${image} image · ${(target.size / 1024).toFixed(1)} KB · ${target.sha256.slice(0, 12)}…`);

    ui.step("find", "running", "Looking for the board…");
    let devices = [];
    for (let tries = 0; !installCancel; tries++) {
      devices = (await api("dfu/list", {})).devices || [];
      if (devices.length) break;
      if (tries === 0) ui.detail([el("strong", {text: "Put the board in DFU mode:"}),
        el("ol", {class: "steps"}, el("li", {text: "Unplug the LiPo."}), el("li", {text: "Hold the BOOT button."}),
          el("li", {text: "Plug in USB-C, or tap RST."}), el("li", {text: "Let go of BOOT. Studio continues by itself."}))]);
      ui.step("find", "running", `Waiting for the board in DFU mode… (${tries + 1})`);
      await new Promise(r => setTimeout(r, 1500));
      if (tries > 200) throw new Error("No board appeared in DFU mode within 5 minutes.");
    }
    if (installCancel) { ui.step("find", "failed", "Cancelled. Nothing was written to the board."); ui.finish(false); return; }
    if (devices.length > 1) { ui.step("find", "failed", "More than one board in DFU mode: unplug the others."); ui.finish(false); return; }
    ui.step("find", "done", `Found board ${devices[0].serial}`);
    ui.detail([]);

    ui.step("flash", "running", "Installing… do not unplug (about 10 s)");
    const result = await api("dfu/flash", {image, sha256: target.sha256, serial: devices[0].serial});
    appState.flashed = {image, sha256: target.sha256, time: Date.now() / 1000};
    if (image === "bench") toggleCheck("b-flash", true); else if (image === "flight") toggleCheck("p-flight", true);
    renderImages(); renderChecklists();
    ui.step("flash", "done", "Installed and verified: every byte matches");
    if (image === "sensortest") ui.detail([...SENSOR_TEST_STEPS, el("details", {}, el("summary", {text: "Install log"}), el("pre", {class: "log", text: result.log}))]);
    else ui.detail([el("strong", {text: "Now restart the board:"}),
      el("p", {text: "Unplug USB-C, wait 2 s, plug it back in (connect the LiPo first if servos are plugged in). At power-up the LED shows red → green → blue, then the servos sweep."}),
      el("details", {}, el("summary", {text: "Install log"}), el("pre", {class: "log", text: result.log}))]);
    ui.finish(true);
  } catch (error) {
    for (const id of ["save", "build", "find", "flash"]) { const li = $(`install-${id}`); if (li && li.className === "running") ui.step(id, "failed"); }
    ui.detail(el("p", {class: "warn-text", text: error.message}));
    ui.finish(false);
  }
}
action("install-board", () => installOnBoard("setup"));
action("install-sensortest", () => installOnBoard("sensortest"));
action("rocket-install", () => installOnBoard("setup"));
action("tune-install", () => installOnBoard("tune"));

/* redraw the static canvases when the appearance changes (3D views redraw every frame anyway) */
window.addEventListener("easytvc-theme", () => {
  try { draw(); } catch (error) { /* tuning chart not ready */ }
  if (rocketModel) drawSideView();
  if (logData) drawLogCharts();
  if (liveStatus) renderLive(); else drawBubble(null, null);
  if (typeof drawSimCharts === "function") drawSimCharts();
});

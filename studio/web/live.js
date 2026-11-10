"use strict";
/* Live: the ground link (USB serial, or DroneBridge Wi-Fi over UDP / TCP). Dashboard: mission
 * clock with the parachute and apogee countdowns, readouts, 3D attitude, events and 30 s plots on
 * one screen. Pad tools: board status, config check, log download and the TVC gimbal test.
 * The link runs in Studio's server (studio/groundlink.py: reader thread, recorder, link stats);
 * this page only polls it. Uses el, api, action, notice, confirmModal, lineChart, pal, cfg,
 * vehicle, rocketModel, appState, saveState, liveStatus, renderLive, showLivePorts, showTab,
 * showLog (launch.js) and rocketMesh, qrot and the vector helpers (sim3d.js). */

const LT = {kind: "udp", timer: null, next: null, hist: [], lastMs: null, raf: null, lastDraw: 0,
            evKey: "", padKey: "", yaw: -0.9, pitch: 0.3, drag: null, boardCfg: null, cfgTried: false,
            countEnd: null, countTimer: null, maxAlt: null, apogeeAlt: null,
            defaults: {host: "192.168.2.1", udp_port: 14550, tcp_port: 5760}};
const LT_WINDOW_S = 30, LT_POLL_MS = 100, LT_DRAW_MS = 50;      /* plots at 20 fps at most */
const BATT_WARN_V = 7.0, LT_STALE_S = 1.0;
const LT_KIND_NAMES = {ble: "Bluetooth", serial: "USB serial", udp: "Wi-Fi · UDP", tcp: "Wi-Fi · TCP"};
const LT_HELP = {
  udp: "Single-board setup: the XIAO in the rocket runs DroneBridge as a Wi-Fi access point and this Mac joins its Wi-Fi. Bridge 192.168.2.1, port 14550 (Defaults fills them in).",
  tcp: "Same single-board Wi-Fi access point, over TCP: bridge 192.168.2.1, port 5760.",
  ble: "Bluetooth LE to the DroneBridge (BLE mode). Short range, and needs the Studio app.",
  serial: "A 3.3 V USB-UART on TX2/RX2 (115200 baud), or an ESP-NOW ground unit plugged in by USB if you add one later."};

/* ---- settings (kept in Studio's state file: the Mac app's web view forgets local storage) ---- */
function ltSettings() { return (appState.ui && appState.ui.live) || {}; }
async function ltSave(update) {
  try { await saveState({ui: {live: {...ltSettings(), ...update}}}); } catch (error) { /* settings are a convenience */ }
}

/* ---- connection ---- */
function ltShowKind(kind) {
  LT.kind = kind;
  document.querySelectorAll("#link-kind button").forEach(b => b.classList.toggle("selected", b.dataset.kind === kind));
  $("link-serial").hidden = kind !== "serial";
  $("link-net").hidden = kind === "serial" || kind === "ble";
  $("link-ble").hidden = kind !== "ble";
  $("link-listen-wrap").hidden = kind !== "udp";
  if (kind !== "serial") {
    const saved = ltSettings()[kind] || {};
    $("link-host").value = saved.host || LT.defaults.host;
    $("link-port").value = saved.port || (kind === "udp" ? LT.defaults.udp_port : LT.defaults.tcp_port);
    $("link-listen").value = saved.listen_port || LT.defaults.udp_port;
  }
  if (!liveStatus || !liveStatus.connected) $("live-state").textContent = LT_HELP[kind];
  ltRenderConnect(liveStatus || {connected: false});
}
document.querySelectorAll("#link-kind button").forEach(b => b.onclick = () => { ltShowKind(b.dataset.kind); ltSave({kind: b.dataset.kind}); });
action("link-defaults", () => {
  $("link-host").value = LT.defaults.host;
  $("link-port").value = LT.kind === "udp" ? LT.defaults.udp_port : LT.defaults.tcp_port;
  $("link-listen").value = LT.defaults.udp_port;
});
action("live-refresh", async () => showLivePorts((await api("monitor/status", {since: Number.MAX_SAFE_INTEGER})).ports));
action("live-connect", async () => {
  const body = {kind: LT.kind};
  if (LT.kind === "ble") {
    /* nothing to fill in: the relay in the Studio app finds the DroneBridge */
  } else if (LT.kind === "serial") {
    body.port = $("live-port").value;
    if (!body.port) throw new Error("Plug in the USB-UART adapter (or the DroneBridge ground unit) and press Refresh.");
  } else {
    body.host = $("link-host").value.trim();
    body.port = parseInt($("link-port").value, 10);
    if (LT.kind === "udp") body.listen_port = parseInt($("link-listen").value, 10);
    if (!body.host) throw new Error("Enter the bridge's address (DroneBridge: 192.168.2.1).");
  }
  ltSave({kind: LT.kind, ...(LT.kind === "serial" || LT.kind === "ble" ? {} : {[LT.kind]: {host: body.host, port: body.port, listen_port: body.listen_port}})});
  ltReset();
  liveStatus = await api("monitor/connect", body);
  ltIngest(liveStatus); ltRender(); ltPoll();
});
action("live-disconnect", async () => {
  clearTimeout(LT.timer);
  liveStatus = await api("monitor/disconnect", {});
  ltRender();
});

/* ---- views: one-screen dashboard, or the pad tools ---- */
function ltShowView(view) {
  document.querySelectorAll("#live-view button").forEach(b => b.classList.toggle("selected", b.dataset.view === view));
  $("live-dash").hidden = view !== "dash"; $("live-pad").hidden = view !== "pad";
  $("live").classList.toggle("dash-mode", view === "dash");
  ltRequestDraw();
}
document.querySelectorAll("#live-view button").forEach(b => b.onclick = () => { ltShowView(b.dataset.view); ltSave({view: b.dataset.view}); });

/* ---- how to connect ---- */
const LT_STEPS = {
  udp: ["In the rocket: XIAO ESP32-C3 wired 5V → 5V, GND → GND, board TX2 → XIAO D2 (GPIO4), board RX2 → XIAO D3 (GPIO5).",
    "DroneBridge on the XIAO: mode Wi-Fi Access Point, UART 115200, TX pin 5, RX pin 4, flow control off, protocol Transparent. No ground unit.",
    "Power the rocket, then join its Wi-Fi on this Mac (DroneBridge's network; password dronebridge unless you changed it). While joined, the Mac has no internet over Wi-Fi.",
    "Pick Wi-Fi UDP: bridge 192.168.2.1, port 14550, listen 14550 (Defaults fills them in), then Connect. Studio pings the bridge every second so it knows where to send."],
  tcp: ["Same single-board setup as UDP: the XIAO is a Wi-Fi access point and this Mac joins it.",
    "Pick Wi-Fi TCP: bridge 192.168.2.1, port 5760, then Connect."],
  ble: ["DroneBridge mode Bluetooth LE instead of Wi-Fi (same wiring and UART settings).",
    "Turn Bluetooth on; the first time, macOS asks to let EasyTVC Studio use it: allow it.",
    "Pick Bluetooth and Connect: Studio finds the DroneBridge by itself. Range is short (tens of metres)."],
  serial: ["A 3.3 V USB-UART on the board: its RX → TX2, its TX → RX2, GND → GND. Or, later, an ESP-NOW ground unit (DroneBridge ESP-NOW LR GND, with the rocket's XIAO set to ESP-NOW LR AIR) plugged in by USB.",
    "Pick USB, choose the port (↻ looks again) and press Connect."]};
function ltHowTo(kind) {
  return [el("ol", {}, ...LT_STEPS[kind].map(t => el("li", {text: t}))),
    el("p", {class: "help", text: "Connected means Studio is receiving: the pill turns green and frames/s counts up. STALE means nothing for over 1 s (out of range, or the board is off). Studio records every frame; losing the link has no effect on the rocket."})];
}
action("link-help", () => confirmModal(`How to connect · ${LT_KIND_NAMES[LT.kind]}`, ltHowTo(LT.kind), "OK"));
function ltRenderConnect(s) {
  const box = $("dash-connect");
  box.hidden = !!s.connected;
  if (s.connected || box.dataset.kind === LT.kind) return;
  box.dataset.kind = LT.kind;
  box.replaceChildren(el("div", {class: "connect-card"}, el("span", {class: "eyebrow", text: "NOT CONNECTED"}),
    el("h2", {text: `Connect over ${LT_KIND_NAMES[LT.kind]}`}), ...ltHowTo(LT.kind),
    el("p", {class: "help", text: "Change the link type in the bar above. The dashboard fills in as soon as frames arrive."})));
}

/* ---- polling: the server keeps the history; we ask only for what's new ---- */
async function ltPoll() {
  clearTimeout(LT.timer);
  try {
    const s = await api("monitor/status", LT.next == null ? {} : {since: LT.next});
    liveStatus = s; ltIngest(s); ltRender();
  } catch (error) { notice(error.message); LT.timer = setTimeout(ltPoll, 1000); return; }
  /* a hidden page polls slowly, just enough to keep the sidebar badge current */
  if (liveStatus.connected) LT.timer = setTimeout(ltPoll, $("live").hidden ? 1000 : LT_POLL_MS);
}
window.liveShown = () => { ltPoll(); };

function ltReset() {
  LT.hist = []; LT.next = null; LT.lastMs = null; LT.evKey = LT.padKey = "";
  LT.boardCfg = null; LT.cfgTried = false; LT.maxAlt = LT.apogeeAlt = null;
}
function ltIngest(s) {
  if (s.history_next != null) {
    if (LT.next != null && s.history_next < LT.next) LT.hist = [];   /* a new connection */
    LT.next = s.history_next;
  }
  for (const h of s.history || []) {
    if (LT.lastMs != null && h.ms < LT.lastMs - 1000) { LT.hist = []; LT.maxAlt = LT.apogeeAlt = null; }   /* the board rebooted */
    LT.lastMs = h.ms; LT.hist.push(h);
    if (h.since_launch_s != null) LT.maxAlt = Math.max(LT.maxAlt ?? h.alt_m, h.alt_m);
  }
  if (LT.lastMs == null) return;
  const cut = LT.lastMs - LT_WINDOW_S * 1000;
  let i = 0; while (i < LT.hist.length && LT.hist[i].ms < cut) i++;
  if (i) LT.hist.splice(0, i);
}

/* ---- DOM, once per poll ---- */
function ltRender() {
  const s = liveStatus; if (!s) return;
  renderLive();                       /* status tiles, bubble, nose axis, sensor test (launch.js) */
  ltRenderLink(s); ltRenderConnect(s); ltRenderReadouts(s); ltRenderClock(s); ltRenderEvents(s); ltRenderPad(s);
  ltRequestDraw();
  /* once per connection, on the pad: read the board's compiled-in recovery plan for the countdown */
  if (s.connected && s.idle && !LT.cfgTried) {
    LT.cfgTried = true;
    api("monitor/config", {}).then(r => { LT.boardCfg = r.board; }).catch(() => { LT.cfgTried = false; });
  }
}
const ltChip = (text, tone = "") => el("span", {class: `chip ${tone}`, text});
function ltRenderLink(s) {
  $("live-connect").disabled = s.connected; $("live-disconnect").disabled = !s.connected;
  document.querySelectorAll("#link-kind button, #link-net input, #link-defaults, #live-port, #live-refresh").forEach(e => e.disabled = s.connected);
  if (s.ports) showLivePorts(s.ports, s.transport === "serial" ? s.port : null);
  const L = s.link || {};
  $("link-title").textContent = !s.connected ? "Not connected" :
    !L.frames ? (s.ble && s.ble.state !== "connected" ? "Bluetooth · " + (BLE_TEXT[s.ble.state] || [s.ble.state])[0] : "Connected · waiting for the board") : L.stale ? "Link lost · STALE" : `Connected · ${L.per_s} frames/s`;
  $("link-pill").className = `link-pill ${!s.connected ? "off" : !L.frames ? "wait" : L.stale ? "bad" : "ok"}`;
  const chips = [];
  if (s.connected) {
    chips.push(ltChip(L.age_s == null ? "no packets yet" : L.stale ? `STALE · last ${L.age_s.toFixed(1)} s ago` : `last ${Math.round(L.age_s * 1000)} ms ago`,
      L.age_s == null ? "" : L.stale ? "bad" : "good"));
    chips.push(ltChip(`${L.per_s} frames/s`));
    chips.push(ltChip(`loss ${L.loss_pct.toFixed(1)} %`, L.loss_pct > 20 ? "bad" : L.loss_pct > 5 ? "warn" : ""));
    chips.push(ltChip(`${L.frames} received · ${L.lost} lost · ${L.bad} damaged`));
    if (L.rtt_ms != null) chips.push(ltChip(`ping ${Math.round(L.rtt_ms)} ms`));
    if (s.recording) chips.push(ltChip(`● REC ${s.recording.file} · ${s.recording.lines} lines`, "rec"));
  }
  $("link-chips").replaceChildren(...chips);
  ltRenderBle(s);
  $("live-state").textContent = s.connected
    ? `${LT_KIND_NAMES[s.transport] || s.transport} · ${s.port} · recording every frame (open it later in Flight logs). Losing the link has no effect on the rocket.`
    : LT_HELP[LT.kind];
}

const BLE_TEXT = {starting: ["Bluetooth starting…", ""], ready: ["Bluetooth ready", ""], scanning: ["Looking for the DroneBridge…", "warn"],
  connecting: ["Connecting…", "warn"], discovering: ["Connecting…", "warn"], connected: ["Bluetooth connected", "good"],
  idle: ["Bluetooth idle", ""], bluetooth_off: ["Bluetooth is off on this Mac", "bad"], unsupported: ["No Bluetooth LE on this Mac", "bad"],
  not_allowed: ["Allow Bluetooth: System Settings → Privacy & Security → Bluetooth", "bad"],
  wrong_device: ["Found a device that isn't a DroneBridge in BLE mode", "bad"],
  no_relay: ["Bluetooth needs the EasyTVC Studio app (not a browser)", "bad"]};
function ltRenderBle(s) {
  const chip = $("ble-state");
  if (LT.kind !== "ble") return;
  if (!s.connected || !s.ble) { chip.textContent = "Bluetooth"; chip.className = "chip"; return; }
  const [text, tone] = BLE_TEXT[s.ble.state] || [s.ble.state, ""];
  chip.textContent = s.ble.state === "connected" ? `${s.ble.name || "DroneBridge"}${s.ble.rssi != null ? ` · ${s.ble.rssi} dBm` : ""}` : text;
  chip.className = `chip ${s.ble.state === "connected" && s.ble.rssi != null && s.ble.rssi < -88 ? "warn" : tone}`;
  chip.title = s.ble.state === "connected" ? "Signal: about −60 dBm is strong, below −88 dBm the link starts dropping" : "";
}

const STATE_TONE = {ARMED: "warn", BOOST: "hot", COAST: "hot", APOGEE: "hot", DESCENT: "good", LANDED: "good", FAULT: "bad"};
function ltReadout(label, value, sub = "", tone = "") {
  return el("div", {class: `readout ${tone}`}, el("span", {text: label}), el("strong", {text: value}), el("small", {text: sub}));
}
function ltBatteryVolts(sys) { const r = ltSettings().batt_ratio; return r ? sys.batt_pin_v * r : null; }
function ltRenderReadouts(s) {
  const t = s.latest.tlm, y = s.latest.sys, ageT = s.age.tlm;
  const stale = ageT == null || ageT > LT_STALE_S;
  $("tlm-age").textContent = ageT == null ? "no data" : stale ? `STALE ${ageT.toFixed(1)} s` : "live";
  $("tlm-age").className = `chip ${ageT == null ? "" : stale ? "bad" : "good"}`;
  const state = t ? t.state : y ? y.state : null;
  $("tlm-headline").textContent = !state ? (s.connected ? "Waiting for telemetry…" : "Not connected")
    : state === "ARMED" ? "ARMED · hands off" : state === "IDLE" ? "IDLE · on the pad" : state;
  const box = $("tlm-readouts"); box.classList.toggle("stale", stale);
  const dash = v => v == null ? "—" : v;
  const tilt = t ? Math.hypot(t.tilt_x, t.tilt_y) : null, limit = cfg ? cfg.MAX_TILT_DEG : 25;
  const g = t ? Math.hypot(...t.accel_g) : null, v = y ? ltBatteryVolts(y) : null;
  const armed = t ? t.flags.armed : y ? !!y.armed : false;
  const plan = ltRecoveryPlan();
  const pyro = !y ? "—" : y.continuity.map((c, i) => `P${i + 1}${c ? "●" : "○"}`).join(" ");
  const w = t ? t.gyro_dps : null, rate = w ? Math.hypot(...w) : null;
  box.replaceChildren(
    ltReadout("State", dash(state), armed ? "ARMED · hands off the rocket" : y ? `${y.image} image${t && t.flags.tvc ? " · TVC on" : ""}` : "",
      armed ? "bad" : STATE_TONE[state] || ""),
    ltReadout("Altitude", t ? `${t.alt_m.toFixed(1)} m` : "—", LT.maxAlt != null ? `max ${LT.maxAlt.toFixed(1)} m` : "baro, above the pad"),
    ltReadout("Vertical speed", t ? `${t.vspeed >= 0 ? "+" : "−"}${Math.abs(t.vspeed).toFixed(1)} m/s` : "—"),
    ltReadout("Tilt", t ? `${tilt.toFixed(1)}°` : "—", t ? `X ${t.tilt_x.toFixed(1)}° · Y ${t.tilt_y.toFixed(1)}°` : "", tilt != null && tilt > limit ? "bad" : ""),
    ltReadout("Acceleration", t ? `${g.toFixed(2)} g` : "—", t ? (t.flags.accel_sat ? "SATURATED" : t.accel_g.map(a => a.toFixed(2)).join(" / ")) : "",
      t && t.flags.accel_sat ? "warn" : ""),
    ltReadout("Rotation", w ? `${rate.toFixed(0)}°/s` : "—", w ? w.map(a => a.toFixed(0)).join(" / ") + " °/s X/Y/Z" : "", t && t.flags.gyro_sat ? "warn" : ""),
    ltReadout("Barometer", t && t.pressure_pa ? `${(t.pressure_pa / 100).toFixed(1)} hPa` : "—", t && t.baro_temp_c != null ? `${t.baro_temp_c.toFixed(1)} °C` : t ? "no barometer data" : ""),
    !y ? ltReadout("Battery", "—") : v == null ? ltReadout("Battery", `${y.batt_pin_v.toFixed(2)} V`, "at the pin · set the divider")
      : ltReadout("Battery", `${v.toFixed(2)} V`, v < BATT_WARN_V ? `LOW · under ${BATT_WARN_V.toFixed(1)} V` : `pin ${y.batt_pin_v.toFixed(3)} V`, v < BATT_WARN_V ? "bad" : ""),
    ltReadout("Pyro", pyro, !plan.charge ? "continuity (raw)" : plan.locked ? `charge on P${plan.charge} · outputs locked` : `charge on P${plan.charge} · LIVE`,
      plan.charge && !plan.locked ? "warn" : ""));
}

/* ---- mission clock: T− (a countdown on this Mac only), T+ from the board, parachute and apogee ---- */
function ltRecoveryPlan() {
  /* the board's compiled-in settings when read on the pad, else the saved Rocket setup */
  const c = LT.boardCfg && LT.boardCfg.PYRO_CHUTE_CHANNEL !== undefined ? LT.boardCfg : cfg, fromBoard = c === LT.boardCfg;
  if (!c) return {servo: 0, charge: 0, backup_s: null, delay_s: 0, locked: true, fromBoard};
  const charge = c.PYRO_CHUTE_CHANNEL || 0, servo = c.CHUTE_CHANNEL || 0;
  return {servo, charge, backup_s: (charge && !servo ? (c.PYRO_BACKUP_MS || c.CHUTE_BACKUP_MS) : c.CHUTE_BACKUP_MS) / 1000,
          delay_s: (c.PYRO_CHUTE_DELAY_MS || 0) / 1000, alt_m: c.PYRO_DEPLOY_ALT_M || 0,
          locked: fromBoard ? !c.PYRO_OUTPUTS_COMPILED_IN : !pyroCompiledIn, fromBoard};
}
const clockText = (sign, secs) => {
  const a = Math.abs(secs), m = Math.floor(a / 60), sec = a - m * 60;
  return `T${sign} ${String(m).padStart(2, "0")}:${sec.toFixed(1).padStart(4, "0")}`;
};
function ltSetClock(id, text, sub, tone) {
  $(id).textContent = text;
  if (sub != null) $(id).nextElementSibling.textContent = sub;
  $(id).parentElement.className = $(id).parentElement.className.replace(/ (go|soon|done|hold)$/, "") + (tone ? ` ${tone}` : "");
}
function ltRenderClock(s) {
  const t = s && s.latest.tlm, fresh = !!(t && s.age.tlm <= LT_STALE_S), launched = !!(t && t.since_launch_s != null);
  /* T−/T+ (with the link lost: the last time the board reported, greyed) */
  if (launched) {
    $("clock-label").textContent = fresh ? "MISSION CLOCK · FROM THE BOARD" : "MISSION CLOCK · LAST REPORT (LINK LOST)";
    ltSetClock("clock-time", clockText("+", t.since_launch_s), null, fresh ? "go" : "hold");
    if (LT.countEnd) ltStopCount();
  } else if (LT.countEnd) {
    const left = (LT.countEnd - performance.now()) / 1000;
    $("clock-label").textContent = left > 0 ? "COUNTDOWN · THIS MAC ONLY" : "COUNTDOWN DONE · WAITING FOR LAUNCH DETECT";
    ltSetClock("clock-time", left > 0 ? clockText("−", left) : clockText("+", -left), null, left > 0 && left <= 10 ? "soon" : "");
  } else {
    $("clock-label").textContent = "MISSION CLOCK";
    ltSetClock("clock-time", "T− --:--.-", null, "hold");
  }
  /* parachute */
  const plan = ltRecoveryPlan();
  const how = !plan.servo && !plan.charge ? "motor ejection (nothing from the board)"
    : [plan.servo ? `servo S${plan.servo}` : "", plan.charge ? `charge P${plan.charge}${plan.locked ? " (locked)" : ""}` : ""].filter(Boolean).join(" + ")
      + ` · ${plan.charge && plan.alt_m ? `${plan.alt_m.toFixed(0)} m on the way down` : "apogee"}${plan.charge && plan.delay_s ? ` + ${plan.delay_s.toFixed(1)} s` : ""}`
      + ` or T+${plan.backup_s != null ? plan.backup_s.toFixed(1) : "?"} s`;
  if (plan.charge && !plan.servo && plan.locked) ltSetClock("chute-eta", "NONE (locked)", "charge is the only recovery and pyro outputs are locked: motor ejection only", "soon");
  else if (!plan.servo && !plan.charge) ltSetClock("chute-eta", "—", how, "hold");
  else if (!launched || !t || t.deploy_eta_s == null) ltSetClock("chute-eta", plan.backup_s != null ? `≤ T+${plan.backup_s.toFixed(1)} s` : "—", how, "hold");
  else if (t.deploy_eta_s <= 0) ltSetClock("chute-eta", plan.charge && plan.locked && !plan.servo ? "DUE · locked" : "DEPLOYED", how, "done");
  else ltSetClock("chute-eta", `in ${t.deploy_eta_s.toFixed(1)} s`, how, t.deploy_eta_s < 2 ? "soon" : "go");
  /* apogee: from the board's own events, else predicted from the climb rate */
  const apogeeSeen = t && ["APOGEE", "DESCENT", "LANDED"].includes(t.state);
  if (apogeeSeen && LT.apogeeAlt == null) LT.apogeeAlt = LT.maxAlt;
  if (!launched) ltSetClock("apogee-eta", "—", "after launch", "hold");
  else if (apogeeSeen) ltSetClock("apogee-eta", LT.apogeeAlt != null ? `${LT.apogeeAlt.toFixed(1)} m` : "reached", "reached", "done");
  else if (t.vspeed > 0.5) {
    const tA = t.vspeed / 9.81, h = t.alt_m + t.vspeed * t.vspeed / (2 * 9.81);
    ltSetClock("apogee-eta", `in ~${tA.toFixed(1)} s`, `~${h.toFixed(0)} m (coasting estimate)`, "go");
  } else ltSetClock("apogee-eta", "now", "climb rate ~0", "soon");
}
function ltStopCount() {
  LT.countEnd = null; clearInterval(LT.countTimer);
  $("count-start").hidden = false; $("count-stop").hidden = true;
  ltRenderClock(liveStatus);
}
action("count-start", () => {
  const secs = parseInt($("count-from").value, 10);
  if (!(secs >= 3 && secs <= 600)) throw new Error("Countdown from 3 to 600 seconds.");
  LT.countEnd = performance.now() + secs * 1000;
  $("count-start").hidden = true; $("count-stop").hidden = false;
  clearInterval(LT.countTimer);
  LT.countTimer = setInterval(() => ltRenderClock(liveStatus), 100);   /* runs even with no link */
  ltRenderClock(liveStatus);
});
action("count-stop", ltStopCount);
action("batt-cal", async () => {
  const y = liveStatus && liveStatus.latest.sys;
  if (!y) throw new Error("Connect first: the battery reading comes in the 1 Hz SYS frame.");
  const input = el("input", {type: "number", step: "0.01", min: "1", max: "30", placeholder: "e.g. 8.12"});
  const ratio = ltSettings().batt_ratio;
  const ok = await confirmModal("Battery voltage divider", [
    el("p", {text: "The board reports the voltage at its battery-sense pin (PA4), after the board's divider. Measure the pack with a multimeter now and type it in: Studio works out the ratio and remembers it."}),
    el("p", {class: "help", text: `Pin now: ${y.batt_pin_v.toFixed(3)} V.${ratio ? ` Current ratio ×${ratio.toFixed(3)}.` : ""}`}),
    el("label", {class: "field"}, "Battery voltage (V)", input)], "Save");
  if (!ok) return;
  const measured = parseFloat(input.value), pin = liveStatus.latest.sys.batt_pin_v;
  if (!(measured > 1 && measured < 30)) throw new Error("Enter the measured battery voltage, 1–30 V.");
  if (!(pin > 0.05)) throw new Error("The sense pin reads about 0 V: is the battery plugged in?");
  await ltSave({batt_ratio: measured / pin});
  notice(`Battery divider saved: ×${(measured / pin).toFixed(3)}. Warning below ${BATT_WARN_V.toFixed(1)} V.`);
  ltRender();
});

function ltRenderEvents(s) {
  const events = (s.events || []).slice().reverse();
  const key = events.length ? `${events.length}:${events[0].ms}:${events[0].text}` : "none";
  if (key === LT.evKey) return;
  LT.evKey = key;
  $("evt-count").textContent = String(events.length);
  $("tlm-events").replaceChildren(...(events.length ? events.map(e => el("li", {class: /FAULT|LOST|MISSING|RECOVERY/.test(e.text) ? "bad" : ""},
      el("time", {text: new Date(e.host * 1000).toLocaleTimeString()}), el("span", {class: "evt-board", text: `${(e.ms / 1000).toFixed(2)} s`}),
      el("strong", {text: e.text.replace(/_/g, " ")})))
    : [el("li", {}, el("span", {class: "help-inline", text: "No events yet."}))]));
}

/* ---- pad tools: requests only while the board reports IDLE (the board enforces it too) ---- */
function ltRenderPad(s) {
  const idle = s.connected && !!s.idle, dl = s.download, running = !!(dl && dl.running);
  $("pad-lock").textContent = !s.connected ? "not connected" : idle ? "IDLE · unlocked" : "locked · board not IDLE";
  $("pad-lock").className = `chip ${!s.connected ? "" : idle ? "good" : "bad"}`;
  for (const id of ["pad-status", "pad-config", "pad-download", "pad-beep", "bs-load-board", "bs-send", "bs-reset", "pt-start", "trim-board"])
    $(id).disabled = !idle || running;
  $("bs-chip").textContent = !s.connected ? "not connected" : !idle ? "locked · not IDLE" : LT.boardCfg && LT.boardCfg.RADIO_SETTINGS ? "board uses radio settings" : "board uses installed settings";
  $("bs-chip").className = `chip ${idle ? (LT.boardCfg && LT.boardCfg.RADIO_SETTINGS ? "warn" : "good") : ""}`;
  $("pt-chip").textContent = !s.connected ? "not connected" : !idle ? "locked · not IDLE" : boardPyroLive() ? "outputs LIVE" : "outputs locked";
  $("pt-chip").className = `chip ${idle ? (boardPyroLive() ? "bad" : "good") : ""}`;
  ltRenderGimbal(s, idle && !running);
  const resumable = dl && !running && !dl.done && dl.bytes > 0;
  $("pad-resume").hidden = !resumable; $("pad-resume").disabled = !idle;
  $("pad-cancel").hidden = !running;
  $("pad-progress").hidden = !dl;
  if (!dl) return;
  const sys = s.latest.sys, total = sys && sys.session === dl.session && sys.records ? 512 + sys.records * 32 : null;
  const pct = dl.done ? 100 : total ? Math.min(99, 100 * dl.bytes / total) : 3;
  $("pad-bar").style.width = `${pct}%`;
  const kb = x => (x / 1024).toFixed(1);
  $("pad-progress-text").textContent = dl.done ? `Session ${dl.session}: ${kb(dl.bytes)} KB, every chunk CRC-checked, saved as ${dl.saved}.`
    : dl.error ? `Session ${dl.session}: stopped at ${kb(dl.bytes)} KB. ${dl.error}`
    : !running && dl.cancelled ? `Session ${dl.session}: cancelled at ${kb(dl.bytes)} KB. Resume carries on from there.`
    : `Session ${dl.session}: ${kb(dl.bytes)} KB${total ? ` of ~${kb(total)} KB` : ""} · ${dl.speed_kb_s.toFixed(1)} KB/s`;
  const key = dl.done ? dl.saved : "";
  if (key && key !== LT.padKey) {
    LT.padKey = key;
    $("pad-result").replaceChildren(el("div", {class: "actions"}, el("button", {text: "Open in Flight logs", onclick: async () => {
      try { showTab("logs"); showLog(await api("logs/open-saved", {name: dl.saved})); } catch (error) { notice(error.message); } }})));
  }
}
action("pad-status", async () => {
  const hello = (await api("monitor/request", {command: "HELLO"})).reply.split(",");
  const status = (await api("monitor/request", {command: "STATUS"})).reply.split(",").map(Number);
  const yes = v => v ? "OK" : "FAIL";
  LT.padKey = "";
  $("pad-result").replaceChildren(el("div", {class: "telemetry-grid pad-grid"},
    ...[["Firmware", `${hello[2] || "?"} image v${hello[3] || "?"}`], ["Build", hello[4] || "?"],
        ["State", ["IDLE", "ARMED", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED", "FAULT"][status[0]] || String(status[0])],
        ["Armed", status[1] ? "YES" : "no"], ["Recovery boot", status[2] ? "YES" : "no"], ["IMU", yes(status[3])],
        ["Barometer", yes(status[4])], ["SD card", yes(status[5])]]
      .map(([label, value]) => { const t = textNode("div", label); t.append(textNode("strong", value)); return t; })));
});
action("pad-config", async () => {
  const r = await api("monitor/config", {});
  const differ = r.rows.filter(x => !x.same);
  LT.padKey = "";
  const fmt = v => v == null ? "—" : typeof v === "number" ? String(Math.round(v * 1000) / 1000) : String(v);
  $("pad-result").replaceChildren(
    el("div", {class: `assessment ${differ.length ? "" : "good"}`, text: differ.length
      ? `${differ.length} of ${r.rows.length} settings on the board differ from your Rocket setup. Build and reinstall over USB-C to update the board.`
      : `All ${r.rows.length} settings on the board match your saved Rocket setup.`}),
    el("table", {class: "cfg-table"}, el("thead", {}, el("tr", {}, el("th", {text: "Setting"}), el("th", {text: "On the board"}), el("th", {text: "Rocket setup"}))),
      el("tbody", {}, ...r.rows.map(x => el("tr", {class: x.same ? "" : "differ"}, el("td", {text: x.name}), el("td", {text: fmt(x.board)}), el("td", {text: fmt(x.saved)}))))));
});
action("pad-download", async () => { LT.padKey = ""; $("pad-result").replaceChildren(); liveStatus.download = (await api("monitor/log/start", {})).download; ltRenderPad(liveStatus); });
action("pad-resume", async () => { liveStatus.download = (await api("monitor/log/start", {session: liveStatus.download.session})).download; ltRenderPad(liveStatus); });
action("pad-cancel", async () => { liveStatus.download = (await api("monitor/log/cancel", {})).download; ltRenderPad(liveStatus); });

/* ---- TVC gimbal test (IDLE only; the board refuses it otherwise and ends it on leaving IDLE) ---- */
const GIM = {held: null, timer: null, busy: false, misses: 0, trim: [0, 0], trimWant: null, trimSending: false, trimAt: 0};
function ltRenderGimbal(s, enabled) {
  document.querySelectorAll("#gim-a, #gim-b, #gim-sweep, #gim-center, [data-trim], #trim-reset, #trim-save")
    .forEach(e => e.disabled = !enabled);
  const t = s.latest.tlm, testing = t && t.flags.servo_test;
  /* the board dropped the trim (it restarted or left IDLE): its frames stopped flagging it well
   * after our last reply (a frame sent just before the reply doesn't count) */
  if (!testing && (GIM.trim[0] || GIM.trim[1]) && t && s.age.tlm < LT_STALE_S && performance.now() - GIM.trimAt > 1500) GIM.trim = [0, 0];
  $("gim-chip").textContent = !enabled ? "locked" : testing ? "test active" : "ready";
  $("gim-chip").className = `chip ${!enabled ? "" : testing ? "warn" : "good"}`;
  for (const [n, k] of [[0, "a"], [1, "b"]]) {
    $(`gim-${k}-out`).textContent = t ? `${t.pulse[n]} µs · ${(ltServoFraction(n, t.pulse[n]) * 100).toFixed(0)} %` : "—";
    const trim = (GIM.trimWant || GIM.trim)[n];
    $(`trim-${k}`).textContent = `${trim > 0 ? "+" : ""}${trim} µs`;
  }
}
async function gimCommand(command, a = 0, b = 0) {
  const r = await api("monitor/gimbal", {command, a, b});
  GIM.trim = r.trim_us; GIM.trimAt = performance.now();
  if (liveStatus) ltRenderGimbal(liveStatus, true);
  return r;
}
/* hold-to-move: repeat the position while the slider is held, well inside the board's 1 s hold */
async function gimSend() {
  if (!GIM.held || GIM.busy) return;
  GIM.busy = true;
  try { await gimCommand("SERVO", parseInt($("gim-a").value, 10) * 10, parseInt($("gim-b").value, 10) * 10); GIM.misses = 0; }
  catch (error) {
    /* a lost reply on a lossy link: the next repeat follows in 200 ms. Give up on a refusal
     * (not IDLE any more, say) or when the link is clearly gone. */
    if (!/^No reply/.test(error.message) || ++GIM.misses >= 5) { notice(error.message); gimRelease(); }
  }
  finally { GIM.busy = false; }
}
function gimHold(id) { GIM.held = id; clearInterval(GIM.timer); GIM.timer = setInterval(gimSend, 200); gimSend(); }
function gimRelease() {
  if (!GIM.held) return;
  GIM.held = null; clearInterval(GIM.timer);
  $("gim-a").value = $("gim-b").value = 0;
  gimCommand("CENTER").catch(error => notice(error.message));
}
for (const id of ["gim-a", "gim-b"]) {
  const slider = $(id);
  slider.addEventListener("pointerdown", () => gimHold(id));
  slider.addEventListener("input", () => { if (!GIM.held) gimHold(id); gimSend(); });
  slider.addEventListener("pointerup", gimRelease);
  slider.addEventListener("pointercancel", gimRelease);
  slider.addEventListener("keyup", gimRelease);
  slider.addEventListener("blur", gimRelease);
}
action("gim-sweep", () => gimCommand("SWEEP"));
action("gim-center", () => gimCommand("CENTER"));
/* Quick clicks add up, and go to the board one request at a time, latest value last: requests
 * sent together can arrive out of order and leave the board on an older trim. */
async function trimFlush() {
  if (GIM.trimSending) return;
  GIM.trimSending = true;
  try {
    while (GIM.trimWant) {
      const want = GIM.trimWant;
      await gimCommand("TRIM", want[0], want[1]);
      if (GIM.trimWant === want) GIM.trimWant = null;
    }
  } catch (error) { notice(error.message); GIM.trimWant = null; if (liveStatus) ltRenderGimbal(liveStatus, true); }
  finally { GIM.trimSending = false; }
}
function trimTo(trim) { GIM.trimWant = trim; if (liveStatus) ltRenderGimbal(liveStatus, true); trimFlush(); }
document.querySelectorAll("[data-trim]").forEach(b => b.addEventListener("click", () => {
  const [n, step] = b.dataset.trim.split(",").map(Number), trim = [...(GIM.trimWant || GIM.trim)];
  trim[n] = Math.max(-100, Math.min(100, trim[n] + step));
  trimTo(trim);
}));
action("trim-reset", () => trimTo([0, 0]));
action("trim-save", async () => {
  if (GIM.trimWant || GIM.trimSending) throw new Error("Still sending the trim to the board: try again in a moment.");
  if (!GIM.trim[0] && !GIM.trim[1]) throw new Error("No trim to save: nudge a servo first.");
  /* the trim is relative to the centre compiled into the board, which may differ from Rocket setup */
  const board = (await api("monitor/config", {})).board, trim = [...GIM.trim];
  const next = ["A", "B"].map((k, n) => board[`SERVO_${k}_CENTER_US`] + trim[n]);
  const rows = ["A", "B"].map((k, n) => el("li", {text: `Servo ${k}: centre ${board[`SERVO_${k}_CENTER_US`]} → ${next[n]} µs (${trim[n] >= 0 ? "+" : ""}${trim[n]})`}));
  if (!await confirmModal("Save the trim to Rocket setup?", [el("ul", {}, ...rows),
      el("p", {text: "Rocket setup gets the new centres. The board keeps flying its compiled-in centres until you build and install over USB-C."})], "Save")) return;
  ["A", "B"].forEach((k, n) => { cfg[`SERVO_${k}_CENTER_US`] = next[n]; });
  renderRocket(); cfgChanged();
  notice(`Trim saved to Rocket setup (A ${next[0]} µs, B ${next[1]} µs). Save, build and install over USB-C to fly it.`);
});

/* ---- board settings over the radio (IDLE only): sent, stored on the SD card, echoed back, compared ---- */
const BS_FIELDS = [["KP", "P gain", 0.05], ["KI", "I gain", 0.01], ["KD", "D gain", 0.01], ["INTEGRATOR_LIMIT", "Integrator limit", 0.01],
  ["OUTPUT_RATE_LIMIT", "Slew · travel/s", 0.5], ["DERIVATIVE_CUTOFF", "D filter · Hz", 1], ["MAX_TILT_DEG", "Give-up tilt · °", 1],
  ["SERVO_A_CENTER_US", "TVC X centre · µs", 1], ["SERVO_B_CENTER_US", "TVC Y centre · µs", 1], ["CHUTE_BACKUP_MS", "Servo backup · ms", 100],
  ["PYRO_DEPLOY_ALT_M", "Charge at height · m (0 = apogee)", 0.5], ["PYRO_CHUTE_DELAY_MS", "Charge delay · ms", 50],
  ["PYRO_FIRE_MS", "Charge on-time · ms", 50], ["PYRO_BACKUP_MS", "Charge backup · ms", 100]];
const BS = {board: null};
(() => {
  $("bs-fields").replaceChildren(...BS_FIELDS.map(([key, label, step]) => {
    const input = el("input", {type: "number", step: String(step), id: `bs-${key}`, "aria-label": label});
    input.oninput = bsMarkChanged;
    return el("label", {class: "field"}, el("span", {text: label}), input);
  }));
})();
function bsFill(values) { BS_FIELDS.forEach(([key]) => { if (values && values[key] !== undefined) $(`bs-${key}`).value = String(values[key]); }); bsMarkChanged(); }
function bsValues() {
  const out = {};
  for (const [key, label] of BS_FIELDS) {
    const v = parseFloat($(`bs-${key}`).value);
    if (!Number.isFinite(v)) throw new Error(`${label}: enter a number.`);
    out[key] = v;
  }
  return out;
}
function bsMarkChanged() {
  BS_FIELDS.forEach(([key]) => {
    const v = parseFloat($(`bs-${key}`).value), b = BS.board && BS.board[key];
    $(`bs-${key}`).parentElement.classList.toggle("changed", BS.board != null && Number.isFinite(v) && Math.abs(v - b) > 1e-6);
  });
}
async function bsReadBoard() {
  const r = await api("monitor/config", {});
  LT.boardCfg = r.board; BS.board = r.board;
  return r.board;
}
action("bs-load-board", async () => { bsFill(await bsReadBoard()); notice("Loaded the values the board is using now."); });
action("bs-load-setup", () => { if (!cfg) throw new Error("Rocket setup isn't loaded."); bsFill(cfg); });
async function bsSend(values, title) {
  const before = BS.board || await bsReadBoard();
  const changes = BS_FIELDS.filter(([key]) => Math.abs(values[key] - before[key]) > 1e-6)
    .map(([key, label]) => el("li", {text: `${label}: ${before[key]} → ${values[key]}`}));
  if (!changes.length) throw new Error("Nothing to change: these are the values the board already uses.");
  if (!await confirmModal(title, [el("ul", {}, ...changes),
      el("p", {text: "The board stores them on its SD card and uses them from now on, until you install firmware again. It only accepts them on the pad (IDLE, not armed)."})], "Send")) return null;
  const r = await api("monitor/settings/send", {values: {...before, ...values}});
  const board = await bsReadBoard();
  const mismatch = BS_FIELDS.filter(([key]) => Math.abs(board[key] - r.board[key]) > 1e-6);
  if (mismatch.length) throw new Error("Read-back mismatch: " + mismatch.map(([, l]) => l).join(", ") + ". Send again.");
  bsFill(board);
  if ($("bs-also-setup").checked && cfg) {
    BS_FIELDS.forEach(([key]) => { cfg[key] = board[key]; });
    renderRocket(); cfgChanged();
  }
  $("bs-result").replaceChildren(el("div", {class: "assessment good", text:
    `Verified: the board stored ${changes.length} change${changes.length > 1 ? "s" : ""}, read them back from its card and echoed every value.` +
    ($("bs-also-setup").checked ? " Rocket setup has them too: press Save there so the next install keeps them." : "")}));
  return board;
}
action("bs-send", () => bsSend(bsValues(), "Send these settings to the board?"));
action("bs-reset", async () => {
  if (!await confirmModal("Use the installed values again?", [el("p", {text: "The settings sent over the radio are cleared from the board's card. It goes back to the values compiled into the firmware you installed."})], "Reset")) return;
  await api("monitor/settings/reset", {});
  bsFill(await bsReadBoard());
  $("bs-result").replaceChildren(el("div", {class: "assessment good", text: "The board is back on the installed values."}));
});
action("trim-board", async () => {
  if (GIM.trimWant || GIM.trimSending) throw new Error("Still sending the trim to the board: try again in a moment.");
  if (!GIM.trim[0] && !GIM.trim[1]) throw new Error("No trim to save: nudge a servo first.");
  const board = await bsReadBoard(), trim = [...GIM.trim];
  const values = {...board, SERVO_A_CENTER_US: board.SERVO_A_CENTER_US + trim[0], SERVO_B_CENTER_US: board.SERVO_B_CENTER_US + trim[1]};
  const was = $("bs-also-setup").checked;
  $("bs-also-setup").checked = true;                 /* the new centre goes into Rocket setup too */
  try {
    if (!await bsSend(values, "Make the trimmed position the new centre?")) return;
    trimTo([0, 0]);                                  /* the centre now includes it */
    notice(`New centres stored on the board: X ${values.SERVO_A_CENTER_US} µs, Y ${values.SERVO_B_CENTER_US} µs. Save Rocket setup so the next install keeps them.`);
  } finally { $("bs-also-setup").checked = was; }
});

/* ---- pyro channel ground test: a test load only, IDLE + not armed, one code one pulse ---- */
/* what the connected board says about its own lock (CFG), else the source file */
function boardPyroLive() { return LT.boardCfg && LT.boardCfg.PYRO_OUTPUTS_COMPILED_IN !== undefined ? !!LT.boardCfg.PYRO_OUTPUTS_COMPILED_IN : pyroCompiledIn; }
action("pt-start", async () => {
  const channel = Number($("pt-channel").value), ms = Number($("pt-ms").value);
  const load = el("input", {type: "checkbox"}), away = el("input", {type: "checkbox"});
  if (!await confirmModal(`Test P${channel}?`, [
      el("p", {text: `The board will switch P${channel} on for ${ms} ms. Only do this with a test load on the terminal.`}),
      el("label", {class: "check"}, load, " Only a test load (LED + resistor, or a bulb) is on the terminal: no igniter, no e-match, no charge."),
      el("label", {class: "check"}, away, " The rocket has no motor in it and nobody is in front of it."),
      el("p", {class: boardPyroLive() ? "warn-text" : "help", text: boardPyroLive() ? "Pyro outputs are LIVE in the firmware on this board: the pin really switches on." : "Pyro outputs are locked in the firmware on this board: the test runs, but the pin stays off."})], "Arm test")) return;
  if (!load.checked || !away.checked) throw new Error("Tick both boxes to run the test.");
  const armed = await api("monitor/pyro/arm", {channel});
  const cont = armed.continuity.map((c, i) => `P${i + 1} ${c ? "●" : "○"}`).join("  ");
  if (!await confirmModal(`Fire one ${ms} ms pulse on P${channel}?`, [
      el("p", {text: `Armed for P${channel} with code ${armed.code} (good for 10 s, one pulse).`}),
      el("p", {class: "help", text: `Continuity now: ${cont} (raw levels). ${armed.locked ? "Outputs locked: the pin will stay off." : "Outputs LIVE."}`})], "Fire")) return;
  let result;
  try { result = await api("monitor/pyro/fire", {channel, code: armed.code, pulse_ms: ms}); }
  catch (error) {
    if (/No reply/.test(error.message)) throw new Error(`No reply from the board: the pulse may or may not have run (the board never repeats it). Look for PYRO_TEST_P${channel} in Events.`);
    throw error;
  }
  $("pt-result").replaceChildren(el("div", {class: `assessment ${result.locked ? "" : "good"}`, text: result.locked
    ? `Test ran on P${channel} (${result.pulse_ms} ms), but the outputs are locked in the firmware, so the pin stayed off: the LED should NOT have lit. That's the expected result for the locked image.`
    : `P${channel} was switched on for ${result.pulse_ms} ms: the test LED should have lit once. If it didn't, check the wiring before going further.`}));
});
action("pad-beep", async () => { await api("monitor/beep", {}); notice("The board is cycling red, green and blue and beeping for 2 seconds."); });

/* ---- canvases: at most every LT_DRAW_MS, only while the page shows ---- */
function ltRequestDraw() {
  if (LT.raf || $("live").hidden) return;
  LT.raf = requestAnimationFrame(now => {
    LT.raf = null;
    const wait = LT_DRAW_MS - (now - LT.lastDraw);
    if (wait > 0) { setTimeout(ltRequestDraw, wait); return; }
    LT.lastDraw = now;
    ltDrawPlots(); ltDrawAttitude();
  });
}
function ltServoFraction(n, pulse) {
  /* inverse of EasyTVC_ServoPulse, with the saved servo calibration */
  if (!cfg) return 0;
  const k = n ? "B" : "A", c = cfg[`SERVO_${k}_CENTER_US`];
  const span = pulse >= c ? cfg[`SERVO_${k}_MAX_US`] - c : c - cfg[`SERVO_${k}_MIN_US`];
  return cfg[`SERVO_${k}_DIRECTION`] * (pulse - c) / Math.max(1, span);
}
const LT_PLOTS = ["plot-alt", "plot-vs", "plot-acc", "plot-gyro", "plot-att", "plot-baro", "plot-tvca", "plot-tvcb"];
function ltDrawPlots() {
  const H = LT.hist;
  $("plot-chip").textContent = H.length ? `${H.length} frames · ${((H[H.length - 1].ms - H[0].ms) / 1000).toFixed(0)} s` : "no data";
  if (!H.length) {
    LT_PLOTS.forEach(id => { const c = $(id); c.getContext("2d").clearRect(0, 0, c.width, c.height); });
    return;
  }
  const end = H[H.length - 1].ms, t = H.map(h => (h.ms - end) / 1000);
  const events = (liveStatus.events || []).filter(e => e.ms <= end && e.ms >= end - LT_WINDOW_S * 1000)
    .map(e => ({t: (e.ms - end) / 1000, event: e.text}));
  lineChart($("plot-alt"), t, [H.map(h => h.alt_m)], events);
  lineChart($("plot-vs"), t, [H.map(h => h.vspeed)], []);          /* events on the altitude plot only: room */
  lineChart($("plot-acc"), t, [0, 1, 2].map(i => H.map(h => h.accel_g[i])), []);
  lineChart($("plot-att"), t, [H.map(h => h.tilt_x), H.map(h => h.tilt_y)], []);
  lineChart($("plot-gyro"), t, [0, 1, 2].map(i => H.map(h => h.gyro_dps[i])), []);
  const baro = H.filter(h => h.pressure_pa);
  if (baro.length) lineChart($("plot-baro"), baro.map(h => (h.ms - end) / 1000), [baro.map(h => h.pressure_pa / 100)], []);
  [[0, "plot-tvca"], [1, "plot-tvcb"]].forEach(([n, id]) =>
    lineChart($(id), t, [H.map(h => h.cmd[n]), H.map(h => ltServoFraction(n, h.pulse[n]))], []));
}

/* the rocket at its estimated attitude, drawn with the 3D sim's mesh and renderer maths */
function ltGimbal(pulse, n) {
  const m = rocketModel, k = n ? "B" : "A";
  if (!m || !cfg) return 0;
  const horn = (pulse - cfg[`SERVO_${k}_CENTER_US`]) * m.servo_deg_per_us * Math.PI / 180;
  const g = Math.asin(Math.max(-1, Math.min(1, m.linkage_ratio * Math.sin(horn)))) * 180 / Math.PI;
  return Math.max(-m.stop_deg, Math.min(m.stop_deg, g)) * Math.PI / 180;
}
function ltDrawAttitude() {
  const canvas = $("att-canvas"), rect = canvas.getBoundingClientRect();
  if (!rect.width) return;
  const k = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(rect.width * k) || canvas.height !== Math.round(rect.height * k)) {
    canvas.width = Math.round(rect.width * k); canvas.height = Math.round(rect.height * k);
  }
  const ctx = canvas.getContext("2d"); ctx.setTransform(k, 0, 0, k, 0, 0);
  const W = rect.width, Hh = rect.height;
  const sky = ctx.createLinearGradient(0, 0, 0, Hh); sky.addColorStop(0, pal().sky0); sky.addColorStop(1, pal().sky1);
  ctx.fillStyle = sky; ctx.fillRect(0, 0, W, Hh);
  const s = liveStatus, t = s && s.latest.tlm, stale = !t || s.age.tlm > LT_STALE_S;
  $("att-chip").textContent = !t ? "no data" : stale ? "STALE" : `tilt ${Math.hypot(t.tilt_x, t.tilt_y).toFixed(1)}°`;
  $("att-chip").className = `chip ${!t ? "" : stale ? "bad" : "good"}`;
  const model = rocketModel;
  ctx.font = pal().font(11); ctx.fillStyle = pal().muted;
  if (!model || !vehicle) { ctx.fillText("Set up the rocket in Design → Rocket setup to see it here.", 14, 22); return; }
  let q = t ? t.q : [1, 0, 0, 0];
  const n = Math.hypot(...q) || 1; q = q.map(x => x / n);
  const L = model.length_mm / 1000, dist = L * 2.4;
  const eye = [Math.cos(LT.pitch) * Math.cos(LT.yaw) * dist, Math.cos(LT.pitch) * Math.sin(LT.yaw) * dist, Math.sin(LT.pitch) * dist];
  const f = unit3(sub3([0, 0, 0], eye)), r = unit3(cross3(f, [0, 0, 1])), u = cross3(r, f);
  const F = Math.min(W, Hh) / (2 * Math.tan(0.45));
  const project = p => { const d = sub3(p, eye), z = dot3(d, f); if (z < 0.01) return null;
    return [W / 2 + dot3(d, r) / z * F, Hh / 2 - dot3(d, u) / z * F, z]; };
  const line = (a, b, color, width = 1) => { const A = project(a), B = project(b); if (!A || !B) return;
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath(); ctx.moveTo(A[0], A[1]); ctx.lineTo(B[0], B[1]); ctx.stroke(); };
  /* ground grid below the rocket and the vertical it should hold */
  const g0 = -L * 0.6, span = L * 0.9, step = span / 3;
  for (let g = -span; g <= span + 1e-9; g += step) {
    line([g, -span, g0], [g, span, g0], pal().line); line([-span, g, g0], [span, g, g0], pal().line);
  }
  ctx.setLineDash([4, 4]); line([0, 0, g0], [0, 0, L * 0.75], pal().line2, 1); ctx.setLineDash([]);
  /* rocket: same mesh as the 3D sim, rotated by the board's attitude estimate */
  const ga = t ? ltGimbal(t.pulse[0], 0) : 0, gb = t ? ltGimbal(t.pulse[1], 1) : 0;
  const light = unit3([0.4, -0.6, 0.7]), faces = [];
  for (const part of rocketMesh(model, ga, gb)) {
    const w = part.pts.map(p => qrot(q, p)), P = w.map(project);
    if (P.some(p => !p)) continue;
    const nrm = unit3(cross3(sub3(w[1], w[0]), sub3(w[2], w[0])));
    faces.push({P, depth: P.reduce((a, p) => a + p[2], 0) / P.length, color: part.color, shade: 0.55 + 0.45 * Math.abs(dot3(nrm, light))});
  }
  faces.sort((a, b) => b.depth - a.depth);
  ctx.globalAlpha = stale ? 0.45 : 1;
  for (const fc of faces) {
    const [r0, g1, b0] = fc.color;
    ctx.fillStyle = `rgb(${r0 * fc.shade | 0},${g1 * fc.shade | 0},${b0 * fc.shade | 0})`;
    ctx.beginPath(); ctx.moveTo(fc.P[0][0], fc.P[0][1]); fc.P.slice(1).forEach(p => ctx.lineTo(p[0], p[1])); ctx.closePath(); ctx.fill();
  }
  ctx.globalAlpha = 1;
  ctx.fillStyle = pal().text; ctx.font = pal().font(12);
  if (t) {
    ctx.fillText(`${t.state}${t.flags.tvc ? " · TVC active" : ""}`, 14, 22);
    ctx.fillStyle = pal().muted; ctx.font = pal().font(11);
    ctx.fillText(`tilt X ${t.tilt_x.toFixed(1)}°  Y ${t.tilt_y.toFixed(1)}° · gimbal A ${(ga * 180 / Math.PI).toFixed(1)}°  B ${(gb * 180 / Math.PI).toFixed(1)}°`, 14, 40);
  }
  if (stale && t) { ctx.fillStyle = pal().badLine; ctx.font = pal().font(14, "600"); ctx.fillText("STALE: no telemetry for over 1 s", 14, Hh - 14); }
}
(() => {
  const canvas = $("att-canvas");
  canvas.addEventListener("pointerdown", e => { LT.drag = [e.clientX, e.clientY]; canvas.setPointerCapture(e.pointerId); });
  canvas.addEventListener("pointermove", e => {
    if (!LT.drag) return;
    LT.yaw -= (e.clientX - LT.drag[0]) * 0.01;
    LT.pitch = Math.max(-0.2, Math.min(1.4, LT.pitch + (e.clientY - LT.drag[1]) * 0.01));
    LT.drag = [e.clientX, e.clientY]; ltRequestDraw();
  });
  canvas.addEventListener("pointerup", () => { LT.drag = null; });
  window.addEventListener("resize", ltRequestDraw);
})();

/* restore the last link choice once Studio's state has loaded */
(async () => {
  await window.studioReady;
  ltShowKind(["ble", "serial", "udp", "tcp"].includes(ltSettings().kind) ? ltSettings().kind : "udp");
  ltShowView(ltSettings().view === "pad" ? "pad" : "dash");
  ltRenderConnect(liveStatus || {connected: false}); ltRenderClock(null);
  try { liveStatus = await api("monitor/status", {}); ltIngest(liveStatus); ltRender(); if (liveStatus.connected) ltPoll(); }
  catch (error) { /* the server reports problems when the page is used */ }
})();

"use strict";
/* Tuning lab: gains tested on the real rocket. Studio's server flies the rocket from Rocket setup
 * in the 3D sim with the flight software in the loop (studio/launch.py: tune_evaluate, TuneJob):
 * a nominal flight, random flights, stress cases and the launch-detection margin.
 * Uses el, api, action, notice, confirmModal, pal, cfg, savedCfg, vehicle, renderRocket,
 * cfgChanged, showTab (app.js / launch.js) and sim (sim3d.js). */

const TL_FIELDS = [["KP", "P · stiffness", 0, 100, 0.5], ["KI", "I · steady error", 0, 100, 0.1], ["KD", "D · damping", 0, 100, 0.05],
  ["DERIVATIVE_CUTOFF", "D filter · Hz", 0, 100, 1], ["OUTPUT_RATE_LIMIT", "Slew · travel/s", 0.01, 100, 1]];
const TL = {gains: null, result: null, previous: null, tested: null, busy: false, timer: null};

const tlSame = (a, b) => !!a && !!b && TL_FIELDS.every(([k]) => Math.abs(a[k] - b[k]) < 1e-9);
const tlFromCfg = () => Object.fromEntries(TL_FIELDS.map(([k]) => [k, cfg[k]]));
const tlText = g => `P ${g.KP} · I ${g.KI} · D ${g.KD} · D filter ${g.DERIVATIVE_CUTOFF} Hz · slew ${g.OUTPUT_RATE_LIMIT}`;

function tlRenderGains() {
  if (!cfg) return;
  if (!TL.gains) TL.gains = tlFromCfg();
  $("tl-gains").replaceChildren(...TL_FIELDS.map(([key, label, min, max, step]) => {
    const input = el("input", {type: "number", min, max, step, "aria-label": label});
    input.value = String(TL.gains[key]);
    input.oninput = () => { TL.gains[key] = input.value === "" ? NaN : Number(input.value); tlRenderState(); };
    return el("label", {class: "field"}, el("span", {text: label}), input);
  }));
  tlRenderState();
}
function tlRenderState() {
  if (!cfg || !TL.gains) return;
  const same = tlSame(TL.gains, tlFromCfg());
  $("tl-source").textContent = same ? "same as Rocket setup" : "different from Rocket setup";
  $("tl-source").className = `chip ${same ? "good" : "warn"}`;
  $("tl-rocket").textContent = `Rocket setup: ${tlText(tlFromCfg())}`;
  $("tl-use").disabled = same; $("tl-revert").disabled = same;
  $("tl-stale").hidden = !TL.tested || tlSame(TL.gains, TL.tested);
}
function tlRequest() {
  if (!cfg || !vehicle) throw new Error("Rocket setup is not loaded yet.");
  for (const [key, label, min, max] of TL_FIELDS)
    if (!(TL.gains[key] >= min && TL.gains[key] <= max)) throw new Error(`${label}: enter a number from ${min} to ${max}.`);
  return {config: {...cfg, ...TL.gains}, vehicle, sim: sim || {}};
}

/* ---- results ---- */
function tlTile(label, value, tone = "") { return el("div", {class: tone}, el("span", {text: label}), el("strong", {text: value})); }
function tlShow(result) {
  if (TL.result && !tlSame(TL.result.gains, result.gains)) TL.previous = TL.result;
  TL.result = result; TL.tested = {...result.gains};
  const all = [result.normal, ...result.stress], flights = all.reduce((a, x) => a + x.flights, 0), ok = all.reduce((a, x) => a + x.ok, 0);
  const worst = Math.max(...all.map(x => x.worst)), n = result.nominal, m = result.launch_margin;
  const bad = result.notes.some(x => x.level === "bad"), warn = result.notes.some(x => x.level === "warn");
  $("tl-headline").textContent = bad ? "Not ready: see below" : warn ? "Holds, with a warning" : "Holds in every case";
  $("tl-tiles").replaceChildren(
    tlTile("FLIGHTS HELD", `${ok} / ${flights}`, ok < flights ? "bad" : ""),
    tlTile("TYPICAL TILT", `${result.normal.mean.toFixed(1)}°`),
    tlTile("WORST TILT", `${worst.toFixed(1)}°`, worst > 0.6 * cfg.MAX_TILT_DEG ? "bad" : ""),
    tlTile("TVC STARTS", n.tvc_on_t >= 0 ? `${n.tvc_on_t.toFixed(2)} s` : "never", n.tvc_on_t < 0 ? "bad" : ""),
    tlTile("LAUNCH MARGIN", m.detected ? `${m.percent.toFixed(0)} %` : "none", !m.detected || m.percent < 10 ? "bad" : ""),
    tlTile("SCORE (LOWER IS BETTER)", result.score.toFixed(2)));
  $("tl-notes").replaceChildren(...result.notes.map(x => el("div", {class: `check-line ${x.level}`, text: x.text})));
  const body = $("tl-stress").querySelector("tbody");
  body.replaceChildren(...[{label: "Random flights (as modelled)", ...result.normal}, ...result.stress].map(x => {
    const why = Object.entries(x.reasons).map(([k, c]) => `${c} × ${k}`).join("; ");
    return el("tr", {class: x.ok < x.flights ? "differ" : ""}, el("td", {text: x.label + (why ? ` — ${why}` : "")}),
      el("td", {text: `${x.ok} / ${x.flights}`}), el("td", {text: `${x.mean.toFixed(1)}°`}), el("td", {text: `${x.worst.toFixed(1)}°`}));
  }));
  $("tl-stress").hidden = false; $("tl-hint").hidden = true;
  tlRenderState(); tlDraw();
}
function tlDraw() {
  const canvas = $("tl-canvas"), rect = canvas.getBoundingClientRect(), r = TL.result;
  if (!rect.width || !r) return;
  const k = window.devicePixelRatio || 1; canvas.width = rect.width * k; canvas.height = rect.height * k;
  const ctx = canvas.getContext("2d"); ctx.scale(k, k);
  const left = 40, top = 10, width = rect.width - 52, height = rect.height - 36;
  const traces = [r.trace, TL.previous ? TL.previous.trace : null];
  /* to burnout only: after it the rocket is unsteered by design, and that tumble would set the scale */
  const t0 = -0.2, t1 = r.nominal.burn_s;
  const during = tr => tr.filter(p => p[0] <= t1);
  const peak = Math.max(2, ...traces.filter(Boolean).flatMap(tr => during(tr).map(p => Math.max(p[1], Math.abs(p[2]), Math.abs(p[3])))));
  const max = Math.ceil(peak / 2) * 2;
  const X = t => left + (t - t0) / (t1 - t0) * width, Y = v => top + (1 - v / max) * height;
  /* before TVC is on: unsteered */
  const on = r.nominal.tvc_on_t >= 0 ? r.nominal.tvc_on_t : t1;
  ctx.fillStyle = pal().fill; ctx.fillRect(X(0), top, X(on) - X(0), height);
  ctx.font = pal().font(10); ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const v = max * i / 4, y = Y(v);
    ctx.strokeStyle = pal().grid; ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(left + width, y); ctx.stroke();
    ctx.fillStyle = pal().muted; ctx.fillText(`${v.toFixed(0)}°`, 6, y + 3);
  }
  for (let t = 0; t <= t1; t += 0.5) { ctx.fillStyle = pal().muted; ctx.fillText(`${t.toFixed(1)}s`, X(t) - 9, top + height + 16); }
  ctx.fillStyle = pal().muted; ctx.fillText("TVC off", X(0) + 3, top + 11);
  if (r.nominal.stop_deg <= max) {
    ctx.strokeStyle = pal().badLine; ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.moveTo(left, Y(r.nominal.stop_deg)); ctx.lineTo(left + width, Y(r.nominal.stop_deg)); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = pal().muted; ctx.fillText("gimbal stop", left + width - 62, Y(r.nominal.stop_deg) - 3);
  }
  const line = (tr, get, color, w) => { ctx.strokeStyle = color; ctx.lineWidth = w; ctx.beginPath();
    during(tr).forEach((p, i) => { const x = X(p[0]), y = Y(Math.min(max, get(p))); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); }); ctx.stroke(); };
  if (TL.previous) line(TL.previous.trace, p => p[1], pal().line2, 1.5);
  line(r.trace, p => Math.max(Math.abs(p[2]), Math.abs(p[3])), pal().green, 1.2);
  line(r.trace, p => p[1], pal().purple, 2);
}
new ResizeObserver(tlDraw).observe($("tl-canvas"));

/* ---- actions ---- */
function tlBusy(on, label) {
  TL.busy = on;
  for (const id of ["tl-test", "tl-auto"]) $(id).disabled = on;
  $("tl-test").innerHTML = on && label ? label : "Test these gains <span>↗</span>";
}
action("tl-test", async () => {
  if (TL.busy) return;
  const request = tlRequest();
  tlBusy(true, "Flying…");
  try { tlShow(await api("tune/evaluate", request)); }
  finally { tlBusy(false); }
});
action("tl-revert", () => { TL.gains = tlFromCfg(); tlRenderGains(); });
action("tl-use", async () => {
  if (TL.tested && !tlSame(TL.gains, TL.tested) || !TL.tested) {
    if (!await confirmModal("These gains haven't been tested", [el("p", {text: "Test them first to see how they hold up, or continue to copy them to Rocket setup as they are."})], "Copy anyway")) return;
  } else if (TL.result && TL.result.notes.some(x => x.level === "bad") && !TL.result.held_all) {
    if (!await confirmModal("The test lost flights with these gains", [el("p", {text: "Copy them to Rocket setup anyway?"})], "Copy anyway")) return;
  }
  Object.assign(cfg, TL.gains);
  renderRocket(); cfgChanged(); tlRenderState();
  notice("Gains copied to Rocket setup. Save there, then install on the board (or send them from Live → Pad tools → Board settings).");
});

async function tlPoll() {
  clearTimeout(TL.timer);
  let s;
  try { s = await api("tune/status", {}); } catch (error) { notice(error.message); tlBusy(false); $("tl-progress").hidden = true; return; }
  $("tl-bar").style.width = `${Math.round((s.progress || 0) * 100)}%`;
  $("tl-stage").textContent = s.running ? `${s.stage}… ${Math.round((s.progress || 0) * 100)} %${s.best ? ` · best so far ${tlText(s.best)}` : ""}` : s.stage;
  if (s.running) { TL.timer = setTimeout(tlPoll, 400); return; }
  tlBusy(false); $("tl-progress").hidden = true;
  if (s.error) return notice(`Auto-tune stopped: ${s.error}`);
  if (s.cancelled || !s.result) return notice("Auto-tune cancelled. Nothing changed.");
  const {before, best, improved} = s.result;
  TL.previous = null; TL.result = null;
  if (improved) { tlShow(before); TL.gains = {...best.gains}; tlRenderGains(); tlShow(best);
    notice(`Auto-tune found better gains (score ${before.score.toFixed(2)} → ${best.score.toFixed(2)}). They are in the fields: review, then Use in Rocket setup.`); }
  else { tlShow(before); notice("Auto-tune tried about 70 gain sets and none beat your current gains by a clear margin. They stay as they are."); }
}
action("tl-auto", async () => {
  if (TL.busy) return;
  const request = tlRequest();
  if (!await confirmModal("Auto-tune?", [el("p", {text: "Studio searches about 70 gain sets on your rocket, then tests the best five against every stress case. It takes about half a minute and starts from the gains in the fields."}),
      el("p", {class: "help", text: "New gains are only suggested if they beat the starting ones clearly, with every flight held. Nothing is changed in Rocket setup until you press Use in Rocket setup."})], "Start")) return;
  tlBusy(true, "Tuning…"); $("tl-progress").hidden = false; $("tl-bar").style.width = "0%"; $("tl-stage").textContent = "Starting…";
  try { await api("tune/start", request); } catch (error) { tlBusy(false); $("tl-progress").hidden = true; throw error; }
  tlPoll();
});
action("tl-cancel", () => api("tune/cancel", {}));

/* fill the fields when the page is first shown, and follow Rocket setup until the user edits */
const tlPreviousTabHook = window.onTabShown;
window.onTabShown = id => {
  if (tlPreviousTabHook) tlPreviousTabHook(id);
  if (id !== "tune" || !cfg) return;
  if (!TL.gains || (!TL.result && !TL.busy && tlSame(TL.gains, TL.lastCfg || TL.gains))) TL.gains = tlFromCfg();
  TL.lastCfg = tlFromCfg();
  tlRenderGains(); tlDraw();
};

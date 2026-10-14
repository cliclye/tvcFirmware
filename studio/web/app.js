"use strict";
/* ---- appearance: light / dark / system ---- */
let paletteCache = null;
function isDarkTheme() {
  const t = document.documentElement.dataset.theme;
  return t === "dark" || (!t && window.matchMedia("(prefers-color-scheme: dark)").matches);
}
/* Colours for canvas drawing, from the CSS theme tokens (cached until the theme changes). */
function pal() {
  if (paletteCache) return paletteCache;
  const css = getComputedStyle(document.documentElement), v = n => css.getPropertyValue(`--${n}`).trim();
  const dark = isDarkTheme();
  paletteCache = {grid: v("line-soft"), muted: v("muted"), line: v("line"), line2: dark ? "#45435e" : "#cfcbe0",
    text: v("text"), text2: v("text-2"), hud: dark ? "rgba(29,28,39,0.9)" : "rgba(255,255,255,0.88)", fill: v("purple-pale"),
    card: v("card"), canvas: v("canvas-bg"), badLine: dark ? "#7a3a44" : "#e7a9a9", shadow: v("shadow"),
    sky0: dark ? "#1b1c30" : "#eef0fb", sky1: dark ? "#121118" : "#f9f8f4", purple: v("purple"), green: v("green"),
    series: [v("purple"), v("green"), v("amber")], dark};
  return paletteCache;
}
function applyTheme(choice) {
  if (choice === "light" || choice === "dark") document.documentElement.dataset.theme = choice;
  else delete document.documentElement.dataset.theme;
  try { localStorage.setItem("easytvc-theme", choice); } catch (error) { /* private mode */ }
  document.querySelectorAll("[data-theme-choice]").forEach(b => b.classList.toggle("selected", b.dataset.themeChoice === choice));
  paletteCache = null;
  window.dispatchEvent(new Event("easytvc-theme"));
}
(() => {
  let saved = "system";
  try { saved = localStorage.getItem("easytvc-theme") || "system"; } catch (error) { /* ignore */ }
  applyTheme(saved);
  document.querySelectorAll("[data-theme-choice]").forEach(b => b.onclick = () => applyTheme(b.dataset.themeChoice));
  window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => { paletteCache = null; window.dispatchEvent(new Event("easytvc-theme")); });
})();
const $ = id => document.getElementById(id);
const token = location.hash.slice(1) || sessionStorage.getItem("easytvc-session");
if (token) sessionStorage.setItem("easytvc-session", token);
history.replaceState(null, "", "/");
let profile, rows = [], busy = false, revision = 0, runRevision = -1;
const axisFields = ["kp", "ki", "kd", "integrator_limit", "output_limit", "output_rate_limit", "derivative_cutoff_hz"];

function notice(message) { $("notice").textContent = message; $("notice").hidden = false; }
async function api(path, data) {
  const response = await fetch(`/api/${path}`, {method: data === undefined ? "GET" : "POST",
    headers: {Authorization: `Bearer ${token || ""}`, "Content-Type": "application/json"},
    body: data === undefined ? undefined : JSON.stringify(data)});
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || "Request failed.");
  return result;
}
function action(id, fn) { $(id).addEventListener("click", async () => {
  try { await fn(); } catch (error) { notice(error.message); }
}); }
function download(name, type, content) {
  const url = URL.createObjectURL(new Blob([content], {type}));
  const a = document.createElement("a"); a.href = url; a.download = name; a.click();
  setTimeout(() => URL.revokeObjectURL(url), 2000);
}
function changed() {
  revision++;
  $("csv").disabled = true;
  if (rows.length) { $("simulation-status").textContent = "Inputs changed. Run again to update this result."; $("simulation-status").className = "assessment"; }
}
function numberInput(object, key, title, min, max, step = .01) {
  const label = document.createElement("label"); label.className = "field";
  const text = document.createElement("span"); text.textContent = title;
  const input = document.createElement("input"); input.type = "number"; input.min = min; input.max = max; input.step = step;
  input.value = Number(object[key].toFixed(6)); input.setAttribute("aria-label", title);
  input.addEventListener("input", () => { object[key] = input.value === "" ? NaN : Number(input.value); changed(); });
  label.append(text, input); return label;
}
function grid(className = "field-grid") { const node = document.createElement("div"); node.className = className; return node; }
function textNode(tag, text) { const node = document.createElement(tag); node.textContent = text; return node; }
function render() {
  $("profile-name").value = profile.name;
  $("maximum-tilt").value = profile.maximum_tilt_deg;
  $("axes").replaceChildren();
  profile.axes.forEach((axis, i) => {
    const block = grid("axis-block");
    const title = grid("axis-title"); title.append(textNode("span", `Axis ${i ? "Y" : "X"}`), textNode("small", `Gimbal ${i+1}`));
    const gains = grid("gain-grid");
    [["kp", "P · stiffness"], ["ki", "I · steady error"], ["kd", "D · damping"]].forEach(([key, label]) => gains.append(numberInput(axis, key, label, 0, 100)));
    const details = document.createElement("details"); details.append(textNode("summary", "Limits & derivative filter"));
    const extra = grid();
    [["integrator_limit", "Integrator limit", 0, 1, .01], ["output_limit", "Command limit", .01, 1, .01],
      ["output_rate_limit", "Slew / second", .01, 100, .1], ["derivative_cutoff_hz", "D filter · Hz", 0, 100, 1]].forEach(args => extra.append(numberInput(axis, ...args)));
    details.append(extra); block.append(title, gains, details); $("axes").append(block);
  });
  $("conditions").replaceChildren();
  [["wind_m_s", "Sustained wind", 0, 30, "m/s"], ["gust_m_s", "Peak gust", 0, 40, "m/s"], ["temperature_c", "Air temperature", -30, 60, "°C"]].forEach(([key, label, min, max, unit]) => {
    const block = grid("range-field"), row = grid("range-label");
    const numeric = document.createElement("input"); numeric.type = "number"; numeric.min = min; numeric.max = max; numeric.step = .1; numeric.value = profile.plant[key]; numeric.setAttribute("aria-label", label);
    const span = textNode("span", ""); span.append(numeric, document.createTextNode(` ${unit}`)); row.append(textNode("span", label), span);
    const slider = document.createElement("input"); slider.type = "range"; slider.min = min; slider.max = max; slider.step = .1; slider.value = profile.plant[key]; slider.setAttribute("aria-label", `${label} slider`);
    numeric.oninput = () => { profile.plant[key] = numeric.value === "" ? NaN : Number(numeric.value); slider.value = numeric.value; changed(); weatherPreview(); };
    slider.oninput = () => { profile.plant[key] = Number(slider.value); numeric.value = slider.value; changed(); weatherPreview(); };
    block.append(row, slider); $("conditions").append(block);
  });
  $("wet").checked = profile.weather.wet; $("validated").checked = profile.weather.limits_validated;
  const limits = grid();
  [["maximum_gust_m_s", "Max gust · m/s", 0, 40], ["minimum_temperature_c", "Min temp · °C", -30, 60], ["maximum_temperature_c", "Max temp · °C", -30, 60]].forEach(args => limits.append(numberInput(profile.weather, ...args, 1)));
  limits.addEventListener("input", weatherPreview); $("weather-limits").replaceChildren(limits);
  $("plant").replaceChildren();
  [["inertia_x", "X inertia · kg·m²", .0001, 10, .001], ["inertia_y", "Y inertia · kg·m²", .0001, 10, .001],
   ["thrust_n", "Thrust · N", .1, 1000, 1], ["thrust_arm_m", "Thrust arm · m", .01, 2, .01],
   ["gimbal_deg", "Full command · °", .1, 15, .1], ["servo_tau_s", "Servo lag · s", .005, 1, .005],
   ["area_m2", "Side area · m²", .0001, 1, .001], ["drag_coefficient", "Drag coefficient", .01, 3, .01],
   ["cp_arm_m", "Aero moment arm · m", .001, 2, .01], ["initial_tilt_deg", "Initial X tilt · °", -20, 20, 1]]
    .forEach(args => $("plant").append(numberInput(profile.plant, ...args)));
  $("servos").replaceChildren();
  profile.servos.forEach((servo, i) => {
    const block = grid("servo-block"), fields = grid(); block.append(textNode("h3", `Axis ${i ? "Y" : "X"} servo`));
    [["minimum_us", "Minimum · µs", 900, 2100], ["center_us", "Center · µs", 900, 2100], ["maximum_us", "Maximum · µs", 900, 2100], ["channel", "Channel", 1, 4]]
      .forEach(args => fields.append(numberInput(servo, ...args, 1)));
    const label = grid("field"); label.append(textNode("span", "Direction")); const select = document.createElement("select"); select.setAttribute("aria-label", `Axis ${i ? "Y" : "X"} direction`);
    [[1, "Normal"], [-1, "Reverse"]].forEach(([value, name]) => { const option = textNode("option", name); option.value = value; select.append(option); });
    select.value = servo.direction; select.onchange = () => { servo.direction = Number(select.value); changed(); }; label.append(select); fields.append(label);
    block.append(fields); $("servos").append(block);
  });
  weatherPreview();
}
function weatherPreview() {
  if (!profile) return;
  const w = profile.weather, p = profile.plant, reasons = [];
  if (!w.limits_validated) reasons.push("Vehicle operating limits are unvalidated.");
  if (w.wet) reasons.push("Wet-weather operation is not qualified.");
  if (p.gust_m_s > w.maximum_gust_m_s) reasons.push("Peak gust exceeds your entered wind limit.");
  if (p.temperature_c < w.minimum_temperature_c || p.temperature_c > w.maximum_temperature_c) reasons.push("Temperature is outside your entered limits.");
  if (p.gust_m_s < p.wind_m_s) reasons.push("Peak gust must be at least the sustained wind.");
  $("weather-result").textContent = reasons.join(" ") || "Within entered limits — not flight clearance.";
  $("weather-result").className = reasons.length ? "assessment" : "assessment good";
}
async function run() {
  if (busy) return;
  busy = true; $("run").disabled = true; $("run").textContent = "Simulating…";
  const requestedRevision = revision;
  try {
    const result = await api("simulate", profile);
    if (requestedRevision !== revision) { notice("Inputs changed during simulation. Run again for the current profile."); return; }
    rows = result.rows; runRevision = revision; $("chart-hint").hidden = true; $("csv").disabled = false;
    const m = result.metrics; $("peak").textContent = `${m.peak_tilt_deg.toFixed(1)}°`; $("final").textContent = `${m.final_tilt_deg.toFixed(1)}°`;
    $("saturation").textContent = `${m.saturation_percent.toFixed(1)}%`;
    $("simulation-status").textContent = m.fault ? "Controller faulted in the model: the tilt limit was exceeded. Commands returned to center." :
      m.saturation_percent > 5 ? "The controller reached its output limit. Check available torque and model assumptions." : "Simulation completed. This is a model result, not hardware or flight validation.";
    $("simulation-status").className = m.fault || m.saturation_percent > 5 ? "assessment" : "assessment good";
    draw();
  } finally { busy = false; $("run").disabled = false; $("run").textContent = "Run simulation ↗"; }
}
function draw(hover = null) {
  const canvas = $("chart"), rect = canvas.getBoundingClientRect();
  if (!rect.width) return;
  const scale = window.devicePixelRatio || 1; canvas.width = rect.width*scale; canvas.height = rect.height*scale;
  const ctx = canvas.getContext("2d"); ctx.scale(scale, scale);
  const left = 38, top = 13, width = rect.width-50, height = rect.height-38;
  const max = rows.length ? Math.max(5, Math.ceil(Math.max(...rows.flatMap(r => [Math.abs(r[1]), Math.abs(r[2])])) / 5)*5) : 10;
  ctx.font = "10px -apple-system, sans-serif"; ctx.lineWidth = 1;
  for (let i=0; i<=4; i++) { const y = top+i*height/4; ctx.strokeStyle = pal().grid; ctx.beginPath(); ctx.moveTo(left,y); ctx.lineTo(left+width,y); ctx.stroke(); ctx.fillStyle = pal().muted; ctx.fillText((max-i*max/2).toFixed(0), 4,y+4); }
  for (let i=0; i<=5; i++) { ctx.fillStyle = pal().muted; ctx.fillText(`${i*2}s`,left+i*width/5-5,top+height+20); }
  [1,2].forEach((column,i) => { ctx.strokeStyle = i ? pal().green : pal().purple; ctx.lineWidth = 2; ctx.beginPath(); rows.forEach((r,j) => { const x = left+r[0]/10*width, y = top+height/2-r[column]/max*height/2; if (j) ctx.lineTo(x,y); else ctx.moveTo(x,y); }); ctx.stroke(); });
  if (hover !== null && rows.length) {
    const time = Math.min(10,Math.max(0,(hover-left)/width*10)), index = Math.min(rows.length-1,Math.round(time*100)), r = rows[index];
    ctx.strokeStyle = pal().muted; ctx.setLineDash([3,3]); ctx.beginPath(); ctx.moveTo(left+r[0]/10*width,top); ctx.lineTo(left+r[0]/10*width,top+height); ctx.stroke();
    $("chart-caption").textContent = `${r[0].toFixed(2)} s · X ${r[1].toFixed(2)}° · Y ${r[2].toFixed(2)}° · command ${r[3].toFixed(2)}, ${r[4].toFixed(2)}`;
  }
}

function showTab(id) {
  document.querySelectorAll(".tab").forEach(tab => tab.hidden = tab.id !== id);
  document.querySelectorAll("[data-tab]").forEach(item => item.classList.toggle("selected", item.dataset.tab === id));
  draw();
  if (window.onTabShown) window.onTabShown(id);
}
document.querySelectorAll("[data-tab]").forEach(button => button.onclick = () => showTab(button.dataset.tab));
document.querySelectorAll("[data-goto]").forEach(button => button.onclick = () => showTab(button.dataset.goto));
$("chart").addEventListener("mousemove",event => draw(event.offsetX));
$("chart").addEventListener("mouseleave",() => { $("chart-caption").textContent = "Tilt · degrees / time · seconds"; draw(); });
new ResizeObserver(() => draw()).observe($("chart"));
$("profile-name").oninput = () => { profile.name = $("profile-name").value; changed(); };
$("maximum-tilt").oninput = () => { profile.maximum_tilt_deg = Number($("maximum-tilt").value); changed(); };
$("wet").onchange = () => { profile.weather.wet = $("wet").checked; changed(); weatherPreview(); };
$("validated").onchange = () => { profile.weather.limits_validated = $("validated").checked; changed(); weatherPreview(); };
action("run",run);
action("reset",async () => { profile = (await api("defaults")).profile; changed(); render(); });
action("export",async () => { await api("validate",profile); download("easytvc-profile.json","application/json",JSON.stringify(profile,null,2)); notice("Profile saved to your Downloads folder. This does not upload it to a board."); });
action("import",() => $("file").click());
$("file").onchange = async () => {
  try { const file = $("file").files[0]; if (!file) return; if (file.size > 32768) throw new Error("Profile is too large.");
    const candidate = JSON.parse(await file.text()); const result = await api("validate",candidate); profile = result.profile; changed(); render(); notice("Profile imported and validated. Run the model to inspect its response.");
  } catch(error) { notice(error.message); } finally { $("file").value = ""; }
};
action("csv",() => { if (runRevision !== revision) throw new Error("Run the model again before exporting."); download("easytvc-simulation.csv","text/csv","seconds,tilt_x_deg,tilt_y_deg,command_x,command_y,pulse_x_us,pulse_y_us,wind_m_s,fault\n"+rows.map(r=>r.join(",")).join("\n")); });
action("quit",async () => { const result = await api("quit",{}); notice(result.message); document.querySelectorAll("button").forEach(b=>b.disabled=true); });
window.addEventListener("beforeunload",event => { if(revision > 0) { event.preventDefault(); event.returnValue=""; } });
window.studioReady = (async () => { try { const initial = await api("defaults"); profile = initial.profile; render(); } catch(error) { notice(error.message + " Open Studio using its launcher to create a valid local session."); } })();

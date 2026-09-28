// Optional browser integration test; Node 22+ and an installed Chrome. No npm packages.
import {spawn} from 'node:child_process';
import {mkdtemp, writeFile} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import {join} from 'node:path';
import assert from 'node:assert/strict';

const url = process.argv[2];
if (!url?.startsWith('http://127.0.0.1:')) throw new Error('Pass the local Studio session URL.');
const directory = await mkdtemp(join(tmpdir(),'easytvc-browser-'));
const chrome = spawn('/Applications/Google Chrome.app/Contents/MacOS/Google Chrome', [
  '--headless=new', '--remote-debugging-port=0', '--no-first-run', '--no-default-browser-check',
  '--disable-background-networking', `--user-data-dir=${directory}`, 'about:blank'
], {stdio:['ignore','ignore','pipe']});
let ws;
const errors = [];
try {
  const debuggerURL = await new Promise((resolve,reject) => {
    let output = '';
    const timer = setTimeout(()=>reject(new Error('Chrome did not start.')),15000);
    chrome.on('error',reject);
    chrome.stderr.on('data',chunk => {
      output += chunk.toString(); const match = output.match(/DevTools listening on (ws:\/\/\S+)/);
      if(match) { clearTimeout(timer); resolve(match[1]); }
    });
  });
  ws = new WebSocket(debuggerURL);
  await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
  let next=0; const pending = new Map();
  ws.onmessage = event => {
    const message = JSON.parse(event.data);
    if(message.method === 'Runtime.exceptionThrown') errors.push(message.params.exceptionDetails);
    if(message.id && pending.has(message.id)) {
      const entry=pending.get(message.id); pending.delete(message.id); clearTimeout(entry.timer);
      if(message.error) entry.reject(new Error(JSON.stringify(message.error))); else entry.resolve(message.result);
    }
  };
  function call(method,params={},sessionId) {
    return new Promise((resolve,reject)=>{
      const id=++next, timer=setTimeout(()=>{pending.delete(id);reject(new Error(`${method} timed out`));},15000);
      pending.set(id,{resolve,reject,timer}); ws.send(JSON.stringify({id,method,params,...(sessionId?{sessionId}:{})}));
    });
  }
  const {targetId} = await call('Target.createTarget',{url:'about:blank'});
  const {sessionId} = await call('Target.attachToTarget',{targetId,flatten:true});
  await call('Runtime.enable',{},sessionId);
  await call('Page.enable',{},sessionId);
  await call('Emulation.setDeviceMetricsOverride',{width:1440,height:1100,deviceScaleFactor:1,mobile:false},sessionId);
  await call('Page.navigate',{url},sessionId);
  const evaluate = async expression => {
    const result = await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true},sessionId);
    if(result.exceptionDetails) throw new Error(JSON.stringify(result.exceptionDetails));
    return result.result.value;
  };
  async function until(expression) {
    const deadline=Date.now()+10000;
    while(Date.now()<deadline) { if(await evaluate(expression)) return; await new Promise(r=>setTimeout(r,100)); }
    throw new Error(`UI did not become ready: ${expression}`);
  }
  await until('document.querySelector("#peak")?.textContent !== "—" && document.querySelector("#peak")?.textContent.includes("°")');
  assert.match(await evaluate('document.querySelector("#simulation-status").textContent'),/Simulation completed/);
  assert.equal(await evaluate('document.querySelectorAll("#axes input").length'),14);
  assert.equal(await evaluate('document.querySelector("#write").disabled'),true);
  const original = await evaluate('document.querySelector("#final").textContent');
  await evaluate(`(() => {const n=document.querySelector('#conditions input[aria-label="Peak gust"]');n.value=20;n.dispatchEvent(new Event('input',{bubbles:true}));document.querySelector('#run').click();})()`);
  await until('!document.querySelector("#run").disabled');
  assert.notEqual(await evaluate('document.querySelector("#final").textContent'),original);
  assert.match(await evaluate('document.querySelector("#weather-result").textContent'),/exceeds/);
  await evaluate('document.querySelector("#reset").click()');
  await until(`document.querySelector('#conditions input[aria-label="Peak gust"]').value === '4'`);
  await evaluate('document.querySelector("#run").click()');
  await until('!document.querySelector("#run").disabled');
  const screenshot = await call('Page.captureScreenshot',{format:'png'},sessionId);
  await writeFile(join(directory,'studio-desktop.png'),Buffer.from(screenshot.data,'base64'));
  await evaluate('document.querySelector("[data-tab=device]").click()');
  assert.equal(await evaluate('document.querySelector("#device").hidden'),false);
  await evaluate('document.querySelector("[data-tab=tune]").click()');
  await call('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true},sessionId);
  await new Promise(r=>setTimeout(r,200));
  assert.equal(await evaluate('document.documentElement.scrollWidth <= 390'),true);
  assert.deepEqual(errors,[]);
  console.log(`Browser smoke test passed: simulation, wind input, limits, reset, navigation, mobile layout. Screenshot: ${directory}/studio-desktop.png`);
} finally {
  ws?.close();
  chrome.kill('SIGTERM');
}

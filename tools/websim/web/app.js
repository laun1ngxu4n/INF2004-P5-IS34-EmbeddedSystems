// app.js: the page.  Draws the track and robot, edits tiles, runs the simulator.
import { Sim, ROBOT } from './sim.js';
import { World, TILE, TAPE, KINDS, DEFAULT_PARAMS } from './world.js';

const $ = id => document.getElementById(id);
const DEG = Math.PI / 180;

// ---- ready-made courses -----------------------------------------------------
const PRESETS = {
  'Full course: ramp, hump, obstacle, barcode, junction': { start: [0, 6, 'N'], text: 'C1 S1 S1 S1 C2 .\nS0 . . . S0 .\nH0 . . . O0 .\nS0 . . . S0 .\nS0 . . . S0 .\nR0 . . . B2:Q .\nS0 . S1 S1 X .' },
  'Barcode then left junction': { start: [2, 4, 'N'], text: '. . S0\nS1 S1 X\n. . S0\n. . B0:K\n. . S0' },
  'Obstacle bypass': { start: [1, 4, 'N'], text: '. S0 .\n. S0 .\n. O0 .\n. S0 .\n. S0 .' },
  'Ramp (20 mm)': { start: [1, 3, 'N'], text: '. S0 .\n. R0 .\n. S0 .\n. S0 .' },
  'Hump under right wheel (30 mm)': { start: [1, 3, 'N'], text: '. S0 .\n. H0 .\n. S0 .\n. S0 .' },
  'S-bend (curves, like your photo)': { start: [1, 2, 'N'], text: 'S0 .\nC0 C2\n. S0' },
  'Straight line': { start: [0, 3, 'N'], text: 'S0\nS0\nS0\nS0' },
};

// ---- state -----------------------------------------------------------------------
let world = new World(PRESETS['Full course: ramp, hump, obstacle, barcode, junction'].text);
let startPose = null, sim = null, fwBytes = null, running = true, placing = false;
let tool = 'S', rot = 0, hover = null, painting = false;
const toolParams = JSON.parse(JSON.stringify(DEFAULT_PARAMS));
let trail = [];
const scope = new Float32Array(8000); let scopeI = 0;

// ---- helpers --------------------------------------------------------------------------
function startFromTile(col, row, dir) {
  const th = { N: -Math.PI / 2, E: 0, S: Math.PI / 2, W: Math.PI }[dir];
  const ox = Math.cos(th) * -60, oy = Math.sin(th) * -60; // axle a bit behind the tile centre
  return { x: (col + 0.5) * TILE + ox, y: (row + 0.5) * TILE + oy, th };
}
function defaultStart() { // bottom-most tile with a straight N-S line
  for (let r = world.rows - 1; r >= 0; r--) for (let c = world.cols - 1; c >= 0; c--) {
    const t = world.grid[r][c];
    if (t.kind === 'S' && t.rot % 2 === 0) return startFromTile(c, r, 'N');
  }
  return startFromTile(0, world.rows - 1, 'N');
}

async function loadFirmware() {
  if (!fwBytes) {
    const r = await fetch('firmware.wasm'); if (!r.ok) throw new Error('firmware.wasm not found: run tools/websim/build.sh');
    fwBytes = await r.arrayBuffer();
  }
  $('fwinfo').textContent = `firmware.wasm ${(fwBytes.byteLength / 1024).toFixed(0)} KB`;
}

async function boot() {
  sim = new Sim(world);
  sim.onLog = (t, line) => addLog(t, line);
  $('log').textContent = '';
  await sim.load(fwBytes);
  const p = startPose ?? defaultStart();
  sim.place(p.x, p.y, p.th);
  trail = [];
  setStatus('booted: send the mapping, then START');
}

function setStatus(s) { $('status').textContent = s; }

function addLog(t, line) {
  if (line.startsWith('[tmg]') && $('hidetmg').checked) return;
  if (/^\d+(\.\d+)?$|^[-() |@]+$|^ ?[\d.]+ ?$/.test(line.trim())) return;
  const el = document.createElement('div');
  const cls = line.includes('[EVT]') || line.includes('[SCAN]') ? 'evt' : /ERROR|fault|FAULT|REJECTED/.test(line) ? 'err' : line.startsWith('[tmg]') ? 'dim' : '';
  el.className = cls;
  el.innerHTML = `<span class="t">${t.toFixed(2).padStart(7)} </span>` + line.replace(/&/g, '&amp;').replace(/</g, '&lt;');
  const log = $('log'), stick = log.scrollTop + log.clientHeight >= log.scrollHeight - 30;
  log.appendChild(el);
  while (log.childElementCount > 500) log.firstChild.remove();
  if (stick) log.scrollTop = log.scrollHeight;
  if (line.includes('[EVT]')) setStatus(line.replace(/^.*\[EVT\]\s*/, ''));
}

// ---- drawing ------------------------------------------------------------------------------
const cv = $('map'), ctx = cv.getContext('2d');
let S = 0.3; // px per mm

function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const maxW = Math.min(980, $('left').clientWidth || 800), maxH = Math.max(380, window.innerHeight * 0.72);
  S = Math.min(maxW / world.width, maxH / world.height, 0.9);
  cv.width = Math.round(world.width * S * dpr); cv.height = Math.round(world.height * S * dpr);
  cv.style.width = Math.round(world.width * S) + 'px'; cv.style.height = Math.round(world.height * S) + 'px';
}

function drawTile(t) {
  const h = TILE / 2;
  ctx.save();
  ctx.translate((t.c + 0.5) * TILE, (t.r + 0.5) * TILE);
  ctx.rotate(t.rot * Math.PI / 2);
  ctx.fillStyle = '#f1f0ec'; ctx.fillRect(-h, -h, TILE, TILE);
  ctx.strokeStyle = '#c4c2ba'; ctx.lineWidth = 1.5; ctx.strokeRect(-h + .75, -h + .75, TILE - 1.5, TILE - 1.5);
  if (t.kind === 'R') { // ramp: shaded slopes at both ends, flat top between
    const L = t.p.slope, g1 = ctx.createLinearGradient(0, -h, 0, -h + L), g2 = ctx.createLinearGradient(0, h, 0, h - L);
    [g1, g2].forEach(g => { g.addColorStop(0, 'rgba(70,90,120,0.05)'); g.addColorStop(1, 'rgba(70,90,120,0.45)'); });
    ctx.fillStyle = g1; ctx.fillRect(-h, -h, TILE, L); ctx.fillStyle = g2; ctx.fillRect(-h, h - L, TILE, L);
    ctx.fillStyle = 'rgba(70,90,120,0.45)'; ctx.fillRect(-h, -h + L, TILE, TILE - 2 * L);
  }
  if (t.kind === 'H') {
    const b = t.p.base / 2, g = ctx.createLinearGradient(0, -b, 0, b);
    g.addColorStop(0, 'rgba(120,90,60,0.05)'); g.addColorStop(.5, 'rgba(120,90,60,0.65)'); g.addColorStop(1, 'rgba(120,90,60,0.05)');
    ctx.fillStyle = g; ctx.fillRect(10, -b, 120, 2 * b);
  }
  ctx.strokeStyle = '#141414'; ctx.lineWidth = TAPE; ctx.lineCap = 'butt';
  ctx.beginPath();
  if (t.kind === 'C') ctx.arc(h, -h, h, Math.PI / 2, Math.PI);
  else if (t.kind !== '.') {
    const dirs = { S: [0, 2], B: [0, 2], O: [0, 2], R: [0, 2], H: [0, 2], T: [1, 2, 3], X: [0, 1, 2, 3] }[t.kind], V = [[0, -1], [1, 0], [0, 1], [-1, 0]];
    for (const d of dirs) { ctx.moveTo(0, 0); ctx.lineTo(V[d][0] * h, V[d][1] * h); }
  }
  ctx.stroke();
  if (t.kind === 'T' || t.kind === 'X') { ctx.fillStyle = '#141414'; ctx.fillRect(-TAPE / 2, -TAPE / 2, TAPE, TAPE); }
  if (t.kind === 'B') {
    ctx.fillStyle = 'rgba(20,20,20,' + (0.35 + 0.65 * t.p.ink) + ')';
    for (const [a, b] of t.bars) ctx.fillRect(-110, t.barLen / 2 - b, 50, b - a);
    ctx.fillStyle = '#555'; ctx.font = '22px system-ui'; ctx.textAlign = 'center'; ctx.fillText('*' + t.p.letter + '*', -85, -t.barLen / 2 - 14);
  }
  if (t.kind === 'O') {
    ctx.fillStyle = '#b4793a'; ctx.strokeStyle = '#6b4519'; ctx.lineWidth = 3;
    ctx.fillRect(t.p.offset - t.p.width / 2, -t.p.depth / 2, t.p.width, t.p.depth); ctx.strokeRect(t.p.offset - t.p.width / 2, -t.p.depth / 2, t.p.width, t.p.depth);
    ctx.fillStyle = '#fff'; ctx.font = '22px system-ui'; ctx.textAlign = 'center'; ctx.fillText(`${t.p.width}x${t.p.depth}`, t.p.offset, 8);
  }
  if (t.kind === 'R' || t.kind === 'H') {
    ctx.fillStyle = '#243247'; ctx.font = '24px system-ui'; ctx.textAlign = 'center';
    ctx.fillText((t.kind === 'R' ? 'RAMP ' : 'HUMP ') + t.p.height + ' mm', t.kind === 'H' ? 70 : 0, t.kind === 'H' ? 8 : 8);
  }
  ctx.restore();
}

function drawRobot() {
  const R = sim.robot, pts = sim.sensorPts;
  ctx.save(); ctx.translate(R.x, R.y); ctx.rotate(R.th);
  // sonar cone: pointing where the servo points; shows the last echo distance
  const az = -sim.servo * DEG, reach = sim.sonarMm ? Math.min(sim.sonarMm, 700) : 400;
  ctx.fillStyle = 'rgba(47,111,237,0.13)'; ctx.beginPath(); ctx.moveTo(ROBOT.sonarAhead, 0);
  ctx.arc(ROBOT.sonarAhead, 0, reach, az - 15 * DEG, az + 15 * DEG); ctx.closePath(); ctx.fill();
  // body
  ctx.fillStyle = 'rgba(40,90,200,0.92)'; ctx.strokeStyle = '#142b66'; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.roundRect(-ROBOT.rear, -ROBOT.width / 2, ROBOT.length, ROBOT.width, 14); ctx.fill(); ctx.stroke();
  ctx.fillStyle = '#0f1d44'; ctx.fillRect(-32, -ROBOT.track / 2 - 12, 65, 22); ctx.fillRect(-32, ROBOT.track / 2 - 10, 65, 22); // wheels
  ctx.fillStyle = '#fff'; ctx.beginPath(); ctx.moveTo(95, 0); ctx.lineTo(45, -22); ctx.lineTo(45, 22); ctx.fill(); // heading arrow
  ctx.save(); ctx.translate(ROBOT.sonarAhead, 0); ctx.rotate(az); ctx.fillStyle = '#ffd24a'; ctx.fillRect(-6, -14, 14, 28); ctx.restore(); // sonar on its servo
  ctx.restore();
  const dot = (p, on, col) => { ctx.beginPath(); ctx.arc(p[0], p[1], 9, 0, 7); ctx.fillStyle = on ? col : '#fff'; ctx.fill(); ctx.strokeStyle = '#142b66'; ctx.lineWidth = 2.5; ctx.stroke(); };
  dot(pts.l, sim.lineL, '#e03c31'); dot(pts.r, sim.lineR, '#e03c31');
  dot(pts.b, sim.analogB > 1500, '#e08a31');
}

function draw() {
  const dpr = window.devicePixelRatio || 1;
  ctx.setTransform(S * dpr, 0, 0, S * dpr, 0, 0);
  ctx.fillStyle = '#c9c7bf'; ctx.fillRect(0, 0, world.width, world.height);
  world.grid.forEach(row => row.forEach(drawTile));
  if ($('trail').checked && trail.length > 1) {
    ctx.strokeStyle = 'rgba(47,111,237,0.7)'; ctx.lineWidth = 6; ctx.lineJoin = 'round'; ctx.beginPath();
    trail.forEach((p, i) => i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])); ctx.stroke();
  }
  if (hover && !placing) { // ghost of the tile about to be placed
    const g = prepGhost({ kind: tool, rot: tool === 'X' || tool === '.' ? 0 : rot, p: { ...toolParams[tool] }, c: hover.c, r: hover.r });
    ctx.globalAlpha = 0.5; drawTile(g); ctx.globalAlpha = 1;
    ctx.strokeStyle = '#2f6fed'; ctx.lineWidth = 6; ctx.strokeRect(hover.c * TILE + 3, hover.r * TILE + 3, TILE - 6, TILE - 6);
  }
  drawRobot();
}

import { barcodeBars } from './world.js';
function prepGhost(g) { if (g.kind === 'B') { const b = barcodeBars(g.p.letter, g.p.narrow, g.p.wide); g.bars = b.bars; g.barLen = b.length; } return g; }

function drawScope() {
  const c = $('scope'), x = c.getContext('2d'), W = c.width, H = c.height, N = 4000;
  x.clearRect(0, 0, W, H); x.strokeStyle = '#e3e1da'; x.lineWidth = 1;
  [0.25, 0.5, 0.75].forEach(f => { x.beginPath(); x.moveTo(0, H * f); x.lineTo(W, H * f); x.stroke(); });
  x.strokeStyle = '#d0782c'; x.lineWidth = 1.6; x.beginPath();
  for (let i = 0; i < N; i += 2) {
    const v = scope[(scopeI - N + i + scope.length * 2) % scope.length], px = i / N * W, py = H - 4 - (v / 4095) * (H - 8);
    i ? x.lineTo(px, py) : x.moveTo(px, py);
  }
  x.stroke();
}

function updateReadouts() {
  const R = sim.robot;
  $('k_t').textContent = (sim.nowUs / 1e6).toFixed(1) + ' s';
  const l = $('k_l'), r = $('k_r'); l.textContent = sim.lineL; r.textContent = sim.lineR; l.className = 'pill' + (sim.lineL ? ' black' : ''); r.className = 'pill' + (sim.lineR ? ' black' : '');
  $('k_b').textContent = Math.round(sim.analogB);
  $('k_v').textContent = `${R.vL.toFixed(0)} / ${R.vR.toFixed(0)}`;
  $('k_d').textContent = (sim.dist || 0).toFixed(0) + ' mm';
  $('k_s').textContent = `${sim.servo.toFixed(0)} deg / ${sim.sonarMm ? sim.sonarMm.toFixed(0) + ' mm' : 'no echo'}`;
  $('k_p').textContent = `${(R.pitch / DEG).toFixed(1)} / ${(R.roll / DEG).toFixed(1)}`;
}

// ---- run loop ----------------------------------------------------------------------------------
let last = performance.now(), acc = 0, frame = 0;
function loop(now) {
  const dt = Math.min(0.05, (now - last) / 1000); last = now;
  if (sim && running && !placing) {
    acc += dt * +$('speed').value / 0.0005;
    let n = Math.min(Math.floor(acc), 24000); acc -= Math.floor(acc);
    for (let i = 0; i < n; i++) {
      sim.step(); scope[scopeI++ % scope.length] = sim.analogB;
      if (i % 200 === 0) sim.drainMqtt();
    }
    const R = sim.robot, lp = trail[trail.length - 1];
    if (!lp || Math.hypot(lp[0] - R.x, lp[1] - R.y) > 6) { trail.push([R.x, R.y]); if (trail.length > 6000) trail.shift(); }
  }
  if (sim && (frame++ % 2 === 0)) { draw(); drawScope(); updateReadouts(); }
  requestAnimationFrame(loop);
}

// ---- editing -------------------------------------------------------------------------------------
function paletteUI() {
  const pal = $('palette'); pal.innerHTML = '';
  for (const k of Object.keys(KINDS)) {
    const b = document.createElement('button'); b.textContent = KINDS[k]; b.dataset.k = k;
    b.onclick = () => { tool = k; placing = false; refreshUI(); };
    pal.appendChild(b);
  }
}
const FIELD_LABELS = { letter: 'Letter', narrow: 'Narrow mm', wide: 'Wide mm', ink: 'Ink 0-1', width: 'Width mm', depth: 'Depth mm', offset: 'Offset mm', height: 'Height mm', slope: 'Slope mm', base: 'Base mm' };
function refreshUI() {
  [...$('palette').children].forEach(b => b.classList.toggle('on', !placing && b.dataset.k === tool));
  $('placeBtn').classList.toggle('on', placing);
  $('rotLbl').textContent = 'rot ' + rot;
  const pr = $('params'); pr.innerHTML = '';
  const def = toolParams[tool];
  if (def) for (const key of Object.keys(def)) {
    const lab = document.createElement('label'), inp = document.createElement('input');
    inp.type = key === 'letter' ? 'text' : 'number'; inp.value = def[key]; if (key === 'letter') { inp.style.width = '34px'; inp.maxLength = 1; }
    inp.step = key === 'ink' ? 0.05 : 1;
    inp.onchange = () => { toolParams[tool][key] = key === 'letter' ? inp.value.toUpperCase() : parseFloat(inp.value); };
    lab.append(FIELD_LABELS[key] + ' ', inp); pr.appendChild(lab);
  }
  $('runBtn').textContent = running ? 'Pause' : 'Run';
}

function cellAt(e) {
  const rect = cv.getBoundingClientRect();
  const x = (e.clientX - rect.left) / rect.width * world.width, y = (e.clientY - rect.top) / rect.height * world.height;
  return { x, y, c: Math.floor(x / TILE), r: Math.floor(y / TILE) };
}
const inGrid = p => p.r >= 0 && p.c >= 0 && p.r < world.rows && p.c < world.cols;

function layoutChanged() { world.build(); $('txt').value = world.serialize(); }
function paint(p) {
  if (!inGrid(p)) return;
  world.grid[p.r][p.c] = { kind: tool, rot: tool === 'X' || tool === '.' ? 0 : rot, p: { ...toolParams[tool] }, c: p.c, r: p.r };
  layoutChanged();
}
function resizeGrid(nc, nr) {
  nc = Math.max(1, Math.min(14, nc)); nr = Math.max(1, Math.min(12, nr));
  const g = [];
  for (let r = 0; r < nr; r++) { g.push([]); for (let c = 0; c < nc; c++) g[r].push(world.grid[r]?.[c] ?? { kind: '.', rot: 0, p: {} }); }
  world.grid = g; world.rows = nr; world.cols = nc; world.width = nc * TILE; world.height = nr * TILE;
  layoutChanged(); resizeCanvas();
}

let placeStart = null;
cv.addEventListener('contextmenu', e => {
  e.preventDefault(); const p = cellAt(e); if (!inGrid(p)) return;
  const t = world.grid[p.r][p.c]; if (t.kind !== '.' && t.kind !== 'X') { t.rot = (t.rot + 1) % 4; layoutChanged(); }
});
cv.addEventListener('pointerdown', e => {
  if (e.button !== 0) return;
  const p = cellAt(e);
  cv.setPointerCapture(e.pointerId);
  if (placing) { placeStart = p; startPose = { x: p.x, y: p.y, th: sim.robot.th }; sim.place(p.x, p.y, sim.robot.th); trail = []; return; }
  if (!inGrid(p)) return; painting = true; paint(p);
});
cv.addEventListener('pointermove', e => {
  const p = cellAt(e); hover = inGrid(p) ? p : null;
  if (placing && placeStart && (e.buttons & 1)) { const th = Math.atan2(p.y - placeStart.y, p.x - placeStart.x); startPose = { x: placeStart.x, y: placeStart.y, th }; sim.place(placeStart.x, placeStart.y, th); }
  else if (painting && hover) paint(p);
});
cv.addEventListener('pointerup', () => { painting = false; if (placing && placeStart) { placeStart = null; placing = false; refreshUI(); boot(); } });
cv.addEventListener('pointerleave', () => { hover = null; });
window.addEventListener('keydown', e => { if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return; if (e.key === 'r' || e.key === 'R') { rot = (rot + 1) % 4; refreshUI(); } });

// ---- buttons ------------------------------------------------------------------------------------------
$('rotBtn').onclick = () => { rot = (rot + 1) % 4; refreshUI(); };
$('clearBtn').onclick = () => { world.grid = world.grid.map(r => r.map((_, c) => ({ kind: '.', rot: 0, p: {}, c: 0, r: 0 }))); layoutChanged(); };
$('cols').onchange = () => resizeGrid(+$('cols').value, +$('rows').value);
$('rows').onchange = () => resizeGrid(+$('cols').value, +$('rows').value);
$('runBtn').onclick = () => { running = !running; refreshUI(); };
$('resetBtn').onclick = () => { boot(); };
$('placeBtn').onclick = () => { placing = !placing; refreshUI(); setStatus(placing ? 'click a spot on the track, drag to aim, release' : 'ready'); };
$('startBtn').onclick = () => sim.start();
$('startBtn2').onclick = () => sim.pressStartButton();
$('stopBtn').onclick = () => { sim.stop(); sim.pressStopButton(); };
$('mapBtn').onclick = () => sim.sendMapping($('map_in').value);
$('trail').onchange = () => { if (!$('trail').checked) trail = []; };
$('clearLog').onclick = () => { $('log').textContent = ''; };
$('exportBtn').onclick = () => { $('txt').value = world.serialize(); };
$('importBtn').onclick = () => {
  try { world.load($('txt').value); $('cols').value = world.cols; $('rows').value = world.rows; $('txtMsg').textContent = ''; resizeCanvas(); startPose = null; boot(); }
  catch (err) { $('txtMsg').textContent = err.message; }
};
$('fwfile').onchange = async e => { const f = e.target.files[0]; if (!f) return; fwBytes = await f.arrayBuffer(); await loadFirmware(); await boot(); setStatus('loaded ' + f.name); };
$('loadPreset').onclick = async () => {
  const p = PRESETS[$('presets').value]; world.load(p.text); $('cols').value = world.cols; $('rows').value = world.rows;
  startPose = startFromTile(...p.start); resizeCanvas(); $('txt').value = world.serialize(); await boot();
};
window.addEventListener('resize', resizeCanvas);

// ---- go --------------------------------------------------------------------------------------------------------
(async function init() {
  for (const k of Object.keys(PRESETS)) { const o = document.createElement('option'); o.textContent = k; $('presets').appendChild(o); }
  const first = PRESETS[Object.keys(PRESETS)[0]]; startPose = startFromTile(...first.start);
  $('cols').value = world.cols; $('rows').value = world.rows; $('txt').value = world.serialize();
  paletteUI(); refreshUI(); resizeCanvas();
  try { await loadFirmware(); await boot(); }
  catch (e) { setStatus(String(e.message || e)); addLog(0, 'ERROR ' + (e.message || e)); }
  requestAnimationFrame(loop);
})();

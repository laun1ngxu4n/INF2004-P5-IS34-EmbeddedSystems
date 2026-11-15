// world.js: the track, the barcode strips, the obstacle and the ramp.
// Pure geometry (no DOM), shared by the browser page and the Node tests.
// Units: millimetres.  x to the right, y DOWN (screen), like the canvas.

export const TILE = 300;       // tile side, mm (setting: Cytron-style foam tile)
export const TAPE = 20;        // tape width, mm (S3_TAPE_MM)
const DIRV = [[0, -1], [1, 0], [0, 1], [-1, 0]]; // N E S W
const BASE = { S: [0, 2], C: [0, 1], T: [1, 2, 3], X: [0, 1, 2, 3], B: [0, 2], O: [0, 2], R: [0, 2], H: [0, 2], '.': [] };
export const KINDS = { S: 'Straight', C: 'Curve', T: 'T-junction', X: 'Crossing', B: 'Barcode', O: 'Obstacle', R: 'Ramp', H: 'Hump', '.': 'Empty' };

export function edges(kind, rot) { return BASE[kind].map(d => (d + rot) % 4); }

// ---- Code 39 (same table as app_program/subsystem3/barcode.c) ----------
const C39 = { '0': 0x034, '1': 0x121, '2': 0x061, '3': 0x160, '4': 0x031, '5': 0x130, '6': 0x070, '7': 0x025, '8': 0x124, '9': 0x064,
  A: 0x109, B: 0x049, C: 0x148, D: 0x019, E: 0x118, F: 0x058, G: 0x00D, H: 0x10C, I: 0x04C, J: 0x01C, K: 0x103, L: 0x043, M: 0x142,
  N: 0x013, O: 0x112, P: 0x052, Q: 0x007, R: 0x106, S: 0x046, T: 0x016, U: 0x181, V: 0x0C1, W: 0x1C0, X: 0x091, Y: 0x190, Z: 0x0D0,
  '-': 0x085, '.': 0x184, ' ': 0x0C4, '*': 0x094 };

// Bars of "*L*": list of [start, end] along the travel direction, mm from the start.
export function barcodeBars(letter, narrow = 4, wide = 10) {
  const bars = []; let pos = 0;
  const text = '*' + letter + '*';
  for (let ci = 0; ci < text.length; ci++) {
    const pat = C39[text[ci]] ?? C39['K'];
    for (let e = 0; e < 9; e++) {
      const w = (pat >> (8 - e)) & 1 ? wide : narrow;
      if (e % 2 === 0) bars.push([pos, pos + w]);
      pos += w;
    }
    pos += narrow; // gap between characters
  }
  return { bars, length: pos - narrow };
}

// ---- small maths ---------------------------------------------------------
function erf(x) { // Abramowitz-Stegun 7.1.26
  const s = Math.sign(x), a = Math.abs(x), t = 1 / (1 + 0.3275911 * a);
  const y = 1 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t * Math.exp(-a * a);
  return s * y;
}
const Phi = z => 0.5 * (1 + erf(z / Math.SQRT2));
function distSeg(px, py, ax, ay, bx, by) {
  const dx = bx - ax, dy = by - ay;
  const t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)));
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
}

export const DEFAULT_PARAMS = {
  B: { letter: 'K', narrow: 4, wide: 10, ink: 0.8 },
  O: { width: 150, depth: 100, offset: 0 },
  R: { height: 20, slope: 110 },
  H: { height: 30, base: 100 },
};

export class World {
  constructor(layoutText) { this.load(layoutText); }

  load(text) {
    const rows = text.trim().split(/\n+/).map(l => l.trim().split(/\s+/).map(tok => {
      const m = /^([A-Za-z.])(\d?)(?::(.*))?$/.exec(tok);
      if (!m) throw new Error("Bad tile '" + tok + "'");
      const kind = m[1].toUpperCase();
      if (!(kind in BASE)) throw new Error("Unknown tile '" + tok + "' (use S C T X B O R H .)");
      const rot = kind === '.' || kind === 'X' ? 0 : ((parseInt(m[2] || '0', 10) % 4) + 4) % 4;
      const p = { ...(DEFAULT_PARAMS[kind] || {}) };
      if (m[3]) {
        const parts = m[3].split(',');
        const keys = Object.keys(p);
        parts.forEach((v, i) => { if (keys[i] !== undefined && v !== '') p[keys[i]] = keys[i] === 'letter' ? v.toUpperCase() : parseFloat(v); });
      }
      return { kind, rot, p };
    }));
    const w = Math.max(...rows.map(r => r.length));
    rows.forEach(r => { while (r.length < w) r.push({ kind: '.', rot: 0, p: {} }); });
    this.grid = rows;
    this.rows = rows.length; this.cols = w;
    this.width = w * TILE; this.height = rows.length * TILE;
    this.build();
  }

  serialize() {
    return this.grid.map(row => row.map(t => {
      if (t.kind === '.') return '.';
      if (t.kind === 'X') return 'X';
      const def = DEFAULT_PARAMS[t.kind];
      let s = t.kind + t.rot;
      if (def && Object.keys(def).some(k => t.p[k] !== def[k])) s += ':' + Object.keys(def).map(k => t.p[k]).join(',');
      return s;
    }).join(' ')).join('\n');
  }

  // Pre-compute barcode strips and obstacle boxes in world coordinates.
  build() {
    this.boxes = []; this.strips = [];
    this.grid.forEach((row, r) => row.forEach((t, c) => {
      t.c = c; t.r = r;
      if (t.kind === 'O') {
        const cx = (c + 0.5) * TILE, cy = (r + 0.5) * TILE;
        this.boxes.push({ ...this.toWorld(t, t.p.offset, 0), hw: t.p.width / 2, hd: t.p.depth / 2, rot: t.rot, cx0: cx, cy0: cy });
      }
      if (t.kind === 'B') { const b = barcodeBars(t.p.letter, t.p.narrow, t.p.wide); t.bars = b.bars; t.barLen = b.length; }
    }));
  }

  // canonical tile coords (relative to the tile centre, travelling north) -> world
  toWorld(t, x, y) {
    let X = x, Y = y;
    for (let i = 0; i < t.rot; i++) { const nx = -Y, ny = X; X = nx; Y = ny; } // 90 deg clockwise on screen
    return { cx: (t.c + 0.5) * TILE + X, cy: (t.r + 0.5) * TILE + Y };
  }
  // world point -> { tile, x, y } in the tile's canonical frame (centre origin)
  toCanon(wx, wy) {
    const c = Math.floor(wx / TILE), r = Math.floor(wy / TILE);
    if (r < 0 || c < 0 || r >= this.rows || c >= this.cols) return null;
    const t = this.grid[r][c];
    let x = wx - (c + 0.5) * TILE, y = wy - (r + 0.5) * TILE;
    for (let i = 0; i < t.rot; i++) { const nx = y, ny = -x; x = nx; y = ny; } // inverse rotation
    return { t, x, y };
  }

  // distance from (wx,wy) to the nearest tape centreline in its tile (large if none)
  tapeDist(wx, wy) {
    const q = this.toCanon(wx, wy); if (!q) return 1e9;
    const { t, x, y } = q, h = TILE / 2;
    if (t.kind === '.') return 1e9;
    if (t.kind === 'C') {
      // canonical curve joins N and E: arc round the NE corner (h, -h)
      return Math.abs(Math.hypot(x - h, y + h) - h);
    }
    let best = 1e9;
    const e = BASE[t.kind];
    for (const d of e) best = Math.min(best, distSeg(x, y, 0, 0, DIRV[d][0] * h, DIRV[d][1] * h));
    return best;
  }

  // how much black the analogue barcode spot (sigma mm) sees: 0..1
  inkCover(wx, wy, sigma = 1.4) {
    const tape = this.tapeDist(wx, wy);
    let cover = tape < 1e8 ? Phi((TAPE / 2 - tape) / sigma) - Phi((-TAPE / 2 - tape) / sigma) : 0;
    const q = this.toCanon(wx, wy);
    if (q && q.t.kind === 'B') {
      const t = q.t, start = TILE / 2 + t.barLen / 2; // south end; travel is north (y decreasing)
      if (q.y > -t.barLen / 2 - 6 * sigma && q.y < t.barLen / 2 + 6 * sigma) {
        const lat = Phi((q.x + 110) / sigma) - Phi((q.x + 60) / sigma);
        let s = 0;
        for (const [a, b] of t.bars) {
          const y1 = (t.barLen / 2) - b, y2 = (t.barLen / 2) - a; // canonical y of the bar's two edges
          s += Phi((q.y - y1) / sigma) - Phi((q.y - y2) / sigma);
        }
        cover = Math.max(cover, t.p.ink * lat * Math.min(1, s));
      }
    }
    return cover;
  }

  // ground height (mm) under a world point: ramp and hump tiles
  groundHeight(wx, wy) {
    const q = this.toCanon(wx, wy); if (!q) return 0;
    const { t, x, y } = q;
    if (t.kind === 'R') {
      const L = t.p.slope, Ht = t.p.height, h = TILE / 2;
      const ay = Math.abs(y);
      if (ay >= h) return 0;
      // rises over [-h, -h+L] ... plateau ... falls over [h-L, h]
      const d = h - ay; // distance from the nearer tile edge
      return Ht * Math.min(1, d / L);
    }
    if (t.kind === 'H') {
      const hw = t.p.base / 2;
      if (Math.abs(y) >= hw || x < 10 || x > 130) return 0;
      return t.p.height * (1 - Math.abs(y) / hw);
    }
    return 0;
  }

  // nearest sonar echo along (x,y,dir) inside a cone: mm, or Infinity
  sonar(x, y, dir, halfCone = 15 * Math.PI / 180, maxDist = 3000) {
    let best = Infinity;
    for (let a = -halfCone; a <= halfCone + 1e-9; a += halfCone / 5) {
      const ang = dir + a, dx = Math.cos(ang), dy = Math.sin(ang);
      for (const b of this.boxes) {
        const hit = rayBox(x, y, dx, dy, b);
        if (hit && hit.t < best && hit.t < maxDist) {
          if (Math.abs(a) <= halfCone && hit.inc < 62 * Math.PI / 180) best = hit.t; // too slanted: echo glances off
        }
      }
    }
    return best;
  }

  // does the robot rectangle (centre, heading, half sizes) touch a box?
  hitsBox(rx, ry, th, hl, hw) {
    for (const b of this.boxes) {
      // sample points around the robot outline, test inside the box
      for (let i = 0; i <= 8; i++) for (let j = 0; j <= 4; j++) {
        const u = (i / 4 - 1) * hl, v = (j / 2 - 1) * hw;
        const px = rx + u * Math.cos(th) - v * Math.sin(th), py = ry + u * Math.sin(th) + v * Math.cos(th);
        if (pointInBox(px, py, b)) return true;
      }
    }
    return false;
  }
}

function boxAxes(b) { // box rotated like its tile: canonical x axis -> world
  const a = b.rot * Math.PI / 2; return { ux: Math.cos(a), uy: Math.sin(a) };
}
function pointInBox(px, py, b) {
  const { ux, uy } = boxAxes(b), dx = px - b.cx, dy = py - b.cy;
  const u = dx * ux + dy * uy, v = -dx * uy + dy * ux;
  return Math.abs(u) <= b.hw && Math.abs(v) <= b.hd;
}
function rayBox(ox, oy, dx, dy, b) { // slab test in the box frame
  const { ux, uy } = boxAxes(b);
  const o = [(ox - b.cx) * ux + (oy - b.cy) * uy, -(ox - b.cx) * uy + (oy - b.cy) * ux];
  const d = [dx * ux + dy * uy, -dx * uy + dy * ux];
  const h = [b.hw, b.hd];
  let t0 = 0, t1 = Infinity, axisHit = -1;
  for (let a = 0; a < 2; a++) {
    if (Math.abs(d[a]) < 1e-9) { if (Math.abs(o[a]) > h[a]) return null; continue; }
    let ta = (-h[a] - o[a]) / d[a], tb = (h[a] - o[a]) / d[a];
    if (ta > tb) [ta, tb] = [tb, ta];
    if (ta > t0) { t0 = ta; axisHit = a; }
    if (tb < t1) t1 = tb;
    if (t0 > t1) return null;
  }
  if (axisHit < 0) return null;
  return { t: t0, inc: Math.acos(Math.min(1, Math.abs(d[axisHit]))) }; // angle to the face normal
}

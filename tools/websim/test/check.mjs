// Regression check: run the real firmware on small courses and look for the events it should report.
//   node tools/websim/test/check.mjs        (after sh tools/websim/build.sh)
import { readFileSync } from 'node:fs';
import { Sim, World, TILE } from '../web/sim.js';

const wasm = readFileSync(new URL('../web/firmware.wasm', import.meta.url));
const here = f => new URL(f, import.meta.url);
const cases = [
  { file: 'barcode_junction.txt', start: [2, 4, 'N'], secs: 40, expect: [/barcode K .* LEFT at next junction/, /junction 1 \(crossing\): LEFT/, /stopped: line lost/] },
  { file: 'obstacle.txt', start: [1, 4, 'N'], secs: 60, expect: [/obstacle width \d+ \+\/- \d+ mm/, /bypass (LEFT|RIGHT)/, /bypass done/] },
  { file: 'ramp.txt', start: [1, 3, 'N'], secs: 25, expect: [/ramp peak height 1\d\.\d mm/] },
  { file: 'hump.txt', start: [1, 3, 'N'], secs: 25, expect: [/ramp peak height \d+\.\d mm \(one wheel/] },
  { file: 'sbend.txt', start: [1, 2, 'N'], secs: 25, expect: [/stopped: line lost/] },
  { file: 'loop_course.txt', start: [0, 6, 'N'], secs: 120, expect: [/ramp peak/, /obstacle width/, /bypass done/, /barcode Q .* RIGHT/, /junction 1 \(crossing\): RIGHT/] },
];
let failed = 0;
for (const c of cases) {
  const world = new World(readFileSync(here(c.file), 'utf8'));
  const sim = new Sim(world), lines = [];
  sim.onLog = (t, l) => lines.push(l);
  await sim.load(wasm);
  const th = { N: -Math.PI / 2, E: 0, S: Math.PI / 2, W: Math.PI }[c.start[2]];
  sim.place((c.start[0] + 0.5) * TILE, (c.start[1] + 0.5) * TILE + 60, th);
  sim.run(2); sim.sendMapping('K=L,Q=R,X=F,M=U'); sim.run(1); sim.start(); sim.run(c.secs);
  const missing = c.expect.filter(re => !lines.some(l => re.test(l)));
  console.log((missing.length ? 'FAIL ' : 'ok   ') + c.file + (missing.length ? '  missing: ' + missing.join(' ') : ''));
  if (missing.length) failed++;
}
process.exit(failed ? 1 : 0);

// Headless check: boot the real firmware and drive it on a track.
//   node tools/websim/test/run.mjs <layout file | -> [seconds] [startCol startRow dir(N|E|S|W)]
import { readFileSync } from 'node:fs';
import { Sim, World, TILE } from '../web/sim.js';

const arg = process.argv[2];
const layout = arg && arg !== '-' ? readFileSync(arg, 'utf8') : 'S0\nS0\nS0\nS0\nS0';
const secs = +process.argv[3] || 10;
const world = new World(layout);
const sim = new Sim(world);
const quiet = process.env.QUIET;
sim.onLog = (t, line) => { if (!quiet && !line.startsWith('[tmg]')) console.log(`[${t.toFixed(2)}] ${line}`); };
await sim.load(readFileSync(new URL('../web/firmware.wasm', import.meta.url)));
const col = process.argv[4] !== undefined ? +process.argv[4] : world.cols - 1, row = process.argv[5] !== undefined ? +process.argv[5] : world.rows - 1;
const dir = (process.argv[6] || 'N').toUpperCase();
const th = { N: -Math.PI / 2, E: 0, S: Math.PI / 2, W: Math.PI }[dir];
sim.place((col + 0.5) * TILE, (row + 0.5) * TILE + (dir === 'N' ? 120 : 0), th);
const t0 = Date.now();
sim.run(2);
sim.sendMapping('K=L,Q=R,X=F,M=U');
sim.run(1);
sim.start();
const R = sim.robot;
for (let s = 0; s < secs; s++) {
  sim.run(1);
  if (process.env.TRACE) console.log(`  t=${(sim.nowUs / 1e6).toFixed(0)} pose=(${R.x.toFixed(0)},${R.y.toFixed(0)}) th=${(R.th * 180 / Math.PI).toFixed(0)} L${sim.lineL}R${sim.lineR} pitch=${(R.pitch * 57.3).toFixed(1)} servo=${sim.servo.toFixed(0)} sonar=${sim.sonarMm?.toFixed(0)}`);
}
console.log(`END pose x=${R.x.toFixed(0)} y=${R.y.toFixed(0)} th=${(R.th * 180 / Math.PI).toFixed(0)} odo=${sim.dist.toFixed(0)}mm  wall=${Date.now() - t0}ms collided=${R.collided}`);

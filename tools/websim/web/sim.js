// sim.js: runs the REAL robot firmware (firmware.wasm, built from app_program/)
// against a simulated Pico W + robot + track.
//
//   Firmware (wasm)  <->  Chip (GPIO, PWM, I2C IMU, ADC, timer)  <->  Robot physics  <->  World
//
// Works in the browser and in Node (see ../test/run.mjs).

import { World, TILE, TAPE } from './world.js';

const DT_US = 500;                     // simulation step: 0.5 ms (ADC runs at 2 kHz)
const TICK_US = 10000;                 // kernel tick: 10 ms
const IO_BANK0 = 0x40014000, PWM_BASE = 0x40050000, I2C1 = 0x40048000, ADC_BASE = 0x4004C000;
const GPIO_IN = 0xD0000004, TIMER_RAW = 0x40054028, RESETS_DONE = 0x4000C008;
const INTNO_GPIO = 13, INTNO_ADC = 22;

// ---- the robot (measured values from app_program headers / CONTEXT.md) -----
export const ROBOT = {
  track: 123,            // S2_TRACK_MM
  wheelMmPerCount: 0.080, // S2_UM_PER_COUNT
  lineAhead: 115,        // S3_SENSOR_TO_AXLE_MM
  lineGap: 30,           // S3_LR_GAP_MM
  barAhead: 115 - 40,    // S3_BAR_AHEAD_MM = -40 (behind the line sensors)
  barLeft: 15 + 70,      // S3_BAR_LEFT_MM = 70 left of the left line sensor
  sonarAhead: 120,       // S5_SONAR_TO_AXLE_MM
  casterAhead: 80,       // S4_L_MM, axle to front support
  length: 200, width: 170, rear: 70, // S2_ROBOT_LENGTH_MM / WIDTH, axle 70 mm from the back
  // no-load wheel speed (mm/s) vs duty, measured 26 Oct, wheels lifted
  speedTable: [[0, 0], [0.10, 0], [0.30, 240], [0.40, 325], [0.50, 407], [0.70, 573], [1.00, 798]],
  motorTau: 0.045,       // s, first-order lag of wheel speed
  tapeSensorWindow: 14,  // mm: tape width the IR line sensors react to
  imuZOffsetG: 0.12,     // LSM303D Z reads 0.12-0.15 g high (CONTEXT.md section 4)
};

function motorSpeed(duty) { // duty -1..1 -> mm/s
  const a = Math.abs(duty), T = ROBOT.speedTable;
  let v = T[T.length - 1][1];
  for (let i = 1; i < T.length; i++) if (a <= T[i][0]) { v = T[i - 1][1] + (T[i][1] - T[i - 1][1]) * (a - T[i - 1][0]) / (T[i][0] - T[i - 1][0]); break; }
  return Math.sign(duty) * v;
}

// small deterministic noise
let seed = 12345;
const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
const gauss = () => Math.sqrt(-2 * Math.log(rnd() + 1e-12)) * Math.cos(2 * Math.PI * rnd());

export class Sim {
  constructor(world) {
    this.world = world;
    this.log = [];                 // console lines from the firmware
    this.onLog = null;
    this.nowUs = 0; this.extra = 0;
    this.regs = new Map();
    this.pinOut = new Array(30).fill(0); this.pinDir = new Array(30).fill(0); this.pinExt = new Array(30).fill(0);
    this.intr = [0, 0, 0, 0]; this.inte = [0, 0, 0, 0];
    this.pwm = Array.from({ length: 8 }, () => ({ top: 0xFFFF, a: 0, b: 0, en: 0 }));
    this.i2c = { tar: 0, phase: 'idle', reg: 0, rx: [], aborted: false };
    this.adc = { on: false, many: false, fifo: [], inte: 0 };
    this.events = [];              // timed pin changes (sonar echo)
    this.trig = { high: false, t: 0 };
    this.irqGpio = false;
    this.pinExt[20] = 1; this.pinExt[21] = 1; // START / STOP buttons: pulled up
    this.robot = { x: 0, y: 0, th: 0, vL: 0, vR: 0, aFwd: 0, vPrev: 0, collided: false, pitch: 0, roll: 0 };
    this.servo = 0;                // deg to the left of straight ahead
    this.enc = [{ acc: 0, ph: 0 }, { acc: 0, ph: 0 }];
    this.sonarMm = null; this.analogB = 300; this.lineL = 0; this.lineR = 0;
    this.stepCount = 0;
  }

  async load(wasmBytes) {
    const env = this.imports();
    const { instance } = await WebAssembly.instantiate(wasmBytes, { env });
    this.fw = instance.exports; this.mem = this.fw.memory;
    this.sp = this.fw.__stack_pointer; this.mainSP = this.sp.value;
    this.scratch = this.fw.__heap_base.value ?? this.fw.__heap_base;
    for (let i = 1; i <= 16; i++) { // Asyncify save areas: [start, end] header
      const b = this.fw.task_astack(i), u = new Uint32Array(this.mem.buffer, b, 2);
      u[0] = b + 8; u[1] = b + this.fw.task_astack_size();
    }
    this.fw.kernel_boot();
    this.schedule();
  }

  // ---------------------------------------------------------------- wasm imports
  imports() {
    const S = this;
    return {
      hw_read: a => S.hwRead(a >>> 0),
      hw_write: (a, v) => S.hwWrite(a >>> 0, v >>> 0),
      hw_set: (a, b) => S.hwWrite(a >>> 0, (S.hwRead(a >>> 0) | b) >>> 0),
      hw_clr: (a, b) => S.hwWrite(a >>> 0, (S.hwRead(a >>> 0) & ~b) >>> 0),
      sim_yield: () => {
        if (S.fw.asyncify_get_state() === 0) S.fw.asyncify_start_unwind(S.fw.task_astack(S.fw.kernel_cur()));
        else S.fw.asyncify_stop_rewind();
      },
      sim_log: (p, n) => { // tm_printf pieces: collect until a newline
        S.logBuf = (S.logBuf || '') + new TextDecoder().decode(new Uint8Array(S.mem.buffer, p, n));
        let i;
        while ((i = S.logBuf.indexOf('\n')) >= 0) {
          const line = S.logBuf.slice(0, i).replace(/\r$/, ''); S.logBuf = S.logBuf.slice(i + 1);
          if (line.length) { S.log.push({ t: S.nowUs / 1e6, text: line }); if (S.onLog) S.onLog(S.nowUs / 1e6, line); }
        }
      },
      gpio_hw_pin_mode: (pin, mode) => { S.pinDir[pin] = mode; },
      gpio_hw_pin_val: (pin, v) => S.driveOut(pin, v),
      pwm_hw_pin: () => {},
      pwm_hw_wrap: (pin, w) => { S.pwm[(pin >> 1) & 7].top = w; },
      pwm_hw_cc: (pin, cc) => { const s = S.pwm[(pin >> 1) & 7]; if (pin & 1) s.b = cc; else s.a = cc; },
      pwm_hw_enable: (pin, on) => { S.pwm[(pin >> 1) & 7].en = on; },
    };
  }

  // ---------------------------------------------------------------- registers
  timerUs() { const t = (this.nowUs + this.extra) >>> 0; this.extra += 2; return t; }

  pinLevel(p) { return this.pinDir[p] ? this.pinOut[p] : this.pinExt[p]; }

  hwRead(a) {
    if (a === GPIO_IN) { let v = 0; for (let p = 0; p < 30; p++) v |= this.pinLevel(p) << p; return v >>> 0; }
    if (a === TIMER_RAW) return this.timerUs();
    if (a === RESETS_DONE) return 0xFFFFFFFF;
    if (a >= IO_BANK0 + 0xF0 && a < IO_BANK0 + 0x100) return this.intr[(a - IO_BANK0 - 0xF0) >> 2];
    if (a >= IO_BANK0 + 0x100 && a < IO_BANK0 + 0x110) return this.inte[(a - IO_BANK0 - 0x100) >> 2];
    if (a >= IO_BANK0 + 0x120 && a < IO_BANK0 + 0x130) { const r = (a - IO_BANK0 - 0x120) >> 2; return (this.intr[r] & this.inte[r] & 0xCCCCCCCC) >>> 0; }
    if (a >= I2C1 && a < I2C1 + 0x100) return this.i2cRead(a - I2C1);
    if (a >= ADC_BASE && a < ADC_BASE + 0x20) return this.adcRead(a - ADC_BASE);
    if (a >= PWM_BASE && a < PWM_BASE + 0xA0) { const o = (a - PWM_BASE) % 0x14, s = this.pwm[Math.floor((a - PWM_BASE) / 0x14)]; if (o === 0x0C) return (s.a | (s.b << 16)) >>> 0; if (o === 0x10) return s.top; }
    return (this.regs.get(a) ?? 0) >>> 0;
  }

  hwWrite(a, v) {
    if (a >= IO_BANK0 + 0xF0 && a < IO_BANK0 + 0x100) { this.intr[(a - IO_BANK0 - 0xF0) >> 2] &= ~v; return; }   // write 1 to clear
    if (a >= IO_BANK0 + 0x100 && a < IO_BANK0 + 0x110) { this.inte[(a - IO_BANK0 - 0x100) >> 2] = v; this.irqGpio = true; return; }
    if (a >= I2C1 && a < I2C1 + 0x100) { this.i2cWrite(a - I2C1, v); return; }
    if (a >= ADC_BASE && a < ADC_BASE + 0x20) { this.adcWrite(a - ADC_BASE, v); return; }
    if (a >= PWM_BASE && a < PWM_BASE + 0xA0) {
      const o = (a - PWM_BASE) % 0x14, s = this.pwm[Math.floor((a - PWM_BASE) / 0x14)];
      if (o === 0x0C) { s.a = v & 0xFFFF; s.b = v >>> 16; } else if (o === 0x10) s.top = v; else if (o === 0x00) s.en = v & 1;
      return;
    }
    this.regs.set(a, v >>> 0);
  }

  // GPIO output from the firmware (TRIG is the one the world cares about)
  driveOut(pin, v) {
    this.pinOut[pin] = v ? 1 : 0;
    if (pin === 6) {
      const t = this.nowUs + this.extra;
      if (v) this.trig = { high: true, t };
      else if (this.trig.high) { this.trig.high = false; this.startPing(t, t - this.trig.t); }
    }
  }

  setPin(pin, level) { // an external input changes: latch an edge, maybe raise the interrupt
    level = level ? 1 : 0;
    if (this.pinExt[pin] === level) return;
    this.pinExt[pin] = level;
    const r = pin >> 3, bit = (level ? 8 : 4) << ((pin & 7) * 4);
    this.intr[r] |= bit;
    if (this.inte[r] & bit) this.irqGpio = true;
  }

  deliverIrqs() {
    let guard = 0;
    while (this.irqGpio && guard++ < 8) {
      this.irqGpio = false;
      if (this.intr.some((v, r) => (v & this.inte[r] & 0xCCCCCCCC) !== 0)) this.fw.kernel_irq(INTNO_GPIO);
    }
  }

  // ---------------------------------------------------------------- I2C (LSM303D at 0x1D)
  i2cRead(o) {
    const I = this.i2c;
    switch (o) {
      case 0x10: return I.rx.length ? I.rx.shift() : 0;
      case 0x34: return I.aborted ? 0x40 : 0;
      case 0x70: return 0x04; // TFE
      case 0x78: return I.rx.length;
      default: return (this.regs.get(I2C1 + o) ?? 0) >>> 0;
    }
  }
  i2cWrite(o, v) {
    const I = this.i2c;
    if (o === 0x04) { I.tar = v & 0x7F; return; }
    if (o === 0x54) { I.aborted = false; return; }
    if (o !== 0x10) { this.regs.set(I2C1 + o, v); return; }
    if (I.tar !== 0x1D) { I.aborted = true; return; }
    const isRead = (v & 0x100) !== 0, stop = (v & 0x200) !== 0;
    if (!isRead) {
      if (I.phase === 'idle') { I.reg = v & 0xFF; I.phase = 'gotReg'; }
      else if (I.phase === 'gotReg') { this.imuWrite(I.reg & 0x7F, v & 0xFF); I.phase = 'idle'; }
      if (stop) I.phase = 'idle';
    } else {
      if (I.phase === 'gotReg') { I.ptr = I.reg & 0x7F; I.inc = (I.reg & 0x80) !== 0; I.phase = 'reading'; if (I.inc) I.snap = this.imuSnapshot(); }
      I.rx.push(this.imuRead(I.ptr, I.snap));
      if (I.inc) I.ptr++;
      if (stop) I.phase = 'idle';
    }
  }
  imuWrite(reg, val) { this.regs.set(0x1D000000 + reg, val); }
  imuRead(reg, snap) {
    if (reg === 0x0F) return 0x49; // WHO_AM_I of the LSM303D
    if (reg >= 0x28 && reg <= 0x2D) { const s = snap ?? this.imuSnapshot(); return s[reg - 0x28]; }
    return (this.regs.get(0x1D000000 + reg) ?? 0) & 0xFF;
  }
  imuSnapshot() { // accelerometer, +-4 g (8192 LSB/g), X forward, Y sideways, Z up
    const R = this.robot, g = 1, n = () => gauss() * 0.004;
    const ax = Math.sin(R.pitch) * g + R.aFwd / 9810 + n();
    const ay = Math.sin(R.roll) * g + n();
    const az = Math.cos(R.pitch) * Math.cos(R.roll) * g + ROBOT.imuZOffsetG + n();
    const out = new Uint8Array(6);
    [ax, ay, az].forEach((v, i) => { const c = Math.max(-32768, Math.min(32767, Math.round(v * 8192))); out[i * 2] = c & 0xFF; out[i * 2 + 1] = (c >> 8) & 0xFF; });
    return out;
  }

  // ---------------------------------------------------------------- ADC (barcode sensor, GP28 = AIN2)
  adcRead(o) {
    const A = this.adc;
    if (o === 0x00) return 0x100 | (A.many ? 0x8 : 0) | 1;        // READY
    if (o === 0x08) return (A.fifo.length ? 0 : 0x100) | (A.fifo.length << 16); // EMPTY, LEVEL
    if (o === 0x0C) return A.fifo.length ? A.fifo.shift() : 0;
    return (this.regs.get(ADC_BASE + o) ?? 0) >>> 0;
  }
  adcWrite(o, v) {
    const A = this.adc;
    if (o === 0x00) { A.on = !!(v & 1); A.many = !!(v & 8); return; }
    if (o === 0x18) { A.inte = v; return; }
    this.regs.set(ADC_BASE + o, v);
  }

  // ---------------------------------------------------------------- sonar
  startPing(tFall, widthUs) {
    if (widthUs < 5) return; // too short a trigger pulse
    const R = this.robot, az = R.th - this.servo * Math.PI / 180;
    const sx = R.x + Math.cos(R.th) * ROBOT.sonarAhead, sy = R.y + Math.sin(R.th) * ROBOT.sonarAhead;
    let d = this.world.sonar(sx, sy, az);
    d = isFinite(d) ? d + gauss() * 1.0 : Infinity;
    const echo = isFinite(d) && d > 30 ? d * 2 / 0.343 : 38000;  // us; no echo = 38 ms
    this.sonarMm = isFinite(d) ? d : null;
    const rise = tFall + 450;
    this.events.push({ t: rise, pin: 7, v: 1 }, { t: rise + echo, pin: 7, v: 0 });
    this.events.sort((a, b) => a.t - b.t);
  }

  // ---------------------------------------------------------------- environment inputs
  sensorPoint(ahead, lateral) { // lateral > 0 = right of the robot
    const R = this.robot, c = Math.cos(R.th), s = Math.sin(R.th);
    return [R.x + ahead * c - lateral * s, R.y + ahead * s + lateral * c];
  }

  updateSensors() {
    const W = this.world, win = ROBOT.tapeSensorWindow / 2;
    const [lx, ly] = this.sensorPoint(ROBOT.lineAhead, -ROBOT.lineGap / 2);
    const [rx, ry] = this.sensorPoint(ROBOT.lineAhead, ROBOT.lineGap / 2);
    this.lineL = W.tapeDist(lx, ly) < win ? 1 : 0;
    this.lineR = W.tapeDist(rx, ry) < win ? 1 : 0;
    this.setPin(1, this.lineL); // PIN_IR_LINE_L
    this.setPin(0, this.lineR); // PIN_IR_LINE_R
    const [bx, by] = this.sensorPoint(ROBOT.barAhead, -ROBOT.barLeft);
    const cover = W.inkCover(bx, by);
    this.analogB = Math.max(0, Math.min(4095, 300 + (4040 - 300) * cover + gauss() * 12));
    this.setPin(2, cover > 0.5 ? 1 : 0);
    this.sensorPts = { l: [lx, ly], r: [rx, ry], b: [bx, by] };
  }

  // ---------------------------------------------------------------- one 0.5 ms step
  step() {
    const R = this.robot, dt = DT_US / 1e6, t0 = this.nowUs, t1 = t0 + DT_US;
    this.stepCount++;
    this.maybeRelease();

    // motors: duty from the PWM registers (slice 4 = left, 5 = right)
    const duty = s => { const top = s.top + 1; return (s.a - s.b) / top; };
    const dL = duty(this.pwm[4]), dR = duty(this.pwm[5]);
    const tL = motorSpeed(dL), tR = motorSpeed(dR);
    R.vL += (tL - R.vL) * dt / ROBOT.motorTau;
    R.vR += (tR - R.vR) * dt / ROBOT.motorTau;
    const v = (R.vL + R.vR) / 2, w = (R.vL - R.vR) / ROBOT.track; // + = turn right on screen

    const nth = R.th + w * dt, mid = R.th + w * dt / 2;
    const nx = R.x + Math.cos(mid) * v * dt, ny = R.y + Math.sin(mid) * v * dt;
    R.collided = false;
    if (this.world.hitsBox(nx + Math.cos(nth) * (ROBOT.length / 2 - ROBOT.rear), ny + Math.sin(nth) * (ROBOT.length / 2 - ROBOT.rear), nth, ROBOT.length / 2, ROBOT.width / 2)) {
      R.collided = true; R.vL = 0; R.vR = 0;           // blocked by the box
    } else { R.x = nx; R.y = ny; R.th = nth; }
    R.aFwd = (v - R.vPrev) / dt; R.vPrev = v;
    this.dist = (this.dist || 0) + Math.abs(v) * dt;

    // terrain: pitch and roll from the three contact points
    const W = this.world, [lx, ly] = this.sensorPoint(0, -ROBOT.track / 2), [rx, ry] = this.sensorPoint(0, ROBOT.track / 2);
    const [fx, fy] = this.sensorPoint(ROBOT.casterAhead, 0);
    const hL = W.groundHeight(lx, ly), hR = W.groundHeight(rx, ry), hF = W.groundHeight(fx, fy);
    R.pitch = Math.atan2(hF - (hL + hR) / 2, ROBOT.casterAhead);
    R.roll = Math.asin(Math.max(-1, Math.min(1, (hR - hL) / ROBOT.track)));
    R.height = (hL + hR) / 2;

    // servo follows its commanded pulse (PWM slice 7, channel A), limited to 600 deg/s
    const pulse = this.pwm[7].a, target = Math.max(-60, Math.min(60, (pulse - 1500) / 11.1));
    const ds = target - this.servo, lim = 600 * dt;
    this.servo += Math.abs(ds) < lim ? ds : Math.sign(ds) * lim;

    // wheel encoders: 0.080 mm per count; the left one counts DOWN for forward
    const dx = [R.vL * dt, R.vR * dt], seq = [0, 1, 3, 2]; // A,B levels: 00 01 11 10
    const pins = [[4, 5], [16, 17]], dirs = [-1, +1];
    const edgeEvents = [];
    for (let k = 0; k < 2; k++) {
      this.enc[k].acc += dx[k] / ROBOT.wheelMmPerCount;
      const n = Math.trunc(this.enc[k].acc);
      this.enc[k].acc -= n;
      const stepDir = Math.sign(n) * dirs[k];
      for (let i = 1; i <= Math.abs(n); i++) {
        edgeEvents.push({ t: t0 + (i / Math.abs(n)) * DT_US - 0.001 * k, wheel: k, step: stepDir, pins: pins[k], seq });
      }
    }
    this.updateSensors();

    // timed events in order: encoder edges and sonar echo
    const evs = edgeEvents.concat(this.events.filter(e => e.t <= t1)).sort((a, b) => a.t - b.t);
    this.events = this.events.filter(e => e.t > t1);
    for (const e of evs) {
      this.nowUs = e.t; this.extra = 0;
      if (e.pins) {
        const E = this.enc[e.wheel]; E.ph = (E.ph + e.step + 4) & 3;
        const ab = e.seq[E.ph];
        this.setPin(e.pins[0], (ab >> 1) & 1); this.setPin(e.pins[1], ab & 1);
      } else this.setPin(e.pin, e.v);
      this.deliverIrqs();
    }
    this.nowUs = t1; this.extra = 0;

    // ADC: one sample per step (2 kHz) while "start many" is on
    if (this.adc.on && this.adc.many) {
      this.adc.fifo.push(Math.round(this.analogB)); if (this.adc.fifo.length > 8) this.adc.fifo.shift();
      if (this.adc.inte & 1) this.fw.kernel_irq(INTNO_ADC);
    }
    this.deliverIrqs();

    if (t1 % TICK_US === 0) this.fw.kernel_tick();
    this.schedule();
  }

  // scheduler: run ready tasks, highest priority first, until all are blocked
  schedule() {
    const F = this.fw;
    for (let guard = 0; guard < 2000; guard++) {
      const idx = F.kernel_pick(); if (!idx) break;
      this.sp.value = F.task_get_sp(idx);
      if (F.task_suspended(idx)) F.asyncify_start_rewind(F.task_astack(idx));
      F.task_tramp(idx);
      if (F.asyncify_get_state() === 1) F.asyncify_stop_unwind();
      F.task_set_sp(idx, this.sp.value);
      this.sp.value = this.mainSP;
      F.kernel_after();
    }
  }

  run(seconds) { const n = Math.round(seconds * 1e6 / DT_US); for (let i = 0; i < n; i++) this.step(); }

  // ---------------------------------------------------------------- controls
  place(x, y, th) { Object.assign(this.robot, { x, y, th, vL: 0, vR: 0, vPrev: 0, aFwd: 0 }); this.updateSensors(); }

  press(pin, ms = 200) { this.setPin(pin, 0); this.deliverIrqs(); this.schedule(); this.releaseAt = { pin, t: this.nowUs + ms * 1000 }; }
  maybeRelease() { if (this.releaseAt && this.nowUs >= this.releaseAt.t) { this.setPin(this.releaseAt.pin, 1); this.releaseAt = null; } }

  mqtt(topic, payload) { // what the WiFi glue does: hand a message to the firmware
    const enc = new TextEncoder(), tb = enc.encode(topic + '\0'), pb = typeof payload === 'string' ? enc.encode(payload) : payload;
    const m = new Uint8Array(this.mem.buffer);
    m.set(tb, this.scratch); m.set(pb, this.scratch + 256);
    this.fw.app_mqtt_rx(this.scratch, this.scratch + 256, pb.length);
    this.schedule();
  }
  drainMqtt() { // the WiFi task polls the outbox every 100 ms; keep the link alive and collect messages
    const out = [], m = new Uint8Array(this.mem.buffer);
    for (let i = 0; i < 8; i++) {
      const n = this.fw.app_mqtt_tx(this.scratch + 512, 64, this.scratch + 640, 128);
      if (n <= 0) break;
      const topic = new TextDecoder().decode(m.slice(this.scratch + 512, this.scratch + 512 + 64)).split('\0')[0];
      out.push({ topic, data: m.slice(this.scratch + 640, this.scratch + 640 + n) });
    }
    return out;
  }
  sendMapping(text) { this.mqtt('is34/cmd/map', text); }
  start() { this.mqtt('is34/cmd/start', ''); }
  stop() { this.mqtt('is34/cmd/stop', ''); }
  pressStartButton() { this.press(20, 200); }
  pressStopButton() { this.press(21, 200); }
}

export { World, TILE, TAPE };

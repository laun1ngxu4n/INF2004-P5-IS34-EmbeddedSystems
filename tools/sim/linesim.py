"""Line-following simulator for the IS34 robot (straddle sensors).

Robot: differential drive, track 123 mm, line sensors 115 mm ahead of the
axle at +-15 mm.  Motion task every 10 ms (speed ramp 400 mm/s^2, wheel
speed follows its target with a first-order lag).  Line task every 20 ms:
a port of app_program/subsystem3/task_line.c, with its settings in a dict
so variants can be compared (BASE = the 9d10fa6 settings, NEW = the ones
in task_line.c since 7 Nov).  Keep both in step with task_line.c.

Calibrated on 7 Nov: with curves about 120 mm in radius, about 50 ms of
extra delay (40 ms speed averaging, motor response, caster) and a 20 mm
sensor window it reproduced that day's three robot runs (150 mm/s lost
the left curves, hard-at-once zigzagged, 9d10fa6 worked).  The sensor
window after the mount was raised is narrower (about 8-12 mm).

    python tools/sim/linesim.py          # old vs new on the calibrated scenarios

No packages needed (plain Python 3).

Sensor: black when the sensor point is within w/2 of the tape centreline
(w = effective detection window: 20 mm = the whole tape; narrower = a
sensor that only sees the middle of the tape, as after the mount was
raised on 7 Nov)."""
import math

TRACK = 123.0
AHEAD = 115.0
HALF_GAP = 15.0
DT = 0.010


# ---------------------------------------------------------------- tracks
class Path:
    """Straights and arcs; exact distance from a point to the tape centre"""
    def __init__(self, segs):
        self.segs = []          # (kind, x0, y0, h0, length, r, d, s0)
        x, y, h, s0 = 0.0, 0.0, 0.0, 0.0
        for sg in segs:
            if sg[0] == 'S':
                L = sg[1]
                self.segs.append(('S', x, y, h, L, 0, 0, s0))
                x += L * math.cos(h); y += L * math.sin(h)
            else:
                _, r, deg, d = sg
                L = r * math.radians(deg)
                self.segs.append(('A', x, y, h, L, r, d, s0))
                cx = x - d * r * math.sin(h); cy = y + d * r * math.cos(h)
                h2 = h + d * math.radians(deg)
                x = cx + d * r * math.sin(h2); y = cy - d * r * math.cos(h2)
                h = h2
            s0 += L
        self.total = s0
        self.start = (0.0, 0.0, 0.0)

    def near(self, px, py):
        """(distance, arc position s) of the nearest tape point"""
        best = (1e9, 0.0)
        for kind, x, y, h, L, r, d, s0 in self.segs:
            if kind == 'S':
                c, s_ = math.cos(h), math.sin(h)
                t = (px - x) * c + (py - y) * s_
                t = min(max(t, 0.0), L)
                qx, qy = x + t * c, y + t * s_
                dist = math.hypot(px - qx, py - qy)
                if dist < best[0]:
                    best = (dist, s0 + t)
            else:
                cx = x - d * r * math.sin(h); cy = y + d * r * math.cos(h)
                ang0 = math.atan2(y - cy, x - cx)
                ang = math.atan2(py - cy, px - cx)
                sweep = (ang - ang0) * d
                sweep = (sweep + math.pi) % (2 * math.pi) - math.pi
                if sweep < 0 and L / r > math.pi:
                    sweep += 2 * math.pi
                if 0.0 <= sweep <= L / r:
                    dist = abs(math.hypot(px - cx, py - cy) - r)
                    sp = s0 + sweep * r
                else:
                    ex = cx + r * math.cos(ang0 + d * L / r)
                    ey = cy + r * math.sin(ang0 + d * L / r)
                    d1 = math.hypot(px - x, py - y); d2 = math.hypot(px - ex, py - ey)
                    dist, sp = (d1, s0) if d1 < d2 else (d2, s0 + L)
                if dist < best[0]:
                    best = (dist, sp)
        return best


def tracks(r=150.0):
    return {
        'straight': Path([('S', 1500)]),
        'left90': Path([('S', 300), ('A', r, 90, +1), ('S', 400)]),
        'left180': Path([('S', 300), ('A', r, 180, +1), ('S', 400)]),
        'right180': Path([('S', 300), ('A', r, 180, -1), ('S', 400)]),
        # the photo: up-left 90, over the top right 180, down-left 90
        'sbend': Path([('S', 300), ('A', r, 90, +1), ('A', r, 180, -1),
                       ('A', r, 90, +1), ('S', 400)]),
        'sbend_r': Path([('S', 300), ('A', r, 90, -1), ('A', r, 180, +1),
                         ('A', r, 90, -1), ('S', 400)]),
    }


# ------------------------------------------------------------- line task
BASE = dict(GENTLE=25, HARD=40, STEP=10, GENTLE_MM=15, MID=60, LOW=35,
            LOW_AFTER=35, SEARCH_PCT=20, FILTER=2, LONG_CORR=15,
            OVERSHOOT=40, LOST_STRAIGHT=400, LOST_MM=250, LOST_MS=4000,
            SPEED=120, PROBE=False, PROBE_AFTER=200, PROBE_MM=70,
            PROBE_STEER=12, SEARCH_STEER=None,
            MEM_D=0, MEM_K=1.0, MEM_MAX=40, MEM_SLOW=0, SEARCH_ON=True)


class LineTask:
    def __init__(self, p):
        self.p = p
        self.stable = self.cand = 0
        self.cand_n = p['FILTER']
        self.out = 0
        self.last_side = 0
        self.single_from = self.white_from = self.both_from = 0.0
        self.long_corr = self.searching = False
        self.search_from = 0.0
        self.search_n = 0
        self.lost = False
        self.steer = 0
        self.pct = 100
        self.n_corr = self.n_sweep = 0
        self.before_both = 0
        self.probe = 0; self.probe_leg = 0; self.probe_from = 0.0
        self.probe_none = False
        self.mem = 0.0
        self.last_odo = 0.0

    def step(self, l, r, odo):
        p = self.p
        pat = (2 if l else 0) | (1 if r else 0)
        prev = self.stable
        if pat == self.cand:
            if self.cand_n < p['FILTER']:
                self.cand_n += 1
        else:
            self.cand = pat
            self.cand_n = 1
        if self.cand_n >= p['FILTER']:
            self.stable = self.cand
        fl = bool(self.stable & 2); fr = bool(self.stable & 1)
        single = fl != fr
        if self.stable != prev:
            if prev in (1, 2):
                self.long_corr = odo - self.single_from >= p['LONG_CORR']
            if single:
                self.single_from = odo
                if prev == 0:
                    self.n_corr += 1
                if prev == 3 and self.before_both and self.before_both != self.stable:
                    self.n_sweep += 1
            if self.stable == 0:
                self.white_from = odo
            if self.stable == 3:
                self.both_from = odo
                self.before_both = prev if prev in (1, 2) else 0
        if self.stable != 0 and self.searching:
            self.searching = False
            self.long_corr = False
        if single:
            self.last_side = 1 if fl else -1
        if (p['SEARCH_ON'] and not self.searching and self.stable == 0 and self.long_corr and
                odo - self.white_from >= p['OVERSHOOT']):
            self.searching = True
            self.search_from = odo
            self.search_n = 0
        # probe (optional)
        if p['PROBE']:
            if self.stable != 0 or self.searching:
                self.probe = self.probe_leg = 0; self.probe_none = False
            elif self.probe == 0 and not self.probe_none and odo - self.white_from >= p['PROBE_AFTER']:
                self.probe = 1 if self.last_side >= 0 else -1
                self.probe_leg = 1; self.probe_from = odo
            elif self.probe and odo - self.probe_from >= p['PROBE_MM'] * self.probe_leg:
                if self.probe_leg == 1:
                    self.probe = -self.probe; self.probe_leg = 2; self.probe_from = odo
                else:
                    self.probe = self.probe_leg = 0; self.probe_none = True
        # steering
        if single:
            s = p['GENTLE'] if odo - self.single_from < p['GENTLE_MM'] else p['HARD']
            if fr:
                s = -s
        elif self.searching:
            ss = p['SEARCH_STEER'] or p['HARD']
            s = ss if self.last_side >= 0 else -ss
        elif self.probe:
            s = self.probe * p['PROBE_STEER']
        elif p['MEM_D']:
            s = max(-p['MEM_MAX'], min(p['MEM_MAX'], p['MEM_K'] * self.mem))
        else:
            s = 0
        t = s
        o = self.out
        if (o > 0 and t < 0) or (o < 0 and t > 0):
            o = 0
        if (o >= 0 and 0 <= t <= o) or (o <= 0 and o <= t <= 0):
            o = t
        elif o < t - p['STEP']:
            o += p['STEP']
        elif o > t + p['STEP']:
            o -= p['STEP']
        else:
            o = t
        self.out = o
        self.steer = o
        if p['MEM_D']:
            ds = max(0.0, odo - self.last_odo)
            a = min(1.0, ds / p['MEM_D'])
            self.mem += (o - self.mem) * a
        self.last_odo = odo
        if self.searching:
            self.pct = p['SEARCH_PCT']
        elif single and odo - self.single_from >= p['LOW_AFTER']:
            self.pct = p['LOW']
        elif single and odo - self.single_from >= p['GENTLE_MM']:
            self.pct = p['MID']
        else:
            self.pct = 100
        if p['MEM_SLOW'] and not self.searching:
            self.pct = min(self.pct, max(35, int(100 - p['MEM_SLOW'] * abs(self.mem))))
        # lost
        if self.searching:
            self.search_n += 1
            if odo - self.search_from > p['LOST_MM'] or self.search_n > p['LOST_MS'] / 20:
                self.lost = True
        elif p['PROBE']:
            if self.probe_none:
                self.lost = True
        elif self.stable == 0 and odo - self.white_from > p['LOST_STRAIGHT']:
            self.lost = True


# ------------------------------------------------------------- the robot
def run(path, p, w=20.0, tau=0.08, start_off=0.0, start_head=0.0, tmax=30.0,
        noise=0.0, rng=None, trace=False, bias=0.0, wl_extra=0.0, delay=0.0):
    """Returns dict: ok, why, dist, max_dev, corr, sweeps"""
    # start: sensors on the tape start, heading along it (+ offsets)
    h = math.radians(start_head)
    x = -AHEAD * math.cos(h) - start_off * math.sin(h)
    y = -AHEAD * math.sin(h) + start_off * math.cos(h)
    vl = vr = 0.0
    v_prof = 0.0
    odo = 0.0
    lt = LineTask(p)
    max_dev = 0.0
    k = 0
    log = []
    nd = int(round(delay / DT))
    q = [(0.0, 0.0)] * (nd + 1)
    while k * DT < tmax:
        c, s_ = math.cos(h), math.sin(h)
        fx, fy = x + AHEAD * c, y + AHEAD * s_
        dev, sp = path.near(fx, fy)
        max_dev = max(max_dev, dev)
        if sp >= path.total - 3:
            return dict(ok=True, why='end', dist=odo, max_dev=max_dev,
                        corr=lt.n_corr, sweeps=lt.n_sweep, t=k * DT, log=log)
        if dev > 90:
            return dict(ok=False, why='off', dist=odo, max_dev=max_dev,
                        corr=lt.n_corr, sweeps=lt.n_sweep, t=k * DT, log=log)
        if k % 2 == 0:      # line task, 20 ms
            lx, ly = fx - HALF_GAP * s_, fy + HALF_GAP * c
            rx, ry = fx + HALF_GAP * s_, fy - HALF_GAP * c
            wl = w + (rng.gauss(0, noise) if rng is not None and noise else 0)
            wr = w + (rng.gauss(0, noise) if rng is not None and noise else 0)
            Lb = path.near(lx, ly)[0] <= (wl + wl_extra) / 2
            Rb = path.near(rx, ry)[0] <= wr / 2
            lt.step(Lb, Rb, odo)
            if trace:
                log.append((round(odo), int(Lb), int(Rb), lt.steer, lt.pct, round(dev), round(sp)))
            if lt.lost:
                return dict(ok=False, why='lost', dist=odo, max_dev=max_dev,
                            corr=lt.n_corr, sweeps=lt.n_sweep, t=k * DT, log=log)
        # motion task, 10 ms
        cap = p['SPEED']
        pct = lt.pct
        off = pct < 100
        if off:
            cap = cap * pct / 100.0
        v_prof += 400.0 * DT
        if v_prof > cap:
            v_prof = cap
        tl = v_prof - lt.steer
        tr = v_prof + lt.steer
        if not off:
            tl = max(tl, 0.0); tr = max(tr, 0.0)
        q.append((tl, tr))
        tl, tr = q.pop(0)
        a = DT / tau
        vl += (tl * (1 - bias) - vl) * a
        vr += (tr * (1 + bias) - vr) * a
        v = (vl + vr) / 2
        om = (vr - vl) / TRACK
        x += v * math.cos(h) * DT
        y += v * math.sin(h) * DT
        h += om * DT
        odo += v * DT
        k += 1
    return dict(ok=False, why='time', dist=odo, max_dev=max_dev,
                corr=lt.n_corr, sweeps=lt.n_sweep, t=k * DT, log=log)


def score(p, ws=(20, 14, 10, 8), r=150.0, names=None, offs=(-6, 0, 6), heads=(-5, 0, 5), quiet=False, **kw):
    T = tracks(r)
    names = names or list(T)
    res = {}
    for w in ws:
        for nm in names:
            ok = tot = sw = 0
            for o in offs:
                for hd in heads:
                    rr = run(T[nm], p, w=w, start_off=o, start_head=hd, **kw)
                    tot += 1; ok += rr['ok']; sw += rr['sweeps']
            res[(w, nm)] = (ok, tot, sw)
    if not quiet:
        print('w   ' + ' '.join(f'{n:>10}' for n in names))
        for w in ws:
            print(f'{w:<3} ' + ' '.join(f'{res[(w, n)][0]:>4}/{res[(w, n)][1]} s{res[(w, n)][2]:<2}' for n in names))
    return res


NEW = dict(GENTLE=15, HARD=30, STEP=10, GENTLE_MM=0, MID=60, LOW=40, LOW_AFTER=50,
           SEARCH_PCT=20, FILTER=1, LONG_CORR=5, OVERSHOOT=60, SPEED=120,
           MEM_D=60, MEM_K=0.4, MEM_SLOW=0.5, SEARCH_STEER=30, PROBE=True)

# calibrated scenarios (the ones the 7 Nov settings were chosen on)
def scenarios():
    sc = []
    for R in (120, 100):
        T = tracks(R)
        for nm in ('straight', 'left180', 'right180', 'sbend', 'sbend_r'):
            for w in (20, 12, 8):
                for tau in (0.1, 0.2):
                    for d in (0.04, 0.07):
                        for b in (0.02, -0.02):
                            for hd in (-5, 5):
                                sc.append((T[nm], nm, w, tau, d, b, hd))
    return sc


def evaluate(p, sc):
    ok = sw = 0
    fails = {}
    for path, nm, w, tau, d, b, hd in sc:
        r = run(path, p, w=w, tau=tau, delay=d, bias=b, start_head=hd)
        ok += r['ok']; sw += r['sweeps']
        if not r['ok']:
            fails[(nm, w)] = fails.get((nm, w), 0) + 1
    return ok, sw, fails


if __name__ == '__main__':
    sc = scenarios()
    for tag, v in (('9d10fa6', dict(PROBE=True)), ('new (7 Nov)', NEW)):
        p = dict(BASE); p.update(v)
        ok, sw, fails = evaluate(p, sc)
        print(f'{tag:12s} pass {ok}/{len(sc)}  sweeps {sw}  fails by (track, window): {dict(sorted(fails.items()))}')

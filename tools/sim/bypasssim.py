"""Obstacle bypass simulator for the IS34 robot.

The robot follows the line to a box sitting on it, stops at the forward
watch, scans (width with realistic errors), drives round, finds the line
again and follows it on.  The line behind the box can be straight or
curve either way.  Each run is checked for: touching the box (robot
footprint vs box), finding the line, following it 600 mm on.

Two rejoin methods:
  OLD  (mission.c until 8 Nov): fixed rectangle; on the way back
       black only counts within LINE_NEAR_MM of where the line was;
       drive 115 mm, turn 90 deg, follow.
  NEW  (mission.c since 8 Nov) watch the line sensors on the leg
       past the box too; on the way back any black counts; drive 115 mm
       (wheels on the line); unless a sensor already sees the tape, spin
       toward the old direction until one does; follow.  Margin 100 mm.

  J    (since 8 Nov, after a box standing on a junction): as NEW, but
       leg 4 watches only once the sensors are past the back of a 150 mm
       deep box (a junction's side line under the box is not the line),
       and the first scan is where the forward watch stopped.
  S    (mission.c since 8 Nov): as J, but on leg 4 the sonar looks
       40 deg toward the box (the cables' limit) and listens for the box's
       side face at its expected sideways distance.  Each echo pushes the
       turning point to at least half the robot's width + MARGIN past
       where it was heard; never earlier than J's.  A deep box no longer
       gets hit (J assumes at most 150 mm).

    python tools/sim/bypasssim.py S      (or any of OLD NEW J S)

Results S alone (8 Nov): 471/486 ordinary cases (as J: the side look
never shortens the leg), junctions under the box 54/54 each, box depth
60 / 150 / 250 / 350 mm: 36/36 each (closest 73 / 74 / 69 / 56 mm).  J on
the same depths: 36 / 36 / 20 (16 hit the box) / 0 (all hit).  Side
echoes missed 50 % or 80 % of the time: 53/54 deep-box runs; never heard
at all: as J.  (Before: a 90 deg look, impossible with the cables; then
"still beside it at x_pass?", which turned a 350 mm deep box's body into
it because the sonar is 120 mm ahead of the axle.)

Results (8 Nov, tape 1600 mm on past the curve): OLD 36/486 (straight
24/54: the narrow window misses the line once spins are a few degrees
off), NEW 476/486, J 471/486 (a few 300 mm curves right behind the box
met too late); nothing touched the box (closest 44 mm).  Boxes on a
junction (side line under the box: T left, T right, X), 54 runs each:
NEW 20 / 45 / 5 (followed the side line), J 54 / 54 / 54.  A side line
200 mm behind the box: 0/54 for all (it looks like the line turning).

Robot model as in tools/sim/linesim.py (track 123 mm, line sensors 115 mm
ahead at +-15 mm, wheel lag and delay, wheel mismatch).  Spins and
drives use the encoders: a spin ends 0-6 % off, wheel size mismatch
bends straight drives.

    python tools/sim/bypasssim.py
"""
import math
import random
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from linesim import Path, LineTask, BASE, NEW, TRACK, AHEAD, HALF_GAP  # noqa: E402

DT = 0.01
SONAR_AHEAD = 120.0          # sonar in front of the axle
TRIP_MM = 250
SCAN_FROM_MM = 350
BEAM_DEG = 15                # sonar beam half width
SIDE_RANGE = 1000.0
SIDE_DEG = 40                # S: the side look (the cables allow 40, not 90)
SIDE_TOL_IN_MM = 120         # S: echo from lat_exp - this ...
SIDE_TOL_OUT_MM = 80         # ... to lat_exp + this = the box beside
SIDE_MISS = 5                # S: pings in a row without it = past
SIDE_DROP = 0.2              # model: square-on side echoes missed
SIDE_STRAY = 0.5             # model: glancing echoes from the beam's axis
SIDE_EXTRA_MM = 300          # S: drive on at most this past x_pass while beside it
SCAN_NEAR_MM = 220           # J: first scan where it stopped, at least this far
REVERSE_MM = 100
D_MAX_MM = 120
MARGIN_MM = 80               # mission.c (70 until 8 Nov, 100 until 23 Nov)
SKIP_IF_ON = True            # on the tape after the 115 mm: no spin
SPIN_BACK = False            # spin back the other way if nothing found
                             # (True: 2 runs found the line pointing back
                             # at the box and drove into it)
ROBOT_W = 170
ROBOT_L = 200
MAX_W = 200
LINE_NEAR_MM = 100
SPEED = 120.0
BODY = (-60.0, 135.0, -65.0, 82.0)   # x rear, x front, y right, y left (B arm left)


class World:
    def __init__(self, path, box_x, box_w, box_d, box_yc, window, branches=()):
        self.path = path
        self.bx0, self.bx1 = box_x, box_x + box_d
        self.by0, self.by1 = box_yc - box_w / 2, box_yc + box_w / 2
        self.window = window
        self.branches = branches             # side lines: (x, y0, y1), across the track

    def black(self, px, py):
        if self.bx0 <= px <= self.bx1 and self.by0 <= py <= self.by1:
            return False                     # under the box: no tape seen
        for bx, y0, y1 in self.branches:
            if abs(px - bx) <= self.window / 2 and min(y0, y1) <= py <= max(y0, y1):
                return True
        return self.path.near(px, py)[0] <= self.window / 2


class Robot:
    def __init__(self, world, rng, mismatch, turn_err):
        self.w = world
        self.rng = rng
        self.x, self.y, self.h = 0.0, 0.0, 0.0
        self.mis = mismatch          # right wheel bigger by this fraction
        self.turn_err = turn_err
        self.hit = False
        self.min_clear = 1e9

    # geometry
    def sensors(self):
        c, s = math.cos(self.h), math.sin(self.h)
        fx, fy = self.x + AHEAD * c, self.y + AHEAD * s
        return ((fx - HALF_GAP * s, fy + HALF_GAP * c), (fx + HALF_GAP * s, fy - HALF_GAP * c))

    def black_lr(self):
        (lx, ly), (rx, ry) = self.sensors()
        return self.w.black(lx, ly), self.w.black(rx, ry)

    def sonar(self):
        """distance from the sonar to the box face straight ahead, or None"""
        c, s = math.cos(self.h), math.sin(self.h)
        ox, oy = self.x + SONAR_AHEAD * c, self.y + SONAR_AHEAD * s
        best = None
        for t in range(0, 1500, 5):
            px, py = ox + t * c, oy + t * s
            if self.w.bx0 <= px <= self.w.bx1 and self.w.by0 - 20 <= py <= self.w.by1 + 20:
                best = t
                break
        return best

    def ray_box(self, ang):
        """distance from the sonar along ang to the box, or None"""
        ox = self.x + SONAR_AHEAD * math.cos(self.h)
        oy = self.y + SONAR_AHEAD * math.sin(self.h)
        dx, dy = math.cos(ang), math.sin(ang)
        t0, t1 = 0.0, SIDE_RANGE
        for o, dd, lo, hi in ((ox, dx, self.w.bx0, self.w.bx1), (oy, dy, self.w.by0, self.w.by1)):
            if abs(dd) < 1e-9:
                if not lo <= o <= hi:
                    return None
                continue
            ta, tb = (lo - o) / dd, (hi - o) / dd
            if ta > tb:
                ta, tb = tb, ta
            t0, t1 = max(t0, ta), min(t1, tb)
        return t0 if t0 <= t1 else None

    def side_sonar(self, side):
        """sonar turned SIDE_DEG (40, the cables' limit) toward one side
        (+1 left).  The beam's axis meets the box's side face at a glancing
        angle, so a flat face mostly reflects it away; but the side of the
        beam hears the face at its square-on distance while the sonar is
        beside it (7 Nov scans: 40 deg off a face still read its square-on
        distance).  Modelled: that square-on echo while the perpendicular
        from the sonar meets the box, missing SIDE_DROP of the pings; else a
        stray glancing echo from the beam's axis (SIDE_STRAY of the pings,
        when the axis meets the box); else nothing."""
        d = self.ray_box(self.h + side * math.pi / 2)
        if d is not None and self.rng.random() >= SIDE_DROP:
            return d
        d = self.ray_box(self.h + side * math.radians(SIDE_DEG))
        if d is not None and self.rng.random() < SIDE_STRAY:
            return d
        return None

    def check_hit(self):
        c, s = math.cos(self.h), math.sin(self.h)
        xr, xf, yr, yl = BODY
        corners = [(xr, yr), (xr, yl), (xf, yr), (xf, yl), (xf, 0), (xr, 0), (0, yl), (0, yr)]
        for bx, by in corners:
            wx = self.x + bx * c - by * s
            wy = self.y + bx * s + by * c
            dx = max(self.w.bx0 - wx, 0, wx - self.w.bx1)
            dy = max(self.w.by0 - wy, 0, wy - self.w.by1)
            d = math.hypot(dx, dy)
            self.min_clear = min(self.min_clear, d)
            if d == 0:
                self.hit = True
        # box corners inside the robot
        for wx, wy in ((self.w.bx0, self.w.by0), (self.w.bx0, self.w.by1),
                       (self.w.bx1, self.w.by0), (self.w.bx1, self.w.by1)):
            rx = (wx - self.x) * c + (wy - self.y) * s
            ry = -(wx - self.x) * s + (wy - self.y) * c
            if xr <= rx <= xf and yr <= ry <= yl:
                self.hit = True
                self.min_clear = 0

    def step(self, vl, vr):
        vl *= (1 - self.mis / 2)
        vr *= (1 + self.mis / 2)
        v = (vl + vr) / 2
        self.x += v * math.cos(self.h) * DT
        self.y += v * math.sin(self.h) * DT
        self.h += (vr - vl) / TRACK * DT
        self.check_hit()
        return v

    # motion commands (encoder based)
    def drive(self, dist, speed=SPEED, stop_on=None):
        """straight; stop_on(robot, done_mm) -> True halts. Returns mm done, halted"""
        sgn = 1 if dist >= 0 else -1
        done = 0.0
        v = 0.0
        while done < abs(dist):
            remain = abs(dist) - done
            v = min(v + 400 * DT, speed, math.sqrt(2 * 400 * remain) + 5)
            done += abs(self.step(sgn * v, sgn * v)) * DT
            if stop_on and stop_on(self, done):
                # brake
                while v > 0:
                    v = max(0.0, v - 400 * DT)
                    done += abs(self.step(sgn * v, sgn * v)) * DT
                return done, True
        return done, False

    def turn(self, deg, stop_on=None, w=60.0):
        """spin; stop_on(robot, turned_deg) -> True halts"""
        target = math.radians(abs(deg)) * (1 + self.turn_err)
        sgn = 1 if deg >= 0 else -1
        turned = 0.0
        v = 0.0
        while turned < target:
            remain = (target - turned) * TRACK / 2
            v = min(v + 400 * DT, w, math.sqrt(2 * 400 * remain) + 5)
            h0 = self.h
            self.step(-sgn * v, sgn * v)
            turned += abs(self.h - h0)
            if stop_on and stop_on(self, math.degrees(turned)):
                while v > 0:
                    v = max(0.0, v - 400 * DT)
                    self.step(-sgn * v, sgn * v)
                return math.degrees(turned), True
        return math.degrees(turned), False

    def follow(self, p, mm, tau=0.15, delay=0.05, trip=False):
        """line following (linesim's line task + motion); returns why it ended"""
        lt = LineTask(p)
        vl = vr = 0.0
        v_prof = 0.0
        odo = 0.0
        q = [(0.0, 0.0)] * (int(delay / DT) + 1)
        k = 0
        close = 0
        while odo < mm and k < 6000:
            if k % 2 == 0:
                L, R = self.black_lr()
                lt.step(L, R, odo)
                if lt.lost:
                    return 'lost'
            if trip and k % 6 == 0:
                d = self.sonar()
                close = close + 1 if (d is not None and d < TRIP_MM) else 0
                if close >= 2:
                    # stop
                    for _ in range(30):
                        vl *= 0.8; vr *= 0.8
                        self.step(vl, vr)
                    return 'obstacle'
            cap = p['SPEED']
            off = lt.pct < 100
            if off:
                cap = cap * lt.pct / 100
            v_prof = min(v_prof + 400 * DT, cap)
            tl, tr = v_prof - lt.steer, v_prof + lt.steer
            if not off:
                tl, tr = max(tl, 0), max(tr, 0)
            q.append((tl, tr))
            tl, tr = q.pop(0)
            a = DT / tau
            vl += (tl - vl) * a
            vr += (tr - vr) * a
            odo += self.step(vl, vr) * DT
            if self.hit:
                return 'hit'
            k += 1
        return 'ok'


def bypass(rb, method, rng, scan_err):
    """the mission's on_obstacle, after the forward watch stopped it"""
    w = rb.w
    # NEW: back up to scan from SCAN_FROM_MM, then REVERSE_MM more.
    # J (8 Nov): scan where the forward watch stopped (at least
    # SCAN_NEAR_MM away), then REVERSE_MM back
    d = rb.sonar() or TRIP_MM
    first = SCAN_NEAR_MM if method in ('J', 'S') else SCAN_FROM_MM
    if first - d > 10:
        rb.drive(-(first - d))
    rb.drive(-REVERSE_MM)
    front = rb.sonar() or 450
    # measured edges (robot frame, + = left), with error
    c, s = math.cos(rb.h), math.sin(rb.h)
    def lat(wx, wy):
        return -(wx - rb.x) * s + (wy - rb.y) * c
    true_l = lat(w.bx0, w.by1)
    true_r = lat(w.bx0, w.by0)
    left = true_l + rng.gauss(0, scan_err)
    right = true_r + rng.gauss(0, scan_err)
    unc = 50
    # side: nearer edge (both clear here)
    sgn = 1 if left <= -right else -1
    w_plan = max((left - right) + unc, MAX_W)
    yc = (left + right) / 2
    edge_plan = yc + sgn * w_plan / 2
    edge_meas = (left if sgn > 0 else right) + sgn * unc
    if sgn * edge_meas > sgn * edge_plan:
        edge_plan = edge_meas
    y_off = sgn * edge_plan + ROBOT_W / 2 + MARGIN_MM
    x_pass = front + D_MAX_MM + ROBOT_L + MARGIN_MM

    rb.turn(sgn * 90)
    rb.drive(y_off)
    rb.turn(-sgn * 90)
    if method == 'S':
        edge = left if sgn > 0 else right
        return bypass_side(rb, sgn, front, y_off, x_pass, y_off - sgn * edge)
    if method == 'OLD':
        rb.drive(x_pass)
        rb.turn(-sgn * 90)
        def near(r, done):
            return done >= y_off - LINE_NEAR_MM and any(r.black_lr())
        done, found = rb.drive(y_off + 200, stop_on=near)
        if not found:
            return 'not found'
        rb.drive(AHEAD)
        rb.turn(sgn * 90)
        return 'rejoined'
    # NEW: leg 4 watches for the line once the sensors are past the box front
    # J: ... past the back of the deepest box (a side line of a junction
    # under the box is not the line: 8 Nov, the robot followed one)
    sensors_past = front + SONAR_AHEAD - AHEAD + 30
    if method == 'J':
        sensors_past += D_MAX_MM
    def any_after(r, done):
        return done >= sensors_past and any(r.black_lr())
    done, found = rb.drive(x_pass, stop_on=any_after)
    if not found:
        rb.turn(-sgn * 90)
        def any_black(r, done):
            return done >= 20 and any(r.black_lr())
        done, found = rb.drive(y_off + 400, stop_on=any_black)
        if not found:
            return 'not found'
    return rejoin(rb, sgn, y_off)


def rejoin(rb, sgn, y_off, found=True):
    """leg 7: wheels onto the line, spin toward the old direction"""
    # wheels onto the line, then spin toward the old direction until a
    # line sensor finds the tape (ignore the first 20 deg)
    rb.drive(AHEAD)
    if SKIP_IF_ON and any(rb.black_lr()):
        return 'rejoined'          # already on the tape: just follow
    def tape(r, turned):
        return turned >= 20 and any(r.black_lr())
    turned, ok = rb.turn(sgn * 170, stop_on=tape, w=40)
    if not ok and SPIN_BACK:
        turned, ok = rb.turn(-sgn * 340, stop_on=lambda r, t: t >= 190 and any(r.black_lr()), w=40)
    if not ok:
        return 'spin found nothing'
    return 'rejoined'


def bypass_side(rb, sgn, front, y_off, x_pass, lat_exp):
    """S (since 8 Nov): leg 4 with the sonar turned 40 deg toward the box.
    An echo from lat_exp - SIDE_TOL_IN_MM to lat_exp + SIDE_TOL_OUT_MM
    (lat_exp: the sideways distance to the box's side face, from the scan)
    counts as "box beside"; beside = one in the last SIDE_MISS pings.  The
    side look only makes the leg longer: it ends when the axle is half
    the robot's width + MARGIN past the last such echo, and never before
    the fixed turning point x_pass (a box up to 150 mm deep), so a stray
    echo only makes it longer, a missed one changes nothing.  (8 Nov: "is
    it still beside it at x_pass?" was wrong: the sonar, 120 mm ahead of
    the axle, had just passed a 350 mm deep box's back there, the body
    had not.)  Line sensors count from the fixed point, not while the box
    is beside (an echo in the last SIDE_MISS pings).  Still going at
    x_pass + SIDE_EXTRA_MM: stop (box too long)."""
    side = -sgn
    st = dict(n=0, seen=False, last=0.0, miss=SIDE_MISS, why=None)
    rb.side_state = st
    sensors_fixed = front + SONAR_AHEAD - AHEAD + 30 + D_MAX_MM

    def leg4(r, done):
        st['n'] += 1
        if st['n'] % 6 == 0:                     # a ping every 60 ms
            d = r.side_sonar(side)
            st.setdefault('pings', []).append((round(done), None if d is None else round(d)))
            if d is not None and lat_exp - SIDE_TOL_IN_MM <= d <= lat_exp + SIDE_TOL_OUT_MM:
                st['seen'], st['last'], st['miss'] = True, done, 0
            else:
                st['miss'] += 1
        beside = st['miss'] < SIDE_MISS
        if done >= sensors_fixed and not beside and any(r.black_lr()):
            st['why'] = 'line'
            return True
        # every echo of the box: the axle must get half the robot's width
        # + MARGIN past where the sonar heard it; never less far than J
        stop = x_pass
        if st['seen']:
            stop = max(stop, st['last'] + SONAR_AHEAD + ROBOT_W / 2 + MARGIN_MM)
        if done >= stop:
            st['why'] = 'past'
            return True
        return False

    done, halted = rb.drive(x_pass + SIDE_EXTRA_MM, stop_on=leg4)
    if not halted:
        return 'box too long'
    if st['why'] == 'past':
        rb.turn(-sgn * 90)

        def any_black(r, d):
            return d >= 20 and any(r.black_lr())
        done, found = rb.drive(y_off + 400, stop_on=any_black)
        if not found:
            return 'not found'
    return rejoin(rb, sgn, y_off)


def shapes():
    out = {'straight': []}
    for d in (+1, -1):
        for R in (300, 500):
            for after in (0, 150):
                out[f'{"L" if d > 0 else "R"}{R} +{after}'] = [('S', after), ('A', R, 90, d)] if after else [('A', R, 90, d)]
    return out


def scenario(shape, method, rng, w_box=150, d_box=100, yc=0.0, mismatch=0.0, turn_err=0.0,
             window=12, scan_err=25, junction=None):
    pre = 1200.0
    segs = [('S', pre + d_box)] + shape + [('S', 1600)]   # 1000 until 8 Nov: some good runs ended past the tape's end
    path = Path(segs)
    branches = []
    if junction:
        where, sides = junction           # x of the side line from the box front; 'L', 'R', 'LR'
        for sd in sides:
            branches.append((pre + where, 0.0, 800.0 if sd == 'L' else -800.0))
    world = World(path, pre, w_box, d_box, yc, window, branches)
    rb = Robot(world, rng, mismatch, turn_err)
    rb.x, rb.y = 300.0 - AHEAD, 0.0
    p = dict(BASE); p.update(NEW)
    why = rb.follow(p, 2000, trip=True)
    if why != 'obstacle':
        return 'no stop: ' + why, rb
    r = bypass(rb, method, rng, scan_err)
    if rb.hit:
        return 'HIT', rb
    if r != 'rejoined':
        return r, rb
    why = rb.follow(p, 600)
    if rb.hit:
        return 'HIT', rb
    if why == 'lost':
        return 'lost after', rb
    # on the line at the end?
    (lx, ly), _ = rb.sensors()
    if path.near(rb.x + AHEAD * math.cos(rb.h), rb.y + AHEAD * math.sin(rb.h))[0] > 30:
        return 'off line', rb
    return 'ok', rb


JUNCTIONS = {
    # the box on a junction: its side line(s) under the box (8 Nov)
    'T-L under': (50, 'L'), 'T-R under': (50, 'R'), 'X under': (50, 'LR'),
    # a junction just behind the box (side line 200 mm past its back)
    'X 200 behind': (100 + 200, 'LR'),
}


def main():
    rng = random.Random(5)
    SH = shapes()
    global MARGIN_MM
    only = sys.argv[1:] or ['OLD', 'NEW', 'J', 'S']
    for method in only:
        MARGIN_MM = 70 if method == 'OLD' else 80
        tot = ok = hits = 0
        per = {}
        clear = []
        for name, sh in SH.items():
            for w_box in (100, 150, 200):
                for yc in (-20, 0, 20):
                    for mis in (-0.01, 0.01):
                        for te in (-0.05, 0.0, 0.05):
                            r, rb = scenario(sh, method, rng, w_box=w_box, yc=yc,
                                             mismatch=mis, turn_err=te)
                            tot += 1
                            ok += r == 'ok'
                            hits += r == 'HIT'
                            per.setdefault(name, [0, 0])
                            per[name][0] += r == 'ok'
                            per[name][1] += 1
                            if r == 'ok':
                                clear.append(rb.min_clear)
        print(f'{method} (margin {MARGIN_MM}): ok {ok}/{tot}, touched the box {hits}, closest pass '
              f'{min(clear) if clear else 0:.0f} mm')
        print('   ' + '  '.join(f'{k}: {v[0]}/{v[1]}' for k, v in per.items()))
        # boxes on a junction, straight line on
        jper = {}
        for name, j in JUNCTIONS.items():
            for w_box in (100, 150, 200):
                for yc in (-20, 0, 20):
                    for mis in (-0.01, 0.01):
                        for te in (-0.05, 0.0, 0.05):
                            r, rb = scenario(SH['straight'], method, rng, w_box=w_box, yc=yc,
                                             mismatch=mis, turn_err=te, junction=j)
                            jper.setdefault(name, [0, 0, {}])
                            jper[name][0] += r == 'ok'
                            jper[name][1] += 1
                            if r != 'ok':
                                jper[name][2][r] = jper[name][2].get(r, 0) + 1
        print('   junctions: ' + '  '.join(f'{k}: {v[0]}/{v[1]}' + (f' {v[2]}' if v[2] else '')
                                       for k, v in jper.items()))
        # box depth: 150 mm is only the team's assumption (A9)
        dper = {}
        for d_box in (60, 150, 250, 350):
            for name in ('straight', 'L500 +0', 'R500 +0'):
                for w_box in (100, 200):
                    for mis in (-0.01, 0.01):
                        for te in (-0.05, 0.0, 0.05):
                            r, rb = scenario(SH[name], method, rng, w_box=w_box, d_box=d_box,
                                             mismatch=mis, turn_err=te)
                            k = f'{d_box} deep'
                            dper.setdefault(k, [0, 0, {}, []])
                            dper[k][0] += r == 'ok'
                            dper[k][1] += 1
                            if r != 'ok':
                                dper[k][2][r] = dper[k][2].get(r, 0) + 1
                            if r == 'ok':
                                dper[k][3].append(rb.min_clear)
        print('   depth: ' + '  '.join(f'{k}: {v[0]}/{v[1]}' + (f' {v[2]}' if v[2] else '')
                                     + (f' (closest {min(v[3]):.0f})' if v[3] else '')
                                     for k, v in dper.items()))


if __name__ == '__main__':
    main()

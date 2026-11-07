"""Re-run the obstacle width logic on real [SCAN] lines from robot logs.

    python tools/sonarcheck.py robot_log.txt [more logs]
    python tools/sonarcheck.py testing/subsystem5/scans_7nov.txt

For each scan it prints the width with the old rule (until 8 Nov: within
IN_MM of the straight-ahead distance) and with the "geometry" rule that
task_obstacle.c side_edge uses since 8 Nov:

A flat box face square to the robot is at front / cos(angle) along a ray
at that angle.  The HC-SR04 also hears the face's echo from straight in
front through the side of its beam, so off to the side it often reports
about the straight-ahead distance even when it points past the box
(7 Nov: whole +-40 deg sweeps "on the box", 400-500 mm widths for 15-23
cm boxes).  The geometry rule counts a ray as on the box only when its
distance fits front / cos(angle) (within GEO_TOL); a ray much nearer
than that is the side-of-beam echo: neither the box nor something else.
"""
import math
import re
import sys

STEP = 5
IN_MM = 150
BG_MM = 250
BEAM_HALF = 6
GEO_TOL_MM = 15
GEO_TOL_FRAC = 0.04
CENTRE_DEG = 10          # within this, any echo near front is the box


def parse(paths):
    scans = []
    for p in paths:
        for line in open(p, encoding='utf-8', errors='replace'):
            if '[SCAN] rays' in line:
                pairs = re.findall(r'(-?\d+):(-?\d+)', line.split(':', 1)[1])
                scans.append([(int(a), int(b)) for a, b in pairs])
    return scans


def edges(rays, rule):
    phi = [a for a, _ in rays]
    d = [b for _, b in rays]
    c = phi.index(0)
    front = d[c]
    if front < 0:
        return None

    def kind(i):
        if d[i] < 0:
            return 'open'
        if rule == 'robot':
            return 'in' if abs(d[i] - front) <= IN_MM else ('other' if d[i] < front + BG_MM else 'bg')
        exp = front / math.cos(math.radians(phi[i]))
        tol = GEO_TOL_MM + GEO_TOL_FRAC * exp
        if abs(phi[i]) <= CENTRE_DEG and abs(d[i] - front) <= IN_MM:
            return 'in'
        if abs(d[i] - exp) <= tol:
            return 'in'
        if d[i] < exp - tol and d[i] >= front - IN_MM:
            return 'lobe'                # side-of-beam echo of the face
        return 'other' if d[i] < front + BG_MM else 'bg'

    res = {}
    for side, step in (('L', 1), ('R', -1)):
        last_in, out_run, first_out, other = c, 0, None, False
        i = c + step
        while 0 <= i < len(d):
            k = kind(i)
            if k == 'in' and out_run < 2:
                last_in, out_run, first_out = i, 0, None
            elif k != 'in':
                if out_run == 0:
                    first_out = i
                out_run += 1
                if out_run >= 2 and k == 'other':
                    other = True
            i += step
        if out_run < 2 or first_out is None:
            res[side] = (None, False)    # filled to the end: unknown
            continue
        a_edge = 0.5 * (phi[last_in] + phi[first_out]) - step * BEAM_HALF
        if a_edge * step < 0:
            a_edge = 0.0
        res[side] = (d[last_in] * math.sin(math.radians(a_edge)), not other)
    return front, res


def main():
    scans = parse(sys.argv[1:])
    print(f'{len(scans)} scans')
    for n, rays in enumerate(scans, 1):
        out = []
        for rule in ('robot', 'geometry'):      # 'robot' = the old rule
            r = edges(rays, rule)
            if r is None:
                out.append('no front')
                continue
            front, e = r
            (l, lc), (rr, rc) = e['L'], e['R']
            if l is None or rr is None:
                txt = 'edge not seen ' + ('L' if l is None else '') + ('R' if rr is None else '')
                if l is not None or rr is not None:
                    known = l if l is not None else -rr
                    txt += f' (other half {known:.0f})'
            else:
                txt = f'{l - rr:4.0f} mm'
            out.append(txt)
        print(f'scan {n:2d} front {rays[len(rays)//2][1]:5d}:  old rule {out[0]:28s} geometry {out[1]}')


if __name__ == '__main__':
    main()

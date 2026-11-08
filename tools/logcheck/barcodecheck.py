"""Replay the robot's barcode decoder on the "code ended" lines of logs.

    python tools/logcheck/barcodecheck.py robot_log.txt [more logs]

A Python copy of task_barcode.c: decode() is the clean decoder (a "*X*"
anywhere in the elements, either direction); decode2() adds what the robot
does since 8 Nov when that fails: one '*' damaged (the other '*' and the
character decode with wide >= 2 x narrow, narrow gaps between characters,
the damaged block as long as a character +-15 %), or one narrow element
lost (27-28 elements: each element tried as wide-or-narrow + narrow +
wide-or-narrow, must then decode cleanly), or cut short (27-28: one or
two narrow elements added at an end, must decode cleanly; 8 Nov F's weak
last bar).  Only a single answer counts.

It prints the failed reads it can now decode, then three checks on the
good reads in the logs: every narrow element lost in turn, the start or
stop replaced by noise, and random bar patterns (must not decode).
8 Nov: 120 recovered / 0 wrong; 1059 / 0 wrong; 0 of 20 000 (3 of
200 000).  The robot's C code was run on the same inputs (gcc on the
PC): 117 / 0 wrong, 988 / 0 wrong, 0 of 20 000.
"""
import random
import re
import statistics
import sys
C39 = {0x034:'0',0x121:'1',0x061:'2',0x160:'3',0x031:'4',0x130:'5',0x070:'6',0x025:'7',0x124:'8',0x064:'9',
0x109:'A',0x049:'B',0x148:'C',0x019:'D',0x118:'E',0x058:'F',0x00D:'G',0x10C:'H',0x04C:'I',0x01C:'J',
0x103:'K',0x043:'L',0x142:'M',0x013:'N',0x112:'O',0x052:'P',0x007:'Q',0x106:'R',0x046:'S',0x016:'T',
0x181:'U',0x0C1:'V',0x1C0:'W',0x091:'X',0x190:'Y',0x0D0:'Z',0x085:'-',0x184:'.',0x0C4:' ',0x094:'*',
0x0A8:'$',0x0A2:'/',0x08A:'+',0x02A:'%'}
PAT = {v: k for k, v in C39.items()}
RATIO = 1.3
def dchar(e):
    idx = sorted(range(9), key=lambda i: -e[i])[:3]
    pat = sum(1 << (8 - i) for i in idx)
    mw = min(e[i] for i in idx); mn = max(e[i] for i in range(9) if i not in idx)
    if mw < mn * RATIO: return None
    return C39.get(pat)
def decode(e):
    n = len(e)
    if n < 29: return None
    for rev in (False, True):
        x = e[::-1] if rev else e
        for off in range(0, n - 28, 2):
            if dchar(x[off:off+9]) != '*': continue
            for k in range((n - off + 1)//10, 2, -1):
                if off + (k-1)*10 + 9 > n: continue
                if dchar(x[off+(k-1)*10:off+(k-1)*10+9]) != '*': continue
                ch = dchar(x[off+10:off+19])
                if ch and ch != '*': return ch
    return None
def clear(e):
    w = sorted(e)
    return w[6] >= 2.0 * w[5]          # 3 widest at least 2x the 6 narrow

def frames(e):
    """(char, how) for every 29-wide frame: both stars, or one star + width check"""
    out = []
    n = len(e)
    for rev in (False, True):
        x = e[::-1] if rev else e
        for off in range(0, n - 28, 2):
            a, ch, b = dchar(x[off:off+9]), dchar(x[off+10:off+19]), dchar(x[off+20:off+29])
            if not ch or ch == '*': continue
            sa, sm, sb = sum(x[off:off+9]), sum(x[off+10:off+19]), sum(x[off+20:off+29])
            nar = statistics.median(x[off:off+29])
            gaps = x[off+9] < 1.6 * nar and x[off+19] < 1.6 * nar
            if a == '*' and b == '*':
                out.append((ch, 'both'))
            elif (a == '*' or b == '*') and gaps and clear(x[off+10:off+19]) and                     clear(x[off:off+9] if a == '*' else x[off+20:off+29]):
                other = sb if a == '*' else sa
                if abs(other - sm) <= 0.15 * sm:
                    out.append((ch, 'one'))
    return out

def decode2(e, allow_one=True):
    r = decode(e)
    if r: return r, 'exact'
    f = frames(e)
    both = {c for c, h in f if h == 'both'}
    one = {c for c, h in f if h == 'one'}
    if len(both) == 1: return both.pop(), 'frame'
    if allow_one and not both and len(one) == 1: return one.pop(), 'one star'
    # repair: 27 elements = one narrow element lost, two neighbours merged
    if len(e) in (27, 28):
        srt = sorted(e)
        nar = statistics.median(srt[:len(srt)*2//3]); wid = statistics.median(srt[-len(srt)//4:])
        res = set()
        for i in range(len(e)):
            for a in (nar, wid):
                for c in (nar, wid):
                    tot = a + nar + c
                    if abs(e[i] - tot) <= 0.2 * tot:
                        x = e[:i] + [a, nar, c] + e[i+1:]
                        r = decode(x)
                        if r: res.add(r)
        if len(res) == 1: return res.pop(), 'repaired'
        if res: return None, f'ambiguous {sorted(res)}'
    # cut short: a weak narrow bar (and the space before it) at an end not seen
    if len(e) in (27, 28):
        srt = sorted(e)
        nar = statistics.median(srt[:len(srt) * 2 // 3])
        res = set()
        for k in (1, 2):
            if len(e) + k < 29:
                continue
            for x in (e + [nar] * k, [nar] * k + e):
                r = decode(x)
                if r:
                    res.add(r)
        if len(res) == 1:
            return res.pop(), 'cut short'
    return None, 'fail'

if __name__ == '__main__':
    goods, fails = [], []
    seen = set()
    for fn in sys.argv[1:]:
        for s in open(fn, encoding='utf-8', errors='replace'):
            m = re.search(r'code ended, (\d+) elements \(mm\): (.*)', s)
            if not m: continue
            e = [float(v) for v in m[2].replace('|', ' ').split()]
            if tuple(e) in seen: continue
            seen.add(tuple(e))
            (goods if decode(e) else fails).append(e)
    print(len(goods), 'good reads,', len(fails), 'failed reads')
    for e in fails:
        if len(e) >= 25: print('  fail', len(e), decode2(e))
    # merge tests on the good reads
    ok = wrong = none = 0
    rng = random.Random(1)
    for e in goods:
        truth = decode(e)
        for i in range(1, len(e) - 1):
            if e[i] > 5: continue                 # only a narrow element is lost
            x = e[:i-1] + [e[i-1] + e[i] + e[i+1]] + e[i+2:]
            r, how = decode2(x)
            if r == truth: ok += 1
            elif r: wrong += 1; print('   WRONG', truth, '->', r, how, 'merge at', i)
            else: none += 1
    print(f'one narrow element lost (every place, every good read): recovered {ok}, wrong {wrong}, not read {none}')
    # damaged start/stop: replace first 3 elements by noise
    ok = wrong = none = 0
    for e in goods:
        truth = decode(e)
        for t in range(200):
            x = list(e)
            k = rng.choice([0, 26])
            for j in range(k, k + 3):
                x[j] = round(rng.uniform(2, 11), 1)
            r, how = decode2(x)
            if r == truth: ok += 1
            elif r: wrong += 1
            else: none += 1
    print(f'start or stop damaged (3 random elements): right {ok}, wrong {wrong}, not read {none}')
    # random junk: must never decode
    wrong = 0
    for t in range(20000):
        n = rng.choice([27, 28, 29, 31])
        x = [round(rng.choice([rng.uniform(2.5, 4.5), rng.uniform(8, 11)]), 1) for _ in range(n)]
        r, how = decode2(x)
        if r: wrong += 1
    print(f'random bar patterns (20000): decoded as something {wrong}')

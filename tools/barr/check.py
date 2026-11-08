"""Count Barr C ("Embedded C Coding Standard", Barr Group 2018) rule breaks
in app_program/ -- the rules a script can measure.

    python tools/barr/check.py              # summary per rule
    python tools/barr/check.py --list 7.1e  # every place for one rule

The micro T-Kernel / template code (kernel/, lib/, include/, device/, etc/)
is not ours and is not checked; neither are its names and types where our
code has to use them (docs/team/BARR_C_DEVIATIONS.md).
"""
import glob
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import transform  # noqa: E402

RULES = {
    '1.2a': 'line longer than 80 characters',
    '1.3a': 'if / else / for / while body without braces',
    '1.3b': 'left brace not on a line of its own',
    '1.4b': '&& / || operand not in brackets',
    '1.7c': 'goto (preferred: none)',
    '1.7d': 'continue (preferred: none)',
    '2.1b': 'comment contains /*, // or a backslash',
    '3.3c': 'no end-of-file comment',
    '3.5a': 'tab character',
    '3.6a': 'CR-LF line end',
    '4.2b': 'header without an include guard',
    '5.1a': 'type name not lower case ending in _t',
    '5.2a': 'T-Kernel integer type (INT, W, UW, UB, H, UH) in our own code',
    '5.4b': 'float instead of float32_t',
    '5.6a': 'BOOL instead of bool',
    '6.1f': 'macro name with lower-case letters',
    '6.2a': 'function longer than 100 lines',
    '6.5b': 'interrupt handler name not ending in _isr',
    '7.1e': 'variable name shorter than 3 characters',
    '8.1a': 'more than one variable in a declaration',
    '8.3b': 'switch without default',
    '8.4c': 'while (1) instead of for (;;)',
    '8.6a': '== with the constant on the right',
}


def files():
    return sorted(glob.glob('app_program/**/*.c', recursive=True) +
                  glob.glob('app_program/**/*.h', recursive=True))


def check_file(f, hits):
    raw = open(f, 'rb').read().decode('utf-8', errors='replace')
    src = raw.replace('\r\n', '\n')
    lines = src.split('\n')

    def hit(rule, ln, text=''):
        hits.setdefault(rule, []).append(f'{f}:{ln}: {text.strip()[:80]}')

    if '\r\n' in raw:
        hit('3.6a', 1)
    for n, line in enumerate(lines, 1):
        if '\t' in line:
            hit('3.5a', n, line)
        if len(line.expandtabs(4)) > 80:
            hit('1.2a', n, line)
        if re.search(r'\bwhile\s*\(\s*1\s*\)', line):
            hit('8.4c', n, line)
        if re.search(r'\bgoto\b', line) and not line.strip().startswith(('/*', '*')):
            hit('1.7c', n, line)
        if re.search(r'\bcontinue\s*;', line):
            hit('1.7d', n, line)
        s = line.strip()
        if s.endswith('{') and s != '{' and not re.search(r'=\s*\{$', s) \
                and not s.startswith(('/*', '*', '#')) and not s.endswith('\\{'):
            hit('1.3b', n, line)
        if re.match(r'^(if|else if|for|while)\b.*\)\s*[^{;\s].*;\s*$', s) or \
                re.match(r'^else\s+[^{i].*;\s*$', s):
            hit('1.3a', n, line)
        m = re.match(r'#\s*define\s+(\w+)', s)
        if m and re.search(r'[a-z]', m.group(1)):
            hit('6.1f', n, line)
        for m in re.finditer(r'\b(INT|UINT|W|UW|UB|H|UH)\b', line):
            if not s.startswith(('/*', '*', '//')):
                hit('5.2a', n, line)
                break
        if re.search(r'\bfloat\b', line) and not s.startswith(('/*', '*')):
            hit('5.4b', n, line)
        if re.search(r'\bBOOL\b', line) and not s.startswith(('/*', '*')):
            hit('5.6a', n, line)
    if not src.rstrip().endswith('end of file ***/'):
        hit('3.3c', len(lines))
    if f.endswith('.h') and not re.search(r'#\s*ifndef\s+\w+_H\b', src):
        hit('4.2b', 1)
    # comments
    for m in re.finditer(r'/\*(.*?)\*/', src, re.S):
        body = m.group(1)
        if '/*' in body or '//' in body or '\\' in body:
            hit('2.1b', src.count('\n', 0, m.start()) + 1, body[:60])
    # type names
    for m in re.finditer(r'}\s*(\w+)\s*;|typedef\s+[\w\s\*]+?\b(\w+)\s*;', src):
        name = m.group(1) or m.group(2)
        if name in ('break', 'return', 'continue'):
            continue
        if name and not re.fullmatch(r'[a-z][a-z0-9_]*_t', name):
            hit('5.1a', src.count('\n', 0, m.start()) + 1, name)
    # switch without default
    for m in re.finditer(r'\bswitch\s*\(', src):
        body_start = src.find('{', m.end())
        depth = 0
        for j in range(body_start, len(src)):
            if src[j] == '{':
                depth += 1
            elif src[j] == '}':
                depth -= 1
                if depth == 0:
                    if not re.search(r'\bdefault\s*:', src[body_start:j]):
                        hit('8.3b', src.count('\n', 0, m.start()) + 1)
                    break
    # interrupt handlers: names given to tk_def_int / app_gpio_irq_add
    for m in re.finditer(r'inthdr\s*=\s*\(FP\)\s*(\w+)|app_gpio_irq_add\([^,]+,[^,]+,\s*(\w+)', src):
        name = m.group(1) or m.group(2)
        if not name.endswith('_isr'):
            hit('6.5b', src.count('\n', 0, m.start()) + 1, name)
    # function lengths
    for m in re.finditer(r'\n(?:EXPORT|LOCAL|static)?\s*[\w\s\*]+?\b(\w+)\s*\([^;{)]*\)\s*\n\{', src):
        start = src.find('{', m.end() - 1)
        depth = 0
        for j in range(start, len(src)):
            if src[j] == '{':
                depth += 1
            elif src[j] == '}':
                depth -= 1
                if depth == 0:
                    n = src.count('\n', start, j)
                    if n > 100:
                        hit('6.2a', src.count('\n', 0, m.start()) + 2, f'{m.group(1)}: {n} lines')
                    break
    # expression rules and declarations, from the transformer's parser
    toks = transform.tokenize(src)
    idx = transform.code_index(toks)
    code = [toks[i] for i in idx]
    regions, decls, _ = transform.find_regions(code)
    ed = transform.Edits()
    for (lo, hi) in regions:
        try:
            transform.walk(code, transform.Parser(code, lo, hi).full(), ed)
        except transform.Fail:
            pass
    line_of = {}
    pos = 0
    ln = 1
    for i, t in enumerate(toks):
        line_of[i] = ln
        ln += t.text.count('\n')
    for (l, r) in ed.swaps:
        hit('8.6a', line_of[idx[l.a]])
    for (side, ci) in ed.ins:
        if side == 'L':
            hit('1.4b', line_of[idx[ci]])
    for (lo, hi) in decls:
        sp = transform.split_decl(code, lo, hi)
        if sp:
            hit('8.1a', line_of[idx[lo]])
        # declarator names
        rng = sp[1] if sp else None
        if rng is None:
            j = lo
            while j < hi and code[j].text in transform.TYPE_WORDS:
                j += 1
            rng = [(j, hi)]
        for (p, q) in rng:
            k = p
            while k < q and code[k].text == '*':
                k += 1
            if k < q and code[k].kind == 'ident':
                name = code[k].text
                if len(name) < 3 and not (k + 1 < q and code[k + 1].text == '('):
                    hit('7.1e', line_of[idx[k]], name)


def main():
    transform.add_typedefs(files())
    hits = {}
    for f in files():
        check_file(f, hits)
    if len(sys.argv) > 2 and sys.argv[1] == '--list':
        for h in hits.get(sys.argv[2], []):
            print(h)
        return
    total = 0
    for rule, what in RULES.items():
        n = len(hits.get(rule, []))
        total += n
        print(f'{rule:6s} {n:5d}  {what}')
    print(f'total  {total:5d}')


if __name__ == '__main__':
    main()

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
    # added 8 Nov, second round
    '1.6a': 'cast without a comment in its statement or above it',
    '2.2h': 'function without a Doxygen comment (/** ... */) before it',
    '3.3b': 'no blank line before / after a block (if, for, while, switch)',
    '4.1b': 'module name not unique in its first 8 characters',
    '4.2a': 'source file without a header of the same name',
    '6.1i': 'public function not prefixed with its module name',
    '8.2b': 'if ... else nested deeper than two levels',
    '8.2d': 'else if chain without a final else',
    '8.4a': 'magic number in a loop start or end test',
    '8.4d': 'empty loop body without braces and a comment',
}

# a module's public functions start with one of these (Barr 6.1i: "prefixed
# with their module name"); after the module renames this is the file root
MODULE_PREFIX = {}

# Deviations (docs/team/BARR_C_DEVIATIONS.md):
# D1: the course template's module names subsystemN and subsystemN_test (all
#     "subsyste" in their first 8 characters, 4.1b); kept as given
TEMPLATE_MODULE = re.compile(r'^subsystem\d(_test)?$')
# D2: hooks the RTOS-side glue calls by these names (weak symbols in
#     lib/liblwip/.../mqtt_utk.c and lib/libtm/.../tm_com.c), 6.1i
FIXED_NAMES = {'app_mqtt_rx', 'app_mqtt_tx', 'tm_console_mirror'}


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
    blank = lambda m: re.sub(r'[^\n]', ' ', m.group(0))   # noqa: E731
    code_only = re.sub(r'"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'', blank, src)
    code_only = re.sub(r'/\*.*?\*/', blank, code_only, flags=re.S)
    code_only = re.sub(r'//[^\n]*', blank, code_only)
    code_lines = code_only.split('\n')
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
        # (a header that goes on to the next line is not a body)
        if (re.match(r'^(if|else if|for|while)\b.*\)\s*[^{;\s].*;\s*$', s) and
                s.count('(') == s.count(')')) or \
                re.match(r'^else\s+[^{i].*;\s*$', s):
            hit('1.3a', n, line)
        m = re.match(r'#\s*define\s+(\w+)', s)
        if m and re.search(r'[a-z]', m.group(1)):
            hit('6.1f', n, line)
        # code only: no strings, character literals or comments (a block
        # comment's later lines too)
        bare = code_lines[n - 1]
        prev_line = lines[n - 2] if n >= 2 else ''
        deviation = 'deviation' in line or 'deviation' in prev_line
        if re.search(r'\b(INT|UINT|W|UW|UB|H|UH)\b', bare) and not deviation:
            hit('5.2a', n, line)
        if re.search(r'\bfloat\b', bare) and not re.search(r'typedef\s+float\s+float32_t', bare):
            hit('5.4b', n, line)
        if re.search(r'\bBOOL\b', bare) and not deviation:
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
    check_structure(f, src, lines, toks, idx, code, line_of, hit)
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


# (void) alone is left out: it converts no value, it only discards one (an
# unused parameter, a result not needed), so there is no range to explain
CAST_RE = re.compile(
    r'\(\s*(?:const\s+|volatile\s+)*'
    r'(?:u?int(?:8|16|32|64)_t|float32_t|bool|BOOL|ER|ID|UINT|INT|void(?=\s*\*)|char|'
    r'unsigned\s+char|unsigned|int|FP|TMO|RELTIM|ATR|PRI|SZ|[a-z_]+_t)'
    r'(?:\s*\*)*\s*\)\s*(?=[\w(\-!~"\'*&])')


def statement_blocks(code):
    """(keyword index, body open index, body close index) of if / else /
    for / while / switch / do bodies in braces"""
    out = []
    for i, t in enumerate(code):
        if t.text in ('if', 'for', 'while', 'switch') and i + 1 < len(code) and code[i + 1].text == '(':
            close = transform.match(code, i + 1)
            if close + 1 < len(code) and code[close + 1].text == '{':
                out.append((i, close + 1, transform.match(code, close + 1)))
        elif t.text in ('else', 'do') and i + 1 < len(code) and code[i + 1].text == '{':
            out.append((i, i + 1, transform.match(code, i + 1)))
    return out


def check_structure(f, src, lines, toks, idx, code, line_of, hit):
    import names
    ln = lambda ci: line_of[idx[ci]]                       # noqa: E731
    # 1.6a casts: a comment that goes with the cast: on its line, earlier in
    # the same statement, or directly above the statement
    def cast_commented(n):
        k = n                       # 1-based line of the cast
        if '/*' in lines[n - 1]:
            return True
        while k >= 2:
            prev = lines[k - 2].rstrip().rstrip('\\').rstrip()
            prev_code = re.sub(r'/\*.*?(\*/|$)', '', prev).strip()
            if prev_code == '' and prev != '':
                return True         # a comment line: above the statement or in it
            if prev_code == '' or prev_code.endswith((';', '{', '}')) or \
                    prev_code.startswith('#'):
                return False        # the statement starts at line k
            if '/*' in prev:
                return True         # a comment earlier in the same statement
            k -= 1
        return False

    for n, line in enumerate(lines, 1):
        bare = re.sub(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'', '""', line)
        code_part = re.sub(r'/\*.*?(\*/|$)|//.*$', '', bare)
        if CAST_RE.search(code_part) and not code_part.lstrip().startswith('#'):
            if not cast_commented(n):
                hit('1.6a', n, line)
    # 2.2h a Doxygen comment before every function
    for (name, plo, phi, blo, bhi) in names.functions(code):
        # first token of the definition: walk back over the return type
        k = plo - 2
        while k > 0 and code[k - 1].text not in (';', '}', '{') and code[k - 1].kind != 'pp':
            k -= 1
        ti = idx[k] - 1
        while ti >= 0 and toks[ti].kind == 'ws':
            ti -= 1
        if ti < 0 or toks[ti].kind != 'comment' or not toks[ti].text.startswith('/**'):
            hit('2.2h', ln(k), name)
    # 8.2b / 8.2d / 3.3b / 8.4 on statements
    blocks = statement_blocks(code)
    owner = {}
    for (kw, bo, bc) in blocks:
        owner[bo] = kw
    for (kw, bo, bc) in blocks:
        word = code[kw].text
        # 8.2b: if inside if/else bodies, more than two levels
        if word == 'if' and not (kw > 0 and code[kw - 1].text == 'else'):
            depth = 1
            for (kw2, bo2, bc2) in blocks:
                if bo2 < kw < bc2 and code[kw2].text in ('if', 'else'):
                    depth += 1
            if depth > 2:
                hit('8.2b', ln(kw), f'depth {depth}')
        # 8.2d: if ... else if ... must end with else
        if word == 'if' and not (kw > 0 and code[kw - 1].text == 'else'):
            j = bc
            had_elif = False
            ends_else = False
            while j + 1 < len(code) and code[j + 1].text == 'else':
                if j + 2 < len(code) and code[j + 2].text == 'if':
                    had_elif = True
                    close = transform.match(code, j + 3)
                    j = transform.match(code, close + 1) if code[close + 1].text == '{' else close
                else:
                    ends_else = True
                    break
            if had_elif and not ends_else:
                hit('8.2d', ln(kw))
        # 8.4a: magic numbers in loop start / end tests
        if word in ('for', 'while'):
            close = transform.match(code, kw + 1)
            parts = code[kw + 2:close]
            if word == 'for':
                semi = [i for i, t in enumerate(parts) if t.text == ';']
                parts = parts[:semi[1]] if len(semi) >= 2 else parts
            for pi, t in enumerate(parts):
                if t.kind != 'number' or t.text.rstrip('uUlL') in ('0', '1'):
                    continue
                if t.text.rstrip('uUlL') == '2' and pi > 0 and parts[pi - 1].text == '/':
                    continue            # count / 2: half way, not a magic number
                hit('8.4a', ln(kw), ' '.join(x.text for x in parts)[:60])
                break
        # 8.4d: empty loop body needs braces and a comment ('{ ; }' is empty)
        if word in ('for', 'while') and all(c.text == ';' for c in code[bo + 1:bc]):
            ti_a, ti_b = idx[bo], idx[bc]
            if not any(toks[t].kind == 'comment' for t in range(ti_a, ti_b)):
                hit('8.4d', ln(kw))
    # loops with a bare ';' body
    for i, t in enumerate(code):
        if t.text in ('for', 'while') and i + 1 < len(code) and code[i + 1].text == '(':
            close = transform.match(code, i + 1)
            if close + 1 < len(code) and code[close + 1].text == ';' and \
                    not (i > 0 and code[i - 1].text == '}'):
                hit('8.4d', ln(i))
    # 3.3b: blank line before and after each if / for / while / switch / do
    for (kw, bo, bc) in blocks:
        word = code[kw].text
        if word == 'else' or (word == 'if' and kw > 0 and code[kw - 1].text == 'else'):
            continue
        if word == 'while' and kw > 0 and code[kw - 1].text == '}':
            continue                                     # do ... while
        start = ln(kw)
        prev = lines[start - 2].strip() if start >= 2 else ''
        if prev not in ('', '{') and not prev.endswith(('*/', ':')) and not prev.startswith(('/*', '*', '#')):
            hit('3.3b', start, 'before')
        # the end of the whole statement (if: after its else chain)
        j = bc
        while j + 1 < len(code) and code[j + 1].text == 'else':
            if code[j + 2].text == 'if':
                close = transform.match(code, j + 3)
                j = transform.match(code, close + 1)
            else:
                j = transform.match(code, j + 2)
        if word == 'do' and j + 1 < len(code) and code[j + 1].text == 'while':
            j = transform.match(code, j + 2) + 1
        end = ln(j)
        nxt = lines[end].strip() if end < len(lines) else ''
        if nxt not in ('', '}') and not nxt.startswith(('break;', '}', 'case ', 'default', '#')):
            hit('3.3b', end, 'after')


def module_checks(all_files, hits):
    roots = {}
    for f in all_files:
        root = os.path.splitext(os.path.basename(f))[0]
        roots.setdefault(root, set()).add(os.path.splitext(f)[1])
    by8 = {}
    for root in roots:
        if TEMPLATE_MODULE.match(root):
            continue                    # deviation D1 (docs/team/BARR_C_DEVIATIONS.md)
        by8.setdefault(root[:8], set()).add(root)
    for key, group in by8.items():
        if len(group) > 1:
            for root in sorted(group):
                hits.setdefault('4.1b', []).append(f'{root}: shares "{key}" with {sorted(group - {root})}')
    for root, exts in roots.items():
        if '.c' in exts and '.h' not in exts:
            hits.setdefault('4.2a', []).append(f'{root}.c has no {root}.h')
    # 6.1i: EXPORT functions of a .c start with the module (file root) name
    for f in all_files:
        if not f.endswith('.c'):
            continue
        root = os.path.splitext(os.path.basename(f))[0]
        src = open(f, encoding='utf-8').read()
        for m in re.finditer(r'^EXPORT\s+[\w\s\*]+?\b(\w+)\s*\(', src, re.M):
            name = m.group(1)
            prefixes = MODULE_PREFIX.get(root, [root])
            if not any(name.startswith(p + '_') or name == p for p in prefixes) and \
                    not name.endswith('_task') and name != 'usermain' and \
                    name not in FIXED_NAMES:
                hits.setdefault('6.1i', []).append(f'{f}: {name}')


def main():
    transform.add_typedefs(files())
    hits = {}
    for f in files():
        check_file(f, hits)
    module_checks(files(), hits)
    # 2.2h: a function documented at its prototype (in a header) counts
    documented = set()
    for f in files():
        src = open(f, encoding='utf-8').read()
        for m in re.finditer(r'/\*\*(?:(?!\*/).)*\*/[ \t]*\n[ \t]*(?:[\w\*]+[ \t]+)*?\**[ \t]*(\w+)[ \t]*\(',
                             src, re.S):
            documented.add(m.group(1))
    hits['2.2h'] = [h for h in hits.get('2.2h', []) if h.rsplit(': ', 1)[-1] not in documented]
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

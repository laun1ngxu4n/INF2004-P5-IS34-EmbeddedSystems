"""Barr C expression and declaration rules, applied to app_program/ sources.

    python tools/barr/transform.py app_program/<file>.c [...]

Rewrites, without changing what the code does:
  8.6   x == CONST            ->  CONST == x
  1.4b  a < b && c            ->  (a < b) && c   (each && / || operand that is
                                   not a single identifier or constant)
  1.4a  a + b * c             ->  a + (b * c)    (an operand of a different
                                   precedence class, and a ?: condition)
  8.1   INT a, b = 1;         ->  INT a;  INT b = 1;
  8.4c  while (1)             ->  for (;;)

Only brackets are inserted and == operands swapped, at the places the C
grammar already groups them, so the meaning cannot change; the images are
then compared byte for byte (tools/barr/compare.sh).  Statements the
parser does not understand are left alone and listed.  Preprocessor lines
(macro bodies) are not touched.
"""
import re
import sys

TYPE_WORDS = set('''
void char short int long float double signed unsigned const volatile static
extern register struct union enum _Bool bool
INT UINT W UW H UH B UB D UD BOOL ID ER FP VP SZ TMO RELTIM ATR PRI
LOCAL EXPORT IMPORT Inline
int8_t int16_t int32_t int64_t uint8_t uint16_t uint32_t uint64_t
float32_t float64_t size_t
T_CTSK T_CCYC T_DINT T_CFLG T_CSEM T_CMTX T_CMBF T_RTSK T_RFLG
MOTION_STATUS BARCODE_RESULT OBSTACLE_PROFILE EDGE C39
motion_status_t barcode_result_t obstacle_profile_t edge_t c39_t
'''.split())

# C binary operators: precedence (higher binds tighter) and class for 1.4a
BINOPS = {
    '*': 13, '/': 13, '%': 13,
    '+': 12, '-': 12,
    '<<': 11, '>>': 11,
    '<': 10, '<=': 10, '>': 10, '>=': 10,
    '==': 9, '!=': 9,
    '&': 8, '^': 7, '|': 6,
    '&&': 5, '||': 4,
}
ASSIGN = {'=', '+=', '-=', '*=', '/=', '%=', '&=', '|=', '^=', '<<=', '>>='}

TOKEN_RE = re.compile(r'''
    (?P<ws>[ \t\r\n]+)
  | (?P<comment>/\*.*?\*/|//[^\n]*)
  | (?P<string>"(?:\\.|[^"\\\n])*")
  | (?P<char>'(?:\\.|[^'\\\n])*')
  | (?P<number>\.?\d[\w.]*(?:[eEpP][+-]\d+)?\w*)
  | (?P<ident>[A-Za-z_]\w*)
  | (?P<punct><<=|>>=|->|\+\+|--|<<|>>|<=|>=|==|!=|&&|\|\||\+=|-=|\*=|/=|%=|&=|\|=|\^=|\.\.\.|[{}()\[\];,.?:~!%^&*+\-=<>|/#])
''', re.S | re.X)


class Tok:
    __slots__ = ('kind', 'text')

    def __init__(self, kind, text):
        self.kind = kind
        self.text = text


def tokenize(src):
    toks = []
    pos = 0
    at_line_start = True
    while pos < len(src):
        if at_line_start:
            m = re.match(r'[ \t]*#', src[pos:])
            if m:
                # whole preprocessor directive, with continuation lines
                end = pos
                while True:
                    nl = src.find('\n', end)
                    if nl < 0:
                        end = len(src)
                        break
                    line = src[end:nl].rstrip('\r')
                    end = nl + 1
                    if open_comment(src[pos:end]):
                        # a /* comment running past the end of the line
                        close = src.find('*/', end)
                        if close < 0:
                            end = len(src)
                            break
                        nl2 = src.find('\n', close)
                        end = len(src) if nl2 < 0 else nl2 + 1
                        if not src[close:end].rstrip('\r\n').endswith('\\'):
                            break
                        continue
                    if not line.endswith('\\'):
                        break
                toks.append(Tok('pp', src[pos:end]))
                pos = end
                at_line_start = True
                continue
        m = TOKEN_RE.match(src, pos)
        if not m:
            raise SyntaxError(f'cannot tokenize at {pos}: {src[pos:pos+20]!r}')
        kind = m.lastgroup
        text = m.group(0)
        toks.append(Tok(kind, text))
        pos = m.end()
        if kind == 'ws' or kind == 'comment':
            if '\n' in text:
                at_line_start = True
        else:
            at_line_start = False
    return toks


def open_comment(text):
    """True if text ends inside a /* comment"""
    i = 0
    inside = False
    while i < len(text):
        if inside:
            if text.startswith('*/', i):
                inside = False
                i += 2
                continue
        elif text.startswith('/*', i):
            inside = True
            i += 2
            continue
        elif text.startswith('//', i):
            nl = text.find('\n', i)
            if nl < 0:
                return False
            i = nl
            continue
        elif text[i] in '"\'':
            quote = text[i]
            i += 1
            while i < len(text) and text[i] != quote:
                i += 2 if text[i] == '\\' else 1
        i += 1
    return inside


def add_typedefs(paths):
    """typedef names declared in our own sources"""
    for f in paths:
        s = open(f, encoding='utf-8', errors='replace').read()
        for m in re.finditer(r'typedef[^;{]*?(\w+)\s*;', s):
            TYPE_WORDS.add(m.group(1))
        for m in re.finditer(r'}\s*(\w+)\s*;', s):
            TYPE_WORDS.add(m.group(1))


class Fail(Exception):
    pass


class Node:
    """an expression: token index range [a, b) in the code-token list"""
    __slots__ = ('kind', 'op', 'a', 'b', 'kids')

    def __init__(self, kind, a, b, op=None, kids=()):
        self.kind = kind      # 'prim', 'paren', 'bin', 'cond', 'assign', 'unary', 'cast', 'post', 'comma'
        self.op = op
        self.a = a
        self.b = b
        self.kids = list(kids)


class Parser:
    """precedence climbing over code tokens (no whitespace / comments)"""

    def __init__(self, code, lo, hi):
        self.c = code
        self.i = lo
        self.hi = hi

    def peek(self, k=0):
        j = self.i + k
        return self.c[j].text if j < self.hi else None

    def take(self, t=None):
        if self.i >= self.hi:
            raise Fail('end')
        if t is not None and self.c[self.i].text != t:
            raise Fail(f'want {t} got {self.c[self.i].text}')
        self.i += 1
        return self.i - 1

    def full(self):
        n = self.comma()
        if self.i != self.hi:
            raise Fail('trailing')
        return n

    def comma(self):
        a = self.i
        n = self.assign()
        kids = [n]
        while self.peek() == ',':
            self.take()
            kids.append(self.assign())
        return n if len(kids) == 1 else Node('comma', a, self.i, ',', kids)

    def assign(self):
        a = self.i
        lhs = self.cond()
        if self.peek() in ASSIGN:
            op = self.c[self.take()].text
            rhs = self.assign()
            return Node('assign', a, self.i, op, [lhs, rhs])
        return lhs

    def cond(self):
        a = self.i
        c = self.binary(0)
        if self.peek() == '?':
            self.take()
            x = self.comma()
            self.take(':')
            y = self.cond()
            return Node('cond', a, self.i, '?', [c, x, y])
        return c

    def binary(self, minp):
        a = self.i
        lhs = self.unary()
        while True:
            op = self.peek()
            p = BINOPS.get(op)
            if p is None or p < minp:
                return lhs
            self.take()
            rhs = self.binary(p + 1)
            lhs = Node('bin', a, self.i, op, [lhs, rhs])

    def is_type_paren(self):
        """( type-name ) at self.i ?"""
        if self.peek() != '(':
            return False
        j = self.i + 1
        seen = False
        while j < self.hi:
            t = self.c[j].text
            if t == ')':
                return seen
            if t in TYPE_WORDS or (seen and t == '*'):
                seen = True
                j += 1
                continue
            return False
        return False

    def unary(self):
        a = self.i
        t = self.peek()
        if t in ('+', '-', '!', '~', '*', '&', '++', '--'):
            self.take()
            k = self.unary()
            return Node('unary', a, self.i, t, [k])
        if t == 'sizeof':
            self.take()
            if self.is_type_paren():
                self.skip_parens()
                return Node('prim', a, self.i)
            k = self.unary()
            return Node('unary', a, self.i, 'sizeof', [k])
        if self.is_type_paren():
            self.skip_parens()
            k = self.unary()
            return Node('cast', a, self.i, None, [k])
        return self.postfix()

    def skip_parens(self):
        depth = 0
        while True:
            t = self.c[self.take()].text
            if t == '(':
                depth += 1
            elif t == ')':
                depth -= 1
                if depth == 0:
                    return

    def postfix(self):
        a = self.i
        n = self.primary()
        while True:
            t = self.peek()
            if t == '(':
                self.take()
                args = []
                if self.peek() != ')':
                    args.append(self.assign())
                    while self.peek() == ',':
                        self.take()
                        args.append(self.assign())
                self.take(')')
                n = Node('post', a, self.i, 'call', [n] + args)
            elif t == '[':
                self.take()
                ix = self.comma()
                self.take(']')
                n = Node('post', a, self.i, 'index', [n, ix])
            elif t in ('.', '->'):
                self.take()
                if self.i >= self.hi or self.c[self.i].kind != 'ident':
                    raise Fail('member')
                self.take()
                n = Node('post', a, self.i, 'member', [n])
            elif t in ('++', '--'):
                self.take()
                n = Node('post', a, self.i, t, [n])
            else:
                return n

    def primary(self):
        a = self.i
        if self.i >= self.hi:
            raise Fail('end')
        tk = self.c[self.i]
        if tk.kind in ('ident', 'number', 'char'):
            if tk.kind == 'ident' and tk.text in TYPE_WORDS:
                raise Fail('type word')
            self.take()
            return Node('prim', a, self.i)
        if tk.kind == 'string':
            self.take()
            while self.i < self.hi and self.c[self.i].kind == 'string':
                self.take()
            return Node('prim', a, self.i)
        if tk.text == '(':
            self.take()
            inner = self.comma()
            self.take(')')
            return Node('paren', a, self.i, None, [inner])
        raise Fail(f'primary {tk.text!r}')


def is_const(code, n):
    if n.kind != 'prim' or n.b - n.a != 1:
        return False
    t = code[n.a]
    if t.kind in ('number', 'char'):
        return True
    return t.kind == 'ident' and re.fullmatch(r'[A-Z][A-Z0-9_]*', t.text) is not None


def is_single(code, n):
    """a single identifier or constant, or already in brackets"""
    if n.kind == 'paren':
        return True
    return n.kind == 'prim' and n.b - n.a == 1


def prec_class(n):
    return BINOPS.get(n.op) if n.kind == 'bin' else None


class Edits:
    def __init__(self):
        self.ins = {}       # code index -> list of (before/after, text)
        self.swaps = []     # (left node, right node)

    def wrap(self, n):
        self.ins.setdefault(('L', n.a), []).append('(')
        self.ins.setdefault(('R', n.b - 1), []).append(')')


def walk(code, n, ed):
    for k in n.kids:
        walk(code, k, ed)
    if n.kind == 'bin':
        l, r = n.kids
        if n.op in ('&&', '||'):
            for k in (l, r):
                # a chain a && b && c: the left operand is the same operator
                if k.kind == 'bin' and k.op == n.op:
                    continue
                if not is_single(code, k):
                    ed.wrap(k)
        else:
            p = BINOPS[n.op]
            for k in (l, r):
                q = prec_class(k)
                if q is not None and q != p and k.op not in ('&&', '||'):
                    ed.wrap(k)
            if n.op == '==' and is_const(code, r) and not is_const(code, l) and                     l.kind in ('prim', 'post', 'paren', 'unary', 'cast'):
                ed.swaps.append((l, r))
    elif n.kind == 'cond':
        c = n.kids[0]
        if c.kind == 'bin' and c.op not in ('&&', '||'):
            ed.wrap(c)
        elif c.kind in ('assign', 'cond'):
            ed.wrap(c)


def code_index(toks):
    return [i for i, t in enumerate(toks) if t.kind not in ('ws', 'comment', 'pp')]


def find_regions(code):
    """expression regions (lo, hi) in code-token positions, and declaration
    statements to split.  Only inside function bodies."""
    regions = []
    decls = []
    while1 = []
    n = len(code)
    i = 0
    depth = 0
    while i < n:
        t = code[i].text
        if t == '{':
            depth += 1
            i += 1
            continue
        if t == '}':
            depth -= 1
            i += 1
            continue
        if t == '(':
            # never start a statement inside brackets (a parameter list,
            # an attribute): skip the whole group
            i = match(code, i) + 1
            continue
        if depth == 0:
            # file scope: declarations only
            j = stmt_end(code, i)
            if j is not None and code[j].text == ';' and is_decl(code, i):
                decls.append((i, j))
                i = j + 1
                continue
            i += 1
            continue
        if t in ('if', 'while', 'switch') and i + 1 < n and code[i + 1].text == '(':
            close = match(code, i + 1)
            if t == 'while' and close == i + 3 and code[i + 2].text == '1':
                while1.append(i)
            regions.append((i + 2, close))
            i = close + 1
            continue
        if t == 'for' and i + 1 < n and code[i + 1].text == '(':
            close = match(code, i + 1)
            parts = []
            s = i + 2
            d = 0
            for j in range(i + 2, close):
                x = code[j].text
                if x in '([{':
                    d += 1
                elif x in ')]}':
                    d -= 1
                elif x == ';' and d == 0:
                    parts.append((s, j))
                    s = j + 1
            parts.append((s, close))
            for (a, b) in parts:
                if b > a:
                    if is_decl(code, a):
                        decls.append((a, b))
                    else:
                        regions.append((a, b))
            i = close + 1
            continue
        if t in ('else', 'do', ';'):
            i += 1
            continue
        if t in ('case', 'default'):
            while i < n and code[i].text != ':':
                i += 1
            i += 1
            continue
        if code[i].kind == 'ident' and i + 1 < n and code[i + 1].text == ':' and t not in TYPE_WORDS:
            i += 2          # a label
            continue
        if t in ('break', 'continue'):
            i += 2
            continue
        if t == 'goto':
            i += 3
            continue
        j = stmt_end(code, i)
        if j is None:
            i += 1
            continue
        if t == 'return':
            if j > i + 1:
                regions.append((i + 1, j))
        elif is_decl(code, i):
            decls.append((i, j))
        else:
            regions.append((i, j))
        i = j + 1
    return regions, decls, while1


def match(code, i):
    d = 0
    for j in range(i, len(code)):
        x = code[j].text
        if x in '([{':
            d += 1
        elif x in ')]}':
            d -= 1
            if d == 0:
                return j
    raise Fail('unbalanced')


def stmt_end(code, i):
    d = 0
    for j in range(i, len(code)):
        x = code[j].text
        if x in '([':
            d += 1
        elif x in ')]':
            d -= 1
        elif x == '{':
            if d == 0:
                # an initializer list after '=' is part of the statement
                if j > i and code[j - 1].text == '=':
                    d += 1
                    continue
                return None
            d += 1
        elif x == '}':
            if d == 0:
                return None
            d -= 1
        elif x == ';' and d == 0:
            return j
    return None


def is_decl(code, i):
    t = code[i].text
    if t in TYPE_WORDS and t not in ('const', 'volatile', 'static') or t in ('const', 'volatile', 'static', 'struct', 'typedef'):
        return t != 'typedef'
    return False


def split_decl(code, a, b):
    """TYPE d1, d2 = x, d3  ->  list of declarator ranges and the specifier range"""
    # specifiers: leading type words; declarators follow
    j = a
    while j < b and (code[j].text in TYPE_WORDS or code[j].text in ('struct', 'enum', 'union')):
        if code[j].text in ('struct', 'enum', 'union'):
            j += 2
            continue
        j += 1
    if j == a or j >= b:
        return None
    spec = (a, j)
    parts = []
    s = j
    d = 0
    for k in range(j, b):
        x = code[k].text
        if x in '([{':
            d += 1
        elif x in ')]}':
            d -= 1
        elif x == ',' and d == 0:
            parts.append((s, k))
            s = k + 1
    parts.append((s, b))
    if len(parts) < 2:
        return None
    # a function prototype (contains '(' in a declarator not after '=') is left alone
    for (p, q) in parts:
        for k in range(p, q):
            if code[k].text == '=':
                break
            if code[k].text == '(':
                return None
    return spec, parts


def transform(src, report):
    toks = tokenize(src)
    idx = code_index(toks)
    code = [toks[i] for i in idx]
    regions, decls, while1 = find_regions(code)
    ed = Edits()
    for (lo, hi) in regions:
        try:
            n = Parser(code, lo, hi).full()
        except Fail as e:
            report.append(f'  skipped: {" ".join(t.text for t in code[lo:hi])[:90]}  ({e})')
            continue
        walk(code, n, ed)
    for (lo, hi) in decls:
        # initializer expressions
        sp = split_decl(code, lo, hi)
        ranges = sp[1] if sp else [(lo, hi)]
        for (p, q) in ranges:
            eq = None
            d = 0
            for k in range(p, q):
                x = code[k].text
                if x in '([{':
                    d += 1
                elif x in ')]}':
                    d -= 1
                elif x == '=' and d == 0:
                    eq = k
                    break
            if eq is not None and eq + 1 < q and code[eq + 1].text != '{':
                try:
                    n = Parser(code, eq + 1, q).full()
                    walk(code, n, ed)
                except Fail as e:
                    report.append(f'  skipped init: {" ".join(t.text for t in code[eq+1:q])[:80]}  ({e})')

    # a swap whose operands also get brackets inside is left for later
    keep = []
    for (l, r) in ed.swaps:
        inside = any(l.a <= ci < r.b and not ((side == 'L' and ci == l.a) or (side == 'R' and ci == r.b - 1))
                     for (side, ci) in ed.ins)
        if inside:
            report.append('  == not swapped (brackets inside): ' +
                          ' '.join(t.text for t in code[l.a:r.b])[:80])
        else:
            keep.append((l, r))
    ed.swaps = keep

    # build the new token text list
    out = [t.text for t in toks]
    # swaps first (on code positions), done on text of whole operand ranges
    sw_text = {}
    for (l, r) in ed.swaps:
        lt = ''.join(t.text for t in toks[idx[l.a]:idx[l.b - 1] + 1])
        rt = ''.join(t.text for t in toks[idx[r.a]:idx[r.b - 1] + 1])
        sw_text[(idx[l.a], idx[l.b - 1])] = rt
        sw_text[(idx[r.a], idx[r.b - 1])] = lt
    # apply swaps: replace first token with the text, blank the rest
    for (a, b), txt in sw_text.items():
        out[a] = txt
        for k in range(a + 1, b + 1):
            out[k] = ''
    # brackets
    for (side, ci), texts in ed.ins.items():
        ti = idx[ci]
        if side == 'L':
            out[ti] = ''.join(texts) + out[ti]
    for (side, ci), texts in ed.ins.items():
        ti = idx[ci]
        if side == 'R':
            out[ti] = out[ti] + ''.join(texts)
    # while (1) -> for (;;)
    for ci in while1:
        out[idx[ci]] = 'for'
        out[idx[ci + 1]] = '('
        out[idx[ci + 2]] = ';;'
        # tokens between are whitespace inside the parens: keep
    # one declaration per statement
    for (lo, hi) in decls:
        sp = split_decl(code, lo, hi)
        if not sp:
            continue
        (sa, sb), parts = sp
        spec_txt = ''.join(t.text for t in toks[idx[sa]:idx[sb]])
        spec_txt = spec_txt.rstrip()
        # indentation of the statement
        line_start = src.rfind('\n', 0, sum(len(t.text) for t in toks[:idx[lo]])) + 1
        indent = re.match(r'[ \t]*', src[line_start:]).group(0)
        for n_part, (p, q) in enumerate(parts):
            if n_part == 0:
                continue
            # the comma before this declarator becomes ";\n<indent><spec> "
            comma_ti = idx[p - 1]
            out[comma_ti] = ';\n' + indent + spec_txt + ' '
            # drop whitespace right after the comma
            k = comma_ti + 1
            while k < idx[p] and toks[k].kind == 'ws':
                out[k] = ''
                k += 1
    if ed.swaps or ed.ins or while1 or decls:
        pass
    return ''.join(out), len(ed.swaps), len(ed.ins), len(while1)


def main():
    import glob
    add_typedefs(glob.glob('app_program/**/*.[ch]', recursive=True))
    total = [0, 0, 0]
    for f in sys.argv[1:]:
        raw = open(f, 'rb').read().decode('utf-8')
        crlf = '\r\n' in raw
        src = raw.replace('\r\n', '\n')
        report = []
        new, ns, ni, nw = transform(src, report)
        if crlf:
            new = new.replace('\n', '\r\n')
        if new.encode('utf-8') != raw.encode('utf-8'):
            open(f, 'wb').write(new.encode('utf-8'))
        total[0] += ns
        total[1] += ni
        total[2] += nw
        print(f'{f}: {ns} == swapped, {ni} brackets, {nw} while(1)')
        for r in report:
            print(r)
    print('total', total)


if __name__ == '__main__':
    main()

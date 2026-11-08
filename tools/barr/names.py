"""Variable names for Barr C rule 7.1, and a scope-aware renamer.

    python tools/barr/names.py list                 # every variable, its scope and type
    python tools/barr/names.py apply <renames.txt>  # rename

renames.txt, one per line ('#' starts a comment):
    type   OLD new                    a type name, every file, comments too
    func   OLD new                    a function name, every file, comments too
    global OLD new                    a variable shared between files
    file   <file> OLD new             a file-scope variable of one file
    local  <file> <function> OLD new  a parameter or local of one function

A local is renamed only inside its own function (parameter list and body,
and macro bodies defined there); a file-scope variable everywhere in its
file except functions that declare a local of the same name.  A name after
'.' or '->' is a structure member and is never renamed.  Comments: only
names with an underscore are renamed there (plain words like "pending" are
left alone).  Names do not change the program: tools/barr/compare.sh
proves it.
"""
import glob
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import transform  # noqa: E402


def functions(code):
    """(name, param_lo, param_hi, body_lo, body_hi) of each definition"""
    out = []
    depth = 0
    i = 0
    n = len(code)
    while i < n:
        t = code[i].text
        if t == '{':
            # a function body: ... IDENT ( params ) {   at depth 0
            if depth == 0 and i > 0 and code[i - 1].text == ')':
                close = i - 1
                d = 0
                j = close
                while j >= 0:
                    x = code[j].text
                    if x == ')':
                        d += 1
                    elif x == '(':
                        d -= 1
                        if d == 0:
                            break
                    j -= 1
                if j > 0 and code[j - 1].kind == 'ident':
                    body_end = transform.match(code, i)
                    out.append((code[j - 1].text, j + 1, close, i + 1, body_end))
                    i = body_end + 1
                    continue
            depth += 1
        elif t == '}':
            depth -= 1
        i += 1
    return out


def declarators(code, lo, hi):
    """names declared by the declaration code[lo:hi] (no ';'), with their
    pointer depth and whether they are arrays"""
    j = lo
    while j < hi and (code[j].text in transform.TYPE_WORDS or code[j].text in ('struct', 'enum', 'union')):
        if code[j].text in ('struct', 'enum', 'union'):
            j += 2
            continue
        j += 1
    spec = ' '.join(t.text for t in code[lo:j])
    parts = []
    s = j
    d = 0
    for k in range(j, hi):
        x = code[k].text
        if x in '([{':
            d += 1
        elif x in ')]}':
            d -= 1
        elif x == ',' and d == 0:
            parts.append((s, k))
            s = k + 1
    parts.append((s, hi))
    out = []
    for (p, q) in parts:
        k = p
        stars = 0
        while k < q and code[k].text in ('*', 'const', 'volatile'):
            stars += code[k].text == '*'
            k += 1
        if k < q and code[k].kind == 'ident':
            if k + 1 < q and code[k + 1].text == '(':
                continue                  # a function prototype
            arr = k + 1 < q and code[k + 1].text == '['
            out.append((code[k].text, k, spec, stars, arr))
    return out


def params(code, lo, hi):
    """names of a parameter list code[lo:hi]"""
    out = []
    s = lo
    d = 0
    for k in range(lo, hi + 1):
        x = code[k].text if k < hi else ','
        if x in '([':
            d += 1
        elif x in ')]':
            d -= 1
        elif x == ',' and d == 0:
            names = [i for i in range(s, k) if code[i].kind == 'ident' and code[i].text not in transform.TYPE_WORDS]
            if names:
                i = names[-1]
                spec = ' '.join(code[m].text for m in range(s, i) if code[m].text != '*')
                stars = sum(1 for m in range(s, i) if code[m].text == '*')
                arr = i + 1 < k and code[i + 1].text == '['
                out.append((code[i].text, i, spec, stars, arr))
            s = k + 1
    return out


def struct_ranges(code):
    """token ranges of struct / union / enum bodies"""
    out = []
    for i, t in enumerate(code):
        if t.text in ('struct', 'union', 'enum'):
            j = i + 1
            while j < len(code) and code[j].text != '{' and code[j].text not in (';', ')', ','):
                j += 1
            if j < len(code) and code[j].text == '{':
                out.append((j, transform.match(code, j)))
    return out


def inventory(f):
    src = open(f, encoding='utf-8').read()
    toks = transform.tokenize(src)
    idx = transform.code_index(toks)
    code = [toks[i] for i in idx]
    funcs = functions(code)
    structs = struct_ranges(code)
    rows = []

    def in_struct(k):
        return any(a < k < b for (a, b) in structs)

    def in_func(k):
        for fn in funcs:
            if fn[1] <= k <= fn[4]:
                return fn
        return None

    _, decls, _ = transform.find_regions(code)
    for (lo, hi) in decls:
        if in_struct(lo):
            continue
        fn = in_func(lo)
        for (name, k, spec, stars, arr) in declarators(code, lo, hi):
            scope = fn[0] if fn else '<file>'
            rows.append((f, scope, name, spec, stars, arr))
    for fn in funcs:
        for (name, k, spec, stars, arr) in params(code, fn[1], fn[2]):
            rows.append((f, fn[0], name, 'param ' + spec, stars, arr))
    return rows


def list_all():
    files = sorted(glob.glob('app_program/**/*.[ch]', recursive=True))
    transform.add_typedefs(files)
    for f in files:
        for (f_, scope, name, spec, stars, arr) in inventory(f):
            print(f'{f_}\t{scope}\t{name}\t{spec}\t{"*" * stars}\t{"[]" if arr else ""}')


# ---- apply -----------------------------------------------------------------

def load(path):
    rules = []
    for line in open(path, encoding='utf-8'):
        line = line.split('#', 1)[0].strip()
        if line:
            rules.append(line.split())
    return rules


def word_sub(text, mapping, comments):
    """rename whole words in a pp line or comment, never after . or ->"""
    def rep(m):
        w = m.group(1)
        if w in mapping and (not comments or '_' in w):
            return mapping[w]
        return w
    if comments:
        return re.sub(r'(?<![\w.])(?<!->)([A-Za-z_]\w*)', rep, text)
    # a preprocessor line: rename in its code only, never in its comments
    # or strings ("barcode sensor B" must not become "int8_t")
    parts = re.split(r'(/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\')',
                     text, flags=re.S)
    for i in range(0, len(parts), 2):
        parts[i] = re.sub(r'(?<![\w.])(?<!->)([A-Za-z_]\w*)', rep, parts[i])
    return ''.join(parts)


def apply(path):
    rules = load(path)
    files = sorted(glob.glob('app_program/**/*.[ch]', recursive=True))
    transform.add_typedefs(files)
    every = {}          # type / func / global: all files
    per_file = {}       # file -> {old: new}
    per_func = {}       # (file, func) -> {old: new}
    for r in rules:
        kind = r[0]
        if kind in ('type', 'func', 'global'):
            every[r[1]] = r[2]
        elif kind == 'file':
            per_file.setdefault(os.path.normpath(r[1]), {})[r[2]] = r[3]
        elif kind == 'local':
            per_func.setdefault((os.path.normpath(r[1]), r[2]), {})[r[3]] = r[4]
    changed = 0
    for f in files:
        key = os.path.normpath(f)
        src = open(f, encoding='utf-8').read()
        toks = transform.tokenize(src)
        idx = transform.code_index(toks)
        code = [toks[i] for i in idx]
        funcs = functions(code)
        fmap = dict(every)
        fmap.update(per_file.get(key, {}))
        # code index -> function
        owner = {}
        local_names = {}
        for fn in funcs:
            for k in range(fn[1], fn[4] + 1):
                owner[k] = fn[0]
            names = {p[0] for p in params(code, fn[1], fn[2])}
            _, decls, _ = transform.find_regions(code[fn[3]:fn[4]])
            for (lo, hi) in decls:
                for d in declarators(code, fn[3] + lo, fn[3] + hi):
                    names.add(d[0])
            local_names[fn[0]] = names
        pos_of = {ti: ci for ci, ti in enumerate(idx)}
        out = []
        for ti, t in enumerate(toks):
            text = t.text
            ci = pos_of.get(ti)
            # which function is this token in (for pp and comments too)
            if ci is None:
                nxt = next((c for c in range(ti, len(toks)) if c in pos_of), None)
                ci_near = pos_of[nxt] if nxt is not None else None
                fn = owner.get(ci_near)
            else:
                fn = owner.get(ci)
            m = dict(fmap)
            if fn is not None:
                for name in local_names.get(fn, ()):
                    if name in m and name not in per_func.get((key, fn), {}):
                        del m[name]       # a local of the same name hides it
                m.update(per_func.get((key, fn), {}))
            if t.kind == 'ident':
                prev = code[ci - 1].text if ci and ci > 0 else ''
                if text in m and prev not in ('.', '->'):
                    text = m[text]
            elif t.kind == 'pp':
                text = word_sub(text, m, comments=False)
            elif t.kind == 'comment':
                text = word_sub(text, m, comments=True)
            out.append(text)
        new = ''.join(out)
        if new != src:
            open(f, 'w', encoding='utf-8', newline='\n').write(new)
            changed += 1
    print(changed, 'files changed')


if __name__ == '__main__':
    if sys.argv[1] == 'list':
        list_all()
    else:
        apply(sys.argv[2])

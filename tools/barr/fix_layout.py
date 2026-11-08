"""Barr C layout rules that clang-format cannot apply, run after it.

    python tools/barr/fix_layout.py app_program/<file>.c [...]

  8.3a  the break of each case lines up with its case label
  3.3c  each file ends with an end-of-file comment and a blank line
  3.6a  lines end with LF only
Only whitespace and comments change.
"""
import re
import sys

EOF_MARK = '/*** end of file ***/'


def wrap_comment(text, indent=''):
    """text as a /* ... */ block of at most 80 columns"""
    words = text.split()
    lines = []
    cur = indent + '/*'
    for w in words:
        if len(cur) + 1 + len(w) > 77 and cur.strip() not in ('/*', '*'):
            lines.append(cur)
            cur = indent + ' *'
        cur += ' ' + w
    lines.append(cur + ' */')
    return lines


def move_define_comments(lines):
    """#define X 1 /* a comment that ... \\
                       goes on */          ->  a comment block above it

    A comment that starts on a #define line and runs onto the next lines
    needs backslashes (clang-format adds them), which Barr 2.1b forbids in
    comments.  Moved above the #define it needs none."""
    out = []
    i = 0
    while i < len(lines):
        line = lines[i]
        # run BEFORE clang-format: the comment simply goes on, no backslash
        m = re.match(r'^(#\s*define\s+\S.*?)\s*/\*(.*)$', line)
        if m and '*/' not in line and not line.rstrip().endswith('\\') \
                and m.group(1).count('(') == m.group(1).count(')'):
            head = m.group(1).rstrip()
            parts = [m.group(2)]
            j = i + 1
            ok = False
            while j < len(lines):
                nxt = lines[j]
                if '*/' in nxt:
                    before, after = nxt.split('*/', 1)
                    parts.append(before)
                    ok = after.strip() == ''
                    break
                parts.append(nxt)
                j += 1
            if ok:
                text = ' '.join(p.strip() for p in parts)
                out.extend(wrap_comment(text))
                out.append(head)
                i = j + 1
                continue
        out.append(line)
        i += 1
    return out


STMT_RE = re.compile(r'^( *)(if|for|while|switch|do)\b')


def block_end(lines, i, indent):
    """index of the line closing the statement that starts at line i
    (its last block, after any else / else if, or the do ... while line)"""
    j = i + 1
    while j < len(lines):
        ln = lines[j]
        if ln.startswith(' ' * indent + '}') and (len(ln) == indent + 1 or ln[indent + 1] in ' ;w'):
            # an else after it continues the statement
            k = j + 1
            if k < len(lines) and re.match(r'^' + ' ' * indent + r'else\b', lines[k]):
                j = k + 1
                continue
            return j
        if ln.strip() and not ln.startswith(' ' * (indent + 1)) and not ln.startswith(' ' * indent + '{') \
                and not ln.startswith(' ' * indent + '}') and not ln.startswith('#'):
            return None                     # no braces: not a block statement
        j += 1
    return None


def blank_lines(lines):
    """Barr 3.3b: a blank line before and after each if / for / while /
    switch / do statement (its comment lines above belong to it)"""
    out = list(lines)
    i = 0
    while i < len(out):
        ln = out[i]
        m = STMT_RE.match(ln)
        if not m or out[i].rstrip().endswith('\\') or (i > 0 and out[i - 1].rstrip().endswith('\\')):
            i += 1
            continue
        indent = len(m.group(1))
        if m.group(2) == 'while' and i > 0 and out[i - 1].strip().startswith('}'):
            i += 1
            continue                         # the while of a do ... while
        if i + 1 >= len(out) or out[i + 1].strip() not in ('{',) and not out[i].rstrip().endswith('{'):
            # a header on several lines: find its brace line
            k = i + 1
            while k < len(out) and out[k].strip() != '{' and not STMT_RE.match(out[k]):
                if out[k].strip().endswith(';'):
                    break
                k += 1
            if k >= len(out) or out[k].strip() != '{':
                i += 1
                continue
        end = block_end(out, i, indent)
        if end is None:
            i += 1
            continue
        # a do ... while (...) closing line that goes on to its ';'
        while end + 1 < len(out) and out[end].strip().startswith('}') and \
                'while' in out[end] and not out[end].rstrip().endswith(';'):
            end += 1
        # before: skip back over the comment lines that belong to it
        top = i
        while top > 0 and out[top - 1].strip().startswith(('/*', '*')) and \
                out[top - 1].startswith(' ' * indent) and not out[top - 1].startswith(' ' * (indent + 1) + '*/'):
            top -= 1
            if out[top].strip().startswith('/*'):
                break
        prev = out[top - 1].strip() if top > 0 else ''
        if prev not in ('', '{') and not prev.endswith(':') and not prev.startswith('#') and \
                not prev.endswith('\\'):
            out.insert(top, '')
            i += 1
            end += 1
        # after
        nxt = out[end + 1].strip() if end + 1 < len(out) else ''
        if nxt not in ('', '}') and not nxt.startswith(('}', 'break;', 'case ', 'default', '#', 'else')) and \
                not out[end].rstrip().endswith('\\'):
            out.insert(end + 1, '')
        i += 1
    return out


def pre(text):
    """before clang-format"""
    lines = text.replace('\r\n', '\n').split('\n')
    return '\n'.join(move_define_comments(lines))


def brace_comments(lines):
    """'{ /* text */' -> '{' and the comment on the next line (Barr 1.3b:
    the brace on a line of its own)"""
    out = []
    for ln in lines:
        m = re.match(r'^( *)\{ +(/\*.*\*/)\s*$', ln)
        if m:
            out.append(m.group(1) + '{')
            out.append(m.group(1) + '    ' + m.group(2))
        else:
            out.append(ln)
    return out


def fix(text):
    lines = text.replace('\r\n', '\n').split('\n')
    # tabs left inside comments (code has none after clang-format)
    lines = [ln.expandtabs(4) for ln in lines]
    lines = brace_comments(lines)
    lines = blank_lines(lines)
    out = []
    case_indent = None
    for line in lines:
        m = re.match(r'^( *)(case\b.*|default\s*:.*)$', line)
        if m:
            case_indent = len(m.group(1))
        b = re.match(r'^( *)break;\s*$', line)
        if b and case_indent is not None and len(b.group(1)) == case_indent + 4:
            line = ' ' * case_indent + 'break;'
        if re.match(r'^ *switch\b', line):
            case_indent = None
        out.append(line.rstrip())
    while out and out[-1] == '':
        out.pop()
    if not out or out[-1] != EOF_MARK:
        out += ['', EOF_MARK]
    return '\n'.join(out) + '\n\n'


def main():
    args = sys.argv[1:]
    step = fix
    if args and args[0] == '--pre':
        step = pre
        args = args[1:]
    for f in args:
        raw = open(f, 'rb').read().decode('utf-8')
        new = step(raw)
        if new != raw:
            open(f, 'wb').write(new.encode('utf-8'))


if __name__ == '__main__':
    main()

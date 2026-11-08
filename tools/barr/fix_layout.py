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


def pre(text):
    """before clang-format"""
    lines = text.replace('\r\n', '\n').split('\n')
    return '\n'.join(move_define_comments(lines))


def fix(text):
    lines = text.replace('\r\n', '\n').split('\n')
    # tabs left inside comments (code has none after clang-format)
    lines = [ln.expandtabs(4) for ln in lines]
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

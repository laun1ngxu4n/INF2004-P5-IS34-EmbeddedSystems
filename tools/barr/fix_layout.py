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


def fix(text):
    lines = text.replace('\r\n', '\n').split('\n')
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
    for f in sys.argv[1:]:
        raw = open(f, 'rb').read().decode('utf-8')
        new = fix(raw)
        if new != raw:
            open(f, 'wb').write(new.encode('utf-8'))


if __name__ == '__main__':
    main()

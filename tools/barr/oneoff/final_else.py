"""Barr C 8.2d: end every if ... else if ... chain with an else.

    python tools/barr/oneoff/final_else.py app_program/<file>.c [...]

Run on clang-formatted code (braces on their own lines).  The added else is
empty, with a comment, so the program does not change.
"""
import re
import sys


def add_final_else(lines):
    out = list(lines)
    i = 0
    added = 0
    while i < len(out):
        m = re.match(r'^( *)if\b', out[i])
        if not m or (i > 0 and re.match(r'^ *else\b', out[i - 1])) or out[i].rstrip().endswith('\\'):
            i += 1
            continue
        indent = m.group(1)
        # walk the chain: each body is '{' ... indent + '}'
        j = i
        had_elif = False
        last_close = None
        while True:
            k = j
            while k < len(out) and out[k] != indent + '{':
                k += 1
            if k >= len(out):
                break
            depth = 0
            close = None
            for n in range(k, len(out)):
                if out[n] == indent + '{':
                    depth += 1
                elif out[n] == indent + '}':
                    depth -= 1
                    if depth == 0:
                        close = n
                        break
            if close is None:
                break
            last_close = close
            nxt = out[close + 1] if close + 1 < len(out) else ''
            if re.match(r'^' + indent + r'else if\b', nxt):
                had_elif = True
                j = close + 1
                continue
            if re.match(r'^' + indent + r'else\b', nxt):
                last_close = None          # already ends with else
            break
        if had_elif and last_close is not None:
            out[last_close + 1:last_close + 1] = [
                indent + 'else',
                indent + '{',
                indent + '    /* nothing to do for any other case */',
                indent + '}',
            ]
            added += 1
        i += 1
    return out, added


def main():
    total = 0
    for f in sys.argv[1:]:
        text = open(f, encoding='utf-8').read()
        lines, n = add_final_else(text.split('\n'))
        if n:
            open(f, 'w', encoding='utf-8', newline='\n').write('\n'.join(lines))
            total += n
    print('else added:', total)


if __name__ == '__main__':
    main()

"""Barr C 2.2h: put the brief description of each undocumented function
(tools/barr/oneoff/doc_briefs.txt) above its definition as a Doxygen comment.

    python tools/barr/oneoff/add_briefs.py

Only functions that have no comment directly above them get one.  Comments
only: the program does not change.
"""
import glob
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    briefs = {}
    for line in open(os.path.join(HERE, 'doc_briefs.txt'), encoding='utf-8'):
        line = line.strip()
        if line and not line.startswith('#'):
            name, text = line.split(':', 1)
            briefs[name.strip()] = text.strip()
    added = 0
    for f in sorted(glob.glob('app_program/**/*.[ch]', recursive=True)):
        lines = open(f, encoding='utf-8').read().split('\n')
        out = []
        for i, ln in enumerate(lines):
            m = re.match(r'^(?:EXPORT|LOCAL|Inline|static)?\s*[\w\s\*]*?\b(\w+)\s*\([^;]*$', ln)
            if m and m.group(1) in briefs and not ln.startswith((' ', '#')):
                # a definition: its body follows (no ';' before the '{')
                k = i
                is_def = False
                while k < len(lines):
                    if lines[k].rstrip().endswith(';'):
                        break
                    if lines[k].strip() == '{':
                        is_def = True
                        break
                    k += 1
                prev = out[-1].strip() if out else ''
                if is_def and not prev.endswith('*/'):
                    text = briefs[m.group(1)]
                    words = text.split()
                    cur = '/**'
                    block = []
                    for w in words:
                        if len(cur) + 1 + len(w) > 76:
                            block.append(cur)
                            cur = ' *'
                        cur += ' ' + w
                    block.append(cur + ' */')
                    out.extend(block)
                    added += 1
            out.append(ln)
        new = '\n'.join(out)
        if new != '\n'.join(lines):
            open(f, 'w', encoding='utf-8', newline='\n').write(new)
    print('briefs added:', added)


if __name__ == '__main__':
    main()

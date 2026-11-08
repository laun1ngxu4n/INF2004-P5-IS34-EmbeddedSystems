"""Barr C 2.2h: comments Doxygen can read.

    python tools/barr/doxygen.py app_program/<file>.c [...]

- a /* comment directly above a function definition or prototype (no blank
  line between) becomes /** ... */, so Doxygen documents the function with it
- the comment block at the top of each file becomes an @file block

Comments only: the program does not change.
"""
import re
import sys

FUNC_START = re.compile(
    r'^(?:EXPORT|IMPORT|LOCAL|Inline|static|extern)?\s*'
    r'(?:const\s+|volatile\s+|unsigned\s+)*[A-Za-z_]\w*(?:\s*\*)*\s+\**\s*[a-z_]\w*\s*\(')


def convert(text, path):
    lines = text.split('\n')
    # file header: the first comment of the file
    if lines and lines[0].startswith('/*') and not lines[0].startswith('/**'):
        name = path.replace('\\', '/').split('/')[-1]
        body = lines[0][2:].strip()
        if body in ('', '*'):
            lines[0] = '/**'
        else:
            lines[0] = '/** ' + body
        # " *  name.c" as the first text line -> " * @file name.c"
        for k in range(1, min(len(lines), 4)):
            m = re.match(r'^ \*\s+' + re.escape(name) + r'\s*$', lines[k])
            if m:
                lines[k] = ' * @file    ' + name
                break
        else:
            lines.insert(1, ' * @file    ' + name)
    i = 0
    while i < len(lines):
        if FUNC_START.match(lines[i]) and not lines[i].rstrip().endswith('\\'):
            j = i - 1
            if j >= 0 and lines[j].rstrip().endswith('*/'):
                # walk up to the start of that comment
                k = j
                while k >= 0 and '/*' not in lines[k]:
                    k -= 1
                if k >= 0 and lines[k].lstrip().startswith('/*') and not lines[k].lstrip().startswith('/**'):
                    lines[k] = lines[k].replace('/*', '/**', 1)
        i += 1
    return '\n'.join(lines)


def main():
    for f in sys.argv[1:]:
        text = open(f, encoding='utf-8').read()
        new = convert(text, f)
        if new != text:
            open(f, 'w', encoding='utf-8', newline='\n').write(new)


if __name__ == '__main__':
    main()

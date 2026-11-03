import sys, re
p = sys.argv[1]; s = open(p, encoding='utf-8').read()
s = re.sub(r'title = \{\{The \$.?arphi\$ accrual failure detector\}\}', lambda m: r'title = {{The $\varphi$ accrual failure detector}}', s)
open(p, 'w', encoding='utf-8').write(s)

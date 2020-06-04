#! py -3

import sys
fin = sys.stdin
fout = sys.stdout

l = fin.readline()
while l != '':
    fout.write(' '.join(l.split(' ')[2:4]) + ' ')
    l = fin.readline()

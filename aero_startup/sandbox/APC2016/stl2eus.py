#!/usr/bin/env python

import sys
import re

f = open(sys.argv[1])
of = open(sys.argv[2], 'w')

pat_facet = re.compile('^facet')
pat_outerloop = re.compile('^outer loop')
pat_endloop = re.compile('^endloop')
pat_endfacet = re.compile('^endfacet')
pat_vertex = re.compile(
    '^vertex (?P<x>[\d\-.]*) (?P<y>[\d\-.]*) (?P<z>[\d\-.]*)')

line = f.readline()

print >> of, '(defun make_kiva_pod_faceset () (list\
(instance faceset :init :faces (list '
while line:
    mvertex = pat_vertex.match(line)
    if pat_outerloop.match(line):
        print >> of, '(instance face :init :vertices (list '
    elif mvertex:
        vx = float(mvertex.group('x')) * 1000.0
        vy = float(mvertex.group('y')) * 1000.0
        vz = float(mvertex.group('z')) * 1000.0
        print >> of, ('(float-vector '
               + str(vx) + ' ' + str(vy) + ' ' + str(vz) + ')')
    elif pat_endloop.match(line):
        print >> of, '))'
    line = f.readline()
print >> of, '))\
))'

f.close()
of.close()


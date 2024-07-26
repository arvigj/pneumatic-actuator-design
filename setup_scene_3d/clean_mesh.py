#!/usr/local/bin/python

import sys
import pathlib

import numpy
import meshio

in_path = pathlib.Path(sys.argv[1])

mesh = meshio.read(in_path)

points = mesh.points

point_map = {}

tets = mesh.cells[0].data

for tet in tets:
    for i, vi in enumerate(tet):
        if vi not in point_map:
            point_map[vi] = len(point_map)
        tet[i] = point_map[vi]

points = numpy.empty((len(point_map), 3))
for k, v in point_map.items():
    points[v] = mesh.points[k]

mesh.points = points
mesh.cells[0].data = tets

meshio.write(in_path.parent / "clean-mesh.msh", mesh, file_format="gmsh22")

import triangle as tr
import numpy as np
import matplotlib.pyplot as plt
import igl

vertices = [
    # outside
    [-2.2, 2],
    [2.2, 2],
    [2.2, -1.2],
    [0.8, -1.2],
    [0.8, 0],
    [-0.8, 0],
    [-0.8, -1.2],
    [-2.2, -1.2],
    # inside right
    [1.8, 1],
    [1.8, -0.8],
    [1.2, -0.8],
    [1.2, 1],
    # inside left
    [-1.8, 1],
    [-1.8, -0.8],
    [-1.2, -0.8],
    [-1.2, 1],
]

segments = [
    [0, 1],
    [1, 2],
    [2, 3],
    [3, 4],
    [4, 5],
    [5, 6],
    [6, 7],
    [7, 0],
    # inside right
    [8, 9],
    [9, 10],
    [10, 11],
    [11, 8],
    # inside left
    [12, 13],
    [13, 14],
    [14, 15],
    [15, 12],
]

holes = [
    [1.5, 0],
    [-1.5, 0],
]

data = dict(vertices=vertices, segments=segments, holes=holes)


walker = tr.triangulate(data, "qpa0.05")
tr.compare(plt, data, walker)
plt.savefig("walker_vis.png")
print(walker)

V = walker["vertices"]
V = np.pad(V, ((0, 0), (0, 1)))
F = walker["triangles"]

igl.write_obj("walker.obj", V, F)

import numpy as np
import meshio
import igl

mm = meshio.read("worm_long_clean.msh")
v = mm.points
t = mm.cells_dict["tetra"]
v, t, _, _ = igl.remove_unreferenced(v, t)
f = igl.boundary_facets(t)

meshio.write_points_cells("worm_long_clean.msh", v, {
                          "tetra": t}, file_format="gmsh")

c = igl.facet_components(f)
print(np.unique(c, return_counts=True))
for i in np.unique(c):
    print(i, len(np.unique(f[c==i, :].flatten())))

with open("surface_selections_long.txt", "w") as file_:
    for i in np.unique(c):
        sel = f[c == i]
        for j in range(sel.shape[0]):
            barycenter = 1/3 * v[sel[j], :].sum(axis=0)
            if (i == 0) and (barycenter[2] < -0.6):
                if barycenter[0] > 10:
                    file_.write(f"{20} {sel[j, 0]} {sel[j, 1]} {sel[j, 2]}\n")
                else:
                    file_.write(f"{21} {sel[j, 0]} {sel[j, 1]} {sel[j, 2]}\n")
            else:
                file_.write(f"{i+1} {sel[j, 0]} {sel[j, 1]} {sel[j, 2]}\n")

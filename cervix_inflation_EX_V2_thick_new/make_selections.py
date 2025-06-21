import igl
import numpy as np
import meshio
import math


def make_selections(volumetric_mesh_fname, CX_fname):
    mm = meshio.read(volumetric_mesh_fname)
    v = mm.points
    # Rescale back by 1000
    v *= 1000
    t = mm.cells_dict["tetra"]
    f = igl.boundary_facets(t)
    c = igl.facet_components(f)
    tet_barycenters = np.mean(v[t, :], axis=1)
    triangle_barycenters = np.mean(v[f, :], axis=1)

    # At the cervix, find: min y and x at min y (x lower bound), max x (x upper bound) and y at max x
    mesh_ori_CX = meshio.read(CX_fname)
    pts_ori_CX = mesh_ori_CX.points   # shape (N,3)
    idx_max_x_ori_CX = np.argmax(pts_ori_CX[:, 0])
    idx_min_y_ori_CX = np.argmin(pts_ori_CX[:, 1])
    p1_ori_CX = pts_ori_CX[idx_max_x_ori_CX]  # [x_max, y_at_xmax, z_at_xmax]
    p2_ori_CX = pts_ori_CX[idx_min_y_ori_CX]  # [x_at_ymin, y_min, z_at_ymin]
    min_y = p2_ori_CX[1]
    x_at_min_y = p2_ori_CX[0]
    max_x = p1_ori_CX[0]
    y_at_max_x = p1_ori_CX[1]
    print(min_y, x_at_min_y, max_x, y_at_max_x)
    lower_x = round(x_at_min_y - 0.5, 1)
    upper_x = math.ceil(max_x)
    print(lower_x, upper_x)
    slope = (max_x - x_at_min_y) / (y_at_max_x - min_y)
    print(slope)
    dirichlet_selection = (triangle_barycenters[:, 0] > lower_x) & \
        (triangle_barycenters[:, 0] < upper_x)
    # & \ ((triangle_barycenters[:, 0] - x_at_min_y) / (triangle_barycenters[:, 1] - min_y) >= slope)
    # print(dirichlet_selection)
    with open("multigrid_selection.txt", "w") as file_:
        # Select inner surface and label as 2
        for i, j, k in f[c == 1]:
            file_.write(f"2 {i} {j} {k}\n")

        # Write dirichlet boundary as 1
        for idx in np.where(dirichlet_selection)[0]:
            i, j, k = f[idx]
            file_.write(f"1 {i} {j} {k}\n")

        # Select outer surface and label as 3 (excluding Dirichlet)
        outer_surface_indices = np.where((c == 0) & ~dirichlet_selection)[0]
        for idx in outer_surface_indices:
            i, j, k = f[idx]
            file_.write(f"3 {i} {j} {k}\n")
    return np.unique(f[c == 1].flatten()).shape[0]


if __name__ == "__main__":
    make_selections("LORIP45V2_UTCX_Thick.msh", "LORIP45V2_CX_Thick.stl")

import numpy as np
import igl
import meshio
import subprocess
import os
import pathlib
import json
import numpy.linalg as la
import argparse
import re
import platform
import shutil

import cervix_inflation_EX_V2_thick_new.make_selections as cervix_inflation_functions


OPTIMIZATION_NAMES = [pathlib.Path(
    example).stem for example in os.listdir(os.path.join(os.path.dirname(os.path.realpath(__file__)), "configs"))]

REMESH_RELOAD_FUNCTIONS = {
    "cervix_inflation_EX_V2_thick_new": lambda fname: cervix_inflation_functions.make_selections(fname, "LORIP45V2_CX_Thick.stl")
}

if platform.system() == "Darwin":
    POLYFEM_BIN = "PolyFEM_bin"
    MMG_BIN = "mmg3d_O3"
    LINEAR_SOLVER = "Eigen::AccelerateLDLT"
elif platform.system() == "Linux":
    POLYFEM_BIN = "PolyFEM_bin"
    MMG_BIN = "mmg3d_O3"
    LINEAR_SOLVER = "Eigen::PardisoLDLT"
elif platform.system() == "Windows":
    POLYFEM_BIN = "PolyFEM_bin.exe"
    MMG_BIN = "mmg3d.exe"
    LINEAR_SOLVER = "Eigen::PardisoLDLT"
else:
    raise AssertionError(
        f"{platform.system()} is currently not supported.")

# def slim_smoothing(v, t, max_iter=50):
#     soft_p = 1e5
#     boundary_vertices = np.unique(igl.boundary_facets(t)[0].flatten())

#     def is_good_enough(new_v, tol=1e-12):
#         return la.norm(v[boundary_vertices, :] - new_v[boundary_vertices, :]) < tol
#     # boundary_constraints = np.zeros([boundary_vertices.size, 3])
#     boundary_constraints = v[boundary_vertices, :]
#     s = igl.SLIM(v.copy(), t, v.copy(), boundary_vertices,
#                  boundary_constraints, igl.SLIM_ENERGY_TYPE_SYMMETRIC_DIRICHLET, soft_p)

#     it = 0
#     while True:
#         s.solve(5)
#         if is_good_enough(s.vertices()):
#             break
#         if it > 50:
#             raise AssertionError("SLIM exceeded max iterations!")
#         it += 1

#     return s.vertices()


def interior_remeshing(v, t, base_path):
    meshio.write_points_cells(os.path.join(base_path, "before_remesh.msh"), v, {
                              "tetra": t}, file_format="gmsh22")

    remesh_args = [
        os.path.join(args.mmg_build_dir, MMG_BIN),
        "-nosurf", "-optim",
        "-in", os.path.join(base_path, "before_remesh.msh"),
        "-out", os.path.join(base_path, "after_remesh.msh")]
    # print(remesh_args)
    with open(os.path.join(base_path, "log"), "a") as file_:
        subprocess.run(remesh_args, stdout=file_)

    mm = meshio.read(os.path.join(base_path, "after_remesh.msh"))

    return mm.points, mm.cells_dict["tetra"]


def reload_control_from_log(log_file_contents, num_variables, state_json):
    control_vars = np.array(re.findall(
        r'(?<=Current pressure boundary )\d+|(?<=\[)(?:-?\d+(?:\.\d+)?(?:,\s*-?\d+(?:\.\d+)?)*)(?=\])', log_file_contents))
    control_vars = control_vars.reshape([-1, 2])
    # assert(control_vars.shape[0] % num_variables == 0)
    control_vars = control_vars[-num_variables:, :]

    for idx, p in enumerate(state_json["boundary_conditions"]["pressure_boundary"]):
        for c in control_vars:
            if p["id"] == int(c[0]):
                control = [float(s) for s in c[1].split(",")]
                control = [0] + control
                state_json["boundary_conditions"]["pressure_boundary"][idx]["value"] = control
                break

    return state_json


def load_from_vtu(base_path, vol_vtu_path, surf_vtu_path, volume_selection, interior_remesh=True):
    vol_mm = meshio.read(vol_vtu_path)
    surf_mm = meshio.read(surf_vtu_path)

    v = vol_mm.points
    t = vol_mm.cells_dict["tetra"]

    # Remove other body ids
    body_ids = vol_mm.point_data["body_ids"]
    new_t = []
    for t_ in t:
        if body_ids[t_[0]] == volume_selection:
            new_t.append(t_)
    t = np.array(new_t, dtype=int)[:, [1, 2, 3, 0]]

    [v, svi, svj, _] = igl.remove_duplicate_vertices(
        v, t, 1e-8)
    t = svj[t]
    v, t, _, _ = igl.remove_unreferenced(v, t)

    surf_v = surf_mm.points
    surf_f = surf_mm.cells_dict["triangle"]
    surf_body_ids = surf_mm.point_data["body_ids"]
    sidesets = surf_mm.point_data["sidesets"]

    # Just do remeshing as the only behavior
    if interior_remesh:
        v, t = interior_remeshing(v, t, base_path)

    boundary_indices = np.unique(igl.boundary_facets(t)[0].flatten())

    meshio.write_points_cells(os.path.join(base_path, "multigrid.msh"), v, {
                              "tetra": t}, file_format="gmsh")
    with open(os.path.join(base_path, "multigrid_selection.txt"), "w") as file_:
        for f_ in surf_f:
            assert (sidesets[f_[0]] == sidesets[f_[1]])
            assert (sidesets[f_[0]] == sidesets[f_[2]])
            assert (len(sidesets[f_[0]]) == 1)
            s = sidesets[f_[0]][0]
            if s > 1e5:
                continue

            assert (surf_body_ids[f_[0]] == surf_body_ids[f_[1]])
            assert (surf_body_ids[f_[0]] == surf_body_ids[f_[2]])
            b = surf_body_ids[f_[0]][0]
            if b != volume_selection:
                continue

            f0 = np.argmin(la.norm(v - surf_v[f_[0], :], axis=1))
            f1 = np.argmin(la.norm(v - surf_v[f_[1], :], axis=1))
            f2 = np.argmin(la.norm(v - surf_v[f_[2], :], axis=1))

            assert (f0 != f1)
            assert (f0 != f2)
            assert (f1 != f2)
            assert (f0 in boundary_indices)
            assert (f1 in boundary_indices)
            assert (f2 in boundary_indices)

            file_.write(f"{int(s)} {f0} {f1} {f2}\n")


def log_energy(base_path):
    with open(os.path.join(base_path, "total_energy"), "w") as file_:
        with open(os.path.join(base_path, "energy"), "r") as read_file:
            file_.write(read_file.read())
        file_.write("\n")


def cache_opt_files(base_path, num_control_pts, num_iters, multigrid_level):
    # last_iter = 0
    for i in range(0, num_iters+1):
        try:
            for postfix in [".vtu", "_surf.vtu", ".vtm"]:
                shutil.move(os.path.join(base_path, f"opt_state_0_iter_{i}{postfix}"), os.path.join(
                    base_path, f"opt_{multigrid_level}_{i}_{num_control_pts if (num_control_pts > 0) else 'full'}{postfix}"))
            try:
                for postfix in ["_surf_contact.vtu"]:
                    shutil.move(os.path.join(base_path, f"opt_state_0_iter_{i}{postfix}"), os.path.join(
                        base_path, f"opt_{multigrid_level}_{i}_{num_control_pts if (num_control_pts > 0) else 'full'}{postfix}"))
            except FileNotFoundError:
                pass # Simulation does not have contact
        except FileNotFoundError:
            # continue
            # break
            raise AssertionError("Multigrid level did not finish!")
        # last_iter = i
    return num_iters


def run_optimization_or_reload(
    state_dict,
    run_dict,
    opt_path,
    num_control_pts,
    num_iters=20,
    num_threads=32,
    multigrid_level=0,
    weights_adjust=None,
    control_variables=None,
    new_opt_vertex_count=None
):
    found_existing = True
    for i in range(0, num_iters+1):
        if not os.path.isfile(f"opt_{multigrid_level}_{i}_{list(num_control_pts.values())[0] if (list(num_control_pts.values())[0] > 0) else 'full'}.vtu"):
            found_existing = False
            break

    # if found_existing and control_variables is not None:
    #     raise AssertionError("Cannot reload existing optimized files with control optimization as log is overriden!")

    if found_existing:
        return num_iters

    with open(os.path.join(opt_path, "state.json"), "w") as file_:
        if control_variables is not None:
            with open(os.path.join(opt_path, "log"), "r") as log_file_:
                state_dict = reload_control_from_log(
                    log_file_.read(), control_variables, state_dict)
        json.dump(state_dict, file_, indent=2)
    with open(os.path.join(opt_path, "run.json"), "w") as file_:
        tmp_run = run_dict.copy()
        tmp_run["states"][0]["path"] = os.path.join(opt_path, "state.json")
        for k, v in num_control_pts.items():
            if v == -1:
                if (len(tmp_run["variable_to_simulation"][int(k)]["composition"]) > 1):
                    # Remove parametrization here and optimize on vertices
                    tmp_run["variable_to_simulation"][int(
                        k)]["composition"].pop()
                    tmp_run["parameters"][int(k)]["number"] = {
                        "surface_selection": tmp_run["variable_to_simulation"][int(k)]["surface_selection"],
                        "state": tmp_run["variable_to_simulation"][int(k)]["state"],
                        "exclude_boundary_nodes": True
                    }
            else:
                tmp_run["variable_to_simulation"][int(
                    k)]["composition"][1]["num_control_vertices"] = v
                if new_opt_vertex_count is not None:
                    tmp_run["variable_to_simulation"][int(
                        k)]["composition"][1]["num_vertices"] = new_opt_vertex_count
                tmp_run["parameters"][int(k)]["number"] = v * 6
        tmp_run["solver"]["nonlinear"]["max_iterations"] = num_iters

        if weights_adjust is not None:
            for k, v in weights_adjust.items():
                for f in range(len(tmp_run["functionals"])):
                    if tmp_run["functionals"][f]["print_energy"] == k:
                        tmp_run["functionals"][f]["weight"] = v
                        break

        json.dump(tmp_run, file_, indent=2)
    with open(os.path.join(opt_path, "log"), "a") as file_:
        polyfem_args = [
            os.path.join(args.polyfem_build_dir, POLYFEM_BIN),
            "--json", os.path.join(opt_path, "run.json"),
            "--log_level", "trace", "--ns"]
        if num_threads > 0:
            polyfem_args.extend(["--max_threads", str(num_threads)])

        print("Running command ---")
        print(" ".join(polyfem_args))
        subprocess.run(polyfem_args, stdout=file_)
        print("---")
    with open(os.path.join(opt_path, "energy"), "w") as energy_file:
        with open(os.path.join(opt_path, "log"), "r") as log_file:
            for line in log_file:
                if re.search(args.opt_algorithm, line) or re.search("Reached iteration limit", line):
                    energy_file.write(line)

    # log_energy(opt_path)
    return cache_opt_files(opt_path, list(num_control_pts.values())[0], num_iters, multigrid_level)


def get_num_iters(file_iters, idx):
    return file_iters[idx]


def do_tetwild_remesh(remesh_reload_function, ftetwild_build_dir, base_path):
    mesh_fname = os.path.join(base_path, "multigrid.msh")
    surf_mesh_fname = mesh_fname[:-4] + ".stl"
    mm = meshio.read(mesh_fname)
    v = mm.points
    t = mm.cells_dict["tetra"]
    f = igl.boundary_facets(t)[0]
    meshio.write_points_cells(surf_mesh_fname, v, {"triangle": f})
    if ftetwild_build_dir is None or ftetwild_build_dir == "":
        raise AssertionError(
            "Must supply a valid fTetWild build directory for remeshing!")
    subprocess.run([
        os.path.join(ftetwild_build_dir, "FloatTetwild_bin"),
        "-i", surf_mesh_fname,
        "-o", mesh_fname
    ], stdout=subprocess.DEVNULL)
    new_opt_vertex_count = remesh_reload_function(mesh_fname)
    return new_opt_vertex_count


def main(opt_example_dict):
    opt_path = args.opt_path
    base_path = opt_example_dict["base_path"]

    with open(os.path.join(base_path, opt_example_dict["state_path"]), "r") as file_:
        state = json.load(file_)
    with open(os.path.join(base_path, opt_example_dict["run_path"]), "r") as file_:
        run = json.load(file_)

    if "paraview" not in state["output"]:
        state["output"]["paraview"] = {}
    state["output"]["paraview"]["high_order_mesh"] = False
    state["output"]["paraview"]["surface"] = True
    state["output"]["paraview"]["volume"] = True
    state["output"]["paraview"]["vismesh_rel_area"] = 1e9
    if "options" not in state["output"]["paraview"]:
        state["output"]["paraview"]["options"] = {}
    state["output"]["paraview"]["options"]["body_ids"] = True
    if "advanced" not in state["output"]:
        state["output"]["advanced"] = {}
    state["output"]["advanced"]["save_time_sequence"] = False

    if "output" not in run:
        run["output"] = {}
    run["output"]["save_frequency"] = 1
    # run["output"]["solve_log_level"] = 1

    state["solver"]["linear"]["solver"] = LINEAR_SOLVER
    # state["solver"]["nonlinear"]["solver"] = [{"type": "Newton"}, {"type": "RegularizedNewton"}, {"type": "GradientDescent"}]
    state["solver"]["nonlinear"]["line_search"] = {"method": "RobustArmijo"}
    # state["solver"]["nonlinear"]["Newton"] = {"use_psd_projection": False, "use_psd_projection_in_regularized": False}
    run["solver"]["nonlinear"]["line_search"] = {"method": "Backtracking"}
    run["solver"]["nonlinear"]["solver"] = args.opt_algorithm
    run["solver"]["nonlinear"]["iterations_per_strategy"] = 2
    run["solver"]["nonlinear"]["StochasticADAM"] = {
        "erase_component_probability": 0.7}
    run["solver"]["nonlinear"]["StochasticGradientDescent"] = {
        "erase_component_probability": 0.7}

    # subprocess.run(["rm", os.path.join(opt_path, "log"), os.path.join(opt_path, "total_energy")],
    #                stderr=subprocess.DEVNULL)

    for fname in opt_example_dict["aux_files"]:
        shutil.copyfile(os.path.join(base_path, fname), os.path.join(opt_path, fname))

    num_control_pts = opt_example_dict["num_control_points"]

    i = {k: v[0] for k, v in num_control_pts.items()}
    for idx in range(len(state["geometry"])):
        orig_mesh = state["geometry"][idx]["mesh"]
        orig_selection = state["geometry"][idx]["surface_selection"]

        dst_mesh = orig_mesh
        dst_selection = orig_selection
        if (idx == opt_example_dict["opt_mesh_idx"]):
            dst_mesh = "multigrid.msh"
            dst_selection = "multigrid_selection.txt"
            state["geometry"][idx]["mesh"] = dst_mesh
            if (type(state["geometry"][idx]["surface_selection"]) == str):
                state["geometry"][idx]["surface_selection"] = dst_selection
        shutil.copyfile(os.path.join(base_path, orig_mesh), os.path.join(opt_path, dst_mesh))
        if type(state["geometry"][idx]["surface_selection"]) == str:
            shutil.copyfile(os.path.join(base_path, orig_selection), os.path.join(opt_path, dst_selection))

    num_iters = get_num_iters(opt_example_dict["num_iters"], 0)
    num_threads = opt_example_dict["threads"]
    weights_adjust = {k: v[0] for k, v in opt_example_dict["weights_adjust"].items(
    )} if "weights_adjust" in opt_example_dict else None
    control_variables = opt_example_dict["control_variables"] if "control_variables" in opt_example_dict else None

    max_iters = run_optimization_or_reload(
        state, run, opt_path, i, num_iters, num_threads, 0, weights_adjust, None, None)

    # If mesh is inferred from vtu, delete applied transformations
    for idx in range(len(state["geometry"])):
        if (idx == opt_example_dict["opt_mesh_idx"]):
            if "transformation" in state["geometry"][idx]:
                del state["geometry"][idx]["transformation"]
            if type(state["geometry"][idx]["surface_selection"]) == list:
                state["geometry"][idx]["surface_selection"] = "multigrid_selection.txt"

    for idx in range(1, len(num_control_pts["0"])):
        last_num_control_pts = list(num_control_pts.values())[0][idx-1]
        volume_mesh_fname = f"opt_{idx-1}_{max_iters}_{last_num_control_pts if (last_num_control_pts > 0) else 'full'}.vtu"
        surface_mesh_fname = f"opt_{idx-1}_{max_iters}_{last_num_control_pts if (last_num_control_pts > 0) else 'full'}_surf.vtu"

        i = {k: v[idx] for k, v in num_control_pts.items()}
        num_iters = get_num_iters(opt_example_dict["num_iters"], idx)
        weights_adjust = {k: v[idx] for k, v in opt_example_dict["weights_adjust"].items(
        )} if "weights_adjust" in opt_example_dict else None
        load_from_vtu(opt_path,
                      os.path.join(opt_path, volume_mesh_fname),
                      os.path.join(opt_path, surface_mesh_fname),
                      state["geometry"][opt_example_dict["opt_mesh_idx"]]["volume_selection"])
        new_opt_vertex_count = None
        if "remesh_reload_function" in opt_example_dict:
            new_opt_vertex_count = do_tetwild_remesh(
                opt_example_dict["remesh_reload_function"],
                args.ftetwild_build_dir,
                opt_path
            )

        max_iters = run_optimization_or_reload(
            state,
            run,
            opt_path,
            i,
            num_iters,
            num_threads,
            idx,
            weights_adjust,
            control_variables,
            new_opt_vertex_count)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    ######################## REQUIRED ########################
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--opt_example",
                       type=str,
                       choices=OPTIMIZATION_NAMES,
                       help="Optimization example to run. Must be in the configs/ folder.")
    group.add_argument("--opt_json",
                       type=str,
                       help="Path to the optimization configuration json.")
    parser.add_argument("--polyfem_build_dir",
                        type=str,
                        required=True,
                        help="Path to PolyFEM binary.")
    parser.add_argument("--mmg_build_dir",
                        type=str,
                        required=True,
                        help="Path to MMG 3D binary.")
    ######################## NOT REQUIRED / SPECIAL CASES ########################
    parser.add_argument("--ftetwild_build_dir",
                        type=str,
                        required=False,
                        help="Path to fTetWild binary. Only needed if total body remeshing is done, by specifying a boundary selection reload function in REMESH_RELOAD_FUNCTIONS.")
    # parser.add_argument("--absolute_path",
    #                     type=str,
    #                     required=False,
    #                     default=os.path.dirname(os.path.realpath(__file__)),
    #                     help="What is the base path of the data directory, should end in 'pneumatic-actuator-design'. This should really only be changed for special cases (HPC, etc).")
    parser.add_argument("--opt_path",
                        type=str,
                        default=os.getcwd(),
                        required=False,
                        help="Where do you want the optimization files to be saved to? The default is the CWD.")
    parser.add_argument("--opt_algorithm",
                        type=str,
                        help="Which optimization algorithm to run?",
                        choices=["L-BFGS",
                                 "GradientDescent",
                                 "ADAM",
                                 "StochasticADAM",
                                 "StochasticGradientDescent",
                                 "BFGS"],
                        required=False,
                        default="L-BFGS")
    args = parser.parse_args()

    absolute_path = None
    opt_config = None
    if args.opt_example:
        absolute_path = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(absolute_path, "configs", args.opt_example), "r") as f:
            opt_config = json.load(f)
        opt_config["base_path"] = os.path.join(
            absolute_path, opt_config["base_path"])
    elif args.opt_json:
        absolute_path = os.path.dirname(args.opt_json)
        with open(args.opt_json, "r") as f:
            opt_config = json.load(f)
        opt_config["base_path"] = absolute_path
    else:
        raise AssertionError()

    if args.opt_example in REMESH_RELOAD_FUNCTIONS:
        opt_config["remesh_reload_function"] = REMESH_RELOAD_FUNCTIONS[args.opt_example]

    main(opt_config)

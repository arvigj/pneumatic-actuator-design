import numpy as np
import igl
import meshio
import subprocess
import os
import json
import numpy.linalg as la
import argparse
import re


optimizations = {
    "finger": {
        "base_path": "finger",
        "state_path": "state_finger.json",
        "run_path": "run_finger.json",
        "num_control_points": {
            "0": [4, 10, 20, 40, 80, 160, 320, 640, -1, -1, -1, -1, -1]
        },
        "num_iters": [5, 5, 5, 5, 5, 5, 10, 10, 20, 20, 20, 20, 20],
        "aux_files": ["finger_tip_target.obj", "finger_target.obj", "finger_target_larger.obj", "finger_target_v2.obj", "finger_target_v2_larger.obj"],
        "opt_mesh_idx": 0,
        "threads": 16
    },
    "frog_quasistatic": {
        "base_path": "frog",
        "state_path": "state_frog_quasistatic.json",
        "run_path": "run_frog_quasistatic.json",
        "num_control_points": {
            "0": [1, 10, 20, 40, 80, 160, 320, 640, 728, 728, 728, 728]
        },
        "num_iters": [5, 5, 5, 5, 5, 5, 10, 10, 10, 10, 10, 10],
        "aux_files": ["frog_selection_10.obj", "frog_selection_11.obj", "frog_selection_12.obj", "frog_neck_target.obj"],
        "opt_mesh_idx": 0,
        "threads": 16
    },
    "frog_quasistatic_base": {
        "base_path": "frog",
        "state_path": "state_frog_quasistatic_vertices.json",
        "run_path": "run_frog_quasistatic_vertices.json",
        "num_control_points": {
            "0": [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
        },
        "num_iters": [5, 5, 5, 5, 5, 5, 10, 10, 10, 10, 10, 10],
        "aux_files": ["frog_selection_10.obj", "frog_selection_11.obj", "frog_selection_12.obj", "frog_neck_target.obj"],
        "opt_mesh_idx": 0,
        "threads": 16
    },
    "frog_quasistatic_base_weights_adjust": {
        "base_path": "frog",
        "state_path": "state_frog_quasistatic_vertices.json",
        "run_path": "run_frog_quasistatic_vertices.json",
        "num_control_points": {
            "0": [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
        },
        "num_iters": [5, 5, 5, 5, 5, 5, 10, 10, 10, 10, 10, 10],
        "weights_adjust": {"boundary_smoothing": [2.8, 2.8, 2.8, 2.8, 1.4, 1.4, 1.4, 1.4, 0.7, 0.7, 0.7, 0.7]},
        "aux_files": ["frog_selection_10.obj", "frog_selection_11.obj", "frog_selection_12.obj", "frog_neck_target.obj"],
        "opt_mesh_idx": 0,
        "threads": 16
    },
        "gripper_bellows_log_inside": {
        "base_path": "gripper_bellows",
        "state_path": "state_gripper_bellows.json",
        "run_path": "run_gripper_bellows_log_inside.json",
        "num_control_points": {
            "0": [5, 20, 80, 320, 640, 1280, 1280, 1280, 1280],
        },
        "num_iters": [5, 5, 5, 5, 5, 10, 10, 10, 20],
        "aux_files": [
            "cylinder_smaller_target_v2.obj",
            "intermediate_target_5.obj",
            "intermediate_target_10.obj",
            "intermediate_target_30.obj"
        ],
        "opt_mesh_idx": 0,
        "threads": 32
    },
    "gripper_bellows_quadratic_inside": {
        "base_path": "gripper_bellows",
        "state_path": "state_gripper_bellows.json",
        "run_path": "run_gripper_bellows_quadratic_inside.json",
        "num_control_points": {
            "0": [5, 20, 80, 320, 640, 1280, 1280, 1280, 1280],
        },
        "num_iters": [5, 5, 5, 5, 5, 10, 10, 10, 20],
        "aux_files": [
            "cylinder_smaller_target_v2.obj",
            "intermediate_target_5.obj",
            "intermediate_target_10.obj",
            "intermediate_target_30.obj"
        ],
        "opt_mesh_idx": 0,
        "threads": 32
    },
        "gripper_bellows_shape_inside": {
        "base_path": "gripper_bellows",
        "state_path": "state_gripper_bellows.json",
        "run_path": "run_gripper_bellows_shape_inside.json",
        "num_control_points": {
            "0": [5, 20, 80, 320, 640, 1280, 1280, 1280, 1280],
        },
        "num_iters": [5, 5, 5, 5, 5, 10, 10, 10, 20],
        "aux_files": [
            "cylinder_smaller_target_v2.obj",
            "intermediate_target_5.obj",
            "intermediate_target_10.obj",
            "intermediate_target_30.obj"
        ],
        "opt_mesh_idx": 0,
        "threads": 32
    },
    "worm": {
        "base_path":  "worm",
        "state_path": "state_worm.json",
        "run_path": "run_worm.json",
        "num_control_points": {
            "0": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "1": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "2": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "3": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "4": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "5": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "6": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "7": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "8": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "9": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "10": [10, 60, 120, 240, 480, 960, 1280, 1280, 1280]
        },
        "num_iters": [3, 5, 5, 5, 5, 10, 10, 10, 10],
        "aux_files": [],
        "opt_mesh_idx": 0,
        "threads": 32
    },
        "worm_control": {
        "base_path":  "worm_control",
        "state_path": "state_worm.json",
        "run_path": "run_worm.json",
        "num_control_points": {
            "0": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "1": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "2": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "3": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "4": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "5": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "6": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "7": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "8": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "9": [1, 6, 12, 24, 48, -1, -1, -1, -1],
            "10": [10, 60, 120, 240, 480, 960, 1280, 1280, 1280]
        },
        "num_iters": [3, 5, 5, 5, 5, 10, 10, 10, 10],
        "control_variables": 10,
        "aux_files": [],
        "opt_mesh_idx": 0,
        "threads": 16
    }
}


def slim_smoothing(v, t, max_iter=50):
    soft_p = 1e5
    boundary_vertices = np.unique(igl.boundary_facets(t).flatten())

    def is_good_enough(new_v, tol=1e-12):
        return la.norm(v[boundary_vertices, :] - new_v[boundary_vertices, :]) < tol
    # boundary_constraints = np.zeros([boundary_vertices.size, 3])
    boundary_constraints = v[boundary_vertices, :]
    s = igl.SLIM(v.copy(), t, v.copy(), boundary_vertices,
                 boundary_constraints, igl.SLIM_ENERGY_TYPE_SYMMETRIC_DIRICHLET, soft_p)

    it = 0
    while True:
        s.solve(5)
        if is_good_enough(s.vertices()):
            break
        if it > 50:
            raise AssertionError("SLIM exceeded max iterations!")
        it += 1

    return s.vertices()


def interior_remeshing(v, t, base_path):
    meshio.write_points_cells(os.path.join(base_path, "before_remesh.msh"), v, {
                              "tetra": t}, file_format="gmsh22")

    remesh_args = [
        os.path.join(args.mmg_build_dir, "mmg3d_O3"),
        "-nosurf", "-optim",
        "-in", os.path.join(base_path, "before_remesh.msh"),
        "-out", os.path.join(base_path, "after_remesh.msh")]
    # print(remesh_args)
    with open(os.path.join(base_path, "log"), "a") as file_:
        subprocess.run(remesh_args, stdout=file_)

    mm = meshio.read(os.path.join(base_path, "after_remesh.msh"))

    return mm.points, mm.cells_dict["tetra"]


def reload_control_from_log(log_file_contents, num_variables, state_json):
    control_vars = np.array(re.findall('(?<=Current pressure boundary )\d+|(?<=\[)(?:-?\d+(?:\.\d+)?(?:,\s*-?\d+(?:\.\d+)?)*)(?=\])', log_file_contents))
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
    f = igl.boundary_facets(t)

    surf_v = surf_mm.points
    surf_f = surf_mm.cells_dict["triangle"]
    surf_body_ids = surf_mm.point_data["body_ids"]
    sidesets = surf_mm.point_data["sidesets"]

    # Just do remeshing as the only behavior
    if interior_remesh:
        v, t = interior_remeshing(v, t, base_path)

    boundary_indices = np.unique(igl.boundary_facets(t).flatten())

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
        subprocess.run(["cat", os.path.join(
            base_path, "energy")], stdout=file_)
        file_.write("\n\n")


def cache_opt_files(base_path, num_control_pts, num_iters, multigrid_level):
    # last_iter = 0
    for i in range(0, num_iters+1):
    # for i in range(0, num_iters):
        try:
            for postfix in [".vtu", "_surf.vtu", ".vtm", ".obj"]:
                subprocess.run(["mv", os.path.join(base_path, f"opt_state_0_iter_{i}{postfix}"), os.path.join(
                    base_path, f"opt_{multigrid_level}_{i}_{num_control_pts if (num_control_pts > 0) else 'full'}{postfix}")], check=True)
            for postfix in ["_surf_contact.vtu"]:
                subprocess.run(["mv", os.path.join(base_path, f"opt_state_0_iter_{i}{postfix}"), os.path.join(
                    base_path, f"opt_{multigrid_level}_{i}_{num_control_pts if (num_control_pts > 0) else 'full'}{postfix}")], check=False)
        except subprocess.CalledProcessError:
            # continue
            # break
            raise AssertionError("Multigrid level did not finish!")
        # last_iter = i
    return num_iters


def run_optimization_or_reload(state_dict, run_dict, opt_path, num_control_pts, num_iters=20, num_threads=32, multigrid_level=0, weights_adjust=None, control_variables=None):
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
                state_dict = reload_control_from_log(log_file_.read(), control_variables, state_dict)
        json.dump(state_dict, file_, indent=2)
    with open(os.path.join(opt_path, "run.json"), "w") as file_:
        tmp_run = run_dict.copy()
        tmp_run["states"][0]["path"] = os.path.join(opt_path, "state.json")
        for k, v in num_control_pts.items():
            if v == -1:
                if (len(tmp_run["variable_to_simulation"][int(k)]["composition"]) > 1):
                    # Remove parametrization here and optimize on vertices
                    tmp_run["variable_to_simulation"][int(k)]["composition"].pop()
                    tmp_run["parameters"][int(k)]["number"] = {
                        "surface_selection": tmp_run["variable_to_simulation"][int(k)]["surface_selection"],
                        "state": tmp_run["variable_to_simulation"][int(k)]["state"],
                        "exclude_boundary_nodes": True
                    }
            else:
                tmp_run["variable_to_simulation"][int(
                    k)]["composition"][1]["num_control_vertices"] = v
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
            os.path.join(args.polyfem_build_dir, "PolyFEM_bin"),
            "--json", os.path.join(opt_path, "run.json"),
            "--log_level", "trace", "--ns"]
        if num_threads > 0:
            polyfem_args.extend(["--max_threads", str(num_threads)])

        subprocess.run(polyfem_args, stdout=file_)
    with open(os.path.join(opt_path, "energy"), "w") as energy_file:
        subprocess.run(["grep", "-e", args.opt_algorithm, "-e", '"Reached iteration limit"', os.path.join(opt_path, "log")], stdout=energy_file)

    # log_energy(opt_path)
    return cache_opt_files(opt_path, list(num_control_pts.values())[0], num_iters, multigrid_level)


def get_num_iters(file_iters, idx):
    return file_iters[idx]


def main():
    opt_example_dict = optimizations[args.opt_example]
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

    state["solver"]["nonlinear"]["solver"] = [{"type": "Newton"}, {"type": "RegularizedNewton"}, {"type": "GradientDescent"}]
    state["solver"]["nonlinear"]["line_search"] = {"method": "RobustArmijo"}
    state["solver"]["nonlinear"]["Newton"] = {"use_psd_projection": False, "use_psd_projection_in_regularized": False}
    run["solver"]["nonlinear"]["line_search"] = {"method": "Backtracking"}
    run["solver"]["nonlinear"]["solver"] = args.opt_algorithm
    run["solver"]["nonlinear"]["iterations_per_strategy"] = 2
    run["solver"]["nonlinear"]["StochasticADAM"] = {"erase_component_probability": 0.7}
    run["solver"]["nonlinear"]["StochasticGradientDescent"] = {"erase_component_probability": 0.7}

    # subprocess.run(["rm", os.path.join(opt_path, "log"), os.path.join(opt_path, "total_energy")],
    #                stderr=subprocess.DEVNULL)

    for fname in opt_example_dict["aux_files"]:
        subprocess.run(
            ["cp", os.path.join(base_path, fname), os.path.join(opt_path, fname)])

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
        subprocess.run(
            ["cp", os.path.join(base_path, orig_mesh), os.path.join(opt_path, dst_mesh)])
        if type(state["geometry"][idx]["surface_selection"]) == str:
            subprocess.run(["cp", os.path.join(
                base_path, orig_selection), os.path.join(opt_path, dst_selection)])

    num_iters = get_num_iters(opt_example_dict["num_iters"], 0)
    num_threads = opt_example_dict["threads"]
    weights_adjust = {k: v[0] for k, v in opt_example_dict["weights_adjust"].items()} if "weights_adjust" in opt_example_dict else None
    control_variables = opt_example_dict["control_variables"] if "control_variables" in opt_example_dict else None

    max_iters = run_optimization_or_reload(state, run, opt_path, i, num_iters, num_threads, 0, weights_adjust, None)

    # If mesh is inferred from vtu, delete applied transformations
    for idx in range(len(state["geometry"])):
        if (idx == opt_example_dict["opt_mesh_idx"]):
            del state["geometry"][idx]["transformation"]
            if type(state["geometry"][idx]["surface_selection"]) == list:
                state["geometry"][idx]["surface_selection"] = "multigrid_selection.txt"

    for idx in range(1, len(num_control_pts["0"])):
        i = {k: v[idx] for k, v in num_control_pts.items()}
        num_iters = get_num_iters(opt_example_dict["num_iters"], idx)
        weights_adjust = {k: v[idx] for k, v in opt_example_dict["weights_adjust"].items()} if "weights_adjust" in opt_example_dict else None
        last_num_control_pts = list(num_control_pts.values())[0][idx-1]
        load_from_vtu(opt_path, os.path.join(
            opt_path, f"opt_{idx-1}_{max_iters}_{last_num_control_pts if (last_num_control_pts > 0) else 'full'}.vtu"),
            os.path.join(
            opt_path, f"opt_{idx-1}_{max_iters}_{last_num_control_pts if (last_num_control_pts > 0) else 'full'}_surf.vtu"),
            state["geometry"][opt_example_dict["opt_mesh_idx"]]["volume_selection"])
        max_iters = run_optimization_or_reload(state, run, opt_path, i, num_iters, num_threads, idx, weights_adjust, control_variables)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("opt_example", help="", type=str,
                        choices=list(optimizations.keys()))
    parser.add_argument("opt_path", help="", type=str)
    parser.add_argument("polyfem_build_dir", help="", type=str)
    parser.add_argument("mmg_build_dir", help="", type=str)
    parser.add_argument("absolute_path",
                        type=str,
                        help="What is the base path of the data directory, should end in 'pneumatic-actuator-design'")
    parser.add_argument("opt_algorithm",
                        type=str,
                        help="Which optimization algorithm to run?",
                        choices=["L-BFGS",
                            "GradientDescent",
                            "ADAM",
                            "StochasticADAM",
                            "StochasticGradientDescent",
                            "BFGS"])
    args = parser.parse_args()

    absolute_path = args.absolute_path

    for k, v in optimizations.items():
        v["base_path"] = os.path.join(absolute_path, v["base_path"])

    main()




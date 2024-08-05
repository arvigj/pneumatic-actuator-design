import numpy as np
import igl
import meshio
import subprocess
import os
import json
import numpy.linalg as la
import argparse

import cascaded_optimization


def main(example_dict, path):
    s = 0
    for idx, i in enumerate(example_dict["num_control_points"][list(example_dict["num_control_points"])[0]]):
        if i == -1:
            i = "full"
        for j in range(example_dict["num_iters"][idx]):
            try:
                for suffix in [".vtu", "_surf.vtu", ".obj"]:
                    subprocess.run(["cp", os.path.join(path, f"opt_{idx}_{j}_{i}{suffix}"), os.path.join(path, f"opt_sequential_{s}{suffix}")], check=True)
                for suffix in ["_surf_contact.vtu"]:
                    subprocess.run(["cp", os.path.join(path, f"opt_{idx}_{j}_{i}{suffix}"), os.path.join(path, f"opt_sequential_{s}{suffix}")], check=False)
                s += 1
            except Exception as e:
                print(e)
                pass
    i = 0
    while True:
        try:
            for suffix in [".vtu", "_surf.vtu", ".obj"]:
                subprocess.run(["cp", os.path.join(path, f"opt_state_0_iter_{i}{suffix}"), os.path.join(path, f"opt_sequential_{s}{suffix}")], check=True)
            for suffix in ["_surf_contact.vtu"]:
                subprocess.run(["cp", os.path.join(path, f"opt_state_0_iter_{i}{suffix}"), os.path.join(path, f"opt_sequential_{s}{suffix}")], check=False)
            i += 1
            s += 1
        except Exception as e:
            break


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--opt_example", help="", type=str,
                        choices=list(cascaded_optimization.OPTIMIZATIONS.keys()))
    parser.add_argument("--opt_path", default=os.getcwd(), help="", type=str)
    args = parser.parse_args()

    main(cascaded_optimization.OPTIMIZATIONS[args.opt_example], args.opt_path)




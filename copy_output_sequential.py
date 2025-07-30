import numpy as np
import os, sys
import argparse
import shutil
import re

def list_sorted_files(path, suffix):
    files = [fname for fname in os.listdir(path) if re.match(f"opt_[0-9]+_[0-9]+_[0-9]+{suffix}", fname)]
    files.extend([fname for fname in os.listdir(path) if re.match(f"opt_state_[0-9]+_iter_[0-9]+{suffix}", fname)])
    sorted_files = sorted([fname for fname in files if ("opt_" in fname)])

    return sorted_files

def copy_files_ordered(path, suffix):
    sorted_files = list_sorted_files(path, suffix)
    for idx, fname in enumerate(sorted_files):
        shutil.copyfile(fname, os.path.join(path, f"opt_sequential_{idx}{suffix}"))

def copy_output_to_ordered(path):
    for suffix in [".vtu", "_surf.vtu"]:
        copy_files_ordered(path, suffix)
    
    for suffix in ["_surf_contact.vtu"]:
        try:
            copy_files_ordered(path, suffix)
        except:
            pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--opt_path", default=os.getcwd(), help="", type=str)
    args = parser.parse_args()

    copy_output_to_ordered(args.opt_path)

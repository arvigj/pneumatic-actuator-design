# [Siggraph 2024] Soft Pneumatic Actuator Design using Differentiable Simulation 

This repository contains the data and configuration files for our work ["Soft Pneumatic Actuator Design using Differentiable Simulation"](https://cims.nyu.edu/gcl/papers/2024-pneumatic.pdf). For supplementary material, please see [here](https://cims.nyu.edu/gcl/papers/2024-pneumatic-supplemental.pdf). The main implementation for the paper is contributed to [PolyFEM](https://github.com/polyfem/polyfem), while the cascaded optimization is handled by the python script `cascaded_optimization.py`.

## How to run the existing simulations
First, build [PolyFEM](https://github.com/polyfem/polyfem) and download (or build) [MMG](https://github.com/MmgTools/mmg), noting down the directories where the binaries are located. Next, navigate to the directory where you want the optimization files to be saved to. You might want to do this in a tmux window as the optimization can take a long time. Then, run the following command:

```
python /path/to/pneumatic-actuator-design/cascaded_optimization.py --opt_example EXAMPLE_NAME --polyfem_build_dir /path/to/polyfem/ --mmg_build_dir /path/to/mmg/
```

where `EXAMPLE_NAME` is one of 
* frog_quasistatic
* finger
* gripper_bellows
* worm

For some of the ablation studies, you can run
* gripper_bellows_shape_inside (gripper optimization using only target matching, no contact force maximization)
* frog_base (optimizatation of frog in vertex space)
* frog_quasistatic_base_weights_adjust (same as above, but starting with 4x the smoothing weights and decreasing over the optimization)

The 2D walker example is a simple optimization, so it can just be run by

```
/path/to/PolyFEM_bin --json /path/to/pneumatic-actuator-design/walker/run_walker.json --max_threads 8 
```

## How to setup new optimizations
If you'd like to setup a new optimization, please see our tutorial [here](http://www.arvigjoka.com/blog/2024/pneumatic-actuator-optimization/) (draft in progress).

## Data Availability
Due to copyright limitations, we cannot distribute the models for the following files:
```
frog/frog_base_mesh.msh
finger/finger_base_mesh.msh
finger/finger_target_larger.obj
```

The base models for these files can be found at:
```
frog: https://www.turbosquid.com/FullPreview/1508446
finger: https://www.turbosquid.com/FullPreview/1103552
```
If you acquire these licenses, we can release the aforementioned models for the optimization via email: `arvi.gjoka@nyu.edu`.

# Citation
If you use this work/data, please cite our paper:
```
@inproceedings{10.1145/3641519.3657467,
author = {Gjoka, Arvi and Knoop, Espen and B\"{a}cher, Moritz and Zorin, Denis and Panozzo, Daniele},
title = {Soft Pneumatic Actuator Design using Differentiable Simulation},
year = {2024},
isbn = {9798400705250},
publisher = {Association for Computing Machinery},
address = {New York, NY, USA},
url = {https://doi.org/10.1145/3641519.3657467},
doi = {10.1145/3641519.3657467},
abstract = {We propose a computational design pipeline for pneumatically-actuated soft robots interacting with their environment through contact. We optimize the shape of the robot with a shape optimization approach, using a physically-accurate high-order finite element model for the forward simulation. Our approach enables fine-grained control over both deformation and contact forces by optimizing the shape of internal cavities, which we exploit to design pneumatically-actuated robots that can assume user-prescribed poses, or apply user-controlled forces. We demonstrate the efficacy of our method on two artistic and two functional examples.},
booktitle = {Special Interest Group on Computer Graphics and Interactive Techniques Conference Conference Papers '24},
articleno = {106},
numpages = {11},
keywords = {Differentiable Simulation, Finite Element Method, Pneumatic Actuator, Shape Optimization, Soft Robotics},
location = {Denver, CO, USA},
series = {SIGGRAPH Conference Papers '24}
}
```

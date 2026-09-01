<div align="center">
  <img src="src/nezha_sim/nezha_gazebo/mesh/logo.gif" width="220" alt="NezhaSim"/>

  # NezhaSim

  **One ROS/Gazebo workspace for air, surface, underwater, and ground robotics**

  NezhaSim combines RotorS, UUV Simulator, ASV-Wave, and Husky with Nezha's
  cross-medium dynamics, wave interaction, force telemetry, and PX4 HITL tools.

  [Installation](#installation) · [Tutorials](src/nezha_sim/docs/tutorials/) ·
  [Source guide](src/nezha_sim/README.md) · [Documentation site](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/)
</div>

---

## Meet NezhaSim2

**NezhaSim2 is the next-generation simulator for vehicles that cross the
air–water interface.** A shared deterministic C++ physics core drives headless,
ROS 2/Gazebo Harmonic, MuJoCo, and NVIDIA Isaac Sim backends, with decomposed
forces, PX4 integration, mission tooling, and underwater perception workflows.

<p align="center">
  <img src="src/nezha_sim/docs/assets/nezhasim2-mini.gif" width="285" alt="NezhaSim2 transmedium simulation and force visualization"/>
  &nbsp;
  <img src="src/nezha_sim/docs/assets/nezhasim2-reconstruction.gif" width="500" alt="NezhaSim2 underwater mission reconstruction"/>
</p>

<p align="center"><i>From cross-medium dynamics to inspection and reconstruction—one physics contract, multiple simulation backends.</i></p>

> NezhaSim2 is under active development. This repository remains the ROS 1 /
> Gazebo 11 NezhaSim workspace.

## What's included

| Package | Purpose |
|---|---|
| [`nezha_description`](src/nezha_sim/nezha_description/) | Nezha robot URDF/Xacro models, meshes, and sensors. |
| [`nezha_gazebo`](src/nezha_sim/nezha_gazebo/) | Gazebo worlds, launch files, models, and experiment utilities. |
| [`nezha_plugins`](src/nezha_sim/nezha_plugins/) | Hydrodynamics, wave, transmedia drag, motor, telemetry, terrain, and MAVLink plugins. |

Third-party simulators live under `src/nezha_sim/third_party/`. The ROS Noetic
UUV port is vendored because it contains compatibility changes; the remaining
upstream repositories are pinned as Git submodules. The matching revisions are
also recorded in [`nezha.repos`](src/nezha_sim/nezha.repos).

## Requirements

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- `catkin_tools` and `rosdep`

## Installation

```bash
# Clone the workspace and every pinned upstream dependency
git clone --recurse-submodules \
  https://github.com/panzerjagerWang/NezhaSim-An-All-Domain-Simulator.git ~/nezha_ws

# Resolve dependencies and build
cd ~/nezha_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
```

## Quick start

```bash
# Start paused; press Play in Gazebo, or add paused:=false
roslaunch nezha_gazebo nezha_mini_lake.launch
```

Then follow the checked, command-level
[`NezhaSim tutorials`](src/nezha_sim/docs/tutorials/). The tutorial now
distinguishes scene launchers from controllers and uses the actual topics and
services exposed by this source tree.

## License and maintainer

Maintainer: **Jiaqing Wang** — `jiaqing.wang@sjtu.edu.cn` (SJTU).
See [the project license](src/nezha_sim/LICENSE) and each package's
`package.xml`. Third-party dependencies retain their upstream licenses.

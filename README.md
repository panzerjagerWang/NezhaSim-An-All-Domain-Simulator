<div align="center">
  <img src="src/nezha_sim/nezha_gazebo/mesh/logo.gif" width="220" alt="NezhaSim"/>

  # NezhaSim

  **One ROS/Gazebo workspace for air, surface, underwater, and ground robotics**

  ### [Open the Tutorial Webpage →](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/tutorials.html)

  [Documentation Website](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/) ·
  [Installation Webpage](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/installation.html) ·
  [System Overview (PDF)](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/assets/fig_main.pdf)

  NezhaSim combines RotorS, UUV Simulator, ASV-Wave, and Husky with Nezha's
  cross-medium dynamics, wave interaction, force telemetry, and PX4 HITL tools.

  [Installation](#installation) · [Tutorials](src/nezha_sim/docs/tutorials/) ·
  [Source guide](src/nezha_sim/README.md) · [Documentation site](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/)
</div>

---


## What's included

| Package | Purpose |
|---|---|
| [`nezha_description`](src/nezha_sim/nezha_description/) | Nezha robot URDF/Xacro models, meshes, and sensors. |
| [`nezha_gazebo`](src/nezha_sim/nezha_gazebo/) | Gazebo worlds, launch files, models, and experiment utilities. |
| [`nezha_plugins`](src/nezha_sim/nezha_plugins/) | Hydrodynamics, wave, transmedia drag, motor, telemetry, terrain, and MAVLink plugins. |

| Folder | Purpose |
|---|---|
| [`leap`](leap/) | LEAP fitting scripts with the NSE and TR training data. |
| [`data`](data/) | Recorded data of Figs. 9 to 11. |
| [`experiments/psc_closed_loop`](experiments/psc_closed_loop/) | Closed-loop PSC and PSC-RS test and per-step timing of Sec. VII-B. |
| [`experiments/scenario_c`](experiments/scenario_c/) | Closed-loop tracking of the Nezha-F field record in Sec. VI-C and Fig. 11. |

## Documentation website

The complete documentation is available on
[GitHub Pages](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/):
[Overview](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/),
[Installation](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/installation.html),
[Tutorials](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/tutorials.html),
[Architecture](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/architecture.html), and
[Robots & Worlds](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/robots.html).

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

# Resolve dependencies and build.
# ROS Noetic is end-of-life, so rosdep needs --include-eol-distros.
cd ~/nezha_ws
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
# The two hardware-in-the-loop packages are not needed for simulation.
catkin init
catkin config --skiplist rotors_hil_interface mavlink_sitl_gazebo
catkin build
source devel/setup.bash
```

For hardware-in-the-loop use, remove `mavlink_sitl_gazebo` from the skiplist and
install `python3-jinja2` first.

## Quick start

```bash
# Start paused; press Play in Gazebo, or add paused:=false
roslaunch nezha_gazebo nezha_mini_lake.launch
```

Then follow the checked, command-level
[`NezhaSim tutorials`](src/nezha_sim/docs/tutorials/). The tutorial now
distinguishes scene launchers from controllers and uses the actual topics and
services exposed by this source tree.

## Reproducing the experiments

These are the experiments of the paper *NezhaSim: Bridging Air, Land and Ocean
Through ROS* (IEEE Transactions on Automation Science and Engineering).

| Result | How to reproduce |
|---|---|
| LEAP fits and the ablation of Sec. VII-A, Table IV | `leap/README.md` |
| Runs of Scenario A in Fig. 9 | Run logs, run counts, resampling and the `hydro_model:=legacy` launch argument in `data/fig9_scenarioA/README.md` |
| Recorded data of Figs. 10 and 11 | `data/fig10_scenarioB` and `data/fig11_scenarioC` |
| Runs of Scenario C in Fig. 11 | `experiments/scenario_c/README.md` |
| Closed-loop PSC and PSC-RS test, per-step cost | `experiments/psc_closed_loop/README.md` |

The analysis scripts need Python 3 with numpy, scipy, pandas and sympy.

## License and maintainer

Maintainer: **Jiaqing Wang** — `jiaqing.wang@sjtu.edu.cn` (SJTU).
See [the project license](src/nezha_sim/LICENSE) and each package's
`package.xml`. Third-party dependencies retain their upstream licenses.

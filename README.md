<div align="center">
  <img src="docs/assets/img/logo.gif" width="220" alt="NezhaSim"/>

  # NezhaSim — An All-Domain Simulator

  **Bridging Air, Land &amp; Ocean Through ROS**

  Gazebo / ROS simulation for the **Nezha** family of *transmedium* robots —
  vehicles that operate across air, water surface, underwater, and ground in one
  continuous physics world.

  ### 📖 [**Read the tutorials &amp; documentation site →**](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/)
</div>



> ⚠️ **Baseline release — NezhaSim-PSC-RS (rigid switching).**
> The code in this repository is **NezhaSim-PSC-RS**, the *rigid medium-switching* baseline
> the paper uses for comparison, and is intended **only as a baseline / benchmark**. The
> continuous-transition behaviour described in the docs (PSC blending + LEAP + MT) is the
> **full NezhaSim**, still under development. The paper is currently under consideration; the
> final NezhaSim will be released once it is accepted.

---

## What is NezhaSim?

Most robotics simulators target a single operating domain. NezhaSim couples four
established single-domain simulators — **ASV-Wave** (surface), the **UUV Simulator**
(underwater), **RotorS** (air) and **Husky** (ground) — into a unified engine and
keeps force and data streams *continuous* as a vehicle crosses every medium
boundary. Three components make this work:

- **Phase Switch Console (PSC)** — evaluates the surface and underwater force
  models every step and blends them with a continuous submergence weight, so the
  air ⇄ water transition is smooth (no force spike at the waterline).
- **LEAP** — distills CFD and field data for the Near-Surface Effect and
  Transmedium Resistance into closed-form, constant-time `O(1)` surrogates
  (~0.09 ms vs. 135 ms for RBF), fast enough for real-time control.
- **Mechanical Transparency (MT)** — exposes every decoupled wrench component
  (buoyancy, drag, slamming, rotor thrust) as an inspectable ROS stream.

## Documentation &amp; tutorials

The full guide lives in [`docs/`](docs/) and is published as a website:

| Page | |
|---|---|
| [Home](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/) | Overview, core innovations, comparison table |
| [Installation](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/installation.html) | Build from source (Ubuntu 20.04 + ROS Noetic + Gazebo 11) |
| [Tutorials](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/tutorials.html) | Water take-off, triple-domain Husky mission, underwater control, reading MT force streams, gantry force-ID |
| [Architecture](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/architecture.html) | PSC, LEAP, MT, replaceable dynamics, robustness |
| [Robots &amp; Worlds](https://panzerjagerwang.github.io/NezhaSim-An-All-Domain-Simulator/robots.html) | Vehicle, world and plugin catalogue |

> The site is served from the `docs/` folder via GitHub Pages.

## Quick start

```bash
# Transmedia quadrotor in a lake (air ⇄ water take-off / landing)
roslaunch nezha_gazebo nezha_mini_lake.launch

# Husky multi-domain: ground + air + underwater
roslaunch nezha_gazebo nezha_husky_UGV_UAV_UUV.launch

# Open-ocean world with spectral waves
roslaunch nezha_gazebo ocean_world.launch
```

See the [source package README](src/nezha_sim/README.md) for the full build
instructions, dependency list (`nezha.repos`), robot models and plugin reference.

## License &amp; maintainer

Maintainer: **Jiaqing Wang** — `jiaqing.wang@sjtu.edu.cn` (SJTU).
Third-party packages listed in `nezha.repos` retain their own upstream licenses.

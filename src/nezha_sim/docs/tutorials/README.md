# NezhaSim Tutorials

A hands-on path for newcomers to the **Nezha Simulator** — the Gazebo/ROS
simulation for the *transmedia* Nezha robots that move across **air, the water
surface, underwater, and (for the Husky) land**.

These tutorials assume you have already **built the workspace** by following the
[top-level README](../../README.md) (Ubuntu 20.04 · ROS Noetic · Gazebo 11).
Work through them in order — each one builds on the previous.

## Before you start

Open a terminal and source the workspace. **Every** new terminal you use with
NezhaSim needs this line:

```bash
source ~/nezha_ws/devel/setup.bash
```

> **Two-terminal convention.** Throughout these tutorials, **Terminal A** runs
> the `roslaunch` (Gazebo) and stays open; **Terminal B** (also sourced) is where
> you run `rostopic`, `rosrun`, etc. `Ctrl-C` in Terminal A shuts the simulation
> down cleanly.

## The path

| # | Tutorial | You will learn |
|---|----------|----------------|
| 1 | [Getting started](01-getting-started.md) | Launch your first simulation, navigate Gazebo, and read the ROS graph. |
| 2 | [Worlds & robots](02-worlds-and-robots.md) | The anatomy of a launch file; how to pick a robot and a world. |
| 3 | [Transmedium take-off](03-transmedium-takeoff.md) | Command the `nezha_mini` quad through the air–water interface. |
| 4 | [Triple-domain Husky](04-triple-domain-husky.md) | Inspect the Husky's staged ground, air, and underwater configurations. |
| 5 | [Underwater control](05-underwater-control.md) | Discover and command an underwater vehicle's thrusters. |
| 6 | [Inspecting forces](06-inspecting-forces.md) | Read the force topic and hydrodynamics/phase services that this build exposes. |
| 7 | [Hardware-in-the-loop](07-hardware-in-the-loop.md) | *(Advanced, needs a Pixhawk 6C)* drive a real PX4 flight controller from the sim. |

## Tutorial scope

These beginner tutorials use launch files, topics, and services present in this
checkout. The repository also contains research controllers and experiment
scripts with hard-coded trajectories, datasets, or hardware assumptions. Those
are useful references, but they are not beginner entry points unless a tutorial
explicitly names their required inputs.

If a launch file misbehaves, the fastest debugging move is almost always to read
the launch file itself (`nezha_gazebo/launch/…`) — they are short and readable.

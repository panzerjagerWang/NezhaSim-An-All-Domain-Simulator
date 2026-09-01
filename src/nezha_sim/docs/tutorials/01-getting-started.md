# Tutorial 1 — Getting started

**Goal:** bring up your first NezhaSim world, learn to drive the Gazebo window,
and peek at the ROS graph that the simulation publishes.

**Prerequisites:** a built workspace (see the [top-level README](../../README.md)).

---

## 1. Source the workspace

In a fresh terminal (we'll call it **Terminal A**):

```bash
source ~/nezha_ws/devel/setup.bash
```

If `roscd nezha_gazebo` changes directory without error, your workspace is sourced
correctly.

## 2. Launch your first simulation

We'll spawn the **nezha_f** aerial quadrotor in the lake world:

```bash
roslaunch nezha_gazebo nezha_f.launch
```

Gazebo opens with the lake and the vehicle sitting at the surface. This launch
starts **paused** — press the **▶ Play** button at the bottom-left of the Gazebo
window (or `Ctrl-Shift-P`) to start the physics clock.

> **What just happened?** `nezha_f.launch` started a Gazebo server + GUI, loaded
> the `nezha_lake` world, and spawned the `nezha_f` robot model under the ROS
> namespace `/nezha_f`. It is a pure-simulation (no hardware) launch — see
> Tutorial 7 for the hardware-in-the-loop variants.

## 3. Find your way around Gazebo

| Action | How |
|--------|-----|
| Orbit the camera | Left-drag |
| Pan | Middle-drag (or Shift + left-drag) |
| Zoom | Scroll wheel |
| Play / pause physics | ▶ / ⏸ buttons, bottom-left |
| Reset the world | `Edit ▸ Reset World` (`Ctrl-R`) |
| Inspect a model | Click it, then expand it in the left **World** panel |

The **Real Time Factor (RTF)** readout at the bottom tells you how fast the sim
runs versus wall-clock. ~1.0 means real time.

## 4. Look at the ROS graph

The whole point of a ROS simulator is the data it publishes. Open a **second**
terminal (**Terminal B**), source it, and explore:

```bash
source ~/nezha_ws/devel/setup.bash

# What nodes are running?
rosnode list

# What topics exist? (everything under the robot's namespace)
rostopic list | grep nezha_f

# Watch the vehicle's ground-truth odometry stream
rostopic echo -n1 /nezha_f/ground_truth/odometry
```

Try `rostopic hz <topic>` to see a topic's publish rate, and `rosrun rqt_graph
rqt_graph` for a visual node/topic diagram.

## 5. Shut down

Back in **Terminal A**, press `Ctrl-C`. Wait a couple of seconds for Gazebo to
close cleanly before relaunching (a lingering `gzserver` is the most common cause
of "it won't start again" — `pgrep -a gzserver` to check, `pkill -9 gzserver` if
needed).

---

**Next:** [Tutorial 2 — Worlds & robots](02-worlds-and-robots.md), where you'll
learn how a launch file chooses *which* robot and *which* world, and how to swap
them.

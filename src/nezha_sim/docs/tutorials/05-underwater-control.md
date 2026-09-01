# Tutorial 5 — Underwater control

**Goal:** discover and command an underwater vehicle's thrusters directly, then
inspect the hydrodynamic response.

The underwater 6-DOF dynamics follow the standard **Fossen** formulation (rigid-
body mass + added-mass + linear & quadratic damping), with every coefficient read
from the vehicle's model config — so you can tune them and re-run.

---

## 1. Bring up an underwater-capable scene

The transmedium mini in the lake works well, because the lake includes the water
volume:

```bash
roslaunch nezha_gazebo nezha_mini_lake.launch
```

For a dedicated heavy ROV instead, use:

```bash
roslaunch nezha_gazebo nezha_rexrov_sim.launch
```

## 2. Find the thruster topics

In **Terminal B** (sourced):

```bash
rostopic list | grep thrusters
```

For the mini you'll see three thruster inputs:

```
/nezha_mini/thrusters/0/input   # pitch thruster
/nezha_mini/thrusters/1/input   # left
/nezha_mini/thrusters/2/input   # right
```

They take a `uuv_gazebo_ros_plugins_msgs/FloatStamped` message — a timestamped
single float (the commanded thrust setpoint).

## 3. Command a thruster by hand

Give the left and right thrusters equal positive thrust to surge forward:

```bash
rostopic pub -r 10 /nezha_mini/thrusters/1/input \
  uuv_gazebo_ros_plugins_msgs/FloatStamped "{data: 20.0}" &
rostopic pub -r 10 /nezha_mini/thrusters/2/input \
  uuv_gazebo_ros_plugins_msgs/FloatStamped "{data: 20.0}" &
```

`-r 10` republishes at 10 Hz (the plugin expects a continuous setpoint). Start
with a modest value and increase it gradually. Stop the background publishers
with `kill %1 %2` (or `Ctrl-C` if you ran them in the foreground). Try a small
pitch command on thruster `0` after stopping the surge publishers.

> **Sign & mapping** are vehicle-specific. If a positive value does the opposite
> of what you expect, that's normal — note the sign and move on.

## 4. Inspect the response

Read the pose and decomposed hydrodynamic force report while commanding the
thrusters:

```bash
rostopic echo /nezha_mini/ground_truth/pose
rosservice call /nezha_mini/get_hydrodynamics_forces "{}"
```

The service response separates buoyancy, damping, added-mass, Coriolis, and wave
terms. Repeat the call at rest and under thrust to compare them.

> `scripts/UUV_CONTROLV17.py` is an experiment script, not a ready-made tutorial
> controller: it expects a `trajectory.csv`, opens a Tk plot, and runs 100
> repeated trials. The required trajectory is not shipped at the path it uses,
> and the file is not installed as a `rosrun` executable. It is therefore not
> used in this beginner sequence.

## 5. Experiment

- Change the damping/added-mass coefficients in the vehicle's Xacro config and
  rebuild/relaunch — watch the response change.
- Log the odometry to a bag (`rosbag record /nezha_mini/ground_truth/odometry`)
  and compare runs.

---

**Next:** [Tutorial 6 — Inspecting forces](06-inspecting-forces.md), where you
*measure* the hydrodynamic forces instead of inferring them from motion.

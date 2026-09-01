# Tutorial 3 — Transmedium take-off

**Goal:** spawn the `nezha_mini` transmedium quadrotor in a lake and command it
through the water surface using the RotorS position controller.

---

## 1. Launch the lake world

```bash
roslaunch nezha_gazebo nezha_mini_lake.launch paused:=false
```

This brings up Gazebo with the lake, wave field, and `nezha_mini` vehicle. If you
omit `paused:=false`, press **Play** before sending a command.

## 2. Command a water exit

In **Terminal B**, publish a position setpoint above the waterline:

```bash
source ~/nezha_ws/devel/setup.bash
rostopic pub -r 20 /nezha_mini/command/pose geometry_msgs/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 2.0}, orientation: {w: 1.0}}}'
```

Keep the publisher running while the vehicle climbs; press `Ctrl-C` to stop it.

> **Why not `nezha_mini_takeoff.launch`?** In this checkout,
> `nezha_mini_lake.launch`, `nezha_mini_takeoff.launch`, and
> `nezha_mini_wavetakeoff.launch` contain the same scene/controller setup. The
> latter two names are compatibility aliases; neither launches a maneuver
> script. The ROS setpoint above is what causes the takeoff.

## 3. What to watch

As the rotors spin up, the dominant vertical force hands off from **buoyancy**
(in water) to **aerodynamic thrust** (in air). Three things happen at the
boundary:

1. **Near-Surface Effect (NSE):** rotor thrust derates as the props approach the
   surface.
2. **Transmedium Resistance (TR):** a brief resistance peak as the body breaches.
3. **Clean hover** in air, with the water drag and buoyancy gone.

The unified hydrodynamics plugin updates water contact continuously, while the
transmedia drag and motor models react to the live phase estimate. You can
inspect their telemetry directly in [Tutorial 6](06-inspecting-forces.md).

## 4. Confirm it from the data

In **Terminal B** (sourced), watch the vehicle climb out of the water:

```bash
# Ground-truth pose (watch position.z)
rostopic echo /nezha_mini/ground_truth/pose

# Phase estimate and surface height at the vehicle
rosservice call /nezha_mini/transmedia/get_phase_sample "{}"
```

You should see `position.z` rise through the reported `surface_z`.

> **Troubleshooting.** If the vehicle just bobs and never rises, the physics
> clock may be paused — press ▶ in Gazebo. If nothing responds, confirm that
> `/nezha_mini/command/pose` has a subscriber with `rostopic info`. If it sinks,
> check that you launched the lake world rather than a dry world.

---

**Next:** [Tutorial 4 — Triple-domain Husky](04-triple-domain-husky.md).

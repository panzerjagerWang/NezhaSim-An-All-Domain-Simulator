# Tutorial 6 — Inspecting forces (Mechanical Transparency)

**Goal:** read the force telemetry this checkout actually exposes: a ROS topic
for transmedia drag and ROS services for the hydrodynamic decomposition and
air/water phase sample.

---

## 1. Discover what your build exposes

Names vary between robot models, so always **discover first**:

```bash
# ROS topics that look like forces
rostopic list | grep -iE 'force|wrench|buoyan|drag|thrust'

# services that return force data
rosservice list | grep -iE 'force|hydro|phase'
```

For `nezha_mini`, this source defines `/nezha_mini/transmedia_drag`,
`/nezha_mini/get_hydrodynamics_forces`, and
`/nezha_mini/transmedia/get_phase_sample`. Some `/debug/forces/...` publishers in
the C++ code use Gazebo Transport rather than ROS, so they do not appear in
`rostopic list`.

## 2. Echo a single effect in real time

Stream the transmedia drag vector while running the transition from [Tutorial
3](03-transmedium-takeoff.md):

```bash
rostopic echo /nezha_mini/transmedia_drag
```

Call the hydrodynamics service to get the remaining force terms:

```bash
rosservice call /nezha_mini/get_hydrodynamics_forces "{}"
rosservice call /nezha_mini/transmedia/get_phase_sample "{}"
```

The first response reports buoyancy, damping, added-mass, Coriolis, wave force,
and submersion ratio. The second reports the local surface height and phase.

## 3. Plot the water-exit force history

Plot the published transmedia drag vector during a transition:

```bash
rosrun rqt_plot rqt_plot
```

In the rqt_plot topic box, add
`/nezha_mini/transmedia_drag/vector/z`. Run the setpoint command from Tutorial 3
in another terminal and watch the signal as the vehicle breaches.

To capture a run for later analysis:

```bash
rosbag record -O takeoff_forces.bag /nezha_mini/transmedia_drag /nezha_mini/ground_truth/pose
# ... run the take-off, then Ctrl-C to stop recording
rqt_bag takeoff_forces.bag    # scrub/plot the recorded streams
```

ROS services are request/response calls and cannot be recorded by `rosbag` as
topics. Poll the hydrodynamics service separately if you need those components,
or adapt an experiment logger to publish its responses.

## 4. Why this matters

This split makes it possible to distinguish interface drag from buoyancy,
damping, added mass, Coriolis, and wave loading instead of inferring every effect
from vehicle motion alone.

---

**Next:** [Tutorial 7 — Hardware-in-the-loop](07-hardware-in-the-loop.md)
*(advanced; needs a real flight controller)*.

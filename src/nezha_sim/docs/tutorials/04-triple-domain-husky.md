# Tutorial 4 — Triple-domain Husky

**Goal:** bring up the transmedia **Husky** and inspect how its ground, aerial,
and underwater components are composed. This tutorial verifies the model and
interfaces; it does not claim to run an autonomous three-domain mission.

---

## 1. Launch the all-domain Husky

```bash
roslaunch nezha_gazebo nezha_husky_UGV_UAV_UUV.launch
```

This spawns the Husky with **U**nmanned **G**round, **A**ir, and **U**nderwater
**V**ehicle components in one model. Start the physics with Gazebo's **Play**
button, or pass `paused:=false`.

### Staged variants

If you want to understand each capability in isolation, bring them up one layer
at a time. The launch files live in `nezha_gazebo/launch/nezha_husky/`:

```bash
# ground only
roslaunch nezha_gazebo nezha_husky_pure_UGV.launch

# ground + air
roslaunch nezha_gazebo nezha_husky_UGV_UAV.launch

# full ground + air + underwater
roslaunch nezha_gazebo nezha_husky.launch
```

> **Heads-up.** These are *simulation* launches. The Husky's hardware-in-the-loop
> flight variant is the separately-named `nezha_husky_hil.launch` (Tutorial 7) —
> the plain launches above never touch any flight controller.

## 2. Inspect what spawned

In **Terminal B** (sourced):

```bash
rosnode list
rostopic list | grep nezha_husky
```

The exact topic set depends on the selected staged model. Discover it rather
than assuming names:

```bash
rostopic list | grep -E 'nezha_husky|cmd_vel|thruster|command'
rosservice list | grep -E 'nezha_husky|hydrodynamics|phase'
```

Use `rostopic info <topic>` before publishing so you can confirm the message type
and that the intended controller is subscribed.

## 3. About the research mission scripts

`nezha_gazebo/scripts/` contains experiment-specific controllers and recorded
trajectory workflows. They require matching CSV inputs, topic layouts, and in
some cases modules that are not part of the beginner install. In particular,
`scenario_run.py` targets `nezha_mini` and a hard-coded field-data file; it is
not the Husky mission driver. Do not run it for this tutorial.

## 4. What to observe at the boundaries

Compare the topic/service lists produced by the staged launches. This shows
which actuator and physics interfaces each composition adds. For the force and
phase interfaces exposed by the transmedium model, continue with [Tutorial
6](06-inspecting-forces.md).

---

**Next:** [Tutorial 5 — Underwater control](05-underwater-control.md).

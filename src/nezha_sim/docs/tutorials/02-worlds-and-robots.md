# Tutorial 2 — Worlds & robots

**Goal:** understand how a launch file chooses a robot and a world, so you can
mix and match them yourself.

---

## 1. The anatomy of a launch file

Open `nezha_gazebo/launch/nezha_f.launch` in an editor. Near the top you'll see
arguments with defaults — these are the knobs you can override from the command
line:

```xml
<arg name="mav_name"   default="nezha_f"/>     <!-- which robot -->
<arg name="world_name" default="nezha_lake"/>  <!-- which world -->
<arg name="paused"     default="true"/>        <!-- start paused? -->
<arg name="gui"        default="true"/>         <!-- show the Gazebo window? -->
```

Further down, the world is loaded …

```xml
<arg name="world_name" value="$(find nezha_gazebo)/worlds/$(arg world_name).world"/>
```

… and the robot is spawned from its model file:

```xml
<include file="$(find nezha_gazebo)/launch/spawn_mav.launch">
  <arg name="model" value="$(find nezha_description)/$(arg mav_name)/config/mav_generic_odometry_sensor.gazebo"/>
</include>
```

So `mav_name` picks both the ROS namespace **and** the model directory under
`nezha_description/`, while `world_name` picks a `.world` file from
`nezha_gazebo/worlds/`. A model also needs matching controller YAML files, so
`mav_name` is not an arbitrary model switch; use the supplied launch file for
each robot family.

## 2. Override an argument from the command line

Any `<arg>` can be set as `name:=value`. For example, launch the same quad in an
empty world, running immediately (not paused), without the GUI:

```bash
roslaunch nezha_gazebo nezha_f.launch world_name:=nezha_empty paused:=false gui:=false
```

This is the single most useful skill in ROS launching — you rarely need to edit a
launch file just to change the world or start state.

## 3. The robot catalogue

Each robot is a directory under `nezha_description/`. Use these supported launch
entries rather than passing every model through `nezha_f.launch`:

| Robot | Domain | Launch file |
|-------|--------|-------------|
| `nezha_f` | Air ⇄ water | `nezha_f.launch` |
| `nezha_mini` | Air ⇄ water | `nezha_mini_lake.launch` |
| `nezha_husky` | Land + air + underwater components | `nezha_husky_UGV_UAV_UUV.launch` |
| `rexrov_spotlight` | Underwater | `nezha_rexrov_sim.launch` or `rexrov_lake.launch` |

There are also dedicated **hardware-in-the-loop** clones (`nezha_f_mav`,
`nezha_mini_hil`, `nezha_husky_hil`) — covered in Tutorial 7 — and experimental
or test models (`nezha_beetle`, `nezha_rocket`, and the gantry rigs). See
[`nezha_description/README.md`](../../nezha_description/README.md) for the model
file convention (`<robot>_base.xacro` + `UAV/`, `UUV/`, `UGV/` sub-macros).

## 4. The world catalogue

List everything your build ships:

```bash
ls $(rospack find nezha_gazebo)/worlds
```

The ones you'll use most:

| `world_name` | What's in it |
|--------------|--------------|
| `nezha_empty`            | Bare ground plane — fast, good for first tests. |
| `nezha_lake`             | Lake with a spectral wave field + lake bottom (default for the transmedium demos). |
| `nezha_ocean`            | Open ocean with waves. |
| `nezha_empty_underwater` | Submerged box for underwater dynamics. |
| `nezha_hitl` / `nezha_hitl_ocean` | Lightweight real-time worlds for hardware-in-the-loop (Tutorial 7). |

## 5. Try it

Spawn the transmedium mini in the ocean instead of the lake:

```bash
roslaunch nezha_gazebo nezha_mini_lake.launch world_name:=nezha_ocean
```

> **Tip:** not every robot/world pairing is meaningful (don't put an aerial-only
> quad underwater and expect it to swim). When in doubt, use the purpose-built
> launch files (`nezha_mini_lake.launch`, `nezha_husky_UGV_UAV_UUV.launch`, …) —
> they pair a robot with a sensible world and any helper nodes it needs.

---

**Next:** [Tutorial 3 — Transmedium take-off](03-transmedium-takeoff.md).

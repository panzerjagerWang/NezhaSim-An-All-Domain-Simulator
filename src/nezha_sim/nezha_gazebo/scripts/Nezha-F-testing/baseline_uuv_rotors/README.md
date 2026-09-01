# Scenario-C Baseline — XX-F (UUV_simulator + RotorS, no NSE / no TR)

Comparison **baseline for Test Scenario C** (XX-F HAUV, water-to-air transition
**trajectory tracking**). It flies the same water-to-air waypoint path as the
original `flight_controllerv3.py`, under the same wave field, but on the
`nezha_f_bl` robot driven by the **upstream simulator cores** — and deliberately
**without XXSim's Near-Surface-Effect (NSE) rotor model and without the
Transmedium-Resistance (TR) drag model**.

| Aspect    | XXSim XX-F (the paper's method)             | This baseline                                   |
|-----------|--------------------------------------------|-------------------------------------------------|
| Hydro     | rewritten hydro + PSC + **TR** drag        | **genuine UUV_simulator** (Fossen), no PSC, **no TR** |
| Rotors    | RotorS + **NSE** k_T/k_Q gains             | **genuine RotorS** motor model, **no NSE**      |
| Tracking  | lee controller flies `/command/trajectory` | identical — lee controller flies for real       |
| Waves     | `nezha_ocean_waves`                         | `nezha_ocean_waves` (identical)                 |

Same waypoints, world and controller ⇒ any tracking/force difference isolates
what NSE + TR + PSC contribute to the transition.

## Output (matches the original Scenario-C recorder `data_recorder.py`)
- `position_x.txt`, `position_y.txt`, `position_z.txt`,
  `velocity_x.txt`, `velocity_y.txt`, `velocity_z.txt`
  — one `[v0,...,v99],` line **appended per epoch** (100 time-uniform samples of
  `/nezha_f_bl/ground_truth/odometry`), exactly the original format.
- `baseline_f_run_<epoch>_<ts>.bag` — replay rosbag per epoch: UUV hydro wrenches
  (`/debug/forces/nezha_f_bl/base_link/{restoring,damping,added_mass,added_coriolis}`),
  `is_submerged`, the 4 RotorS rotor speeds, `command/{trajectory,motor_speed}`,
  ground-truth odometry, `/tf`. (The decoupled hydro/rotor forces live here.)
- `waypoints.csv` — the water-to-air path (copied from the original; +0.5 m z
  offset applied in the trajectory, as the original does).

## What was added to the repo
- `nezha_description/nezha_f_bl/` — copy of `nezha_f` with the hydro stack swapped
  to the **genuine UUV_simulator** plugin (`plugins.xacro`: macro
  `uuv_baseline_hydro_model`, nezha_f Fossen coefficients in uuv sign convention,
  no surface plugin / no Phase Switch Console / no TR) and the rotor plugin swapped
  to the **genuine RotorS** `librotors_gazebo_motor_model.so`
  (`UAV/UAV_plugin_baseline.xacro`, no NSE k_T/k_Q tags).
  (Note: `UUV/UUV.xacro` namespace property was retargeted to `nezha_f_bl`.)
- `nezha_gazebo/launch/nezha_f_baseline.launch` — same world/waves, spawns
  `nezha_f_bl`, **keeps** the lee controller (real flight).

## Reproduce
```bash
# terminal 1
source devel/setup.bash
roslaunch nezha_gazebo nezha_f_baseline.launch gui:=false paused:=false

# terminal 2
source devel/setup.bash
rosrun nezha_gazebo baseline_f_recorder.py 10     # or: python3 baseline_f_recorder.py 10
```

## Observed (smoke test)
- Submerged buoyancy `restoring_z ≈ 10.0 N` = ρ·g·V (1028·9.81·0.001) ✓; rotors
  spin (~770 rad/s real; `motor_speed` = value × `rotorVelocitySlowdownSim` 20).
- Horizontal tracking is good (ends at the final waypoint x,y), but the **vertical
  water-to-air transition degrades without NSE/TR** (altitude dips well below the
  reference and the final height undershoots) — the intended baseline contrast.

# Closed-loop PSC and PSC-RS test and per-step cost (Sec. VII-B and IV-B)

The test flies Nezha-Mini through one mission under PSC and under PSC-RS. Both arms
use the same world, wave field (0.1 m amplitude, 4 s period), NSE and TR plugins,
RotorS Lee position controller and the same 30 initial conditions. Only the
`<rigid_switching>` flag of the unified hydrodynamics plugin differs. With the
flag set, the plugin applies the surface model or the Fossen model one at a time
according to the sign of the local submergence instead of blending them.

Mission, relative to the first command: hover at 1.5 m for 10 s, descend to
-0.3 m for 8 s (water entry), hover at 1.5 m again for 10 s. Seed `s` draws the
horizontal start position in [-3, 3] m, the depth in [-0.8, -0.3] m and the
command time in [4, 8] s.

A run succeeds when the vehicle holds 1.5 m within 0.2 m and stays within 0.3 m
horizontally during [7, 10] s and [25, 28] s, goes below z = 0 during [10, 18) s,
and never diverges (finite state, speed below 10 m/s, tilt below 60 deg).

## Files

| File | Role |
|---|---|
| `run_trial.py` | Runs one trial headless and logs odometry, the MT force service and the phase service |
| `run_batch.sh` | Runs seeds 0 to 29 for both arms |
| `analyze_batch.py` | Success and transient metrics, paired comparison |
| `analyze_timing.py` | Per-physics-step cost from the plugin timers |
| `rerun_crashed.py` | Repeats runs that ended in a gzserver crash, with the same seed |
| `make_world.py` | Writes `nezha_lake_waves.world` with the wave field and the step timer |
| `results/` | Per-run metadata, metrics and logs, summary and timing files |

The launch file is `src/nezha_sim/nezha_gazebo/launch/nezha_mini_psc_trial.launch`.

## Per-step timing

Setting `NEZHA_PROFILE_DIR=<dir>` makes the NSE motor, TR and unified plugins and the
`nezha_step_timer` world plugin write one float32 duration in microseconds per call
(`src/nezha_sim/nezha_plugins/include/nezha_step_profiler.hh`). Without the variable the
timers are inactive. `run_trial.py --profile` sets it per run.

## Run

```bash
source devel/setup.bash
python3 experiments/psc_closed_loop/make_world.py src/nezha_sim/nezha_gazebo 0.1 4.0 nezha_lake_waves.world
catkin build nezha_gazebo
bash experiments/psc_closed_loop/run_batch.sh results
python3 experiments/psc_closed_loop/analyze_batch.py results summary.json
python3 experiments/psc_closed_loop/analyze_timing.py results psc timing_psc.json
```

The runs in `results/` use the TR plugin after the fix of its residual
expression in `src/nezha_sim/nezha_plugins/src/nezha_transmediaDragPlugin.cc`.
Before the fix the residual was read from freed memory, which could alter the
water-entry model and crash gzserver. None of
the runs in `results/` crashed. `rerun_crashed.py` repeats a run that ends in
a crash with the same seed.

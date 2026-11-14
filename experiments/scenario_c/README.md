# Scenario C: closed-loop tracking of the Nezha-F field record (Sec. V-C, Fig. 11)

One script drives both arms, so NezhaSim (`nezha_f`) and the UUV Simulator and RotorS baseline
(`nezha_f_bl`) share every part of the protocol below. They differ only in the plugins:
the baseline uses the upstream UUV Simulator hydrodynamics and the upstream RotorS motor
model, without PSC, NSE and TR.

| Item | Value |
|---|---|
| Waypoints | All 100 points of the field record (`waypoints.csv`, equal to `data/fig11_scenarioC/field_p*.csv`) |
| Start state | Waypoint 0, 0.394 m below the surface, level attitude |
| Pacing | Uniform over T = 18.93 s, the path length of 14.449 m divided by the mean field speed of 0.7634 m/s |
| Clock | Simulation time for the waypoints and the samples (`use_sim_time:=true`) |
| Controller | RotorS Lee position controller with the model mass of 2 kg |
| Motor constant | 8.54858e-06 in both arms |
| World | `nezha_lake` with `nezha_ocean_waves` |
| Samples | 100 points of the ground-truth odometry, uniform in time over [0, T] |
| Repetitions | 100 runs per arm |

The field record has no time stamps along its waypoints, so the pacing comes from its
path length and mean speed.

## Files

| File | Role |
|---|---|
| `run_scc.py` | Flies N runs of one arm and logs the samples |
| `run_arm.sh` | Starts Gazebo for one arm and restarts it after a crash |
| `run_all.sh` | Runs both arms side by side |
| `metrics.py` | Computes the metrics and writes the data files of Fig. 11 |
| `waypoints.csv` | The waypoints of the field record |
| `results/` | `runs.jsonl`, `run.log` per arm, and `summary.json` |

The launch files are `src/nezha_sim/nezha_gazebo/launch/nezha_f.launch` and `nezha_f_baseline.launch`.

## Run

Inside the ROS Noetic container, with `NEZHA_WS` pointing to the built workspace:

```bash
experiments/scenario_c/run_all.sh 100 <results dir>
python3 experiments/scenario_c/metrics.py <results dir> data/fig11_scenarioC --export
```

## Results

| Metric | NezhaSim | Baseline |
|---|---|---|
| Successful runs | 100 of 100 | 100 of 100 |
| Mean position error [m] | 0.148 | 0.141 |
| Peak of the mean position error [m] | 0.50 | 0.44 |
| Standard deviation of the per-run position error [m] | 0.006 | 0.007 |
| Mean speed error [m/s] | 0.58 | 0.58 |
| Surface crossing relative to the waypoints [s] | +0.18 | -0.08 |

A two-sided Mann-Whitney test on the per-run position errors gives p = 3.7e-12, with the
baseline lower by 7 mm. The same test on the per-run speed errors gives p = 0.10.

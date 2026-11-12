# Scenario A (Fig. 9)

## Runs

- `raw_runs/`: the logs of all 21 NezhaSim runs, recorded on 29 and 30 December 2025 from the same start pose.
  Every run completed the land, water and air segments. Run 1767102720 logged no forces (its force columns are zero).
- `trajectory/`: all 21 runs, resampled to 100 points per mode by `data_process2.py`. Fig. 9(b) averages them.
- `force/processed_data_100pts/`: the same files without run 1767102720. Fig. 9(c) averages these 20 runs.
- `baseline/`: the 10 successful runs of the baseline built on UUV Simulator and RotorS. The 3 failed baseline runs,
  in which the depth control diverged and the vehicle sank to the lake bottom, are in
  `src/nezha_sim/nezha_gazebo/scripts/nezha_husky/United_controller/baseline_uuv_rotors/failed_runs/`.

## Hydrodynamics of Nezha-Husky

The runs used the surface plugin, the Fossen plugin and the phase switch with the coefficients of Table II.
Load this stack with

```bash
roslaunch nezha_gazebo nezha_husky_UGV_UAV_UUV.launch hydro_model:=legacy
```

Without the argument Nezha-Husky loads the submergence-gated unified plugin. Its coefficients were chosen for that
plugin after these runs and differ from Table II.

# LEAP fitting scripts and training data

| Path | Content |
|---|---|
| `nse/<wing>/data` | Near-surface-effect thrust and torque measurements of the test rig, one folder per propeller-wing configuration (`fold`, `thick`, `thin`, `winglet`) |
| `nse/<wing>/symbolic_regression` | Symbolic search of the residual term |
| `nse/<wing>/outputs/*/hall_of_fame.csv` | Expressions found by the symbolic search, read by the report scripts |
| `nse/<wing>/residual_source.json`, `tr/<phase>/residual_source.json` | The `hall_of_fame.csv` files whose residual forms enter Table IV |
| `nse/<wing>/rbf_interp_ktkq` | RBF baseline |
| `nse/thin/make_nse_report.py` | Fits the backbone, the residual and the baselines and writes the metrics |
| `tr/original_data` | Water-entry and water-exit resistance data from CFD and prior experiments |
| `tr/<phase>/data`, `tr/<phase>/symbolic_regression` | Processed data and symbolic search for `Water_Entry` and `Water_Exit` |
| `tr/Water_Exit/make_ctr_report.py` | Report script for the transmedium-resistance models |
| `table_iv/sweep.py` | Reruns the ablation of Table IV over all cases and both splits with five seeds per case |
| `table_iv/sweep/sweep_metrics.csv` | The five-seed runs reported in Table IV |

The fitted models used by the simulator are in
`src/nezha_sim/nezha_plugins/Settings/prop_data` and
`src/nezha_sim/nezha_plugins/Settings/Transmeida_liftdrag_force`.

## Run

```bash
cd leap
# one NSE case and one TR phase
python3 nse/thin/make_nse_report.py --root nse/winglet --target both \
    --split extrapolation --seed 42 --outdir out/nse_winglet
python3 tr/Water_Exit/make_ctr_report.py --root tr/Water_Entry \
    --split extrapolation --seed 42 --outdir out/tr_entry
# all cases, both splits, five seeds
python3 table_iv/sweep.py table_iv/sweep
```

The extrapolation fits are ill-conditioned by construction, so single values can
differ slightly between numpy and scipy versions and between BLAS thread counts.
Table IV was produced with numpy 2.5.2, scipy 1.18.1 and OpenBLAS 0.3.34 with its
default threads on 24 cores, which reproduces the RMSE and MaxErr values of
`table_iv/sweep/sweep_metrics.csv` up to floating-point noise below 1e-12 in
relative terms (only the RBF interpolation of Water-Entry shows any). The symbolic search needs PySR;
the report scripts reuse the stored `hall_of_fame.csv` expressions and refit
their constants on the training split.

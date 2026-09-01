# TR — Transmedium-Resistance (C_TR) model

This folder is the documented home of the **transmedium-resistance** drag model
used during air ⇄ water crossings. It stores the fitted model, the authoritative
reference predictor, the held-out accuracy reports, and a script to re-verify the
model against the experimental data.

## What the model is

The transmedium drag plugin (`nezha_plugins/src/nezha_transmediaDragPlugin.cc`)
multiplies a reference drag by a **resistance coefficient C_TR**, predicted as a
function of two inputs:

- `x0 = h = z/l` — body depth normalized by length
- `x1 = v = |velocity|`

The predictor is a **physics skeleton + symbolic residual** (the "Full LEAP"
form):

```
phys(h, v) = 1 + A1·e^(-B1·h) + A2·e^(-B2·h) + A3 / (1+h)^n
             with A1=A10+A11·v,  A2=A20+A21·v,  A3=A30+A31·v,
                  B1=B10+B11·v,  B2=B20+B21·v
C_TR(h, v) = max(0,  phys(h, v) + ε_residual(x0, x1))
```

Each `Water_*.json` holds a **piecewise** model: a `boundary_on_z_over_l` plus
`theta[22]` = two 11-coefficient segments
(`[A10,A11,A20,A21,A30,A31,B10,B11,B20,B21,n]`), the left segment used when
`h ≤ boundary`, the right segment otherwise — and the symbolic `equation_residual`
(note: it uses `^` for exponentiation).

## Files

| File | Role |
|------|------|
| `Water_Entry.json` | C_TR model for the **water-entry** regime (descending through the surface). |
| `Water_Exit.json`  | C_TR model for the **water-exit** regime (breaching upward). |
| `Water_above.json` | C_TR model for the **above-water** regime. |
| `CTR_predict.py`   | Authoritative runtime reference predictor (the C++ `Eval()` mirrors this exactly). |
| `verify_tr_model.py` | Re-scores a model JSON against a ground-truth CSV (`z_over_l,velocity,CTR`). |
| `reports/*_summary.md`, `reports/*_metrics.csv` | Held-out accuracy reports (4-model comparison). |

> **Runtime copy.** At runtime the plugin loads these JSONs from
> `nezha_plugins/Settings/Transmeida_liftdrag_force/`. The copies here are
> byte-identical and serve as the documented, verifiable source of record. If you
> update a model, update **both** locations (or repoint the plugin — see below).

## Provenance

The models were fitted by the CFD/experiment pipeline in
`Data_Exp/water_drag/Resis_Extract/` (`Water_Entry/`, `Water_Exit/`): a piecewise
physics backbone fit to the merged CFD + tow-tank data, plus a PySR symbolic
residual (hall-of-fame form, constants re-fit leakage-free per split). The full
fitted object lives in the experiment's `*.pkl`; the deployed JSON exports
`theta`, `boundary`, and the final `equation_residual`.

## Accuracy (verified)

Re-implementing the deployed JSON predictor and scoring it against the **full**
experimental ground truth (`merged_rows.csv`):

| Regime | N | C_TR range | RMSE | RMSE / range | Max err |
|--------|---|-----------|------|--------------|---------|
| Water entry | 5850 | 0.024 – 0.646 | **0.0184** | 2.96 % | 0.064 |
| Water exit  | 229  | 0.041 – 0.452 | **0.0148** | 3.60 % | 0.062 |

These match the held-out **Full LEAP** figures in `reports/` (entry random-split
RMSE 0.019 / extrapolation 0.015; exit random 0.020 / extrapolation 0.031),
confirming the simulator runs the genuine fitted model — and that it
**extrapolates** well (the degree-4 polynomial surrogate is ~15× worse on the
extrapolation split).

**Real-time:** the model is a closed-form O(1) expression (~0.06 ms per curve,
~58 µs per query). The alternative RBF interpolation is ~44 ms/query (≈500×
slower) and would stall a Gazebo physics step — so the deployed LEAP model runs
**fluently** in the loop. The C++ `Eval()` was previously verified bit-for-bit
against `CTR_predict.py`.

## Re-verify

```bash
cd nezha_gazebo/TR
python3 verify_tr_model.py \
    --model Water_Entry.json \
    --csv   ~/文档/Timulator/Data_Exp/water_drag/Resis_Extract/Water_Entry/data/merged_rows.csv
```

## Changing the deployed model

The plugin reads `Settings/Transmeida_liftdrag_force/Water_*.json`. To change the
TR model: edit the JSON there (keep this folder in sync), or change `entryJson` /
`exitJson` / `aboveJson` in `nezha_transmediaDragPlugin.cc` to point here and
rebuild. The `ctrScale` SDF parameter scales the whole coefficient at spawn time.

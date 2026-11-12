#!/usr/bin/env python3
"""5-seed sweep of the LEAP ablation harness (NSE 4 wings + TR 2 phases)."""
import json, subprocess, sys, csv
from pathlib import Path

PY   = sys.executable
BASE = Path(__file__).resolve().parents[1]
NSE  = BASE / "nse"
TR   = BASE / "tr"
OUT  = Path(sys.argv[1])
SEEDS = [42, 43, 44, 45, 46]
SPLITS = ["random", "extrapolation"]
rows = []

def run(cmd, out):
    out.mkdir(parents=True, exist_ok=True)
    subprocess.run([str(c) for c in cmd], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    return json.loads((out / "metrics.json").read_text())

for wing in ["thick", "thin", "winglet", "fold"]:
    for split in SPLITS:
        for seed in SEEDS:
            o = OUT / "nse" / wing / split / f"seed{seed}"
            rep = run([PY, NSE/"thin"/"make_nse_report.py", "--root", NSE/wing,
                       "--target", "both", "--split", split, "--seed", seed,
                       "--outdir", o], o)
            for tgt, blk in rep["targets"].items():
                for m in blk["metrics"]:
                    rows.append(dict(task="NSE", case=wing, target=tgt, split=split,
                                     seed=seed, n_train=rep.get("n_train"),
                                     n_test=rep.get("n_test"), **m))
            print("NSE", wing, split, seed, flush=True)

BOUND = {"Water_Exit": 0.4112672070690474, "Water_Entry": 0.3953641455851612}
for phase in ["Water_Exit", "Water_Entry"]:
    for split in SPLITS:
        for seed in SEEDS:
            o = OUT / "tr" / phase / split / f"seed{seed}"
            rep = run([PY, TR/"Water_Exit"/"make_ctr_report.py", "--root", TR/phase,
                       "--boundary", BOUND[phase], "--split", split, "--seed", seed,
                       "--outdir", o], o)
            for m in rep["metrics"]:
                rows.append(dict(task="TR", case=phase, target="CTR", split=split,
                                 seed=seed, n_train=rep.get("n_train"),
                                 n_test=rep.get("n_test"), **m))
            print("TR", phase, split, seed, flush=True)

cols = ["task","case","target","split","seed","n_train","n_test","model",
        "RMSE","MaxErr","t_inf_us","t_curve_ms"]
with open(OUT/"sweep_metrics.csv","w",newline="") as f:
    w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore"); w.writeheader()
    w.writerows(rows)
print("wrote", OUT/"sweep_metrics.csv", len(rows), "rows")

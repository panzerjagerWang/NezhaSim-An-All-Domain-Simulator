#!/usr/bin/env python3
"""Scenario C metrics and the data files of Fig. 11.

Position error: distance between index-aligned points of a run and of the field
record, after expressing both relative to their start point. Speed error:
|speed_run - speed_field| without rescaling. Curves are averaged over the runs.
A run counts as a success when it never diverges (tilt below 60 deg, speed below 10 m/s)
and its mean position error stays below 1 m.

usage: metrics.py RESULTS_DIR DATA_DIR [--export]
RESULTS_DIR holds nezhasim/runs.jsonl and baseline/runs.jsonl.
DATA_DIR holds field_p*.csv and field_v*.csv. With --export the runs are written there as well.
"""
import json
import os
import sys

import numpy as np
from scipy import stats

results, data = sys.argv[1], sys.argv[2]
export = "--export" in sys.argv[3:]
FP = np.stack([np.loadtxt(os.path.join(data, f"field_p{a}.csv"), delimiter=",") for a in "xyz"], -1)
FV = np.stack([np.loadtxt(os.path.join(data, f"field_v{a}.csv"), delimiter=",") for a in "xyz"], -1)
fs = np.linalg.norm(FV, axis=1)

out = {}
per_run = {}
for arm in ("nezhasim", "baseline"):
    rows = [json.loads(l) for l in open(os.path.join(results, arm, "runs.jsonl"))]
    P = np.array([[r["px"], r["py"], r["pz"]] for r in rows]).transpose(0, 2, 1)
    V = np.array([[r["vx"], r["vy"], r["vz"]] for r in rows]).transpose(0, 2, 1)
    E = np.linalg.norm((P - P[:, :1]) - (FP - FP[:1])[None], axis=-1)
    Sp = np.linalg.norm(V, axis=-1)
    S = np.abs(Sp - fs[None])
    Ev, Sv = E.mean(0), S.mean(0)
    tilt = np.array([max(r["tilt"]) for r in rows])
    ok = (tilt < 60) & (Sp.max(1) < 10) & (E.mean(1) < 1.0)
    per_run[arm] = dict(pos=E.mean(1), speed=S.mean(1))
    out[arm] = dict(
        runs=len(rows), success=int(ok.sum()),
        pos_mean=float(Ev.mean()), pos_peak=float(Ev.max()), pos_frac_below_0p5=float((Ev < 0.5).mean()),
        speed_mean=float(Sv.mean()), speed_peak=float(Sv.max()),
        run_sd_pos=float(E.mean(1).std(ddof=1)), run_sd_speed=float(S.mean(1).std(ddof=1)),
        end_z_error=float(((P[:, -1, 2]) - FP[-1, 2]).mean()), peak_z=float(P[:, :, 2].max(1).mean()),
        start_z=float(P[:, 0, 2].mean()), max_tilt=float(tilt.max()),
        rtf_min=float(min(r["rtf"] for r in rows)), rtf_median=float(np.median([r["rtf"] for r in rows])),
        kt_range=[min((r["kt_min"] or 0) for r in rows), max((r["kt_max"] or 0) for r in rows)])
    if export:
        for name, A in (("px", P[..., 0]), ("py", P[..., 1]), ("pz", P[..., 2]), ("vx", V[..., 0]), ("vy", V[..., 1]), ("vz", V[..., 2])):
            np.savetxt(os.path.join(data, f"{arm}_{name}.csv"), A, fmt="%.6f", delimiter=",")

out["comparison"] = {}
for k in ("pos", "speed"):
    a, b = per_run["nezhasim"][k], per_run["baseline"][k]
    u = stats.mannwhitneyu(a, b, alternative="two-sided")
    out["comparison"][k] = dict(difference=float(a.mean() - b.mean()), mann_whitney_p=float(u.pvalue),
                                prob_nezhasim_lower=float(1.0 - u.statistic / (len(a) * len(b))))
print(json.dumps(out, indent=1))
json.dump(out, open(os.path.join(results, "summary.json"), "w"), indent=1)

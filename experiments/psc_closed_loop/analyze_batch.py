#!/usr/bin/env python3
"""Score the closed-loop PSC vs PSC-RS batch.

Success (fixed before the batch was run), with t relative to the first command:
  1. first exit: |z - z_target| <= 0.2 m and horizontal error <= 0.3 m for all of t in [7, 10) s
  2. water entry: the body center goes below z = 0 during t in [10, 18) s
  3. second exit: the same hover test for t in [25, 28] s
  4. no divergence at any time: finite state, |v| < 10 m/s, tilt < 60 deg,
     and no out-of-range veto of the hydrodynamics plugin in the log
Metrics: RMS height error over both hover windows ([5, 10) and [23, 28] s), peak
vertical acceleration and peak step of the MT vertical force within 0.5 s of
each surface crossing.

Usage: analyze_batch.py <results dir> <out json>
"""
import csv
import glob
import json
import lzma
import os
import sys

import numpy as np
from scipy import stats

root, out_path = sys.argv[1], sys.argv[2]


def open_text(path, **kw):
    """Open path, or its .xz copy when only the compressed file is stored."""
    if not os.path.exists(path) and os.path.exists(path + ".xz"):
        return lzma.open(path + ".xz", "rt", **kw)
    return open(path, **kw)


def load(path):
    with open_text(path) as f:
        rows = list(csv.reader(f))
    return rows[0], rows[1:]


def tilt_deg(qx, qy, qz, qw):
    # angle between body z axis and world z axis
    zz = 1.0 - 2.0 * (qx * qx + qy * qy)
    return np.degrees(np.arccos(np.clip(zz, -1.0, 1.0)))


def score(run):
    meta = json.load(open(os.path.join(run, "meta.json")))
    if meta["status"] != "done":
        # gzserver crashed. Before the first command this is a start-up fault and
        # the run is repeated; after it the run counts as failed.
        startup = meta.get("sim_end", 0.0) < meta["t_cmd"]
        nan = float("nan")
        return dict(arm=meta["arm"], seed=meta["seed"], status=meta["status"], rtf=meta.get("rtf"),
                    startup_fault=startup, crashed=not startup, success=False, exit1=False,
                    entry=False, exit2=False, diverged=True, veto=0, max_tilt=nan,
                    rms_hover=nan, n_cross=0, peak_az=nan, peak_dfz=nan, mt_dt=nan,
                    sim_end=meta.get("sim_end"))
    _, od = load(os.path.join(run, "odom.csv"))
    o = np.array(od, float)
    t, x, y, z = o[:, 0], o[:, 1], o[:, 2], o[:, 3]
    v = o[:, 4:7]
    tr = t - meta["t_cmd"]
    zt, x0, y0 = meta["z_target"], meta["x0"], meta["y0"]
    horiz = np.hypot(x - x0, y - y0)
    tilt = tilt_deg(o[:, 7], o[:, 8], o[:, 9], o[:, 10])

    def hover(a, b):
        m = (tr >= a) & (tr <= b)
        return bool(m.any() and np.all(np.abs(z[m] - zt) <= 0.2) and np.all(horiz[m] <= 0.3))

    with open_text(os.path.join(run, "launch.log"), errors="ignore") as f:
        log = f.read()
    veto = log.count("Skipping force")
    after = tr >= 0
    finite = bool(np.all(np.isfinite(o)))
    diverged = (not finite) or bool(np.nanmax(np.linalg.norm(v, axis=1)) >= 10.0) \
        or bool(np.nanmax(tilt[after]) >= 60.0) or veto > 0
    entry = bool(np.any(z[(tr >= 10) & (tr < 18)] < 0.0))
    e1, e2 = hover(7.0, 10.0), hover(25.0, 28.0)

    hov = ((tr >= 5) & (tr < 10)) | ((tr >= 23) & (tr <= 28))
    rms = float(np.sqrt(np.mean((z[hov] - zt) ** 2)))

    # vertical acceleration from the ground-truth velocity over 10 ms
    k = max(1, int(round(0.01 / np.median(np.diff(t)))))
    az = (v[k:, 2] - v[:-k, 2]) / (t[k:] - t[:-k])
    ta = 0.5 * (t[k:] + t[:-k])
    # surface crossings of the body center (mean water level)
    s = np.sign(z)
    cross = t[1:][(s[1:] != s[:-1]) & (t[1:] > meta["t_cmd"])]

    _, mt = load(os.path.join(run, "mt.csv"))
    m = np.array(mt, float)
    tm = m[:, 0]
    fz = m[:, 1] + m[:, 4] + m[:, 5] + m[:, 6]   # buoyancy + damping + wave + heave
    dfz = np.abs(np.diff(fz))
    tdf = tm[1:]
    pk_a, pk_f = 0.0, 0.0
    for c in cross:
        wa = np.abs(ta - c) <= 0.5
        wf = np.abs(tdf - c) <= 0.5
        if wa.any():
            pk_a = max(pk_a, float(np.max(np.abs(az[wa]))))
        if wf.any():
            pk_f = max(pk_f, float(np.max(dfz[wf])))
    return dict(arm=meta["arm"], seed=meta["seed"], status=meta["status"], rtf=meta.get("rtf"),
                startup_fault=False, crashed=False,
                success=bool(e1 and entry and e2 and not diverged), exit1=e1, entry=entry,
                exit2=e2, diverged=diverged, veto=veto, max_tilt=float(np.nanmax(tilt[after])),
                rms_hover=rms, n_cross=int(len(cross)), peak_az=pk_a, peak_dfz=pk_f,
                mt_dt=float(np.median(np.diff(tm))))


runs = sorted(glob.glob(os.path.join(root, "psc*_*")))
rows = [score(r) for r in runs if os.path.exists(os.path.join(r, "meta.json"))]
faults = [(r["arm"], r["seed"]) for r in rows if r["startup_fault"]]
rows = [r for r in rows if not r["startup_fault"]]
by = {a: {r["seed"]: r for r in rows if r["arm"] == a} for a in ("psc", "psc_rs")}
seeds = sorted(set(by["psc"]) & set(by["psc_rs"]))


def ci(k, n):
    lo = stats.beta.ppf(0.025, k, n - k + 1) if k > 0 else 0.0
    hi = stats.beta.ppf(0.975, k + 1, n - k) if k < n else 1.0
    return [float(lo), float(hi)]


summary = {"n_pairs": len(seeds), "startup_faults_excluded": faults}
for a in ("psc", "psc_rs"):
    rs = [by[a][s] for s in seeds]
    k = sum(r["success"] for r in rs)
    summary[a] = dict(success=k, success_ci95=ci(k, len(rs)),
                      diverged=sum(r["diverged"] for r in rs),
                      crashed=sum(r["crashed"] for r in rs),
                      **{f"{m}_median": float(np.nanmedian([r[m] for r in rs]))
                         for m in ("rms_hover", "peak_az", "peak_dfz", "max_tilt")},
                      **{f"{m}_mean": float(np.nanmean([r[m] for r in rs]))
                         for m in ("rms_hover", "peak_az", "peak_dfz", "max_tilt")},
                      rtf_min=float(min(r["rtf"] for r in rs if not r["crashed"])))
for m in ("rms_hover", "peak_az", "peak_dfz"):
    a = np.array([by["psc"][s][m] for s in seeds])
    b = np.array([by["psc_rs"][s][m] for s in seeds])
    ok = np.isfinite(a) & np.isfinite(b)
    a, b = a[ok], b[ok]
    w = stats.wilcoxon(a, b) if len(a) > 5 and np.any(a != b) else None
    summary[f"paired_{m}"] = dict(n=int(len(a)), psc_lower_in=int(np.sum(a < b)),
                                  p_wilcoxon=float(w.pvalue) if w else None,
                                  median_ratio_rs_over_psc=float(np.median(b / np.maximum(a, 1e-9))))
json.dump(dict(summary=summary, runs=rows), open(out_path, "w"), indent=1)
print(json.dumps(summary, indent=1))

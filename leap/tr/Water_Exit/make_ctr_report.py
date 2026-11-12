#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_ctr_report.py
==================================================================
Standalone TEST pipeline for the reviewer rebuttal (R1#2 / R1#3 / AE),
TRANSMEDIA RESISTANCE / WATER-EXIT version.

Goal: on the Water-Exit drag-coefficient dataset
    (z_over_l, velocity v -> CTR),
compare FOUR models on a held-out test set and report
    RMSE, Max Abs Error, single-inference + 600-pt curve time.

  Baseline 1  Simple Surrogate   : degree-d 2-D polynomial regression
  Baseline 2  LEAP w/o residual  : physical backbone f_phys only (eps_SR = 0)
  Baseline 3  RBF interpolation  : thin-plate-spline (accurate but O(N) / slow)
  Proposed    Full LEAP          : f_phys + eps_SR

This is the 2-input / single-target sibling of make_nse_report.py. It DOES NOT
modify or import the training code. Backbone, RBF and the polynomial surrogate
are re-fit on the TRAIN split only (no leakage). The symbolic residual eps_SR
keeps the PySR-discovered FORM (read as a STRING from hall_of_fame.csv) but its
CONSTANTS are re-fit on the train split's backbone residual (leakage-free);
the form is chosen by an inner validation split.

Runs on the base python3 (numpy / pandas / scipy / sympy).

Usage
-----
    python3 make_ctr_report.py                     # random split, 80/20
    python3 make_ctr_report.py --split extrapolation
    python3 make_ctr_report.py --root /path/to/Water_Exit --outdir ./CTR_reports/random
"""
from __future__ import annotations
import argparse, json, time
from pathlib import Path

import numpy as np
import pandas as pd
import sympy as sp
from scipy.optimize import curve_fit, least_squares
from scipy.interpolate import RBFInterpolator

import warnings
warnings.filterwarnings("ignore")
np.seterr(all="ignore")

# deployed piecewise boundary on z_over_l (= median of the full dataset)
DEFAULT_BOUNDARY = 0.4112672070690474


# ==================================================================
# 1.  Physical backbone  (verbatim form from train_ctr_two_var_piecewise.py)
#       y = 1 + A1*exp(-B1*x) + A2*exp(-B2*x) + A3/(1+x)^n
#       A1=A10+A11*v, A2=A20+A21*v, A3=A30+A31*v, B1=B10+B11*v, B2=B20+B21*v
#     11 params / segment ; x = z_over_l, v = velocity
# ==================================================================
def phys2(inputs, A10, A11, A20, A21, A30, A31, B10, B11, B20, B21, n):
    x, v = inputs
    A1 = A10 + A11 * v
    A2 = A20 + A21 * v
    A3 = A30 + A31 * v
    B1 = B10 + B11 * v
    B2 = B20 + B21 * v
    wall = A3 / np.power(1.0 + x, n)
    return 1.0 + A1 * np.exp(-B1 * x) + A2 * np.exp(-B2 * x) + wall

_lhs = lambda th: th[:11]
_rhs = lambda th: th[11:]

def predict_piecewise(x, v, theta, boundary):
    x = np.asarray(x, float); v = np.asarray(v, float)
    mask = x <= boundary
    out = np.empty_like(x)
    if mask.any():
        out[mask] = phys2((x[mask], v[mask]), *_lhs(theta))
    if (~mask).any():
        out[~mask] = phys2((x[~mask], v[~mask]), *_rhs(theta))
    return out

def _fd_deriv(x0, v0, theta, side, h=1e-6):
    part = _lhs(theta) if side == "left" else _rhs(theta)
    fp = phys2((np.array([x0 + h]), np.array([v0])), *part)[0]
    fm = phys2((np.array([x0 - h]), np.array([v0])), *part)[0]
    return (fp - fm) / (2 * h)

def _residual(theta, x, v, y, vlvls, boundary, lc0, lc1, enforce_c1):
    res_data = predict_piecewise(x, v, theta, boundary) - y
    res_c0, res_c1 = [], []
    for vv in vlvls:
        l = phys2((np.array([boundary]), np.array([vv])), *_lhs(theta))[0]
        rr = phys2((np.array([boundary]), np.array([vv])), *_rhs(theta))[0]
        res_c0.append(l - rr)
        if enforce_c1:
            res_c1.append(_fd_deriv(boundary, vv, theta, "left")
                          - _fd_deriv(boundary, vv, theta, "right"))
    out = [res_data, np.array(res_c0) * np.sqrt(lc0)]
    if enforce_c1:
        out.append(np.array(res_c1) * np.sqrt(lc1))
    return np.concatenate(out)

def fit_backbone(x, v, y, boundary=DEFAULT_BOUNDARY, lc0=500., lc1=100., enforce_c1=True):
    """Re-fit the piecewise physical backbone on the given (train) split."""
    p0 = [0.1] * 10 + [2.0]
    lb = [0.0] * 10 + [1.0]
    ub = [np.inf] * 10 + [8.0]
    mL, mR = x <= boundary, x >= boundary
    th_l, _ = curve_fit(phys2, (x[mL], v[mL]), y[mL], p0=p0, bounds=(lb, ub), maxfev=30000)
    th_r, _ = curve_fit(phys2, (x[mR], v[mR]), y[mR], p0=p0, bounds=(lb, ub), maxfev=30000)
    th0 = np.concatenate([th_l, th_r])
    opt = least_squares(_residual, th0, max_nfev=40000,
                        kwargs=dict(x=x, v=v, y=y, vlvls=np.unique(v),
                                    boundary=boundary, lc0=lc0, lc1=lc1,
                                    enforce_c1=enforce_c1))
    return opt.x


# ==================================================================
# 2.  Symbolic residual eps_SR  (FORM from hall_of_fame.csv, constants re-fit)
#     variables: x0 = z_over_l, x1 = velocity
# ==================================================================
_SYMS = sp.symbols("x0 x1")

def _parse_symbolic(eq: str):
    """Parse a PySR equation string into (sympy_expr, [const_symbols], [init_values]).
    Numeric literals become free parameters c0,c1,... to be re-fit on the train split."""
    x0, x1 = _SYMS
    expr = sp.sympify(eq.replace("^", "**"), locals={"x0": x0, "x1": x1})
    consts = sorted(expr.atoms(sp.Float), key=lambda c: float(c))
    csyms, cinit, subs = [], [], {}
    for i, c in enumerate(consts):
        s = sp.Symbol(f"c{i}")
        csyms.append(s); cinit.append(float(c)); subs[c] = s
    expr_p = expr.xreplace(subs)
    return expr_p, csyms, np.array(cinit, float)


def fit_symbolic_residual(hof_csv, x, v, resid, seed=0,
                          comp_lo=4, comp_hi=24, val_frac=0.25):
    """Keep the PySR-discovered symbolic FORMS, re-fit their constants on the train
    backbone-residual, and select the form by an inner validation split. Leakage-free
    (only the symbolic structure carries any full-data information).
    Returns (callable f(x,v)->residual, eq_string, info_dict). Falls back to 0."""
    df = pd.read_csv(hof_csv).sort_values("Complexity").reset_index(drop=True)
    cand = df[(df.Complexity >= comp_lo) & (df.Complexity <= comp_hi)]

    rng = np.random.default_rng(seed)
    n = len(x); idx = rng.permutation(n); cut = int(round(n * (1 - val_frac)))
    tr, va = idx[:cut], idx[cut:]
    Xtr = (x[tr], v[tr]); Xva = (x[va], v[va])
    best = dict(val=float(np.sqrt(np.mean(resid[va] ** 2))), f=lambda xx, vv: 0.0,
                eq="0", refit=None, complexity=0)   # zero-residual baseline

    for _, row in cand.iterrows():
        eq = str(row["Equation"])
        try:
            expr_p, csyms, cinit = _parse_symbolic(eq)
            if len(csyms) == 0:
                continue
            f_lam = sp.lambdify((_SYMS[0], _SYMS[1], *csyms), expr_p, "numpy")

            def model(c, X):
                out = np.asarray(f_lam(X[0], X[1], *c), float)
                out = np.broadcast_to(out, X[0].shape).astype(float).copy()
                out[~np.isfinite(out)] = 0.0
                return out

            sol = least_squares(lambda c: model(c, Xtr) - resid[tr], cinit,
                                loss="soft_l1", max_nfev=4000)
            # reject numerically degenerate forms (negative base ** real power -> NaN)
            full_finite = np.isfinite(np.asarray(
                f_lam(x, v, *sol.x), float)).mean()
            if full_finite < 0.98:
                continue
            vpred = model(sol.x, Xva)
            vrmse = float(np.sqrt(np.mean((vpred - resid[va]) ** 2)))
            if vrmse < best["val"]:
                cfit = sol.x
                f_full = sp.lambdify((_SYMS[0], _SYMS[1], *csyms), expr_p, "numpy")
                eq_refit = str(expr_p.xreplace({s: float(vv) for s, vv in zip(csyms, cfit)}))
                def make_f(fl, c):
                    def f(xx, vv):
                        xx = np.atleast_1d(np.asarray(xx, float))
                        out = np.asarray(fl(xx, np.atleast_1d(np.asarray(vv, float)), *c), float)
                        out = np.broadcast_to(out, xx.shape).astype(float).copy()
                        out[~np.isfinite(out)] = 0.0
                        return out
                    return f
                best = dict(val=vrmse, f=make_f(f_full, cfit), eq=eq_refit,
                            refit=cfit.tolist(), complexity=int(row["Complexity"]))
        except Exception:
            continue
    return best["f"], best["eq"], dict(val_rmse=best["val"], complexity=best["complexity"])


# ==================================================================
# 3.  Simple polynomial surrogate  (numpy lstsq, no sklearn)
# ==================================================================
from itertools import combinations_with_replacement

def _poly_design(X, degree):
    n = X.shape[0]
    cols = [np.ones(n)]
    idx = range(X.shape[1])
    for d in range(1, degree + 1):
        for combo in combinations_with_replacement(idx, d):
            term = np.ones(n)
            for j in combo:
                term = term * X[:, j]
            cols.append(term)
    return np.column_stack(cols)

class PolySurrogate:
    def __init__(self, degree=4): self.degree = degree
    def fit(self, X, y):
        A = _poly_design(X, self.degree)
        self.coef_, *_ = np.linalg.lstsq(A, y, rcond=None)
        return self
    def predict(self, X):
        return _poly_design(np.atleast_2d(X), self.degree) @ self.coef_


# ==================================================================
# 4.  Metrics & timing
# ==================================================================
def rmse(yt, yp):    return float(np.sqrt(np.mean((yt - yp) ** 2)))
def maxerr(yt, yp):  return float(np.max(np.abs(yt - yp)))

# query ranges for timing (z_over_l in [0,1], velocity in [0.1,0.5])
_Q_X = (0.005, 1.0)
_Q_V = (0.1, 0.5)

def single_inference_us(predict_one, n=2000):
    """Median wall-time of a single-point prediction, microseconds. Python call
    overhead dominates at this scale for all models; curve throughput below is the
    meaningful timing metric."""
    rng = np.random.default_rng(0)
    xs = rng.uniform(*_Q_X, n); vs = rng.uniform(*_Q_V, n)
    ts = np.empty(n)
    for i in range(n):
        t0 = time.perf_counter()
        predict_one(xs[i], vs[i])
        ts[i] = (time.perf_counter() - t0) * 1e6
    return float(np.median(ts))

def curve_time_ms(predict, npts=600, reps=20):
    """Wall-time to predict an npts-point curve (paper regime). RBF is O(npts*N),
    LEAP is O(npts): where the O(1)-vs-O(N) advantage is visible."""
    x = np.linspace(*_Q_X, npts); v = np.full(npts, 0.2)
    predict(x, v)  # warmup
    t0 = time.perf_counter()
    for _ in range(reps):
        predict(x, v)
    return float((time.perf_counter() - t0) / reps * 1e3)


# ==================================================================
# 5.  Train/test split
# ==================================================================
def make_split(df, kind, test_size, seed):
    rng = np.random.default_rng(seed)
    n = len(df)
    if kind == "random":
        idx = rng.permutation(n)
        cut = int(round(n * (1 - test_size)))
        return df.iloc[idx[:cut]].copy(), df.iloc[idx[cut:]].copy()
    elif kind == "extrapolation":
        # hold out the sparse far end (largest z_over_l) -> tests extrapolation
        order = np.argsort(df["z_over_l"].to_numpy())
        cut = int(round(n * (1 - test_size)))
        return df.iloc[order[:cut]].copy(), df.iloc[order[cut:]].copy()
    raise ValueError(kind)


# ==================================================================
# 6.  Build the four predictors
# ==================================================================
def build_models(train, hof_csv, degree, boundary, seed=0):
    import pickle
    x = train["z_over_l"].to_numpy(float); v = train["velocity"].to_numpy(float)
    y = train["CTR"].to_numpy(float)
    Xtr = np.column_stack([x, v])

    # Baseline 2: backbone (re-fit on train)
    theta = fit_backbone(x, v, y, boundary=boundary)

    # Proposed: residual eps_SR (form from hall_of_fame, constants re-fit on train)
    resid = y - predict_piecewise(x, v, theta, boundary)
    f_resid, eq, rinfo = fit_symbolic_residual(hof_csv, x, v, resid, seed=seed)

    # Baseline 1: polynomial surrogate
    poly = PolySurrogate(degree).fit(Xtr, y)

    # Baseline 3: RBF interpolation (time the O(N) construction = "cold start")
    t0 = time.perf_counter()
    rbf = RBFInterpolator(Xtr, y, kernel="thin_plate_spline", smoothing=1e-4)
    rbf_build_ms = (time.perf_counter() - t0) * 1e3
    rbf_blob = pickle.dumps(rbf)
    t0 = time.perf_counter(); _ = pickle.loads(rbf_blob); rbf_load_ms = (time.perf_counter() - t0) * 1e3

    def f_backbone(xx, vv):
        return predict_piecewise(np.atleast_1d(xx), np.atleast_1d(vv), theta, boundary)
    def f_leap(xx, vv):
        return f_backbone(xx, vv) + f_resid(xx, vv)
    def f_poly(xx, vv):
        return poly.predict(np.column_stack([np.atleast_1d(xx), np.atleast_1d(vv)]))
    def f_rbf(xx, vv):
        return rbf(np.column_stack([np.atleast_1d(xx), np.atleast_1d(vv)]))

    models = {
        "Simple surrogate (poly d%d)" % degree: f_poly,
        "LEAP w/o residual":                     f_backbone,
        "RBF interpolation":                     f_rbf,
        "Full LEAP (proposed)":                  f_leap,
    }
    coldstart = dict(
        rbf_model_bytes=len(rbf_blob), rbf_build_ms=rbf_build_ms, rbf_load_ms=rbf_load_ms,
        rbf_n_centers=len(x),
        leap_model_bytes=len(json.dumps({"theta": theta.tolist(), "residual": eq}).encode()),
        leap_build_ms=0.0, leap_load_ms=0.0)
    meta = dict(residual_eq=eq, residual_val_rmse=rinfo["val_rmse"],
                residual_complexity=rinfo["complexity"], theta=theta.tolist(),
                coldstart=coldstart)
    return models, meta


# ==================================================================
# 7.  Main
# ==================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=str(Path(__file__).resolve().parent),
                    help="experiment folder (contains data/merged_rows.csv + outputs/)")
    ap.add_argument("--degree", type=int, default=4)
    ap.add_argument("--boundary", type=float, default=DEFAULT_BOUNDARY)
    ap.add_argument("--test-size", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--split", default="random", choices=["random", "extrapolation"])
    ap.add_argument("--outdir", default=None, help="output directory (default <root>/ctr_report)")
    args = ap.parse_args()

    root = Path(args.root)
    df = pd.read_csv(root / "data" / "merged_rows.csv").dropna()
    out_dir = Path(args.outdir) if args.outdir else root / "ctr_report"
    out_dir.mkdir(parents=True, exist_ok=True)

    # residual form: the hall_of_fame.csv named in residual_source.json (the file behind Table IV);
    # without that file, the newest hall_of_fame.csv by mtime (CTR is the single target)
    src = root / "residual_source.json"
    if src.exists():
        hof = root / json.loads(src.read_text())["CTR"]
    else:
        hofs = sorted((root / "outputs").glob("*/hall_of_fame.csv"), key=lambda p: p.stat().st_mtime)
        if not hofs:
            raise FileNotFoundError("no outputs/*/hall_of_fame.csv found for the residual")
        hof = hofs[-1]

    train, test = make_split(df, args.split, args.test_size, args.seed)
    print(f"\n[data] {len(df)} rows -> {len(train)} train / {len(test)} test  "
          f"(split={args.split}, seed={args.seed})\n")

    models, meta = build_models(train, hof, args.degree, args.boundary, seed=args.seed)
    yt = test["CTR"].to_numpy(float)
    Xte, Vte = test["z_over_l"].to_numpy(float), test["velocity"].to_numpy(float)
    rows = []
    for name, f in models.items():
        yp = np.asarray(f(Xte, Vte), float)
        rows.append(dict(model=name, RMSE=rmse(yt, yp), MaxErr=maxerr(yt, yp),
                         t_inf_us=single_inference_us(f), t_curve_ms=curve_time_ms(f)))

    cs = meta["coldstart"]
    report = dict(root=str(root), split=args.split, test_size=args.test_size,
                  seed=args.seed, n_train=len(train), n_test=len(test),
                  poly_degree=args.degree, boundary=args.boundary,
                  mode="refit_constants_hof", target="CTR",
                  residual_eq=meta["residual_eq"],
                  residual_complexity=meta["residual_complexity"],
                  residual_val_rmse=meta["residual_val_rmse"],
                  coldstart=cs, metrics=rows)

    print(f"==== target CTR  (re-fit eps_SR = {meta['residual_eq']}) ====")
    print(f"{'model':32s} {'RMSE':>10s} {'MaxErr':>10s} {'t_single(us)':>13s} {'t_curve600(ms)':>15s}")
    for m in rows:
        print(f"{m['model']:32s} {m['RMSE']:10.4f} {m['MaxErr']:10.4f} "
              f"{m['t_inf_us']:13.2f} {m['t_curve_ms']:15.3f}")
    print(f"  [cold start]  RBF: {cs['rbf_model_bytes']/1024:.1f} KB model, "
          f"{cs['rbf_build_ms']:.1f} ms build / {cs['rbf_load_ms']:.2f} ms load, "
          f"{cs['rbf_n_centers']} centers (O(N)/query)")
    print(f"                LEAP: {cs['leap_model_bytes']} B model (equation, O(1)/query)\n")

    (out_dir / "metrics.json").write_text(json.dumps(report, indent=2, ensure_ascii=False))
    pd.DataFrame([dict(target="CTR", **m) for m in rows]).to_csv(out_dir / "metrics.csv", index=False)
    print(f"[done] wrote: {out_dir}/metrics.json, metrics.csv")


if __name__ == "__main__":
    main()

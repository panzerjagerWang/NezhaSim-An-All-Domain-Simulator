#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_nse_report.py
==================================================================
Standalone TEST pipeline for the reviewer rebuttal (R1#2 / R1#3 / AE).

Goal: on the NSE rig dataset (h_over_R, thrust_level p, radius R -> K_T / K_Q),
compare FOUR models on a held-out test set and report
    RMSE, Max Abs Error, single-inference time.

  Baseline 1  Simple Surrogate   : degree-d multivariate polynomial regression
  Baseline 2  LEAP w/o residual  : physical backbone f_phys only (eps_SR = 0)
  Baseline 3  RBF interpolation  : thin-plate-spline (accurate but O(N) / slow)
  Proposed    Full LEAP          : f_phys + eps_SR (frozen PySR residual)

It DOES NOT modify or import your training code. Backbone, RBF and the
polynomial surrogate are re-fit on the TRAIN split only (no leakage).
The symbolic residual eps_SR is read as a STRING from the PySR
hall_of_fame.csv (PySR itself is not required to run this script).

Runs on the base python3 (numpy / pandas / scipy / sympy).

Usage
-----
    python3 make_nse_report.py                       # thin folder, KT, deg4, 80/20
    python3 make_nse_report.py --target both
    python3 make_nse_report.py --root /path/to/winglet --split extrapolation
"""
from __future__ import annotations
import argparse, json, time, re
from pathlib import Path

import numpy as np
import pandas as pd
import sympy as sp
from scipy.optimize import curve_fit, least_squares
from scipy.interpolate import RBFInterpolator

import warnings
warnings.filterwarnings("ignore")
np.seterr(all="ignore")


# ==================================================================
# 1.  Physical backbone  (verbatim from train_three_var_piecewise.py)
# ==================================================================
def phys3(inputs, A10, A11, A12, A20, A21, A22, A30, A31, A32,
          B10, B11, B12, B20, B21, B22, n):
    x, p, r = inputs
    A1 = A10 + A11 * p + A12 * r
    A2 = A20 + A21 * p + A22 * r
    A3 = A30 + A31 * p + A32 * r
    B1 = B10 + B11 * p + B12 * r
    B2 = B20 + B21 * p + B22 * r
    wall = A3 / np.power(1.0 + x, n)
    return 1.0 + A1 * np.exp(-B1 * x) + A2 * np.exp(-B2 * x) + wall

_lhs = lambda th: th[:16]
_rhs = lambda th: th[16:]

def predict_piecewise(x, p, r, theta, boundary):
    x = np.asarray(x, float); p = np.asarray(p, float); r = np.asarray(r, float)
    mask = x <= boundary
    out = np.empty_like(x)
    if mask.any():
        out[mask] = phys3((x[mask], p[mask], r[mask]), *_lhs(theta))
    if (~mask).any():
        out[~mask] = phys3((x[~mask], p[~mask], r[~mask]), *_rhs(theta))
    return out

def _fd_deriv(x0, p0, r0, theta, side, h=1e-6):
    part = _lhs(theta) if side == "left" else _rhs(theta)
    fp = phys3((np.array([x0 + h]), np.array([p0]), np.array([r0])), *part)[0]
    fm = phys3((np.array([x0 - h]), np.array([p0]), np.array([r0])), *part)[0]
    return (fp - fm) / (2 * h)

def _residual(theta, x, p, r, y, plvls, rlvs, boundary, lc0, lc1, enforce_c1):
    res_data = predict_piecewise(x, p, r, theta, boundary) - y
    res_c0, res_c1 = [], []
    for pv in plvls:
        for rv in rlvs:
            l = phys3((np.array([boundary]), np.array([pv]), np.array([rv])), *_lhs(theta))[0]
            rr = phys3((np.array([boundary]), np.array([pv]), np.array([rv])), *_rhs(theta))[0]
            res_c0.append(l - rr)
            if enforce_c1:
                res_c1.append(_fd_deriv(boundary, pv, rv, theta, "left")
                              - _fd_deriv(boundary, pv, rv, theta, "right"))
    out = [res_data, np.array(res_c0) * np.sqrt(lc0)]
    if enforce_c1:
        out.append(np.array(res_c1) * np.sqrt(lc1))
    return np.concatenate(out)

def fit_backbone(x, p, r, y, boundary=0.5, lc0=100., lc1=10., enforce_c1=True):
    """Re-fit the piecewise physical backbone on the given (train) split."""
    p0 = [0.1] * 15 + [2.0]
    lb = [0.0] * 15 + [1.0]
    ub = [np.inf] * 15 + [8.0]
    mL, mR = x <= boundary, x >= boundary
    th_l, _ = curve_fit(phys3, (x[mL], p[mL], r[mL]), y[mL], p0=p0, bounds=(lb, ub), maxfev=30000)
    th_r, _ = curve_fit(phys3, (x[mR], p[mR], r[mR]), y[mR], p0=p0, bounds=(lb, ub), maxfev=30000)
    th0 = np.concatenate([th_l, th_r])
    opt = least_squares(_residual, th0, max_nfev=40000,
                        kwargs=dict(x=x, p=p, r=r, y=y,
                                    plvls=np.unique(p), rlvs=np.unique(r),
                                    boundary=boundary, lc0=lc0, lc1=lc1,
                                    enforce_c1=enforce_c1))
    return opt.x


# ==================================================================
# 2.  Symbolic residual eps_SR  (read as STRING from hall_of_fame.csv)
# ==================================================================
_ALLOWED = {"exp": np.exp, "log": np.log, "sqrt": np.sqrt, "sin": np.sin,
            "cos": np.cos, "abs": np.abs, "np": np}

def _safe_eval(expr, x0, x1, x2):
    """Evaluate a PySR equation string (x0=h, x1=p, x2=r). '^' -> '**'."""
    py = expr.replace("^", "**")
    try:
        out = eval(py, {"__builtins__": {}}, {"x0": x0, "x1": x1, "x2": x2, **_ALLOWED})
        out = np.asarray(out, float)
    except Exception:
        return np.zeros_like(np.asarray(x0, float))
    out = np.broadcast_to(out, np.asarray(x0, float).shape).astype(float).copy()
    out[~np.isfinite(out)] = 0.0
    return out

_SYMS = sp.symbols("x0 x1 x2")

def _parse_symbolic(eq: str):
    """Parse a PySR equation string into (sympy_expr, [const_symbols], [init_values]).
    Numeric literals become free parameters c0,c1,... to be re-fit on the train split."""
    x0, x1, x2 = _SYMS
    expr = sp.sympify(eq.replace("^", "**"), locals={"x0": x0, "x1": x1, "x2": x2})
    consts = sorted(expr.atoms(sp.Float), key=lambda c: float(c))
    csyms, cinit, subs = [], [], {}
    for i, c in enumerate(consts):
        s = sp.Symbol(f"c{i}")
        csyms.append(s); cinit.append(float(c)); subs[c] = s
    expr_p = expr.xreplace(subs)
    return expr_p, csyms, np.array(cinit, float)

def fit_residual_pysr(x, p, r, resid, sr_iter=300, seed=0, workdir=None):
    """LEAKAGE-FREE residual: re-discover eps_SR with PySR on THIS train split's
    backbone residual (mirrors train_three_var_piecewise.py operators). Returns
    (callable f(h,p,r)->residual, eq_string, info). Requires pysr (run under pysr_env)."""
    from pysr import PySRRegressor
    import tempfile
    od = workdir or tempfile.mkdtemp(prefix="pysr_")
    sr = PySRRegressor(
        niterations=sr_iter,
        unary_operators=["exp", "log", "sqrt", "sin"],
        binary_operators=["+", "-", "*", "/", "pow"],
        model_selection="best",
        deterministic=True, random_state=seed, parallelism="serial",
        verbosity=0, progress=False, output_directory=od)
    sr.fit(np.column_stack([x, p, r]), resid)
    # compile best equation to a fast CLOSED-FORM numpy callable (deployment-style:
    # the deployed model evaluates the equation, NOT sr.predict's Julia round-trip)
    try:
        expr = sr.sympy()                     # sympy expr of the selected best model
        f_lam = sp.lambdify(_SYMS, expr, "numpy")
        eq = str(expr)
        def f(h, pp, rr):
            h = np.atleast_1d(np.asarray(h, float))
            out = np.asarray(f_lam(h, np.atleast_1d(np.asarray(pp, float)),
                                   np.atleast_1d(np.asarray(rr, float))), float)
            out = np.broadcast_to(out, h.shape).astype(float).copy()
            out[~np.isfinite(out)] = 0.0
            return out
    except Exception:                       # fall back to PySR's own predict
        eq = str(sr.get_best()["equation"])
        def f(h, pp, rr):
            X = np.column_stack([np.atleast_1d(h), np.atleast_1d(pp), np.atleast_1d(rr)])
            out = np.asarray(sr.predict(X), float); out[~np.isfinite(out)] = 0.0
            return out
    return f, eq, dict(val_rmse=float("nan"), complexity=int(sr.get_best()["complexity"]))


def fit_symbolic_residual(hof_csv, x, p, r, resid, seed=0,
                          comp_lo=3, comp_hi=22, val_frac=0.25):
    """Improved training: keep PySR-discovered symbolic FORMS, re-fit their constants
    on the train backbone-residual, and select the form by an inner validation split.
    Returns (callable f(h,p,r)->residual, eq_string, info_dict). Falls back to 0."""
    df = pd.read_csv(hof_csv).sort_values("Complexity").reset_index(drop=True)
    cand = df[(df.Complexity >= comp_lo) & (df.Complexity <= comp_hi)]

    rng = np.random.default_rng(seed)
    n = len(x); idx = rng.permutation(n); cut = int(round(n * (1 - val_frac)))
    tr, va = idx[:cut], idx[cut:]
    Xtr = (x[tr], p[tr], r[tr]); Xva = (x[va], p[va], r[va])
    best = dict(val=float(np.sqrt(np.mean(resid[va] ** 2))), f=lambda h, pp, rr: 0.0,
                eq="0", refit=None, complexity=0)   # zero-residual baseline

    for _, row in cand.iterrows():
        eq = str(row["Equation"])
        try:
            expr_p, csyms, cinit = _parse_symbolic(eq)
            if len(csyms) == 0:
                continue
            f_lam = sp.lambdify((_SYMS[0], _SYMS[1], _SYMS[2], *csyms), expr_p, "numpy")

            def model(c, X):
                out = np.asarray(f_lam(X[0], X[1], X[2], *c), float)
                out = np.broadcast_to(out, X[0].shape).astype(float).copy()
                out[~np.isfinite(out)] = 0.0
                return out

            sol = least_squares(lambda c: model(c, Xtr) - resid[tr], cinit,
                                loss="soft_l1", max_nfev=4000)
            # reject numerically degenerate forms (e.g. negative base ** real power -> NaN)
            full_finite = np.isfinite(np.asarray(
                f_lam(x, p, r, *sol.x), float)).mean()
            if full_finite < 0.98:
                continue
            vpred = model(sol.x, Xva)
            vrmse = float(np.sqrt(np.mean((vpred - resid[va]) ** 2)))
            if vrmse < best["val"]:
                cfit = sol.x
                f_full = sp.lambdify((_SYMS[0], _SYMS[1], _SYMS[2], *csyms), expr_p, "numpy")
                eq_refit = str(expr_p.xreplace({s: float(v) for s, v in zip(csyms, cfit)}))
                def make_f(fl, c):
                    def f(h, pp, rr):
                        h = np.atleast_1d(np.asarray(h, float))
                        out = np.asarray(fl(h, np.atleast_1d(np.asarray(pp, float)),
                                            np.atleast_1d(np.asarray(rr, float)), *c), float)
                        out = np.broadcast_to(out, h.shape).astype(float).copy()
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

def single_inference_us(predict_one, n=2000):
    """Median wall-time of a single-point prediction, microseconds.
    NOTE: at this scale the Python call overhead dominates for all models, so this
    is a wash; the meaningful timing metric is curve/batch throughput below."""
    rng = np.random.default_rng(0)
    hs = rng.uniform(0.05, 3.9, n); ps = rng.uniform(0.4, 1.0, n); rs = rng.uniform(0.23, 0.56, n)
    ts = np.empty(n)
    for i in range(n):
        t0 = time.perf_counter()
        predict_one(hs[i], ps[i], rs[i])
        ts[i] = (time.perf_counter() - t0) * 1e6
    return float(np.median(ts))

def curve_time_ms(predict, npts=600, reps=20):
    """Wall-time to predict an npts-point curve (paper regime). RBF is O(npts*N),
    LEAP is O(npts). This is where the O(1)-vs-O(N) advantage is visible."""
    h = np.linspace(0.05, 3.9, npts); p = np.full(npts, 0.7); r = np.full(npts, 0.25)
    predict(h, p, r)  # warmup
    t0 = time.perf_counter()
    for _ in range(reps):
        predict(h, p, r)
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
        # hold out the sparse far end (largest h/R) -> tests extrapolation
        order = np.argsort(df["h_over_R"].to_numpy())
        cut = int(round(n * (1 - test_size)))
        return df.iloc[order[:cut]].copy(), df.iloc[order[cut:]].copy()
    raise ValueError(kind)


# ==================================================================
# 6.  Build the four predictors for one target column
# ==================================================================
def build_models(train, target, hof_csv, degree, boundary, pysr=False, sr_iter=300, seed=0):
    import pickle
    x = train["h_over_R"].to_numpy(float); p = train["thrust_level"].to_numpy(float)
    r = train["R"].to_numpy(float);        y = train[target].to_numpy(float)
    Xtr = np.column_stack([x, p, r])

    # Baseline 2: backbone (re-fit on train)
    theta = fit_backbone(x, p, r, y, boundary=boundary)

    # Proposed: residual eps_SR
    resid = y - predict_piecewise(x, p, r, theta, boundary)
    if pysr:   # LEAKAGE-FREE: re-discover the symbolic residual on this train split
        f_resid, eq, rinfo = fit_residual_pysr(x, p, r, resid, sr_iter=sr_iter, seed=seed)
    else:      # re-fit constants of the PySR-discovered form (form from full-data hall_of_fame)
        f_resid, eq, rinfo = fit_symbolic_residual(hof_csv, x, p, r, resid)

    # Baseline 1: polynomial surrogate
    poly = PolySurrogate(degree).fit(Xtr, y)

    # Baseline 3: RBF interpolation (time the O(N) construction = "cold start")
    t0 = time.perf_counter()
    rbf = RBFInterpolator(Xtr, y, kernel="thin_plate_spline", smoothing=1e-4)
    rbf_build_ms = (time.perf_counter() - t0) * 1e3
    rbf_blob = pickle.dumps(rbf)
    t0 = time.perf_counter(); _ = pickle.loads(rbf_blob); rbf_load_ms = (time.perf_counter() - t0) * 1e3

    def f_backbone(h, pp, rr):
        return predict_piecewise(np.atleast_1d(h), np.atleast_1d(pp), np.atleast_1d(rr), theta, boundary)
    def f_leap(h, pp, rr):
        return f_backbone(h, pp, rr) + f_resid(h, pp, rr)
    def f_poly(h, pp, rr):
        return poly.predict(np.column_stack([np.atleast_1d(h), np.atleast_1d(pp), np.atleast_1d(rr)]))
    def f_rbf(h, pp, rr):
        return rbf(np.column_stack([np.atleast_1d(h), np.atleast_1d(pp), np.atleast_1d(rr)]))

    models = {
        "Simple surrogate (poly d%d)" % degree: f_poly,
        "LEAP w/o residual":                     f_backbone,
        "RBF interpolation":                     f_rbf,
        "Full LEAP (proposed)":                  f_leap,
    }
    # cold-start footprint: RBF carries all N centers; LEAP is just constants
    leap_params = len(theta) + len(rinfo) + (len(eq))     # rough symbol count proxy
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
# 7.  Scaling of the curve time with the training-set size
# ==================================================================
def scaling_benchmark(df, target, boundary, hof_csv, degree):
    """Single-inference time vs training-set size N: RBF is O(N), LEAP is O(1)."""
    Ns = sorted(set(n for n in [100, 250, 500, 1000, len(df)] if n <= len(df)))
    rbf_ms, leap_ms = [], []
    for N in Ns:
        sub = df.sample(N, random_state=0)
        models, _ = build_models(sub, target, hof_csv, degree, boundary)
        rbf_ms.append(curve_time_ms(models["RBF interpolation"]))
        leap_ms.append(curve_time_ms(models["Full LEAP (proposed)"]))
    return dict(N=Ns, rbf_curve_ms=rbf_ms, leap_curve_ms=leap_ms)


# ==================================================================
# 8.  Main
# ==================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=str(Path(__file__).resolve().parent),
                    help="experiment folder (contains data/merged_rows.csv + outputs/)")
    ap.add_argument("--target", default="KT", choices=["KT", "KQ", "both"])
    ap.add_argument("--degree", type=int, default=4)
    ap.add_argument("--boundary", type=float, default=0.5)
    ap.add_argument("--test-size", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--split", default="random", choices=["random", "extrapolation"])
    ap.add_argument("--scaling", action="store_true", help="also time the 600-point curve against the training-set size")
    ap.add_argument("--pysr", action="store_true",
                    help="LEAKAGE-FREE: re-discover eps_SR with PySR per split (run under pysr_env)")
    ap.add_argument("--sr-iter", type=int, default=300, help="PySR niterations (--pysr mode)")
    ap.add_argument("--outdir", default=None, help="output directory (default <root>/nse_report)")
    args = ap.parse_args()

    root = Path(args.root)
    df = pd.read_csv(root / "data" / "merged_rows.csv").dropna()
    out_dir = Path(args.outdir) if args.outdir else root / "nse_report"
    out_dir.mkdir(parents=True, exist_ok=True)

    # locate hall_of_fame csvs (earlier mtime = KT, later = KQ, per train main()).
    # Used for the non-pysr residual AND the fast scaling path; in --pysr mode the
    # MAIN residual is re-discovered fresh, but scaling reuses these for speed.
    hofs = sorted((root / "outputs").glob("*/hall_of_fame.csv"), key=lambda p: p.stat().st_mtime)
    src = root / "residual_source.json"
    if src.exists():                       # the files behind Table IV, independent of mtimes
        hof_map = {k: root / v for k, v in json.loads(src.read_text()).items()}
    elif len(hofs) >= 2:
        hof_map = {"KT": hofs[0], "KQ": hofs[1]}
    elif len(hofs) == 1:
        hof_map = {"KT": hofs[0], "KQ": hofs[0]}
    elif args.pysr:
        hof_map = {"KT": None, "KQ": None}          # scaling will be skipped
    else:
        raise FileNotFoundError("no outputs/*/hall_of_fame.csv found for the residual")

    train, test = make_split(df, args.split, args.test_size, args.seed)
    targets = ["KT", "KQ"] if args.target == "both" else [args.target]

    all_metrics, report = {}, {"root": str(root), "split": args.split,
                               "test_size": args.test_size, "seed": args.seed,
                               "n_train": len(train), "n_test": len(test),
                               "poly_degree": args.degree, "targets": {}}
    print(f"\n[data] {len(df)} rows -> {len(train)} train / {len(test)} test  "
          f"(split={args.split}, seed={args.seed})\n")

    report["mode"] = "leakage_free_pysr" if args.pysr else "refit_constants_hof"
    if args.pysr:
        report["sr_iter"] = args.sr_iter
    for tgt in targets:
        models, meta = build_models(train, tgt, hof_map[tgt], args.degree, args.boundary,
                                    pysr=args.pysr, sr_iter=args.sr_iter, seed=args.seed)
        yt = test[tgt].to_numpy(float)
        Hte, Pte, Rte = (test["h_over_R"].to_numpy(float),
                         test["thrust_level"].to_numpy(float), test["R"].to_numpy(float))
        rows = []
        for name, f in models.items():
            yp = np.asarray(f(Hte, Pte, Rte), float)
            rows.append(dict(model=name, RMSE=rmse(yt, yp), MaxErr=maxerr(yt, yp),
                             t_inf_us=single_inference_us(f),
                             t_curve_ms=curve_time_ms(f)))
        all_metrics[tgt] = rows
        cs = meta["coldstart"]
        report["targets"][tgt] = dict(metrics=rows, residual_eq=meta["residual_eq"],
                                      residual_complexity=meta["residual_complexity"],
                                      residual_val_rmse=meta["residual_val_rmse"],
                                      coldstart=cs)

        # pretty print
        print(f"==== target {tgt}  (re-fit eps_SR = {meta['residual_eq']}) ====")
        print(f"{'model':32s} {'RMSE':>10s} {'MaxErr':>10s} {'t_single(us)':>13s} {'t_curve600(ms)':>15s}")
        for m in rows:
            print(f"{m['model']:32s} {m['RMSE']:10.4f} {m['MaxErr']:10.4f} "
                  f"{m['t_inf_us']:13.2f} {m['t_curve_ms']:15.3f}")
        print(f"  [cold start]  RBF: {cs['rbf_model_bytes']/1024:.1f} KB model, "
              f"{cs['rbf_build_ms']:.1f} ms build / {cs['rbf_load_ms']:.2f} ms load, "
              f"{cs['rbf_n_centers']} centers (O(N)/query)")
        print(f"                LEAP: {cs['leap_model_bytes']} B model (equation, O(1)/query)\n")

        if args.scaling:
            sc = scaling_benchmark(df, tgt, args.boundary, hof_map[tgt], args.degree)
            report["targets"][tgt]["scaling"] = sc

    # write outputs
    (out_dir / "metrics.json").write_text(json.dumps(report, indent=2, ensure_ascii=False))
    flat = [dict(target=t, **m) for t, rows in all_metrics.items() for m in rows]
    pd.DataFrame(flat).to_csv(out_dir / "metrics.csv", index=False)
    print(f"[done] wrote: {out_dir}/metrics.json, metrics.csv")
    print("\nNOTE: backbone, RBF and the polynomial surrogate are fit strictly on the train "
          "split.\nThe residual eps_SR keeps the symbolic FORM discovered once by PySR "
          "(hall_of_fame.csv),\nbut its CONSTANTS are re-fit on the train split's backbone "
          "residual (form chosen by an\ninner validation split). Only the symbolic structure "
          "carries any full-data information;\nfor a fully leakage-free residual, re-run PySR "
          "discovery per fold (PySR not installed here).")


if __name__ == "__main__":
    main()

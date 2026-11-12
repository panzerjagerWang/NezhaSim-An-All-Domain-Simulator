#!/usr/bin/env python3
# File: piecewise_phys_sr_ktkq.py
"""
3-input  →  2-output piece-wise physics + symbolic-regression model
------------------------------------------------------------------
• inputs : h_over_R (x), thrust_level (p), R (r)
• outputs: KT, KQ  ——  同一份 pkl
"""

import numpy as np
import pandas as pd
import pickle
from pathlib import Path
from scipy.optimize import curve_fit, least_squares

# 可选：关掉 PySR（速度快）
USE_SYMBOLIC = True
if USE_SYMBOLIC:
    from pysr import PySRRegressor

# ────────────────────────────────────────────────────────────────
# 1. 物理启发三层函数（与原版相同，只换了名字）
# ────────────────────────────────────────────────────────────────
def phys3(inputs,
          A10, A11, A12,
          A20, A21, A22,
          A30, A31, A32,
          B10, B11, B12,
          B20, B21, B22,
          n):
    x, p, r = inputs
    A1 = A10 + A11 * p + A12 * r
    A2 = A20 + A21 * p + A22 * r
    A3 = A30 + A31 * p + A32 * r
    B1 = B10 + B11 * p + B12 * r
    B2 = B20 + B21 * p + B22 * r
    wall = A3 / np.power(1.0 + x, n)
    return 1.0 + A1*np.exp(-B1*x) + A2*np.exp(-B2*x) + wall


# ────────────────────────────────────────────────────────────────
# 2. 通用工具：左右段、连续性残差
# ────────────────────────────────────────────────────────────────
def _lhs(theta):   return theta[:16]
def _rhs(theta):   return theta[16:]

def predict_piecewise(x, p, r, theta, boundary):
    mask = x <= boundary
    out = np.empty_like(x)
    if mask.any():
        out[mask] = phys3((x[mask], p[mask], r[mask]), *_lhs(theta))
    if (~mask).any():
        out[~mask] = phys3((x[~mask], p[~mask], r[~mask]), *_rhs(theta))
    return out

def fd_deriv(x0, p0, r0, theta, side, h=1e-6):
    part = _lhs(theta) if side == "left" else _rhs(theta)
    f_plus = phys3((np.array([x0+h]), np.array([p0]), np.array([r0])), *part)[0]
    f_minus= phys3((np.array([x0-h]), np.array([p0]), np.array([r0])), *part)[0]
    return (f_plus - f_minus)/(2*h)

def residual(theta, x, p, r, y, plvls, rlvs,
             boundary, λc0, λc1, enforce_c1):
    res_data = predict_piecewise(x, p, r, theta, boundary) - y
    res_c0, res_c1 = [], []
    for pv in plvls:
        for rv in rlvs:
            l = phys3((np.array([boundary]), np.array([pv]), np.array([rv])), *_lhs(theta))[0]
            rr = phys3((np.array([boundary]), np.array([pv]), np.array([rv])), *_rhs(theta))[0]
            res_c0.append(l - rr)
            if enforce_c1:
                dl = fd_deriv(boundary, pv, rv, theta, "left")
                dr = fd_deriv(boundary, pv, rv, theta, "right")
                res_c1.append(dl - dr)
    res_c0 = np.array(res_c0)*np.sqrt(λc0)
    out = [res_data, res_c0]
    if enforce_c1:
        res_c1 = np.array(res_c1)*np.sqrt(λc1)
        out.append(res_c1)
    return np.concatenate(out)


# ────────────────────────────────────────────────────────────────
# 3. 针对单个目标列训练一次
# ────────────────────────────────────────────────────────────────
def fit_target(y, x, p, r,
               boundary=0.5,
               λc0=100., λc1=10., enforce_c1=True,
               sr_iter=800,
               target_name=''):
    p_levels = np.unique(p)
    r_levels = np.unique(r)

    print(f"[{target_name}] curve_fit 左段 ...")
    p0 = [0.1]*15 + [2.0]
    lb = [0.0]*15 + [1.0]
    ub = [np.inf]*15 + [8.0]
    θ_l, _ = curve_fit(phys3,
                       (x[x <= boundary], p[x <= boundary], r[x <= boundary]),
                       y[x <= boundary], p0=p0, bounds=(lb, ub), maxfev=30000)
    print(f"[{target_name}] curve_fit 右段 ...")
    θ_r, _ = curve_fit(phys3,
                       (x[x >= boundary], p[x >= boundary], r[x >= boundary]),
                       y[x >= boundary], p0=p0, bounds=(lb, ub), maxfev=30000)
    θ0 = np.concatenate([θ_l, θ_r])

    print(f"[{target_name}] least_squares 全局拟合 ...")
    opt = least_squares(residual, θ0, verbose=2, max_nfev=40000,
                        kwargs=dict(x=x, p=p, r=r, y=y,
                                    plvls=p_levels, rlvs=r_levels,
                                    boundary=boundary,
                                    λc0=λc0, λc1=λc1,
                                    enforce_c1=enforce_c1))
    θ_best = opt.x

    # 残差符号回归
    sr = None
    if USE_SYMBOLIC:
        print(f"[{target_name}] PySR 残差符号回归 ...")
        res = y - predict_piecewise(x, p, r, θ_best, boundary)
        sr = PySRRegressor(
            niterations=sr_iter,
            unary_operators=["exp", "log", "sqrt", "sin"],
            binary_operators=["+", "-", "*", "/", "pow"],
            model_selection="best")
        sr.fit(np.column_stack([x, p, r]), res)
        print(f"[{target_name}] PySR 完成")
    print(f"[{target_name}] 拟合结束！")
    return dict(theta=θ_best,
                boundary=boundary,
                sr_model=sr)


# ────────────────────────────────────────────────────────────────
# 4. 统一 2-输出模型封装
# ────────────────────────────────────────────────────────────────
class PiecewiseKTKQModel:
    def __init__(self, arte_kt, arte_kq):
        self.kt = arte_kt
        self.kq = arte_kq

    @staticmethod
    def _eval_one(x, p, r, art):
        y = predict_piecewise(x, p, r, art["theta"], art["boundary"])
        if art["sr_model"] is not None:
            y += art["sr_model"].predict(np.column_stack([x, p, r]))
        return y

    def predict(self, h_over_R, thrust_level, R, output="both"):
        h = np.asarray(h_over_R, dtype=float)
        t = np.asarray(thrust_level, dtype=float)
        r = np.asarray(R, dtype=float)
        shape = np.broadcast(h, t, r).shape
        xx, pp, rr = np.broadcast_arrays(h, t, r)

        kt = self._eval_one(xx.ravel(), pp.ravel(), rr.ravel(), self.kt).reshape(shape)
        kq = self._eval_one(xx.ravel(), pp.ravel(), rr.ravel(), self.kq).reshape(shape)

        if output == "both":
            return kt, kq
        elif output.upper() == "KT":
            return kt
        elif output.upper() == "KQ":
            return kq
        else:
            raise ValueError("output 仅支持 'both'|'KT'|'KQ'")

# ────────────────────────────────────────────────────────────────
# 5. 主流程
# ────────────────────────────────────────────────────────────────
def main():
    df = pd.read_csv("./data/merged_rows.csv")   # ← 文件名改这里
    x = df["h_over_R"].values
    p = df["thrust_level"].values
    r = df["R"].values

    arte_kt = fit_target(df["KT"].values, x, p, r, target_name="KT")
    arte_kq = fit_target(df["KQ"].values, x, p, r, target_name="KQ")

    model = PiecewiseKTKQModel(arte_kt, arte_kq)

    with open("piecewise_ktkq.pkl", "wb") as f:
        pickle.dump(model, f)
    print("✔ Saved piecewise_ktkq.pkl")

    # 简易基准
    kt1, kq1 = model.predict(0.12, 0.6, 0.25)
    print(f"Example predict: KT={float(kt1):.4f}, KQ={float(kq1):.4f}")

if __name__ == "__main__":
    main()

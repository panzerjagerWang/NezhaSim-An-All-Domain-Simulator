#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File: piecewise_phys_sr_ctr.py
"""
2-input  →  1-output piece-wise physics + symbolic-regression model
------------------------------------------------------------------
• inputs : z_over_l (x), velocity (v)
• output : ctr
• I/O    : 读取 ./merged_rows.csv
• 保存   : <当前文件夹名>.json / <当前文件夹名>.pkl  到当前工作目录
"""

import json
import pickle
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import curve_fit, least_squares

# 是否进行残差的符号回归（与原文件保持一致的开关）
USE_SYMBOLIC = True
if USE_SYMBOLIC:
    from pysr import PySRRegressor

# ────────────────────────────────────────────────────────────────
# 1) 物理启发函数（两输入版，与原文件风格一致）
#    结构： 1 + A1*exp(-B1*x) + A2*exp(-B2*x) + wall
#    其中 A1,A2,wall,B1,B2 的系数对速度 v 线性依赖
# ────────────────────────────────────────────────────────────────
def phys2(inputs,
          A10, A11,
          A20, A21,
          A30, A31,
          B10, B11,
          B20, B21,
          n):
    """
    params 共 11 个：
      - A1(v) = A10 + A11*v
      - A2(v) = A20 + A21*v
      - A3(v) = A30 + A31*v
      - B1(v) = B10 + B11*v
      - B2(v) = B20 + B21*v
      - wall = A3 / (1 + x)^n
    """
    x, v = inputs
    A1 = A10 + A11 * v
    A2 = A20 + A21 * v
    A3 = A30 + A31 * v
    B1 = B10 + B11 * v
    B2 = B20 + B21 * v
    wall = A3 / np.power(1.0 + x, n)
    return 1.0 + A1 * np.exp(-B1 * x) + A2 * np.exp(-B2 * x) + wall


# ────────────────────────────────────────────────────────────────
# 2) 工具：左右段、连续性/可导性残差（与原文件风格一致）
# ────────────────────────────────────────────────────────────────
def _lhs(theta):   return theta[:11]
def _rhs(theta):   return theta[11:]

def predict_piecewise(x, v, theta, boundary):
    mask = x <= boundary
    out = np.empty_like(x, dtype=float)
    if mask.any():
        out[mask] = phys2((x[mask], v[mask]), *_lhs(theta))
    if (~mask).any():
        out[~mask] = phys2((x[~mask], v[~mask]), *_rhs(theta))
    return out

def fd_deriv(x0, v0, theta, side, h=1e-6):
    part = _lhs(theta) if side == "left" else _rhs(theta)
    f_plus  = phys2((np.array([x0 + h]), np.array([v0])), *part)[0]
    f_minus = phys2((np.array([x0 - h]), np.array([v0])), *part)[0]
    return (f_plus - f_minus) / (2.0 * h)

def residual(theta, x, v, y, v_levels,
             boundary, λc0, λc1, enforce_c1):
    # 数据残差
    res_data = predict_piecewise(x, v, theta, boundary) - y

    # C0/C1 连续性约束（在所有速度水平上约束边界点）
    res_c0, res_c1 = [], []
    for vv in v_levels:
        l  = phys2((np.array([boundary]), np.array([vv])), *_lhs(theta))[0]
        rr = phys2((np.array([boundary]), np.array([vv])), *_rhs(theta))[0]
        res_c0.append(l - rr)
        if enforce_c1:
            dl = fd_deriv(boundary, vv, theta, "left")
            dr = fd_deriv(boundary, vv, theta, "right")
            res_c1.append(dl - dr)

    res_c0 = np.array(res_c0) * np.sqrt(λc0)
    out = [res_data, res_c0]
    if enforce_c1:
        res_c1 = np.array(res_c1) * np.sqrt(λc1)
        out.append(res_c1)
    return np.concatenate(out)


# ────────────────────────────────────────────────────────────────
# 3) 针对 ctr 训练一次（与原文件 fit_target 风格一致）
# ────────────────────────────────────────────────────────────────
def fit_ctr(y, x, v,
            boundary=1.0,     # 对应 z_over_l 的物理分段边界，必要时可改
            λc0=100., λc1=10., enforce_c1=True,
            sr_iter=1000):
    v_levels = np.unique(v)

    # —— 重点加权：放大 v=0.2（和可选 v=0.4）以及 x≈0.2~0.3 的峰值区域 —— #
    w = np.ones_like(y, dtype=float)
    w[v == 0.2] *= 6.0          # 强力关照 0.2
    w[v == 0.4] *= 2.0          # 适度关照 0.4（可选）
    # 针对局部峰值（例如 x0≈0.23, 宽度σ≈0.06）再加一层高斯核
    x0, sigma = 0.23, 0.06
    w *= (1.0 + 3.0 * np.exp(-((x - x0) / sigma) ** 2))
    print("[ctr] curve_fit 左段 ...")
    p0 = [0.1]*10 + [2.0]      # 11 参数：10 个系数 + n
    lb = [0.0]*10 + [1.0]
    ub = [np.inf]*10 + [8.0]
    θ_l, _ = curve_fit(phys2,
                       (x[x <= boundary], v[x <= boundary]),
                       y[x <= boundary],
                       p0=p0, bounds=(lb, ub), maxfev=1000)

    print("[ctr] curve_fit 右段 ...")
    θ_r, _ = curve_fit(phys2,
                       (x[x >= boundary], v[x >= boundary]),
                       y[x >= boundary],
                       p0=p0, bounds=(lb, ub), maxfev=1000)

    θ0 = np.concatenate([θ_l, θ_r])

    print("[ctr] least_squares 全局拟合 ...")
    opt = least_squares(residual, θ0, verbose=2, max_nfev=1000,loss="soft_l1", f_scale=0.1,
                        kwargs=dict(x=x, v=v, y=y, v_levels=v_levels,
                                    boundary=boundary,
                                    λc0=λc0, λc1=λc1, enforce_c1=enforce_c1))
    θ_best = opt.x

    # 残差符号回归（与原代码一致：对残差做 SR）
    sr = None
    if USE_SYMBOLIC:
        print("[ctr] PySR 残差符号回归 ...")
        res = y - predict_piecewise(x, v, θ_best, boundary)
        sr = PySRRegressor(
            niterations=sr_iter,
            unary_operators=["exp", "log", "sqrt", "sin", "cos", "tanh"],
            binary_operators=["+", "-", "*", "/", "^"],
            constraints={"^": (-1, 1)},
            model_selection="accuracy",
            population_size=40,
        )
        sr.fit(np.column_stack([x, v]), res)
        print("[ctr] PySR 完成")

    print("[ctr] 拟合结束！")
    return dict(theta=θ_best,
                boundary=boundary,
                sr_model=sr)


# ────────────────────────────────────────────────────────────────
# 4) 单输出模型封装（与原文件 Piecewise*Model 风格一致）
# ────────────────────────────────────────────────────────────────
class PiecewiseCTRModel:
    def __init__(self, arte_ctr):
        self.ctr = arte_ctr

    @staticmethod
    def _eval_one(x, v, art):
        y = predict_piecewise(x, v, art["theta"], art["boundary"])
        if art["sr_model"] is not None:
            y += art["sr_model"].predict(np.column_stack([x, v]))
        return y

    def predict(self, z_over_l, velocity):
        x = np.asarray(z_over_l, dtype=float)
        v = np.asarray(velocity, dtype=float)
        shape = np.broadcast(x, v).shape
        xx, vv = np.broadcast_arrays(x, v)
        y = self._eval_one(xx.ravel(), vv.ravel(), self.ctr).reshape(shape)
        return y


# ────────────────────────────────────────────────────────────────
# 5) 主流程：读取 CSV，训练并保存 JSON/PKL（文件名=当前文件夹名）
# ────────────────────────────────────────────────────────────────
def main():
    # —— 读取数据 ——（列名与既有数据约定：z_over_l, velocity, ctr）
    df = pd.read_csv("./data/merged_rows.csv")
    # 兼容大小写
    cols = {c.lower(): c for c in df.columns}
    x = df[cols.get("z_over_l", "z_over_l")].values.astype(float)
    v = df[cols.get("velocity", "velocity")].values.astype(float)
    y = df[cols.get("ctr", "ctr")].values.astype(float)

    # —— 训练 ——（与原始文件同样的打印与流程）
    arte_ctr = fit_ctr(y, x, v, boundary=np.median(x), λc0=500., λc1=100., enforce_c1=True, sr_iter=1000)
    model = PiecewiseCTRModel(arte_ctr)

    # —— 生成输出文件名（以当前工作目录名命名）——
    folder_name = Path.cwd().name
    json_path = Path(f"{folder_name}.json")
    pkl_path = Path(f"{folder_name}.pkl")

    # —— 保存 PKL：包含可直接 predict 的模型对象 ——
    with open(pkl_path, "wb") as f:
        pickle.dump(model, f)
    print(f"✔ Saved {pkl_path.name}")

    # —— 保存 JSON：保存可读方程与元信息（不序列化模型本体）——
    sr_eq = None
    if arte_ctr["sr_model"] is not None:
        try:
            sr_eq = str(arte_ctr["sr_model"])
        except Exception:
            sr_eq = "<pysr model>"

    json_payload = {
        "engine": "piecewise+residual-PySR",
        "equation_residual": sr_eq,     # 残差 SR 的可读表达式
        "piecewise": {
            "boundary_on_z_over_l": float(arte_ctr["boundary"]),
            "theta_len": len(arte_ctr["theta"]),
        },
        "inputs": ["z_over_l", "velocity"],
        "output": "ctr",
        "csv": "merged_rows.csv",
        "notes": "PKL 保存可直接用于 predict 的 PiecewiseCTRModel；JSON 仅保存方程与元信息。",
    }
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(json_payload, f, ensure_ascii=False, indent=2)
    print(f"✔ Saved {json_path.name}")


if __name__ == "__main__":
    main()

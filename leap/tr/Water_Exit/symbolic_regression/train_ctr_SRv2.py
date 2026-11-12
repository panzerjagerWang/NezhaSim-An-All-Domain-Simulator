#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File: piecewise_phys_sr_ctr.py
"""
2-input → 1-output (global) physics + symbolic-regression model  【无分段版】
------------------------------------------------------------------
• inputs : z_over_l (x), velocity (v)
• output : ctr
• I/O    : 读取 ./data/merged_rows.csv
• 保存   : <当前文件夹名>.json / <当前文件夹名>.pkl  到当前工作目录
"""

import json
import pickle
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import curve_fit

# 是否进行残差的符号回归（保持原开关）
USE_SYMBOLIC = True
if USE_SYMBOLIC:
    from pysr import PySRRegressor

# ────────────────────────────────────────────────────────────────
# 1) 物理启发函数（两输入，单段全局拟合）
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
    共 11 个参数：
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
# 2) 训练（全局单段拟合 + 残差 SR）
# ────────────────────────────────────────────────────────────────
def fit_ctr_global(y, x, v, sr_iter=800):
    print("[ctr] curve_fit 全局单段 ...")
    p0 = [0.1]*10 + [2.0]      # 11 参数：10 个系数 + n
    lb = [0.0]*10 + [1.0]
    ub = [np.inf]*10 + [8.0]
    theta, _ = curve_fit(
        phys2, (x, v), y,
        p0=p0, bounds=(lb, ub), maxfev=40000
    )

    # 残差符号回归
    sr = None
    if USE_SYMBOLIC:
        print("[ctr] PySR 残差符号回归 ...")
        res = y - phys2((x, v), *theta)
        sr = PySRRegressor(
            niterations=sr_iter,
            unary_operators=["exp", "log", "sqrt", "sin"],
            binary_operators=["+", "-", "*", "/", "pow"],
            model_selection="best",
        )
        sr.fit(np.column_stack([x, v]), res)
        print("[ctr] PySR 完成")

    print("[ctr] 拟合结束！")
    # 兼容旧结构：保留 boundary 键，但置为 None
    return dict(theta=theta, boundary=None, sr_model=sr)


# ────────────────────────────────────────────────────────────────
# 3) 单输出模型封装（类名与接口保持不变）
# ────────────────────────────────────────────────────────────────
class PiecewiseCTRModel:
    def __init__(self, arte_ctr):
        self.ctr = arte_ctr

    @staticmethod
    def _eval_one(x, v, art):
        # 无分段：直接用单段物理模型 + 残差 SR
        y = phys2((x, v), *art["theta"])
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
# 4) 主流程：读取 CSV，训练并保存 JSON/PKL（文件名=当前文件夹名）
# ────────────────────────────────────────────────────────────────
def main():
    # —— 读取数据 ——（列名与既有数据约定：z_over_l, velocity, ctr）
    df = pd.read_csv("./data/merged_rows.csv")
    cols = {c.lower(): c for c in df.columns}
    x = df[cols.get("z_over_l", "z_over_l")].values.astype(float)
    v = df[cols.get("velocity", "velocity")].values.astype(float)
    y = df[cols.get("ctr", "ctr")].values.astype(float)

    # —— 训练（单段）——
    arte_ctr = fit_ctr_global(y, x, v, sr_iter=10)
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
        "engine": "global+residual-PySR",   # 标明为“无分段”
        "equation_residual": sr_eq,
        "piecewise": {                      # 为兼容保留该段
            "boundary_on_z_over_l": None,
            "theta_len": int(len(arte_ctr["theta"])),
        },
        "inputs": ["z_over_l", "velocity"],
        "output": "ctr",
        "csv": "merged_rows.csv",
        "notes": "PKL 保存可直接用于 predict 的模型；JSON 仅保存方程与元信息（无分段）。",
    }
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(json_payload, f, ensure_ascii=False, indent=2)
    print(f"✔ Saved {json_path.name}")

    # —— 简易基准 ——（保持原示例）
    y1 = model.predict(0.8, 1.0)
    print(f"Example predict: ctr={float(np.asarray(y1).ravel()[0]):.5f}")


if __name__ == "__main__":
    main()

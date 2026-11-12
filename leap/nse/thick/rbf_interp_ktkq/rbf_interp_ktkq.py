#!/usr/bin/env python3
# rbf_interp_ktkq.py  ——  Smooth C¹ 3‑D RBF 插值器 (修正版)
"""
⚠ 修复：SciPy 的 `RBFInterpolator` 使用关键字 `smoothing` 而不是 `smooth`。
   若 SciPy < 1.7 则没有该类，请先升级：
       pip install --upgrade "scipy>=1.7"

本脚本读入 all_process_smooth.csv，训练两个 RBFInterpolator
(KT / KQ)，并保存为同目录 pkl 文件，接口与旧 LinearND 版本一致。
"""

import time, pickle, json
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.interpolate import RBFInterpolator  # SciPy ≥ 1.7 required

# ── 1. 读数据 ───────────────────────────────────
df = pd.read_csv("./data/merged_rows.csv")
X  = df[["h_over_R", "thrust_level", "R"]].to_numpy(float)
KT = df["KT"].to_numpy(float)
KQ = df["KQ"].to_numpy(float)

# ── 2. 训练 RBFInterpolators ────────────────────
# 可调参数 ↓↓↓
KERNEL    = "thin_plate_spline"   # thin_plate_spline | cubic | multiquadric | linear
EPSILON   = 1.0                   # 基函数半径，越大越平滑
SMOOTHING = 1e-4                  # ≥0，正则化因子，越大越平滑
# ↑↑↑ 根据跳变程度调 EPSILON (0.5–2.0) 和 SMOOTHING (1e‑6–1e‑2)

start = time.perf_counter()
rbf_kt = RBFInterpolator(X, KT, kernel=KERNEL, epsilon=EPSILON, smoothing=SMOOTHING)
rbf_kq = RBFInterpolator(X, KQ, kernel=KERNEL, epsilon=EPSILON, smoothing=SMOOTHING)
train_ms = (time.perf_counter() - start) * 1e3
print(f"[TIME] RBF 训练耗时: {train_ms:.1f} ms")

# ── 3. 封装兼容旧接口 ────────────────────────────
class KTQModel:
    def __init__(self, rbf_kt, rbf_kq):
        self.rbf_kt, self.rbf_kq = rbf_kt, rbf_kq

    def predict(self, h_over_R, thrust_level, R, output="both"):
        h, t, r = np.asarray(h_over_R), np.asarray(thrust_level), np.asarray(R)
        shape   = np.broadcast(h, t, r).shape
        pts     = np.column_stack([h.ravel(), t.ravel(), r.ravel()])
        kt = self.rbf_kt(pts).reshape(shape)
        kq = self.rbf_kq(pts).reshape(shape)
        if output == "both":
            return kt, kq
        output = output.upper()
        if output == "KT":
            return kt
        if output == "KQ":
            return kq
        raise ValueError("output 仅支持 'both'|'KT'|'KQ'")

model = KTQModel(rbf_kt, rbf_kq)

# ── 4. 保存模型与简单 benchmark ───────────────────
pkl_path = Path(__file__).with_suffix(".pkl")
with open(pkl_path, "wb") as f:
    pickle.dump(model, f)
print("✔ RBF 模型已保存 →", pkl_path.name)

start = time.perf_counter()
kt1, kq1 = model.predict(0.25, 0.6, 0.25)
pred_ms  = (time.perf_counter() - start) * 1e3
print(f"[TIME] 单点预测: {pred_ms:.2f} ms → KT={float(kt1):.4f}, KQ={float(kq1):.4f}")

json.dump({
    "kernel": KERNEL,
    "epsilon": EPSILON,
    "smoothing": SMOOTHING,
    "train_ms": train_ms,
    "single_ms": pred_ms,
    "KT_sample": float(kt1),
    "KQ_sample": float(kq1)
}, open("rbf_interp_param_report.json", "w"), indent=2, ensure_ascii=False)
print("✔ 参数报告 → rbf_interp_param_report.json")

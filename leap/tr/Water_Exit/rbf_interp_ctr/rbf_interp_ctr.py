#!/usr/bin/env python3
# rbf_interp_ctr.py —— Smooth C¹ 2‑D RBF 插值器（按 rbf_interp_ktkq.py 改：读取 ./merged_rows.csv）
"""
依赖：
    pip install "scipy>=1.7" numpy pandas
数据要求（当前目录 merged_rows.csv）包含列：
    ["z_over_l", "velocity", "ctr"]
输出：
    - rbf_interp_ctr.pkl                （与脚本同名）
    - rbf_interp_ctr_report.json        （训练/预测耗时与采样结果）
"""

import time, pickle, json
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.interpolate import RBFInterpolator  # SciPy ≥ 1.7 required

# ── 1) 读数据（当前目录） ──────────────────────────────────
csv_path = Path("./data/merged_rows.csv")
df = pd.read_csv(csv_path)

# 两输入一输出
X   = df[["z_over_l", "velocity"]].to_numpy(float)
CTR = df["CTR"].to_numpy(float)

# ── 2) 训练 RBFInterpolator（与原脚本一致的可调参数） ─────
KERNEL    = "thin_plate_spline"   # 可选：thin_plate_spline | cubic | multiquadric | linear
EPSILON   = 1.0                   # 基函数尺度，越大越平滑（典型 0.5–2.0）
SMOOTHING = 1e-4                  # ≥0，正则化越大越平滑（典型 1e‑6–1e‑2）

start = time.perf_counter()
rbf_ctr = RBFInterpolator(X, CTR, kernel=KERNEL, epsilon=EPSILON, smoothing=SMOOTHING)
train_ms = (time.perf_counter() - start) * 1e3
print(f"[TIME] RBF 训练耗时: {train_ms:.1f} ms")

# ── 3) 封装与原接口风格一致的模型对象 ─────────────────────
class CTRModel:
    def __init__(self, rbf_ctr):
        self.rbf_ctr = rbf_ctr

    def predict(self, z_over_l, velocity):
        x = np.asarray(z_over_l)
        v = np.asarray(velocity)
        shape = np.broadcast(x, v).shape
        pts   = np.column_stack([x.ravel(), v.ravel()])
        ctr = self.rbf_ctr(pts).reshape(shape)
        return ctr

model = CTRModel(rbf_ctr)

# ── 4) 保存模型与简单 benchmark（与原版一致的输出风格） ─────
pkl_path = Path(__file__).with_suffix(".pkl")
with open(pkl_path, "wb") as f:
    pickle.dump(model, f)
print("✔ RBF 模型已保存 →", pkl_path.name)

# 单点预测耗时与示例（可按需修改采样点）
start = time.perf_counter()
ctr1 = model.predict(0.25, 0.6)
pred_ms  = (time.perf_counter() - start) * 1e3
print(f"[TIME] 单点预测: {pred_ms:.2f} ms → CTR={float(ctr1):.6f}")

# 保存参数与采样结果报告
json.dump({
    "csv": str(csv_path),
    "kernel": KERNEL,
    "epsilon": EPSILON,
    "smoothing": SMOOTHING,
    "train_ms": train_ms,
    "single_ms": pred_ms,
    "CTR_sample": float(ctr1)
}, open("rbf_interp_ctr_report.json", "w"), indent=2, ensure_ascii=False)
print("✔ 参数报告 → rbf_interp_ctr_report.json")

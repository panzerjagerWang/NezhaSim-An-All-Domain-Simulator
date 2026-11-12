#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
train_ctr_two_var_piecewise_v3.py
• 2 输入 (h, v) → CTR
• 分段 + 对数拉伸 + 增强物理骨架 + 残差符号回归(K-fold +AIC)
"""

from __future__ import annotations
from pathlib import Path
import numpy as np
import pandas as pd
import pickle
from scipy.optimize import curve_fit
from sklearn.model_selection import KFold
from pysr import PySRRegressor

# ───────────────────────────────────
DATA_CANDIDATES = [
    "./data/merged_rows.csv",
    "./data/ctr_merged_dataset.csv",
]
COL_H, COL_V, COL_Y = "z_over_l", "velocity", "ctr"

# 分段区间
SEGMENTS = [(0.0, 0.3), (0.3, 1.0)]
COMPARE_V = [0.20, 0.40]

# ───────────────────────────────────
# 物理骨架模型（13 参数）
def model_flex(inputs,
               A10, A11, A20, A21, A30, A31,    # 6
               B10, B11, B20, B21,              # 4
               n,                               # 1
               C0, C1):                         # 2
    h_log, v = inputs                 # h_log = log(1+h)
    A1 = A10 + A11*v
    A2 = A20 + A21*v
    A3 = A30 + A31*v
    B1 = B10 + B11*v
    B2 = B20 + B21*v

    exp1  = A1 * np.exp(-B1 * h_log)
    exp2  = A2 * np.exp(-B2 * h_log)
    wall  = A3 / np.power(1.0 + h_log, n)
    poly  = (C0 + C1 * v) * h_log**2          # 新增二次多项式

    return 1.0 + exp1 + exp2 + wall + poly

# ───────────────────────────────────
def fit_initial(h_log, v, y):
    #            6×A/B  1×n  2×C
    p0    = [0.3,0.3, 0.3,0.3, 0.2,0.2,
             2.0,0.5, 5.0,1.0,
             2.0,
             0.0,0.0]
    lower = [0]*13
    upper = [np.inf]*13
    popt, _ = curve_fit(model_flex, (h_log, v), y,
                        p0=p0, bounds=(lower, upper), maxfev=50000)
    return popt

# ───────────────────────────────────
def residual_sr_cv(h_log, v, y, y_pred, kfold: int = 5):
    X = np.column_stack([h_log, v])
    resid = y - y_pred

    sr = PySRRegressor(
        niterations      = 250,
        unary_operators  = ["exp","log","sqrt","sin","cos"],
        binary_operators = ["+","-","*","/"],     # 去掉 pow 避免奇异
        variable_names   = ["hL","v"],
        model_selection  = "AIC",
        loss             = "L2DistLoss()",
        random_state     = 0,
    )
    sr.fit(X, resid)

    # ── K-fold 重新甄选最优表达式 ───────────────────
    kf = KFold(n_splits=kfold, shuffle=True, random_state=0)
    best_expr, best_mse, best_aic = None, np.inf, np.inf

    for eq in sr.equations_["equation"]:
        preds = np.zeros_like(resid)
        for train_idx, test_idx in kf.split(X):
            preds_fold = sr.predict(X[test_idx], select=str(eq))  # 旧版用 select
            preds[test_idx] = preds_fold
        mse = np.mean((resid - preds)**2)

        # AIC ＝ 2k + n·ln(MSE)，k≈复杂度
        k_param = len(str(eq))               # 近似使用字符串长度衡量复杂度
        aic = 2*k_param + len(y)*np.log(mse)

        if mse < best_mse - 1e-10 or (abs(mse-best_mse)<1e-10 and aic < best_aic):
            best_expr, best_mse, best_aic = str(eq), mse, aic

    sr.set_best(best_expr)                   # 固定表达式，以免 predict 摇摆
    return sr, best_expr, best_mse

# ───────────────────────────────────
def main():
    # 0) 读数据
    for p in DATA_CANDIDATES:
        if Path(p).exists():
            csv = Path(p); break
    else:
        raise FileNotFoundError("未找到数据文件")

    df = pd.read_csv(csv).rename(columns=str.lower)
    h_raw = df[COL_H].to_numpy(float)
    v = np.abs(df[COL_V]).to_numpy(float)
    y = df[COL_Y].to_numpy(float)
    h_log = np.log1p(h_raw)                 # log 拉伸
    print(f"[INFO] 数据读取完毕：N={len(h_raw)}")

    # 1) 分段建模
    results = {}   # {(start,end): {params, sr_model}}
    for (start, end) in SEGMENTS:
        m = (h_raw >= start) & (h_raw <= end)
        if not m.any():
            print(f"[WARN] 区间 {start}-{end} 无数据，跳过")
            continue

        h_seg_raw, h_seg_log = h_raw[m], h_log[m]
        v_seg, y_seg = v[m], y[m]
        print(f"\n[物理骨架] 区间 {start}-{end}，样本 {len(h_seg_log)}")

        # 1.1) 物理骨架拟合
        popt = fit_initial(h_seg_log, v_seg, y_seg)
        y_base = model_flex((h_seg_log, v_seg), *popt)

        # 1.2) 残差符号回归 + K-fold 选式
        print("[符号回归]  拟合残差 (K-fold)...")
        sr, expr, mse = residual_sr_cv(h_seg_log, v_seg, y_seg, y_base)
        print(f"   ↳ 选中式: {expr}   (CV-MSE={mse:.4e})")

        results[(start, end)] = dict(params=popt, sr_model=sr, expr=expr)

    # 2) 保存整体模型
    with open("piecewise_model.pkl", "wb") as f:
        pickle.dump(results, f)
    print("✔ 模型已保存为 piecewise_model.pkl")

    # 3) 生成 v=0.20 / 0.40 预测曲线
    h_min, h_max = h_raw.min(), h_raw.max()
    hh_raw = np.linspace(h_min, h_max, 600)
    hh_log = np.log1p(hh_raw)

    for v_level in COMPARE_V:
        yy = []
        for hi_raw, hi_log in zip(hh_raw, hh_log):
            for (start,end), res in results.items():
                if start <= hi_raw <= end:
                    y_base = model_flex((np.array([hi_log]), np.array([v_level])),
                                        *res["params"])[0]
                    y_corr = res["sr_model"].predict(np.array([[hi_log, v_level]]))[0]
                    yy.append(y_base + y_corr)
                    break
            else:
                yy.append(np.nan)

        out = pd.DataFrame({"h": hh_raw, "ctr_pred": yy})
        out.to_csv(f"ctr_predicted_v{int(v_level*100):02d}.csv", index=False)
        print(f"✔ 预测曲线 v={v_level:.2f} 已保存")

    # 4) 打印各段表达式
    print("\n" + "="*42)
    for (s,e), res in results.items():
        print(f"[{s}, {e}]  {res['expr']}")

# ───────────────────────────────────
if __name__ == "__main__":
    main()

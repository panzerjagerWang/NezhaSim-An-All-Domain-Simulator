#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
direct_symbolic_regression.py
─────────────────────────────
• 2 输入 (h = z_over_l, v = |velocity|) → 1 输出 (CTR)
• 直接符号回归，无物理骨架
• 分段处理 + 生成 v = 0.20 与 v = 0.40 两条预测曲线

输出：
  - symbolic_model.pkl                  # {'(start,end)': {'sr_model', 'symbolic_expr'}}
  - ctr_predicted_v20.csv
  - ctr_predicted_v40.csv
"""

from __future__ import annotations
from pathlib import Path
import numpy as np
import pandas as pd
import pickle
from pysr import PySRRegressor

# ───────────────────────────────────
# 0) 数据路径 & 列名（如有不同请修改）
DATA_CANDIDATES = [
    "./data/merged_rows.csv",
    "./data/ctr_merged_dataset.csv",
    "/mnt/data/merged_rows.csv",
    "/mnt/data/ctr_merged_dataset.csv",
]
COL_H = "z_over_l"  # h
COL_V = "velocity"  # v
COL_Y = "ctr"  # 目标 CTR


# ───────────────────────────────────
def main():
    # 1) 读取数据
    for p in DATA_CANDIDATES:
        if Path(p).exists():
            csv_path = Path(p)
            break
    else:
        raise FileNotFoundError("未找到 CTR 数据文件，请检查 DATA_CANDIDATES")

    df = pd.read_csv(csv_path).rename(columns=str.lower)
    h = df[COL_H.lower()].to_numpy(float)
    v = np.abs(df[COL_V.lower()].to_numpy(float))
    y = df[COL_Y.lower()].to_numpy(float)
    print(f"[INFO] 数据 {csv_path} 读取完毕：N={len(h)}")

    # 2) 分段区间
    segments = [(0.0, 0.3),
                (0.3, 1.0)]

    results: dict[tuple[float, float], dict] = {}

    for (start, end) in segments:
        m = (h >= start) & (h <= end)
        h_seg, v_seg, y_seg = h[m], v[m], y[m]
        if h_seg.size == 0:
            print(f"[WARN] 区间 {start}-{end} 无数据，跳过")
            continue

        print(f"\n[符号回归] 区间 {start}-{end}  (N={h_seg.size})")

        # 直接对原始数据进行符号回归
        X = np.column_stack([h_seg, v_seg])

        sr_model = PySRRegressor(
            niterations=2000,  # 增加迭代次数以获得更好的拟合
            populations=20,  # 增加种群数量
            unary_operators=["exp", "log", "sqrt", "sin", "cos", "abs"],
            binary_operators=["+", "-", "*", "/", "pow"],
            model_selection="best",
            variable_names=["h", "v"],
            loss="loss(prediction, target) = (prediction - target)^2",
            maxsize=30,  # 允许更复杂的表达式
            timeout_in_seconds=300,  # 5分钟超时
        )

        sr_model.fit(X, y_seg)
        best_expr = sr_model.get_best()
        print("   ↳ 符号回归最佳表达式:", best_expr)

        # 计算拟合质量
        y_pred = sr_model.predict(X)
        r2 = 1 - np.sum((y_seg - y_pred) ** 2) / np.sum((y_seg - np.mean(y_seg)) ** 2)
        rmse = np.sqrt(np.mean((y_seg - y_pred) ** 2))
        print(f"   ↳ R² = {r2:.4f}, RMSE = {rmse:.6f}")

        results[(start, end)] = {
            "sr_model": sr_model,
            "symbolic_expr": best_expr,
            "r2": r2,
            "rmse": rmse
        }

    # 3) 保存整体模型
    with open("symbolic_model.pkl", "wb") as f:
        pickle.dump(results, f)
    print("✔ 符号回归模型已保存为 symbolic_model.pkl")

    # 4) 生成 v=0.20 & 0.40 预测曲线
    COMPARE_V = [0.20, 0.40]
    h_min, h_max = h.min(), h.max()

    for v_level in COMPARE_V:
        hh = np.linspace(h_min, h_max, 600)
        yy = []
        for hi in hh:
            for (start, end), res in results.items():
                if start <= hi <= end:
                    y_pred = res["sr_model"].predict(np.array([[hi, v_level]]))[0]
                    yy.append(y_pred)
                    break
            else:
                yy.append(np.nan)  # 区间外填 nan

        out = pd.DataFrame({"h": hh, "ctr_pred": yy})
        csv_name = f"ctr_predicted_v{int(v_level * 100):02d}.csv"
        out.to_csv(csv_name, index=False)
        print(f"✔ 预测曲线 v={v_level:.2f} 保存为 {csv_name}")

    # 5) 打印最终的符号表达式
    print("\n" + "=" * 50)
    print("最终符号表达式:")
    print("=" * 50)
    for (start, end), res in results.items():
        print(f"区间 [{start}, {end}]: {res['symbolic_expr']}")
        print(f"  R² = {res['r2']:.4f}, RMSE = {res['rmse']:.6f}")


# ───────────────────────────────────
if __name__ == "__main__":
    main()


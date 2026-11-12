#!/usr/bin/env python3
# two_stage_piecewise_regression.py

import numpy as np
import pandas as pd
import pickle
from scipy.optimize import curve_fit
from pysr import PySRRegressor

# 三层物理骨架模型定义
def model_3layer(inputs, A10,A11,A20,A21,A30,A31,B10,B11,B20,B21,n):
    x, p = inputs
    A1 = A10 + A11*p
    A2 = A20 + A21*p
    A3 = A30 + A31*p
    B1 = B10 + B11*p
    B2 = B20 + B21*p
    wall = A3 / np.power(1.0 + x, n)
    return 1.0 + A1*np.exp(-B1*x) + A2*np.exp(-B2*x) + wall

# 第一阶段拟合模型
def fit_initial(x, p, y):
    p0 = [0.3,0.3,0.3,0.3,0.2,0.2,2.0,0.5,5.0,1.0,2.0]
    lower = [0]*11
    upper = [np.inf]*11
    popt, _ = curve_fit(model_3layer, (x, p), y, p0=p0, bounds=(lower, upper), maxfev=30000)
    return popt

# 残差符号回归（第二阶段）
def residual_sr(x, p, y, y_pred):
    X = np.column_stack([x, p])
    residual = y - y_pred
    sr = PySRRegressor(
        niterations=1000,
        unary_operators=["exp", "log", "sin", "cos", "sqrt"],
        binary_operators=["+", "-", "*", "/", "pow"],
        model_selection="best"
    )
    sr.fit(X, residual)
    return sr

# 分段拟合主流程
def main():
    df = pd.read_csv("./data/smoothed_data.csv")
    x = df['x'].values
    ct_levels = {'ct_40':0.4, 'ct_70':0.7, 'ct_100':1.0}
    segments = [(0.1,1.0), (1.0,4.0)]

    results = {}

    for (start, end) in segments:
        mask = (x >= start) & (x <= end)
        x_seg = np.tile(x[mask], 3)
        p_seg = np.repeat(list(ct_levels.values()), np.sum(mask))
        y_seg = np.concatenate([df[col].values[mask] for col in ct_levels])

        print(f"\n拟合区间: {start} - {end}")
        popt = fit_initial(x_seg, p_seg, y_seg)
        y_pred = model_3layer((x_seg, p_seg), *popt)

        sr_model = residual_sr(x_seg, p_seg, y_seg, y_pred)
        best_expr = sr_model.get_best()
        print("分段符号回归模型最佳表达式:", best_expr)

        results[(start, end)] = {'params': popt, 'symbolic_expr': best_expr, 'sr_model': sr_model}

    # 保存模型到本地
    with open("piecewise_model.pkl", "wb") as f:
        pickle.dump(results, f)
    print("分段模型已保存为 piecewise_model.pkl")

    # 生成60%和80%预测曲线并保存
    for p_level in [0.6, 0.8]:
        xx = np.linspace(0.1, 4.0, 600)
        yy = []
        for xi in xx:
            for (start, end), res in results.items():
                if start <= xi <= end:
                    y_base = model_3layer((np.array([xi]), np.array([p_level])), *res['params'])[0]
                    y_corr = y_base + res['sr_model'].predict(np.array([[xi, p_level]]))[0]
                    yy.append(y_corr)
                    break
        pred_df = pd.DataFrame({'x': xx, 'ct_predicted': yy})
        pred_df.to_csv(f"predicted_curve_{int(p_level*100)}.csv", index=False)
        print(f"预测曲线 {int(p_level*100)}% 已保存为 predicted_curve_{int(p_level*100)}.csv")

if __name__ == "__main__":
    main()

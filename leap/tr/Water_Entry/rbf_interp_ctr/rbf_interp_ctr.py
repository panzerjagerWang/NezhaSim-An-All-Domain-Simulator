#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rbf_interp_ctr.py
-----------------
两变量 RBF 插值基线：X = [h = z_over_l, v = |velocity|] → y = CTR

子命令：
  • train   ：从 CSV 训练并保存模型 rbf_ctr.pkl
  • predict ：单点或 CSV 批量预测（追加 CTR_pred_rbf）
  • xval    ：K 折交叉验证 RMSE/MAE

示例：
  # 训练（薄板样条核，与 make_ctr_report.py 里的 RBF 基线相同）
  python rbf_interp_ctr.py train --csv ./data/ctr_merged_dataset.csv \
         --kernel thin_plate_spline --epsilon 1.0 --smoothing 1e-4 --out ./rbf_ctr.pkl

  # 交叉验证
  python rbf_interp_ctr.py xval --csv ./data/ctr_merged_dataset.csv --kfold 5 \
         --kernel thin_plate_spline --epsilon 1.0 --smoothing 1e-4

  # 批量预测
  python rbf_interp_ctr.py predict --model ./rbf_ctr.pkl \
         --csv ./data/samples.csv --out ./data/samples_pred.csv

  # 单点预测
  python rbf_interp_ctr.py predict --model ./rbf_ctr.pkl --h 0.5 --v 0.3
"""

from __future__ import annotations
import argparse
import pickle
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple, Iterable

import numpy as np
import pandas as pd

# 优先使用新版 API（SciPy ≥ 1.10），否则回退到旧接口
try:
    from scipy.interpolate import RBFInterpolator as _RBFI
    HAVE_RBFI = True
except Exception:
    HAVE_RBFI = False
    from scipy.interpolate import Rbf as _Rbf  # type: ignore

# -----------------------------
# 实用函数：列名规范化 & CSV 读取
# -----------------------------
def _normalize_colnames(cols: Iterable[str]) -> list[str]:
    out = []
    for c in cols:
        s = str(c).strip().lower()
        for t in ["(m/s)", "(m^2)", "(m)", "(n)", "[m/s]", "[m]"]:
            s = s.replace(t, "")
        s = s.replace(" ", "")
        out.append(s)
    return out

def load_ctr_csv(path: Path) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    期望列名（大小写/单位友好）：
      - z_over_l 或 z/l
      - velocity（内部取绝对值）
      - CTR
    """
    df = pd.read_csv(path)
    names = _normalize_colnames(df.columns)
    name_map = {n: c for n, c in zip(names, df.columns)}

    h_col = next((name_map[k] for k in name_map if "z_over_l" in k or "z/l" in k or k == "z"), None)
    if h_col is None:
        raise KeyError("未找到 z_over_l / z/l 列")

    v_col = next((name_map[k] for k in name_map if "velocity" in k or k == "v"), None)
    if v_col is None:
        raise KeyError("未找到 velocity 列")

    y_col = name_map.get("ctr", None)
    if y_col is None:
        raise KeyError("未找到 CTR 列")

    h = df[h_col].to_numpy(float)
    v = np.abs(df[v_col].to_numpy(float))  # 速度不分正负
    y = df[y_col].to_numpy(float)
    return h, v, y

# -----------------------------
# 简单标准化（避免依赖 sklearn）
# -----------------------------
@dataclass
class StdScaler:
    mean_: np.ndarray
    std_: np.ndarray

    @classmethod
    def fit(cls, X: np.ndarray):
        m = np.mean(X, axis=0)
        s = np.std(X, axis=0)
        s[s == 0.0] = 1.0
        return cls(m, s)

    def transform(self, X: np.ndarray) -> np.ndarray:
        return (X - self.mean_) / self.std_

# -----------------------------
# RBF 模型封装（统一新旧 API）
# -----------------------------
class RBFModel:
    """
    统一封装 RBFInterpolator / Rbf，两者接口对齐：
      • kernel: multiquadric, inverse_multiquadric, linear, cubic,
                thin_plate_spline, gaussian, quintic
      • epsilon: 形状参数（两 API 同名）
      • smoothing: 平滑项（仅 RBFInterpolator 使用；Rbf 使用 smooth）
    """
    def __init__(self, kernel: str = "multiquadric", epsilon: float = 1.0, smoothing: float = 0.0):
        self.kernel = kernel
        self.epsilon = float(epsilon)
        self.smoothing = float(smoothing)
        self.scaler: StdScaler | None = None
        self.model = None

    @staticmethod
    def _map_kernel_for_rbf(kernel: str) -> str:
        # 旧接口 Rbf 的函数名
        if kernel == "thin_plate_spline":
            return "thin_plate"
        return kernel

    def fit(self, X: np.ndarray, y: np.ndarray):
        self.scaler = StdScaler.fit(X)
        Z = self.scaler.transform(X)

        if HAVE_RBFI:
            self.model = _RBFI(
                Z, y.reshape(-1, 1),
                kernel=self.kernel,
                epsilon=self.epsilon,
                smoothing=self.smoothing,
            )
        else:
            self.model = _Rbf(
                Z[:, 0], Z[:, 1], y,
                function=self._map_kernel_for_rbf(self.kernel),
                epsilon=self.epsilon,
                smooth=self.smoothing
            )
        return self

    def predict(self, X: np.ndarray) -> np.ndarray:
        assert self.scaler is not None and self.model is not None, "模型尚未训练"
        Z = self.scaler.transform(X)
        if HAVE_RBFI:
            y = self.model(Z).ravel()
        else:
            y = self.model(Z[:, 0], Z[:, 1]).ravel()
        # 非负截断（可按需关掉）
        return np.maximum(y, 0.0)

# -----------------------------
# 误差度量 & 简易 K 折
# -----------------------------
def rmse(res: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(res))))

def mae(res: np.ndarray) -> float:
    return float(np.mean(np.abs(res)))

def kfold_indices(n: int, k: int, seed: int = 42):
    rng = np.random.default_rng(seed)
    idx = np.arange(n)
    rng.shuffle(idx)
    folds = np.array_split(idx, k)
    for i in range(k):
        te = folds[i]
        tr = np.concatenate([folds[j] for j in range(k) if j != i]) if k > 1 else te
        yield tr, te

# -----------------------------
# 子命令实现
# -----------------------------
def cmd_train(args):
    csv = Path(args.csv)
    h, v, y = load_ctr_csv(csv)
    X = np.column_stack([h, v])

    mdl = RBFModel(kernel=args.kernel, epsilon=args.epsilon, smoothing=args.smoothing).fit(X, y)
    outp = Path(args.out) if args.out else Path("./rbf_ctr.pkl")
    outp.write_bytes(pickle.dumps({
        "rbf": mdl,
        "meta": {
            "csv": str(csv),
            "kernel": args.kernel,
            "epsilon": float(args.epsilon),
            "smoothing": float(args.smoothing),
        }
    }))
    print(f"✔ 已保存模型：{outp}")

def cmd_xval(args):
    csv = Path(args.csv)
    h, v, y = load_ctr_csv(csv)
    X = np.column_stack([h, v])

    ks = int(args.kfold)
    rmses, maes = [], []
    for tr, te in kfold_indices(len(X), ks, seed=42):
        mdl = RBFModel(kernel=args.kernel, epsilon=args.epsilon, smoothing=args.smoothing).fit(X[tr], y[tr])
        yhat = mdl.predict(X[te])
        rmses.append(rmse(yhat - y[te]))
        maes.append(mae(yhat - y[te]))

    print(f"K={ks}  RMSE={np.mean(rmses):.6g} ± {np.std(rmses):.3g}  "
          f"MAE={np.mean(maes):.6g} ± {np.std(maes):.3g}")

def _load_model(path: Path) -> RBFModel:
    blob = pickle.loads(path.read_bytes())
    return blob["rbf"]

def _pick_cols(df: pd.DataFrame) -> Tuple[str, str]:
    names = _normalize_colnames(df.columns)
    name_map = {n: c for n, c in zip(names, df.columns)}
    h_col = next((name_map[k] for k in name_map if "z_over_l" in k or "z/l" in k or k == "z"), None)
    if h_col is None:
        raise KeyError("CSV 中未找到 z_over_l / z/l 列")
    v_col = next((name_map[k] for k in name_map if "velocity" in k or k == "v"), None)
    if v_col is None:
        raise KeyError("CSV 中未找到 velocity 列")
    return h_col, v_col

def cmd_predict(args):
    mdl = _load_model(Path(args.model))

    # 单点
    if args.h is not None and args.v is not None:
        h = float(args.h); v = abs(float(args.v))
        y = mdl.predict(np.array([[h, v]])).item()
        print(f"[RBF CTR] h={h:.6g}, v={v:.6g}  ->  {y:.6g}")

    # 批量 CSV
    if args.csv:
        df = pd.read_csv(args.csv)
        h_col, v_col = _pick_cols(df)
        X = np.column_stack([df[h_col].to_numpy(float),
                             np.abs(df[v_col].to_numpy(float))])
        df["CTR_pred_rbf"] = mdl.predict(X)
        outp = Path(args.out) if args.out else Path(args.csv).with_name(Path(args.csv).stem + "_pred.csv")
        df.to_csv(outp, index=False)
        print(f"✔ 已写出：{outp}")

# -----------------------------
# CLI
# -----------------------------
def main():
    ap = argparse.ArgumentParser(description="Two-variable RBF baseline for CTR(h, v)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    # train
    ap_tr = sub.add_parser("train", help="从 CSV 训练 RBF 模型")
    ap_tr.add_argument("--csv", type=str, required=True, help="ctr_merged_dataset.csv 路径")
    ap_tr.add_argument("--kernel", type=str, default="thin_plate_spline",
                       help="RBF kernel: multiquadric/inverse_multiquadric/linear/cubic/"
                            "thin_plate_spline/gaussian/quintic")
    ap_tr.add_argument("--epsilon", type=float, default=1.0, help="RBF 形状参数 epsilon")
    ap_tr.add_argument("--smoothing", type=float, default=1e-4, help="RBF 平滑项（RBFInterpolator 的 smoothing / Rbf 的 smooth）")
    ap_tr.add_argument("--out", type=str, default="./rbf_ctr.pkl", help="输出模型文件")
    ap_tr.set_defaults(func=cmd_train)

    # predict
    ap_pr = sub.add_parser("predict", help="模型预测（单点或 CSV 批量）")
    ap_pr.add_argument("--model", type=str, required=True, help="rbf_ctr.pkl")
    ap_pr.add_argument("--h", type=float, default=None, help="单点 h = z_over_l")
    ap_pr.add_argument("--v", type=float, default=None, help="单点 v = velocity (m/s)")
    ap_pr.add_argument("--csv", type=str, default=None, help="批量预测 CSV（追加 CTR_pred_rbf 列）")
    ap_pr.add_argument("--out", type=str, default=None, help="批量预测输出 CSV 路径")
    ap_pr.set_defaults(func=cmd_predict)

    # xval
    ap_xv = sub.add_parser("xval", help="K 折交叉验证")
    ap_xv.add_argument("--csv", type=str, required=True, help="ctr_merged_dataset.csv 路径")
    ap_xv.add_argument("--kfold", type=int, default=5, help="K 值（默认 5）")
    ap_xv.add_argument("--kernel", type=str, default="thin_plate_spline",
                       help="RBF kernel（同 train 子命令）")
    ap_xv.add_argument("--epsilon", type=float, default=1.0, help="epsilon（同 train）")
    ap_xv.add_argument("--smoothing", type=float, default=1e-4, help="smoothing（同 train）")
    ap_xv.set_defaults(func=cmd_xval)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()

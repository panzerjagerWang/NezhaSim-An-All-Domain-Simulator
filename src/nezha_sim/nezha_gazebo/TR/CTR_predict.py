#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CTR_predict.py
--------------
运行时推理：根据 (h = z_over_l, v = velocity) 预测 CTR

优先加载 piecewise_ctr.json（键：{"CTR": {"theta": [...], "boundary": ..., "sr_formula": "..."}}）
若未找到则回退加载 piecewise_ctr.pkl（含 sr_model 对象）。

用法示例：
  1) 单点预测
     python CTR_predict.py --h 0.5 --v 0.3

  2) 批量预测（CSV 追加一列 CTR_pred）
     python CTR_predict.py --csv ./data/samples.csv --out ./data/samples_pred.csv
     # 期待列名包含：z_over_l 或 z/l、velocity（速度正负均可，程序会取绝对值）

  3) 指定模型路径
     python CTR_predict.py --model ./piecewise_ctr.json
"""

from __future__ import annotations
import argparse
import json
import pickle
from pathlib import Path
from typing import Optional, Tuple

import numpy as np


# ────────────────────────────────────────────────────────────────
# 物理骨架：y = 1 + A1*exp(-B1*h) + A2*exp(-B2*h) + A3/(1+h)^n
# 其中 A1 = A10 + A11*v,  A2 = A20 + A21*v,  A3 = A30 + A31*v,
#      B1 = B10 + B11*v,  B2 = B20 + B21*v
# 每段参数向量 θ 段长为 11：
# [A10,A11, A20,A21, A30,A31, B10,B11, B20,B21, n]
# ────────────────────────────────────────────────────────────────
def _phys_core(h: np.ndarray, v: np.ndarray, theta11: np.ndarray) -> np.ndarray:
    A10, A11, A20, A21, A30, A31, B10, B11, B20, B21, n = theta11
    A1 = A10 + A11 * v
    A2 = A20 + A21 * v
    A3 = A30 + A31 * v
    B1 = B10 + B11 * v
    B2 = B20 + B21 * v
    return 1.0 + A1 * np.exp(-B1 * h) + A2 * np.exp(-B2 * h) + A3 / np.power(1.0 + h, n)


def _predict_piecewise_base(h: np.ndarray, v: np.ndarray, theta22: np.ndarray, boundary: float) -> np.ndarray:
    theta_left = theta22[:11]
    theta_right = theta22[11:]
    mask = h <= boundary
    y = np.empty_like(h, dtype=float)
    if mask.any():
        y[mask] = _phys_core(h[mask], v[mask], theta_left)
    if (~mask).any():
        y[~mask] = _phys_core(h[~mask], v[~mask], theta_right)
    return y


# ────────────────────────────────────────────────────────────────
# 残差表达式求值（来自 JSON 的 sr_formula）
# 约定变量：x0 → h,  x1 → v
# 支持运算：+,-,*,/,pow,exp,log,sqrt,sin,cos
# ────────────────────────────────────────────────────────────────
_ALLOWED_FUNCS = {
    "exp": np.exp,
    "log": np.log,
    "sqrt": np.sqrt,
    "sin": np.sin,
    "cos": np.cos,
    "pow": np.power,
    # 允许 numpy 命名空间（可选）
    "np": np,
    "numpy": np,
}

def _eval_sr_formula(sr_formula: str, h: np.ndarray, v: np.ndarray) -> np.ndarray:
    if not sr_formula or sr_formula.strip() in ("0", "0.0"):
        return np.zeros_like(h, dtype=float)

    # 有些表达式可能引用不同变量名，尽量兼容：
    local_ctx = {
        "x0": h, "x1": v,      # PySR 常用命名
        "h": h, "v": v,        # 兜底别名
        **_ALLOWED_FUNCS,
    }
    try:
        # 禁止内建函数，限制可见名，降低风险
        return np.asarray(eval(sr_formula, {"__builtins__": {}}, local_ctx), dtype=float)
    except Exception:
        # 回退：尝试把 'x0','x1' 替换为 'h','v'
        try:
            expr = sr_formula.replace("x0", "h").replace("x1", "v")
            return np.asarray(eval(expr, {"__builtins__": {}}, local_ctx), dtype=float)
        except Exception:
            # 最后兜底：返回 0 残差
            return np.zeros_like(h, dtype=float)


# ────────────────────────────────────────────────────────────────
# 模型封装
# ────────────────────────────────────────────────────────────────
class CTRModel:
    def __init__(self,
                 theta: np.ndarray,
                 boundary: float,
                 sr_formula: Optional[str] = None,
                 sr_model: Optional[object] = None,
                 clip_nonneg: bool = True):
        self.theta = np.asarray(theta, dtype=float)
        if self.theta.shape[0] != 22:
            raise ValueError(f"theta 长度应为 22（左11+右11），实际为 {self.theta.shape[0]}")
        self.boundary = float(boundary)
        self.sr_formula = sr_formula or "0"
        self.sr_model = sr_model    # 仅当从 PKL 读取时可用
        self.clip_nonneg = bool(clip_nonneg)

    @classmethod
    def from_json(cls, path: Path, clip_nonneg: bool = True) -> "CTRModel":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if "CTR" not in data:
            raise KeyError("JSON 中未找到 'CTR' 键")
        block = data["CTR"]
        theta = np.array(block["theta"], dtype=float)
        boundary = float(block["boundary"])
        sr_formula = block.get("sr_formula", "0")
        return cls(theta, boundary, sr_formula=sr_formula, sr_model=None, clip_nonneg=clip_nonneg)

    @classmethod
    def from_pkl(cls, path: Path, clip_nonneg: bool = True) -> "CTRModel":
        arte = pickle.loads(Path(path).read_bytes())
        theta = np.asarray(arte["theta"], dtype=float)
        boundary = float(arte["boundary"])
        sr_model = arte.get("sr_model", None)
        # pkl 中可能没有字符串表达式，这里置为 "0"
        return cls(theta, boundary, sr_formula="0", sr_model=sr_model, clip_nonneg=clip_nonneg)

    def predict(self, h, v) -> np.ndarray:
        """向量化预测；h = z_over_l，v = velocity（内部取绝对值）"""
        h = np.asarray(h, dtype=float)
        v = np.abs(np.asarray(v, dtype=float))

        # 广播到同形状
        h, v = np.broadcast_arrays(h, v)

        # 物理骨架
        y = _predict_piecewise_base(h, v, self.theta, self.boundary)

        # 残差修正（优先 JSON 字符串表达式，若无则尝试 pkl 的 sr_model）
        if self.sr_formula and self.sr_formula.strip() not in ("0", "0.0"):
            y += _eval_sr_formula(self.sr_formula, h, v)
        elif self.sr_model is not None:
            try:
                X = np.column_stack([h.ravel(), v.ravel()])
                res = np.asarray(self.sr_model.predict(X), dtype=float).reshape(h.shape)
                y += res
            except Exception:
                pass

        if self.clip_nonneg:
            y = np.maximum(y, 0.0)
        return y


# ────────────────────────────────────────────────────────────────
# CSV I/O 辅助
# ────────────────────────────────────────────────────────────────
def _normalize_colnames(cols):
    out = []
    for c in cols:
        s = str(c).strip().lower()
        for t in ["(m/s)", "(m^2)", "(m)", "(n)", "[m/s]", "[m]"]:
            s = s.replace(t, "")
        s = s.replace(" ", "")
        out.append(s)
    return out

def _pick_cols(df) -> Tuple[str, str]:
    names = _normalize_colnames(df.columns)
    name_map = {n: c for n, c in zip(names, df.columns)}

    h_col = None
    for key in name_map:
        if "z_over_l" in key or "z/l" in key or key == "z":
            h_col = name_map[key]; break
    if h_col is None:
        raise KeyError("CSV 中未找到 z_over_l / z/l 列")

    v_col = None
    for key in name_map:
        if "velocity" in key or key == "v":
            v_col = name_map[key]; break
    if v_col is None:
        raise KeyError("CSV 中未找到 velocity 列")

    return h_col, v_col


# ────────────────────────────────────────────────────────────────
# CLI
# ────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="CTR predictor (piecewise physics + SR residual)")
    ap.add_argument("--model", type=str, default=None,
                    help="模型文件路径（.json 优先；若未提供则在当前目录查找 piecewise_ctr.json/ .pkl）")
    ap.add_argument("--h", type=float, default=None, help="单点预测：z_over_l")
    ap.add_argument("--v", type=float, default=None, help="单点预测：velocity (m/s)")
    ap.add_argument("--csv", type=str, default=None, help="批量预测：输入 CSV 路径")
    ap.add_argument("--out", type=str, default=None, help="批量预测：输出 CSV 路径（默认在输入旁新增 *_pred.csv）")
    ap.add_argument("--no-clip", action="store_true", help="不对结果做非负截断（默认开启截断）")
    args = ap.parse_args()

    # 1) 准备模型路径
    model_path: Optional[Path] = Path(args.model) if args.model else None
    model: Optional[CTRModel] = None

    # 2) 加载模型：优先 JSON，再尝试 PKL
    tried = []
    if model_path:
        tried.append(str(model_path))
        if model_path.suffix.lower() == ".json":
            model = CTRModel.from_json(model_path, clip_nonneg=not args.no_clip)
        elif model_path.suffix.lower() == ".pkl":
            model = CTRModel.from_pkl(model_path, clip_nonneg=not args.no_clip)
        else:
            # 未指明扩展名，则先试 json 再试 pkl
            p_json = model_path.with_suffix(".json")
            p_pkl = model_path.with_suffix(".pkl")
            tried.extend([str(p_json), str(p_pkl)])
            if p_json.exists():
                model = CTRModel.from_json(p_json, clip_nonneg=not args.no_clip)
            elif p_pkl.exists():
                model = CTRModel.from_pkl(p_pkl, clip_nonneg=not args.no_clip)
    else:
        # 默认文件名
        p_json = Path("./piecewise_ctr.json")
        p_pkl = Path("./piecewise_ctr.pkl")
        tried.extend([str(p_json), str(p_pkl)])
        if p_json.exists():
            model = CTRModel.from_json(p_json, clip_nonneg=not args.no_clip)
        elif p_pkl.exists():
            model = CTRModel.from_pkl(p_pkl, clip_nonneg=not args.no_clip)

    if model is None:
        raise FileNotFoundError(f"未找到模型文件，请提供 --model；已尝试：{tried}")

    # 3) 单点预测
    if args.h is not None and args.v is not None:
        y = model.predict(args.h, args.v).item()
        print(f"[CTR] h={args.h:.6g}, v={args.v:.6g}  ->  {y:.6g}")
        # 如果同时提供了 CSV，也继续执行批量

    # 4) 批量预测（CSV）
    if args.csv:
        import pandas as pd
        in_path = Path(args.csv)
        df = pd.read_csv(in_path)
        h_col, v_col = _pick_cols(df)
        h = df[h_col].to_numpy(float)
        v = np.abs(df[v_col].to_numpy(float))
        df["CTR_pred"] = model.predict(h, v)
        out_path = Path(args.out) if args.out else in_path.with_name(in_path.stem + "_pred.csv")
        df.to_csv(out_path, index=False)
        print(f"已写出：{out_path}")

if __name__ == "__main__":
    main()

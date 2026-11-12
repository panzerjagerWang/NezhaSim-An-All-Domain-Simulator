# -*- coding: utf-8 -*-
"""
NezhaSim CFD 出水/浸没数据归档脚本（修正版）
- 时间列:  Physical Time (s)
- 总力列:  Fz Monitor (N)  ≡ TF = G + B + TR
- 动态浮力: B(z/L) = ρ g A_base * clip(L*(1 - z/L), 0, L)

输出两种归一化：
- CTR  (论文口径): TR / (0.5 ρ v^2 A_base)
- CTR_mg（对照）: TR / (m g)

注意：
- 仅处理原始输入（匹配 v 的 CSV），自动跳过派生文件（含 'ctr_', '<1', '_lt1', '_exit'）。
"""

import os
import re
import math
import numpy as np
import pandas as pd
from glob import glob

# ===== 常量与配置 =====
MASS_KG = 0.92
GRAVITY = 9.81
RHO_WATER = 1000.0              # kg/m^3
CYL_LENGTH_M = 0.350            # L (m)
A_BASE_M2 = 18_737.718e-6       # 18737.718 mm² -> m²
EXPORT_WITH_CTR_MG = False   # 如果你的下游脚本需要 CTR_mg，就改为 True

TIME_COL  = "Physical Time (s)"
FORCE_COL = "Fz Monitor (N)"    # TF (G+B+TR)

USE_FIRST_TIME_AS_T0 = True
MANUAL_T0_S = 0.0

INPUT_DIR = "./"
OUTPUT_DIR = "./"
TARGET_POINTS_PER_SPEED = 50     # >1 段抽样条数
TARGET_POINTS_LT1 = 50           # <1 段抽样条数

# ===== 工具函数 =====
def is_raw_input_file(path: str) -> bool:
    """只允许原始数据文件进入管线；排除派生输出。"""
    name = os.path.basename(path)
    blacklist_keys = ["ctr_", "<1", "_lt1", "_exit"]
    if any(k in name for k in blacklist_keys):
        return False
    return re.search(r"v[=\-_]?\d", name, flags=re.IGNORECASE) is not None

def parse_velocity_from_filename(fname: str) -> float:
    base = os.path.basename(fname).replace("p", ".")
    m = re.search(r"v[=\-_]?(\d+(?:\.\d+)?)", base, flags=re.IGNORECASE)
    if m: return float(m.group(1))
    m = re.search(r"(\d+(?:\.\d+)?)", base)  # 兜底
    return float(m.group(1)) if m else float("nan")

def z_over_l_from_time(t: pd.Series, v: float, L: float,
                       use_first_as_t0=True, manual_t0=0.0) -> pd.Series:
    t = pd.to_numeric(t, errors="coerce")
    t0 = t.iloc[0] if use_first_as_t0 else manual_t0
    v_abs = abs(v)  # 保证几何意义：出水 z/L 单调上升
    return (v_abs * (t - t0)) / L

def buoyancy_from_z_over_l(z_over_l: pd.Series) -> pd.Series:
    """B = ρ g A * L_sub, L_sub = clip(L*(1 - z/L), 0, L)"""
    sub_length = (CYL_LENGTH_M * (1 - z_over_l)).clip(lower=0.0, upper=CYL_LENGTH_M)
    V_sub = A_BASE_M2 * sub_length
    return RHO_WATER * GRAVITY * V_sub

def compute_ctr(TF: pd.Series, z_over_l: pd.Series, v: float) -> pd.Series:
    """
    CTR (论文口径，非负):
    TR = TF - B(z/L)
    CTR = |TR| / (0.5 ρ v² A_base)
    """
    B = buoyancy_from_z_over_l(z_over_l)
    TR = TF - B
    q = 0.5 * RHO_WATER * (abs(v) ** 2) * A_BASE_M2
    if q == 0:
        return pd.Series(np.nan, index=TR.index)
    return TR.abs() / q


def downsample_evenly(df: pd.DataFrame, n: int) -> pd.DataFrame:
    if len(df) <= n: return df.copy()
    idxs = np.linspace(0, len(df) - 1, n).astype(int)
    return df.iloc[idxs].copy()

def process_one_csv(path: str) -> pd.DataFrame:
    raw = pd.read_csv(path)
    if TIME_COL not in raw.columns:
        raise ValueError(f"未找到时间列 '{TIME_COL}'（{path}）")
    v = parse_velocity_from_filename(path)
    if math.isnan(v):
        raise ValueError(f"无法从文件名解析速度：{path}")

    zL = z_over_l_from_time(raw[TIME_COL], v, CYL_LENGTH_M,
                            use_first_as_t0=USE_FIRST_TIME_AS_T0,
                            manual_t0=MANUAL_T0_S)
    TF = pd.to_numeric(raw[FORCE_COL], errors="coerce")
    ctr_df = compute_ctr(TF, zL, v)  # ★ 传入 v
    CTR = compute_ctr(TF, zL, v)

    out = pd.DataFrame({
        "z_over_l": zL,
        "velocity": abs(v),
        "contact_area": A_BASE_M2,
        "CTR": CTR
    })
    out = pd.concat([out, ctr_df], axis=1)
    out = out.dropna(subset=["z_over_l", "CTR"]).sort_values("z_over_l").reset_index(drop=True)
    return out

def save_exit_segments_by_speed(dfs_by_v: dict, out_dir: str, max_points_each: int) -> list:
    saved = []
    for v, df in dfs_by_v.items():
        seg = df[df["z_over_l"] > 1.0].copy().sort_values("z_over_l")
        seg = downsample_evenly(seg, max_points_each)
        seg = seg[["z_over_l", "velocity", "contact_area", "CTR"]]
        out_name = f"ctr_v{str(abs(v)).replace('.', 'p')}_exit.csv"
        out_path = os.path.join(out_dir, out_name)
        seg.to_csv(out_path, index=False)
        saved.append(out_path)
    return saved

def save_lt1_per_input(path: str, df_full: pd.DataFrame, out_dir: str, max_points_each: int) -> str:
    """为每个输入文件导出 z_over_l < 1 段（部分/完全浸没）。"""
    seg = df_full[df_full["z_over_l"] < 1.0].copy().sort_values("z_over_l")
    seg = downsample_evenly(seg, max_points_each)
    seg = seg[["z_over_l", "velocity", "contact_area", "CTR"]]

    stem, _ = os.path.splitext(os.path.basename(path))
    preferred = f"{stem}<1.csv"
    fallback  = f"{stem}_lt1.csv"
    out_path_preferred = os.path.join(out_dir, preferred)
    out_path_fallback  = os.path.join(out_dir, fallback)

    try:
        seg.to_csv(out_path_preferred, index=False)
        return out_path_preferred
    except OSError:
        seg.to_csv(out_path_fallback, index=False)
        return out_path_fallback

def main():
    # 仅收集原始输入（过滤派生输出）
    patterns = [
        os.path.join(INPUT_DIR, "竖直v=*.csv"),
        os.path.join(INPUT_DIR, "v*.csv"),
        os.path.join(INPUT_DIR, "*v*case*.csv"),
    ]
    files_all = sorted(set(p for pt in patterns for p in glob(pt)))
    files = [f for f in files_all if is_raw_input_file(f)]

    # 兜底
    fallback = os.path.join(INPUT_DIR, "竖直v=1.csv")
    if not files and os.path.exists(fallback) and is_raw_input_file(fallback):
        files = [fallback]
    if not files:
        raise FileNotFoundError(f"在 {INPUT_DIR} 未找到原始 CSV。")

    by_v, errors = {}, []
    saved_lt1 = []

    for f in files:
        try:
            df = process_one_csv(f)
            # <1：每个输入单独导出
            out_lt1 = save_lt1_per_input(f, df, OUTPUT_DIR, TARGET_POINTS_LT1)
            saved_lt1.append(out_lt1)
            # >1：按速度聚合
            v = parse_velocity_from_filename(f)
            by_v[v] = pd.concat([by_v[v], df], ignore_index=True) if v in by_v else df
        except Exception as e:
            errors.append((f, str(e)))

    saved_exit = save_exit_segments_by_speed(by_v, OUTPUT_DIR, TARGET_POINTS_PER_SPEED)

    print("=== 处理完成 ===")
    print("输入文件（原始）：")
    for f in files: print("  -", f)
    print("\n导出（z_over_l<1，每个输入 ~50 条）：")
    for s in saved_lt1: print("  -", s)
    print("\n导出（z_over_l>1，按速度 ~50 条）：")
    for s in saved_exit: print("  -", s)
    if errors:
        print("\n以下文件处理失败：")
        for f, msg in errors: print(f"  - {f}: {msg}")

if __name__ == "__main__":
    main()

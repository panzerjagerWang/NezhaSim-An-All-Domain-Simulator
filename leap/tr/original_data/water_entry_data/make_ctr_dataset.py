#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_ctr_dataset.py  —  合并 water entry/exit 的描点数据（无表头两列：z_over_l, CTR）
输出：ctr_merged_dataset.csv，列：
  phase {entry|exit}, velocity(m/s), contact_area(m^2), z_over_l, CTR

支持的文件名：
  entry :  01v.csv  02v.csv  03v.csv  04v.csv  05v.csv   -> +0.1 ... +0.5
  exit  : -01v.csv -02v.csv -03v.csv -04v.csv -05v.csv   -> -0.1 ... -0.5

若只放 exit 或只放 entry 也可，脚本会自动跳过不存在的文件。
"""

from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
import pandas as pd
from math import pi

# ============ 论文几何参数（可替换成你自己的机体） ============
DC = 0.090   # cylinder diameter (m)
DA = 0.016   # arm diameter (m)
N_ARMS = 4   # number of arms

A_CYL  = pi * (DC**2) / 4.0
A_ARM1 = pi * (DA**2) / 4.0
A_ARMS = N_ARMS * A_ARM1

def contact_area_from_z(z_over_l: np.ndarray) -> np.ndarray:
    """按 z/l 计算投影接触面积（m^2）"""
    z = np.asarray(z_over_l, dtype=float)
    S = np.zeros_like(z, dtype=float)
    mask_cyl = (z > 0.25) & (z < 1.0)        # 只有圆柱
    S[mask_cyl] = A_CYL
    mask_cyl_arms = (z >= 0.0) & (z <= 0.25) # 圆柱 + 手臂
    S[mask_cyl_arms] = A_CYL + A_ARMS
    mask_below0 = (z < 0.0)                  # (如有) 进入水下：按“全入水”处理
    S[mask_below0] = A_CYL + A_ARMS
    # z>=1 默认 0（完全在水面上方）
    return S

# 速度映射（正=入水，负=出水）
SPEED_MAP = {
    "01v.csv":  +0.1, "02v.csv":  +0.2, "03v.csv":  +0.3, "04v.csv":  +0.4, "05v.csv":  +0.5,
    "-01v.csv": -0.1, "-02v.csv": -0.2, "-03v.csv": -0.3, "-04v.csv": -0.4, "-05v.csv": -0.5,
}

# 需要尝试读取的文件名（存在就读）
FILES = [f"{sign}{i:02d}v.csv" for sign in ("", ) for i in range(1, 6)]

def read_no_header_two_cols(file: Path) -> pd.DataFrame:
    """读取无表头描点数据（只取前两列：z_over_l, CTR；支持逗号或空白分隔，#注释）"""
    df = pd.read_csv(file, header=None, comment="#", engine="python", sep=r"[,\s]+")
    if df.shape[1] < 2:
        raise ValueError(f"{file.name}: 需要至少两列（z_over_l, CTR），当前列数={df.shape[1]}")
    df = df.iloc[:, :2].copy()
    df.columns = ["z_over_l", "CTR"]
    df["z_over_l"] = pd.to_numeric(df["z_over_l"], errors="coerce")
    df["CTR"]      = pd.to_numeric(df["CTR"], errors="coerce")
    return df.dropna(subset=["z_over_l", "CTR"])

def load_one(file: Path) -> pd.DataFrame:
    df = read_no_header_two_cols(file)

    fname = file.name
    if fname not in SPEED_MAP:
        raise ValueError(f"{fname}: 未在 SPEED_MAP 中指定速度（m/s）")
    vel = abs(float(SPEED_MAP[fname]))  # 取绝对值
    df["velocity"] = vel
    df["phase"] = "exit" if vel < 0 else "entry"

    # 接触面积由 z/l 计算
    df["contact_area"] = contact_area_from_z(df["z_over_l"].to_numpy())

    # 调整列顺序
    return      df[[ "z_over_l", "velocity", "contact_area", "CTR"]]

def main(argv: list[str]) -> None:
    merged, missing = [], []
    for name in FILES:
        p = Path(name)
        if not p.exists():
            missing.append(name); continue
        try:
            merged.append(load_one(p))
        except Exception as e:
            print(f"[!] 读取 {name} 失败：{e}")

    if not merged:
        raise SystemExit("未找到可读取的数据文件（entry/exit 任意一组均可）")

    out = pd.concat(merged, ignore_index=True)
    out = out.replace([np.inf, -np.inf], np.nan).dropna(subset=["velocity","contact_area","z_over_l","CTR"])
    # 合理范围裁剪（可按需调整）
    out = out[(out["z_over_l"] >= -0.05) & (out["z_over_l"] <= 1.05)]

    out_path = Path("ctr_merged_dataset.csv")
    out.to_csv(out_path, index=False)

    print(f"✅ 已生成 {out_path.resolve()}")
    print("列：phase, velocity(m/s), contact_area(m^2), z_over_l, CTR")
    # 仅提示哪些文件未出现，非错误
    not_found = [n for n in FILES if not Path(n).exists()]
    if not_found:
        print(f"ℹ️ 未发现的文件（忽略即可）：{not_found}")

if __name__ == "__main__":
    main(sys.argv)

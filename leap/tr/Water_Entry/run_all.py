#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_all.py — 一键串起 Water_Entry 项目的训练

流程：
  0) （可选）运行 data/data_process.py 生成/更新数据
  1) 训练 CTR 两变量一输出（分段物理骨架 + 残差符号回归）
  2) 训练 RBF 插值基线

文件结构（与你给的一致）：
  Water_Entry/
    data/merged_rows.csv
    rbf_interp_ctr/rbf_interp_ctr.py
    symbolic_regression/train_ctr_two_var_piecewise.py
"""

import subprocess
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent

def run(relpath: str, *args: str, check: bool = True, cwd: Path | None = None):
    """用当前 Python 解释器执行相对路径脚本"""
    script = BASE / relpath
    if not script.exists():
        raise FileNotFoundError(f"脚本不存在：{script}")
    cmd = [sys.executable, str(script), *map(str, args)]
    print(f"\n[RUN] {' '.join(cmd)}")
    subprocess.run(cmd, check=check, cwd=cwd or BASE)

def find_dataset() -> Path:
    """按你的目录优先查找数据文件；找不到就报错"""
    cands = [
        BASE / "data" / "merged_rows.csv",
        BASE / "data" / "ctr_merged_dataset.csv",
        Path("/mnt/data/merged_rows.csv"),
        Path("/mnt/data/ctr_merged_dataset.csv"),
    ]
    for p in cands:
        if p.exists():
            return p
    raise FileNotFoundError("未找到数据集：请把 CSV 放到 ./data/merged_rows.csv")

def main():


    csv_path = find_dataset()
    print(f"[INFO] 使用数据集：{csv_path}")

    # 1) CTR 分段 + SR 训练（输出 piecewise_ctr.json / .pkl 到项目根目录）
    train_args = [
        "--csv", str(csv_path),
        "--boundary", "1.0",     # 需要可改
        "--sr-iters", "1000"     # 需要可改；若未安装 PySR，脚本会自动跳过 SR
    ]
    run("symbolic_regression/train_ctr_two_var_piecewise.py", *train_args)

    # 2) RBF 基线（输出 rbf_ctr.pkl 到项目根目录）
    rbf_args = [
        "train",
        "--csv", str(csv_path),
        "--kernel", "thin_plate_spline",
        "--epsilon", "1.0",
        "--smoothing", "1e-4",
        "--out", str(BASE / "rbf_ctr.pkl")
    ]
    run("rbf_interp_ctr/rbf_interp_ctr.py", *rbf_args)

    print("\n✅ 完成：请查看输出文件：")
    print(f"   - {BASE/'piecewise_ctr.json'} / {BASE/'piecewise_ctr.pkl'}")
    print(f"   - {BASE/'rbf_ctr.pkl'}")

if __name__ == "__main__":
    main()

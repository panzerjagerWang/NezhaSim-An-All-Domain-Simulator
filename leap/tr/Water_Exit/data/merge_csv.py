#!/usr/bin/env python3
"""
将当前文件夹中所有 *.csv 合并为 merged_rows.csv
合并后列顺序与表头：z_over_l, velocity, CTR
velocity 从文件名提取：01v.csv → 0.1, 02v.csv → 0.2, 依此类推
"""

import glob
import os
import re
import pandas as pd

def filename_to_velocity(fname: str) -> float:
    """
    根据文件名提取速度值：
    - 取连续数字部分，如 01、02、15 …
    - 按 1/10 缩放：01 → 0.1，15 → 1.5
    """
    m = re.search(r'(\d+)', fname)
    if not m:
        raise ValueError(f'文件名中未找到速度数字: {fname}')
    val = int(m.group(1))
    return val / 10.0

def main():
    # 1. 找到所有 csv（可自行排除目标文件以防止再次读取）
    csv_files = [f for f in glob.glob("*.csv") if f != "merged_rows.csv"]
    if not csv_files:
        print("当前目录下没有找到 *.csv 文件。")
        return

    dfs = []
    for f in csv_files:
        # 2. 读入没有表头的 csv，指定列名
        df = pd.read_csv(f, header=None, names=["z_over_l", "CTR"])
        # 3. 增加 velocity 列
        df["velocity"] = filename_to_velocity(f)
        # 4. 统一列顺序
        df = df[["z_over_l", "velocity", "CTR"]]
        dfs.append(df)

    # 5. 合并并导出
    merged = pd.concat(dfs, ignore_index=True)
    merged.to_csv("merged_rows.csv", index=False)

    print(f"已生成 merged_rows.csv，共 {len(merged)} 行。")

if __name__ == "__main__":
    main()

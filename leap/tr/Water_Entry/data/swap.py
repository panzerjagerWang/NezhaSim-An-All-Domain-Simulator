#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
swap_columns.py
读取 merged_rows.csv，将 velocity 与 z_over_l 两列互换后写回新文件 merged_rows_swapped.csv
"""

import pandas as pd

def swap_velocity_and_zoverl(
        in_file: str = "merged_rows.csv",
        out_file: str = "merged_rows.csv") -> None:
    """
    交换 velocity 与 z_over_l 两列的位置、列名和数据。
    如果只想调整列顺序而保留原列名，取消相应 rename 即可。
    """
    # 1. 读取 CSV
    df = pd.read_csv(in_file)

    # 2. 交换列名（可选，不想改列名可跳过）
    df = df.rename(columns={"velocity": "z_over_l_tmp",
                            "z_over_l": "velocity"})
    df = df.rename(columns={"z_over_l_tmp": "z_over_l"})

    # 3. 调整列顺序 (velocity 放前还是 z_over_l 放前可自行决定)
    df = df[["z_over_l","velocity", "CTR"]]

    # 4. 写出新 CSV
    df.to_csv(out_file, index=False, encoding="utf-8")
    print(f"已生成: {out_file}")

if __name__ == "__main__":
    swap_velocity_and_zoverl()

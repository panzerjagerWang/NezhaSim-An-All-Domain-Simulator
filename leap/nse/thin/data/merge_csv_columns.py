#!/usr/bin/env python3
"""
merge_csv_rows.py
=================
功能
----
1. 扫描当前目录中全部 *.csv
2. 以 **表头 (列名) 为准纵向合并**，自动补齐缺失列
3. 可选:
   --source-col    把文件名写入一列，方便追溯
   -o/--output     指定输出文件名 (默认 merged_rows.csv)
   --sort-col      按字母排序列名
用法
----
# 最简单：把所有 csv 纵向追加
python merge_csv_rows.py

# 记录来源文件名，并指定输出
python merge_csv_rows.py --source-col _file -o all_data.csv

# 纵向追加后再对列名排序
python merge_csv_rows.py --sort-col
"""
from __future__ import annotations
import argparse
from pathlib import Path
import sys
import pandas as pd


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="根据行对齐、表头一致性纵向合并当前目录下所有 CSV")
    p.add_argument("-o", "--output", default="merged_rows.csv",
                   help="输出文件名 (默认为 merged_rows.csv)")
    p.add_argument("--source-col", metavar="COLNAME",
                   help="把原文件名写入指定列名；如 --source-col _file")
    p.add_argument("--sort-col", action="store_true",
                   help="合并后按字母顺序重新排序列名")
    return p.parse_args()


def read_one(fp: Path, source_col: str | None) -> pd.DataFrame:
    """读取单个 csv，并在需要时添加来源列"""
    df = pd.read_csv(fp)
    if source_col:
        df[source_col] = fp.stem          # 只保留文件名不带扩展名
    return df


def main() -> None:
    args = parse_args()
    csvs = sorted(Path(".").glob("*.csv"))
    if not csvs:
        print("❌ 当前目录未找到任何 .csv 文件", file=sys.stderr)
        sys.exit(1)

    # 读取全部表格
    dfs = [read_one(fp, args.source_col) for fp in csvs]

    # 按列名对齐纵向堆叠（outer join），缺失值会变 NaN
    merged = pd.concat(dfs, ignore_index=True, join="outer")

    # 按需排序列名
    if args.sort_col:
        merged = merged.reindex(sorted(merged.columns), axis=1)

    merged.to_csv(args.output, index=False)
    print(f"✅ 已合并 {len(csvs)} 个 CSV，共 {len(merged)} 行 → {args.output}")


if __name__ == "__main__":
    main()

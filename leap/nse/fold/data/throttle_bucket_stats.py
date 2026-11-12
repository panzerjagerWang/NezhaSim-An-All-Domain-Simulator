#!/usr/bin/env python3
# throttle_bucket_stats.py  (v2)
# ---------------------------------------------------------------------------
# ① 最近邻把 thrust_level 映射到 0.40 / 0.70 / 1.00
# ② 可输出：
#    --raw FILE : 映射后但“不聚合”的原始行
#    --avg FILE : 对同 (h_over_R, thrust_level) 的行取 KT、KQ 平均，只留一行
#    -o         : KT/KQ 均值、标准差、n 统计长表
#    --pivot    : 长表再转宽表
# ---------------------------------------------------------------------------
from __future__ import annotations
import argparse
from pathlib import Path
import numpy as np
import pandas as pd
from tabulate import tabulate

TARGET_LEVELS = np.array([0.40, 0.70, 1.00])


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="把 thrust_level 归并到 0.40/0.70/1.00 并导出多种格式")
    p.add_argument("csvs", nargs="+", help="输入 CSV 文件")
    p.add_argument("-o", "--output", default="bucket_stats.csv",
                   help="汇总长表输出名")
    p.add_argument("--pivot", action="store_true",
                   help="是否同时生成透视宽表")
    p.add_argument("--raw", metavar="FILE",
                   help="保存最近邻映射后的原始行")
    p.add_argument("--avg", metavar="FILE",
                   help="保存( h_over_R, thrust_level )聚合后的平均表")
    return p.parse_args()


def read_concat(paths: list[str]) -> pd.DataFrame:
    return pd.concat([pd.read_csv(p) for p in paths], ignore_index=True)


def bucketize(df: pd.DataFrame) -> pd.DataFrame:
    levels = df["thrust_level"].to_numpy()[:, None]
    idx = np.abs(levels - TARGET_LEVELS[None, :]).argmin(1)

    out = df.copy()
    out.rename(columns={"thrust_level": "orig_thrust_level"}, inplace=True)
    out["thrust_level"] = TARGET_LEVELS[idx]           # 覆盖
    # 排列列顺序
    core = ["h_over_R", "thrust_level", "R", "KT", "KQ"]
    return out[core + [c for c in out.columns if c not in core]]


def average_rows(df_bkt: pd.DataFrame) -> pd.DataFrame:
    """
    对同 (h_over_R, thrust_level) 求 KT、KQ 的平均，
    保留 R 的首值（通常恒定）。
    列顺序与 025_processed.csv 一致
    """
    grp = (df_bkt.groupby(["h_over_R", "thrust_level"])
                   .agg(R=("R", "first"),
                        KT=("KT", "mean"),
                        KQ=("KQ", "mean"))
                   .reset_index()
                   .sort_values(["h_over_R", "thrust_level"]))
    return grp[["h_over_R", "thrust_level", "R", "KT", "KQ"]]


def aggregate(df_bkt: pd.DataFrame) -> pd.DataFrame:
    grp = (df_bkt.groupby(["h_over_R", "thrust_level"])
           .agg(KT_mean=("KT", "mean"),
                KT_std=("KT", "std"),
                KQ_mean=("KQ", "mean"),
                KQ_std=("KQ", "std"),
                n=("KT", "size"))
           .reset_index()
           .sort_values(["h_over_R", "thrust_level"]))
    return grp


def make_pivot(stats: pd.DataFrame) -> pd.DataFrame:
    piv = stats.pivot(index="h_over_R",
                      columns="thrust_level",
                      values=["KT_mean", "KQ_mean"])
    piv.columns = [f"{m}@{lvl:.2f}" for m, lvl in piv.columns]
    return piv.reset_index().sort_values("h_over_R")


def main() -> None:
    a = parse_args()

    df_orig = read_concat(a.csvs)
    df_bkt  = bucketize(df_orig)

    if a.raw:
        df_bkt.to_csv(a.raw, index=False)
        print(f"✔ raw 行级文件 → {a.raw}")

    if a.avg:
        df_avg = average_rows(df_bkt)
        df_avg.to_csv(a.avg, index=False)
        print(f"✔ avg 平均后文件 → {a.avg}")

    # 长表统计
    stats = aggregate(df_bkt)
    stats.to_csv(a.output, index=False)
    print(f"✔ 统计长表 → {a.output}")

    if a.pivot:
        piv_path = Path(a.output).with_stem(Path(a.output).stem + "_pivot")
        make_pivot(stats).to_csv(piv_path, index=False)
        print(f"  ↳ 透视宽表 → {piv_path}")

    print("\n预览（avg 或 stats 前几行）：")
    preview = average_rows(df_bkt) if a.avg else stats
    print(tabulate(preview.head(12), headers="keys", floatfmt=".4f"))


if __name__ == "__main__":
    main()

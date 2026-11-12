#!/usr/bin/env python3
# throttle_bucket_stats_022.py
# 专用：把 022_processed.csv → {022_raw.csv, 022_avg.csv, 022_stats.csv, 022_stats_pivot.csv}

from pathlib import Path
import numpy as np
import pandas as pd
from tabulate import tabulate

# -------------------- 常量区（如需改动仅改这里） --------------------
CSV_IN      = "022_processed.csv"         # 输入文件
RAW_OUT     = "022_raw.csv"               # 映射后行级
AVG_OUT     = "022_avg.csv"               # (h_over_R, thrust_level) 均值
STATS_OUT   = "022_stats.csv"             # 长表：均值、标准差、n
PIVOT_OUT   = "022_stats_pivot.csv"       # 宽表
TARGET_LEVELS = np.array([0.40, 0.70, 1.00])
# -------------------------------------------------------------------

def bucketize(df: pd.DataFrame) -> pd.DataFrame:
    levels = df["thrust_level"].to_numpy()[:, None]
    idx = np.abs(levels - TARGET_LEVELS[None, :]).argmin(1)

    out = df.copy()
    out.rename(columns={"thrust_level": "orig_thrust_level"}, inplace=True)
    out["thrust_level"] = TARGET_LEVELS[idx]
    core = ["h_over_R", "thrust_level", "R", "KT", "KQ"]
    return out[core + [c for c in out.columns if c not in core]]

def average_rows(df_bkt: pd.DataFrame) -> pd.DataFrame:
    grp = (df_bkt.groupby(["h_over_R", "thrust_level"])
           .agg(R=("R", "first"),
                KT=("KT", "mean"),
                KQ=("KQ", "mean"))
           .reset_index()
           .sort_values(["h_over_R", "thrust_level"]))
    return grp[["h_over_R", "thrust_level", "R", "KT", "KQ"]]

def aggregate(df_bkt: pd.DataFrame) -> pd.DataFrame:
    return (df_bkt.groupby(["h_over_R", "thrust_level"])
            .agg(KT_mean=("KT", "mean"),
                 KT_std=("KT", "std"),
                 KQ_mean=("KQ", "mean"),
                 KQ_std=("KQ", "std"),
                 n=("KT", "size"))
            .reset_index()
            .sort_values(["h_over_R", "thrust_level"]))

def make_pivot(stats: pd.DataFrame) -> pd.DataFrame:
    piv = stats.pivot(index="h_over_R",
                      columns="thrust_level",
                      values=["KT_mean", "KQ_mean"])
    piv.columns = [f"{m}@{lvl:.2f}" for m, lvl in piv.columns]
    return piv.reset_index().sort_values("h_over_R")

def main() -> None:
    print(f"▶ 读取 {CSV_IN} ...")
    df_orig = pd.read_csv(CSV_IN)

    # # 1 归档三档位
    df_bkt = bucketize(df_orig)
    # df_bkt.to_csv(RAW_OUT, index=False)
    # print(f"✔ 行级文件 → {RAW_OUT}")

    # 2 均值表
    df_avg = average_rows(df_bkt)
    df_avg.to_csv(AVG_OUT, index=False)
    print(f"✔ 平均表   → {AVG_OUT}")

    # # 3 长表 (均值 + 标准差 + n)
    # stats = aggregate(df_bkt)
    # stats.to_csv(STATS_OUT, index=False)
    # print(f"✔ 统计长表 → {STATS_OUT}")
    #
    # # 4 宽表
    # make_pivot(stats).to_csv(PIVOT_OUT, index=False)
    # print(f"✔ 透视宽表 → {PIVOT_OUT}")
    #
    # # 预览
    # print("\n预览（长表前 10 行）:")
    # print(tabulate(stats.head(10), headers="keys", floatfmt=".4f"))

if __name__ == "__main__":
    main()

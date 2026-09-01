#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import seaborn as sns
import glob
import os
from scipy.signal import argrelmin
from scipy.interpolate import CubicSpline

# ==================== 配置 ===================
plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams['pdf.fonttype'] = 42
sns.set_theme(style="whitegrid", font="Times New Roman")

INPUT_DIR = "processed_data_100pts"
INPUT_PATTERN = "trajectory_data_with_forces*.csv"

CONFIDENCE_TYPE = 'std'
CONFIDENCE_MULTIPLIER = 1.0
SMOOTH_WINDOW = 3
OSC_DIST_START = 35.0   # 振荡段起始距离（映射后）


# ==================== 工具函数 ===================
def vec_mag(df, prefix: str) -> np.ndarray:
    x = df.get(f"{prefix}_x", pd.Series(np.zeros(len(df))))
    y = df.get(f"{prefix}_y", pd.Series(np.zeros(len(df))))
    z = df.get(f"{prefix}_z", pd.Series(np.zeros(len(df))))
    return np.sqrt(x.astype(float)**2 + y.astype(float)**2 + z.astype(float)**2)


def safe_minmax_scale(arr, out_min, out_max):
    arr = np.asarray(arr, dtype=float)
    a_min, a_max = np.nanmin(arr), np.nanmax(arr)
    if not np.isfinite(a_min) or not np.isfinite(a_max) or abs(a_max - a_min) < 1e-12:
        return np.full_like(arr, fill_value=out_min, dtype=float)
    return out_min + (arr - a_min) * (out_max - out_min) / (a_max - a_min)


def build_mission_distance(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    if "timestamp" in df.columns:
        df = df.sort_values("timestamp").reset_index(drop=True)
    if "robot_type" not in df.columns or "x" not in df.columns:
        raise ValueError("CSV 必须包含 robot_type 与 x 列。")

    rt = df["robot_type"].astype(str).str.upper()
    df["Distance"] = np.nan

    # UAV: [0, 20]（反向）
    m = rt == "UAV"
    if m.any():
        x = df.loc[m, "x"].astype(float).values
        x_rev = np.nanmax(x) - x
        df.loc[m, "Distance"] = safe_minmax_scale(x_rev, 0.0, 20.0)

    # UUV: [20, 40]
    m = rt == "UUV"
    if m.any():
        df.loc[m, "Distance"] = safe_minmax_scale(
            df.loc[m, "x"].astype(float).values, 20.0, 40.0
        )

    # 兜底
    unknown = df["Distance"].isna()
    if unknown.any():
        df.loc[unknown, "Distance"] = safe_minmax_scale(
            df.loc[unknown, "x"].astype(float).values, 0.0, 40.0
        )
    return df


def apply_moving_average(df, window_size, cols_to_smooth):
    df_smooth = df.copy()
    window_size = int(max(1, window_size))
    for col in cols_to_smooth:
        if col in df_smooth.columns:
            df_smooth[col] = (
                df_smooth[col]
                .rolling(window=window_size, min_periods=1, center=True)
                .mean()
            )
    return df_smooth


# ==================== 核心：极小值下包络平滑 ====================
def smooth_by_lower_envelope(dist_arr: np.ndarray,
                              wave_arr: np.ndarray,
                              osc_start: float,
                              order: int = 5) -> np.ndarray:
    result = wave_arr.copy()
    osc_mask = dist_arr >= osc_start
    if osc_mask.sum() < 4:
        return result

    osc_d = dist_arr[osc_mask]
    osc_w = wave_arr[osc_mask]

    min_idx = argrelmin(osc_w, order=order)[0]

    anchor_d = [osc_d[0]]
    anchor_w = [osc_w[0]]

    if len(min_idx) >= 1:
        anchor_d += list(osc_d[min_idx])
        anchor_w += list(osc_w[min_idx])

    # 末尾锚点取末尾10%的最小值
    tail_n = max(1, len(osc_w) // 10)
    anchor_d.append(osc_d[-1])
    anchor_w.append(np.min(osc_w[-tail_n:]))

    anchor_d = np.array(anchor_d)
    anchor_w = np.array(anchor_w)

    sort_idx = np.argsort(anchor_d)
    anchor_d = anchor_d[sort_idx]
    anchor_w = anchor_w[sort_idx]
    _, unique_idx = np.unique(anchor_d, return_index=True)
    anchor_d = anchor_d[unique_idx]
    anchor_w = anchor_w[unique_idx]

    if len(anchor_d) < 2:
        print("[下包络] 节点不足，跳过替换")
        return result

    cs = CubicSpline(anchor_d, anchor_w, bc_type='not-a-knot')
    result[osc_mask] = cs(osc_d)
    print(f"[下包络] {len(anchor_d)} 个节点，替换 {osc_mask.sum()} 个点 (order={order})")
    return result


# ==================== 数据读取 ===================
def load_single_file(csv_file: str) -> pd.DataFrame:
    df = pd.read_csv(csv_file)
    if "surface_z" not in df.columns:
        raise ValueError(f"{csv_file} 必须包含 surface_z 列。")

    # 去除 x/y/z 完全重复的行
    xyz_cols = [c for c in ["x", "y", "z"] if c in df.columns]
    before = len(df)
    df = df.drop_duplicates(subset=xyz_cols).reset_index(drop=True)
    after = len(df)
    if before != after:
        print(f"[去重] 删除 {before - after} 条重复行（{before} → {after}）")

    df = build_mission_distance(df)
    df = df.sort_values("Distance").reset_index(drop=True)

    dist_arr = df["Distance"].astype(float).values
    wave_arr = df["surface_z"].astype(float).values

    # 对振荡段用下包络替换
    wave_clean = smooth_by_lower_envelope(
        dist_arr, wave_arr,
        osc_start=OSC_DIST_START,
        order=5
    )

    df_processed = pd.DataFrame({
        "Distance":    dist_arr,
        "Buoyancy":    vec_mag(df, "buoyancy"),
        "Damping":     vec_mag(df, "damping"),
        "Added_Mass":  vec_mag(df, "added_mass"),
        "Coriolis":    vec_mag(df, "coriolis"),
        "Wave_Force":  vec_mag(df, "wave"),
        "Wave_Height": wave_clean,
        "robot_type":  df["robot_type"].astype(str)
    })
    return df_processed.sort_values("Distance").reset_index(drop=True)


def load_and_average_data(input_dir, pattern):
    files = glob.glob(os.path.join(input_dir, pattern))
    if not files:
        raise ValueError(f"未找到匹配文件：{os.path.join(input_dir, pattern)}")

    print(f"找到 {len(files)} 个文件")
    all_dfs = [load_single_file(f) for f in files]

    # 以最短文件的点数为基准，统一插值对齐
    min_len = min(len(df) for df in all_dfs)
    distance_base = all_dfs[0]["Distance"].values[:min_len]
    numeric_cols = ["Buoyancy", "Damping", "Added_Mass",
                    "Coriolis", "Wave_Force", "Wave_Height"]

    data_array = np.stack(
        [[df[col].values[:min_len] for col in numeric_cols] for df in all_dfs],
        axis=0
    )
    data_array = data_array.transpose(0, 2, 1)

    mean_values = np.mean(data_array, axis=0)
    if CONFIDENCE_TYPE == 'sem':
        std_values = np.std(data_array, axis=0, ddof=1) / np.sqrt(len(files))
    else:
        ddof = 1 if len(files) > 1 else 0
        std_values = np.std(data_array, axis=0, ddof=ddof)

    df_mean = pd.DataFrame({"Distance": distance_base,
                             **{c: mean_values[:, i]
                                for i, c in enumerate(numeric_cols)}})
    df_std  = pd.DataFrame({"Distance": distance_base,
                             **{c: std_values[:, i] * CONFIDENCE_MULTIPLIER
                                for i, c in enumerate(numeric_cols)}})
    return df_mean, df_std


# ==================== 绘图 ===================
def plot_forces_with_confidence(df_mean: pd.DataFrame, df_std: pd.DataFrame):
    if df_mean is None or df_mean.empty:
        print("无数据。")
        return

    df_mean = df_mean.copy()
    df_std  = df_std.copy()

    # IQR 裁剪 UAV 段 Damping + Wave_Force 异常高值
    uav_mask = df_mean["Distance"] < 20.0
    for col in ["Damping", "Wave_Force"]:
        seg = df_mean.loc[uav_mask, col]
        if len(seg) > 0:
            q1, q3 = seg.quantile(0.25), seg.quantile(0.75)
            upper  = q3 + 3.0 * (q3 - q1)
            bad    = uav_mask & (df_mean[col] > upper)
            if bad.any():
                df_mean.loc[bad, col] = upper
                print(f"[IQR裁剪] {col}：{bad.sum()} 个异常点 → 上限 {upper:.4f} N")

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(10, 6), sharex=True,
        gridspec_kw={"height_ratios": [3, 1]}
    )

    distance = df_mean["Distance"].values

    # ================== 上图 (Top Plot) 映射 ==================
    # 原 x 轴 0~40 映射到 0~6.5，保持图形形状不变
    scale_top = 6.5 / 40.0
    distance_top = distance * scale_top
    SPLIT_TOP = 20.0 * scale_top

    force_configs = [
        ("Buoyancy",   "#2ecc71", 2.5, "-"),
        ("Damping",    "#e74c3c", 2.5, "-"),
        ("Added_Mass", "#3498db", 2.5, "-"),
        ("Coriolis",   "#9b59b6", 2.0, "--"),
        ("Wave_Force", "#f1c40f", 2.0, "-"),
    ]

    for col, color, lw, ls in force_configs:
        mean_v = df_mean[col].values
        std_v  = df_std[col].values
        ax1.plot(distance_top, mean_v, label=col.replace("_", " "),
                 linewidth=lw, color=color, linestyle=ls)
        ax1.fill_between(distance_top, mean_v - std_v, mean_v + std_v,
                         color=color, alpha=0.2)

    ax1.axvline(x=SPLIT_TOP, color="gray", linestyle=":", alpha=0.6)

    # 设置上图 Y 轴为 -2.5 到 10
    ax1.set_ylim(-2.5, 10)

    # 调整文本和矩形框位置 (适应新的 Y 轴 -2.5~10)
    y_text = 8.5
    ax1.text(SPLIT_TOP / 2, y_text, "UAV", ha="center",
             fontsize=14, fontweight="bold", color="#666")
    ax1.text((SPLIT_TOP + 6.5) / 2, y_text, "UUV", ha="center",
             fontsize=14, fontweight="bold", color="#666")

    rect_width = 3.0 * scale_top
    rect_x = (20.0 - 1.5) * scale_top
    # 矩形框从 -2.5 开始，高度为 12.5 (10 - (-2.5))
    rect_air_water = patches.Rectangle(
        (rect_x, -2.5), rect_width, 12.5,
        linewidth=2, edgecolor="#3498db", facecolor="none", linestyle="--"
    )
    ax1.add_patch(rect_air_water)
    
    # 调整说明文字的 Y 轴位置到 -1.0
    ax1.text(
        SPLIT_TOP, -1.0,
        "Transition Process\nFrom Air to Water",
        ha="center", va="top", fontsize=11,
        bbox=dict(facecolor="white", edgecolor="none", alpha=0.7)
    )

    ax1.set_ylabel("Force Magnitude (N)", fontsize=12, fontweight="bold")
    ax1.legend(loc="upper right", frameon=True, shadow=True, fontsize=10)
    ax1.grid(True, alpha=0.3)

    # ================== 下图 (Bottom Plot) 映射 ==================
    # 只保留 0~34 的部分，并映射到整个 0~6.5 的 X 轴
    bot_mask = distance <= 34.0
    scale_bot = 6.5 / 34.0
    distance_bot = distance[bot_mask] * scale_bot
    SPLIT_BOT = 20.0 * scale_bot

    wave_mean = df_mean["Wave_Height"].values[bot_mask]
    wave_std  = df_std["Wave_Height"].values[bot_mask]

    ax2.plot(distance_bot, wave_mean, label="Wave Surface Height",
             color="#1abc9c", linewidth=2)
    ax2.fill_between(distance_bot,
                     wave_mean - wave_std,
                     wave_mean + wave_std,
                     color="#1abc9c", alpha=0.2,
                     label=f"±{CONFIDENCE_MULTIPLIER}σ Confidence Interval")

    ax2.axvline(x=SPLIT_BOT, color="gray", linestyle=":", alpha=0.6)

    ax2.set_ylabel("Surface Z (m)", fontsize=12, fontweight="bold")
    ax2.set_xlabel("Distance Traveled (m)", fontsize=12, fontweight="bold")
    ax2.legend(loc="upper right", fontsize=10)
    ax2.grid(True, alpha=0.3)

    # 设置统一的 X 轴范围 0~6.5
    ax1.set_xlim(0, 6.5)

    plt.tight_layout()
    plt.savefig("fig_S1_force.pdf", dpi=300, bbox_inches="tight")
    print("图表已保存: fig_S1_force.pdf")
    plt.show()


# ==================== 主程序 ===================
if __name__ == "__main__":
    df_mean, df_std = load_and_average_data(INPUT_DIR, INPUT_PATTERN)

    cols_to_smooth = ["Buoyancy", "Damping", "Added_Mass", "Coriolis", "Wave_Force"]
    df_mean_s = apply_moving_average(df_mean, SMOOTH_WINDOW, cols_to_smooth)
    df_std_s  = apply_moving_average(df_std,  SMOOTH_WINDOW, cols_to_smooth)

    plot_forces_with_confidence(df_mean_s, df_std_s)

    print("\n=== 统计摘要 ===")
    n = len(glob.glob(os.path.join(INPUT_DIR, INPUT_PATTERN)))
    print(f"样本数: {n} | 数据点数: {len(df_mean)}")
    print(f"实际距离范围: [{df_mean['Distance'].min():.2f}, {df_mean['Distance'].max():.2f}] m")
    for col in ["Buoyancy", "Damping", "Added_Mass", "Coriolis", "Wave_Force"]:
        print(f"  {col}: [{df_mean[col].min():.4f}, {df_mean[col].max():.4f}] N")


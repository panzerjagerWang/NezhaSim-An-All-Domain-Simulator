import pandas as pd
import numpy as np

def process_force_file_simple_shape(
        path,
        v,
        L=1.0,
        smooth=True,
        smooth_window=11,   # 小窗口滚动平均（奇数）
        min_points=40,
        t0_mode='threshold',  # 'threshold' | 'first_positive'
        threshold_ratio=0.02, # 阈值 = 全段正值均值 * ratio（备用）
        absolute_threshold=None, # 若给定，则直接用这个绝对阈值
        p_low=1.0,            # 下对齐分位（1%）
        p_high=99.0,          # 缩放参考分位（99%）
        resample_points=150,
        clip_to_velocity=True,
        verbose=True
    ):
    """
    将 (time, Fz) 转成 (z_over_l, C)；保证 C 范围大致 0~v（或 <=v）。
    不做激进 baseline，避免整段变 0。
    """

    df = pd.read_csv(path)
    # 自动列名
    time_col = [c for c in df.columns if 'Time' in c or 'time' in c][0]
    force_col = [c for c in df.columns if 'Fz' in c or 'Force' in c or 'Monitor' in c][0]

    t_raw = df[time_col].to_numpy()
    F_raw = df[force_col].to_numpy()

    # 简单去 NaN
    mask_valid = np.isfinite(t_raw) & np.isfinite(F_raw)
    t_raw = t_raw[mask_valid]
    F_raw = F_raw[mask_valid]

    if smooth and len(F_raw) >= smooth_window:
        # 滚动平均平滑
        s = pd.Series(F_raw)
        F_smooth = s.rolling(smooth_window, center=True, min_periods=1).mean().to_numpy()
    else:
        F_smooth = F_raw.copy()

    # 选 t0
    if t0_mode == 'first_positive':
        idx_pos = np.where(F_smooth > 0)[0]
        t0 = t_raw[idx_pos[0]] if len(idx_pos) else t_raw[0]
    else:  # threshold
        # 定义阈值
        if absolute_threshold is not None:
            thr = absolute_threshold
        else:
            # 取正值部分均值 * ratio，如果正值很少则回退到 0
            pos_vals = F_smooth[F_smooth > 0]
            mean_pos = pos_vals.mean() if len(pos_vals) else 0.0
            thr = mean_pos * threshold_ratio
            thr = max(thr, 0.0)  # 不让它是负
        # 找首次连续 5 点 >= thr 的起点
        win = 5
        t0 = t_raw[0]
        for i in range(0, len(F_smooth)-win):
            if np.all(F_smooth[i:i+win] >= thr):
                t0 = t_raw[i]
                break

    # 计算 z
    z_raw = v * (t_raw - t0)
    mask_z = z_raw >= 0
    z = z_raw[mask_z]
    F_use = F_smooth[mask_z]

    if len(F_use) < min_points:
        if verbose:
            print(f"[WARN] 有效点不足，len={len(F_use)}，尝试放宽条件或检查文件。")
    # 不做“去所有负值”，只是在后面对齐分位时处理

    # 下分位对齐
    low_val = np.percentile(F_use, p_low)
    F_shift = F_use - low_val
    F_shift[F_shift < 0] = 0.0

    # 高分位参考
    high_val = np.percentile(F_shift, p_high)
    if high_val <= 0:
        # 兜底：全部置 0
        C = np.zeros_like(F_shift)
    else:
        C = (F_shift / high_val) * v

    if clip_to_velocity:
        C = np.clip(C, 0, v)

    # 防止全 0：如果非零占比 < 5%，放宽策略：用 max 而不是分位
    nonzero_ratio = (C > 0).mean()
    if nonzero_ratio < 0.05 and high_val > 0:
        if verbose:
            print("[INFO] 非零比例过低，改用最大值缩放重算。")
        high_val2 = F_shift.max()
        if high_val2 > 0:
            C = (F_shift / high_val2) * v
            if clip_to_velocity:
                C = np.clip(C, 0, v)

    # 重新采样
    z_over_l = z / L
    if resample_points and len(z_over_l) > 5:
        z_min, z_max = z_over_l.min(), z_over_l.max()
        z_uni = np.linspace(z_min, z_max, resample_points)
        C_uni = np.interp(z_uni, z_over_l, C)
        z_over_l, C = z_uni, C_uni

    out = pd.DataFrame({'z_over_l': z_over_l, 'C': C})

    if verbose:
        print(f"[DEBUG] path={path}")
        print(f"        t0={t0:.6f}, low(p{p_low})={low_val:.4g}, high(p{p_high})={high_val:.4g}")
        print(f"        C范围: min={C.min():.4g}, max={C.max():.4g}, 非零比例={nonzero_ratio:.2%}")

    return out

# ==== 示例调用 ====
df_v05 = process_force_file_simple_shape('竖直v=0.5.csv', v=0.5,
                                         smooth=True, smooth_window= 11)
# 上面一行里 'nine:=11' 如果你复制粘贴报错, 直接改成 11 即可:
# smooth_window=11

df_v1  = process_force_file_simple_shape('竖直v=1.csv', v=1.0,
                                         smooth=True, smooth_window=11)

df_v05.to_csv('05v_scaled_fix.csv', index=False)
df_v1.to_csv('1v_scaled_fix.csv', index=False)

print(df_v05.head())
print(df_v1.head())

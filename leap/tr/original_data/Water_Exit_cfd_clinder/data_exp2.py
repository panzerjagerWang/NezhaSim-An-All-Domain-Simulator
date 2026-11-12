import pandas as pd
import numpy as np

def rescale_and_downsample_subset(
        in_csv,
        out_csv,
        target_z = 0.5330335570469521,
        n_points = 50,
        z_col='z_over_l',
        c_col='C',
        atol=1e-9,
        verbose=True
    ):
    df = pd.read_csv(in_csv)
    if z_col not in df or c_col not in df:
        raise ValueError("列名不正确")
    z = df[z_col].to_numpy()
    C = df[c_col].to_numpy()

    # 找 target
    idx_exact = np.where(np.isclose(z, target_z, atol=atol))[0]
    if len(idx_exact)==0:
        nearest_idx = int(np.argmin(np.abs(z-target_z)))
        target_z_eff = z[nearest_idx]
        if verbose:
            print(f"[INFO] 用最接近值 {target_z_eff}")
    else:
        target_z_eff = target_z

    mask = z <= target_z_eff
    z_sub = z[mask]
    C_sub = C[mask]

    # 线性缩放 z
    span = z_sub[-1] - z_sub[0]
    if span == 0:
        raise ValueError("截取区间 z 无跨度")
    z_scaled = (z_sub - z_sub[0]) / span  # 最后一个就是 1

    # 如果原始点数 <= n_points，直接全部输出
    if len(z_scaled) <= n_points:
        out_df = pd.DataFrame({z_col: z_scaled, c_col: C_sub})
    else:
        # 均匀选取索引
        indices = np.linspace(0, len(z_scaled)-1, n_points).round().astype(int)
        out_df = pd.DataFrame({z_col: z_scaled[indices], c_col: C_sub[indices]})

    out_df.to_csv(out_csv, index=False)
    if verbose:
        print(f"[DONE] 子集抽样 {len(out_df)} 点保存到 {out_csv}")
    return out_df

# 示例
if __name__ == "__main__":
    rescale_and_downsample_subset("1v_scaled_fix.csv", "1v_scaled_fix_rescaled_subset_50.csv")

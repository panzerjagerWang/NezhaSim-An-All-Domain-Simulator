import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
import glob
import os


def interpolate_trajectory(df, target_points=100):
    """
    对轨迹数据进行插值，统一点位数

    参数:
    df: DataFrame - 输入的轨迹数据
    target_points: int - 目标点位数

    返回:
    DataFrame - 插值后的数据
    """
    # 获取原始点数
    original_points = len(df)

    # 如果点数已经符合要求，直接返回
    if original_points == target_points:
        return df

    # 创建新的索引（用于插值）
    old_indices = np.linspace(0, 1, original_points)
    new_indices = np.linspace(0, 1, target_points)

    # 存储插值后的数据
    interpolated_data = {}

    # 对数值列进行插值
    numeric_columns = df.select_dtypes(include=[np.number]).columns
    for col in numeric_columns:
        f = interp1d(old_indices, df[col].values, kind='cubic', fill_value='extrapolate')
        interpolated_data[col] = f(new_indices)

    # 对非数值列（如phase_name, robot_type）使用最近邻插值
    categorical_columns = df.select_dtypes(exclude=[np.number]).columns
    for col in categorical_columns:
        # 使用最近邻方法
        indices = np.round(new_indices * (original_points - 1)).astype(int)
        indices = np.clip(indices, 0, original_points - 1)
        interpolated_data[col] = df[col].iloc[indices].values

    return pd.DataFrame(interpolated_data)


def process_csv_files(input_pattern='trajectory_data_with_forces*.csv',
                      output_dir='processed_data',
                      target_points=100):
    """
    批量处理CSV文件

    参数:
    input_pattern: str - 输入文件的匹配模式
    output_dir: str - 输出目录
    target_points: int - 目标点位数
    """
    # 创建输出目录
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 获取所有匹配的文件
    files = glob.glob(input_pattern)
    files.sort()  # 按文件名排序

    print(f"找到 {len(files)} 个文件待处理\n")

    # 统计信息
    stats = []

    for file in files:
        print(f"处理文件: {file}")

        # 读取CSV
        df = pd.read_csv(file)
        original_count = len(df)

        # 按robot_type分组处理
        robot_types = df['robot_type'].unique()
        processed_dfs = []

        for robot_type in robot_types:
            # 筛选当前类型的数据
            type_df = df[df['robot_type'] == robot_type].copy()
            type_original_count = len(type_df)

            # 插值处理
            type_interpolated = interpolate_trajectory(type_df, target_points)
            processed_dfs.append(type_interpolated)

            print(f"  - {robot_type}: {type_original_count} 点 -> {len(type_interpolated)} 点")

        # 合并所有类型的数据
        result_df = pd.concat(processed_dfs, ignore_index=True)

        # 保存结果
        output_file = os.path.join(output_dir, os.path.basename(file))
        result_df.to_csv(output_file, index=False)

        # 记录统计信息
        stats.append({
            'file': os.path.basename(file),
            'original_points': original_count,
            'processed_points': len(result_df),
            'robot_types': ', '.join(robot_types)
        })

        print(f"  总计: {original_count} 点 -> {len(result_df)} 点")
        print(f"  已保存至: {output_file}\n")

    # 输出统计摘要
    print("=" * 60)
    print("处理完成！统计摘要：")
    print("=" * 60)
    stats_df = pd.DataFrame(stats)
    print(stats_df.to_string(index=False))

    # 保存统计信息
    stats_df.to_csv(os.path.join(output_dir, 'processing_stats.csv'), index=False)
    print(f"\n统计信息已保存至: {os.path.join(output_dir, 'processing_stats.csv')}")


# 使用示例
if __name__ == "__main__":
    # 方案1: 每个robot_type统一为100个点
    process_csv_files(
        input_pattern='trajectory_data_with_forces*.csv',
        output_dir='processed_data_100pts',
        target_points=100
    )

    # 方案2: 如果你想要不同的点数，可以修改target_points参数
    # process_csv_files(
    #     input_pattern='trajectory_data_with_forces*.csv',
    #     output_dir='processed_data_150pts',
    #     target_points=150
    # )

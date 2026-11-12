import pandas as pd
import numpy as np


def make_ctr_positive(input_file='merged_rows.csv', output_file='merged_rows.csv'):
    """
    读取CSV文件，将CTR列的负值转换为正值

    Parameters:
    input_file (str): 输入文件名
    output_file (str): 输出文件名
    """

    # 读取CSV文件
    df = pd.read_csv(input_file)

    # 显示原始数据的基本信息
    print("原始数据信息:")
    print(f"数据形状: {df.shape}")
    print(f"CTR列的范围: {df['CTR'].min():.6f} 到 {df['CTR'].max():.6f}")
    print(f"负值数量: {(df['CTR'] < 0).sum()}")
    print(f"正值数量: {(df['CTR'] > 0).sum()}")

    # 将CTR列的所有值转换为正值（取绝对值）
    df['CTR'] = np.abs(df['CTR'])

    # 显示转换后的数据信息
    print("\n转换后数据信息:")
    print(f"CTR列的范围: {df['CTR'].min():.6f} 到 {df['CTR'].max():.6f}")
    print(f"负值数量: {(df['CTR'] < 0).sum()}")
    print(f"正值数量: {(df['CTR'] > 0).sum()}")

    # 保存到新文件
    df.to_csv(output_file, index=False)
    print(f"\n数据已保存到: {output_file}")

    return df


# 执行脚本
if __name__ == "__main__":
    # 转换数据
    df_positive = make_ctr_positive()

    # 显示前几行数据作为验证
    print("\n转换后的前10行数据:")
    print(df_positive.head(10))


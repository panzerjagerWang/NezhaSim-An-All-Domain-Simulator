import pandas as pd

# 读取CSV文件
df = pd.read_csv('smoothed_trajectory_data.csv')

# 找到UAV类型的数据并将z坐标提高0.2米
df.loc[df['vehicle_type'] == 'UAV', 'z'] = df.loc[df['vehicle_type'] == 'UAV', 'z'] + 0.2

# 直接覆盖原文件
df.to_csv('smoothed_trajectory_data.csv', index=False)

print("UAV的z坐标已提高0.2米并保存到原文件")

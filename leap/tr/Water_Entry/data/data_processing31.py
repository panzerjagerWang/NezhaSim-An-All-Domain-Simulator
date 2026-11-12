import pandas as pd

# 1. 读取原始文件
df = pd.read_csv("merged_rows.csv")

# 2. 调换前两列（v ↔ h）
df = df.iloc[:, [1, 0, 2]]

# 3. 重命名列名
df.columns = ["z_over_l", "velocity", "CTR"]

# 4. 保存为新文件
df.to_csv("merged_rows_swapped.csv", index=False)

print("完成！已生成 merged_rows_swapped.csv")

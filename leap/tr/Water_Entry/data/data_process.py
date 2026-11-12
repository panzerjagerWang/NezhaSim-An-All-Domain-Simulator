import pandas as pd

input_file = "merged_rows.csv"
output_file = "merged_rows.csv"

# 读取
df = pd.read_csv(input_file)

# 如果列存在则删除
if "contact_area" in df.columns:
    df = df.drop(columns=["contact_area"])

# 保存（保持原有顺序里去掉该列）
df.to_csv(output_file, index=False)

print(f"已保存到 {output_file}")

import pandas as pd

def main():
    # 1. 读取文件
    infile  = "merged_rows.csv"          # 输入文件
    outfile = "merged_rows_sorted.csv"   # 输出文件

    # 2. 利用 pandas 读取 CSV
    df = pd.read_csv(infile)

    # 3. 先按 velocity 升序，再按 z_over_l 升序排序
    df_sorted = df.sort_values(by=["velocity", "z_over_l"],
                               ascending=[True, True],
                               ignore_index=True)

    # 4. 保存排序后的结果
    df_sorted.to_csv(outfile, index=False)

    # 5. 打印到终端检查
    print(df_sorted)

if __name__ == "__main__":
    main()

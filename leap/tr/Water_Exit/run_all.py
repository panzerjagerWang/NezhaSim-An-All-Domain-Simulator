#!/usr/bin/env python3
# run_all.py ── 调度两个子模块，依次训练并保存 RBF 插值模型和符号回归模型
import subprocess, sys
from pathlib import Path

BASE = Path(__file__).resolve().parent

def run(module_relative_path: str):
    """用当前 Python 解释器执行单个脚本文件"""
    full = BASE / module_relative_path
    print(f"\n[RUN] {full}")
    subprocess.run([sys.executable, str(full)], check=True)


def run_all():
    # run("data/smooth_and_denoise.py")
    # # # # # # 1) 训练 + 保存线性插值模型
    run("rbf_interp_ctr/rbf_interp_ctr.py")
    # # # # # # # # 2) 训练 + 保存符号回归模型
    run("symbolic_regression/train_ctr_two_var_piecewise.py")

if __name__ == "__main__":
    run_all()
    print("\n✅  完成：模型已保存在各子目录里。")
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_nse_all.py
==================================================================
Driver: run make_nse_report.py across all propeller variants and aggregate
the per-variant metrics into one combined CSV.

Run with the SAME interpreter you want the report to use, e.g. leakage-free:
    ~/anaconda3/envs/pysr_env/bin/python run_nse_all.py \
        --variants thick thin winglet fold --target both \
        --split extrapolation --pysr --sr-iter 300

Each variant writes <variant>/nse_report/ ; combined outputs go to
./nse_report_combined/ .
"""
from __future__ import annotations
import argparse, json, subprocess, sys
from pathlib import Path

import pandas as pd

HERE = Path(__file__).resolve().parent
REPORT = HERE / "thin" / "make_nse_report.py"   # the generic harness (uses --root)


def run_variant(variant_dir: Path, passthrough: list[str]) -> dict | None:
    cmd = [sys.executable, str(REPORT), "--root", str(variant_dir)] + passthrough
    print(f"\n========== {variant_dir.name} ==========\n$ {' '.join(cmd)}")
    subprocess.run(cmd, check=True)
    mj = variant_dir / "nse_report" / "metrics.json"
    return json.loads(mj.read_text()) if mj.exists() else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variants", nargs="+", default=["thick", "thin", "winglet", "fold"])
    ap.add_argument("--parent", default=str(HERE))
    # everything else is passed through to make_nse_report.py
    ap.add_argument("--target", default="both")
    ap.add_argument("--split", default="extrapolation")
    ap.add_argument("--pysr", action="store_true")
    ap.add_argument("--sr-iter", type=int, default=300)
    ap.add_argument("--scaling", action="store_true")
    args = ap.parse_args()

    passthrough = ["--target", args.target, "--split", args.split]
    if args.pysr:    passthrough += ["--pysr", "--sr-iter", str(args.sr_iter)]
    if args.scaling: passthrough += ["--scaling"]

    parent = Path(args.parent)
    all_rows = []
    for v in args.variants:
        vdir = parent / v
        if not (vdir / "data" / "merged_rows.csv").exists():
            print(f"[skip] {v}: no data/merged_rows.csv"); continue
        rep = run_variant(vdir, passthrough)
        if not rep: continue
        for tgt, blk in rep["targets"].items():
            for m in blk["metrics"]:
                all_rows.append(dict(variant=v, target=tgt, **m))

    out_dir = parent / "nse_report_combined"; out_dir.mkdir(exist_ok=True)
    df = pd.DataFrame(all_rows)
    tag = args.split
    df.to_csv(out_dir / f"metrics_all_{tag}.csv", index=False)

    # console summary
    print("\n================ COMBINED ================")
    if not df.empty:
        with pd.option_context("display.width", 160, "display.max_rows", None):
            print(df[["variant", "target", "model", "RMSE", "MaxErr", "t_curve_ms"]]
                  .to_string(index=False))
    print(f"\n[done] {out_dir}/metrics_all_{tag}.csv")


if __name__ == "__main__":
    main()

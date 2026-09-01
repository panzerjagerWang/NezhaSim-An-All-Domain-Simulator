#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_tr_model.py — score a deployed TR (transmedium-resistance) C_TR model JSON
against experimental ground-truth data.

The TR model predicts the transmedium resistance coefficient C_TR as a function
of normalized depth h = z/l and speed v. This script re-implements the exact
runtime predictor (physics skeleton + symbolic residual, see CTR_predict.py) from
the JSON and reports RMSE / max-error versus a ground-truth CSV.

Usage:
  python3 verify_tr_model.py \
      --model Water_Entry.json \
      --csv   /path/to/Resis_Extract/Water_Entry/data/merged_rows.csv

The CSV must have columns: z_over_l, velocity, CTR.
"""
import argparse, json
import numpy as np

_F = {"exp": np.exp, "log": np.log, "sqrt": np.sqrt, "sin": np.sin,
      "cos": np.cos, "tanh": np.tanh, "pow": np.power}


def _phys_core(h, v, th):
    A10, A11, A20, A21, A30, A31, B10, B11, B20, B21, n = th
    A1, A2, A3 = A10 + A11 * v, A20 + A21 * v, A30 + A31 * v
    B1, B2 = B10 + B11 * v, B20 + B21 * v
    return 1.0 + A1 * np.exp(-B1 * h) + A2 * np.exp(-B2 * h) + A3 / np.power(1.0 + h, n)


def predict(js, h, v):
    """y = piecewise physics base + symbolic residual, clipped to >= 0."""
    th = np.asarray(js["theta"], float)
    b = js["piecewise"]["boundary_on_z_over_l"]
    y = np.where(h <= b, _phys_core(h, v, th[:11]), _phys_core(h, v, th[11:]))
    res = js.get("equation_residual", "0")
    if res and res.strip() not in ("0", "0.0"):
        expr = res.replace("^", "**")              # JSON uses ^ for power
        y = y + np.asarray(eval(expr, {"__builtins__": {}},
                                {"x0": h, "x1": v, **_F}), float)
    return np.clip(y, 0.0, None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="TR model JSON (e.g. Water_Entry.json)")
    ap.add_argument("--csv", required=True, help="ground-truth CSV (z_over_l,velocity,CTR)")
    a = ap.parse_args()

    js = json.load(open(a.model))
    data = np.genfromtxt(a.csv, delimiter=",", names=True)
    h, v, gt = data["z_over_l"], np.abs(data["velocity"]), data["CTR"]

    pred = predict(js, h, v)
    err = pred - gt
    rmse, mx, rng = np.sqrt(np.mean(err ** 2)), np.max(np.abs(err)), gt.max() - gt.min()
    print(f"model : {a.model}")
    print(f"N     : {len(gt)}   CTR range [{gt.min():.3f}, {gt.max():.3f}]")
    print(f"RMSE  : {rmse:.4f}   ({100 * rmse / rng:.2f}% of range)")
    print(f"MaxErr: {mx:.4f}   ({100 * mx / rng:.2f}% of range)")


if __name__ == "__main__":
    main()

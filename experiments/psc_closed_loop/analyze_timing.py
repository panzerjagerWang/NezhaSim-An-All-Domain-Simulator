#!/usr/bin/env python3
"""Per-physics-step cost of the NSE and TR plugins on Nezha-Mini.

Each profiled plugin wrote one float32 duration in microseconds per call to
<run>/profile/<tag>_<pid>_<n>.f32. The motor (NSE), TR and unified (PSC)
plugins and the world step timer are called once per physics step, so their
series are aligned on the step index. The first second of every run is
skipped. LEAP query time is the total time inside the closed-form
evaluations divided by the number of steps.

Usage: analyze_timing.py <results dir> <arm> <out json>
"""
import glob
import json
import os
import sys

import numpy as np

root, arm, out = sys.argv[1], sys.argv[2], sys.argv[3]
SKIP = 500  # 1 s at the 2 ms step

plug, step, psc, leap_us, nsteps, runs = [], [], [], 0.0, 0, 0
for run in sorted(glob.glob(os.path.join(root, f"{arm}_[0-9]*"))):  # psc_* would also match psc_rs_*
    if not os.path.exists(os.path.join(run, "meta.json")):
        continue  # run in progress
    meta = json.load(open(os.path.join(run, "meta.json")))
    if meta["status"] != "done":
        continue
    prof = os.path.join(run, "profile")
    load = lambda tag: [np.fromfile(f, dtype=np.float32) for f in sorted(glob.glob(os.path.join(prof, f"{tag}_*.f32")))]
    motors, tr, world, uni = load("nse_motor"), load("tr_drag"), load("world_step"), load("psc_unified")
    if len(motors) != 4 or len(tr) != 1 or len(world) != 1:
        continue
    n = min(min(len(m) for m in motors), len(tr[0]), len(world[0]), len(uni[0]))
    tail = lambda a: a[len(a) - n:][SKIP:]       # align on the last step
    per_step = sum(tail(m) for m in motors) + tail(tr[0])
    plug.append(per_step); step.append(tail(world[0])); psc.append(tail(uni[0]))
    leap = sum(float(a[SKIP:].sum()) for a in load("nse_leap_query") + load("tr_leap_query"))
    leap_us += leap; nsteps += n - SKIP; runs += 1

plug, step, psc = np.concatenate(plug), np.concatenate(step), np.concatenate(psc)
q = lambda a, p: float(np.percentile(a, p))
res = dict(arm=arm, runs=runs, steps=int(len(plug)), step_budget_us=2000.0,
           leap_query_mean_us=leap_us / nsteps,
           nse_tr_mean_us=float(plug.mean()), nse_tr_p99_us=q(plug, 99), nse_tr_max_us=float(plug.max()),
           psc_mean_us=float(psc.mean()), psc_p99_us=q(psc, 99),
           world_step_mean_us=float(step.mean()), world_step_p99_us=q(step, 99),
           world_step_max_us=float(step.max()), frac_steps_over_budget=float((step > 2000.0).mean()),
           world_step_p999_us=q(step, 99.9), steps_over_10ms=int((step > 10000.0).sum()),
           nse_tr_p999_us=q(plug, 99.9))
json.dump(res, open(out, "w"), indent=1)
print(json.dumps(res, indent=1))

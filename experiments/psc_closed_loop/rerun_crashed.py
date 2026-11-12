#!/usr/bin/env python3
"""Repeat every run that ended in a gzserver crash, with the same seed.

A crashed run is moved to <results>/crashed/<name>_try<k> and recorded in
<results>/crashes.json; the trial is then run again, up to three times.

Usage: rerun_crashed.py <results dir> <run_trial.py> [--port 11411]
"""
import glob
import json
import os
import shutil
import subprocess
import sys

root, runner = sys.argv[1], sys.argv[2]
port = sys.argv[sys.argv.index("--port") + 1] if "--port" in sys.argv else "11411"
log_path = os.path.join(root, "crashes.json")
crashes = json.load(open(log_path)) if os.path.exists(log_path) else []
os.makedirs(os.path.join(root, "crashed"), exist_ok=True)


def crashed(d):
    m = os.path.join(d, "meta.json")
    return (not os.path.exists(m)) or json.load(open(m))["status"] != "done"


for attempt in range(1, 4):
    todo = [d for d in sorted(glob.glob(os.path.join(root, "psc*_[0-9][0-9]"))) if crashed(d)]
    if not todo:
        break
    for d in todo:
        name = os.path.basename(d)
        meta = json.load(open(os.path.join(d, "meta.json"))) if os.path.exists(os.path.join(d, "meta.json")) else {}
        k = 1 + sum(1 for c in crashes if c["run"] == name)
        dst = os.path.join(root, "crashed", f"{name}_try{k}")
        shutil.move(d, dst)
        crashes.append(dict(run=name, arm=meta.get("arm"), seed=meta.get("seed"), attempt=k,
                            status=meta.get("status"), sim_end=meta.get("sim_end"), t_cmd=meta.get("t_cmd"),
                            startup=(meta.get("sim_end") or 0.0) < (meta.get("t_cmd") or 0.0), moved_to=dst))
        json.dump(crashes, open(log_path, "w"), indent=1)
        arm, seed = name.rsplit("_", 1)
        subprocess.run(["timeout", "300", "python3", runner, "--arm", arm, "--seed", str(int(seed)),
                        "--out", root, "--profile", "--port", port], check=False)
left = [os.path.basename(d) for d in glob.glob(os.path.join(root, "psc*_[0-9][0-9]")) if crashed(d)]
print("crash records", len(crashes), "still crashed", left)

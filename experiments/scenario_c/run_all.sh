#!/bin/bash
# Run both arms of Scenario C side by side, N runs per arm.
# usage: run_all.sh [N] [OUT]
HERE=$(cd "$(dirname "$0")" && pwd)
N=${1:-100}
OUT=${2:-$HERE/results}
"$HERE/run_arm.sh" nezha_f "$N" "$OUT/nezhasim" 11511 &
"$HERE/run_arm.sh" nezha_f_bl "$N" "$OUT/baseline" 11811 &
wait

#!/bin/bash
# Robust XX-F baseline run: clean -> launch gazebo (detached) -> run the recorder
# for N epochs, writing outputs directly into the figure baseline folder.
set -u
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS=${NEZHA_WS:-$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)}
FIG=${NEZHA_F_OUTPUT_DIR:-$SCRIPT_DIR}
N=${1:-10}

mkdir -p "$FIG"
export NEZHA_F_OUTPUT_DIR="$FIG"

cd "$WS"
source devel/setup.bash
export GAZEBO_PLUGIN_PATH="$WS/devel/lib:${GAZEBO_PLUGIN_PATH:-}"

echo "[f-run] cleaning..."
for p in gzserver gzclient roslaunch rosmaster rosout baseline_f_recorder lee_position publish_world; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 2
pid=$(ss -tlnp 2>/dev/null | grep 11311 | grep -oP 'pid=\K[0-9]+' | head -1)
[ -n "$pid" ] && kill -9 "$pid"
sleep 1
rosclean purge -y >/dev/null 2>&1 || true

echo "[f-run] launching gazebo (detached)..."
setsid bash -c "roslaunch nezha_gazebo nezha_f_baseline.launch gui:=false paused:=false verbose:=false > /tmp/f_L.log 2>&1" < /dev/null &
disown

echo "[f-run] waiting for robot..."
for t in $(seq 1 50); do
  rostopic list 2>/dev/null | grep -q "/nezha_f_bl/ground_truth/odometry" && break
  sleep 3
done
if ! rostopic list 2>/dev/null | grep -q "/nezha_f_bl/ground_truth/odometry"; then
  echo "[f-run] ERROR: robot never came up"; tail -8 /tmp/f_L.log; exit 1
fi
sleep 6
pgrep -x gzserver >/dev/null && echo "[f-run] gazebo UP" || { echo "[f-run] gazebo crashed"; exit 1; }

echo "[f-run] running recorder for $N epochs (output -> $FIG)..."
cd "$FIG"
rm -f position_*.txt velocity_*.txt baseline_f_run_*.bag
python3 -u "$SCRIPT_DIR/baseline_f_recorder.py" "$N" > /tmp/f_run.log 2>&1
echo "[f-run] recorder done. tail:"; tail -4 /tmp/f_run.log

echo "[f-run] cleanup..."
for p in gzserver gzclient roslaunch rosmaster baseline_f_recorder; do pkill -9 -f "$p" 2>/dev/null; done
echo "[f-run] DONE. epochs(lines) in position_z.txt: $(wc -l < "$FIG/position_z.txt" 2>/dev/null)"

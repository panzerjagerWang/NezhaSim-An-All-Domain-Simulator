#!/bin/bash
# Run one arm of Scenario C until N runs are logged, restarting Gazebo after a crash.
# usage: run_arm.sh ARM N OUT PORT     (ARM = nezha_f or nezha_f_bl; PORT = a free ROS master port)
# NEZHA_WS is the built catkin workspace, /r3/nezha_ws by default.
ARM=$1; N=$2; OUT=$3; PORT=$4
HERE=$(cd "$(dirname "$0")" && pwd)
NEZHA_WS=${NEZHA_WS:-/r3/nezha_ws}
source /opt/ros/noetic/setup.bash
source "$NEZHA_WS/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:$PORT
export GAZEBO_MASTER_URI=http://localhost:$((PORT + 100))
export GAZEBO_PLUGIN_PATH=$NEZHA_WS/devel/lib:${GAZEBO_PLUGIN_PATH:-}
case "$ARM" in
  nezha_f) LAUNCH=nezha_f.launch ;;
  nezha_f_bl) LAUNCH=nezha_f_baseline.launch ;;
  *) echo "unknown arm $ARM"; exit 2 ;;
esac
mkdir -p "$OUT"
tries=0
while [ "$(cat "$OUT/runs.jsonl" 2>/dev/null | wc -l)" -lt "$N" ] && [ "$tries" -lt 40 ]; do
  tries=$((tries + 1))
  setsid roslaunch -p "$PORT" nezha_gazebo "$LAUNCH" gui:=false paused:=false use_sim_time:=true > "$OUT/launch_$tries.log" 2>&1 &
  LPID=$!
  up=0
  for _ in $(seq 1 60); do
    rostopic list 2>/dev/null | grep -q "/$ARM/ground_truth/odometry" && { up=1; break; }
    sleep 3
  done
  if [ "$up" = 1 ]; then
    sleep 6
    python3 -u "$HERE/run_scc.py" --ns "$ARM" --n "$N" --out "$OUT" --waypoints "$HERE/waypoints.csv" >> "$OUT/run.log" 2>&1
    echo "try $tries exit $? runs $(cat "$OUT/runs.jsonl" 2>/dev/null | wc -l)" >> "$OUT/run.log"
  else
    echo "try $tries: robot never came up" >> "$OUT/run.log"
  fi
  kill -INT -- "-$LPID" 2>/dev/null
  sleep 5
  kill -KILL -- "-$LPID" 2>/dev/null
  sleep 2
done
echo "arm $ARM finished: $(cat "$OUT/runs.jsonl" 2>/dev/null | wc -l) runs in $tries tries" >> "$OUT/run.log"

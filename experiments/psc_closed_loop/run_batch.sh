#!/bin/bash
# Closed-loop PSC vs PSC-RS batch: seeds 0..29, both arms, sequential.
# run from the workspace root after sourcing devel/setup.bash
OUT=${1:-results}
mkdir -p $OUT
HERE=$(cd $(dirname $0) && pwd)
for s in $(seq 0 29); do
  for a in psc_rs psc; do
    if [ -f $OUT/${a}_$(printf %02d $s)/odom.csv ]; then continue; fi
    timeout 300 python3 $HERE/run_trial.py --arm $a --seed $s --out $OUT --profile | tail -1 >> $OUT/batch.log
  done
done
echo BATCH_DONE >> $OUT/batch.log

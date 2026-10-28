#!/usr/bin/env bash
# Ivan live again on the two point-2 recordings (same code, his config default), also saving his keyframes for the maps
cd ~/eval_live/dimos/dimos/navigation/jnav/components/loop_closure/eval_loop
export EVAL_LOOP_OUT=~/eval_live/out PYTHONPATH=~/eval_live/dimos
for n in small_loop_go2 huge_loop_go2_fastlio; do
  echo "== $n $(date +%H:%M) load $(cut -d" " -f1 /proc/loadavg) containers $(docker ps -q | wc -l)"
  ~/repos/dimos/.venv/bin/python ivan_live.py $n 2>&1 | grep -E "ivan_live|Error|Traceback" | tail -3
done
echo MAPSDONE

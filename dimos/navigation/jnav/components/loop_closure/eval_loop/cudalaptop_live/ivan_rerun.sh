#!/usr/bin/env bash
# on CudaLaptop: Ivan's module again on every recording, emitting its map every 5 frames as the go2 blueprint runs it
cd ~/eval_live/dimos/dimos/navigation/jnav/components/loop_closure/eval_loop
export EVAL_LOOP_OUT=~/eval_live/out PYTHONPATH=~/eval_live/dimos
PY=~/repos/dimos/.venv/bin/python
NAMES="mid360_athens_stairs hk_village3 hk_village5 hk_village2 hk_village1 hk_village6 hk_village4 go2_hongkong_office small_loop_go2 roscon office555 huge_loop_pointlio huge_loop_fastlio huge_loop_go2_pointlio huge_loop_go2_fastlio huge_loop_go2"
for n in $NAMES; do
  echo "== $n $(date +%H:%M) load $(cut -d' ' -f1 /proc/loadavg) containers $(docker ps -q | wc -l)"
  $PY ivan_live.py $n 2>&1 | grep -vE "^\S+ \[inf\]" | tail -4
done
$PY run.py --tables
echo IVANDONE

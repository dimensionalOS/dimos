#!/usr/bin/env bash
# on CudaLaptop: once the batch prints RUNDONE, redo small_loop_go2 gsc_live (its 18:22 run crashed) and reprint the tables
until grep -q RUNDONE ~/eval_live/run_2026-10-18.log; do sleep 60; done
cd ~/eval_live/dimos/dimos/navigation/jnav/components/loop_closure/eval_loop
export EVAL_LOOP_OUT=~/eval_live/out PGO_EVAL=~/eval_live/target/release/pgo-eval PYTHONPATH=~/eval_live/dimos
PY=~/repos/dimos/.venv/bin/python
echo "== small_loop_go2 gsc_live $(date +%H:%M)"
$PY variant.py gsc_live "{}" small_loop_go2 2>&1 | tail -20
$PY run.py --tables
echo AFTERDONE

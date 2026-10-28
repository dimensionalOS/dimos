#!/usr/bin/env bash
# gsc_live on huge_loop_go2 again, with pgo-eval reading scans by key (its RAM was the old reader's sort)
cd ~/eval_live/dimos/dimos/navigation/jnav/components/loop_closure/eval_loop
export EVAL_LOOP_OUT=~/eval_live/out PGO_EVAL=~/eval_live/target/release/pgo-eval PYTHONPATH=~/eval_live/dimos
PY=~/repos/dimos/.venv/bin/python
echo "start $(date +%H:%M)"
$PY variant.py gsc_live "{}" huge_loop_go2 2>&1 | grep -v gnc-solve | tail -5
$PY run.py --tables huge_loop_go2 2>&1 | grep huge_loop_go2
rm -f ~/eval_live/out/huge_loop_go2.gsc.db*
echo HLG2DONE

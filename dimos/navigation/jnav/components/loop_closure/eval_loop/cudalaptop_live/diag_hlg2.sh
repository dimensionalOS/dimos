#!/usr/bin/env bash
cd ~/eval_live/dimos/dimos/navigation/jnav/components/loop_closure/eval_loop
export EVAL_LOOP_OUT=~/eval_live/out PGO_EVAL=~/eval_live/target/release/pgo-eval PYTHONPATH=~/eval_live/dimos
PY=~/repos/dimos/.venv/bin/python; n=huge_loop_go2
echo "build gsc.db $(date +%H:%M)"
$PY -c "import sys, bench; from paths import OUT; t, p, _ = bench.load(OUT / f\"{sys.argv[1]}.src.db\"); bench.write_gsc_input(t, p, OUT / f\"{sys.argv[1]}.src.db\", OUT / f\"{sys.argv[1]}.gsc.db\")" $n 2>&1 | tail -5
ls -la ~/eval_live/out/$n.gsc.db; df -h / | tail -1
echo "pgo-eval $(date +%H:%M)"
~/eval_live/target/release/pgo-eval --db ~/eval_live/out/$n.gsc.db --odom-stream bench_odom --lidar-stream bench_lidar --output /tmp/hlg2_test.json --config ~/eval_live/out/gsc_live.config.json 2>&1 | grep -v gnc-solve | tail -20
echo "exit=${PIPESTATUS[0]} $(date +%H:%M)"
echo DIAGDONE

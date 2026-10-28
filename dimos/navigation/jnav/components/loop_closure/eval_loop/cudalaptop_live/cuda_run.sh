#!/usr/bin/env bash
# on CudaLaptop: refresh raw tag spread from the new markers, then replay every recording live through both PGOs
cd ~/eval_live/dimos/dimos/navigation/jnav/components/loop_closure/eval_loop
export EVAL_LOOP_OUT=~/eval_live/out PGO_EVAL=~/eval_live/target/release/pgo-eval PYTHONPATH=~/eval_live/dimos
PY=~/repos/dimos/.venv/bin/python
NAMES="mid360_athens_stairs hk_village3 hk_village5 hk_village2 hk_village1 hk_village6 hk_village4 go2_hongkong_office small_loop_go2 roscon office555 huge_loop_pointlio huge_loop_fastlio huge_loop_go2_pointlio huge_loop_go2_fastlio huge_loop_go2"
$PY - $NAMES <<'PY'
import json, sys
import bench
from paths import OUT
for name in sys.argv[1:]:
    path = OUT / f"{name}.summary.json"
    row = json.loads(path.read_text())
    markers, note = bench.markers_for(name, None, None) if (OUT / f"{name}.markers.json").exists() else (None, row.get("tag_note"))
    row["tag_spread_raw_m"] = bench.tag_spread(markers, None) if markers else None
    row["tag_tracks"] = len(markers) if markers else None
    row["tag_note"] = note
    path.write_text(json.dumps(row, indent=2))
    print(name, "raw tags", row["tag_spread_raw_m"], flush=True)
PY
for n in $NAMES; do
  echo "== $n $(date +%H:%M)"
  made=""
  if [ ! -s ~/eval_live/out/$n.gsc.db ]; then
    # disk is tight here: build the huge loops' gsc input just before their run, and drop it after
    $PY -c "import sys, bench; from paths import OUT; t, p, _ = bench.load(OUT / f'{sys.argv[1]}.src.db'); bench.write_gsc_input(t, p, OUT / f'{sys.argv[1]}.src.db', OUT / f'{sys.argv[1]}.gsc.db')" $n && made=1
  fi
  $PY variant.py gsc_live '{}' $n 2>&1 | grep -E 'gsc_live|Error|Traceback' | tail -3
  $PY ivan_live.py $n 2>&1 | grep -E 'ivan_live|Error|Traceback' | tail -3
  [ -n "$made" ] && rm -f ~/eval_live/out/$n.gsc.db ~/eval_live/out/$n.gsc.db-shm ~/eval_live/out/$n.gsc.db-wal
done
$PY run.py --tables
echo RUNDONE

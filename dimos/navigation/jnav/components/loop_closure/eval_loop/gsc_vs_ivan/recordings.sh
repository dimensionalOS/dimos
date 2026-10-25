# Sourced by gsc_run / gsc_eval: per-recording db, pgo-eval args, ground truth, video step.
WORKTREE=${GSC_WORKTREE:-$HOME/repos/_mls_reform_wt}
PGO_EVAL=${PGO_EVAL:-$HOME/.cache/gsc_pgo_target/release/pgo-eval}
PY="env -u PYTHONPATH $WORKTREE/.venv/bin/python"
BODY=0,-0.374607,0,0.927184
SMALL_DB=$HOME/.cache/dimos/eval_loop/cd84b786a09c41c8a5435a02534a4355.db
HUGE_DB=$HOME/.cache/dimos/eval_loop/dbf5283feeb446d49f065a100985a78e.db
GIR2_DB=$HOME/datasets/mid360_recordings/2026-06-02_10-03pm-PST__gir_stairs2/mem2.db
MID=(--lidar-tf 0,0,0,0,0,0,1 --body-tf 0,0,0,$BODY)
world_db=
case $recording in
    hk[1-6]) db=$HOME/.dimos/recordings/hk_village_tf/hk_village${recording#hk}.db; eval_args=(--odom-stream go2_odometry --lidar-stream l1_cloud); gt=(--gt-db $db --gt-stream go2_odometry_corrected); step=2; world_db=$db ;;
    small_l1) db=$SMALL_DB; eval_args=(--odom-stream go2_odometry --lidar-stream l1_cloud); gt=(); step=2; world_db=$SMALL_DB ;;
    huge_l1) db=$HUGE_DB; eval_args=(--odom-stream go2_odometry --lidar-stream l1_cloud); gt=(); step=5; world_db=$HUGE_DB ;;
    small_go2) db=$SMALL_DB; eval_args=(--odom-stream odom --lidar-stream lidar); gt=(); step=2; world_db=$SMALL_DB ;;
    small_mid) db=$SMALL_DB; eval_args=(--odom-stream fastlio_odometry --lidar-stream fastlio_lidar "${MID[@]}"); gt=(--gt-db $SMALL_DB --gt-stream fastlio_odometry_corrected); step=2 ;;
    gir2) db=$GIR2_DB; eval_args=(--odom-stream pointlio_odometry --lidar-stream pointlio_lidar "${MID[@]}"); gt=(--gt-db $GIR2_DB --gt-stream odometry_corrected); step=3 ;;
    huge) db=$HUGE_DB; eval_args=(--odom-stream fastlio_odometry --lidar-stream fastlio_lidar "${MID[@]}"); gt=(--gt-db $HUGE_DB --gt-stream fastlio_odometry_corrected); step=5 ;;
    gir1) db=$(ls -d "$HOME"/datasets/mid360_recordings/*__gir_stairs1/mem2.db); eval_args=(--odom-stream pointlio_odometry --lidar-stream pointlio_lidar "${MID[@]}"); gt=(--gt-db $db --gt-stream pointlio_odometry_corrected); step=3 ;;
    china) db=$(ls -d "$HOME"/datasets/mid360_recordings/*__china_office1/mem2.db); eval_args=(--odom-stream pointlio_odometry --lidar-stream pointlio_lidar "${MID[@]}"); gt=(); step=3 ;;  # its fastlio diverged (77x110 m vs 35x58 m)
    grassy|park1|park1_2)
        db=$(ls -d "$HOME"/datasets/mid360_recordings/*__$(case $recording in grassy) echo grassy_field;; gir1) echo gir_stairs1;; park1) echo gir_park1;; park1_2) echo gir_park1_2;; china) echo china_office1;; esac)/mem2.db)
        eval_args=(--odom-stream fastlio_odometry --lidar-stream fastlio_lidar "${MID[@]}"); gt=(); step=3 ;;
    athens) db=$HOME/repos/dimos/data/mid360_athens_stairs.db; eval_args=(--odom-stream pointlio_odometry --lidar-stream pointlio_lidar --lidar-tf 0,0,0,0,0,0,1 --body-tf 0,0,0,0,0,0,1); gt=(); step=3; world_body=0,0,0,1 ;;  # handheld, level Mid-360
    office555) db=$HOME/.cache/dimos/eval_loop/office_555_loop1.db; eval_args=(--odom-stream pointlio_odometry --lidar-stream pointlio_lidar --lidar-tf 0,0,0,0,0,0,1 --body-tf 0,0,0,0,0,0,1); gt=(); step=2; world_body=0,0,0,1 ;;  # handheld rig (mcap via dtk data to_db), level
    *) echo "unknown recording $recording"; exit 1 ;;
esac

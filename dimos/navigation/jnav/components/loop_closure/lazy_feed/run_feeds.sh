#!/usr/bin/env bash
# Build lazy-feed and record the planner inputs for the huge loop (FAST-LIO Mid-360, then Go2 L1), one at a time.
# usage: run_feeds.sh <out dir with tracer_config.json, gsc_fastlio.json, gsc_l1.json> [map ...]
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
out="$1"
shift
maps=("$@")
[ ${#maps[@]} -eq 0 ] && maps=(huge_fastlio huge_l1)
target="${CARGO_TARGET_DIR:-$HOME/.cache/lazy_feed_target}"
(cd "$here" && CARGO_TARGET_DIR="$target" cargo build --release)
huge_db="$HOME/.cache/dimos/eval_loop/dbf5283feeb446d49f065a100985a78e.db"
small_db="$HOME/.cache/dimos/eval_loop/cd84b786a09c41c8a5435a02534a4355.db"
mkdir -p "$out/feed"
for map in "${maps[@]}"; do
    case "$map" in
        huge_fastlio) db=$huge_db odom=fastlio_odometry lidar=fastlio_lidar gsc=gsc_fastlio.json ;;
        huge_l1) db=$huge_db odom=go2_odometry lidar=l1_cloud gsc=gsc_l1.json ;;
        small_l1) db=$small_db odom=go2_odometry lidar=l1_cloud gsc=gsc_l1.json ;;
        *) echo "unknown map $map" >&2; exit 1 ;;
    esac
    LAZY_FEED_GIT_HEAD="$(git -C "$here" rev-parse HEAD)" nice "$target/release/lazy-feed" \
        --db "$db" --odom-stream "$odom" --lidar-stream "$lidar" \
        --gsc-config "$out/$gsc" --tracer-config "$out/tracer_config.json" \
        --output "$out/feed/$map.zst" >"$out/feed/$map.json" 2>"$out/feed/$map.log" &
    pid=$!
    caffeinate -i -w "$pid" &
    wait "$pid"
    cat "$out/feed/$map.json"
done

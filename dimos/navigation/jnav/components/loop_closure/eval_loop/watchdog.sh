#!/bin/bash
# kill $1 if its RSS passes $2 GB or the system's free+inactive memory drops under 12 GB (no swap on this mac)
pid=$1; limit_kb=$(( $2 * 1024 * 1024 ))
while kill -0 $pid 2>/dev/null; do
    rss=$(ps -o rss= -p $pid | tr -d ' ')
    pages=$(vm_stat | awk '/Pages free|Pages inactive|Pages speculative/ {gsub("\\.","",$NF); s+=$NF} END {print s}')
    avail_gb=$(( pages * 16384 / 1024 / 1024 / 1024 ))
    echo "$(date +%T) rss=$((rss/1024))MB avail=${avail_gb}GB"
    if [ "${rss:-0}" -gt "$limit_kb" ] || [ "$avail_gb" -lt 12 ]; then
        echo "WATCHDOG KILL pid $pid rss=$((rss/1024))MB avail=${avail_gb}GB"; kill $pid; sleep 3; kill -9 $pid 2>/dev/null; exit 1
    fi
    sleep 5
done

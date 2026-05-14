#!/usr/bin/env bash
# Simple profiler: one-line-per-sample, appends to terminal (no clearing).
# Usage: bash scripts/profile-live.sh [interval_seconds]

INTERVAL=${1:-3}

printf "%-6s  %8s %8s  %8s %8s  %8s %8s  %8s %8s  %s\n" \
  "TIME" "PY_CPU%" "PY_MB" "DENO_CPU%" "DENO_MB" "SIM_UI%" "SIM_UI_MB" "GPU_CPU%" "GPU_MB" "LOAD"
printf "%-6s  %8s %8s  %8s %8s  %8s %8s  %8s %8s  %s\n" \
  "------" "--------" "--------" "--------" "--------" "--------" "--------" "--------" "--------" "--------"

while true; do
  py_cpu=0; py_mem=0
  while IFS= read -r line; do
    c=$(echo "$line" | awk '{print $1}'); m=$(echo "$line" | awk '{print $2}')
    py_cpu=$(awk "BEGIN{print $py_cpu + $c}")
    py_mem=$(awk "BEGIN{print $py_mem + $m}")
  done < <(ps -eo %cpu,rss,command | grep -i '[p]ython.*dimos' 2>/dev/null)

  deno_cpu=0; deno_mem=0
  while IFS= read -r line; do
    c=$(echo "$line" | awk '{print $1}'); m=$(echo "$line" | awk '{print $2}')
    deno_cpu=$(awk "BEGIN{print $deno_cpu + $c}")
    deno_mem=$(awk "BEGIN{print $deno_mem + $m}")
  done < <(ps -eo %cpu,rss,command | grep -E '[d]eno|[d]imsim' 2>/dev/null | grep -v grep)

  # Find browser PIDs connected to bridge port 8090 (exclude deno server itself)
  chrome_cpu=0; chrome_mem=0
  while IFS= read -r pid; do
    [ -z "$pid" ] && continue
    while IFS= read -r line; do
      c=$(echo "$line" | awk '{print $1}'); m=$(echo "$line" | awk '{print $2}')
      chrome_cpu=$(awk "BEGIN{print $chrome_cpu + $c}")
      chrome_mem=$(awk "BEGIN{print $chrome_mem + $m}")
    done < <(ps -p "$pid" -o %cpu,rss 2>/dev/null | tail -n +2)
  done < <(lsof -i :8090 2>/dev/null | grep -v 'deno\|LISTEN' | awk 'NR>1{print $2}' | sort -u)

  # GPU processes — macOS Metal/WindowServer GPU usage
  gpu_cpu=0; gpu_mem=0
  while IFS= read -r line; do
    c=$(echo "$line" | awk '{print $1}'); m=$(echo "$line" | awk '{print $2}')
    gpu_cpu=$(awk "BEGIN{print $gpu_cpu + $c}")
    gpu_mem=$(awk "BEGIN{print $gpu_mem + $m}")
  done < <(ps -eo %cpu,rss,command | grep -E '[W]indowServer|[G]PU.*Driver|com\.apple\.gpu' 2>/dev/null)

  py_mb=$(awk "BEGIN{printf \"%.0f\", $py_mem/1024}")
  deno_mb=$(awk "BEGIN{printf \"%.0f\", $deno_mem/1024}")
  chrome_mb=$(awk "BEGIN{printf \"%.0f\", $chrome_mem/1024}")
  gpu_mb=$(awk "BEGIN{printf \"%.0f\", $gpu_mem/1024}")
  load=$(sysctl -n vm.loadavg 2>/dev/null | awk '{print $2}' || echo "?")

  printf "%-6s  %8.1f %6sMB  %8.1f %6sMB  %8.1f %6sMB  %8.1f %6sMB  %s\n" \
    "$(date '+%H:%M:%S')" "$py_cpu" "$py_mb" "$deno_cpu" "$deno_mb" "$chrome_cpu" "$chrome_mb" "$gpu_cpu" "$gpu_mb" "$load"

  sleep "$INTERVAL"
done

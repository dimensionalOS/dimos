#!/bin/bash
# Launch m20-nav container with memory monitoring
# Logs system memory state before, during, and after container launch
# to help diagnose OOM issues on the resource-constrained NOS

set -euo pipefail

LOGFILE="/tmp/m20-nav-memlog-$(date +%Y%m%d-%H%M%S).log"
CONTAINER_NAME="m20-nav"
IMAGE="ghcr.io/aphexcx/m20-nav:latest"

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOGFILE"
}

log_mem() {
    log "=== MEMORY SNAPSHOT ==="
    free -h 2>&1 | tee -a "$LOGFILE"
    echo "" >> "$LOGFILE"

    log "=== TOP 10 MEMORY CONSUMERS ==="
    ps aux --sort=-%mem | head -11 2>&1 | tee -a "$LOGFILE"
    echo "" >> "$LOGFILE"

    log "=== DOCKER MEMORY USAGE ==="
    docker stats --no-stream --format "table {{.Name}}\t{{.MemUsage}}\t{{.MemPerc}}" 2>&1 | tee -a "$LOGFILE"
    echo "" >> "$LOGFILE"

    log "=== OOM KILLER STATUS ==="
    dmesg | grep -i "oom\|out of memory\|killed process" | tail -5 2>&1 | tee -a "$LOGFILE" || true
    echo "" >> "$LOGFILE"

    log "=== SSHD OOM SCORE ADJ ==="
    for pid in $(pgrep sshd 2>/dev/null); do
        echo "  sshd PID $pid: oom_score_adj=$(cat /proc/$pid/oom_score_adj 2>/dev/null || echo 'N/A')" | tee -a "$LOGFILE"
    done
    echo "" >> "$LOGFILE"
}

log "Starting m20-nav launch with memory logging"
log "Log file: $LOGFILE"

# Pre-launch snapshot
log ">>> PRE-LAUNCH STATE <<<"
log_mem

# Clean up any existing container
docker rm -f "$CONTAINER_NAME" 2>/dev/null || true

# Launch the container with proper env and memory limit
log ">>> LAUNCHING CONTAINER <<<"
DOCKER_CMD="docker run -d --name $CONTAINER_NAME --network host --memory=1.5g --shm-size=512m -e LOCALIZATION_METHOD=fastlio -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp $IMAGE"
log "Command: $DOCKER_CMD"
CONTAINER_ID=$($DOCKER_CMD)
log "Container started: $CONTAINER_ID"

# Monitor memory for 60 seconds after launch
for i in $(seq 1 12); do
    sleep 5
    log ">>> POST-LAUNCH +${i}x5s <<<"
    log_mem

    # Check if container is still running
    if ! docker ps -q -f "name=$CONTAINER_NAME" | grep -q .; then
        log "!!! CONTAINER DIED !!!"
        log "Exit code: $(docker inspect --format='{{.State.ExitCode}}' "$CONTAINER_NAME" 2>/dev/null || echo 'unknown')"
        log "OOM Killed: $(docker inspect --format='{{.State.OOMKilled}}' "$CONTAINER_NAME" 2>/dev/null || echo 'unknown')"
        docker logs --tail 50 "$CONTAINER_NAME" 2>&1 | tee -a "$LOGFILE"
        break
    fi
done

# Final state
log ">>> FINAL STATE <<<"
log_mem
log "Container status: $(docker inspect --format='{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null || echo 'unknown')"

log "Memory log complete: $LOGFILE"
echo ""
echo "=== Log saved to: $LOGFILE ==="

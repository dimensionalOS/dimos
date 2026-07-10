#!/usr/bin/env bash
# Launch the dimos container ON the robot.
#
#   ./run.sh                                  # default: dimos run r1pro-coordinator
#   ./run.sh dimos run <other-blueprint>      # override the command
#   REPO=/home/agx/dimos ./run.sh             # mount a checkout over the baked
#                                             # code for on-robot development
set -e
IMAGE="${IMAGE:-dimos-r1pro:latest}"
NAME="${NAME:-dimos-r1pro}"

MOUNT_ARGS=()
if [ -n "${REPO:-}" ]; then
    MOUNT_ARGS+=(-v "${REPO}:/app")
fi

docker rm -f "${NAME}" 2>/dev/null || true

# --network host  DDS (to HDAS) + LCM multicast + rerun gRPC all need it
# --ipc host      shared /dev/shm so FastDDS can use SHM with host HDAS
#                 (falls back to UDP loopback if perms differ — also fine)
# --cap-add NET_ADMIN  entrypoint routes the LCM multicast group via lo
docker run -d -t \
    --name "${NAME}" \
    --network host \
    --ipc host \
    --cap-add NET_ADMIN \
    --restart unless-stopped \
    "${MOUNT_ARGS[@]}" \
    "${IMAGE}" \
    "$@"

echo "Started ${NAME}."
echo "  logs:    docker logs -f ${NAME}"
echo "  shell:   docker exec -it ${NAME} bash"
echo "  viewer:  the bridge logs its gRPC URI at startup (docker logs); from the"
echo "           laptop: rerun --connect rerun+http://<robot-ip>:9876/proxy"

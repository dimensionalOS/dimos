#!/usr/bin/env bash
# Launch the dimos container ON the robot.
#
#   ./run.sh                                  # PID 1 idles; exec in and run a
#                                             # blueprint by hand (see below)
#   ./run.sh dimos run <blueprint>            # auto-run a blueprint instead
#   REPO=/home/agx/dimos ./run.sh             # mount a checkout over the baked
#                                             # code for on-robot development
#
# Why idle by default: `dimos run` prompts interactively ("Apply system config
# changes? [y/N]"). A detached PID 1 has no stdin to answer it, so it stalls.
# Idling PID 1 keeps the container up so you `docker exec -it` in — with a TTY
# the prompt works — and gives a single, uncontested coordinator instance (two
# instances collide on the LCM + rerun-gRPC ports → "Address already in use").
set -e
IMAGE="${IMAGE:-dimos-r1pro:latest}"
NAME="${NAME:-dimos-r1pro}"

MOUNT_ARGS=()
if [ -n "${REPO:-}" ]; then
    MOUNT_ARGS+=(-v "${REPO}:/app")
fi

# No command given → idle PID 1 (nothing auto-launches). Any args passed
# through override this and run as the container command.
CMD_ARGS=("$@")
if [ ${#CMD_ARGS[@]} -eq 0 ]; then
    CMD_ARGS=(sleep infinity)
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
    "${CMD_ARGS[@]}"

echo "Started ${NAME} (idle). Run a blueprint manually:"
echo "  docker exec -it ${NAME} bash"
echo "  dimos run r1pro-coordinator        # answer 'y' at the system-config prompt"
echo ""
echo "  logs:    docker logs -f ${NAME}"
echo "  viewer:  the bridge logs its gRPC URI at startup; from the"
echo "           laptop: rerun --connect rerun+http://<robot-ip>:9877/proxy"

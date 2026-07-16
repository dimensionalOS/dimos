#!/bin/bash
# `dimos` on the robot's PATH -> runs inside the dimos-r1lite container.
# Installed to /usr/local/bin/dimos by setup.sh.
#
#   dimos list
#   dimos run r1lite-keyboard-teleop     (needs ssh -X; DISPLAY is forwarded)
#
# The coordinator runs as the always-on compose service; to run it in the
# foreground instead: docker compose -f /opt/dimos/compose.yaml stop dimos
set -e
cd /opt/dimos

DC="docker compose"
docker info >/dev/null 2>&1 || DC="sudo docker compose"

XARGS=()
if [ -n "$DISPLAY" ]; then
    XARGS=(-e DISPLAY="$DISPLAY"
           -v /tmp/.X11-unix:/tmp/.X11-unix
           -v "$HOME/.Xauthority:/root/.Xauthority")
fi

exec $DC run --rm "${XARGS[@]}" dimos "$@"

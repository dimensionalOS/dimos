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

# X11 for pygame teleop (`dimos run r1lite-keyboard-teleop` over ssh -X).
#
# XAUTHORITY is set explicitly rather than relying on $HOME/.Xauthority: the
# container runs as uid 1000 (HOME=/home/dimos) to match the vendor stack, so a
# cookie mounted at /root/.Xauthority is never read and X rejects the
# connection ("wrong authentication" -> SDL's unhelpful "x11 not available").
# Pointing XAUTHORITY at a fixed path makes this independent of which uid runs.
#
# The container inherits the host's hostname via network_mode: host, which
# matters: X cookies are keyed by (hostname, display), so a mismatched hostname
# would fail the same way even with the file in the right place.
XARGS=()
if [ -n "$DISPLAY" ]; then
    XARGS=(-e DISPLAY="$DISPLAY"
           -e XAUTHORITY=/tmp/.docker.xauth
           -v /tmp/.X11-unix:/tmp/.X11-unix
           -v "${XAUTHORITY:-$HOME/.Xauthority}:/tmp/.docker.xauth:ro")
fi

exec $DC run --rm "${XARGS[@]}" dimos "$@"

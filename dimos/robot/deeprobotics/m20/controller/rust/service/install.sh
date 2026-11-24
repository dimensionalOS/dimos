#!/usr/bin/env bash
# install.sh [--allow-gains] (AOS, as root): the m20-controller unit and its launch.json; starts nothing.
# The binary is the dimos_host checkout's (deployed like every native); launch.json comes from that checkout's venv.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
checkout=/var/opt/robot/data/dimos_host
conf=/var/opt/robot/data/m20_controller
install -d -o user -g user "$conf"
sudo -u user "$checkout/.venv/bin/python" -m dimos.robot.deeprobotics.m20.controller.service \
    --policy-dir /var/opt/robot/data/m20_shadow/policies "$@" > "$conf/launch.json.new"
mv "$conf/launch.json.new" "$conf/launch.json"
install -m 644 "$here/m20-controller.service" /etc/systemd/system/m20-controller.service
systemctl daemon-reload
echo "installed m20-controller.service; launch: $(head -c 300 "$conf/launch.json")"

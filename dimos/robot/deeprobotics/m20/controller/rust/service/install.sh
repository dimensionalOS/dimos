#!/usr/bin/env bash
# install.sh [--allow-gains] (AOS, root): the m20-controller unit and launch.json from the dimos_host checkout; starts nothing.
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

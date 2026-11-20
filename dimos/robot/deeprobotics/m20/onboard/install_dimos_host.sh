#!/usr/bin/env bash
# Install (or re-sync) a minimal dimos + `dimos host` service on the M20's AOS and NOS boards.
# Run on GOS, from the dimos checkout the blueprint root runs from: a Host only takes
# fragments built from its own revision (commit + hash of uncommitted changes), so the boards
# get exactly this checkout, uncommitted changes included.
#
#   install_dimos_host.sh [aos|nos ...]     # default: both
#
# Layout: GOS builds the venv once at $dest (the same path the boards use, so the venv's
# absolute paths hold), then rsyncs checkout + venv + uv + uv's Python to each board. The
# boards need no internet (NOS has none) and their system Python 3.8 is not touched.
#
# Routers: every Host is its own zenoh router on the robot LAN (eth 10.21.31.0/24), linked
# AOS <- NOS and both -> GOS:7447 (a Host there, when it runs; zenoh keeps redialing until
# then). None of it goes through the drdds-zenoh-bridge's router (NOS:7447), so dropping the
# bridge changes nothing here; NOS's Host may then move to :7447.
set -euo pipefail

src="${DIMOS_CHECKOUT:-$(git -C "$(dirname "$0")" rev-parse --show-toplevel 2>/dev/null || echo "$HOME/dimos")}"
dest=/var/opt/robot/data/dimos_host
boards=("$@")
[ ${#boards[@]} -gt 0 ] || boards=(aos nos)
sudo_pass="${M20_SUDO_PASS-}"
[ -n "$sudo_pass" ] || sudo_pass="'"

# name ip listen connect tags cpus: NOS's :7447 belongs to the drdds-zenoh-bridge, so its Host is on :7448.
board_config() {
    case "$1" in
        aos) echo "10.21.31.103 tcp/127.0.0.1:7447,tcp/10.21.31.103:7447 tcp/10.21.31.104:7447 board=aos,m20,lidar,imu,ecan,camera 1-3" ;;
        nos) echo "10.21.31.106 tcp/127.0.0.1:7448,tcp/10.21.31.106:7448 tcp/10.21.31.103:7447,tcp/10.21.31.104:7447 board=nos,m20 0-3" ;;
        *) echo "unknown board $1" >&2; return 1 ;;
    esac
}

stage() {
    echo "== staging $(git -C "$src" rev-parse --short HEAD) + uncommitted changes at GOS:$dest"
    sudo install -d -o "$USER" -g "$USER" "$dest"
    cd "$dest"
    [ -d .git ] || git init -q
    # Revisions hashed without --full-index carry short blob ids, so short ids match the root's length.
    git config core.abbrev "$(($(git -C "$src" rev-parse --short HEAD | wc -c) - 1))"
    git -c core.hooksPath=/dev/null fetch -q "$src" HEAD
    # LFS assets stay pointers: dimos fetches the data it needs on first use.
    GIT_LFS_SKIP_SMUDGE=1 git -c advice.detachedHead=false checkout -q --force --detach FETCH_HEAD
    git -C "$src" diff HEAD --binary > "$dest/.gos_uncommitted.patch"
    [ ! -s "$dest/.gos_uncommitted.patch" ] || git apply "$dest/.gos_uncommitted.patch"
    rm -f "$dest/.gos_uncommitted.patch"
    # The base install is all `dimos host` and native-module launching need; no extras, no dev groups.
    UV_CACHE_DIR="$dest/.uv-cache" nice -n 15 "$HOME/.local/bin/uv" sync --frozen --no-dev --no-default-groups --link-mode=copy
    rm -rf "$dest/.uv-cache"
    echo "staged revision: $(cd "$dest" && .venv/bin/python -c 'from dimos.hosted.machine import code_revision; print(code_revision())')"
}

install_board() {
    local name="$1" ip listen connect tags cpus
    read -r ip listen connect tags cpus < <(board_config "$name")
    echo "== $name ($ip)"
    ssh "$ip" "echo \"$sudo_pass\" | sudo -S -p '' install -d -o \$USER -g \$USER $dest && mkdir -p ~/.local/bin ~/.local/share/uv/python"
    nice rsync -a "$HOME/.local/bin/uv" "$ip:.local/bin/uv"
    nice rsync -a "$HOME/.local/share/uv/python/" "$ip:.local/share/uv/python/"
    nice rsync -a --delete --exclude /logs --exclude /.env "$dest/" "$ip:$dest/"
    ssh "$ip" bash -s -- "$name" "$listen" "$connect" "$tags" "$cpus" "$dest" "$sudo_pass" <<'REMOTE'
set -euo pipefail
name="$1" listen="$2" connect="$3" tags="$4" cpus="$5" dest="$6" sudo_pass="$7"
cd "$dest"
.venv/bin/dimos host config name="$name" tags="$tags" listen="$listen" connect="$connect" autodiscovery=false >/dev/null
unit=$(mktemp)
cat > "$unit" <<UNIT
[Unit]
Description=DimOS Host ($name): zenoh router + fragment supervisor
After=network-online.target
Wants=network-online.target

[Service]
User=$USER
WorkingDirectory=$dest
ExecStart=$dest/.venv/bin/dimos host start --foreground
Environment=PATH=$dest/.venv/bin:/usr/local/bin:/usr/bin:/bin
# Off the vendor's real-time cores (isolcpus); fragments inherit both.
CPUAffinity=$cpus
Nice=10
Restart=always
RestartSec=3
KillSignal=SIGINT
TimeoutStopSec=45

[Install]
WantedBy=multi-user.target
UNIT
echo "$sudo_pass" | sudo -S -p '' install -m 644 "$unit" /etc/systemd/system/dimos-host.service
rm -f "$unit"
echo "$sudo_pass" | sudo -S -p '' systemctl daemon-reload
echo "$sudo_pass" | sudo -S -p '' systemctl enable dimos-host.service >/dev/null 2>&1
# A Host reads its revision once at start, so a re-sync restarts it.
echo "$sudo_pass" | sudo -S -p '' systemctl restart dimos-host.service
systemctl is-active dimos-host.service
REMOTE
}

stage
for board in "${boards[@]}"; do
    install_board "$board"
done

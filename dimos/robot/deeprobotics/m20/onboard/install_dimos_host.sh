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
# Routers: every Host is its own zenoh router. AOS's only link to the robot LAN (10.21.31.0/24) is a
# USB 2.0 ethernet adapter that carries ~5 MB/s, too little for both lidars, so AOS and NOS link over
# the gigabit lidar network (10.21.33.0/24, AOS eth0 / NOS eth0, ~110 MB/s) and NOS links to GOS over
# the robot LAN (~95 MB/s): AOS dials NOS's lidar-network port 7448 (NOS's Host listens there in both
# modes, and the drdds bridge never does), NOS dials GOS:7447 (a Host there, when it runs; zenoh keeps
# redialing), and AOS listens only on the lidar network so nothing routes over its USB link.
# NOS:7447 is the router everything else dials (GOS's ZENOH_CONNECT, AOS's m20-camera). In
# vendor mode the drdds-zenoh-bridge owns it and NOS's Host is dimos-host.service on :7448,
# unlinked from the bridge. In rust mode (bridge stopped) NOS's Host is
# dimos-host-router.service on :7447 and takes over as that router. The two units conflict;
# this script enables the one that fits whether the bridge is running now.
set -euo pipefail

src="${DIMOS_CHECKOUT:-$(git -C "$(dirname "$0")" rev-parse --show-toplevel 2>/dev/null || echo "$HOME/dimos")}"
dest=/var/opt/robot/data/dimos_host
boards=("$@")
[ ${#boards[@]} -gt 0 ] || boards=(aos nos)
sudo_pass="${M20_SUDO_PASS-}"
[ -n "$sudo_pass" ] || sudo_pass="'"

# ssh-ip robot-LAN-listen-ip (- for none) lidar-network-listen connect tags cpus port [rust-mode port]
board_config() {
    case "$1" in
        aos) echo "10.21.31.103 - tcp/10.21.33.103:7447 tcp/10.21.33.106:7448 board=aos,m20,lidar,imu,ecan,camera 1-3 7447" ;;
        nos) echo "10.21.31.106 10.21.31.106 tcp/10.21.33.106:7448 tcp/10.21.31.104:7447 board=nos,m20 0-3 7448 7447" ;;
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
    local name="$1" ip lan_ip lidar_listen connect tags cpus port router_port
    read -r ip lan_ip lidar_listen connect tags cpus port router_port < <(board_config "$name")
    echo "== $name ($ip)"
    ssh "$ip" "printf '%s\n' $(printf '%q' "$sudo_pass") | sudo -S -p '' install -d -o \$USER -g \$USER $dest && mkdir -p ~/.local/bin ~/.local/share/uv/python"
    nice rsync -a "$HOME/.local/bin/uv" "$ip:.local/bin/uv"
    nice rsync -a "$HOME/.local/share/uv/python/" "$ip:.local/share/uv/python/"
    nice rsync -a --delete --exclude /logs --exclude /.env "$dest/" "$ip:$dest/"
    # ssh joins its arguments unquoted, and the sudo password is a quote.
    ssh "$ip" "bash -s -- $(printf '%q ' "$name" "$lan_ip" "$lidar_listen" "$connect" "$tags" "$cpus" "$port" "${router_port:-}" "$dest" "$sudo_pass")" <<'REMOTE'
set -euo pipefail
name="$1" lan_ip="$2" lidar_listen="$3" connect="$4" tags="$5" cpus="$6" port="$7" router_port="$8" dest="$9" sudo_pass="${10}"
as_root() { echo "$sudo_pass" | sudo -S -p '' "$@"; }
cd "$dest"
# Listen is per unit (HOST__LISTEN in its environment beats .env); the rest is the Host's .env.
# gossip off: a Host links only to the routers it is told to dial, never to ones a link advertises
# (NOS would hand AOS GOS's robot-LAN locator, and AOS would link to GOS over its USB adapter).
.venv/bin/dimos host config name="$name" tags="$tags" connect="$connect" listen= autodiscovery=false gossip=false >/dev/null

# Loopback (the Host's own fragments), the robot LAN when the board listens there, and the lidar network.
listen_endpoints() {
    local endpoints="tcp/127.0.0.1:$1"
    [ "$lan_ip" = - ] || endpoints="$endpoints,tcp/$lan_ip:$1"
    echo "$endpoints,$lidar_listen"
}

write_unit() {
    local unit="$1" port="$2" conflicts="$3" role="$4" tmp
    tmp=$(mktemp)
    cat > "$tmp" <<UNIT
[Unit]
Description=DimOS Host $name on :$port ($role): zenoh router + fragment supervisor
After=network-online.target
Wants=network-online.target
${conflicts:+Conflicts=$conflicts}

[Service]
User=$USER
WorkingDirectory=$dest
ExecStart=$dest/.venv/bin/dimos host start --foreground
Environment=PATH=$dest/.venv/bin:/usr/local/bin:/usr/bin:/bin
# Loopback first: the Host's own fragments dial it.
Environment=HOST__LISTEN=$(listen_endpoints "$port")
# Off the vendor's real-time cores (isolcpus); fragments inherit both.
CPUAffinity=$cpus
Nice=10
# The memlock a fragment's system check asks for (it cannot sudo prlimit itself).
LimitMEMLOCK=67108864
# Raw receive-only capture for M20BodyImuSniff (it copies the vendor IMU's DDS samples off loopback); fragments inherit it.
AmbientCapabilities=CAP_NET_RAW
Restart=always
RestartSec=3
KillSignal=SIGINT
TimeoutStopSec=45

[Install]
WantedBy=multi-user.target
UNIT
    as_root install -m 644 "$tmp" "/etc/systemd/system/$unit"
    rm -f "$tmp"
}

if [ -z "$router_port" ]; then
    write_unit dimos-host.service "$port" "" "the board's Host"
    wanted=dimos-host.service
else
    write_unit dimos-host.service "$port" dimos-host-router.service "vendor mode, beside the drdds bridge"
    write_unit dimos-host-router.service "$router_port" dimos-host.service "rust mode, the robot's router"
    wanted=dimos-host.service other=dimos-host-router.service
    if ! systemctl is-active -q drdds-zenoh-bridge.service; then
        wanted=dimos-host-router.service other=dimos-host.service
    fi
    as_root systemctl disable "$other" >/dev/null 2>&1
fi
as_root systemctl daemon-reload
as_root systemctl enable "$wanted" >/dev/null 2>&1
# A Host reads its revision once at start, so a re-sync restarts it.
as_root systemctl restart "$wanted"
echo "$wanted: $(systemctl is-active "$wanted")"
REMOTE
}

stage
for board in "${boards[@]}"; do
    install_board "$board"
done

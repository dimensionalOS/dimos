#!/usr/bin/env bash
# One-tap Go2 cockpit for a SteamOS handheld. No terminal, no keyboard.
# usage: go2-app.sh            find a Go2 on this WiFi, drive it with the sticks, record, offer upload
#        go2-app.sh sim        same UI on a simulated Go2 (no robot needed)
#        go2-app.sh replay     same UI on a recorded dataset
mode="${1:-robot}"
# one instance: a second tap while this runs does nothing
exec 9>~/.go2-app.lock
flock -n 9 || exit 0
exec >> ~/go2-app.log 2>&1
echo "=== $(date) mode=$mode"
APP="Go2 Cockpit"
say() { notify-send -a "$APP" -i input-gaming "$APP" "$1"; echo "$1"; }
in_dimos() { distrobox enter dimos -- bash -c "cd ~/dimos/dimensional-applications && source .venv/bin/activate && $1"; }

# The built-in sticks reach apps only through Steam Input (hid_lenovo_go_s exposes a Valve HID
# device that only Steam reads); the desktop layout was installed by the setup script.
pgrep -x steam >/dev/null || { systemd-run --user --scope --collect --quiet bash -c "steam -silent >/dev/null 2>&1 &"; sleep 8; }

# one session at a time
in_dimos "dimos stop" >/dev/null 2>&1
for _ in $(seq 1 15); do curl -fs http://127.0.0.1:7780/api/info >/dev/null 2>&1 || break; sleep 1; done

case "$mode" in
  sim)    args="--simulation mujoco run unitree-go2-gamepad-cockpit" ;;
  replay) args="--replay --replay-loop run unitree-go2-gamepad-cockpit" ;;
  *)
    say "Looking for a Go2 on this WiFi..."
    found=$(in_dimos "python -c 'from dimos.robot.unitree.go2.cli.landiscovery import discover; d=discover(timeout=6.0); print(d[0].ip, d[0].serial) if d else print()'" 2>/dev/null | tail -1)
    ip=${found%% *}; serial=${found#* }; [ "$serial" = "$found" ] && serial=""
    echo "discovery: ip=$ip serial=$serial"
    # On the dog's own access point the multicast reply often does not come back, but the
    # WiFi name is the dog's alias (Go2_60968), which the key file also carries.
    ssid=$(nmcli -t -f active,ssid dev wifi 2>/dev/null | awk -F: '$1 == "yes" {print $2; exit}')
    echo "ssid: $ssid"
    [ -z "$ip" ] && ping -c1 -W1 192.168.12.1 >/dev/null 2>&1 && ip=192.168.12.1
    if [ -z "$ip" ]; then
      kdialog --title "$APP" --sorry "No Go2 found on this WiFi.\n\nTurn the dog on, join its WiFi (or the same network), and tap again."
      exit 1
    fi
    # per-robot AES key: ~/.config/dimos/go2-keys has one "SERIAL KEY ALIAS" per line (device-local, never in git)
    keys=~/.config/dimos/go2-keys
    key=""
    [ -n "$serial" ] && key=$(awk -v s="$serial" '$1 == s && $2 != "(empty)" {print $2}' "$keys" 2>/dev/null | head -1)
    # the AP name is the alias plus a suffix, e.g. Go2_60968_83d2f1
    [ -z "$key" ] && [ -n "$ssid" ] && key=$(awk -v a="$ssid" '$2 != "(empty)" && (a == $3 || index(a, $3 "_") == 1) {print $2}' "$keys" 2>/dev/null | head -1)
    if [ -z "$key" ]; then
      kdialog --title "$APP" --sorry "Found a Go2 at $ip (serial ${serial:-unknown}, WiFi ${ssid:-unknown}) but there is no AES key for it on this device.\n\nAdd a line to ~/.config/dimos/go2-keys:  SERIAL  KEY  ALIAS"
      exit 1
    fi
    say "Go2 ${serial:-$ssid} found at $ip. Starting..."
    args="--record run unitree-go2-gamepad-cockpit --robot-ip $ip --unitree-aes-128-key $key"
    ;;
esac

in_dimos "dimos $args --local-relay --open-browser false" &
dimos_pid=$!
until curl -fs http://127.0.0.1:7780/api/info >/dev/null 2>&1; do
  if ! kill -0 $dimos_pid 2>/dev/null; then
    kdialog --title "$APP" --error "Could not start. Details in ~/go2-app.log"
    exit 1
  fi
  sleep 1
done
say "Connected. Press A on the pad to arm, left stick drives, B stops. Close the window when done."

profile=~/.var/app/org.mozilla.firefox/cockpit-profile
mkdir -p "$profile"
[ -f "$profile/user.js" ] || cat > "$profile/user.js" <<'JS'
user_pref("browser.aboutwelcome.enabled", false);
user_pref("browser.shell.checkDefaultBrowser", false);
user_pref("datareporting.policy.dataSubmissionPolicyBypassNotification", true);
user_pref("datareporting.policy.firstRunURL", "");
user_pref("termsofuse.bypassNotification", true);
user_pref("termsofuse.acceptedVersion", 4);
user_pref("termsofuse.acceptedDate", "1759900000000");
user_pref("browser.startup.homepage_override.mstone", "ignore");
user_pref("browser.sessionstore.resume_from_crash", false);
user_pref("browser.startup.page", 0);
user_pref("browser.sessionstore.resume_from_crash", false);
user_pref("browser.tabs.warnOnClose", false);
user_pref("network.captive-portal-service.enabled", false);
user_pref("network.connectivity-service.enabled", false);
JS
grep -q captive-portal "$profile/user.js" || printf 'user_pref("network.captive-portal-service.enabled", false);\nuser_pref("network.connectivity-service.enabled", false);\n' >> "$profile/user.js"
# a normal window, so the close button is the only control the user needs
systemd-inhibit --what=idle:sleep --who=go2-cockpit --why="driving the robot" \
  flatpak run org.mozilla.firefox --new-instance --profile "$profile" http://127.0.0.1:7780/
# window closed: stop the session, then offer the upload
kill $dimos_pid 2>/dev/null
in_dimos "dimos stop" >/dev/null 2>&1
if [ "$mode" = robot ]; then
  # upload now if there is internet, otherwise the go2-upload timer does it later
  ~/go2-upload.sh
fi

#!/usr/bin/env bash
# Upload every finished Go2 recording that has not been uploaded yet. Safe to run any time:
# skips the recording of a running session, skips without internet, server dedups repeats.
# usage: go2-upload.sh [--quiet]     (--quiet: no dialogs, just a notification on success)
exec 9>~/.go2-upload.lock; flock -n 9 || exit 0
quiet=0; [ "$1" = --quiet ] && quiet=1
APP="Go2 Cockpit"
# the systemd timer runs without the desktop's bus and display; notifications need both
export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=/run/user/$(id -u)/bus}"
export DISPLAY="${DISPLAY:-:0}"
in_dimos() { distrobox enter dimos -- bash -c "cd ~/dimos/dimensional-applications && source .venv/bin/activate && $1"; }
pending=$(for d in ~/dimos/recordings/*/; do [ -f "$d/memory.db" ] && [ ! -f "$d/.uploaded" ] && echo "$d"; done)
[ -z "$pending" ] && exit 0
# a session still writing its store is not finished
running=$(in_dimos "dimos status 2>/dev/null" | awk '/Run ID:/ {print $3}')
if ! curl -fs --max-time 5 -o /dev/null https://api.dimensional.org/ 2>/dev/null && ! curl -s --max-time 5 -o /dev/null https://api.dimensional.org/; then
  [ $quiet = 1 ] || kdialog --title "$APP" --msgbox "No internet here. $(echo "$pending" | wc -l) recording(s) are saved and will upload automatically when this device is back online."
  exit 0
fi
ok=0; fail=0
for d in $pending; do
  case "$d" in *"$running"*) [ -n "$running" ] && continue ;; esac
  if out=$(in_dimos "dimos data upload '$d/memory.db'" 2>&1); then
    touch "$d/.uploaded"; ok=$((ok+1)); echo "$(date) uploaded $d: $(echo "$out" | grep -oE "\([0-9a-f]{12}\)|already uploaded" | head -1)" >> ~/go2-upload.log
  else
    fail=$((fail+1)); echo "$(date) FAILED $d: $(echo "$out" | tail -2 | tr '\n' ' ')" >> ~/go2-upload.log
  fi
done
if [ $ok -gt 0 ]; then notify-send -a "$APP" -i cloud-upload "$APP" "Uploaded $ok recording(s) to Dimensional."; fi
if [ $fail -gt 0 ]; then
  [ $quiet = 1 ] && notify-send -a "$APP" -i dialog-warning "$APP" "$fail upload(s) failed, will retry. See ~/go2-upload.log" || kdialog --title "$APP" --error "$fail upload(s) failed; they will be retried automatically. Details in ~/go2-upload.log"
fi

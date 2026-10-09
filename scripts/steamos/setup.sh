#!/usr/bin/env bash
# One-shot DimOS setup for a SteamOS handheld (Steam Deck, Legion Go S): the Go2 cockpit app,
# the upload queue, and everything the built-in sticks need. Run once in Konsole, Desktop Mode,
# after bootstrap.sh. Safe to rerun. Steam must have been signed in once.
#
#   DIMOS_BRANCH=main bash setup.sh        # branch of dimensionalOS/dimos to check out (default main)
#   WITH_SIM=1 bash setup.sh               # also install MuJoCo for the "Go2 Cockpit (sim)" icon
set -euo pipefail
BOX=dimos
BRANCH="${DIMOS_BRANCH:-main}"
HERE="$(cd "$(dirname "$0")" && pwd)"

# 1. Ubuntu container with the DimOS checkout, installed editable into the library venv
distrobox list | grep -q "| *$BOX " || distrobox create -Y -n $BOX -i ubuntu:24.04
distrobox enter $BOX -- bash -ec "
  sudo dpkg --configure -a
  sudo apt-get update
  sudo apt-get install -y git curl mesa-vulkan-drivers libgl1-mesa-dri libxkbcommon-x11-0 libvulkan1 libxcursor1 libxi6 libxrandr2 libxinerama1
  [ -d ~/dimos ] || GIT_LFS_SKIP_SMUDGE=1 git clone --branch '$BRANCH' https://github.com/dimensionalOS/dimos ~/dimos
  cd ~/dimos && GIT_LFS_SKIP_SMUDGE=1 git pull -q --ff-only || true
  bash scripts/install.sh --non-interactive --mode library --no-cuda --no-sysctl --skip-tests --project-dir ~/dimos/dimensional-applications --extras base,unitree,control
  cd ~/dimos/dimensional-applications
  ~/.local/bin/uv pip install --python .venv/bin/python --no-deps -e ~/dimos
  if [ '${WITH_SIM:-0}' = 1 ]; then ~/.local/bin/uv pip install --python .venv/bin/python --torch-backend cpu 'dimos[sim]' 'mujoco==3.10.0'; fi
"

# 2. the app files
S=~/dimos/scripts/steamos
install -m 755 "$S/go2-app.sh" "$S/go2-upload.sh" "$S/go2-upload-ui.sh" ~/
install -m 644 "$S/go2-upload-ui.py" ~/
mkdir -p ~/Desktop ~/.config/systemd/user ~/.config/dimos
install -m 755 "$S"/desktop/*.desktop ~/Desktop/
install -m 644 "$S"/systemd/go2-upload.service "$S"/systemd/go2-upload.timer ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now go2-upload.timer
[ -f ~/.config/dimos/go2-keys ] || { [ -f "$HERE/go2-keys" ] && install -m 600 "$HERE/go2-keys" ~/.config/dimos/go2-keys; } || echo "note: add ~/.config/dimos/go2-keys (see go2-keys.example)"

# 3. root bits: memlock for zenoh's shm pool, the stick unlock helper and its sudo rule
sudo bash -c "
  printf 'deck hard memlock 65536\ndeck soft memlock 65536\n' > /etc/security/limits.d/99-dimos-memlock.conf
  mkdir -p /etc/go2 && install -m 755 '$S/go2-sticks-unlock' /etc/go2/sticks-unlock
  printf 'deck ALL=(root) NOPASSWD: /etc/go2/sticks-unlock\n' > /etc/sudoers.d/zz-go2-cockpit && chmod 440 /etc/sudoers.d/zz-go2-cockpit
"

# 4. browser: SteamOS ships only a stub icon. Firefox needs /run/udev to see gamepads from its sandbox.
flatpak remote-add --user --if-not-exists flathub https://dl.flathub.org/repo/flathub.flatpakrepo
flatpak install --user -y --noninteractive flathub org.mozilla.firefox
flatpak override --user --filesystem=/run/udev:ro --device=all org.mozilla.firefox

# 5. Steam Input desktop layout. SteamOS's hid_lenovo_go_s driver re-exposes the Legion Go S
# controller as a Valve HID device (28de:12ff) with no evdev gamepad node; only Steam reads it and
# emits the virtual X-Box 360 pad. The stock desktop layout for it is empty (controller_base/empty.vdf),
# so no app sees the sticks until a gamepad layout is selected for app 413080 (the desktop).
B=~/.local/share/Steam/controller_base
for C in ~/.local/share/Steam/steamapps/common/"Steam Controller Configs"/*/config; do
  [ -d "$C" ] || continue
  mkdir -p "$C/413080"
  sed 's/"controller_type"\t\t"controller_neptune"/"controller_type"\t\t"controller_legion_go_s"/; s/"title" "#Title"/"title" "Go2 cockpit gamepad"/' \
    "$B/templates/controller_neptune_gamepad_joystick.vdf" > "$C/413080/controller_legion_go_s.vdf"
  for f in "$C"/configset_28de-12ff-*.vdf "$C/configset_controller_legion_go_s.vdf"; do
    [ -e "$f" ] || continue
    cp "$C/413080/controller_legion_go_s.vdf" "$C/413080/$(basename "$f" .vdf | sed 's/^configset_//').vdf"
    printf '"controller_config"\n{\n\t"413080"\n\t{\n\t\t"autosave"\t\t"1"\n\t}\n}\n' > "$f"
  done
done
steam -shutdown >/dev/null 2>&1 || true

# 6. sign the device in for uploads (device code, needs a browser on any machine)
distrobox enter $BOX -- bash -c "cd ~/dimos/dimensional-applications && .venv/bin/dimos whoami >/dev/null 2>&1" || echo "note: run  distrobox enter dimos -- bash -c 'cd ~/dimos/dimensional-applications && .venv/bin/dimos login'  to enable uploads"

echo "done: reboot into Desktop Mode, then tap Go2 Cockpit."

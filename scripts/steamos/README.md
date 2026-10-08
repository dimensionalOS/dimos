# Go2 cockpit on a SteamOS handheld

Turns a stock SteamOS device (Steam Deck, Legion Go S) into a one-tap Go2 driving and recording
station: find the dog on the WiFi, drive it with the built-in sticks, record every stream, upload
when a network is available. No terminal or keyboard needed after setup.

## Per device, once

1. Bootstrap, with a USB keyboard and a stick holding this directory plus your ssh public key as `key`:
   `bash /run/media/deck/<STICK>/steamos/bootstrap.sh` sets a password, enables sshd, installs the key,
   names the device `deck-<mac>`.
2. `bash ~/dimos/scripts/steamos/setup.sh` (or from the stick), 15 minutes. `WITH_SIM=1` adds MuJoCo.
3. `~/.config/dimos/go2-keys`: the robots' AES keys, see `go2-keys.example`. Not in git, ever.
4. `dimos login` inside the container, device code, enables uploads.

## What's installed

| file | role |
|---|---|
| `go2-app.sh` | the "Go2 Cockpit" icon: Steam up, discover the dog (multicast, else the AP's WiFi name), key by serial or alias, `dimos --record run unitree-go2-gamepad-cockpit --local-relay`, Firefox on the cockpit; closing the window stops and uploads |
| `go2-upload.sh` | the queue: every recording without `.uploaded`, skipped while recording or offline; run on close, by the timer, by the icon |
| `go2-upload-ui.py` / `.sh` | the "Upload recordings" window: list, status, live progress |
| `systemd/go2-upload.timer` | retries pending uploads every 3 minutes |
| `go2-sticks-unlock` | root helper that clears Steam's ACL mask on the joystick nodes |
| `desktop/*.desktop` | the icons |

Runtime: an Ubuntu 24.04 distrobox named `dimos` with the checkout at `~/dimos` installed editable into
`~/dimos/dimensional-applications/.venv`. Recordings land in `~/dimos/recordings/`.

## Why the odd parts

- The built-in sticks reach apps only through Steam Input: `hid_lenovo_go_s` exposes a Valve HID device
  that only Steam reads. Steam must run, with a gamepad desktop layout (setup writes one).
- Firefox is a Flatpak and needs `/run/udev` to enumerate gamepads.
- Logind kills background jobs at ssh logout; long jobs go under `systemd-run --user --scope`.

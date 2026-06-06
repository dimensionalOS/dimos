# Go2 Local Wi-Fi Teleop Troubleshooting

This guide covers local-network setup and troubleshooting for a Unitree Go2 running DimOS over Wi-Fi, including the command center, Rerun dashboard, browser keyboard control, and phone teleop.

Use these checks before sending any movement command. They are written to verify discovery, connectivity, video, and UI routing without making the robot walk.

## Safety First

- Put the robot in a clear area before enabling any control UI.
- Keep a physical stop option available.
- Do not use `dimos topic send /cmd_vel ...` as a connectivity test.
- Do not press movement keys or engage phone teleop until the robot IP, dashboard, and video feed are all verified.
- Stop stale DimOS runs before reconnecting after a Wi-Fi reset:

```bash
dimos status
dimos stop
```

## Quick Recovery Checklist

1. Discover the robot:

```bash
dimos go2tool discover --lan --timeout 10
```

2. Export the discovered address:

```bash
export ROBOT_IP=<discovered_go2_ip>
```

3. Check that the robot is reachable:

```bash
ping -c 3 "$ROBOT_IP"
```

4. Start the Go2 phone teleop stack on all local interfaces:

```bash
dimos --listen-host 0.0.0.0 --robot-ip "$ROBOT_IP" --viewer rerun --rerun-open web \
  run teleop-phone-go2
```

5. Confirm the expected local UI listeners are running:

```bash
lsof -nP \
  -iTCP:7779 -iTCP:8444 -iTCP:9878 -iTCP:9877 -iTCP:3030 \
  -sTCP:LISTEN
```

6. Find the operator machine's current LAN IP:

```bash
# macOS, usually Wi-Fi
ipconfig getifaddr en0

# Linux
hostname -I | awk '{print $1}'
```

7. Open the UI from the operator machine or another device on the same network:

```text
http://<operator_lan_ip>:7779/
http://<operator_lan_ip>:7779/command-center
https://<operator_lan_ip>:8444/teleop
```

The phone teleop page uses HTTPS because mobile browsers require a secure origin for motion sensors. Accept the local self-signed certificate when prompted.

## Discovery And Reconnects

The Go2 may not use the default Unitree IP after it joins a local Wi-Fi network. Prefer `dimos go2tool discover` over hardcoding an address.

Useful commands:

```bash
# BLE and LAN discovery
dimos go2tool discover --timeout 10

# LAN-only discovery
dimos go2tool discover --lan --timeout 10

# Provision Wi-Fi over BLE
dimos go2tool connect-wifi --ssid <wifi_name> --password <wifi_password>
```

`dimos go2tool discover` prints rows with `SOURCE`, `NAME`, `IP`, `MAC`, and `SERIAL`. Use a LAN row's `IP` for `--robot-ip`.

After a Wi-Fi reset, two addresses can change:

- The robot IP. Re-run `dimos go2tool discover` and restart DimOS with the new `--robot-ip`.
- The operator machine IP. Reload browser tabs using the new `http://<operator_lan_ip>:7779/` and `https://<operator_lan_ip>:8444/teleop` URLs.

Stale tabs often look like a broken dashboard even when the new DimOS run is healthy.

## Launching Local Wi-Fi Teleop

For phone teleop to a Go2:

```bash
dimos --listen-host 0.0.0.0 --robot-ip "$ROBOT_IP" --viewer rerun --rerun-open web \
  run teleop-phone-go2
```

For the standard Go2 navigation stack:

```bash
dimos --listen-host 0.0.0.0 --robot-ip "$ROBOT_IP" --viewer rerun --rerun-open web \
  run unitree-go2
```

`--listen-host 0.0.0.0` is important when a phone or another computer needs to load the DimOS web UI from the operator machine. Without it, web servers may bind only to localhost.

Expected local ports:

| Port | Purpose |
|------|---------|
| `7779` | Web dashboard and command center |
| `8444` | Phone teleop HTTPS UI |
| `9878` | Rerun web viewer |
| `9877` | Rerun gRPC proxy |
| `3030` | Rerun keyboard/control WebSocket |

Check listeners:

```bash
lsof -nP \
  -iTCP:7779 -iTCP:8444 -iTCP:9878 -iTCP:9877 -iTCP:3030 \
  -sTCP:LISTEN
```

## Dashboard

Open the dashboard at:

```text
http://<operator_lan_ip>:7779/
```

Open the command center directly at:

```text
http://<operator_lan_ip>:7779/command-center
```

If the dashboard is blank or only partly loaded:

1. Confirm the operator machine IP is current.
2. Confirm port `7779` is listening.
3. Confirm the Rerun web viewer ports `9878` and `9877` are listening.
4. Reload tabs that still point at an old IP or at `localhost` from the wrong device.
5. Check recent logs:

```bash
dimos log -n 100
```

## Video Feed

A healthy Go2 run should publish camera frames on the `color_image` stream, and the Rerun layout should show a camera view.

Safe checks:

```bash
dimos log -n 100
dimos lcmspy
```

In `dimos lcmspy`, look for camera traffic such as `/color_image` or `/color_image#sensor_msgs.Image`. Press `Ctrl-C` after confirming the stream is active.

If the robot is reachable but no video appears:

- Re-check `ROBOT_IP`; the Go2 may have received a new address.
- Restart the DimOS run after a Wi-Fi reset. A stale WebRTC session can leave reachability checks green while camera frames stop arriving.
- Check whether the selected blueprint exposes `color_image` on a transport visible to Rerun. The Rerun bridge renders LCM-visible streams with `.to_rerun` support. Some macOS Go2 configurations use shared memory for high-bandwidth camera frames, which is useful locally but may not appear in the Rerun web dashboard unless the stream is also exposed through an LCM-compatible camera transport.

Relevant code:

- Go2 connection: [`dimos/robot/unitree/go2/connection.py`](https://github.com/dimensionalOS/dimos/blob/main/dimos/robot/unitree/go2/connection.py)
- Go2 basic transports: [`dimos/robot/unitree/go2/blueprints/basic/unitree_go2_basic.py`](https://github.com/dimensionalOS/dimos/blob/main/dimos/robot/unitree/go2/blueprints/basic/unitree_go2_basic.py)
- JPEG-LCM Go2 video helper: [`dimos/robot/unitree/go2/blueprints/smart/_with_jpeg.py`](https://github.com/dimensionalOS/dimos/blob/main/dimos/robot/unitree/go2/blueprints/smart/_with_jpeg.py)
- Rerun visualization: [Viewer Backends](/docs/usage/visualization.md)

## Keyboard Control

Browser keyboard control should be treated as a live motion interface.

Before pressing movement keys:

1. Confirm the camera feed is live.
2. Confirm the dashboard is connected to the current operator machine IP.
3. Confirm the robot is in a clear area.
4. Click the viewer or command center so the browser has keyboard focus.
5. Click the UI control that enables keyboard control.

If "Start Keyboard Control" appears to do nothing:

- Make sure the browser tab has focus. Browsers do not deliver key events to unfocused frames.
- Check that the WebSocket server on port `3030` is listening.
- Check `dimos log -n 100` for viewer connection messages.
- Confirm the keyboard-control stream reaches the Go2 command stream in the selected blueprint. In DimOS navigation stacks, teleop commands usually publish `tele_cmd_vel` and are muxed into `cmd_vel` by the movement manager. In direct teleop stacks, the browser teleop stream must be wired to the Go2 connection's `cmd_vel` input.

Relevant code:

- Phone teleop blueprints: [`dimos/teleop/phone/blueprints.py`](https://github.com/dimensionalOS/dimos/blob/main/dimos/teleop/phone/blueprints.py)
- Rerun keyboard WebSocket server: [`dimos/visualization/rerun/websocket_server.py`](https://github.com/dimensionalOS/dimos/blob/main/dimos/visualization/rerun/websocket_server.py)
- Web command center module: [`dimos/web/websocket_vis/websocket_vis_module.py`](https://github.com/dimensionalOS/dimos/blob/main/dimos/web/websocket_vis/websocket_vis_module.py)
- Navigation command mux: [Navigation Stack](/docs/capabilities/navigation/nav_stack.md)

## Phone Teleop

Open the phone UI from the phone itself:

```text
https://<operator_lan_ip>:8444/teleop
```

The phone must be on the same local network as the operator machine, not just the same internet connection. If the page does not load:

- Confirm DimOS was started with `--listen-host 0.0.0.0`.
- Confirm port `8444` is listening.
- Use the operator machine's LAN IP, not `localhost`.
- Accept the self-signed certificate.
- Allow motion sensor permissions in the browser.

Do not hold the drive/engage control until video and dashboard status are already verified.

## macOS Notes

macOS support is useful for local development but has a few network and transport differences:

- Install the macOS dependencies in [macOS Install](/docs/installation/osx.md), especially `libjpeg-turbo`.
- LCM uses UDP multicast. If DimOS asks to configure routes or system settings, run from an interactive terminal and follow the prompt.
- Camera frames are high bandwidth. Some macOS blueprints prefer shared memory transports for image streams; Rerun web visualization may need a JPEG-LCM-visible camera stream.
- If Wi-Fi reconnects, both the robot IP and the operator machine IP can change. Restart DimOS and reload browser tabs with the new addresses.

## Symptom Reference

| Symptom | No-motion check | Likely cause | Action |
|---------|-----------------|--------------|--------|
| `dimos run` cannot connect | `dimos go2tool discover --lan --timeout 10` | Wrong or stale robot IP | Restart with the discovered `--robot-ip` |
| Dashboard tab stopped after Wi-Fi reset | `ipconfig getifaddr en0` or `hostname -I` | Operator machine IP changed | Reload tabs with the new operator IP |
| Phone cannot load teleop page | `lsof -nP -iTCP:8444 -sTCP:LISTEN` | Server bound to localhost or phone on another network | Start with `--listen-host 0.0.0.0`; verify same Wi-Fi |
| No camera in Rerun | `dimos lcmspy` and `dimos log -n 100` | Stale WebRTC session or camera transport not visible to Rerun | Restart DimOS; use a blueprint/config with LCM-visible camera frames |
| Keyboard button has no effect | `lsof -nP -iTCP:3030 -sTCP:LISTEN` and logs | Browser focus, WebSocket, or blueprint stream wiring issue | Focus the viewer; verify `tele_cmd_vel` reaches `cmd_vel` |
| WebRTC `:8081/offer` error appears in logs | Continue reading logs | Go2 connection fallback may still succeed | Look for connection success and live streams before treating it as fatal |

## When To Open A Bug

Open an issue or pull request if:

- `dimos go2tool discover` finds the robot, but `dimos run ... --robot-ip <ip>` cannot connect.
- A supported blueprint publishes `color_image`, but the Rerun dashboard cannot render it because of transport mismatch.
- Keyboard control connects in the UI but its command stream is not wired to the robot or movement manager.
- Dashboard URLs assume `localhost` when the page is loaded from another device on the LAN.
- Logs report nonfatal WebRTC fallback errors as if the connection failed.

Include:

- Robot model and firmware if known.
- Host OS.
- Exact blueprint and command.
- `dimos go2tool discover --lan --timeout 10` output.
- `dimos status`.
- `dimos log -n 200`.
- Whether `/color_image` appears in `dimos lcmspy`.

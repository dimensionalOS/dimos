# Go2 Wi-Fi Provisioning

Use this runbook when a Go2 needs to move from its AP/BLE setup state onto a
site Wi-Fi network, or when `dimos go2tool connect-wifi` behaves differently on
macOS than it does on Linux.

## Why The Last Run Took Time

The slow part was not the Go2 provisioning protocol. It was macOS Bluetooth
permission handling.

`go2tool` uses Bleak over Bluetooth. On macOS, direct Python execution from a
terminal app such as Terminal or Warp can crash or fail before the Bluetooth
permission flow is usable. A LaunchServices-opened `.app` bundle worked because
macOS TCC could authorize Bluetooth access against the app bundle and its
`Info.plist` usage keys.

If this happens again, treat it as a macOS Bluetooth/TCC issue first, not as a
robot firmware issue.

## Normal Flow

Start with discovery. This shows robots seen over BLE and LAN:

```sh skip
dimos go2tool discover
```

If the robot is still in AP/BLE setup mode, provision it onto the target Wi-Fi.
Do not pass the Wi-Fi password in process args. Omit `--password`; the CLI will
prompt with hidden input.

Public-safe example: move the robot/AP label `dimair09` onto SSID
`dimensional_5G`. The AP label is not necessarily the BLE name, so use
discovery to select the robot interactively:

```sh skip
dimos go2tool connect-wifi --ssid dimensional_5G
```

If more than one robot appears, select the intended robot at the prompt or use
`--name`, `--serial`, or `--mac` from the discovery output.

## macOS Helper Requirements

When direct terminal execution crashes on macOS, run the provisioning helper as
a LaunchServices-opened `.app`, for example with `open`, and include these
Bluetooth usage keys in `Info.plist`:

- `NSBluetoothAlwaysUsageDescription`
- `NSBluetoothPeripheralUsageDescription`
- `NSBluetoothUsageDescription`

Keep secrets out of process args. Pass the SSID and robot selector through
non-secret configuration, then prompt for the Wi-Fi password inside the helper
or read it from a protected secret source.

## Post-Provision Checks

Give the robot time to join the network, then rediscover:

```sh skip
dimos go2tool discover
```

Look for a LAN row with the robot IP. Probe the Go2 WebRTC notification endpoint:

```sh skip
curl -fsS http://<robot-ip>:9991/con_notify
```

Then run DimOS with the discovered IP:

```sh skip
dimos --robot-ip <robot-ip> run unitree-go2
```

For agentic stacks, use the same `--robot-ip` flag with the target blueprint.

## Coding-Agent Safety

Verification is discovery, endpoint probing, and process startup only. Do not
send movement commands, teleop input, MCP skill calls, or `agent-send` messages
during Wi-Fi provisioning verification.

# Go2 wifi provisioning

Use this runbook when a Go2 needs to move from its AP/BLE setup state onto a
site wifi network, or when `dimos go2tool connect-wifi` behaves differently on
macOS than it does on Linux.

## macOS Bluetooth Authorization

`go2tool` uses Bleak over Bluetooth. On macOS, direct Python execution from some
terminal environments can fail before Bluetooth authorization is usable. The
default `auto` BLE backend uses a LaunchServices-opened helper `.app` for finite
scans and wifi provisioning so macOS TCC can authorize Bluetooth access against
the app bundle and its `Info.plist` usage keys.

If direct BLE fails only on macOS, treat Bluetooth/TCC authorization as a likely
cause before assuming a robot firmware issue.

## Normal Flow

Start with discovery. This shows robots seen over BLE and LAN:

```sh skip
dimos go2tool discover
```

If the robot is still in AP/BLE setup mode, provision it onto the target wifi.
Do not pass the wifi password in process args. Omit `--password`; the CLI will
prompt with hidden input.

Example:

```sh skip
dimos go2tool connect-wifi --ssid <wifi>
```

If more than one robot appears, select the intended robot at the prompt or use
`--name`, `--serial`, or `--mac` from the discovery output.

## macOS Helper Requirements

On macOS, `--ble-backend auto` builds and validates a cached helper `.app` when
needed. To use a custom helper bundle, pass `--ble-helper <path>` or set
`DIMOS_GO2_BLE_HELPER`. The helper bundle must include these Bluetooth usage
keys in `Info.plist`:

- `NSBluetoothAlwaysUsageDescription`
- `NSBluetoothPeripheralUsageDescription`
- `NSBluetoothUsageDescription`

The helper passes the wifi password through a mode-`0600` temporary file, not
argv. Keep using the CLI's hidden password prompt unless automation requires a
different secret source.

## Post-Provision Checks

Give the robot time to join the network, then rediscover:

```sh skip
dimos go2tool discover
```

Look for a LAN row with the robot IP. Probe the Go2 WebRTC notification endpoint:

```sh skip
curl -fsS -X POST http://<robot-ip>:9991/con_notify
```

Then run DimOS with the discovered IP:

```sh skip
dimos --robot-ip <robot-ip> run unitree-go2
```

For agentic stacks, use the same `--robot-ip` flag with the target blueprint.

## Coding-Agent Safety

Verification is discovery, endpoint probing, and process startup only. Do not
send movement commands, teleop input, MCP skill calls, or `agent-send` messages
during wifi provisioning verification.

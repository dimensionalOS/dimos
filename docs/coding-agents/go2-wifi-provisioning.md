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

If the robot is still in AP/BLE setup mode, run setup to provision wifi,
rediscover the robot on LAN, and print the IP to use with DimOS. For
interactive use, omit `--password`; the CLI will prompt with hidden input.
For automation, `--password <password>` is supported.

Example:

```sh skip
dimos go2tool setup --ssid <wifi>
```

If more than one robot may be visible, use one selector from the discovery
output so setup does not choose the wrong robot:

```sh skip
dimos go2tool setup --ssid <wifi> --serial <serial>
dimos go2tool setup --ssid <wifi> --name <ble-name>
dimos go2tool setup --ssid <wifi> --mac <ble-address>
```

Use `connect-wifi` instead of `setup` only when you want the lower-level BLE
provisioning step without LAN rediscovery and verification.

## macOS Helper Requirements

On macOS, `--ble-backend auto` builds and validates a cached helper `.app` when
needed. To use a custom helper bundle, pass `--ble-helper <path>` or set
`DIMOS_GO2_BLE_HELPER`. The helper bundle must include these Bluetooth usage
keys in `Info.plist`:

- `NSBluetoothAlwaysUsageDescription`
- `NSBluetoothPeripheralUsageDescription`
- `NSBluetoothUsageDescription`

The helper passes the wifi password through a mode-`0600` temporary file, not
argv. Prefer the CLI's hidden password prompt for interactive use; use
`--password` when a script or test needs a fully non-interactive command.

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

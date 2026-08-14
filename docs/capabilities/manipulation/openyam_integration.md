# OpenYAM integration

OpenYAM uses the generic Damiao whole-body adapter backed by
`can-motor-control`. The adapter owns one upstream robot containing the six-joint
arm and its gripper; motor topology is independent of the host operating system.

## CAN transport

The generic Damiao layer selects the native transport provided by
`can-motor-control>=0.0.6`:

| Host | Default | `--can-port` override |
|------|---------|-----------------------|
| Linux | `SocketCanBus("can0")` | SocketCAN interface, such as `can1` |
| macOS | first `1d50:606f` gs_usb adapter | USB serial number |

The macOS transport is implemented in Rust with native IOKit access. It does
not require PyUSB, libusb, `python-can`, or the Python `gs_usb` package.

Logical buses are ordered. Without explicit selectors, a future two-bus Damiao
robot maps to `can0`/`can1` on Linux and gs_usb indices 0/1 on macOS. Production
multi-arm deployments should configure USB serial numbers because enumeration
order is not a stable device identity.

## Run

Use the default CAN device:

```bash
dimos run coordinator-openyam
```

Select another Linux SocketCAN interface:

```bash
dimos --can-port can1 run coordinator-openyam
```

Select a macOS adapter by USB serial number:

```bash
dimos --can-port <USB-SERIAL> run coordinator-openyam
```

Linux interfaces must already be configured for classical CAN at 1 Mbit/s.
The native macOS transport configures the adapter bitrate directly.

## Hardware topology

| Motor | Send ID | Reply ID | Type |
|-------|---------|----------|------|
| joint1–3 | `0x01`–`0x03` | send ID + `0x10` | DM4340 |
| joint4–6 | `0x04`–`0x06` | send ID + `0x10` | DM4310 |
| gripper | `0x08` | `0x18` | DM4310 |

The bus uses classical CAN at 1 Mbit/s. Opening the gripper decreases motor
position. The arm and gripper are exposed together as one whole-body hardware
component.

## Safety

- Keep the workspace clear and the emergency stop reachable during first
  activation.
- Verify motor IDs and types against the physical unit before commanding large
  motions; an incorrect type scales velocity and torque incorrectly.
- Deactivation removes motor torque, so support the arm before stopping it.

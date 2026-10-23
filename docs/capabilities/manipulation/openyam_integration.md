# OpenYAM integration

OpenYAM uses the generic Damiao whole-body adapter backed by
`can-motor-control`. The adapter owns one upstream robot containing the six-joint
arm and its gripper; motor topology is independent of the host operating system.

## CAN transport

The generic Damiao layer selects the native transport provided by
`can-motor-control>=0.0.8`:

| Host | Default | `runtime_config.bus_devices["openyam"]` override |
|------|---------|-----------------------|
| Linux | `SocketCanBus("can0")` | SocketCAN interface, such as `can1` |
| macOS | first `1d50:606f` gs_usb adapter | USB serial number |

The macOS transport is implemented in Rust with native IOKit access. It does
not require PyUSB, libusb, `python-can`, or the Python `gs_usb` package.

Logical buses are ordered. Without explicit selectors, a future two-bus Damiao
robot maps to `can0`/`can1` on Linux and gs_usb indices 0/1 on macOS. Production
multi-arm deployments should configure USB serial numbers because enumeration
order is not a stable device identity.

List the selectors accepted on the current host:

```bash
dimos hardware can list
```

On Linux this prints SocketCAN network-interface names. Configure each physical
interface before use:

```bash
dimos hardware can setup can0
dimos hardware can setup can1
```

On macOS, connect a `gs_usb`/candleLight-compatible adapter with USB ID
`1d50:606f` for each arm. No SocketCAN interface or extra USB library is needed.
The motor buses still require CAN-H/CAN-L wiring, motor power, and correct bus
termination. To assign sides reliably, connect one adapter at a time, run the
list command, record and label its USB serial, then repeat for the other arm.
Adapters used together must expose unique serial numbers.

## Run

Use the default CAN device:

```bash
dimos run coordinator-openyam
```

Select a Linux SocketCAN interface or a macOS adapter serial in a blueprint of your own with `openyam_hardware(bus_devices={"openyam": "can1"})`. The device lives in the adapter's `runtime_config`, not in `controlcoordinator.addresses`; see [Hardware configuration](/docs/usage/configuration.md#hardware-configuration).

The dual-arm WebXR blueprint is identical on both operating systems; only the
selector values differ:

```bash
# Linux
dimos run teleop-webxr-dual-openyam --left-can-port can0 --right-can-port can1

# macOS
dimos run teleop-webxr-dual-openyam \
  --left-can-port <LEFT-USB-SERIAL> \
  --right-can-port <RIGHT-USB-SERIAL>
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

## Pick-and-place benchmark

The xArm grasp stack on the workcell, both grippers bound to their planning groups:

```bash
dimos run dual-openyam-grasp --left-can-port follower_l --right-can-port follower_r
dimos run dual-openyam-grasp-agent --left-can-port follower_l --right-can-port follower_r
dimos humancli      # second terminal: "scan for a soup can", "pick up the soup can", "put it in the tray"
```

`dual-openyam-grasp-left-agent` and `-right-agent` pin the agent to one arm;
`dual-openyam-grasp-graspgenx` swaps in the learned grasp provider. Every
skill takes a planning group, `left_manipulator` or `right_manipulator`, and
the manipulation module routes gripper commands and readback to that group's
gripper task.
Every run records joint states, the three cameras and TF to sqlite.

What the stack does not have on the workcell, and therefore what the benchmark
measures as is: grasps are one top-down heuristic candidate per object; a pick
executes as soon as it is planned, so the operator keeps a hand on the e-stop;
grasp success is judged from the jaw readback alone. The planner knows the
arms, the wrist cameras, the table, the post and the overhead camera from the
workcell model; the bin is added per scene with the `add_obstacle` RPC. The
place target is a taped tray square on the table on the arm's side (0.26,
-0.20 for the right arm, 0.26, 0.20 for the left, tcp released at z 0.05):
the pick-and-place module places top-down with the grasp orientation and a
pre-place 0.05 above the target, and the arm's fingers-down reach ends near
z 0.16 at the bin's x, so the bin rim cannot be cleared on this stack.

## Model assets

Two dual-arm model packages ship as LFS data:

- `dual_openyam_workcell_v1` is the default, `DUAL_OPENYAM_MODEL`. It describes
  the physical workcell: arm bases 0.43 m apart (tape, 2026-10-08), the table top 42 mm below the motor bases (tape, 2026-10-13) and joint 1
  limits of -225 to +90 deg, inside the stops measured when the motors were
  zeroed with Anvil's `damiao-set-zero-limit` on 2026-10-12. Its zero
  configuration is the folded rest pose on the supports. Every link carries a
  convex hull collision mesh, each gripper carries boxes for its wrist D405 and
  bracket, and a fixed `workcell` link carries the table slab, the camera post
  and the overhead camera, so planners see the rig without extra configuration.
- `dual_openyam_abc_box_v2`, exported as `DUAL_OPENYAM_ABC_MODEL`, keeps the
  0.62 m spacing of the ABC simulation bench for scenes that reproduce it.

Both packages carry the same i2rt arm geometry and meshes; only the base
spacing and the joint 1 limits differ. Each package's `SOURCE.md` records the
measurement or upstream commit every number comes from.

## Safety

- Keep the workspace clear and the emergency stop reachable during first
  activation.
- Verify motor IDs and types against the physical unit before commanding large
  motions; an incorrect type scales velocity and torque incorrectly.
- Deactivation removes motor torque, so support the arm before stopping it.

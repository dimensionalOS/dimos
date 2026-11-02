# Seeed reBot B601-DM

The reBot B601-DM is a six-axis arm with a motorized parallel gripper, driven by
seven Damiao motors. DimOS runs it on the shared Damiao whole-body adapter and
the [`can-motor-control`](https://github.com/dimensionalOS/can-motor-control)
driver, through the Damiao USB-CAN bridge shipped with the arm or any SocketCAN
interface.

## Hardware topology

| Motor | Type | CAN ID | Reply ID |
|---|---|---|---|
| joint1 to joint3 | DM4340P (DM4340 driver profile) | 0x01 to 0x03 | 0x11 to 0x13 |
| joint4 to joint6 | DM4310 | 0x04 to 0x06 | 0x14 to 0x16 |
| gripper | DM4310 | 0x07 | 0x17 |

## Install

Follow the [official installation guide](/docs/installation/index.md) and select
the `manipulation` extra. The B601-DM needs a `can-motor-control` release with
`DmSerialBus` and gripper `calibration_travel`.

The bridge enumerates as a serial port (`/dev/ttyACM*` on Linux, owned by the
`uucp` or `dialout` group; `/dev/cu.usbmodem*` on macOS). On Linux, add your
user to that group and log in again.

## Safety

The arm has no brakes. Stopping DimOS disables the motors, and a raised arm
falls. Activation also leaves the arm motors without torque for about three
seconds while it calibrates the gripper. Start and stop with the arm resting in
its folded pose, clear the workspace and keep the 24 V switch within reach.

## Run

Without a port, the planner runs against mock hardware:

```bash skip
dimos run seeed-b601-dm-planner-coordinator
```

Pass `--can-port` after the blueprint name to drive the real arm. It takes the
bridge's serial port or a SocketCAN interface such as `can0`:

```bash skip
dimos run seeed-b601-dm-planner-coordinator --can-port /dev/ttyACM0
```

Plan, preview and execute arm motions from the Viser page. Open and close the
gripper from [`dimos shell`](/docs/capabilities/manipulation/python_api.md) with
`Arm.from_app(app).set_gripper_position(...)`, from 0.0 (closed) to 1.0 (open).

`coordinator-seeed-b601-dm` runs the same coordinator without the planner.
With `--simulation`, both run mock hardware and ignore `--can-port`.

## Behavior

- **Activation** switches the arm motors to MIT mode, enables them and
  calibrates the gripper by driving it, current-limited, to its closed and open
  stops. The motors return to their stored mode on power-up.
- **Control** sends MIT position targets with Seeed's SDK gains and gravity
  compensation from the vendor URDF. Disable compensation with
  `--gravity-comp false`.
- **Feedback outside the URDF limits** by more than 0.05 rad latches a fault and
  disables the motors. Smaller excursions are clamped.

## Units and model

- Arm joints `joint1` to `joint6` use radians and the vendor URDF joint names.
  joint2's upper limit is widened to 0.02 rad, because the arm's folded rest
  reads just past the URDF's zero.
- The gripper joint `arm/gripper` is the calibrated opening, 0.0 closed and
  1.0 open.
- The planning model fixes both fingers closed, so Viser does not animate the
  gripper.

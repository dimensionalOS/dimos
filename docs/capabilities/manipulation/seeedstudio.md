# Seeed Studio reBot B601-DM

The reBot B601-DM is a six-axis arm with a motorized parallel gripper, driven by
seven Damiao motors. DimOS runs it on the shared Damiao whole-body adapter and
the [`can-motor-control`](https://github.com/dimensionalOS/can-motor-control)
driver, through either the Damiao USB-CAN bridge shipped with the arm or any
SocketCAN interface.

| Motor | Type | CAN ID | Reply ID |
|---|---|---|---|
| joint1 to joint3 | DM4340P (DM4340 driver profile) | 0x01 to 0x03 | 0x11 to 0x13 |
| joint4 to joint6 | DM4310 | 0x04 to 0x06 | 0x14 to 0x16 |
| gripper | DM4310 | 0x07 | 0x17 |

## Install

Follow the [official installation guide](/docs/installation/index.md) and select
the `manipulation` extra. The Damiao USB-CAN bridge needs a `can-motor-control`
release with `DmSerialBus`.

The bridge enumerates as a serial port (`/dev/ttyACM*` on Linux, owned by the
`uucp` or `dialout` group; `/dev/cu.usbmodem*` on macOS). On Linux, add your
user to that group or install a udev rule for USB ID `2e88:4603`.

## Run

The arm has no brakes. Rest it in its folded pose, clear the workspace and keep
the 24 V switch within reach before starting or stopping DimOS: stopping
disables the motors and a raised arm falls.

Without an address, the planner runs against mock hardware:

```bash skip
dimos run seeedstudio-planner-coordinator
```

Give the bridge's serial port (or a SocketCAN interface such as `can0`) as the
address of hardware `arm` to drive the real arm:

```bash skip
dimos run seeedstudio-planner-coordinator --controlcoordinator.addresses '{"arm": "/dev/ttyACM0"}'
```

Plan, preview and execute arm motions from the Viser page. Open and close the
gripper from [`dimos shell`](/docs/capabilities/manipulation/python_api.md) with
`Arm.from_app(app).set_gripper_position(...)`, from 0.0 (closed) to 1.0 (open).

`coordinator-seeedstudio` runs the same coordinator without the planner.

## Behavior

- **Activation** switches the arm motors to MIT mode, enables them and
  calibrates the gripper by driving it, current-limited, to its closed and open
  stops. The motors return to their stored mode on power-up.
- **Control** sends MIT position targets with the gains in
  `dimos/robot/manipulators/seeedstudio/config.py`. Gravity compensation from
  the vendor URDF is off by default; enable it with
  `seeedstudio_hardware(gravity_comp=True)`.
- **Feedback outside the URDF limits** by more than 0.05 rad latches a fault and
  disables the motors. Smaller excursions are clamped, which covers joint2
  resting just past the URDF's zero.

## Units

- Arm joints `joint1` to `joint6` use radians and the vendor URDF joint names.
- The gripper joint `arm/gripper` is the calibrated opening, 0.0 closed and
  1.0 open.
- The planning model fixes both fingers closed, so Viser does not animate the
  gripper.

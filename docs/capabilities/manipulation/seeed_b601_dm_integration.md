# Seeed B601-DM integration

The B601 family has DM (Damiao) and RS (RobStride) versions. This integration
supports B601-DM only. The `seeed_b601_dm` adapter and
`coordinator-seeed-b601-dm` blueprint must not be used for B601-RS.

B601-DM uses the same `DamiaoWholeBodyAdapter` and `can-motor-control`
stack as OpenYAM and OpenArm. The robot adapter declares one six-axis arm
and one gripper on one CAN bus. It adds no motor codec, lifecycle loop,
or vendor SDK dependency.

## Motor profiles

At the driver level **4340 = 4340P**. The B601's first three motors use
`damiao.MotorType.DM4340`. Both names use protocol ranges of +/-12.5 rad,
+/-10 rad/s, and +/-28 Nm. This is a documented profile mapping, not an
assumption that model names must match Python enum names.

| Motors | Send IDs | Feedback IDs | Driver profile |
| --- | --- | --- | --- |
| joint1 through joint3 | 0x01 through 0x03 | 0x11 through 0x13 | DM4340 |
| joint4 through joint6 | 0x04 through 0x06 | 0x14 through 0x16 | DM4310 |
| gripper | 0x07 | 0x17 | DM4310 |

The topology follows Seeed's
[pinned SDK configuration](https://github.com/Seeed-Projects/reBotArm_control_py/blob/d4f3591bc0183de03f3ae8651298db0269aadfdd/config/rebotarm_dm.yaml).
The equivalent 4340 and 4340P codec ranges are also recorded in the
[vendor motor table](https://github.com/motorbridge/motorbridge/blob/d5700bcd5b7f9355dd2012488d463c3bac982259/motor_vendors/damiao/src/motor.rs).
That SDK is reference material only, not a runtime dependency.
Mechanical joint limits come from Seeed's pinned DM robot model.

## Run with mock hardware

Run `dimos run coordinator-seeed-b601-dm` with the `control` extra installed.
Without a CAN selector, the coordinator uses in-memory hardware.
No model checkout or hardware connection occurs at import time.

The trajectory task owns `joint1` through `joint6`. A separate
`b601_dm_gripper` task owns `arm/gripper`. Gripper opening is normalized:
`0` is closed and `1` is open. Arm positions use radians.

## Select physical hardware

Use a shared driver release containing the
[per-motor feedback decoding fix](https://github.com/dimensionalOS/can-motor-control/pull/4)
before physical commissioning. The existing 0.0.8 release decodes every
Damiao model with DM4340 feedback ranges. B601 mixes DM4340 and DM4310,
so that release mis-scales wrist and gripper velocity and torque feedback.
The fix belongs in the shared driver, not in B601-specific rescaling.

Complete Seeed's assembly, motor ID, and zero-position procedure first.
Pass `--can-port` after the blueprint name. It selects a SocketCAN interface
on Linux, or a native gs_usb adapter serial number on macOS, through
`DamiaoRuntimeConfig.bus_devices["b601_dm"]`.
See [CAN setup](/docs/capabilities/manipulation/openyam_integration.md#can-transport)
for the shared transport configuration.

Also supply `--gripper-opening-direction` as `increasing_position` or
`decreasing_position`, and `--gripper-current` as your commissioned per-unit
current in the interval `(0, 1]`. These mechanical settings are required;
the adapter does not copy another arm's opening direction or current.
Do not pass a `/dev/ttyACM*` path as a SocketCAN interface. Supporting a
vendor serial bridge belongs in the shared driver's transport layer.

Selecting physical hardware activates the common Damiao lifecycle.
The upstream gripper performs opening calibration during activation and
can move toward its travel endpoints. Support the arm and clear the
gripper's travel before starting. Deactivation removes motor torque.

The arm model is checked out lazily from the pinned Seeed description.
The finger joints are fixed for six-axis model validation. Gravity
compensation is disabled for initial B601 commissioning. The shared adapter
still validates model order and feedback against angular limits.

Position gains follow Seeed's MIT configuration. The first three derivative
gains are limited to the shared codec's maximum of 5 instead of forwarding
Seeed's value of 8. Planning, WebXR, payload tuning, and physical performance
validation are outside this initial integration.

## Extending support

Follow [the custom-arm guide](/docs/capabilities/manipulation/adding_a_custom_arm.md#choose-the-existing-motor-stack-first).
Resolve a missing SKU name by checking protocol scaling first. Extend actual
missing profiles or transports in `can-motor-control`; keep dimOS adapters
as topology declarations. The B601 adapter inherits connection, activation,
feedback, command dispatch, and faults from the existing Damiao base.

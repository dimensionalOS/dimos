# Seeed Studio reBot B601-DM

The reBot B601-DM is a six-axis arm with a motorized parallel gripper, driven by
seven Damiao motors behind an HDSC USB serial CAN bridge. DimOS talks to the
bridge directly over serial; no vendor SDK is required.

## Install

Follow the [official installation guide](/docs/installation/index.md) and select
the `control` and `manipulation` extras. `control` provides `pyserial` for the
bridge.

## Check the arm

Connect USB and 24 V power, then run the read-only diagnostic. It finds the
bridge by USB identity, reads every motor's parameters and feedback, and never
enables, disables or reconfigures a motor:

```bash
dimos hardware seeedstudio doctor
```

Pass the serial port explicitly if more than one bridge is attached. Every motor
must report POS_VEL mode and a healthy status, and each arm joint must be inside
its limits.

## Run

The arm has no brakes. Rest it in its folded pose, clear the workspace and keep
the 24 V switch within reach before starting or stopping DimOS.

Without a port, the planner runs against mock hardware:

```bash
dimos run seeedstudio-planner-coordinator
```

Select the bridge's serial port to drive the real arm:

```bash
dimos --can-port /dev/cu.usbmodem00000000050C1 run seeedstudio-planner-coordinator
```

Plan, preview and execute arm motions from the Viser page. Open and close the
gripper from [`dimos shell`](/docs/capabilities/manipulation/python_api.md) with
`Arm.from_app(app).set_gripper_position(...)`, from 0.0 (closed) to 1.0 (open).

`coordinator-seeedstudio` runs the same coordinator without the planner.

## Behavior

- **Activation** checks that every motor is in POS_VEL mode, disabled and inside
  its limits, seeds each target at the measured position, then enables and waits
  for all seven motors to confirm.
- **Lost communication** (a motor missing several consecutive replies, a failed
  write, or a closed port) stops commanding. The motors keep holding their last
  target and the adapter refuses further commands until it is deactivated. Only
  the 24 V supply removes holding torque in that state.
- **A motor fault** (a motor reporting an error or dropping out of the enabled
  state) disables every motor, and a raised arm falls.
- **Gripper closing** stops advancing once the motor meets resistance. POS_VEL
  mode has no torque limit of its own; this is not calibrated force control.

The adapter never changes motor mode, gains or zero.

## Units and calibration

Constants live in `dimos/hardware/manipulators/seeedstudio/adapter.py`.

- Arm joints use radians and the vendor URDF limits, except joint2's upper
  limit, which is raised slightly because this arm's joint2 rests just past the
  URDF's zero.
- The gripper joint `arm/gripper` is jaw opening in metres. Its range is the
  vendor URDF's finger travel. The motor travel it maps to is the open angle
  from Seeed's grasp driver, which states a larger maximum opening. Seeed's
  published documentation does not specify the stroke.
- The planning model fixes both fingers closed, so Viser does not animate the
  gripper.

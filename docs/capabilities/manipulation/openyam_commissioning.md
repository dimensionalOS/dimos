---
title: "OpenYAM Direction Commissioning"
---

## Purpose

Before using normal OpenYAM planning or teleoperation on hardware, obtain an
approved vendor/bench-tool direction verification for every planning joint.
This driver cannot perform that verification: its OpenYAM gravity-compensation
path intentionally uses zero position gains and therefore cannot safely issue
position-step commissioning commands. Do not use this procedure to validate
the gripper.

The six planning joints are mapped as follows:

| Hardware joint | Planning joint | Expected positive mapping |
|---|---|---|
| `arm/joint1` | `yam_joint1` | A positive command increases `yam_joint1` position |
| `arm/joint2` | `yam_joint2` | A positive command increases `yam_joint2` position |
| `arm/joint3` | `yam_joint3` | A positive command increases `yam_joint3` position |
| `arm/joint4` | `yam_joint4` | A positive command increases `yam_joint4` position |
| `arm/joint5` | `yam_joint5` | A positive command increases `yam_joint5` position |
| `arm/joint6` | `yam_joint6` | A positive command increases `yam_joint6` position |

The expected result is about the commanded joint only. The physical direction
depends on the joint's zero pose and installation; use the measured encoder
change, not a visual guess about clockwise or counter-clockwise motion, as the
pass criterion.

## Lifecycle and safety behavior

Stopping the driver or handling a fault does **not** automatically park the arm.
Neither event should be documented or relied on as a motion command. On stop or
fault, disable motion output and keep the workspace clear; the operator must
assess the hardware state and use the emergency or manual disable path as
appropriate.

Parking is an explicit operator action that is permitted only while the system
is healthy and the operator has confirmed that a controlled park motion is safe.
Do not attempt to park from a fault handler, during an unhealthy state, or as an
implicit part of shutdown. A no-motion disable is the safe default when motion
must be prevented: it disables commanded motion without issuing a park move.

Gravity-compensation mode also requires a preflight before activation. Verify
the supported OpenYAM inertial model, its required asset, joint state, limits,
and the mechanical support and workspace conditions. If any preflight check is
missing or invalid, do not enable gravity compensation; use the no-motion
disable path and resolve the issue first.

## Safety prerequisites

Complete all of the following before enabling the hardware:

- Remove the payload, tool, and any object held by the gripper.
- Secure the base and support the arm so an unexpected motion cannot cause a
  fall, collision, or pinch.
- Clear the workspace. Keep people, cables, and tools outside the motion
  envelope, and keep an operator at the emergency stop.
- Confirm the emergency stop, motor power cut-off, and manual disable path
  work before the first command.
- Confirm the correct CAN interface, motor IDs, joint limits, and zero/calibration
  state. Stop if any state reading is missing, implausible, or stale.
- Use the lowest approved position gains and speed, and choose a small test
  increment that stays well inside the joint limits. Never test at a limit.

If the arm is not mechanically supported or any prerequisite is uncertain, do
not enable it.

## Gripper status

Physical gripper control and physical gripper position readback are **not
available** in this integration. Do not use, advertise, or validate a gripper
command or readback as a supported capability. Upstream must first release a
calibrated normalized-opening getter; only after that upstream dependency is
released and integrated can gripper support be reconsidered. The direction
commissioning procedure below applies to the six planning joints only, not the
gripper.

## Operator approval gate

Physical OpenYAM startup is default-deny. The operator must explicitly approve
the external direction-verification record and the safety preconditions before
enabling the driver:

```bash
dimos --openyam-operator-approved run openyam-planner-coordinator
```

`--openyam-operator-approved` is a per-run acknowledgement, not a calibration
or a bypass of the gravity-model and state preflights. Without it, the adapter
will not enable motors, including an enable attempted during error recovery.
Mock/simulation hardware is unaffected by this physical approval gate.

## External direction verification precondition

Before starting this driver, have an approved vendor tool or controlled bench
procedure verify `arm/joint1` through `arm/joint6` individually. That external
procedure must use its own documented safe low-speed command path; this driver
must not be used to issue the direction-test steps. Record the measured encoder
sign, joint identity, and any coupled motion. Stop and correct wiring,
calibration, or configuration if a sign is reversed, a joint is swapped, or
unexpected motion occurs.

Only after the approved record is complete may this driver be connected for
gravity-compensated operation. On connection, verify state and limits without
requesting commissioning steps; disable output if any reading is stale or
implausible.

## Record and approve the result

Record one entry per joint in the commissioning log, including:

- date, operator, hardware/firmware identity, and CAN interface;
- initial position and measured direction result from the approved external
  procedure, including the tool used;
- commanded joint, observed `yam_jointN`, and whether any other joint moved;
- pass/fail status, faults or warnings, and the corrective action for failures.

Normal hardware motion is **prohibited until all six directions pass** and the
record is reviewed by the responsible operator. A software test pass is not an
approval to skip this step.

## Software coverage versus hardware validation

The software tests can verify that OpenYAM exposes six joints, that
`arm/joint1` through `arm/joint6` map to `yam_joint1` through `yam_joint6`, and
that mock hardware accepts position updates. These tests do not energize a
motor and cannot detect reversed motor wiring, encoder polarity, swapped CAN
IDs, installation-specific motion, or unexpected mechanical coupling.

Only the approved external procedure above verifies the actual direction
mapping on connected OpenYAM hardware. Treat the software coverage and the
on-hardware commissioning record as separate requirements; both must be
complete before normal planning, execution, or teleoperation.

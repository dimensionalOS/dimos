---
title: "A1Z Robotic Arm"
---

The A1Z keyboard teleop blueprint provides a deterministic simulation scene for
manually testing Cartesian end-effector control and gripper commands.

## Quick Start

Run the A1Z teleop simulation:

```bash
dimos --simulation run keyboard-teleop-a1z
```

The scene contains a table, a cube, and an A1Z arm with a wrist-mounted camera.
The gripper includes explicit visible physical A1Z finger-pad inserts. Each insert has
matching render and collision geometry so the pads shown to the operator are
also the geometry used for manual contact tests. The inserts are simulation
fixtures, not vendor CAD: their shape and placement are deliberately
heuristic, chosen to make the contact surface visible and usable in the
deterministic scene.

The scene is heuristic and deterministic: it is intended to make repeated
manual teleoperation runs visually consistent, not to model a particular
physical setup. The pad fixture must therefore be validated in simulation by
opening and closing the gripper through its full range, checking for
self-collision and unintended table contact, and making a manual cube-contact
check.

Open the visualization URL printed in the terminal to view the robot and scene.

## Keyboard Controls

The Cartesian end-effector controls are:

| Key | Action |
|-----|--------|
| W/S | +X/-X |
| A/D | +Y/-Y |
| Q/E | +Z/-Z |
| R/F | +Roll/-Roll |
| T/G | +Pitch/-Pitch |
| Y/H | +Yaw/-Yaw |
| `[` | Open gripper |
| `]` | Close gripper |
| ESC | Quit |

The gripper keys send raw driver positions: `[` commands `0.015 m` (open),
and `]` commands `0.0 m` (closed). The Cartesian keys publish spatial
end-effector twist commands at `0.05 m/s` linear and `0.5 rad/s` angular speed;
they are integrated and solved by the simulation coordinator.

## Resetting the Scene

Reset is manual. Press `ESC` to quit, then run the command again:

```bash
dimos --simulation run keyboard-teleop-a1z
```

Restarting the simulation recreates the deterministic table, cube, robot, and
wrist-camera scene and resets the simulated robot state. There is no automatic
episode or scene reset during a run.

## Scope and Limitations

This guide covers interactive keyboard teleoperation in the deterministic A1Z
simulation only. It does **not** provide:

- recording or dataset collection;
- episode management or automatic resets; or
- hardware-fidelity claims or real-hardware teleoperation; or
- hardware/data parity for the simulated finger-pad inserts. The inserts do
  not establish the shape, material, compliance, friction, or contact behavior
  of physical A1Z pads, and data collected with this fixture must not be
  treated as hardware-equivalent without separate validation.

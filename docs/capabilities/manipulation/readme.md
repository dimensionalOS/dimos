# Manipulation

Motion planning and teleoperation for robotic manipulators. Uses Drake for physics simulation and Meshcat for 3D visualization.

## Quick Start

### Keyboard Teleop (single command)

Each blueprint launches the full stack — keyboard UI, mock controller, IK solver, and Drake visualization:

```bash
dimos run keyboard-teleop-piper   # Piper 6-DOF
dimos run keyboard-teleop-xarm6   # XArm6 6-DOF
dimos run keyboard-teleop-xarm7   # XArm7 7-DOF
```

Open the Meshcat URL printed in the terminal (default `http://localhost:7000`) to see the robot.

Keyboard controls:

| Key | Action |
|-----|--------|
| W/S | +X/-X (forward/back) |
| A/D | -Y/+Y (left/right) |
| Q/E | +Z/-Z (up/down) |
| R/F | +Roll/-Roll |
| T/G | +Pitch/-Pitch |
| Y/H | +Yaw/-Yaw |
| SPACE | Reset to home pose |
| ESC | Quit |

### Motion Planning (two terminals)

```bash
# Terminal 1: Mock coordinator
dimos run coordinator-mock

# Terminal 2: Planner with Drake visualization
dimos run xarm7-planner-coordinator
```

Then use the IPython client:

```bash
python -m dimos.manipulation.planning.examples.manipulation_client
```

```python
joints()                # Get current joints
plan([0.1] * 7)         # Plan to target
preview()               # Preview in Meshcat
execute()               # Execute via coordinator
```

### Perception + Agent

```bash
# Terminal 1: Coordinator with real xarm7
dimos run coordinator-xarm7

# Terminal 2: Perception + manipulation + LLM agent
dimos run xarm-perception-agent
```

## Architecture

```
KeyboardTeleopModule ──→ ControlCoordinator ──→ ManipulationModule
  (pygame UI)              (100Hz tick loop)      (Drake + Meshcat)
       │                        │                       │
  PoseStamped            CartesianIK task         RRT planner
  commands               (Pinocchio IK)           JacobianIK
                              │                   DrakeWorld
                         JointState ────────────→ (visualization)
```

- **KeyboardTeleopModule** — Pygame UI publishing cartesian pose commands
- **ControlCoordinator** — 100Hz control loop with mock or real hardware adapters
- **ManipulationModule** — Drake physics, Meshcat viz, RRT motion planning, obstacle management

## Blueprints

| Blueprint | Description |
|-----------|-------------|
| `keyboard-teleop-piper` | Piper 6-DOF keyboard teleop with Drake viz |
| `keyboard-teleop-xarm6` | XArm6 6-DOF keyboard teleop with Drake viz |
| `keyboard-teleop-xarm7` | XArm7 7-DOF keyboard teleop with Drake viz |
| `xarm6-planner-only` | XArm6 standalone planner (no coordinator) |
| `xarm7-planner-coordinator` | XArm7 planner with coordinator integration |
| `dual-xarm6-planner` | Dual XArm6 planning |
| `xarm-perception` | XArm7 + RealSense camera for perception |
| `xarm-perception-agent` | XArm7 perception + LLM agent |

## Supported Robots

| Robot | DOF | Teleop | Planning | Perception |
|-------|-----|--------|----------|------------|
| Piper | 6 | Y | Y | — |
| XArm6 | 6 | Y | Y | — |
| XArm7 | 7 | Y | Y | Y |

## Key Files

| File | Description |
|------|-------------|
| [`manipulation_module.py`](/dimos/manipulation/manipulation_module.py) | Main module (RPC interface, state machine) |
| [`manipulation_blueprints.py`](/dimos/manipulation/manipulation_blueprints.py) | Planner and perception blueprints |
| [`robot/manipulators/piper/blueprints.py`](/dimos/robot/manipulators/piper/blueprints.py) | Piper keyboard teleop blueprint |
| [`robot/manipulators/xarm/blueprints.py`](/dimos/robot/manipulators/xarm/blueprints.py) | XArm keyboard teleop blueprints |
| [`teleop/keyboard/keyboard_teleop_module.py`](/dimos/teleop/keyboard/keyboard_teleop_module.py) | Keyboard teleop module |
| [`planning/world/drake_world.py`](/dimos/manipulation/planning/world/drake_world.py) | Drake physics backend |
| [`planning/planners/rrt_planner.py`](/dimos/manipulation/planning/planners/rrt_planner.py) | RRT-Connect motion planner |

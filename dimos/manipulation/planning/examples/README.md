# Manipulation Client

IPython interface for motion planning and execution.

## Quick Start

```bash
# Terminal 1: Coordinator + Planner (combined for Piper)
bash dimos/hardware/manipulators/piper/can_activate.sh  # CAN setup
dimos run piper-manipulation

# Terminal 2: Interactive client
python -m dimos.manipulation.planning.examples.manipulation_client
```

For mock testing (no hardware):
```bash
dimos run coordinator-mock          # Terminal 1
dimos run xarm7-planner-coordinator # Terminal 2
python -m dimos.manipulation.planning.examples.manipulation_client  # Terminal 3
```

## Workflow: Plan → Preview → Execute

```python
# 1. Plan to position (keeps current orientation)
plan_pose(0.3, 0.0, 0.2)

# 2. Preview in Meshcat (optional)
url()       # Get visualization URL
preview()   # Animate the planned path

# 3. Execute on hardware
execute()
```

### Planning Options

```python
# Joint space planning
plan([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

# Cartesian - position only (orientation from current EE pose)
plan_pose(0.3, 0, 0.2)

# Cartesian - full pose
plan_pose(0.3, 0, 0.2, roll=0, pitch=3.14, yaw=0)
```

## Commands

| Command | Description |
|---------|-------------|
| `joints()` | Get current joint positions |
| `ee()` | Get end-effector pose |
| `plan([...])` | Plan to joint config |
| `plan_pose(x, y, z)` | Plan to cartesian pose |
| `preview()` | Preview path in Meshcat |
| `execute()` | Execute trajectory |
| `state()` | Get module state (IDLE/PLANNING/PLANNED/EXECUTING) |
| `reset()` | Reset to IDLE |

### Obstacles

```python
box("table", 0.4, 0, -0.02, 0.8, 0.6, 0.04)  # Add box
sphere("ball", 0.3, 0.1, 0.2, 0.05)          # Add sphere
remove("table")                               # Remove obstacle
```

### Multi-Robot

```python
robots()                           # List robots: ['left_arm', 'right_arm']
plan([...], robot_name='left_arm') # Plan for specific robot
execute('left_arm')                # Execute for specific robot
```

## Notes

- `plan_pose(x, y, z)` fetches current orientation via `ee()` and uses it as target
- States: IDLE → PLANNING → PLANNED → EXECUTING → IDLE
- Use `collision([...])` to check if a config is collision-free before planning

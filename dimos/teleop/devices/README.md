# Teleop Devices

Input device modules that capture tracking data, compute deltas, and transform to robot commands.

## Modules

### BaseTeleopModule

Abstract base class providing:
- Multi-controller calibration (capture initial poses)
- Delta computation (current - initial)
- Transform to robot commands (PoseStamped or Twist based on `output_types`)
- Publishing to `controller_delta_*` outputs
- Optional RPC calls for initial robot pose
- Rerun visualization

### VRTeleopModule

VR-specific implementation that:
- Subscribes to LCM Transform messages from Deno bridge
- Applies coordinate frame transformation (WebXR → Robot)
- Transforms deltas to robot commands based on output type
- Runs control loop at 50Hz

## Data Flow

```
LCM Input (Transform)
       │
       ▼
┌──────────────────┐
│ Coordinate Frame │  WebXR: X=right, Y=up, Z=back
│  Transformation  │  Robot: X=forward, Y=left, Z=up
└──────────────────┘
       │
       ▼
┌──────────────────┐
│   Calibration    │  On X button: capture initial controller pose
│                  │  (optional) RPC call for initial robot pose
└──────────────────┘
       │
       ▼
┌──────────────────┐
│  Delta Compute   │  delta = current_controller - initial_controller
└──────────────────┘
       │
       ▼
┌──────────────────┐
│    Transform     │  PoseStamped: target = initial_robot + delta
│                  │  Twist: velocity = scale(delta)
└──────────────────┘
       │
       ▼
   LCM Output (controller_delta_0/1/2/3)
```

## Configuration

```python
from dimos.teleop.devices import VRTeleopConfig
from dimos.msgs.geometry_msgs import PoseStamped, Twist

config = VRTeleopConfig(
    # Output types determine active indices:
    # PoseStamped → indices 0,1 | Twist → indices 2,3
    output_types=[PoseStamped, Twist],
    input_labels=["left_vr", "right_vr"],

    # Optional RPC methods for initial robot pose (per output)
    robot_pose_rpc_methods=["ArmDriver.get_ee_pose", None],

    # Visualization
    visualize_in_rerun=True,

    # Control
    control_loop_hz=50.0,

    # Transform settings
    linear_scale=1.0,
    angular_scale=1.0,
    max_linear_velocity=0.5,
    max_angular_velocity=1.0,
    gripper_threshold=0.5,
)
```

## Output Indices

The `output_types` parameter auto-computes active indices:

| Configuration | Active Indices | Outputs |
|--------------|----------------|---------|
| `[PoseStamped, PoseStamped]` | `[0, 1]` | Dual arm |
| `[Twist, Twist]` | `[2, 3]` | Dual locomotion |
| `[PoseStamped, Twist]` | `[0, 2]` | Arm + quadruped |

## Adding New Devices

1. Inherit from `BaseTeleopModule`
2. Override `start()` to set up input subscriptions
3. Transform input to 4x4 pose matrix in robot frame
4. Call `compute_deltas(poses, trigger_values)` to get delta poses
5. Use `transform_delta()` from utils to convert to command
6. Call `publish_command(index, command, aux_command)` to publish

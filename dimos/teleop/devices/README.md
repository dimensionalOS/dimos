# Teleop Devices [TO BE UPDATED]

Input device modules that capture tracking data and compute deltas for teleoperation.

## Modules

### BaseTeleopModule

Abstract base class providing:
- Calibration (capture initial poses on X button)
- Delta computation (current - initial)
- Publishing to `controller_delta_*` outputs
- Rerun visualization

### VRTeleopModule

VR-specific implementation that:
- Subscribes to LCM Transform messages from Deno bridge
- Applies coordinate frame transformation (WebXR → Robot)
- Routes deltas through connectors
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
│   Calibration    │  On X button: capture initial pose
└──────────────────┘
       │
       ▼
┌──────────────────┐
│  Delta Compute   │  delta = current - initial
└──────────────────┘
       │
       ▼
┌──────────────────┐
│    Connector     │  Transform to robot command
└──────────────────┘
       │
       ▼
   LCM Output
```

## Configuration

```python
from dimos.teleop.devices import VRTeleopConfig

config = VRTeleopConfig(
    num_inputs=2,                    # Number of controllers
    enable_inputs=[True, True],      # Which inputs to process
    input_labels=["left", "right"],  # Labels for logging/viz
    visualize_in_rerun=True,         # Enable Rerun visualization
    control_loop_hz=50.0,            # Control loop frequency
)
```

## Adding New Devices

1. Inherit from `BaseTeleopModule`
2. Override `start()` to set up input subscriptions
3. Call `_on_lcm_transform(index, pose_matrix)` when new data arrives
4. Use `compute_deltas()` and `publish_command()` from base class

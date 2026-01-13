# Base Teleoperation Module [TODO: Update]

Abstract base class for all teleoperation devices in dimos.

## Overview

`BaseTeleopModule` provides common functionality for teleoperation devices:
- **Calibration**: Capture initial controller poses as reference
- **Delta computation**: Calculate `current - initial` poses
- **Publishing**: Stream delta poses and trigger states to LCM
- **Visualization**: Rerun integration for controller poses
- **RPC interface**: Standard methods across all devices

## Architecture

```
BaseTeleopModule (base class)
├── Calibration logic
├── Delta pose computation
├── LCM publishing (per-controller topics)
├── Rerun visualization
└── update_controller_poses() ← Device-specific code calls this

Device-Specific Module (e.g., VRTeleopModule)
├── Connection logic (WebSocket, USB, etc.)
├── Data parsing
└── Calls: update_controller_poses(controller_poses, controller_gripper_values)
```

## Creating a New Teleoperation Module

1. **Inherit from `BaseTeleopModule`**
2. **Define device-specific config** (inherit from `BaseTeleopConfig`)
3. **Implement connection logic** in `start()` method
4. **Call `update_controller_poses()`** when new tracking data arrives

The module also exposes a blueprint helper as `base_teleop_module`.

### Example: SpaceMouse Device

```python
from dimos.teleop.devices.base import BaseTeleopConfig, BaseTeleopModule

@dataclass
class SpaceMouseTeleopConfig(BaseTeleopConfig):
    usb_device_id: str = "/dev/input/spacemouse"
    # Any additional params

class SpaceMouseTeleopModule(BaseTeleopModule):
    default_config = SpaceMouseTeleopConfig

    @rpc
    def start(self):
        super().start()
        # Start USB reader thread
        self._usb_thread = threading.Thread(target=self._read_usb_loop)
        self._usb_thread.start()

    def _read_usb_loop(self):
        while self._running:
            # Parse USB data to 4x4 transformation matrices
            controller_poses = parse_spacemouse_data(...)

            # Base class handles everything else!
            self.update_controller_poses(
                controller_poses,
                controller_gripper_values=[0.0] * len(controller_poses),
            )
```

## Configuration

### BaseTeleopConfig

```python
@dataclass
class BaseTeleopConfig(ModuleConfig):
    num_inputs: int = 1                      # Number of controllers
    enable_inputs: list[bool] = []           # Defaults to all enabled
    input_labels: list[str] = []             # Defaults to controller_{i}
    position_scale: float = 1.0              # Scale factor for positions
    visualize_in_rerun: bool = True          # Visualize in Rerun
    log_input_data: bool = False             # Log input pose/gripper data
    log_input_data_interval: int = 100       # Log every N publishes when enabled
    safety_limits: bool = True               # Enable safety limits
    max_velocity: float = 0.5                # m/s
    workspace_limits: dict[str, tuple[float, float]]  # x, y, z limits
```

## RPC Methods (Inherited)

All devices get these methods automatically:

- `calibrate()` → Capture initial controller poses
- `reset_calibration()` → Reset calibration state
- `is_calibrated()` → Check if calibrated
- `get_status()` → Get teleoperation status

## LCM Topics (Inherited)

All devices publish these topics:

- `controller_delta_{i}: Out[PoseStamped]` - Controller i delta pose
- `trigger_value_{i}: Out[Float32]` - Controller i trigger/gripper value (0.0-1.0)

## Status Keys

`get_status()` returns a flat dict keyed by `TeleopStatusKey` templates:

- `is_calibrated`
- `controller_{index}_enabled`
- `controller_{index}_has_data`
- `controller_{index}_gripper_value`
- `controller_{index}_label`

## Benefits

- **Reusability**: Write device connection once, inherit everything else
- **Consistency**: Same RPC interface and behavior across all devices
- **Easy testing**: Base class tested independently

## Existing Implementations

- **VRTeleopModule** (`dimos/teleop/devices/vr_headset/`) - VR headset via WebXR (Quest 3 tested)

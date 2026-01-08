# Base Teleoperation Module

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
├── LCM publishing
├── Rerun visualization
└── update_controller_poses() ← Device-specific code calls this

Device-Specific Module (e.g., Quest3TeleopModule)
├── Connection logic (WebSocket, USB, etc.)
├── Data parsing
└── Calls: update_controller_poses(left_pose, right_pose, left_gripper, right_gripper)
```

## Creating a New Teleoperation Module

1. **Inherit from `BaseTeleopModule`**
2. **Define device-specific config** (inherit from `BaseTeleopConfig`)
3. **Implement connection logic** in `start()` method
4. **Call `update_controller_poses()`** when new tracking data arrives

### Example: SpaceMouse Device

```python
from dimos.teleop.base import BaseTeleopConfig, BaseTeleopModule

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
            left_pose, right_pose = parse_spacemouse_data(...)

            # Base class handles everything else!
            self.update_controller_poses(
                left_pose, right_pose,
                left_gripper=0.0, right_gripper=0.0
            )
```

## Configuration

### BaseTeleopConfig

```python
@dataclass
class BaseTeleopConfig(ModuleConfig):
    position_scale: float = 1.0              # Scale factor for positions
    enable_left_arm: bool = True             # Enable left arm
    enable_right_arm: bool = True            # Enable right arm
    visualize_in_rerun: bool = True          # Visualize in Rerun
    safety_limits: bool = True               # Enable safety limits
    max_velocity: float = 0.5                # m/s
    workspace_limits: dict[str, tuple[float, float]]  # x, y, z limits
```

## RPC Methods (Inherited)

All devices get these methods automatically:

- `calibrate_vr()` → Capture initial controller poses
- `reset_calibration()` → Reset calibration state
- `is_vr_calibrated()` → Check if calibrated
- `get_status()` → Get teleoperation status

## LCM Topics (Inherited)

All devices publish these topics:

- `left_controller_delta: Out[PoseStamped]` - Left controller delta pose
- `right_controller_delta: Out[PoseStamped]` - Right controller delta pose
- `left_trigger: Out[Bool]` - Left trigger state
- `right_trigger: Out[Bool]` - Right trigger state

## Benefits

- **Reusability**: Write device connection once, inherit everything else
- **Consistency**: Same RPC interface and behavior across all devices
- **Easy testing**: Base class tested independently

## Existing Implementations

- **Quest3TeleopModule** (`dimos/teleop/quest3/`) - Meta Quest 3 VR headset via WebXR

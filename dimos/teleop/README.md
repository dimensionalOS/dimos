# Teleoperation

VR teleoperation system for controlling robots with Meta Quest controllers.

## Folder Structure

```
teleop/
├── __init__.py              # Exports quest3_teleop blueprint
├── teleop_blueprints.py     # Pre-built system configurations
├── devices/                 # Teleop input modules
│   ├── base_teleop_module.py # BaseTeleopModule (calibration, deltas, transforms)
│   └── vr_teleop_module.py  # VRTeleopModule (LCM inputs)
└── web/                     # Quest WebXR interface
    ├── teleop_server.ts     # Deno WebSocket-LCM bridge
    ├── static/index.html    # WebXR client
    └── certs/               # SSL certificates
```

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Quest 3 Headset                             │
│  ┌─────────────┐                                                    │
│  │  WebXR App  │ ──── Controller poses + triggers ────┐             │
│  └─────────────┘                                      │             │
└───────────────────────────────────────────────────────│─────────────┘
                                                        │ WebSocket
                                                        ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      Deno Bridge (teleop_server.ts)                 │
│                   WebSocket ←→ LCM packet forwarding                │
└───────────────────────────────────────────────────────│─────────────┘
                                                        │ LCM
                                                        ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        VRTeleopModule                               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐          │
│  │  Calibrate   │ →  │ Compute Delta│ →  │  Transform   │          │
│  │  (X button)  │    │ curr - init  │    │  & Publish   │          │
│  └──────────────┘    └──────────────┘    └──────────────┘          │
└───────────────────────────────────────────────────────│─────────────┘
                                                        │
                    ┌───────────────────────────────────┴───────────┐
                    ▼                                               ▼
           controller_delta_0/1                          controller_delta_2/3
           (PoseStamped)                                 (TwistStamped)
           target = init + Δ                             vel = scale(Δ)
```

## Quick Start

```python
from dimos.teleop import quest3_teleop

coordinator = quest3_teleop.build()
coordinator.loop()
```

Then on Quest 3: Open `https://<your-ip>:8443`, press X to calibrate and start.

## Output Types Configuration

The `output_types` parameter determines how each controller input is mapped:

| Output Type | Indices | Use Case |
|-------------|---------|----------|
| `PoseStamped` | 0, 1 | Manipulators, end-effector control |
| `TwistStamped` | 2, 3 | Locomotion, velocity control |

The system auto-computes active indices from output types:
- `[PoseStamped, PoseStamped]` → indices `[0, 1]` (dual arm)
- `[TwistStamped, TwistStamped]` → indices `[2, 3]` (dual locomotion)
- `[PoseStamped, TwistStamped]` → indices `[0, 2]` (arm + quadruped)

## Custom Setup

```python
from dimos.teleop.devices import vr_teleop_module
from dimos.msgs.geometry_msgs import PoseStamped, TwistStamped
from dimos.core.blueprints import autoconnect

# Dual arm setup (both controllers → PoseStamped)
dual_arm = autoconnect(
    vr_teleop_module(
        output_types=[PoseStamped, PoseStamped],
        input_labels=["left_arm", "right_arm"],
    ),
)

# Arm + Quadruped (left → PoseStamped index 0, right → TwistStamped index 2)
arm_and_quad = autoconnect(
    vr_teleop_module(
        output_types=[PoseStamped, TwistStamped],
        input_labels=["left_arm", "right_quad"],
    ),
)

# With RPC for initial robot pose (arm calibration)
arm_with_rpc = autoconnect(
    vr_teleop_module(
        output_types=[PoseStamped, PoseStamped],
        input_labels=["left_arm", "right_arm"],
        robot_pose_rpc_methods=["LeftArm.get_ee_pose", "RightArm.get_ee_pose"],
    ),
)
```

## Submodules

- [devices/](devices/) - Teleop input modules
- [web/](web/) - Quest WebXR client and Deno server

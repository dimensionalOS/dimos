# Teleoperation [TO BE UPDATED]

VR teleoperation system for controlling robots with Meta Quest controllers.

## Folder Structure

```
teleop/
├── __init__.py              # Exports quest3_teleop blueprint
├── teleop_blueprints.py     # Pre-built system configurations
├── connectors/              # Robot command transformers
│   ├── base_connector.py    # BaseTeleopConnector (abstract)
│   ├── arm_connector.py     # ArmConnector → PoseStamped
│   └── quadruped_connector.py # QuadrupedConnector → Twist
├── devices/                 # Teleop input modules
│   ├── base_teleop_module.py # BaseTeleopModule (calibration, deltas)
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
│  │  Calibrate   │ →  │ Compute Delta│ →  │ Route to     │          │
│  │  (X button)  │    │ curr - init  │    │ Connectors   │          │
│  └──────────────┘    └──────────────┘    └──────────────┘          │
└───────────────────────────────────────────────────────│─────────────┘
                                                        │
                    ┌───────────────────────────────────┴───────────┐
                    ▼                                               ▼
        ┌───────────────────┐                         ┌───────────────────┐
        │   ArmConnector    │                         │QuadrupedConnector │
        │ delta → PoseStamped                         │ delta → Twist     │
        │ target = init + Δ │                         │ vel = scale(Δ)    │
        └─────────┬─────────┘                         └─────────┬─────────┘
                  │                                             │
                  ▼                                             ▼
           controller_delta_0/1                          controller_delta_2/3
           (PoseStamped)                                 (Twist)
```

## Quick Start

```python
from dimos.teleop import quest3_teleop

coordinator = quest3_teleop.build()
coordinator.loop()
```

Then on Quest 3: Open `https://<your-ip>:8443`, press X to calibrate and start.

## Modular Connector System

Each VR controller can be assigned any connector type. The connector determines how deltas are transformed into robot commands.

| Connector | Output | Use Case |
|-----------|--------|----------|
| `ArmConnector` | PoseStamped | Manipulators, end-effector control |
| `QuadrupedConnector` | Twist | Locomotion, velocity control |

Output indices determine the message type:
- Index 0, 1 → PoseStamped
- Index 2, 3 → Twist

## Custom Setup

```python
from dimos.teleop.connectors import ArmConnector, ArmConnectorConfig, QuadrupedConnector, QuadrupedConnectorConfig
from dimos.teleop.devices import vr_teleop_module
from dimos.core.blueprints import autoconnect

# Two arms
left = ArmConnector(ArmConnectorConfig(driver_module_name="LeftArm"))
right = ArmConnector(ArmConnectorConfig(driver_module_name="RightArm"))

dual_arm = autoconnect(
    vr_teleop_module(
        num_inputs=2,
        enable_inputs=[True, True],
        connectors=[left, right],
    ),
)

# Or: arm + quadruped (use index 0 and 2)
arm = ArmConnector(ArmConnectorConfig(driver_module_name="Arm"))
quad = QuadrupedConnector(QuadrupedConnectorConfig(driver_module_name="Go2"))

arm_and_quad = autoconnect(
    vr_teleop_module(
        num_inputs=3,
        enable_inputs=[True, False, True],  # index 0 and 2 enabled
        connectors=[arm, None, quad],
    ),
)
```

## Submodules

- [connectors/](connectors/) - Robot command transformers
- [devices/](devices/) - Teleop input modules
- [web/](web/) - Quest WebXR client and Deno server

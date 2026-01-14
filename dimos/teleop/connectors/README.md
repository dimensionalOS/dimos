# Teleop Connectors [TO BE UPDATED]

Connectors transform delta poses from teleop devices into robot-specific commands.

## Available Connectors

### ArmConnector

For manipulators and end-effector control.

- **Input**: Delta pose (Pose)
- **Output**: PoseStamped (target = initial_robot_pose + delta)
- **Aux Output**: Bool (gripper state from trigger)

```python
from dimos.teleop.connectors import ArmConnector, ArmConnectorConfig

connector = ArmConnector(ArmConnectorConfig(
    driver_module_name="MyArmDriver",  # For RPC calibration
    dummy_driver=True,                  # Skip RPC, use zeros
    gripper_threshold=0.5,              # Trigger threshold
))
```

### QuadrupedConnector

For quadrupeds and mobile bases.

- **Input**: Delta pose (Pose)
- **Output**: Twist (velocity = scale(delta))
- **Aux Output**: Bool (trigger state)

```python
from dimos.teleop.connectors import QuadrupedConnector, QuadrupedConnectorConfig

connector = QuadrupedConnector(QuadrupedConnectorConfig(
    linear_scale=1.0,           # Position delta → linear velocity
    angular_scale=1.0,          # Orientation delta → angular velocity
    max_linear_velocity=0.5,    # m/s clamp
    max_angular_velocity=1.0,   # rad/s clamp
))
```

## Creating Custom Connectors

```python
from dimos.teleop.connectors import BaseTeleopConnector, ConnectorConfig
from dimos.msgs.geometry_msgs import Pose

class MyConnector(BaseTeleopConnector):
    def set_initial_pos(self, rpc_result=None) -> bool:
        # Initialize robot state (optional RPC)
        self._has_initial_pos = True
        return True

    def transform_delta(self, delta_pose: Pose, trigger_value: float):
        # Transform delta to your command type
        command = ...
        aux_command = ...
        return command, aux_command
```

## Output Index Mapping

Connectors must be assigned to indices matching their output type:

| Index | Output Type | Connector |
|-------|-------------|-----------|
| 0, 1  | PoseStamped | ArmConnector |
| 2, 3  | Twist       | QuadrupedConnector |

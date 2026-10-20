# Record coordinator position targets

`ControlCoordinator.applied_joint_position_command` publishes `JointState` after
successful adapter writes. It contains POSITION and SERVO_POSITION targets after
task computation, IK, task-side limits, arbitration, routing, and wrapper hold-last
expansion.

`coordinator_joint_state` contains measured feedback. This stream instead contains
the full position vector sent to each successful adapter in that tick. Names
follow `HardwareComponent.joints` order within each hardware block. Hardware blocks follow dispatch order; consumers must select joints by
name rather than assume a fixed whole-robot vector. Grippers retain their adapter
units, as do all other joints. `velocity` and `effort` are empty. Whole-body
entries contain `MotorCommand.q`, without PD gains or feed-forward torque.
Velocity commands and twist bases do not appear in this position stream.

The timestamp is the tick's Unix time in seconds, shared with
`coordinator_joint_state`. It identifies the control snapshot, not the precise
write time. `frame_id` uses `joint_state_frame_id`.

Partial commands include held positions for unspecified joints on that adapter.
Initial held positions come from feedback, as required by the existing wrapper.
Hardware without a write in that tick is absent. Failed, rejected, or unready
hardware is absent, and an empty tick produces no message. If one adapter fails,
the message can still contain the other successful adapters. A failed adapter
may already have sent part of its payload; its boolean return cannot describe
that partial success. Publication failures are logged and do not suppress measured
state, per-robot state, or TF output.

Acceptance means the adapter returned `True`. This is not motor acknowledgment
or proof of motion. The stream records the coordinator-to-adapter payload;
adapter or driver transformations, internal clamping, asynchronous trajectory
execution, and physical tracking remain outside this boundary. Both position
modes share this stream because the existing `JointState` schema has no mode
field. Use a graph with a known position mode for policy collection.

## Record with the existing SQLite recorder

Define this subclass in your application's importable Blueprint module and add
it to the existing mock or simulated robot Blueprint:

```python skip
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In
from dimos.memory.module import Recorder
from dimos.msgs.sensor_msgs.JointState import JointState


class ActionRecorder(Recorder):
    applied_joint_position_command: In[JointState]


recording_blueprint = autoconnect(
    robot_blueprint,
    ActionRecorder.blueprint(
        db_path="actions.db",
        record_tf=False,
        poseless_streams=["applied_joint_position_command"],
    ),
)
```

`robot_blueprint` is your existing Blueprint containing one coordinator.
Autoconnect matches the new port by name and type. The recorder stores it as
`applied_joint_position_command`; normal Blueprint shutdown flushes the database.
Reindex by joint name and check coverage before using these samples as actions.

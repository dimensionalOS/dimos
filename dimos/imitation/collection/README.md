# Collection profiles

A `CollectionProfile` describes **what to record and how to interpret it as a
dataset**. It connects typed robot streams to named observation/action features.
The same declaration supplies recorder inputs and the schema saved beside the
recording. The robot blueprint owns hardware, cameras, transports and lifecycle;
a profile does not construct them or select a policy backend.

## Types and fields

| Type | Responsibility |
| --- | --- |
| `FeatureSpec` | Dataset projection plus an optional live-capture `message_type`. |
| `CollectionProfile` | Robot contract: name, robot_type, observations, actions, sync and quality. |
| `RecordingSchema` | Portable JSON snapshot of the contract, without Python message classes. |

See [profile.py](/dimos/imitation/collection/profile.py),
[recording.py](/dimos/imitation/collection/recording.py), and
[dataprep/schema.py](/dimos/imitation/dataprep/schema.py).

Collection and offline preparation use the same feature class. A live profile
requires `message_type` on every feature; the recorder also requires each class
to be importable at module level and implement the native codec. Offline
features need no message class. `message_type` is omitted from feature JSON and
JSON Schema, and `to_schema()` clears it when saving the portable contract.

## A minimal custom arm

This two-joint example records measured positions as observations and accepted
targets as actions. It runs without hardware. Put the declaration in your robot
package; joint names must match what its producers publish.

```python session=profile no-result
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.dataprep.schema import FeatureSpec
from dimos.imitation.dataprep.core import SyncConfig
from dimos.msgs.sensor_msgs.JointState import JointState

joints = ["arm/joint1", "arm/gripper"]
profile = CollectionProfile(
    name="custom-arm", robot_type="custom-arm-2dof",
    observations={"observation.state": FeatureSpec(
        stream="measured", message_type=JointState, field="position",
        dtype="float32", shape=(2,), names=joints,
    )},
    actions={"action": FeatureSpec(
        stream="accepted", message_type=JointState, field="position",
        dtype="float32", shape=(2,), names=joints,
        source_kind="joint_position_updates",
    )},
    sync=SyncConfig(anchor="observation.state", rate_hz=30, tolerance_ms=20),
)
assert profile.input_types() == {"measured": JointState, "accepted": JointState}
```

The outer keys (`observation.state`, `action`) are **dataset feature names**.
`stream` names (`measured`, `accepted`) identify **recorder input ports** and must
match blueprint outputs or be connected through remappings. `message_type` is
the raw message class; `dtype` and `shape` describe its projected dataset value.

| Field | Meaning |
| --- | --- |
| `name`, `robot_type` | Identities saved in metadata, not blueprint lookup keys. |
| `observations`, `actions` | Nonempty mappings with distinct feature names. |
| `sync` | Observation-feature anchor, dataset rate and snapshot tolerance. |
| `quality` | Validation policy; defaults to strict mode. |
| Feature `field` | Optional message attribute or dictionary key, such as `position`. |
| Feature `dtype`, `shape` | Numerical dtype or `video`, and exact per-frame dimensions. |
| Feature `names` | Ordered vector elements, or image axis labels. |
| Feature `source_kind` | `snapshot` (default) or `joint_position_updates`. |

`shape=(2,)` is one two-element vector, not two independently declared features.
JointState projection follows configured names regardless of incoming order:

```python session=profile no-result
from dimos.imitation.dataprep.core import resolve_field

message = JointState(name=["arm/gripper", "arm/joint1"], position=[0.2, 1.0])
value = resolve_field(message, profile.observations["observation.state"])
assert value.tolist() == [1.0, 0.2]
```

Dataset preparation converts numerical values to the declared dtype and checks
shape, missing joints and finite values. Missing accepted targets are not filled
from measured state or future commands.

### State versus action

- `snapshot` selects a nearby recorded observation within the alignment
  tolerance. Hand-guided teaching can use measured positions for both state
  and action with this source kind.
- `joint_position_updates` reconstructs accepted JointState position targets
  causally. Sparse updates retain omitted joints' last accepted target, including
  history before the episode. Every configured joint needs an accepted target.
  This source kind requires JointState messages and a named position vector.

Several features may project different fields or subsets from one raw stream.
`input_types()` deduplicates recorder inputs, not dataset features. Shared-stream
features must agree on message class and source kind. Measured state and accepted
commands are different streams, even when both use JointState.

### Add a camera

An image feature declares an input; the blueprint still provides its camera.
This adds a 64×64 RGB stream as the timeline anchor:

```python session=profile no-result
from dimos.msgs.sensor_msgs.Image import Image

profile = CollectionProfile(
    name=profile.name, robot_type=profile.robot_type,
    observations={**profile.observations, "observation.images.wrist": FeatureSpec(
        stream="wrist_rgb", message_type=Image, field="data",
        dtype="video", shape=(64, 64, 3), names=["height", "width", "channels"],
    )},
    actions=profile.actions,
    sync=SyncConfig(anchor="observation.images.wrist", rate_hz=30, tolerance_ms=20),
)
assert profile.input_types()["wrist_rgb"] is Image
```

Use the producer's actual resolution. Video input must be a uint8 image; its
names label axes, while vector names label elements. Add more camera features
and matching producers for more cameras; there is no camera-count setting.

## Recorder and saved schema

The factory in [recorder.py](/dimos/imitation/collection/recorder.py) returns an
ordinary blueprint with typed ports declared before `autoconnect`:

```python session=profile no-result
from pathlib import Path
from dimos.imitation.collection.recorder import collection_recorder

recorder = collection_recorder(
    profile=profile, recording=Path("recordings/custom-arm"),
    format="mcap", instance_name="recorder",
)
```

Construction does not start recording. Compose this with your robot producers,
camera and episode monitor in a blueprint. The recorder additionally requires
`status: In[String]` carrying versioned JSON episode events. Port names must be
nonreserved Python identifiers; message classes must be importable at module
level and support native LCM encode/decode. No workflow registration is needed.

At runtime, wiring checks require every declared input. Capture stores native-rate,
unaligned streams in a new directory containing `schema.json` and `recording.mcap`,
or `recording.db` for `format="sqlite"`. Remapped recorded names are reflected in
the saved schema. Copy the complete directory so schema and payload stay together.

`to_schema()` drops `message_type` but preserves dataset interpretation. Export
uses the saved schema, not a later edited robot profile. This round-trip builds
a config without reading a recording or starting the isolated runtime:

```python session=profile no-result
from dimos.imitation.collection.recording import RecordingSchema
from dimos.imitation.dataprep.core import OutputConfig

schema = RecordingSchema.model_validate_json(profile.to_schema().model_dump_json())
config = schema.dataprep_config(
    Path("recordings/custom-arm"),
    OutputConfig(format="lerobot", path=Path("datasets/custom-arm")),
)
assert config.source == "recordings/custom-arm/recording.mcap"
assert config.action["action"].names == joints
assert "message_type" not in schema.model_dump_json()
```

For a real directory, use `RecordingSchema.read(directory)` first, then pass its
config to `run_lerobot_dataprep` in
[formats/lerobot/adapter.py](/dimos/imitation/dataprep/formats/lerobot/adapter.py), or to `run_dataprep`
in [dataprep/build.py](/dimos/imitation/dataprep/build.py) for HDF5.
`inspect_recording(..., config=config)` uses the same interpretation for quality
inspection. These APIs are available at this layer; the later imitation CLI
is not required.

Profile validation checks declarations; wiring checks inputs; preparation checks
actual recorded values. A valid profile alone does not prove recording quality
or checkpoint compatibility.

# Record a collection

`CollectionProfile` declares observations and actions. `collect()` returns a
recorder Blueprint. Your platform Blueprint supplies control, cameras, and episode
controls through ordinary `autoconnect()` and `remappings()`.

## Declare the recorded features

This example declares measured joint state, accepted joint targets, and one RGB
image. Constructing the profile does not connect to hardware.

```python session=profile no-result
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.dataprep.schema import FeatureSpec
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState

joints = ["arm/joint1", "arm/gripper"]
state = FeatureSpec(
    stream="measured", message_type=JointState, field="position",
    dtype="float32", shape=(2,), names=joints,
)
profile = CollectionProfile(
    observations={
        "observation.state": state,
        "observation.images.wrist": FeatureSpec(
            stream="wrist_image", message_type=Image, field="data",
            dtype="video", shape=(480, 640, 3),
            names=["height", "width", "channels"],
        ),
    },
    actions={"action": state.model_copy(update={
        "stream": "accepted", "source_kind": "joint_position_updates",
    }, deep=True)},
    metadata={"name": "custom-arm", "robot_type": "custom-arm-2dof"},
)
assert profile.input_types() == {
    "measured": JointState, "wrist_image": Image, "accepted": JointState,
}
```

Metadata is optional. `name` labels the recording and supplies the default local
dataset name. `robot_type` labels the dataset; it does not load hardware or determine
joint order. Preparation output metadata overrides saved metadata.

Feature keys name configured projections. LeRobot export applies fixed policy-facing
names; HDF5 preserves the configured keys. The saved export sidecar records the
computed mapping. See the [LeRobot export rules](/dimos/imitation/dataprep/README.md#lerobot-export). `stream` identifies a typed recorder input.
`names` specifies joint order even when incoming JointState messages use another
order. Several projections can share a raw stream, which the recorder captures
once. Shared projections must agree on message class and source semantics.

For hand-guided teaching, use the measured state as both observation and action.
For custom action representations, declare additional action features directly.
The collection API has no action-mode enumeration.

## Compose the platform

```python session=profile no-result
from pathlib import Path
from dimos.imitation.collection.recorder import collect

recorder = collect(profile=profile, recording=Path("recordings/session-001"))
```

Include `recorder`, your robot, camera producers, and an
`EpisodeMonitorModule.blueprint(instance_name="episodes")` in `autoconnect()`.
The monitor supplies `status: In[String]` containing versioned JSON episode events.
Robot-specific readiness rules belong on that monitor. The recorder must precede
producers in the Blueprint so it stops after them.

The Python graph supports any number of coordinators. Connect each recorded stream
explicitly when names differ. The recorder requires every declared input and status
before recording. Message classes must be importable at module level and support
native LCM encoding. Missing or ambiguous producers are wiring errors.

You can supply the directory through `--recorder.recording` instead of Python.
The directory must be new; existing directories are errors. Recording creates
`schema.json` and `recording.mcap`, or `recording.db` with `format="sqlite"`.
The profile's metadata name is not a filesystem path.

## Replace USB cameras with RealSense

Give cameras stable logical names such as `left_wrist`, `right_wrist`, and
`overhead`. The platform selects drivers and devices. For example, these are real
Blueprint declarations, not functions that start cameras:

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.camera.webcam import WebcamConfig

usb = autoconnect(*(
    WebcamModule.blueprint(
        instance_name=name,
        hardware=WebcamConfig(camera_index=index, width=640, height=480, fps=30),
    )
    for index, name in enumerate(("left_wrist", "right_wrist", "overhead"))
))
realsense = autoconnect(*(
    RealSenseCamera.blueprint(
        instance_name=name, serial_number=serial, width=640, height=480, fps=30,
    )
    for name, serial in {
        "left_wrist": "LEFT_DEVICE_SERIAL",
        "right_wrist": "RIGHT_DEVICE_SERIAL",
        "overhead": "OVERHEAD_DEVICE_SERIAL",
    }.items()
))
remappings = [
    (name, "color_image", f"{name}_image")
    for name in ("left_wrist", "right_wrist", "overhead")
]
usb = usb.remappings(remappings)
realsense = realsense.remappings(remappings)
```

Replace the example serial numbers with your devices' serial numbers.
Replace `usb` with `realsense` in the platform composition. Keep the profile when
RGB message types, output dimensions, and logical stream names stay the same.
Existing ROS or remote producers use the same stream contract.

If the output dimensions change, change the image feature shape or normalize the
camera output. Preparation checks actual shape and dtype. Recording preserves raw
streams and does not decode every image for live shape validation. To record depth,
add an explicit depth feature and connect its producer.

## Prepare the saved recording

Capture stores asynchronous streams at their native rates. The saved schema fixes
the alignment to a 30 Hz grid, anchored at the first video observation, or the first
observation when there is no video. Snapshot matching uses a 20 ms tolerance.
Accepted joint updates use causal hold: only commands at or before the target time
contribute, and omitted joints retain their last accepted target. Missing accepted
targets never fall back to measured state or future commands.

The collection API exposes no `sync` or `quality` options. Existing low-level
preparation APIs retain their alignment and quality settings for offline use.

```python session=profile no-result
from dimos.imitation.dataprep.schema import RecordingSchema
from dimos.imitation.dataprep.schema import OutputConfig

schema = RecordingSchema.model_validate_json(profile.to_schema().model_dump_json())
config = schema.dataprep_config(
    Path("recordings/session-001"),
    OutputConfig(format="lerobot", path=Path("datasets/custom-arm")),
)
assert config.recording_schema.action["action"].names == joints
assert config.recording_schema.sync.rate_hz == 30
assert "message_type" not in schema.model_dump_json()
```

For an existing recording, call `RecordingSchema.read(directory)` before creating
the preparation config. Copy the complete directory to move it between machines.
Preparation reads the saved schema, including resolved recorded stream names, and
does not import the current robot configuration. Existing schemas with top-level
`name` and `robot_type` remain readable.

See [offline alignment](/dimos/imitation/dataprep/README.md) and
[the isolated LeRobot exporter](/dimos/imitation/dataprep/README.md#lerobot-export).

## Run the profile-to-dataset E2E

```bash
cd native/python/lerobot
uv run --frozen --group tests --with-editable ../../.. pytest dimos_lerobot/test_profile_collection_e2e.py
```

The test generates typed synthetic data from a collection profile, writes its
schema and recording with the Python SQLite store, moves the recording directory,
and converts it to a real LeRobot dataset. It checks one and two cameras, canonical
training keys, joint order, causal sparse targets, decoded video, and saved-schema
independence from the original profile. It does not test the native recorder and
requires no Rust toolchain, recorder executable, hardware, or policy training.

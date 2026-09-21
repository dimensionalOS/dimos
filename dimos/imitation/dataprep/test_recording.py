# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from contextlib import contextmanager

import h5py
from mcap.writer import Writer as McapWriter
import numpy as np
import pytest

from dimos.imitation.collection.profile import CollectionFeature, CollectionProfile
from dimos.imitation.collection.recording import RecordingSchema
from dimos.imitation.dataprep.build import inspect_recording, run_dataprep
from dimos.imitation.dataprep.core import FeatureSpec, OutputConfig, SyncConfig
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState


@contextmanager
def _raw_writer(path, types):
    if path.suffix == ".db":
        with SqliteStore(path=str(path)) as store:
            streams = {name: store.stream(name, kind, codec="lcm") for name, kind in types.items()}
            yield lambda name, message: streams[name].append(message, ts=message.ts)
    else:
        with path.open("wb") as file:
            writer = McapWriter(file)
            writer.start()
            channels = {
                name: writer.register_channel(
                    topic=name,
                    message_encoding="lcm",
                    schema_id=0,
                    metadata={
                        "dimos.payload_type": f"{kind.__module__}.{kind.__qualname__}",
                        "dimos.observation_time": "publish_time",
                    },
                )
                for name, kind in types.items()
            }

            def append(name, message):
                stamp = round(message.ts * 1_000_000_000)
                writer.add_message(
                    channel_id=channels[name],
                    log_time=stamp,
                    publish_time=stamp,
                    data=message.lcm_encode(),
                )

            try:
                yield append
            finally:
                writer.finish()


@pytest.mark.parametrize("format", ["mcap", "db"])
@pytest.mark.parametrize("camera_count", [1, 2, 4])
def test_moved_recording_directory_prepares_saved_episodes(format, camera_count, tmp_path):
    cameras = {
        f"camera_{index}": CollectionFeature(
            stream=f"view_{index}",
            message_type=Image,
            field="data",
            dtype="video",
            shape=(8, 8, 3),
            names=["height", "width", "channels"],
        )
        for index in range(camera_count)
    }
    joints = CollectionFeature(
        stream="measured",
        message_type=JointState,
        field="position",
        dtype="float32",
        shape=(2,),
        names=["right", "left"],
    )
    profile = CollectionProfile(
        name="external-robot",
        robot_type="custom",
        observations={**cameras, "state": joints},
        actions={"action": joints},
        sync=SyncConfig(anchor="camera_0", rate_hz=30, tolerance_ms=20),
    )
    directory = tmp_path / "original"
    directory.mkdir()
    schema = profile.to_schema()
    schema.payload = f"recording.{format}"
    (directory / "schema.json").write_text(schema.model_dump_json())
    types = {**profile.input_types(), "status": EpisodeStatus}
    with _raw_writer(directory / schema.payload, types) as append:
        for start, event in [(10.0, "save"), (20.0, "discard"), (30.0, None)]:
            append(
                "status",
                EpisodeStatus(
                    task_label="pick",
                    ts=start,
                    state="recording",
                    last_event="start",
                    episodes_saved=0,
                    episodes_discarded=0,
                ),
            )
            for index in range(3):
                ts = start + index / 30
                append(
                    "measured",
                    JointState(ts=ts, name=["left", "right"], position=[index, index + 10]),
                )
                for camera in cameras.values():
                    append(
                        camera.stream,
                        Image(
                            ts=ts, format=ImageFormat.RGB, data=np.zeros((8, 8, 3), dtype=np.uint8)
                        ),
                    )
            if event is not None:
                append(
                    "status",
                    EpisodeStatus(
                        task_label="pick",
                        ts=ts,
                        state="idle",
                        last_event=event,
                        episodes_saved=1,
                        episodes_discarded=0,
                    ),
                )
    moved = tmp_path / "moved"
    directory.rename(moved)
    restored = RecordingSchema.read(moved)
    config = restored.dataprep_config(
        moved, OutputConfig(format="hdf5", path=tmp_path / "dataset.h5")
    )
    result = run_dataprep(config)
    with h5py.File(result, "r") as dataset:
        assert list(dataset["episodes"]) == ["episode_000000"]
        np.testing.assert_array_equal(
            dataset["episodes"]["episode_000000"]["observation"]["state"][:],
            [[10, 0], [11, 1], [12, 2]],
        )
    assert config.source == str(moved / f"recording.{format}")
    assert restored.observation == schema.observation


@pytest.mark.parametrize("format", ["mcap", "db"])
def test_sparse_actions_inspect_and_convert_with_recording_history(format, tmp_path):
    schema = RecordingSchema(
        name="held-positions",
        robot_type="custom",
        payload=f"recording.{format}",
        observation={
            "state": FeatureSpec(
                stream="measured",
                field="position",
                dtype="float32",
                shape=(3,),
                names=["left", "right", "gripper"],
            )
        },
        action={
            "action": FeatureSpec(
                stream="commands",
                field="position",
                dtype="float32",
                shape=(3,),
                names=["left", "right", "gripper"],
                source_kind="joint_position_updates",
            )
        },
        sync=SyncConfig(anchor="state", rate_hz=1, tolerance_ms=20),
    )
    (tmp_path / "schema.json").write_text(schema.model_dump_json())
    source = tmp_path / schema.payload
    with _raw_writer(
        source,
        {
            "measured": JointState,
            "commands": JointState,
            "status": EpisodeStatus,
        },
    ) as append:
        # Deliberately insert updates out of timestamp order.
        append("commands", JointState(ts=2, name=["gripper"], position=[0]))
        append("commands", JointState(ts=1, name=["right", "left"], position=[2, 1]))
        append("commands", JointState(ts=11, name=["gripper"], position=[1]))
        for index, start in enumerate([10, 20]):
            append(
                "status",
                EpisodeStatus(
                    ts=start,
                    state="recording",
                    last_event="start",
                    episodes_saved=index,
                    episodes_discarded=0,
                    task_label="fold",
                ),
            )
            for ts in [start, start + 1]:
                append(
                    "measured",
                    JointState(
                        ts=ts,
                        name=["left", "right", "gripper"],
                        position=[0, 0, 0],
                    ),
                )
            append(
                "status",
                EpisodeStatus(
                    ts=start + 1,
                    state="idle",
                    last_event="save",
                    episodes_saved=index + 1,
                    episodes_discarded=0,
                    task_label="fold",
                ),
            )
    restored = RecordingSchema.read(tmp_path)
    config = restored.dataprep_config(
        tmp_path,
        OutputConfig(format="hdf5", path=tmp_path / "dataset.h5"),
    )

    info = inspect_recording(source, config=config)
    result = run_dataprep(config)

    assert [
        (report["valid"], report["emitted_frames"], report["filled_frames"])
        for report in info["quality"]
    ] == [(True, 2, 0), (True, 2, 0)]
    with h5py.File(result, "r") as dataset:
        np.testing.assert_array_equal(
            dataset["episodes/episode_000000/action/action"],
            [[1, 2, 0], [1, 2, 1]],
        )
        np.testing.assert_array_equal(
            dataset["episodes/episode_000001/action/action"],
            [[1, 2, 1], [1, 2, 1]],
        )

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

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import open3d as o3d
import pytest

from dimos.agents.code_policy_core import FrozenMemoryEnvironment
from dimos.benchmark.short_horizon_qa.models import MapperSettings
from dimos.benchmark.short_horizon_qa.prepare import (
    prepare_bundle,
    resolve_progress,
)
from dimos.benchmark.short_horizon_qa.service import frozen_qa_config, load_bundle
from dimos.memory2.store.frozen import FrozenMemoryStore
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def _point_cloud(x: float, *, frame_id: str = "world", ts: float) -> PointCloud2:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.asarray([[x, 0.0, 0.5]]))
    return PointCloud2(cloud, frame_id=frame_id, ts=ts)


@pytest.fixture
def recording(tmp_path: Path) -> Path:
    path = tmp_path / "recording.db"
    with SqliteStore(path=str(path)) as store:
        lidar = store.stream("lidar", PointCloud2)
        odom = store.stream("odom", int)
        for index in range(10):
            timestamp = 100.0 + index
            lidar.append(_point_cloud(index * 0.2, ts=timestamp), ts=timestamp)
            odom.append(index, ts=timestamp)
    return path


def test_prepare_builds_reusable_runtime_maps_without_changing_source(
    recording: Path, tmp_path: Path
) -> None:
    output = tmp_path / "bundle"
    before = recording.read_bytes()
    progress: list[tuple[int, int]] = []

    manifest = prepare_bundle(
        recording,
        [4.0, 9.0],
        output,
        mapper=MapperSettings(device="CPU:0"),
        map_progress=lambda current, total: progress.append((current, total)),
    )

    assert recording.read_bytes() == before
    assert [item.map_frame_count for item in manifest.cutoffs] == [5, 10]
    assert [item.map_timestamp for item in manifest.cutoffs] == [104.0, 109.0]
    assert progress == [(10, 10)]
    assert (output / "derived.db").is_file()
    encoded = json.loads((output / "manifest.v1.json").read_text())
    assert encoded["source_size_bytes"] == recording.stat().st_size
    assert encoded["source_mtime_ns"] == recording.stat().st_mtime_ns
    assert "source_sha256" not in encoded

    with FrozenMemoryStore(
        SqliteStore(path=str(recording), must_exist=True, read_only=True),
        derived=SqliteStore(path=str(output / "derived.db"), must_exist=True, read_only=True),
        through_timestamp=104.0,
    ) as memory:
        assert memory.streams.lidar.count() == 5
        assert memory.streams.odom.last().data == 4
        assert memory.streams.global_map.last().tags["frame_count"] == 5
        assert len(memory.streams.global_map.last().data) == 5


def test_prepare_reuses_one_derived_map_for_nearby_cutoffs(recording: Path, tmp_path: Path) -> None:
    output = tmp_path / "bundle"

    manifest = prepare_bundle(
        recording,
        [4.0, 4.5],
        output,
        mapper=MapperSettings(device="CPU:0"),
    )

    assert manifest.cutoffs[0].map_observation_id == manifest.cutoffs[1].map_observation_id
    with SqliteStore(path=str(output / "derived.db"), must_exist=True, read_only=True) as derived:
        assert derived.stream("global_map").count() == 1


def test_progress_resolves_exact_end_and_persists_provenance(
    recording: Path, tmp_path: Path
) -> None:
    output = tmp_path / "progress-bundle"
    manifest = prepare_bundle(
        recording,
        [],
        output,
        progress=[4 / 9, 1.0, 1.0],
        mapper=MapperSettings(device="CPU:0"),
    )

    assert [item.normalized_progress for item in manifest.cutoffs] == [4 / 9, 1.0]
    assert [item.cutoff_timestamp for item in manifest.cutoffs] == [104.0, 109.0]
    assert manifest.cutoffs[-1].stream_boundaries[0].count == 10
    loaded_manifest, cutoff, _, _ = load_bundle(output, progress=1.0)
    assert cutoff.cutoff_timestamp == loaded_manifest.recording_end_timestamp

    encoded = json.loads((output / "manifest.v1.json").read_text())
    assert encoded["cutoffs"][-1]["normalized_progress"] == 1.0


def test_progress_resolution_has_exact_endpoints_and_linear_interior() -> None:
    assert resolve_progress(0.0, 100.0, 109.0) == 100.0
    assert resolve_progress(0.5, 100.0, 110.0) == 105.0
    assert resolve_progress(1.0, 100.0, 109.0) == 109.0


@pytest.mark.parametrize("progress", [-0.1, 1.1, float("inf"), float("nan")])
def test_prepare_rejects_invalid_progress(recording: Path, tmp_path: Path, progress: float) -> None:
    with pytest.raises(ValueError, match="progress|Progress"):
        prepare_bundle(
            recording,
            [],
            tmp_path / "bundle",
            progress=[progress],
            mapper=MapperSettings(device="CPU:0"),
        )


def test_progress_before_first_map_preserves_runtime_emission_rule(
    recording: Path, tmp_path: Path
) -> None:
    with pytest.raises(ValueError, match="No runtime map was emitted"):
        prepare_bundle(
            recording,
            [],
            tmp_path / "bundle",
            progress=[0.0],
            mapper=MapperSettings(device="CPU:0"),
        )


def test_prepare_rejects_cutoff_before_first_runtime_emission(
    recording: Path, tmp_path: Path
) -> None:
    output = tmp_path / "bundle"

    with pytest.raises(ValueError, match="No runtime map was emitted"):
        prepare_bundle(
            recording,
            [2.0],
            output,
            mapper=MapperSettings(device="CPU:0"),
        )

    assert not output.exists()


def test_prepare_rejects_non_world_lidar(tmp_path: Path) -> None:
    recording = tmp_path / "sensor-frame.db"
    with SqliteStore(path=str(recording)) as store:
        lidar = store.stream("lidar", PointCloud2)
        for index in range(5):
            timestamp = 100.0 + index
            lidar.append(_point_cloud(float(index), frame_id="lidar", ts=timestamp), ts=timestamp)

    with pytest.raises(ValueError, match="does not match mapper frame"):
        prepare_bundle(
            recording,
            [4.0],
            tmp_path / "bundle",
            mapper=MapperSettings(device="CPU:0"),
        )


def test_bundle_loads_into_standalone_code_policy_config(recording: Path, tmp_path: Path) -> None:
    output = tmp_path / "bundle"
    prepare_bundle(
        recording,
        [4.0],
        output,
        mapper=MapperSettings(device="CPU:0"),
    )

    _, cutoff, source_path, derived_path = load_bundle(output, 4.0)
    config = frozen_qa_config(source_path, derived_path, cutoff)
    assert isinstance(config.environment, FrozenMemoryEnvironment)
    assert config.environment.recording_path == str(recording.resolve())
    assert config.environment.derived_recording_path == str(derived_path)
    assert config.environment.memory_cutoff_timestamp == 104.0


def test_bundle_identity_check_rejects_changed_source_recording(
    recording: Path, tmp_path: Path
) -> None:
    output = tmp_path / "bundle"
    prepare_bundle(
        recording,
        [4.0],
        output,
        mapper=MapperSettings(device="CPU:0"),
    )
    with recording.open("ab") as stream:
        stream.write(b"tampered")

    with pytest.raises(ValueError, match="Source recording identity changed"):
        load_bundle(output, 4.0)

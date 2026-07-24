from __future__ import annotations

from typing import Any, cast

import numpy as np

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.point_cloud_self_filter import (
    PointCloudSelfFilter,
    PointCloudSelfFilterConfig,
    SelfFilterRegion,
)
from dimos.protocol.tf.tf import MultiTBuffer


def _cloud(
    points: list[tuple[float, float, float]], intensities: list[float] | None = None
) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float32),
        frame_id="cloud",
        timestamp=12.5,
        intensities=None if intensities is None else np.asarray(intensities, dtype=np.float32),
    )


def _filter(regions: list[SelfFilterRegion], drop_missing: bool = False) -> PointCloudSelfFilter:
    module = object.__new__(PointCloudSelfFilter)
    state = cast("dict[str, Any]", module.__dict__)
    state["config"] = PointCloudSelfFilterConfig(
        regions=regions, drop_cloud_on_missing_tf=drop_missing, tf_tolerance_s=100.0
    )
    state["_tf"] = MultiTBuffer()
    return module


def test_self_filter_removes_robot_points_and_preserves_metadata() -> None:
    module = _filter([SelfFilterRegion(shape="sphere", frame_id="tool", radius=1.0)])
    cast("MultiTBuffer", module.__dict__["_tf"]).receive_transform(
        Transform(
            translation=Vector3(1.0, 0.0, 0.0), frame_id="cloud", child_frame_id="tool", ts=12.5
        )
    )
    cloud = _cloud([(1.0, 0.0, 0.0), (1.9, 0.0, 0.0), (3.0, 0.0, 0.0)], [1.0, 2.0, 3.0])

    filtered = module.filter_cloud(cloud)

    assert filtered is not None
    np.testing.assert_allclose(filtered.points_f32(), [[3.0, 0.0, 0.0]])
    intensities = filtered.intensities_f32()
    assert intensities is not None
    np.testing.assert_allclose(intensities, [3.0])
    assert filtered.frame_id == "cloud"
    assert filtered.ts == 12.5


def test_self_filter_handles_empty_and_non_robot_clouds() -> None:
    module = _filter([SelfFilterRegion(shape="box", frame_id="tool", size=(2.0, 2.0, 2.0))])
    module.__dict__["_tf"].receive_transform(
        Transform(frame_id="cloud", child_frame_id="tool", ts=12.5)
    )
    empty = module.filter_cloud(_cloud([]))
    outside = module.filter_cloud(_cloud([(3.0, 0.0, 0.0)]))

    assert empty is not None and len(empty) == 0
    assert outside is not None and len(outside) == 1


def test_missing_tf_is_deterministic() -> None:
    region = SelfFilterRegion(shape="sphere", frame_id="missing", radius=1.0)
    assert len(_filter([region]).filter_cloud(_cloud([(0.0, 0.0, 0.0)]))) == 1  # type: ignore[arg-type]
    assert _filter([region], drop_missing=True).filter_cloud(_cloud([(0.0, 0.0, 0.0)])) is None


def test_partial_required_tf_drops_whole_cloud() -> None:
    module = _filter(
        [
            SelfFilterRegion(shape="sphere", frame_id="present", radius=1.0),
            SelfFilterRegion(shape="sphere", frame_id="missing", radius=1.0),
        ],
        drop_missing=True,
    )
    cast("MultiTBuffer", module.__dict__["_tf"]).receive_transform(
        Transform(frame_id="cloud", child_frame_id="present", ts=12.5)
    )

    assert module.filter_cloud(_cloud([(0.0, 0.0, 0.0)])) is None


def test_self_filter_stream_types_are_voxel_map_compatible() -> None:
    assert PointCloudSelfFilter.__annotations__["pointcloud"]
    assert PointCloudSelfFilter.__annotations__["filtered_pointcloud"]

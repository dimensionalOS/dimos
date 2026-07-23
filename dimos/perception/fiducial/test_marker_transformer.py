# Copyright 2025-2026 Dimensional Inc.
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

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.marker import Detection3DMarker
from dimos.perception.fiducial.apriltag_aggregation import AggregationConfig, pose7_from_matrix
from dimos.perception.fiducial.marker_transformer import AggregateTagBursts, DetectMarkers
from dimos.perception.fiducial.test_helpers import (
    blank_image,
    camera_info,
    synthetic_marker_image,
)


def test_detect_markers_transformer_can_emit_empty_frame_sentinel() -> None:
    image = Image(
        data=np.full((480, 640, 3), 255, dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera_optical",
        ts=13.0,
    )
    obs = Observation[Image](
        id=43,
        ts=image.ts,
        data_type=Image,
        pose=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0),
        _data=image,
    )
    transformer = DetectMarkers(
        camera_info=camera_info(image.ts),
        marker_length_m=0.18,
        aruco_dictionary="DICT_APRILTAG_36h11",
        emit_empty_frames=True,
    )

    results = list(transformer(iter([obs])))

    assert len(results) == 1
    out = results[0]
    assert out.id == obs.id
    assert out.ts == obs.ts
    assert out.data is None
    assert out.tags["marker_frame_image"] is image
    assert out.tags["marker_frame_count"] == 0


def test_detect_markers_transformer_uses_callablecamera_info_source() -> None:
    """Invariant: a callable camera_info returning None skips the frame; once it yields
    info the transformer detects and carries the observation context through -- id/ts
    preserved, marker_id/track_id tags stamped, data.track_id defaulted to -1."""
    image = synthetic_marker_image(marker_id=7, ts=14.0)
    obs = Observation[Image](
        id=44,
        ts=image.ts,
        data_type=Image,
        pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        _data=image,
    )
    latest_info: CameraInfo | None = None
    transformer = DetectMarkers(
        camera_info=lambda: latest_info,
        marker_length_m=0.18,
        aruco_dictionary="DICT_APRILTAG_36h11",
        emit_empty_frames=True,
    )

    assert list(transformer(iter([obs]))) == []

    latest_info = camera_info(image.ts)
    results = list(transformer(iter([obs])))

    assert len(results) == 1
    out = results[0]
    assert out.id == obs.id
    assert out.ts == obs.ts
    assert out.pose is not None
    assert out.data.marker_id == 7
    assert out.data.track_id == -1
    assert out.tags["marker_id"] == 7
    assert out.tags["track_id"] == -1
    assert out.tags["marker_frame_count"] == 1


def test_detect_markers_rebuilds_intrinsics_without_resetting_smoothing_track() -> None:
    marker_id = 7
    marker_length_m = 0.18
    image_a = synthetic_marker_image(marker_id=marker_id, ts=15.0)
    image_b = synthetic_marker_image(marker_id=marker_id, ts=15.2)
    info_a = camera_info(image_a.ts)
    info_b = camera_info(image_b.ts)
    info_b.K = info_b.K.copy()
    info_b.P = info_b.P.copy()
    info_b.K[0] = info_b.K[4] = 900.0
    info_b.P[0] = info_b.P[5] = 900.0
    latest_info: CameraInfo | None = info_a

    obs_a = Observation[Image](
        id=45,
        ts=image_a.ts,
        data_type=Image,
        pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        _data=image_a,
    )
    obs_b = Observation[Image](
        id=46,
        ts=image_b.ts,
        data_type=Image,
        pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        _data=image_b,
    )
    transformer = DetectMarkers(
        camera_info=lambda: latest_info,
        marker_length_m=marker_length_m,
        aruco_dictionary="DICT_APRILTAG_36h11",
        smoothing_window=1.0,
    )

    first = next(transformer(iter([obs_a])))
    latest_info = info_b
    second = next(transformer(iter([obs_b])))
    raw_after_k_change = next(
        DetectMarkers(
            camera_info=info_b,
            marker_length_m=marker_length_m,
            aruco_dictionary="DICT_APRILTAG_36h11",
        )(iter([obs_b]))
    )

    assert first.data.track_id == second.data.track_id
    assert first.data.track_id > 0
    assert raw_after_k_change.data.center.z != pytest.approx(first.data.center.z, abs=0.05)
    assert second.data.center.z == pytest.approx(
        (first.data.center.z + raw_after_k_change.data.center.z) / 2.0,
        abs=0.02,
    )


# ---------------------------------------------------------------------------
# AggregateTagBursts: the four AggregationConfig glimpse gates + ONE robustly-aggregated
# pose per tag visit. All four gates were dead before the move -- corners_px, the
# reprojection error and the camera transform are dropped by the wire
# Detection3DArray, so nothing downstream could ever evaluate them. Every glimpse
# below is CONSTRUCTED from a known camera_optical<-marker pose (SIMULATED: no
# image decode, no detector, no bus), so each gate's input is exact rather than
# fitted, and `published` is the same seam Out.publish gives the live module.
# ---------------------------------------------------------------------------

_CAMERA = Transform(
    translation=Vector3(1.2, 0.4, 0.7),
    rotation=Quaternion(*Rotation.from_euler("z", 25.0, degrees=True).as_quat()),
    frame_id="world",
    child_frame_id="camera_optical",
    ts=10.0,
)


def _optical_T_marker(distance_m: float, view_angle_deg: float) -> np.ndarray:
    """A marker ``distance_m`` straight ahead, tilted ``view_angle_deg`` about the
    camera x-axis. The line of sight is then +z and the tag normal is R[:, 2], so
    view_quality reads back exactly these two numbers -- known truth, not a fit."""
    optical_T_marker = np.eye(4)
    optical_T_marker[:3, :3] = Rotation.from_euler("x", view_angle_deg, degrees=True).as_matrix()
    optical_T_marker[:3, 3] = (0.0, 0.0, distance_m)
    return optical_T_marker


def _square_corners_px(side_px: float) -> np.ndarray:
    """An axis-aligned square quad, so tag_side_px (sqrt of shoelace area) == side_px."""
    return np.array(
        [[0.0, 0.0], [side_px, 0.0], [side_px, side_px], [0.0, side_px]], dtype=np.float32
    )


def _glimpse(
    *,
    marker_id: int = 7,
    optical_T_marker: np.ndarray | None = None,
    reproj_px: float = 0.5,
    tag_side_px: float = 110.0,
    camera: Transform | None = None,
    ts: float = 10.0,
) -> Observation[Detection3DMarker | None]:
    """One observation shaped exactly as DetectMarkers yields it: world_T_marker on
    center/orientation, world_T_optical on ``.transform``, and the pixel-domain gate
    inputs beside them. Every default clears every gate."""
    camera = _CAMERA if camera is None else camera
    if optical_T_marker is None:
        optical_T_marker = _optical_T_marker(0.5, 20.0)
    x, y, z, qx, qy, qz, qw = pose7_from_matrix(camera.to_matrix() @ optical_T_marker)
    det = Detection3DMarker(
        bbox=(0.0, 0.0, tag_side_px, tag_side_px),
        track_id=-1,
        class_id=marker_id,
        confidence=1.0,
        name="",
        ts=ts,
        image=blank_image(ts=ts),
        center=Vector3(x, y, z),
        size=Vector3(0.1, 0.1, 0.0),
        transform=camera,
        frame_id="world",
        orientation=Quaternion(qx, qy, qz, qw),
        marker_id=marker_id,
        corners_px=_square_corners_px(tag_side_px),
        dictionary="DICT_APRILTAG_36h11",
        reprojection_error=reproj_px,
    )
    return Observation(
        id=0,
        ts=ts,
        data_type=Detection3DMarker,
        pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        _data=det,
    )


def _aggregator(
    config: AggregationConfig | None = None,
) -> tuple[AggregateTagBursts, list[Detection3DArray]]:
    """An AggregateTagBursts publishing into a capture list -- the same callable seam
    ``Out.publish`` hands it live, with no transport and no bus."""
    published: list[Detection3DArray] = []
    return AggregateTagBursts(published.append, config or AggregationConfig()), published


def test_aggregate_tag_bursts_gate_suppresses_a_bad_visit_then_accepts_a_clean_one() -> None:
    """Invariant: a gate quality computed from the wrapper's transform/corners suppresses
    publish. Twice min_observations glimpses at 60 deg (> max_view_angle_deg 45) publish
    nothing -- proving view_quality(inv(transform) @ world_T_marker) reaches the gate --
    then min_observations clean ones publish, so the silence is the gate, not inertness.
    Which gate fires for which violation is pinned in test_apriltag_aggregation."""
    aggregator, published = _aggregator()

    for i in range(6):
        aggregator(_glimpse(ts=10.0 + 0.5 * i, optical_T_marker=_optical_T_marker(0.5, 60.0)))
    assert published == []

    for i in range(3):
        aggregator(_glimpse(ts=20.0 + 0.5 * i))
    assert len(published) == 1


def test_aggregate_tag_bursts_publishes_once_per_burst_not_once_per_frame() -> None:
    """Invariant: emission is EDGE-triggered. The glimpse that first brings the
    window to min_observations publishes; every later glimpse of the same visit
    keeps it satisfied and must publish nothing, or one unchanged fix would ship at
    the detector's rate (~2/s behind the 0.5 s QualityWindow)."""
    aggregator, published = _aggregator()

    for i in range(2):
        aggregator(_glimpse(ts=10.0 + 0.5 * i))
    assert published == []  # 2 < min_observations 3

    aggregator(_glimpse(ts=11.0))
    assert len(published) == 1  # the 3rd glimpse IS the edge

    for i in range(3, 11):  # the visit continues inside time_window_s: still one
        aggregator(_glimpse(ts=10.0 + 0.5 * i))
    assert len(published) == 1

    # The edge re-arms per VISIT: a gap longer than time_window_s purges the marker's
    # window on its next sighting, so completing a second burst publishes again.
    aggregator(_glimpse(ts=41.0))  # 30 s later: the 5 s window purges down to this one
    assert len(published) == 1
    aggregator(_glimpse(ts=41.5))
    aggregator(_glimpse(ts=42.0))
    assert len(published) == 2


def test_aggregate_tag_bursts_publishes_one_message_per_marker_id() -> None:
    """Invariant: one message per (marker, visit). Two tags in view are two bursts --
    each composes its own map_T_world downstream, so they cannot share an array."""
    aggregator, published = _aggregator()

    for i in range(3):
        aggregator(_glimpse(marker_id=7, ts=10.0 + 0.5 * i))
        aggregator(_glimpse(marker_id=9, ts=10.0 + 0.5 * i))

    assert [msg.detections[0].id for msg in published] == ["7", "9"]


def test_aggregate_tag_bursts_ignores_the_empty_frame_sentinel() -> None:
    """Invariant: DetectMarkers(emit_empty_frames=True) yields data=None for a
    tag-free frame, which is the FIRST thing most streams carry. It must be skipped,
    not dereferenced."""
    aggregator, published = _aggregator()

    aggregator(
        Observation(
            id=0,
            ts=10.0,
            data_type=type(None),
            pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            _data=None,
        )
    )

    assert published == []


def test_aggregate_tag_bursts_stamps_the_covariance_and_score_health_signal() -> None:
    """Invariant: an aggregated entry ships its own trust, and the two channels are the
    same number twice -- score == 1/scale, covariance == the base blocks * scale -- so
    they cannot disagree. Median range 0.8 m (2x REF_DISTANCE_M) and median reproj
    1.5 px (1.5x REF_REPROJ_PX) make scale = 2^2 * 1.5^2 = 9.0; the head-on camera at the
    world origin leaves R identity so every diagonal is readable."""
    origin_camera = Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
        child_frame_id="camera_optical",
        ts=10.0,
    )
    aggregator, published = _aggregator()

    for i in range(3):
        aggregator(
            _glimpse(
                optical_T_marker=_optical_T_marker(0.8, 0.0),
                reproj_px=1.5,
                camera=origin_camera,
                ts=10.0 + 0.5 * i,
            )
        )

    assert len(published) == 1
    result = published[0].detections[0].results[0]
    assert result.hypothesis.score == pytest.approx(1.0 / 9.0)
    np.testing.assert_allclose(
        np.asarray(result.pose.covariance).reshape(6, 6),
        np.diag(np.array([0.0025, 0.0025, 0.25, 0.04, 0.04, 0.0025]) * 9.0),
        atol=1e-12,
    )

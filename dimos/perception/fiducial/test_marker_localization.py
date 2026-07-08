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

"""No-hardware: a real ``cv2.aruco``-rendered frame decoded end to end, plus the module publish."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np
from pydantic import ValidationError
import pytest

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.perception.fiducial.marker_localization import (
    MAP_FRAME,
    OPTICAL_FRAME,
    LocalizationConfig,
    detect_markers,
    load_marker_map,
    localize_from_detections,
)
from dimos.perception.fiducial.marker_localization_module import MarkerLocalizationModule
from dimos.perception.fiducial.marker_pose import (
    camera_info_to_cv_matrices,
    create_aruco_detector,
    estimate_marker_pose_candidates,
    rvec_tvec_to_transform,
)
from dimos.perception.fiducial.test_helpers import (
    blank_image,
    camera_info,
    synthetic_marker_image,
    world_T_optical,
)

_MARKER_LENGTH_M, _MARKER_ID = 0.15, 7
_MARKER_MAP = {_MARKER_ID: Transform(frame_id=MAP_FRAME, child_frame_id=f"marker_{_MARKER_ID}")}
_CFG_TIGHT = LocalizationConfig(_MARKER_LENGTH_M, max_reprojection_error_px=1.0)
_CFG_LOOSE = LocalizationConfig(_MARKER_LENGTH_M, max_reprojection_error_px=3.0)


def test_localize_from_rendered_marker() -> None:
    """Recovered pose matches the pinhole depth; a corrupted corner makes the gate reject."""
    info, image = camera_info(), synthetic_marker_image(_MARKER_ID, ts=10.0)
    detections = detect_markers(
        image.to_grayscale().as_numpy(), create_aruco_detector("DICT_APRILTAG_36h11")
    )
    pose = localize_from_detections(detections, _MARKER_MAP, info, _CFG_TIGHT, 10.0)
    assert pose is not None
    expected_tz = info.K[0] * _MARKER_LENGTH_M / 220  # 220px: synthetic tile size; K[0] = fx
    assert abs(pose.translation.z - expected_tz) < 0.01
    detections[0][1][0] += (60.0, -45.0)  # yank one corner off -> inconsistent quad
    assert localize_from_detections(detections, _MARKER_MAP, info, _CFG_LOOSE, 10.0) is None


async def test_module_publishes_world_map_tf() -> None:
    module = MarkerLocalizationModule(marker_length_m=_MARKER_LENGTH_M, camera_info=camera_info())
    module._marker_map = _MARKER_MAP
    try:
        module.tf.publish(Transform(frame_id="world", child_frame_id="camera_optical", ts=10.0))
        await module.handle_color_image(synthetic_marker_image(_MARKER_ID, ts=10.0))
        assert module.tf.get("world", "map") is not None  # .now()-stamped, not msg.ts
    finally:
        module.stop()


# --- detect_markers -----------------------------------------------------


def test_detect_markers_returns_empty_for_blank_frame() -> None:
    """No markers in frame -> empty detection list, no crash."""
    detections = detect_markers(
        blank_image().to_grayscale().as_numpy(), create_aruco_detector("DICT_APRILTAG_36h11")
    )
    assert detections == []


# --- localize_from_detections --------------------------------------------


def test_localize_from_detections_ignores_unmapped_marker() -> None:
    """A detected id absent from the marker map is skipped cleanly, not raising."""
    corners = np.array(
        [[300.0, 220.0], [340.0, 220.0], [340.0, 260.0], [300.0, 260.0]], dtype=np.float32
    )
    assert (
        localize_from_detections([(99, corners)], _MARKER_MAP, camera_info(), _CFG_LOOSE, 10.0)
        is None
    )


def test_localize_from_detections_skips_degenerate_detection_without_crash() -> None:
    """Corners collapsed to a single point are an ill-posed solvePnP input (a corrupt detector
    output); estimate_marker_pose returns None and the detection is skipped, not raising."""
    degenerate = np.array([[320.0, 240.0]] * 4, dtype=np.float32)
    assert (
        localize_from_detections(
            [(_MARKER_ID, degenerate)], _MARKER_MAP, camera_info(), _CFG_LOOSE, 10.0
        )
        is None
    )


def test_localize_from_detections_requires_min_tags_corroboration() -> None:
    """A single good detection is rejected when config demands more corroborating tags."""
    info, image = camera_info(), synthetic_marker_image(_MARKER_ID, ts=10.0)
    detections = detect_markers(
        image.to_grayscale().as_numpy(), create_aruco_detector("DICT_APRILTAG_36h11")
    )
    cfg = LocalizationConfig(_MARKER_LENGTH_M, min_tags=2, max_reprojection_error_px=3.0)
    assert localize_from_detections(detections, _MARKER_MAP, info, cfg, 10.0) is None


def test_localize_from_detections_uses_lowest_error_tag_when_multiple_pass_gate() -> None:
    """Multi-tag: the returned pose must be at least as accurate as the best single tag.

    Regression test: the gate used to return ``good[0]`` — whichever detection happened to
    come first in detector order — silently degrading accuracy whenever a noisier-but-still
    gate-passing tag was first. Two markers project from the same known camera pose; one
    detection carries injected corner noise that still clears the reprojection gate, the
    other is clean. The result must match the clean tag's accuracy regardless of which one
    was listed first.
    """
    info = camera_info()
    k, d = camera_info_to_cv_matrices(info)
    h = _MARKER_LENGTH_M / 2.0
    obj = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float32)
    rvec0 = np.array([[0.02], [-0.01], [0.0]], dtype=np.float64)
    tvec0 = np.array([[0.05], [-0.03], [1.8]], dtype=np.float64)
    clean_pts, _ = cv2.projectPoints(obj, rvec0, tvec0, k, d)
    clean_corners = clean_pts.reshape(4, 2).astype(np.float32)
    noisy_corners = clean_corners.copy()
    noisy_corners[0] += (2.5, -2.0)
    noisy_corners[2] += (-1.5, 2.0)

    marker_map = {
        1: Transform(frame_id=MAP_FRAME, child_frame_id="marker_1"),
        2: Transform(frame_id=MAP_FRAME, child_frame_id="marker_2"),
    }
    # ambiguity_ratio_min=1.0 disables the mirror-ambiguity gate: this test pins
    # the lowest-error-wins fusion order-independence, and the injected corner
    # noise deliberately degrades the noisy tag enough that the (separately
    # tested) ambiguity gate would reject it outright.
    cfg = LocalizationConfig(
        _MARKER_LENGTH_M, min_tags=2, max_reprojection_error_px=5.0, ambiguity_ratio_min=1.0
    )
    truth = rvec_tvec_to_transform(
        rvec0, tvec0, frame_id=OPTICAL_FRAME, child_frame_id="marker_truth", ts=10.0
    ).inverse()

    noisy_first = localize_from_detections(
        [(1, noisy_corners), (2, clean_corners)], marker_map, info, cfg, 10.0
    )
    clean_first = localize_from_detections(
        [(2, clean_corners), (1, noisy_corners)], marker_map, info, cfg, 10.0
    )
    assert noisy_first is not None
    assert clean_first is not None

    solo_cfg = LocalizationConfig(
        _MARKER_LENGTH_M, max_reprojection_error_px=5.0, ambiguity_ratio_min=1.0
    )
    best_single = localize_from_detections([(2, clean_corners)], marker_map, info, solo_cfg, 10.0)
    assert best_single is not None
    best_error = truth.translation.distance(best_single.translation)

    # Order must not matter, and the fused result must be at least as accurate as the best tag.
    assert truth.translation.distance(noisy_first.translation) == pytest.approx(
        truth.translation.distance(clean_first.translation), abs=1e-6
    )
    assert truth.translation.distance(noisy_first.translation) <= best_error + 1e-6


def _projected_corners(
    rvec: np.ndarray, tvec: np.ndarray, info: CameraInfo, perturb: np.ndarray
) -> np.ndarray:
    """Project the marker square through the test camera and add a fixed sub-pixel
    perturbation (deterministic stand-in for corner-detection noise)."""
    k, d = camera_info_to_cv_matrices(info)
    h = _MARKER_LENGTH_M / 2.0
    obj = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float32)
    pts, _ = cv2.projectPoints(obj, rvec, tvec, k, d)
    corners: np.ndarray = (pts.reshape(4, 2) + perturb).astype(np.float32)
    return corners


_SUBPIXEL_NOISE = np.array(
    [[0.3, -0.2], [-0.25, 0.15], [0.2, 0.3], [-0.3, -0.25]], dtype=np.float32
)


def test_ambiguity_gate_rejects_mirror_ambiguous_view() -> None:
    """Weak perspective (small, near-head-on tag): the flipped IPPE candidate reprojects
    almost as well as the true one (ratio ~1.2 here, measured), so neither can be trusted —
    the absolute reprojection gate alone would happily accept it (~0.26px). The rejection
    must come from the ambiguity gate specifically: disabling it (ratio_min=1.0) accepts."""
    info = camera_info()
    corners = _projected_corners(
        np.array([[0.05], [0.03], [0.0]]), np.array([[0.1], [0.05], [5.0]]), info, _SUBPIXEL_NOISE
    )
    gated = LocalizationConfig(_MARKER_LENGTH_M)  # default ambiguity_ratio_min=2.0
    assert localize_from_detections([(_MARKER_ID, corners)], _MARKER_MAP, info, gated, 10.0) is None
    ungated = LocalizationConfig(_MARKER_LENGTH_M, ambiguity_ratio_min=1.0)
    assert (
        localize_from_detections([(_MARKER_ID, corners)], _MARKER_MAP, info, ungated, 10.0)
        is not None
    )


def test_ambiguity_gate_accepts_unambiguous_view() -> None:
    """Strong perspective (close, tilted tag): the runner-up candidate reprojects ~38x worse
    (measured), the view is geometrically unambiguous, and the default gate accepts it."""
    info = camera_info()
    rvec0, tvec0 = np.array([[0.5], [0.2], [0.0]]), np.array([[0.05], [0.02], [0.6]])
    corners = _projected_corners(rvec0, tvec0, info, _SUBPIXEL_NOISE)
    pose = localize_from_detections(
        [(_MARKER_ID, corners)], _MARKER_MAP, info, LocalizationConfig(_MARKER_LENGTH_M), 10.0
    )
    assert pose is not None
    truth = rvec_tvec_to_transform(
        rvec0, tvec0, frame_id=OPTICAL_FRAME, child_frame_id="marker_truth", ts=10.0
    ).inverse()
    assert truth.translation.distance(pose.translation) < 0.05


def test_estimate_marker_pose_candidates_returns_only_finite_poses() -> None:
    """The multi-candidate helper returns both IPPE solutions on a clean view and never a
    non-finite pose on degenerate input (solvePnPGeneric can emit all-NaN solutions there)."""
    info = camera_info()
    k, d = camera_info_to_cv_matrices(info)
    corners = _projected_corners(
        np.array([[0.05], [0.03], [0.0]]), np.array([[0.1], [0.05], [5.0]]), info, _SUBPIXEL_NOISE
    )
    candidates = estimate_marker_pose_candidates(corners, _MARKER_LENGTH_M, k, d)
    assert len(candidates) == 2
    assert all(np.all(np.isfinite(r)) and np.all(np.isfinite(t)) for r, t in candidates)
    degenerate = np.array([[320.0, 240.0]] * 4, dtype=np.float32)
    for r, t in estimate_marker_pose_candidates(degenerate, _MARKER_LENGTH_M, k, d):
        assert np.all(np.isfinite(r)) and np.all(np.isfinite(t))


# --- load_marker_map ------------------------------------------------------


def test_load_marker_map_round_trip(tmp_path: Path) -> None:
    yaml_path = tmp_path / "markers.yaml"
    yaml_path.write_text(
        "markers:\n"
        "  7:\n"
        "    translation: [1.0, 2.0, 3.0]\n"
        "    rotation: [0.0, 0.0, 0.0, 1.0]\n"
        "  12:\n"
        "    translation: [-0.5, 0.0, 1.25]\n"
        "    rotation: [0.0, 0.0, 0.7071068, 0.7071068]\n"
    )
    result = load_marker_map(yaml_path)
    assert set(result) == {7, 12}
    m7 = result[7]
    assert m7.frame_id == MAP_FRAME
    assert m7.child_frame_id == "marker_7"
    assert (m7.translation.x, m7.translation.y, m7.translation.z) == (1.0, 2.0, 3.0)
    assert (m7.rotation.x, m7.rotation.y, m7.rotation.z, m7.rotation.w) == (0.0, 0.0, 0.0, 1.0)
    assert result[12].child_frame_id == "marker_12"


def test_load_marker_map_missing_file_raises(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        load_marker_map(tmp_path / "does_not_exist.yaml")


def test_load_marker_map_malformed_entry_raises(tmp_path: Path) -> None:
    """A marker entry missing a required field fails loudly and typed, not silently."""
    yaml_path = tmp_path / "malformed.yaml"
    yaml_path.write_text("markers:\n  7:\n    translation: [1.0, 2.0, 3.0]\n")  # no rotation
    with pytest.raises(KeyError):
        load_marker_map(yaml_path)


def test_load_marker_map_no_markers_key_returns_empty(tmp_path: Path) -> None:
    """A YAML file with no top-level ``markers`` key is valid input -> empty map, not a crash."""
    yaml_path = tmp_path / "empty.yaml"
    yaml_path.write_text("other_key: 1\n")
    assert load_marker_map(yaml_path) == {}


@pytest.mark.parametrize(
    "translation,rotation",
    [
        ("[1.0]", "[0.0, 0.0, 0.0, 1.0]"),  # short translation would zero-fill silently
        ("1.0", "[0.0, 0.0, 0.0, 1.0]"),  # scalar translation, same silent Vector3 branch
        ("[1.0, 2.0, 3.0]", "[0.0, 0.0, 1.0]"),  # 3-element rotation is not a quaternion
        ("[1.0, 2.0, 3.0]", "[0.0, 0.0, 0.0, 0.0]"),  # zero norm crashes inverse() much later
        ("[.nan, 2.0, 3.0]", "[0.0, 0.0, 0.0, 1.0]"),  # NaN translation would publish as a pose
        ("[1.0, .inf, 3.0]", "[0.0, 0.0, 0.0, 1.0]"),  # infinite translation, same silent path
        ("[1.0, 2.0, 3.0]", "[.nan, 0.0, 0.0, 1.0]"),  # NaN rotation: norm is NaN, not usable
    ],
)
def test_load_marker_map_malformed_values_raise(
    tmp_path: Path, translation: str, rotation: str
) -> None:
    """Malformed *values* fail loudly at load time, naming the marker — never a silent
    zero-fill into a published correction, never a deferred ``Quaternion.inverse()`` crash."""
    yaml_path = tmp_path / "bad_values.yaml"
    yaml_path.write_text(
        f"markers:\n  7:\n    translation: {translation}\n    rotation: {rotation}\n"
    )
    with pytest.raises(ValueError, match="marker 7"):
        load_marker_map(yaml_path)


# --- MarkerLocalizationModule ----------------------------------------------


async def test_handle_color_image_no_markers_in_frame_skips_publish() -> None:
    module = MarkerLocalizationModule(marker_length_m=_MARKER_LENGTH_M, camera_info=camera_info())
    module._marker_map = _MARKER_MAP
    try:
        module.tf.publish(world_T_optical())
        await module.handle_color_image(blank_image(ts=10.0))
        assert module.tf.get("world", "map") is None
    finally:
        module.stop()


async def test_handle_color_image_unmapped_marker_skips_publish() -> None:
    """A detected tag whose id isn't in the loaded marker map is ignored cleanly."""
    module = MarkerLocalizationModule(marker_length_m=_MARKER_LENGTH_M, camera_info=camera_info())
    module._marker_map = {
        99: Transform(frame_id=MAP_FRAME, child_frame_id="marker_99")
    }  # id 7 not present
    try:
        module.tf.publish(world_T_optical())
        await module.handle_color_image(synthetic_marker_image(_MARKER_ID, ts=10.0))
        assert module.tf.get("world", "map") is None
    finally:
        module.stop()


async def test_handle_color_image_rejects_high_reprojection_error() -> None:
    """A corrupted detection (high reprojection error) makes the gate reject; publish is skipped."""
    module = MarkerLocalizationModule(marker_length_m=_MARKER_LENGTH_M, camera_info=camera_info())
    module._marker_map = _MARKER_MAP
    image = synthetic_marker_image(_MARKER_ID, ts=10.0)
    detections = detect_markers(
        image.to_grayscale().as_numpy(), create_aruco_detector("DICT_APRILTAG_36h11")
    )
    corrupted = [(detections[0][0], detections[0][1].copy())]
    corrupted[0][1][0] += (60.0, -45.0)  # yank one corner off -> inconsistent quad
    try:
        module.tf.publish(world_T_optical())
        with patch(
            "dimos.perception.fiducial.marker_localization_module.detect_markers",
            return_value=corrupted,
        ):
            await module.handle_color_image(image)
        assert module.tf.get("world", "map") is None
    finally:
        module.stop()


async def test_handle_color_image_without_camera_info_is_noop() -> None:
    """Static camera_info config path: no camera_info configured -> frames are ignored, not crash."""
    module = MarkerLocalizationModule(marker_length_m=_MARKER_LENGTH_M)  # camera_info left as None
    module._marker_map = _MARKER_MAP
    try:
        module.tf.publish(world_T_optical())
        await module.handle_color_image(synthetic_marker_image(_MARKER_ID, ts=10.0))
        assert module.tf.get("world", "map") is None
    finally:
        module.stop()


async def test_handle_color_image_without_marker_map_is_noop() -> None:
    """No marker map loaded (e.g. ``marker_map_file`` unset / ``start()`` never called) -> no-op."""
    module = MarkerLocalizationModule(marker_length_m=_MARKER_LENGTH_M, camera_info=camera_info())
    try:
        module.tf.publish(world_T_optical())
        await module.handle_color_image(synthetic_marker_image(_MARKER_ID, ts=10.0))
        assert module.tf.get("world", "map") is None
    finally:
        module.stop()


def test_module_config_rejects_non_positive_marker_length() -> None:
    with pytest.raises(ValidationError):
        MarkerLocalizationModule(marker_length_m=0.0)


def test_module_config_rejects_non_positive_reprojection_threshold() -> None:
    with pytest.raises(ValidationError):
        MarkerLocalizationModule(marker_length_m=_MARKER_LENGTH_M, max_reprojection_error_px=0.0)


def test_module_config_threads_min_tags_into_gate() -> None:
    """``min_tags`` is exposed on the module config and reaches the localization gate."""
    module = MarkerLocalizationModule(marker_length_m=_MARKER_LENGTH_M, min_tags=2)
    try:
        assert module._cfg.min_tags == 2
    finally:
        module.stop()
    with pytest.raises(ValidationError):
        MarkerLocalizationModule(marker_length_m=_MARKER_LENGTH_M, min_tags=0)


def test_module_config_threads_ambiguity_ratio_into_gate() -> None:
    """``ambiguity_ratio_min`` is exposed on the module config, reaches the localization
    gate, and rejects sub-1.0 values (a ratio below 1 is meaningless — the best candidate
    always beats itself)."""
    module = MarkerLocalizationModule(marker_length_m=_MARKER_LENGTH_M, ambiguity_ratio_min=3.0)
    try:
        assert module._cfg.ambiguity_ratio_min == 3.0
    finally:
        module.stop()
    with pytest.raises(ValidationError):
        MarkerLocalizationModule(marker_length_m=_MARKER_LENGTH_M, ambiguity_ratio_min=0.5)


async def test_module_uses_configured_camera_frame_and_warns_on_tf_miss() -> None:
    """A non-default ``camera_optical_frame`` is used for the TF lookup, and a lookup miss
    (mis-named frame, missing static chain) warns instead of going dark."""
    module = MarkerLocalizationModule(
        marker_length_m=_MARKER_LENGTH_M, camera_info=camera_info(), camera_optical_frame="cam_x"
    )
    module._marker_map = _MARKER_MAP
    try:
        module.tf.publish(Transform(frame_id="world", child_frame_id="cam_x", ts=10.0))
        await module.handle_color_image(synthetic_marker_image(_MARKER_ID, ts=10.0))
        assert module.tf.get("world", "map") is not None  # lookup used cam_x, not the default
    finally:
        module.stop()

    missing = MarkerLocalizationModule(marker_length_m=_MARKER_LENGTH_M, camera_info=camera_info())
    missing._marker_map = _MARKER_MAP
    try:  # no world -> camera_optical TF published at all -> warning, no publish
        with patch("dimos.perception.fiducial.marker_localization_module.logger.warning") as warned:
            await missing.handle_color_image(synthetic_marker_image(_MARKER_ID, ts=10.0))
        assert any("no TF" in str(c.args[0]) for c in warned.call_args_list)
        assert missing.tf.get("world", "map") is None
    finally:
        missing.stop()

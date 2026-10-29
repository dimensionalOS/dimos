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

from __future__ import annotations

from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest

from dimos.hardware.sensors.camera.stereo.backends import OpenCvStereo, VpiCudaStereo
from dimos.hardware.sensors.camera.stereo.calibration import StereoCalibration, metric_depth
from dimos.hardware.sensors.camera.stereo.module import (
    StereoDepth,
    StereoDepthConfig,
    StereoPair,
    StereoPairer,
)
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.galaxea.r1pro.blueprints.perception.r1pro_head_stereo import r1pro_head_stereo
from dimos.robot.galaxea.r1pro.head_cameras import HeadLeftCamera, HeadRightCamera


@pytest.fixture
def calibration_file(tmp_path):
    p = tmp_path / "stereo.yaml"
    f = cv2.FileStorage(str(p), cv2.FILE_STORAGE_WRITE)
    f.write("image_width", 128)
    f.write("image_height", 96)
    for side in ("left", "right"):
        f.write(f"K_{side}", np.array([[120.0, 0, 64], [0, 120, 48], [0, 0, 1]]))
        f.write(f"D_{side}", np.zeros((1, 14)))
    f.write("R", np.eye(3))
    f.write("T", np.array([[-0.1], [0.0], [0.0]]))
    f.release()
    return p


def compressed(side, ts):
    return CompressedImage(data=b"", format="png", frame_id=side, ts=ts)


def test_scaled_rectification_keeps_metric_baseline_and_updates_intrinsics(calibration_file):
    c = StereoCalibration.load(calibration_file, (64, 48))
    assert c.baseline_m == pytest.approx(0.1)
    assert c.projection[0, 0] == pytest.approx(60.0, rel=0.01)
    info = c.camera_info("rectified", 5.0)
    assert (info.width, info.height, info.frame_id, info.ts) == (64, 48, "rectified", 5.0)
    assert info.D == [0.0] * 5
    depth = metric_depth(
        np.full((48, 64), 6, np.float32), c.q, np.full((48, 64), 65535, np.uint16), 32767, 0.3, 3.0
    )
    assert np.allclose(depth, 1.0, atol=0.01)


def test_reprojection_handles_nonzero_principal_point_difference():
    q = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 100], [0, 0, 10, -20]], np.float64)
    d = np.array([[12, 7, 0, -1, np.nan]], np.float32)
    conf = np.array([[65535, 0, 65535, 65535, 65535]], np.uint16)
    assert metric_depth(d, q, conf, 32767, 0.3, 3.0).tolist() == [[1.0, 0.0, 0.0, 0.0, 0.0]]


def test_reprojection_rejects_out_of_range_or_nonfinite_depth():
    q = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 100], [0, 0, 10, 0]], np.float64)
    d = np.array([[1, 100, 10, np.inf]], np.float32)
    conf = np.full(d.shape, 65535, np.uint16)
    assert metric_depth(d, q, conf, 0, 0.3, 3.0).tolist() == [[0.0, 0.0, 1.0, 0.0]]


def test_calibration_requires_stereo_extrinsics(tmp_path):
    p = tmp_path / "invalid.yaml"
    f = cv2.FileStorage(str(p), cv2.FILE_STORAGE_WRITE)
    f.write("image_width", 128)
    f.write("image_height", 96)
    f.release()
    with pytest.raises(ValueError, match="K_left"):
        StereoCalibration.load(p, (64, 48))


def test_frame_size_mismatch_does_not_silently_scale_wrong_calibration(calibration_file):
    c = StereoCalibration.load(calibration_file, (64, 48))
    image = np.zeros((48, 64, 3), np.uint8)
    with pytest.raises(ValueError, match="match calibration"):
        c.rectify(image, image)


def test_translation_units_are_explicit(calibration_file):
    c = StereoCalibration.load(calibration_file, (64, 48), 0.001)
    assert c.baseline_m == pytest.approx(0.0001)


def test_pairer_rejects_skew_and_never_reuses_a_frame():
    p = StereoPairer("left", "right", 0.002)
    assert p.push("left", compressed("left", 1.0)) is None
    assert p.push("right", compressed("right", 1.03)) is None
    pair = p.push("left", compressed("left", 1.031))
    assert pair.left.ts == 1.031 and pair.right.ts == 1.03
    assert p.push("left", compressed("left", 2.0)) is None


@pytest.mark.parametrize(
    "image", [compressed("wrong", 1), compressed("left", float("nan")), compressed("left", 1)]
)
def test_pairer_drops_wrong_frames_nan_or_duplicate_timestamps(image):
    p = StereoPairer("left", "right", 0.002)
    p.push("left", compressed("left", 1))
    assert p.push("left", image) is None
    assert p.dropped == 1


def test_sgbm_reference_recovers_synthetic_pixel_shift():
    left = np.random.default_rng(1).integers(0, 255, (64, 160), dtype=np.uint8)
    right = np.zeros_like(left)
    right[:, :-8] = left[:, 8:]
    d, c = OpenCvStereo(16).estimate(left, right)
    interior = d[8:-8, 24:-16]
    assert float(np.median(interior)) == pytest.approx(8.0, abs=0.25)
    assert np.mean(c[8:-8, 24:-16] > 0) > 0.8


@pytest.fixture
def processor(calibration_file, mocker):
    now = [100.1]
    mocker.patch(
        "dimos.hardware.sensors.camera.stereo.module.time.time", side_effect=lambda: now[0]
    )
    module = StereoDepth(
        calibration_path=str(calibration_file),
        left_frame="left",
        right_frame="right",
        width=64,
        height=48,
        max_disparity=16,
        backend="opencv_sgbm",
    )
    module._calibration = StereoCalibration.load(calibration_file, (64, 48))
    backend = MagicMock()
    backend.estimate.return_value = (
        np.full((48, 64), 6, np.float32),
        np.full((48, 64), 65535, np.uint16),
    )
    module._backend = backend
    outputs = {
        name: mocker.patch.object(getattr(module, name), "publish")
        for name in ("color_image", "depth_image", "camera_info", "confidence")
    }
    rgb = np.zeros((96, 128, 3), np.uint8)
    pair = StereoPair(
        CompressedImage.from_image(
            Image(rgb, ImageFormat.RGB, frame_id="left", ts=100.0), format="png"
        ),
        CompressedImage.from_image(
            Image(rgb, ImageFormat.RGB, frame_id="right", ts=100.001), format="png"
        ),
    )
    try:
        yield module, backend, outputs, pair, now
    finally:
        module.stop()


def test_outputs_share_one_rectified_frame_stamp_and_metric_depth(processor):
    module, _, outputs, pair, _ = processor
    assert module._process_pair(pair)
    for output in outputs.values():
        output.assert_called_once()
        msg = output.call_args.args[0]
        assert (msg.ts, msg.frame_id) == (100.0, "stereo_left_rectified_optical")
    depth = outputs["depth_image"].call_args.args[0]
    assert depth.format == ImageFormat.DEPTH and np.allclose(depth.data, 1.0, atol=0.01)
    assert module.get_status()["robot_mount_verified"] is False


def test_stale_pair_is_dropped_before_decode_or_backend(processor):
    module, backend, outputs, pair, now = processor
    now[0] = 101.0
    assert module._process_pair(pair) is False
    backend.estimate.assert_not_called()
    assert module.get_status()["expired_pairs"] == 1
    assert module.get_status()["backpressure_replacements"] == 0
    for output in outputs.values():
        output.assert_not_called()


def test_processing_delay_never_publishes_stale_output(processor):
    module, backend, outputs, pair, now = processor
    value = backend.estimate.return_value

    def slow(left, right):
        now[0] = 101.0
        return value

    backend.estimate.side_effect = slow
    assert module._process_pair(pair) is False
    for output in outputs.values():
        output.assert_not_called()


def test_stop_never_publishes_or_accepts_pending_frames(processor):
    module, _, outputs, pair, _ = processor
    module.stop()
    module._receive("left", pair.left)
    module._receive("right", pair.right)
    assert module._pending is None
    for output in outputs.values():
        output.assert_not_called()


def test_backpressure_keeps_only_latest_complete_pair(processor):
    module, _, _, pair, _ = processor
    module._receive("left", pair.left)
    module._receive("right", pair.right)
    module._receive("left", compressed("left", 101.0))
    module._receive("right", compressed("right", 101.001))
    assert module._pending.left.ts == 101.0
    assert module.get_status()["dropped_pairs"] == 1
    assert module.get_status()["backpressure_replacements"] == 1
    assert module.get_status()["expired_pairs"] == 0


@pytest.mark.parametrize(
    "extra", [{"max_disparity": 17}, {"width": 128}, {"max_depth_m": 0.1}, {"right_frame": "left"}]
)
def test_invalid_configuration_is_rejected(extra):
    data = dict(calibration_path="/calibration.yaml", left_frame="left", right_frame="right")
    data.update(extra)
    with pytest.raises(ValueError):
        StereoDepthConfig(**data)


def test_vpi_fixed_point_disparity_and_confidence_are_not_visualization_scaled(mocker):
    vpi = MagicMock()
    vpi.stereodisp.return_value.cpu.return_value = np.array([[320, -32]], np.int16)
    vpi.Image.return_value.cpu.return_value = np.array([[65535, 0]], np.uint16)
    mocker.patch(
        "dimos.hardware.sensors.camera.stereo.backends.importlib.import_module", return_value=vpi
    )
    d, c = VpiCudaStereo(256, 32767).estimate(
        np.zeros((1, 2), np.uint8), np.zeros((1, 2), np.uint8)
    )
    assert d.tolist() == [[10.0, -1.0]]
    assert c.tolist() == [[65535, 0]]
    assert vpi.stereodisp.call_args.kwargs["maxdisp"] == 256
    assert vpi.stereodisp.call_args.kwargs["out_confmap"] is vpi.Image.return_value


def test_head_blueprint_contains_no_robot_control():
    assert {b.module for b in r1pro_head_stereo.blueprints} == {
        HeadLeftCamera,
        HeadRightCamera,
        StereoDepth,
    }


def test_output_frame_must_not_mislabel_rectified_data_as_raw():
    with pytest.raises(ValueError):
        StereoDepthConfig(
            calibration_path="/calibration.yaml",
            left_frame="left",
            right_frame="right",
            frame_id="left",
        )


def test_idle_worker_lifecycle_never_initializes_camera_capture(processor, mocker):
    module, _, outputs, _, _ = processor
    mocker.patch.object(module.left, "subscribe", return_value=lambda: None)
    mocker.patch.object(module.right, "subscribe", return_value=lambda: None)
    module.start()
    thread = module._thread
    assert thread is not None and thread.is_alive()
    module.stop()
    assert not thread.is_alive()
    for output in outputs.values():
        output.assert_not_called()


def test_depth_sample_does_not_reuse_stale_or_stopped_data(processor):
    module, _, _, pair, now = processor
    assert module.sample_depth(0, 0)["available"] is False
    assert module._process_pair(pair)
    sample = module.sample_depth(20, 20)
    assert sample["available"] and sample["depth_m"] == pytest.approx(1.0, abs=0.01)
    assert sample["valid_pixels"] == 25
    now[0] = 101.0
    assert module.sample_depth(20, 20)["available"] is False
    module.stop()
    assert module.sample_depth(20, 20)["available"] is False

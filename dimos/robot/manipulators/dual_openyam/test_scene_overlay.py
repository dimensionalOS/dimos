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

from collections.abc import Iterator
from contextlib import contextmanager
import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.manipulators.dual_openyam.scene_overlay import (
    SceneOverlayModule,
    alignment_scores,
    blend,
    edge_overlay,
    fit_reference,
    foreground_mask,
    scene_paths,
    segment_objects,
)


def _frame(value: int, width: int = 64, height: int = 48) -> Image:
    data = np.full((height, width, 3), value, dtype=np.uint8)
    return Image(data=data, format=ImageFormat.BGR, frame_id="camera_link", ts=1.0)


@contextmanager
def _module(tmp_path: Path, **overrides: Any) -> Iterator[SceneOverlayModule]:
    config: dict[str, Any] = {"scenes_dir": tmp_path, "scene": "s1", "max_fps": 0.0, "label": False}
    config.update(overrides)
    module = SceneOverlayModule(**config)
    try:
        yield module
    finally:
        module.stop()


def test_scene_paths_reject_names_that_leave_the_directory(tmp_path: Path) -> None:
    image_path, meta_path = scene_paths(tmp_path, "scene-01")
    assert image_path == tmp_path / "scene-01.png"
    assert meta_path == tmp_path / "scene-01.json"
    for bad in ("../x", "a/b", "", ".hidden"):
        with pytest.raises(ValueError):
            scene_paths(tmp_path, bad)


def test_blend_mixes_by_opacity() -> None:
    live = np.full((2, 2, 3), 0, dtype=np.uint8)
    reference = np.full((2, 2, 3), 200, dtype=np.uint8)
    assert blend(live, reference, 0.5)[0, 0, 0] == 100
    assert blend(live, reference, 0.0)[0, 0, 0] == 0
    assert blend(live, reference, 1.0)[0, 0, 0] == 200


def test_edge_overlay_marks_reference_edges_in_green() -> None:
    live = np.zeros((40, 40, 3), dtype=np.uint8)
    reference = np.zeros((40, 40, 3), dtype=np.uint8)
    reference[10:30, 10:30] = 255
    out = edge_overlay(live, reference)
    assert (out == (0, 255, 0)).all(axis=2).any()
    assert not out[0, 0].any()


def test_fit_reference_resizes_only_on_mismatch() -> None:
    reference = np.zeros((10, 20, 3), dtype=np.uint8)
    assert fit_reference(reference, (10, 20)) is reference
    assert fit_reference(reference, (5, 8)).shape == (5, 8, 3)


def test_without_reference_the_live_frame_passes_through(tmp_path: Path, mocker: Any) -> None:
    with _module(tmp_path) as module:
        publish = mocker.patch.object(module.overlay_image, "publish")

        module._on_frame(_frame(10))

        published = publish.call_args.args[0]
        assert isinstance(published, Image)
        assert published.frame_id == "camera_link" and published.ts == 1.0
        assert (published.to_opencv() == 10).all()
        assert module.get_status()["has_reference"] is False


def test_capture_reference_writes_files_and_blends_following_frames(
    tmp_path: Path, mocker: Any
) -> None:
    with _module(tmp_path, opacity=0.5) as module:
        publish = mocker.patch.object(module.overlay_image, "publish")
        module._on_frame(_frame(200))

        path = module.capture_reference("scene-02")

        assert Path(path) == tmp_path / "scene-02.png"
        meta = json.loads((tmp_path / "scene-02.json").read_text())
        assert meta["scene"] == "scene-02" and meta["width"] == 64 and meta["height"] == 48
        assert module.list_scenes() == ["scene-02"]
        status = module.get_status()
        assert status["scene"] == "scene-02" and status["has_reference"] is True

        module._on_frame(_frame(0))
        assert (publish.call_args.args[0].to_opencv() == 100).all()


def test_capture_without_a_frame_is_an_error(tmp_path: Path) -> None:
    with _module(tmp_path) as module, pytest.raises(RuntimeError, match="No camera frame"):
        module.capture_reference()


def test_set_scene_loads_a_saved_reference_and_reports_missing_ones(
    tmp_path: Path, mocker: Any
) -> None:
    with _module(tmp_path) as module:
        mocker.patch.object(module.overlay_image, "publish")
        module._on_frame(_frame(50))
        module.capture_reference("a")

    with _module(tmp_path, scene="a") as fresh:
        assert fresh.get_status()["has_reference"] is True
        assert fresh.set_scene("missing") is False
        status = fresh.get_status()
        assert status["scene"] == "missing" and status["has_reference"] is False
        assert fresh.set_scene("a") is True


def test_reference_of_another_size_is_resized(tmp_path: Path, mocker: Any) -> None:
    with _module(tmp_path) as module:
        publish = mocker.patch.object(module.overlay_image, "publish")
        module._on_frame(_frame(200, width=32, height=24))
        module.capture_reference()

        module._on_frame(_frame(0))

        assert publish.call_args.args[0].to_opencv().shape == (48, 64, 3)


def test_edges_mode_and_opacity_rpcs(tmp_path: Path, mocker: Any) -> None:
    with _module(tmp_path) as module:
        publish = mocker.patch.object(module.overlay_image, "publish")
        module._on_frame(_frame(200))
        module.capture_reference()

        assert module.set_opacity(0.25) == 0.25
        module._on_frame(_frame(0))
        assert (publish.call_args.args[0].to_opencv() == 50).all()

        assert module.set_mode("edges") == "edges"
        module._on_frame(_frame(0))
        assert (publish.call_args.args[0].to_opencv() == 0).all()

        with pytest.raises(ValueError):
            module.set_opacity(1.5)
        with pytest.raises(ValueError):
            module.set_mode("solid")  # type: ignore[arg-type]


def test_max_fps_drops_frames_but_keeps_the_latest(tmp_path: Path, mocker: Any) -> None:
    with _module(tmp_path, max_fps=1.0) as module:
        publish = mocker.patch.object(module.overlay_image, "publish")

        module._on_frame(_frame(1))
        module._on_frame(_frame(2))

        publish.assert_called_once()
        assert module.get_status()["frames"] == 2
        assert module._latest is not None and (module._latest.data == 2).all()


def _table(*squares: tuple[int, int], width: int = 160, height: int = 120) -> Image:
    """A grey table with 30 px white squares at the given top-left corners."""
    data = np.full((height, width, 3), 90, dtype=np.uint8)
    for x, y in squares:
        data[y : y + 30, x : x + 30] = 255
    return Image(data=data, format=ImageFormat.BGR, frame_id="camera_link", ts=1.0)


def test_objects_are_segmented_against_the_empty_table_and_scored() -> None:
    background = _table().to_opencv()
    reference = _table((10, 10), (100, 60)).to_opencv()
    masks = segment_objects(foreground_mask(reference, background))
    assert len(masks) == 2

    same = alignment_scores(foreground_mask(reference, background), masks)
    assert all(score > 0.9 for score in same)

    shifted = _table((25, 10), (100, 60)).to_opencv()
    scores = alignment_scores(foreground_mask(shifted, background), masks)
    assert 0.1 < scores[0] < 0.5 and scores[1] > 0.9

    missing = _table((100, 60)).to_opencv()
    assert alignment_scores(foreground_mask(missing, background), masks)[0] == 0.0


def test_background_capture_enables_alignment_in_the_stream(tmp_path: Path, mocker: Any) -> None:
    with _module(tmp_path, label=True) as module:
        publish = mocker.patch.object(module.overlay_image, "publish")
        module._on_frame(_table())
        assert Path(module.capture_background()) == tmp_path / "background.png"
        module._on_frame(_table((10, 10), (100, 60)))
        module.capture_reference("scene-01")
        assert module.get_status()["objects"] == 2

        module._on_frame(_table((25, 10), (100, 60)))
        alignment = module.get_alignment()
        assert alignment["objects"] == 2 and 0.1 < alignment["scores"][0] < 0.5
        assert alignment["scores"][1] > 0.9 and alignment["mean"] is not None
        assert publish.call_args.args[0].to_opencv().shape == (120, 160, 3)

    with _module(tmp_path, scene="scene-01") as fresh:
        status = fresh.get_status()
        assert status["has_background"] is True and status["objects"] == 2


def test_no_scene_selected_passes_the_live_frame_through(tmp_path: Path, mocker: Any) -> None:
    with _module(tmp_path, scene="", label=True) as module:
        publish = mocker.patch.object(module.overlay_image, "publish")
        module._on_frame(_frame(10))
        status = module.get_status()
        assert status["scene"] == "" and status["has_reference"] is False
        assert publish.call_args.args[0].to_opencv().shape == (48, 64, 3)
        with pytest.raises(ValueError, match="No scene selected"):
            module.capture_reference()
        assert module.set_scene("") is False

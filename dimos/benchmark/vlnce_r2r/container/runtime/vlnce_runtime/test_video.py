# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

import importlib
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import numpy as np

visualization_utils = ModuleType("habitat.utils.visualizations.utils")
visualization_utils.images_to_video = lambda *args, **kwargs: None
visualization_utils.observations_to_image = lambda *args, **kwargs: None
with patch.dict(
    sys.modules,
    {
        "habitat": ModuleType("habitat"),
        "habitat.utils": ModuleType("habitat.utils"),
        "habitat.utils.visualizations": ModuleType("habitat.utils.visualizations"),
        "habitat.utils.visualizations.utils": visualization_utils,
    },
):
    video = importlib.import_module(__package__ + ".video")


def test_initial_capture_omits_uninitialized_top_down_map(monkeypatch) -> None:
    captured_info = []

    def observations_to_image(observations, info):
        captured_info.append(info)
        if "top_down_map" in info:
            info["top_down_map"]["map"]
        return observations["rgb"]

    monkeypatch.setattr(video, "observations_to_image", observations_to_image)
    environment = SimpleNamespace(
        observations={
            "rgb": np.zeros((224, 224, 3), dtype=np.uint8),
            "depth": np.zeros((224, 224, 1), dtype=np.float32),
        },
        visualization_info=lambda: {"success": 0.0, "top_down_map": None},
    )
    renderer = video.NativeEpisodeVideo("unused.mp4", "unused.json", environment)

    renderer.capture_initial()

    assert renderer.diagnostic is None
    assert len(renderer.frames) == video.FPS
    assert captured_info == [{"success": 0.0}]


def test_close_pads_reset_frames_to_map_frame_size(tmp_path, monkeypatch) -> None:
    visualization_info = iter(
        [
            {"top_down_map": None},
            {"top_down_map": {"map": np.zeros((2, 2), dtype=np.uint8)}},
        ]
    )

    def observations_to_image(_observations, info):
        width = 672 if "top_down_map" in info else 448
        return np.zeros((224, width, 3), dtype=np.uint8)

    def images_to_video(frames, directory, name, fps):
        assert {frame.shape for frame in frames} == {(224, 672, 3)}
        assert fps == video.FPS
        Path(directory, name + ".mp4").write_bytes(b"video")

    monkeypatch.setattr(video, "observations_to_image", observations_to_image)
    monkeypatch.setattr(video, "images_to_video", images_to_video)
    environment = SimpleNamespace(
        observations={
            "rgb": np.zeros((224, 224, 3), dtype=np.uint8),
            "depth": np.zeros((224, 224, 1), dtype=np.float32),
        },
        visualization_info=lambda: next(visualization_info),
    )
    output_path = tmp_path / "native-render.mp4"
    metadata_path = tmp_path / "native-render.v1.json"
    renderer = video.NativeEpisodeVideo(output_path, metadata_path, environment)

    renderer.capture_initial()
    renderer.capture_control()
    renderer.close()

    assert output_path.read_bytes() == b"video"
    assert json.loads(metadata_path.read_text())["status"] == "completed"

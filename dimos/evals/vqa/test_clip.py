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

from pathlib import Path

import numpy as np
import pytest

from dimos.evals.environments.image_clip import ImageClip
from dimos.evals.vqa.clip import (
    dense_clip_around,
    ego_memory_clip,
    format_clip_span,
    write_image_clip,
)
from dimos.evals.vqa.generate import PublicCase
from dimos.evals.vqa.suite import load_suite
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def _img(ts: float) -> Image:
    rgb = np.full((8, 8, 3), 30, dtype=np.uint8)
    return Image.from_numpy(rgb, format=ImageFormat.RGB, frame_id="cam", ts=ts)


def test_ego_memory_band_and_balance() -> None:
    from dimos.evals.vqa.generate import (
        PrivateLabel,
        PublicCase,
        _balance_ego_memory_bands,
        _ego_memory_band,
    )

    assert _ego_memory_band("ego_fore_aft", "behind") == "hard"
    assert _ego_memory_band("ego_fore_aft", "ahead") == "soft"
    assert _ego_memory_band("ego_turn", "left") == "hard"
    assert _ego_memory_band("ego_turn", "ahead") == "soft"
    assert _ego_memory_band("ego_any_fore_aft", "yes") == "soft"
    assert _ego_memory_band("ego_any_fore_aft", "no") == "soft"

    def _pair(i: int, band: str) -> tuple[PublicCase, PrivateLabel]:
        case = PublicCase(
            id=f"c-{band}-{i}",
            clip=f"assets/clips/c-{band}-{i}",
            question="q?",
            choices=("a", "b"),
        )
        return case, PrivateLabel(id=case.id, answer="a")

    hard = [_pair(i, "hard") for i in range(2)]
    soft = [_pair(i, "soft") for i in range(5)]
    balanced = _balance_ego_memory_bands(hard, soft)
    assert len(balanced) == 4  # 1:1 → 2+2
    assert {p[0].id.split("-")[1] for p in balanced} == {"hard", "soft"}
    assert _balance_ego_memory_bands([], soft) == soft
    assert _balance_ego_memory_bands(hard, []) == hard


def test_public_case_requires_exactly_one_modality() -> None:
    with pytest.raises(ValueError, match="exactly one"):
        PublicCase(id="x", question="q?", choices=("a", "b"))
    with pytest.raises(ValueError, match="exactly one"):
        PublicCase(
            id="x",
            question="q?",
            choices=("a", "b"),
            image="assets/a.png",
            clip="assets/clips/x",
        )
    with pytest.raises(ValueError, match="at least two"):
        PublicCase(id="x", question="q?", choices=("a", "b"), clips=("assets/clips/a",))


def test_ego_memory_clip_spans_first_spot_to_query() -> None:
    frames = [(_img(t).ts, _img(t)) for t in (1.0, 2.0, 3.0, 4.0, 5.0)]
    # Rebuild with consistent images
    frames = [(float(t), _img(float(t))) for t in (1.0, 2.0, 3.0, 4.0, 5.0)]
    picked, t0 = ego_memory_clip(frames, first_spot_ts=2.0, query_ts=5.0, max_frames=8)
    assert t0 == 2.0
    assert [ts for ts, _ in picked] == [2.0, 3.0, 4.0, 5.0]


def test_ego_memory_clip_keeps_anchors_when_truncated() -> None:
    frames = [(float(t), _img(float(t))) for t in range(0, 20)]
    picked, _t0 = ego_memory_clip(frames, first_spot_ts=0.0, query_ts=19.0, max_frames=5)
    assert len(picked) == 5
    assert picked[0][0] == 0.0
    assert picked[-1][0] == 19.0


def test_dense_clip_and_image_clip_env(tmp_path: Path) -> None:
    window = [(float(t), _img(float(t))) for t in (10.0, 10.5, 11.0, 11.5, 12.0)]
    picked, _ = dense_clip_around(window, 11.0, radius_s=1.0, max_frames=8)
    clip_dir = tmp_path / "clip"
    manifest = write_image_clip(clip_dir, picked, anchors=(11.0,))
    assert manifest.duration_s == pytest.approx(picked[-1][0] - picked[0][0])
    assert format_clip_span(manifest.duration_s).startswith("t=0.0s to t=")

    env = ImageClip(clip_dir)
    running = env.start(())
    obs = list(running.streams[0])
    assert len(obs) == len(picked)
    env.stop()


def test_load_suite_wires_clips(tmp_path: Path) -> None:
    window = [(float(t), _img(float(t))) for t in (1.0, 1.5, 2.0)]
    first = tmp_path / "assets" / "clips" / "c" / "first"
    second = tmp_path / "assets" / "clips" / "c" / "second"
    write_image_clip(first, window[:2], anchors=(1.0,))
    write_image_clip(second, window[1:], anchors=(2.0,))
    case = PublicCase(
        id="c",
        clips=("assets/clips/c/first", "assets/clips/c/second"),
        question="Stream first_clip covers t=0.0s to t=0.5s. Same object?",
        choices=("yes", "no"),
    )
    (tmp_path / "cases.jsonl").write_text(
        case.model_dump_json(exclude_none=True) + "\n", encoding="utf-8"
    )
    (tmp_path / "labels.jsonl").write_text(
        '{"id":"c","answer":"yes"}\n', encoding="utf-8"
    )
    suite = load_suite(tmp_path)
    assert len(suite) == 1
    assert isinstance(suite[0].environment, ImageClip)
    assert len(suite[0].environment.config.paths) == 2

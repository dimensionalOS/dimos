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

from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.perception.localize.localize import localize
from dimos.perception.localize.types import LocalizeParams


def test_localize_releases_embedder_after_retrieve(monkeypatch: pytest.MonkeyPatch) -> None:
    embedder = MagicMock()
    embedder.stop = MagicMock()
    detector = MagicMock()
    segmenter = MagicMock()

    monkeypatch.setattr(
        "dimos.perception.localize.localize.retrieve",
        lambda *args, **kwargs: MagicMock(name="candidates"),
    )
    monkeypatch.setattr(
        "dimos.perception.localize.localize.support_planes",
        lambda *args, **kwargs: {},
    )

    def fake_detect(*args: Any, **kwargs: Any) -> MagicMock:
        stream = MagicMock()
        return stream

    monkeypatch.setattr("dimos.perception.localize.localize.detect", fake_detect)

    def fake_segment(detections: Any, _segmenter: Any) -> Any:
        return detections

    monkeypatch.setattr("dimos.perception.localize.localize.segment", fake_segment)

    associate_stream = MagicMock()
    associate_stream.drain = MagicMock()
    monkeypatch.setattr(
        "dimos.perception.localize.localize.associate",
        lambda *args, **kwargs: associate_stream,
    )
    monkeypatch.setattr(
        "dimos.perception.localize.localize._finalize",
        lambda *args, **kwargs: [],
    )

    localize(
        ["chair"],
        index=MagicMock(),
        rig=MagicMock(),
        embedder=embedder,
        detector=detector,
        segmenter=segmenter,
        params=LocalizeParams(release_embedder_after_retrieve=True),
    )

    embedder.stop.assert_called_once_with()

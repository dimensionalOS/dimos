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

"""BoT-SORT with camera-motion compensation. No FastReID."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import replace
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox


class Mot(Protocol):
    def update(
        self, detections: Sequence[Detection2DBBox], frame_bgr: NDArray[np.uint8]
    ) -> list[Detection2DBBox]: ...


def assign_track_ids(
    detections: Sequence[Detection2DBBox],
    tracker_ids: Sequence[int],
    input_index: Sequence[int] | None = None,
) -> list[Detection2DBBox]:
    """Copy *detections* with BoT-SORT ids. Drops rows whose id is negative."""
    out: list[Detection2DBBox] = []
    for i, track_id in enumerate(tracker_ids):
        ident = int(track_id)
        if ident < 0:
            continue
        if input_index is None:
            src_i = i
        else:
            src_i = int(input_index[i])
        if src_i < 0 or src_i >= len(detections):
            continue
        out.append(replace(detections[src_i], track_id=ident))
    return out


class BotSortMot:
    """Apache BoT-SORT (roboflow/trackers). CMC on, package ReID off."""

    def __init__(self, tracker: Any | None = None) -> None:
        self._make_tracker = None
        if tracker is not None:
            self._tracker = tracker
            return
        try:
            from trackers import BoTSORTTracker
        except ImportError as exc:
            raise ImportError(
                "BoT-SORT needs the Apache `trackers` package (`pip install trackers`). "
                "Do not use ultralytics track mode."
            ) from exc

        def make_tracker() -> Any:
            return BoTSORTTracker(enable_cmc=True)

        self._make_tracker = make_tracker
        self._tracker = self._make_tracker()

    def reset(self) -> None:
        if self._make_tracker is not None:
            self._tracker = self._make_tracker()
            return
        reset = getattr(self._tracker, "reset", None)
        if callable(reset):
            reset()

    def update(
        self, detections: Sequence[Detection2DBBox], frame_bgr: NDArray[np.uint8]
    ) -> list[Detection2DBBox]:
        import supervision as sv

        if not detections:
            self._tracker.update(sv.Detections.empty(), frame=frame_bgr)
            return []
        xyxy = np.asarray([det.bbox for det in detections], dtype=np.float32)
        confidence = np.asarray([det.confidence for det in detections], dtype=np.float32)
        class_id = np.asarray([det.class_id for det in detections], dtype=np.int32)
        names = np.asarray([det.name for det in detections])
        tracked = self._tracker.update(
            sv.Detections(
                xyxy=xyxy,
                confidence=confidence,
                class_id=class_id,
                data={
                    "class_name": names,
                    "input_index": np.arange(len(detections), dtype=np.int32),
                },
            ),
            frame=frame_bgr,
        )
        ids = tracked.tracker_id
        if ids is None or len(tracked) == 0:
            return []
        index = tracked.data.get("input_index")
        input_index = None if index is None else [int(v) for v in index]
        return assign_track_ids(detections, [int(v) for v in ids], input_index)

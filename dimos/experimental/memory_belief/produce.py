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

"""Frames in, sightings out. The half of the pipeline a live robot needs.

:mod:`~dimos.experimental.memory_belief.pipeline` folds sightings into views;
this produces the sightings. Both take iterators and never materialise them, so
the same call works against a recording that ends and a robot that does not --
which is what keeps a live producer, when there is one, from being a second copy
that drifts from this one without anything raising.

The drift here is quieter than in the folds. Two runs that used different
placement thresholds produce records that look identical -- same fields, same
types -- and differ only in which detections got a position at all. Nothing
downstream can tell, so :class:`DetectParams` carries every value that changes
what a record *means*, separately from the values that only change how long a
run takes.

What is deliberately not here: opening stores, choosing an output path, printing
progress, and building the detector. Those differ between a batch job and a
robot, and none of them changes a record.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.experimental.memory_belief.detect import bright_enough, detect_to_belief
from dimos.experimental.memory_belief.locate import locate_detections
from dimos.msgs.geometry_msgs.Transform import Transform

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable, Iterator

    from dimos.experimental.memory_belief.types import BeliefObservation


@dataclass(frozen=True, slots=True)
class DetectParams:
    """Everything that changes what a sighting means.

    The counterpart to :class:`~dimos.experimental.memory_belief.pipeline.ViewParams`,
    and split from the run's own settings on the same test: a field belongs here
    if two stores built with different values cannot be compared. ``stride`` and
    ``limit`` are not here -- skipping frames yields fewer records, not different
    ones.
    """

    #: Detections below this are dropped rather than stored with a low score: a
    #: record that exists at all is evidence downstream.
    min_confidence: float = 0.0
    #: Mean pixel value under which a frame is treated as too dark to detect in.
    #: A dark frame yields no records, and that absence is indistinguishable
    #: from an empty room unless the threshold travels with the data.
    min_brightness: float = 40.0
    #: Lidar returns needed inside a box before it earns a position. Lower it
    #: and more detections get placed, less reliably -- the single knob that
    #: most changes what a store can answer.
    min_points: int = 8
    #: Depth inlier band, metres. Returns outside it are background seen through
    #: or around the object.
    depth_band_m: float = 0.6
    #: Lattice square size for ``place_ref`` when a detection is placed.
    place_size_m: float = 5.0


def detect_stream(
    frames: Iterable[Any],
    detector: Any,
    *,
    stream_name: str,
    vocabulary: tuple[str, ...] | None,
    source: str,
    params: DetectParams | None = None,
    camera: Any = None,
    stride: int = 1,
    limit: int | None = None,
    on_skip: Callable[[], None] | None = None,
) -> Iterator[BeliefObservation]:
    """Run ``detector`` over ``frames``, yielding one belief record per detection.

    ``frames`` is consumed as an iterator and never materialised, so the same
    call works against a live robot where the stream does not end. When
    ``camera`` is given the frames are expected to be image/scan pairs, as
    produced by aligning a camera stream against lidar; without it they are bare
    image observations and every record is written with no position.

    The pose carried by each frame is used as the camera's world pose directly,
    not composed with a mount transform. A recorded frame's pose is resolved by
    the recorder as ``world <- frame_id``, and the go2 stamps its images with
    ``camera_optical`` -- so the mount is already in it. Composing again applied
    the optical rotation and the 0.3 m offset twice, which put every detection
    in the map as though the camera had been yawed 90 degrees.

    ``vocabulary`` is recorded verbatim on every record, which is what lets a
    later question about an unlisted term answer OUT_OF_VOCABULARY rather than
    "no". ``on_skip`` is called once per frame dropped for darkness, because a
    caller that cannot see that count cannot tell an unlit corridor from an
    empty one.
    """
    params = params or DetectParams()
    processed = 0
    # One slot, not a dictionary keyed by frame: the locator runs on the frame
    # that was just yielded and never on an older one, so nothing here grows
    # with the length of the stream.
    pending: dict[str, Any] = {"points": None, "pose": None}

    def _selected(source_frames: Iterable[Any]) -> Iterator[Any]:
        """Stride and limit by counting. Slicing would need a length."""
        nonlocal processed
        for i, item in enumerate(source_frames):
            if i % stride:
                continue
            if limit is not None and processed >= limit:
                return
            processed += 1
            if camera is None:
                yield item
                continue
            image_obs, scan_obs = item.data
            pending["points"] = scan_obs.data.points_f32()
            pending["pose"] = getattr(image_obs, "pose", None)
            yield image_obs

    def _locate(_observation: Any, detections: Any) -> Any:
        points, pose = pending["points"], pending["pose"]
        if pose is None or points is None or not len(points):
            return [None] * len(detections)
        return locate_detections(
            detections,
            np.asarray(points).reshape(-1, 3),
            # The frame's own pose, as a transform. Not composed with a camera
            # mount: the recorder already resolved this pose as
            # `world <- camera_optical`, so the mount is in it.
            world_from_camera=Transform.from_pose("world", pose),
            camera=camera,
            min_points=params.min_points,
            depth_band_m=params.depth_band_m,
        )

    def _lit(observation: Any) -> bool:
        keep = bright_enough(observation, min_mean=params.min_brightness)
        if not keep and on_skip is not None:
            on_skip()
        return keep

    yield from detect_to_belief(
        _selected(frames),
        detector,
        stream_name=stream_name,
        vocabulary=vocabulary,
        source=source,
        min_confidence=params.min_confidence,
        place_size_m=params.place_size_m,
        frame_filter=_lit,
        locate=None if camera is None else _locate,
    )

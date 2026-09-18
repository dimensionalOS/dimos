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

"""Frames first, geometry second: one good image per episode, handed to a detector.

The patches rank *frames*, which is the thing they are actually good at, and an
open-vocabulary detector does the finding. Each episode's best frame goes to OWLv2 with
the query words; its 2D box plus that frame's full depth image become a 3D box.

What this removes, compared with placing every hot patch in the world and hoping the
pyramids overlap: no overlap margin to tune, no group score to invent, and no
requirement that per-patch depth be accurate -- the box comes from the detector and the
depth image, not from a pyramid. Cross-model agreement survives as *which models voted
for this episode*, which needs nothing to coincide in space.

Results are yielded per episode, as they are found. A detector frame costs the better
part of a second, so a caller that waits for the whole list waits for all of them.
"""

from __future__ import annotations

from collections.abc import Iterator, Sequence
from dataclasses import dataclass, field, replace
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.mapping.hyperspace import patches as hs
from dimos.mapping.hyperspace.flextf import FlexTf
from dimos.mapping.hyperspace.frames import Episode, Frame
from dimos.mapping.hyperspace.ingest import (
    TF_STREAM,
    decoded,
    filled_stream_for,
    frame_stream_for,
    info_stream_for,
    intrinsics_of,
)
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()

OWLV2_CHECKPOINT = "google/owlv2-base-patch16-ensemble"


@dataclass
class DetectConfig:
    """Everything the detector path can be turned by, in one place."""

    # OWLv2's own per-box acceptance score. Its scores are calibrated, so this is a real
    # refusal threshold rather than a ranking cut, and half is Jeff's call (2026-09-13):
    # the runs at 0.15 answered with boxes down at 0.16-0.23 that were the detector
    # reaching rather than finding, and every one of them cost a place. On grocery and
    # sf_office it left the two real cones, both real whiteboards and six of seven real
    # baskets, and it dropped the AprilTag board that used to win "a whiteboard".
    #
    # KNOWN BLIND SPOT, measured and accepted rather than discovered later: OWLv2 does
    # not score two words on the same scale. A green plastic basket comes back at 0.66
    # and a real loaf of bread at 0.42, so at half "bread" on grocery answers nothing at
    # all while its correct answers sit just under the line. ("cheese" also answers
    # nothing, but that one costs no recall -- its best was 0.35 on a table of packaged
    # bread.) One global number cannot be right for both; per-word calibration is the
    # fix if the blind spot ever matters more than the quiet.
    threshold: float = 0.5
    checkpoint: str = OWLV2_CHECKPOINT
    device: str = ""
    # Frames of one episode to try before calling it undetected. The peak frame is
    # usually right, but an object can be half out of the frame at the moment it scores
    # highest, and the next look is free of that.
    attempts: int = 3
    # Boxes to keep from one photograph. The detector returns every box over its
    # threshold and a frame really can hold two of the thing asked for; past a handful
    # they are the same shelf seen as several.
    per_frame: int = 4
    # Frames handed to the detector in one forward pass. One, because batching was
    # measured on this Mac and LOST: OWLv2 pads every frame to 960x960, so the cost is
    # per pixel and there is little per-call overhead to amortize, while the bigger
    # activation tensor pushes MPS around -- 275 ms a frame at 1, 300 at 4, 514 at 12.
    # MEASURED ON CUDA TOO, 2026-09-14, and it loses there as well -- on an RTX 5070 with
    # 720p frames: batch 1 = 784 ms/frame at 1.9 GB, 2 = 765 at 3.7, 4 = 780 at 6.2, and
    # 8 is out of memory. Flat, for the same reason it was flat on the Mac. The knob
    # stays, but nobody should expect it to pay.
    batch: int = 1
    # Precision for the detector's forward pass. "" leaves it at the detector's own
    # float32. MEASURED on an RTX 5070 over eight real frames from the bike ride, forward
    # pass only: fp32 439 ms, fp16 222 ms (2.0x), bf16 240 ms.
    #
    # fp16, not bf16, and the reason is the threshold. bf16 has three fewer mantissa bits
    # and every one of those eight scores came back 0.03-0.08 LOW (0.797 -> 0.751,
    # 0.685 -> 0.610), which is enough to drop a box under the 0.5 acceptance cut and
    # lose a real answer. fp16 held seven of the eight within 0.008.
    #
    # "auto" is fp16 on CUDA and float32 everywhere else. MEASURED on the Mac, sf_office,
    # four queries: fp16 detect 10.30 / 8.34 / 6.69 / 1.88 s against float32's 10.46 /
    # 8.36 / 6.21 / 1.86 -- a wash, same places -- so Metal gets nothing for the score
    # risk and does not take it. "" still means float32, as it always did.
    dtype: str = "auto"
    # Prepare the images in torch on the card instead of numpy on the CPU. MEASURED on an
    # RTX 5070 at 1280x720: 347 ms a frame -> 3.5 ms, which is a third of a detection gone.
    #
    # "auto" is on for CUDA and off for everything else, and Metal is not a judgement call:
    # torch has no `aten::_upsample_bilinear2d_aa` for MPS, so asking for it there raises
    # NotImplementedError mid-run. `PYTORCH_ENABLE_MPS_FALLBACK=1` "fixes" it by doing that
    # very resize on the CPU, which is the expensive step, so there would be nothing left
    # to win. "on"/"off" say it outright.
    #
    # It is not bit-identical -- see `Owlv2Config.gpu_preprocess` -- and every caller here
    # compares a score against `threshold`, so the CUDA default is the one that needed a
    # whole run's answers behind it rather than four frames.
    gpu_preprocess: str = "auto"
    # Subtract the best of a handful of generic prompts -- floor, wall, ceiling, shelf,
    # a room -- from every patch's score. On by default because CLIP scores almost
    # everything somewhat highly, so without it a wall answers most questions moderately
    # well and the frame ranking stops meaning much. Off gives the plain similarity to
    # the query, which is the comparison to make when asking whether the correction is
    # earning its place on a particular recording -- and the right answer when the thing
    # being looked for IS a wall or a floor, since then the contrast subtracts the
    # target. The prompts themselves are `frames.BACKGROUND_PROMPTS`.
    contrast: bool = True
    # Episodes considered before the strongest `max_episodes` of them are detected.
    # Only bounds the work of placing them; a query with more candidates than this is
    # already answering about a very common thing.
    episode_pool: int = 500
    # Ask several models where they agree, and take the candidates from that instead
    # of from time-split episodes. Needs more than one model searched to mean anything.
    agreement: bool = True
    # Score this member over the whole index, then score the others ONLY on the frames
    # it liked. A search costs what it reads, and on bike.db the three members are
    # 19.2 GB of fp32 -- so the cheap member ranking and the expensive ones confirming
    # is most of a query's search time, while agreement survives because all three still
    # have hits on the frames that matter. Empty = every model over everything.
    # The trade, stated because it is real: a frame only the expensive members would
    # have found is never seen, since nothing looks there.
    #
    # "auto" picks the member with the fewest numbers in it -- rows x width, not the
    # smallest-sounding name. ON by default since 2026-09-15, on eleven queries across
    # all three recordings rather than the three of one that were not enough before.
    # Baseline -> auto, places and time to first answer, Mac, two members:
    #
    #   sf_office  a chair             2 ->  2   1.69 -> 1.66 s   search 0.60 -> 0.51
    #              a computer monitor  1 ->  1   1.66 -> 1.56     search 0.60 -> 0.49
    #              a trash can         5 ->  6   1.01 -> 0.94     search 0.57 -> 0.50
    #              a traffic cone      2 ->  2   0.96 -> 0.89     search 0.57 -> 0.50
    #   bike       a stop sign        11 -> 11   1.86 -> 1.22     search 1.17 -> 0.64
    #              a traffic light    12 -> 11   2.87 -> 2.28     search 1.03 -> 0.64
    #              a bicycle           7 ->  9   2.33 -> 1.54     search 1.02 -> 0.64
    #              a traffic cone     17 -> 17   1.62 -> 1.23     search 0.65 -> 0.65
    #   grocery    a shopping basket   5 ->  6   2.13 -> 1.37     search 1.43 -> 0.71
    #              a can of soda       4 ->  4   2.10 -> 1.58     search 1.45 -> 0.92
    #              a shopping cart     8 ->  9   2.64 -> 2.28     search 1.29 -> 0.74
    #
    # Faster on all eleven, and the answers move BOTH WAYS -- +1 trash can, +2 bicycle,
    # +1 basket, +1 cart, -1 traffic light. So this is not recall traded for seconds; it
    # is a different set of mistakes that happens to be cheaper and slightly larger. The
    # lost light is the honest cost and is why the trade is written out here rather than
    # summarised. "" is the way back to searching every member over everything.
    #
    # It matters far more on a machine whose memory is slower: search is 4.7-5.2 s of a
    # 7.5 s first answer on the CudaLaptop against 1.0-1.4 s of 2.1 s on the Mac.
    rank_with: str = "auto"
    # How many of the ranking member's best frames the others confirm. MEASURED on
    # bike.db: leaving this at "all of them" saves NOTHING and costs a little -- the cheap
    # member's 4000 hot patches for one query land on 1335 distinct frames of 2308, so
    # there is nothing to narrow, and paying for the gather made "a stop sign" SLOWER than
    # not doing it at all (2.27 s to first answer against 2.01 s). The cut is the whole
    # point. 0 means no cut, which is now only useful for showing that.
    #
    # Where 400 comes from, warm passes on the Mac, first answer and places kept:
    #
    #                    stop sign      traffic light    trash can
    #   all models       2.01 s / 13    2.15 s / 12      1.72 s / 11
    #   every frame      2.27 s / 13    2.54 s / 12      2.09 s / 11
    #   top 400          2.03 s / 12    1.35 s / 11      1.08 s / 11
    #   top 200          0.98 s / 12    1.04 s /  7      0.85 s / 11
    #   top 60           0.79 s / 11    0.73 s /  7      0.72 s / 11
    #
    # 400 stays within one place of searching everything on all three. Below it there is a
    # CLIFF rather than a slope -- traffic light falls from 12 places to 7 between 400 and
    # 200 and does not recover at 60 -- so the default does not sit next to the edge of it.
    # 200 is the setting to reach for when the wait matters more than the last place, and
    # it is the only one that got "a stop sign" under a second here.
    rank_frames: int = 400
    # Group candidate episodes by roughly where they are and give every group a look
    # before any group gets a second one. Off means strongest-first, which spends the
    # detector on four looks at the nearest chair before it has seen the far one.
    spread_places: bool = True
    # Cells whose depth is within this of the strongest hit's are the same surface it
    # is on. What keeps a cone's cells and drops the floor under it.
    place_band_m: float = 0.3
    # How the agreement step is turned, when it is the one choosing candidates.
    # `HeatConfig`, kept loose to avoid importing it for every config built.
    heat: Any = field(default=None)
    # How close two episodes have to be to count as the same place for that ordering.
    # Deliberately looser than the radius answers are merged at: this estimate comes
    # from patch rays before any detector has looked, and one cone at a metre placed
    # itself over a 1.35 m spread, so 0.75 split it three ways and spent three looks on
    # it. Measured on that cone against the two that exist: at 0.75 the first twelve
    # held two looks at the near cone and reached the far one seventh; at 1.5 one look
    # and fifth; at 2.0, fourth; at 3.0, third. Wider costs a genuinely separate object
    # 2 m away its turn in the first round, so this stops at the first value that fixed
    # the repeat.
    place_radius_m: float = 1.5
    # Episodes to run the detector over at all, strongest first. Detection is ~0.7 s a
    # frame, so this is the knob that decides what a query costs.
    max_episodes: int = 12
    # A run of fewer frames than this is a stray: at ~4 Hz an object the camera really
    # passed is seen several times running.
    min_episode_frames: int = 2
    episode_gap_s: float = 1.0
    # Depth beyond this is a hole, not a reading: RealSense 65535 mm sentinels, and
    # stereo that has given up.
    max_depth_m: float = 10.0
    # Depth pixels inside the 2D box this far from the box's median depth are not the
    # object -- they are the aisle behind it showing through, or the shelf in front.
    depth_band_m: float = 0.5
    # Below this many usable depth pixels the 3D box would be noise; the detection is
    # still reported, without one.
    min_depth_pixels: int = 20
    # The box is the 2nd-98th percentile of the object's points per axis, so one stray
    # pixel on a background surface cannot stretch it across the aisle.
    trim_percentile: float = 2.0
    # How far a depth frame may be from the colour frame to be its pair.
    depth_max_dt: float = 0.05
    # Run depth2depth on the frames a query places, when the recording carries no
    # filled-depth stream: "" off, "auto" only when nothing has filled it already,
    # "default" the package checkpoint, or a checkpoint by name. Off here and "auto"
    # from the CLIs, because a library default that quietly downloads a depth model the
    # first time a box is placed is a surprise; the recordings this is aimed at have a
    # filled stream anyway, and for them "auto" costs nothing.
    depth2depth: str = ""
    world_frame: str = "odom"


@dataclass
class Box3D:
    """An axis-aligned box in the world frame, with what it was measured from."""

    frame: str
    centre: tuple[float, float, float]
    extent: tuple[float, float, float]
    pixels: int
    depth_m: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "frame": self.frame,
            "centre": list(self.centre),
            "extent": list(self.extent),
            "pixels": self.pixels,
            "depth_m": self.depth_m,
        }


@dataclass
class Detection:
    """One episode's answer: where in the recording, and where in the world."""

    query: str
    rank: int
    ts: float
    camera_frame: str
    episode_frames: int
    episode_span: float
    episode_score: float
    models: list[str]
    attempts: int
    score: float = 0.0
    box2d: tuple[float, float, float, float] | None = None
    box3d: Box3D | None = None
    note: str = ""
    # Set by `merge_duplicates`: the rank of the detection this one is another look at.
    duplicate_of: int | None = None
    # Which place in the world this answer belongs to. Stable across the query, so a
    # caller can tell a new thing from a better look at a thing it already has.
    place_id: int | None = None
    # That place's box once this answer is folded in: a second look sharpens the box
    # rather than adding another one beside it.
    refined: Box3D | None = None
    image: Image | None = field(default=None, repr=False)

    @property
    def found(self) -> bool:
        return self.box2d is not None

    def as_dict(self) -> dict[str, Any]:
        return {
            "query": self.query,
            "rank": self.rank,
            "ts": self.ts,
            "camera_frame": self.camera_frame,
            "episode_frames": self.episode_frames,
            "episode_span": self.episode_span,
            "episode_score": self.episode_score,
            "models": self.models,
            "attempts": self.attempts,
            "score": self.score,
            "box2d": None if self.box2d is None else list(self.box2d),
            "box3d": None if self.box3d is None else self.box3d.as_dict(),
            "note": self.note,
            "arrived": self.arrived,
            "duplicate_of": self.duplicate_of,
            "place_id": self.place_id,
            "refined": None if self.refined is None else self.refined.as_dict(),
        }


@dataclass
class Measured:
    """The object's points in the camera's frame, or why the box could not be measured.

    The reason is half the value. "No usable depth inside the box" was one sentence for
    four different situations, and the one that matters -- a working sensor whose every
    reading in the box is past the cut-off -- looked exactly like the one depth2depth
    exists to fix. They are told apart here so that nobody has to guess which happened.
    """

    points: NDArray[np.float64] | None = None
    median: float = 0.0
    why: str = ""

    def __bool__(self) -> bool:
        return self.points is not None


def object_points(
    box: Sequence[float],
    image_size: tuple[int, int],
    depth: NDArray[np.floating],
    intrinsics: hs.Intrinsics,
    *,
    max_depth_m: float = 10.0,
    band_m: float = 0.5,
    min_pixels: int = 20,
) -> Measured:
    """The object's points, in the camera's own frame, from a 2D box and a depth image.

    *box* is ``(x1, y1, x2, y2)`` in the pixels of an image of *image_size*; *depth* is
    metres on the colour camera's grid, zero where there is no reading.

    A detector box is tight around the object but never only the object: its corners
    see whatever is behind. So the box's median depth is taken to be the object -- for a
    box that is mostly its subject, it is -- and pixels further than *band_m* from that
    are dropped as background showing through.

    The *max_depth_m* cut lives here rather than where the depth was read, because here
    it can be counted: a box on something twelve metres out has perfectly good depth and
    is refused, and that is worth saying out loud instead of reporting it as no depth.
    """
    height, width = depth.shape
    image_width, image_height = image_size
    if not image_width or not image_height or not width or not height:
        return Measured(why="the box or the depth image has no size")
    # The box arrives in the decoded image's pixels; the depth may be on a different
    # grid. Going through fractions of the frame keeps the two independent of each other.
    left = int(np.clip(np.floor(box[0] / image_width * width), 0, width - 1))
    top = int(np.clip(np.floor(box[1] / image_height * height), 0, height - 1))
    right = int(np.clip(np.ceil(box[2] / image_width * width), left + 1, width))
    bottom = int(np.clip(np.ceil(box[3] / image_height * height), top + 1, height))

    window = np.asarray(depth[top:bottom, left:right], dtype=np.float64)
    read = np.isfinite(window) & (window > 0)
    usable = read & (window <= max_depth_m)
    if not int(read.sum()):
        return Measured(why="the depth image has no reading at all inside the box")
    if not int(usable.sum()):
        return Measured(
            why=f"every depth reading inside the box is past the {max_depth_m:g} m cut-off "
            f"(nearest {float(window[read].min()):.1f} m)"
        )
    if int(usable.sum()) < min_pixels:
        return Measured(
            why=f"only {int(usable.sum())} depth reading(s) inside the box within "
            f"{max_depth_m:g} m, {min_pixels} needed"
        )
    median = float(np.median(window[usable]))
    keep = usable & (np.abs(window - median) <= band_m)
    if int(keep.sum()) < min_pixels:
        return Measured(
            why=f"only {int(keep.sum())} depth reading(s) within {band_m:g} m of the box's "
            f"median {median:.1f} m, {min_pixels} needed"
        )

    rows, cols = np.nonzero(keep)
    z = window[rows, cols]
    # Intrinsics scaled onto whatever grid the depth is actually on.
    scale_x, scale_y = width / intrinsics.width, height / intrinsics.height
    us = (cols + left + 0.5 - intrinsics.cx * scale_x) / (intrinsics.fx * scale_x)
    vs = (rows + top + 0.5 - intrinsics.cy * scale_y) / (intrinsics.fy * scale_y)
    return Measured(np.stack([us * z, vs * z, z], axis=1), median)


def box_from_points(
    points: NDArray[np.floating],
    world_from_camera: NDArray[np.floating],
    frame: str,
    depth_m: float,
    *,
    trim_percentile: float = 2.0,
) -> Box3D:
    """Camera-frame points through a pose into an axis-aligned world box."""
    moved = (world_from_camera[:3, :3] @ np.asarray(points, dtype=np.float64).T).T
    moved += world_from_camera[:3, 3]
    low = np.percentile(moved, trim_percentile, axis=0)
    high = np.percentile(moved, 100.0 - trim_percentile, axis=0)
    centre = (low + high) / 2.0
    extent = high - low
    return Box3D(
        frame=frame,
        centre=(float(centre[0]), float(centre[1]), float(centre[2])),
        extent=(float(extent[0]), float(extent[1]), float(extent[2])),
        pixels=len(moved),
        depth_m=float(depth_m),
    )


class RecordingFrames:
    """The recording's own images, depth and transforms, read one moment at a time.

    The index says *which* moments are interesting; the pictures themselves still live
    in the recording. Depth is re-rendered onto the colour camera's grid on the way out,
    the same way the ingest pairs them, so a box drawn on the colour image indexes
    straight into it.
    """

    def __init__(
        self,
        recording: Any,
        *,
        color_stream: str = "color_image",
        depth_stream: str = "depth_image",
        color_info_stream: str = "camera_info",
        depth_info_stream: str = "depth_camera_info",
        tf_stream: str = TF_STREAM,
        config: DetectConfig | None = None,
    ) -> None:
        self.recording = recording
        self.color_stream = color_stream
        self.depth_stream = depth_stream
        self.config = config or DetectConfig()
        self.intrinsics: dict[str, hs.Intrinsics] = {}
        # The ingest's own record last, so it wins: live it is the only one there is,
        # and offline it is the camera the patches were actually measured through.
        for name in (color_info_stream, depth_info_stream, info_stream_for("")):
            if name not in recording.list_streams():
                continue
            for observation in recording.streams[name].order_by("ts"):
                info = observation.data
                self.intrinsics[info.frame_id] = intrinsics_of(info)
        if not self.intrinsics:
            logger.warning(
                f"hyperspace: no camera_info in {color_info_stream!r}, {depth_info_stream!r} "
                f"or {info_stream_for('')!r} -- no answer can be placed in the world"
            )
        self.tf = FlexTf()
        self._tf_stream = tf_stream
        self._tf_loaded = False
        filled = filled_stream_for("")
        self._filled_stream = filled if filled in recording.list_streams() else None
        if self._filled_stream:
            logger.info(f"hyperspace: placing boxes off {filled}")
        # Live, the colour stream is the frames the ingest kept: the camera's own 30 Hz
        # was never written anywhere, and the only pictures that exist are the ones an
        # embedding frame was made from.
        kept = frame_stream_for("")
        self._kept_frames = kept if kept in recording.list_streams() else None
        if self._kept_frames:
            logger.info(f"hyperspace: showing the detector {kept}")
        self._fusion: Any = None
        self._fused_cache: dict[tuple[str, float], NDArray[np.float32]] = {}

    def warm(self) -> float:
        """Do the first lookup's work now, while nobody is waiting on an answer.

        A recording's transforms are read in one pass the first time anything is placed
        -- a quarter of a million rows on sf_office -- and the image streams build their
        by-stamp lookup on first use. Together that was three seconds charged to whoever
        asked the first question. Returns the seconds spent.
        """
        started = time.monotonic()
        self.load_tf()
        for name in (self.color_stream, self.depth_stream):
            if name not in self.recording.list_streams():
                continue
            first = self.recording.streams[name].order_by("ts").limit(1).to_list()
            if first:
                self.recording.streams[name].at(
                    float(first[0].ts), tolerance=self.config.depth_max_dt
                ).to_list()
        return time.monotonic() - started

    def load_tf(self) -> None:
        if self._tf_loaded or self._tf_stream not in self.recording.list_streams():
            self._tf_loaded = True
            return
        for observation in self.recording.stream(self._tf_stream, TFMessage).order_by("ts"):
            self.tf.receive_tfmessage(observation.data)
        self._tf_loaded = True

    def filled(self, camera_frame: str, ts: float) -> NDArray[np.float32] | None:
        """Depth with stereo's holes filled, if this recording carries any."""
        del camera_frame
        if self._filled_stream is None:
            return None
        tolerance = self.config.depth_max_dt
        found = self.recording.streams[self._filled_stream].at(ts, tolerance=tolerance).to_list()
        if not found:
            return None
        nearest = min(found, key=lambda observation: abs(float(observation.ts) - ts))
        if abs(float(nearest.ts) - ts) > tolerance:
            return None
        return _holes_as_zero(np.asarray(nearest.data["depth_mm"], dtype=np.float32) * 0.001)

    def pose(self, camera_frame: str, ts: float, world_frame: str) -> NDArray[np.float64] | None:
        self.load_tf()
        poses, valid = self.tf.batch_get(world_frame, camera_frame, [ts])
        return poses[0] if valid[0] else None

    def color(self, ts: float) -> Image | None:
        """The picture taken at this moment, from wherever this recording keeps them.

        The kept frames first: on a live run they are the only pictures there are, and
        on a recording that has both they are the same photograph the patches were made
        from, which is the one the box was measured against.
        """
        for name in (self._kept_frames, self.color_stream):
            if name is None or name not in self.recording.list_streams():
                continue
            found = self.recording.streams[name].at(ts, tolerance=0.05).to_list()
            if found:
                nearest = min(found, key=lambda observation: abs(float(observation.ts) - ts))
                return decoded(nearest.data)
        return None

    def depth(self, camera_frame: str, ts: float) -> NDArray[np.float32] | None:
        """Metres on the colour camera's grid, zero where there is no reading.

        Three sources, cheapest first. Filled depth if the recording carries any --
        written live by the depth2depth module or afterwards by `fill_depth`, either way
        already on the colour grid and already aligned. Otherwise the stereo, run
        through depth2depth here and now if this instance was given a model: a query
        places a dozen boxes and the model is fifty milliseconds against the detector's
        seven hundred, so a recording nobody thought to fill is not thereby placed off
        stereo's holes. Otherwise the stereo as it came.
        """
        filled = self.filled(camera_frame, ts)
        if filled is not None:
            return filled
        raw = self.raw_depth(camera_frame, ts)
        if raw is None:
            return None
        return self.fused(camera_frame, ts, raw)

    def raw_depth(self, camera_frame: str, ts: float) -> NDArray[np.float32] | None:
        """The stereo's own depth, in metres, re-rendered onto the colour camera's grid."""
        tolerance = self.config.depth_max_dt
        if self.depth_stream not in self.recording.list_streams():
            return None
        found = self.recording.streams[self.depth_stream].at(ts, tolerance=tolerance).to_list()
        if not found:
            return None
        nearest = min(found, key=lambda observation: abs(float(observation.ts) - ts))
        if abs(float(nearest.ts) - ts) > tolerance:
            return None
        image = decoded(nearest.data)
        raw = np.asarray(image.as_numpy())
        metres = _holes_as_zero(raw.astype(np.float32) * (0.001 if raw.dtype == np.uint16 else 1.0))

        color = self.intrinsics.get(camera_frame)
        depth_frame = image.frame_id
        if color is None:
            return metres
        if depth_frame == camera_frame:
            return metres
        depth_intrinsics = self.intrinsics.get(depth_frame)
        if depth_intrinsics is None:
            return metres
        self.load_tf()
        poses, valid = self.tf.batch_get(camera_frame, depth_frame, [ts])
        if not valid[0]:
            return metres
        return hs.reproject_depth(metres, depth_intrinsics, color, poses[0])

    def depth2depth(self) -> Any:
        """The fusion model, loaded on first use, or None if this run does not want one.

        ``"auto"`` means "only if nobody has filled this recording already", which is
        the honest reading of the knob: `fill_depth` and this do the same arithmetic,
        and paying for it twice buys nothing.
        """
        wanted = self.config.depth2depth
        if not wanted or wanted == "off":
            return None
        if wanted == "auto" and self._filled_stream is not None:
            return None
        if self._fusion is None:
            from dimos.mapping.hyperspace.module import depth2depth_model_of
            from dimos.perception.depth2depth.fusion import Depth2Depth

            name = depth2depth_model_of("default" if wanted == "auto" else wanted)
            try:
                model = Depth2Depth(model_name=name, device=self.config.device or "auto")
                model.start()
            except Exception as failure:  # no checkpoint, no network, OOM -- all survivable
                # Worse depth beats no answer: the boxes still land, off the stereo, and
                # the log says why they are the ones from before rather than the better
                # ones somebody asked for.
                logger.warning(
                    f"hyperspace: {name} would not load, placing off raw depth: {failure}"
                )
                self.config = replace(self.config, depth2depth="")
                return None
            logger.info(f"hyperspace: filling depth with {name} as boxes are placed")
            self._fusion = model
        return self._fusion

    def fused(self, camera_frame: str, ts: float, raw: NDArray[np.float32]) -> NDArray[np.float32]:
        """*raw* with stereo's holes filled, if this run carries a model; *raw* if not.

        Cached per frame: one photograph is placed once per box the detector drew on it,
        and the model must not be paid four times for the same picture.
        """
        model = self.depth2depth()
        if model is None:
            return raw
        key = (camera_frame, round(float(ts), 4))
        held = self._fused_cache.get(key)
        if held is not None:
            return held
        image = self.color(ts)
        if image is None:
            return raw
        rgb = np.ascontiguousarray(np.asarray(image.to_rgb().data, dtype=np.uint8))
        if rgb.shape[:2] != raw.shape[:2]:
            from PIL import Image as PILImage

            resized = PILImage.fromarray(rgb).resize((raw.shape[1], raw.shape[0]))
            rgb = np.ascontiguousarray(np.asarray(resized, dtype=np.uint8))
        try:
            filled = np.asarray(model.fuse(rgb, raw).fused, dtype=np.float32)
        except Exception as failure:  # a bad frame, an OOM -- the raw depth still places
            logger.warning(f"hyperspace: depth2depth failed, placing off raw depth: {failure}")
            self.config = replace(self.config, depth2depth="")
            return raw
        # The frames a query places are scattered through the recording, so the cache is
        # bounded rather than the whole run: it exists for the repeat within one frame.
        if len(self._fused_cache) > 64:
            self._fused_cache.clear()
        self._fused_cache[key] = filled
        return filled


def _torch_dtype(name: str) -> Any:
    """`"fp16"` -> `torch.float16`, and a wrong name says so rather than running fp32.

    Silently falling back would look exactly like a speedup that failed to arrive.
    """
    import torch

    known = {
        "fp16": torch.float16,
        "float16": torch.float16,
        "half": torch.float16,
        "bf16": torch.bfloat16,
        "bfloat16": torch.bfloat16,
        "fp32": torch.float32,
        "float32": torch.float32,
    }
    if name not in known:
        raise ValueError(f"unknown detector dtype {name!r}; one of {sorted(known)}")
    return known[name]


def _holes_as_zero(metres: NDArray[np.float32]) -> NDArray[np.float32]:
    """Zero where the sensor said nothing. Everything it did say is kept.

    The far cut-off deliberately does NOT happen here. It used to, and it turned "the
    stereo returned 65 m" and "the stereo returned nothing" into the same array, so the
    only thing `object_points` could report was that there was no depth.
    """
    metres[~np.isfinite(metres) | (metres < 0)] = 0.0
    return metres


def detector_dtype_for(setting: str, device: str) -> str:
    """The precision to run the detector at, once the device is known.

    "auto" means fp16 on CUDA, where it halves the forward pass, and float32 everywhere
    else, where it buys nothing -- measured on MPS, four queries, detect within noise and
    the same places. Anything else is taken at its word, including "" for float32.
    """
    if setting != "auto":
        return setting
    return "fp16" if device.startswith("cuda") else ""


def gpu_preprocess_for(setting: str, device: str) -> bool:
    """Whether to prepare the detector's images in torch rather than in the processor.

    "auto" is CUDA only, and that is a recall decision rather than a speed one.

    THE SPEED IS REAL AND SO IS THE COST. The win was never the GPU: on an M-series Mac
    the processor's preprocessing is 155 ms against 4.0 ms for the same four steps in
    torch on the CPU, beside a 141 ms forward pass -- so even MPS, which cannot run them
    (no `aten::_upsample_bilinear2d_aa`), could prepare on the cpu and halve a detector
    pass. MEASURED end to end on bike, detector time 4.33 / 10.66 / 5.02 / 4.74 s fell
    to 2.81 / 5.88 / 2.69 / 2.98.

    AND THE ANSWERS MOVED: places went 11 / 11 / 9 / 17 to 10 / 7 / 9 / 17. Four of
    eleven traffic lights, gone. That is the documented cost of a different resize kernel
    arriving at a 0.5 acceptance cut, and it is exactly what "re-check against a whole
    run's answers" was written to catch. Twice the speed for a third of the traffic
    lights is not a trade to make quietly, so Metal keeps the slow, faithful path until
    someone decides otherwise with this in front of them.

    `preprocess_device_for` still knows where the fast path would run, so turning it on
    is `gpu_preprocess="on"` and nothing more.
    """
    if setting == "auto":
        return device.startswith("cuda")
    return setting.lower() in {"1", "on", "true", "yes"}


def preprocess_device_for(device: str) -> str:
    """Where the image preparation runs, given where the model lives.

    CUDA does it in place. Metal cannot -- the antialiased resize does not exist there --
    so it prepares on the cpu and pays one small copy, which is nothing against the
    151 ms it saves.
    """
    return "cpu" if device == "mps" else ""


class Owlv2Boxes:
    """Core's OWLv2 detector, loaded once and asked about whole rounds of frames."""

    def __init__(self, config: DetectConfig | None = None) -> None:
        self.config = config or DetectConfig()
        self._detector: Any = None

    @property
    def detector(self) -> Any:
        if self._detector is None:
            from dimos.perception.detection.detectors.owlv2 import Owlv2Detector

            # A Configurable builds its own config from keyword arguments; handing it a
            # ready-made one is rejected as an extra input.
            settings: dict[str, Any] = {"model_name": self.config.checkpoint}
            if self.config.device:
                settings["device"] = self.config.device
            from dimos.mapping.hyperspace.module import pick_device

            chosen = settings.get("device") or pick_device("auto")
            dtype = detector_dtype_for(self.config.dtype, chosen)
            if dtype:
                settings["dtype"] = _torch_dtype(dtype)
            if gpu_preprocess_for(self.config.gpu_preprocess, chosen):
                settings["gpu_preprocess"] = True
                where = preprocess_device_for(chosen)
                if where:
                    settings["preprocess_device"] = where
            self._detector = Owlv2Detector(**settings)
        return self._detector

    def warm(self) -> float:
        """Load the weights and run one frame through, before anyone is waiting.

        Building the detector is lazy twice over -- the object defers the model, and the
        model defers the weights until something is detected -- so the first real query
        of a process paid about five seconds that had nothing to do with it. A blank
        frame costs one forward pass and moves that cost to startup, where a wait is
        free. Returns the seconds spent, for a caller that wants to say so.
        """
        started = time.monotonic()
        blank = Image.from_numpy(np.zeros((32, 32, 3), dtype=np.uint8), frame_id="warmup", ts=0.0)
        self.best_many([blank], "a thing")
        return time.monotonic() - started

    def best(
        self, image: Image, text: str
    ) -> tuple[tuple[float, float, float, float], float] | None:
        """The strongest box for *text*, or nothing if the detector refuses the image."""
        return self.best_many([image], text)[0]

    def best_many(
        self, images: Sequence[Image], text: str
    ) -> list[tuple[tuple[float, float, float, float], float] | None]:
        """The strongest box per image, or None where the detector refused it."""
        return [found[0] if found else None for found in self.all_many(images, text)]

    def all_many(
        self, images: Sequence[Image], text: str
    ) -> list[list[tuple[tuple[float, float, float, float], float]]]:
        """Every box the detector accepted, strongest first, per image.

        Two cones in one photograph are two answers. Keeping only the strongest made a
        frame worth at most one thing, which quietly lost every second instance that
        happened to share a view with a better one.

        One list per image, in input order, so a caller can keep its own bookkeeping
        beside it. The images of one round are unrelated to each other -- the batch is
        purely about paying the per-call overhead once instead of once per frame.
        """
        answers: list[list[tuple[tuple[float, float, float, float], float]]] = []
        size = max(1, self.config.batch)
        for start in range(0, len(images), size):
            chunk = list(images[start : start + size])
            for found in self.detector.query_detections_batch(
                chunk, [text], threshold=self.config.threshold
            ):
                here: list[tuple[tuple[float, float, float, float], float]] = []
                for detection in sorted(
                    found.detections, key=lambda detection: -detection.confidence
                ):
                    x1, y1, x2, y2 = (float(v) for v in detection.bbox)
                    here.append(((x1, y1, x2, y2), float(detection.confidence)))
                answers.append(here)
        return answers


def detect_episode(
    episode: Episode,
    query: str,
    frames: RecordingFrames,
    boxes: Owlv2Boxes,
    *,
    rank: int,
    config: DetectConfig,
    keep_image: bool = False,
) -> Detection:
    """Run the detector over one episode's best frames until one of them answers.

    A frame can fail twice over: the detector may refuse it, or it may be detected and
    then not placeable because the stereo gave nothing back inside the box (glass,
    a dark shelf, a shiny floor). Either way the next-best frame of the same episode is
    another look at the same thing, so both failures fall through rather than ending it.
    A detection that was found but never placed is kept as the answer of last resort.
    """
    peak = episode.peak
    Detection(
        query=query,
        rank=rank,
        ts=peak.ts,
        camera_frame=peak.frame,
        episode_frames=len(episode.frames),
        episode_span=episode.span,
        episode_score=episode.score,
        models=sorted(episode.members),
        attempts=0,
    )
    return detect_episodes(
        [episode], query, frames, boxes, config=config, keep_images=keep_image, first_rank=rank
    )[0]


@dataclass
class _Try:
    """One episode part-way through its attempts, so a round can be shared."""

    detection: Detection
    candidates: list[Frame]
    answer: Detection | None = None
    flat: Detection | None = None
    refusals: int = 0
    # Other things the detector found in the same photograph. Two cones in one frame
    # are two answers, and the second one is nobody else's episode to report.
    beside: list[Detection] = field(default_factory=list)

    @property
    def settled(self) -> bool:
        return self.answer is not None

    def finish(self) -> list[Detection]:
        return [self.answer or self.flat or self.detection, *self.beside]


def place_of(
    episode: Episode, frames: RecordingFrames, world_frame: str, *, band_m: float = 0.3
) -> NDArray[np.float64] | None:
    """Roughly where an episode's match is, before any detector has looked at it.

    A hot patch already carries the ray through its cell and the depth the sensor read
    there, which is all the dense path ever had; one transform puts it in the world.

    The hot cells are not the object. Measured on one cone: a 24x42 grid over an
    848x480 frame makes a cell about 11 cm of scene at 2.3 m, so a 40 cm cone is four
    cells -- and a hundred cells came back hot, a tenth of the frame. Those extra cells
    are mostly the floor under and in front of it, at their own perfectly correct
    depths, so averaging over all of them lands between two surfaces: the strongest hit
    alone was 0.17 m from the cone where the median of the top five was 1.54 m.

    So the strongest hit picks the surface and *band_m* keeps the cells that agree with
    it, which the object's do and the floor's do not. The same move `object_points`
    makes inside the detector's box, one step earlier.
    """
    peak = episode.peak
    usable = sorted(
        (hit for hit in peak.hits if np.isfinite(hit.depth) and hit.depth > 0),
        key=lambda hit: -hit.score,
    )
    if not usable:
        return None
    surface = usable[0].depth
    kept = [hit for hit in usable if abs(hit.depth - surface) <= band_m]
    weights = np.array([max(hit.score, 1e-6) for hit in kept])
    points = np.array([[hit.ray[0] * hit.depth, hit.ray[1] * hit.depth, hit.depth] for hit in kept])
    here = (points * weights[:, None]).sum(axis=0) / weights.sum()
    pose = frames.pose(peak.frame, peak.ts, world_frame)
    if pose is None:
        return None
    return np.asarray(pose @ np.append(here, 1.0))[:3]


def spread_by_place(
    episodes: Sequence[Episode], frames: RecordingFrames, *, config: DetectConfig
) -> list[Episode]:
    """Order episodes so every place gets a look before any place gets a second one.

    Strongest-first spends the detector on whatever the camera saw most of: on
    sf_office, seven of twelve cone episodes were the same cone, while a second cone
    across the room never got a look. Grouping by where the patches say they are and
    taking one from each group in turn buys distinct answers with the same budget.

    Only the order changes. An episode the grouping gets wrong is detected sooner or
    later than it would have been, which is a different thing from being dropped, and
    the grouping leans on patch depth -- reliable up close, not at ten metres.
    """
    places = [
        place_of(episode, frames, config.world_frame, band_m=config.place_band_m)
        for episode in episodes
    ]
    groups: list[list[int]] = []
    centres: list[NDArray[np.float64] | None] = []
    for index, here in enumerate(places):
        joined = False
        if here is not None:
            for group, centre in zip(groups, centres, strict=True):
                if centre is None:
                    continue
                if float(np.linalg.norm(here - centre)) <= config.place_radius_m:
                    group.append(index)
                    joined = True
                    break
        if not joined:
            # An episode we could not place is its own group rather than dropped: not
            # knowing where it is says nothing about whether it is worth detecting.
            groups.append([index])
            centres.append(here)

    # The episodes arrive strongest-first, so the groups are already in that order and
    # so is each group's own list.
    ordered: list[Episode] = []
    for round_ in range(max((len(group) for group in groups), default=0)):
        for group in groups:
            if round_ < len(group):
                ordered.append(episodes[group[round_]])
    return ordered


def detect_episodes(
    episodes: Sequence[Episode],
    query: str,
    frames: RecordingFrames,
    boxes: Owlv2Boxes,
    *,
    config: DetectConfig,
    keep_images: bool = False,
    first_rank: int = 1,
) -> list[Detection]:
    """Every episode's answer, in rank order. See `stream_episodes` for the order of work."""
    return list(
        stream_episodes(
            episodes,
            query,
            frames,
            boxes,
            config=config,
            keep_images=keep_images,
            first_rank=first_rank,
        )
    )


def stream_episodes(
    episodes: Sequence[Episode],
    query: str,
    frames: RecordingFrames,
    boxes: Owlv2Boxes,
    *,
    config: DetectConfig,
    keep_images: bool = False,
    first_rank: int = 1,
    timings: dict[str, float] | None = None,
) -> Iterator[Detection]:
    """Answers as they settle, in rank order.

    A detector call costs the same whether it is shown one frame or several, so when a
    batch is worth having the episodes go through it together: every unanswered one's
    next-best frame in a single pass, then the round after that. Nothing can be said
    about any of them until the round returns, so the answers arrive in a burst.

    At a batch of one -- the default, because batching measured slower on this
    hardware -- there is nothing to gather, and waiting would buy only a longer silence.
    So each episode is finished and handed back before the next one starts, and the
    first answer arrives after one detector call rather than after twelve.
    """
    tries = [
        _Try(
            detection=Detection(
                query=query,
                rank=rank,
                ts=episode.peak.ts,
                camera_frame=episode.peak.frame,
                episode_frames=len(episode.frames),
                episode_span=episode.span,
                episode_score=episode.score,
                models=sorted(episode.members),
                attempts=0,
            ),
            candidates=list(episode.by_weight()[: max(1, config.attempts)]),
        )
        for rank, episode in enumerate(episodes, first_rank)
    ]

    rank = first_rank + len(tries)

    def count_passes() -> None:
        """Forward passes this query actually cost, summed HERE and nowhere else.

        It has to be counted over `tries`, one entry per episode, because `finish()`
        hands back the episode's own detection plus a `beside` copy for every extra box
        in the same photograph -- and those copies carry the SAME `attempts`. Summing
        over the yielded answers would multiply an episode's cost by how many things the
        detector happened to see in one frame.

        Worth having on the wire because it is the number that tells "more work" from
        "dearer work", and nothing outside could tell them apart. MEASURED on grocery,
        three queries at 12x3, times ranged 138-274 s with the pass count assumed
        constant at 30-36 -- an assumption that was an upper bound (`candidates` is
        capped at `min(attempts, len(episode.frames))`, so a one-frame episode costs one
        pass) treated as a count. With this, nobody has to assume it again.
        """
        if timings is not None:
            timings["passes"] = float(sum(one.detection.attempts for one in tries))

    if config.batch <= 1:
        for one in tries:
            _attempt_rounds([one], query, frames, boxes, config=config, keep_images=keep_images)
            for answer in one.finish():
                if answer.rank == 0:
                    answer.rank, rank = rank, rank + 1
                yield answer
        count_passes()
        return

    _attempt_rounds(tries, query, frames, boxes, config=config, keep_images=keep_images)
    count_passes()
    for one in tries:
        for answer in one.finish():
            if answer.rank == 0:
                answer.rank, rank = rank, rank + 1
            yield answer


def _attempt_rounds(
    tries: Sequence[_Try],
    query: str,
    frames: RecordingFrames,
    boxes: Owlv2Boxes,
    *,
    config: DetectConfig,
    keep_images: bool,
) -> None:
    """Take every unsettled episode through its next frame, until they run out."""
    for round_ in range(max(1, config.attempts)):
        pending: list[tuple[_Try, Frame, Image]] = []
        for attempt_of in tries:
            if attempt_of.settled or round_ >= len(attempt_of.candidates):
                continue
            candidate = attempt_of.candidates[round_]
            attempt_of.detection.attempts += 1
            image = frames.color(candidate.ts)
            if image is None:
                attempt_of.detection.note = "no colour frame at that stamp"
                continue
            pending.append((attempt_of, candidate, image))
        if not pending:
            break
        found_in_round = boxes.all_many([image for _, _, image in pending], query)
        for (attempt_of, candidate, image), found in zip(pending, found_in_round, strict=True):
            if not found:
                attempt_of.refusals += 1
                attempt_of.detection.note = f"detector refused {attempt_of.refusals} frame(s)"
                continue
            for position, box in enumerate(found[: max(1, config.per_frame)]):
                attempt = replace(
                    attempt_of.detection, ts=candidate.ts, camera_frame=candidate.frame, note=""
                )
                attempt.box2d, attempt.score = box
                if keep_images:
                    attempt.image = image
                _place(attempt, candidate, image, frames, config)
                if position == 0:
                    # The strongest box is this episode's answer; the rest are other
                    # things in the same photograph and get ranks of their own later.
                    if attempt.box3d is not None:
                        attempt_of.answer = attempt
                    else:
                        attempt_of.flat = attempt_of.flat or attempt
                elif attempt.box3d is not None:
                    attempt.rank = 0
                    attempt_of.beside.append(attempt)


def _place(
    detection: Detection,
    frame: Frame,
    image: Image,
    frames: RecordingFrames,
    config: DetectConfig,
) -> None:
    """Turn a 2D box into a 3D one, or say why it could not be."""
    assert detection.box2d is not None
    depth = frames.depth(frame.frame, frame.ts)
    if depth is None:
        detection.note = "no depth frame paired with that image"
        return
    intrinsics = frames.intrinsics.get(frame.frame)
    if intrinsics is None:
        detection.note = f"no camera_info for {frame.frame!r}"
        return
    rgb = np.asarray(image.to_rgb().data)
    measured = object_points(
        detection.box2d,
        (int(rgb.shape[1]), int(rgb.shape[0])),
        depth,
        intrinsics,
        max_depth_m=config.max_depth_m,
        band_m=config.depth_band_m,
        min_pixels=config.min_depth_pixels,
    )
    if not measured:
        detection.note = measured.why
        return
    points, median = measured.points, measured.median
    pose = frames.pose(frame.frame, frame.ts, config.world_frame)
    if pose is None:
        detection.note = f"no transform {config.world_frame} <- {frame.frame}"
        return
    detection.box3d = box_from_points(
        points, pose, config.world_frame, median, trim_percentile=config.trim_percentile
    )


def merge_duplicates(detections: Sequence[Detection], merge_m: float = 0.75) -> int:
    """Group answers that are the same place, and sharpen each place as looks arrive.

    Episodes are split on time deliberately: the trolley passes the cheese counter four
    times and that is four chances at it. But the four answers are one place, so the
    last step is to say so -- in 3D, where "the same place" means something.

    A second look does not add a box beside the first. It joins the place, and the
    place's box becomes the average of its looks weighted by what the detector thought
    of each, so `refined` on any answer is that place's box as of that moment. Nothing
    is dropped: every answer keeps its own box too, because a second look is evidence.

    Answers are folded in the order they arrived rather than strongest-first, and the
    place belongs to the look that found it. A caller replaying a query then sees what
    a caller watching it saw.
    """
    placed = [detection for detection in detections if detection.box3d is not None]
    for detection in detections:
        detection.duplicate_of = None
        detection.place_id = None
        detection.refined = None

    places: list[dict[str, Any]] = []
    for detection in sorted(placed, key=lambda detection: detection.rank):
        assert detection.box3d is not None
        here = np.asarray(detection.box3d.centre, dtype=float)
        size = np.asarray(detection.box3d.extent, dtype=float)
        # A weak answer should not drag a place around, but a zero score still counts.
        weight = max(float(detection.score), 1e-6)
        joined = None
        for place in places:
            if float(np.linalg.norm(place["centre"] - here)) <= merge_m:
                joined = place
                break
        if joined is None:
            joined = {
                "id": len(places) + 1,
                "centre": here,
                "extent": size,
                "weight": weight,
                "first": detection.rank,
            }
            places.append(joined)
        else:
            total = joined["weight"] + weight
            joined["centre"] = (joined["centre"] * joined["weight"] + here * weight) / total
            joined["extent"] = (joined["extent"] * joined["weight"] + size * weight) / total
            joined["weight"] = total
            detection.duplicate_of = int(joined["first"])
        detection.place_id = int(joined["id"])
        detection.refined = Box3D(
            frame=detection.box3d.frame,
            centre=tuple(float(v) for v in joined["centre"]),
            extent=tuple(float(v) for v in joined["extent"]),
            pixels=detection.box3d.pixels,
            depth_m=detection.box3d.depth_m,
        )
    return len(places)


def _agreed_candidates(
    matched: Sequence[Frame],
    frames: RecordingFrames,
    *,
    config: DetectConfig,
    models: Sequence[str] | None,
) -> list[Episode] | None:
    """Candidates from where several models agree, or None if that is not on offer.

    One model's hot cells are a tenth of the frame and mostly floor, so splitting them
    on time and hoping gives the detector a lot of floor to refuse. Where three models
    agree is a much smaller set, and each place arrives already knowing which frame it
    was clearest in.
    """
    if not config.agreement or not matched:
        return None
    members = {hit.member for frame in matched for hit in frame.hits}
    if len(members) < 2:
        return None

    from dimos.mapping.hyperspace import heat

    settings = config.heat or heat.HeatConfig()
    # Every frame's pose in one call. Asking one at a time walked the transform tree
    # eight hundred times and cost more than the agreement it was feeding.
    frames.load_tf()
    found_poses, usable = frames.tf.batch_get(
        config.world_frame,
        [frame.frame for frame in matched],
        [frame.ts for frame in matched],
    )
    poses = {
        (frame.frame, frame.ts): found_poses[index]
        for index, frame in enumerate(matched)
        if usable[index]
    }
    by_ts = {(frame.frame, frame.ts): frame for frame in matched}
    found: list[Episode] = []
    for place in heat.places_of(matched, poses, config=settings):
        looks = {}
        warmth = {}
        for box in place.boxes:
            key = (box.camera_frame, box.ts)
            frame = by_ts.get(key)
            if frame is None:
                continue
            looks[key] = frame
            warmth[box.ts] = max(warmth.get(box.ts, 0.0), box.heat)
        if looks:
            found.append(Episode(frames=list(looks.values()), heat=warmth))
    del models
    return found


def find(
    store: Any,
    recording: Any,
    query: str,
    *,
    config: DetectConfig | None = None,
    models: Sequence[str] | None = None,
    towers: Any = None,
    frames: RecordingFrames | None = None,
    boxes: Owlv2Boxes | None = None,
    keep_images: bool = False,
    resident: Any = None,
    timings: dict[str, float] | None = None,
    background_prompts: Sequence[str] | None = None,
) -> Iterator[Detection]:
    """The whole chain: text in, one detection per episode out, in rank order.

    *store* holds the patch index, *recording* the pictures. They are usually the same
    file -- a recording indexes itself -- but an .mcap keeps its index alongside.

    The episodes are detected as a group rather than one at a time, so results arrive
    together at the end rather than trickling out. That is the price of batching, and
    it is worth paying: the detector is a fixed cost per call, so twelve episodes in
    one pass is most of a query's detector time saved, while the trickle only ever
    bought a progress bar.

    *background_prompts* replaces what the contrast subtracts, for this call only. The
    default set is generic room surfaces, which is right for a thing and wrong for
    anything those surfaces are part of.
    """
    from dimos.mapping.hyperspace.frames import hot_frames, ranked_episodes

    config = config or DetectConfig()
    frames = frames or RecordingFrames(recording, config=config)
    boxes = boxes or Owlv2Boxes(config)
    asked = at = time.monotonic()
    matched = hot_frames(
        store,
        query,
        towers=towers,
        models=models,
        resident=resident,
        contrast=config.contrast,
        background_prompts=background_prompts,
        rank_with=config.rank_with,
        rank_frames=config.rank_frames,
    )
    if timings is not None:
        timings["search"] = time.monotonic() - at
        timings["frames_matched"] = float(len(matched))
    if not matched:
        return
    at = time.monotonic()
    agreed = _agreed_candidates(matched, frames, config=config, models=models)
    if agreed is not None:
        found = agreed
    else:
        found = ranked_episodes(
            matched,
            gap_s=config.episode_gap_s,
            min_frames=config.min_episode_frames,
            limit=config.episode_pool if config.spread_places else config.max_episodes,
        )
        if config.spread_places:
            # Ordered before it is cut, or the cut decides which places are ever seen.
            found = spread_by_place(found, frames, config=config)
    found = found[: config.max_episodes]
    if timings is not None:
        timings["episodes"] = time.monotonic() - at
    at = time.monotonic()
    first: float | None = None
    for answer in stream_episodes(
        found, query, frames, boxes, config=config, keep_images=keep_images, timings=timings
    ):
        # From the question, not from the detector: this is when someone watching the
        # module would have seen the answer appear.
        answer.arrived = time.monotonic() - asked
        if first is None:
            first = answer.arrived
        yield answer
    if timings is not None:
        timings["detect"] = time.monotonic() - at
        # From the QUESTION, like `arrived`, and for the same reason. This used to be
        # measured from the start of detection, which quietly omitted the search and the
        # episode grouping in front of it -- it reported 0.4 s for a wait that was
        # really 7. The number a person can check against a stopwatch is the only honest
        # one, and on this recording search is most of it.
        timings["first_result"] = first or 0.0

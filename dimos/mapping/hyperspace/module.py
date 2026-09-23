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

"""Hyperspace as two memory modules.

``HyperspacePatches`` watches colour + depth + tf, keeps the frames worth
keeping, and writes each kept frame's 576 text-aligned patch embeddings into
the recording's memory store (one vector per patch, so the store's own vector
search finds them). ``Hyperspace`` answers questions against that store: text
in, scored voxels out, placing every keyframe through the recorded tf at query
time: a keyframe stores no pose, so corrected transforms would move old answers
once the transform buffer lets a rewrite win (see test_rewriting_tf_moves_the_answer).
"""

from __future__ import annotations

import asyncio
from dataclasses import replace
import json
import threading
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.mapping.hyperspace import patches as hs
from dimos.mapping.hyperspace.embedder import (
    DEFAULT_MEMBERS,
    SIGLIP2_MODEL_NAME,
    PatchEnsemble,
)
from dimos.mapping.hyperspace.frames import spec_of
from dimos.mapping.hyperspace.ingest import IngestConfig, PatchIngestor, transform_to_matrix
from dimos.mapping.hyperspace.live import LiveConfig, LiveQuery
from dimos.mapping.hyperspace.msgs import FoundObject, FoundObjects
from dimos.mapping.hyperspace.queries import (
    Place,
    Query,
    QueryBook,
    near_enough,
    negative_prompts,
)
from dimos.mapping.hyperspace.query import HyperspaceQuery
from dimos.mapping.hyperspace.refine import refine_config_of
from dimos.memory.module import MemoryModule, MemoryModuleConfig
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Sequence

    from numpy.typing import NDArray

logger = setup_logger()

# What one published answer's evidence frames may weigh. **LCM SILENTLY DROPS A PICKLED
# MESSAGE OVER ~16 MB** -- measured on this transport, 16 MB arrives and 19 MB does not,
# which is the receiver's fragment buffer, not the publisher's limit. `publish` reports
# success either way. An item answer carries one 1280x720 colour frame per place, 2.77 MB
# each, so THREE places fit and SEVEN do not: roscon's "how many fire extinguishers"
# answered 7 places in words while the viewer drew nothing at all, and the failure looked
# like a broken viewer rather than a dropped message. Seven frames at half size is 4.8 MB.
PUBLISHED_FRAME_BUDGET_BYTES = 8 * 1024 * 1024


def _fits_the_transport(result: FoundObjects) -> FoundObjects:
    """The same answer, with its evidence frames scaled down until it will arrive.

    Decimation rather than a resampling filter: this is a thumbnail for a viewer to hang
    beside a box, the caller that wants the real frame has the returned `result`, and a
    dependency-free step keeps this out of the way of the answer itself.
    """
    frames = [found.image for found in result.objects if found.image is not None]
    weight = sum(int(image.data.nbytes) for image in frames)
    if weight <= PUBLISHED_FRAME_BUDGET_BYTES:
        return result
    # Integer decimation, so the step is what the budget asks for rounded UP: landing
    # just over the budget is the one outcome this must not have.
    step = int(np.ceil(np.sqrt(weight / PUBLISHED_FRAME_BUDGET_BYTES)))
    logger.info(
        "hyperspace: %.1f MB of evidence frames is over the %.0f MB a message may carry; "
        "publishing them at 1/%d scale",
        weight / 1e6,
        PUBLISHED_FRAME_BUDGET_BYTES / 1e6,
        step,
    )
    smaller = []
    for found in result.objects:
        if found.image is None:
            smaller.append(found)
            continue
        image = found.image
        smaller.append(
            replace(
                found,
                image=Image(
                    data=np.ascontiguousarray(image.data[::step, ::step]),
                    format=image.format,
                    frame_id=image.frame_id,
                    ts=image.ts,
                ),
                # `box2d` is pixels, so it moves WITH the picture. A subscriber drawing
                # the box on the frame it was handed would otherwise put it at twice the
                # coordinates, off the edge; the two only mean anything together.
                box2d=tuple(value / step for value in found.box2d),
            )
        )
    # `replace` on the envelope too: the caller holds `result` and is still using it.
    return replace(result, objects=smaller)


def pick_device(device: str, *, allow_mps: bool = True) -> str:
    """The fastest device this process can actually use. One answer for the whole package.

    Anything but "auto" is returned verbatim, so a caller can always name its own device.
    `allow_mps=False` keeps "auto" off Metal without naming a device -- see the last
    paragraph for the one stack that needs it.

    APPLE SILICON IS INCLUDED, and it used to be excluded. MEASURED 2026-09-15: the query
    module runs OWLv2 on MPS inside a real dimos forkserver worker and answers sf_office's
    "a chair" in 10.7 s, against ~140 s on the CPU -- and finds the same place. The blanket
    ban that cost that came from `b0d3047904`, "Metal asserts in a forkserver child", which
    turns out to be half of the rule. The whole rule, three repeats each way:

        parent never compiles a Metal kernel -> the worker's MPS works      (exit 0, 3/3)
        parent compiles ONE 64x64 matmul on mps first -> the worker aborts  (SIGABRT, 3/3)

    Importing torch, `mps.is_available()`, even `torch.empty(1, device="mps")` are all
    harmless; it takes a real kernel compile in the parent. A `dimos run` parent starts
    workers and does not run models, so the working case is the normal one.

    There is deliberately no probe. The failure is an abort rather than an exception, so it
    cannot be caught, and a forked probe cannot answer it either: a fork of a worker cannot
    reach MTLCompilerService whether the worker is poisoned or not, so such a probe says
    "no" even when MPS would have worked (measured, both directions). So a stack whose
    parent process does touch Metal says so with `allow_mps=False`, which every module
    carries as a config field -- or names its device outright, which still wins over all
    of this.
    """
    if device != "auto":
        return device
    import torch

    if torch.cuda.is_available():
        return "cuda"
    if allow_mps and torch.backends.mps.is_available():
        return "mps"
    return "cpu"


def depth2depth_model_of(name: str) -> str:
    """ "" = off, "default" = the package's checkpoint, anything else verbatim."""
    if name != "default":
        return name
    from dimos.perception.depth2depth.fusion import DEPTH_MODEL_NAME

    return DEPTH_MODEL_NAME


class HyperspacePatchesConfig(MemoryModuleConfig):
    # The checkpoints that embed every keyframe, Hugging Face ids or local
    # directories, NaFlex ones optionally with "@<patch budget>". More than
    # one makes an ensemble whose scores are pooled per cell at query time
    # (embedder.PatchEnsemble); the default pair is what the size sweep
    # picked. ["google/siglip2-so400m-patch16-384"] is the original single model.
    models: list[str] = DEFAULT_MEMBERS
    # Kept for callers that predate `models`; used only when `models` is empty.
    model_name: str = SIGLIP2_MODEL_NAME
    # "auto" = cuda if available, else cpu (never mps inside a worker, see start()).
    device: str = "auto"
    # Let "auto" pick Metal on Apple silicon. Off is for the one stack that breaks it: a
    # parent process that compiles a Metal kernel before its workers start leaves every
    # worker unable to reach MTLCompilerService, and the worker aborts rather than raising.
    # `pick_device` has the measurement. Naming `device` outright ignores this entirely.
    allow_mps: bool = True
    # Frame the quality gate measures camera motion against.
    motion_reference_frame: str = "odom"
    # Never embed frames closer together than this (s), which is also the ceiling on
    # the keyframe rate: novelty spikes the keeps up to here and no further.
    min_frame_interval_s: float = 1.0 / hs.MAX_KEYFRAME_HZ

    # Keyframe gate. Negative turns a gate off.
    # Frames held after a candidate to swap a blurry one for a sharp one; the rate
    # comes from novelty, not from this.
    lookahead: int = 2
    quality_margin: float = 0.25
    novelty_threshold: float = 0.30
    # Negative = off, which is the default: see KeyframeGateConfig.
    patch_novelty_threshold: float = -1.0
    max_angular_velocity: float = 1.5
    max_linear_velocity: float = -1.0
    max_dark_fraction: float = 0.6
    max_bright_fraction: float = -1.0
    min_keyframe_interval: float = 1.0 / hs.MAX_KEYFRAME_HZ

    # Depth readings beyond this (m) are holes: RealSense frames carry 65535 mm
    # "no reading" sentinels and occasional 20-40 m glitches.
    max_depth_m: float = 10.0
    # Fill those holes from the colour frame before measuring patch depth: a
    # depth-anything checkpoint id (see dimos.perception.depth2depth), or ""
    # for raw sensor depth. "default" takes the package's own checkpoint.
    depth2depth_model: str = ""
    # A colour frame pairs with the depth frame within this many seconds of it.
    depth_max_dt: float = 0.05
    depth_history: int = 64
    # Stride of the depth thumbnail kept per keyframe, for scene rendering.
    depth_thumbnail_stride: int = 4
    # Keep the colour frame behind each embedding frame, so the detector can be shown
    # it later. Required for a LIVE run and pointless for an ingest of a recording:
    # see IngestConfig.keep_frames.
    keep_frames: bool = False
    # The layout the detector path reads. Live it has to be on -- a keyframe blob is
    # not searchable one patch at a time -- so it defaults on here rather than in
    # IngestConfig, which still has every old recording to think about.
    flat: bool = True


def _optional(value: float) -> float | None:
    return None if value < 0 else value


def store_members(store: Any, wait_s: float = 0.0) -> list[str]:
    """The checkpoint specs an existing store's keyframes were embedded with,
    or [] when the store is empty or predates the member record. Waits up to
    ``wait_s`` for the first keyframe when the writer starts alongside."""
    from dimos.mapping.hyperspace.ingest import KEYFRAME_STREAM

    deadline = time.monotonic() + wait_s
    while True:
        if KEYFRAME_STREAM in store.list_streams():
            first = next(iter(store.stream(KEYFRAME_STREAM, dict).order_by("ts")), None)
            if first is not None:
                return list(first.data.get("member_specs", []))
        if time.monotonic() >= deadline:
            return []
        time.sleep(0.5)


def open_store_with_retry(module: MemoryModule, attempts: int = 20, wait_s: float = 0.5) -> None:
    """Touch ``module.store`` until it opens. The writer and the reader start in
    parallel on the same fresh file, and sqlite refuses the second
    ``PRAGMA journal_mode=WAL`` while the first is still creating it."""
    import sqlite3

    for attempt in range(attempts):
        try:
            _ = module.store
            return
        except sqlite3.OperationalError as error:
            if "locked" not in str(error) or attempt == attempts - 1:
                raise
            module._store = None
            time.sleep(wait_s)


class HyperspacePatches(MemoryModule):
    """Keeps the frames worth keeping and writes their patch embeddings to memory."""

    config: HyperspacePatchesConfig

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    tf: In[TFMessage]

    ingestor: PatchIngestor | None = None

    @rpc
    def start(self) -> None:
        # Everything the handlers need exists before Module.start binds them:
        # frames arrive the moment the ports connect.
        # "auto" includes Metal on Apple silicon; `pick_device` carries the measurement
        # and the one parent-process condition that breaks it. `allow_mps=False` opts out.
        device = pick_device(self.config.device, allow_mps=self.config.allow_mps)
        specs = self.config.models or [self.config.model_name]
        logger.info(f"hyperspace patches: loading {specs} on {device}")
        self.model = self.register_disposable(PatchEnsemble(specs, device=device, towers="vision"))
        self.model.start()
        logger.info(f"hyperspace patches: model ready, opening {self.config.db_path}")
        open_store_with_retry(self)
        gate = hs.KeyframeGateConfig(
            lookahead=self.config.lookahead,
            quality_margin=self.config.quality_margin,
            novelty_threshold=self.config.novelty_threshold,
            patch_novelty_threshold=_optional(self.config.patch_novelty_threshold),
            max_angular_velocity=_optional(self.config.max_angular_velocity),
            max_linear_velocity=_optional(self.config.max_linear_velocity),
            max_dark_fraction=_optional(self.config.max_dark_fraction),
            max_bright_fraction=_optional(self.config.max_bright_fraction),
            min_interval=_optional(self.config.min_keyframe_interval),
        )
        self.ingestor = PatchIngestor(
            self.store,
            self.model,
            IngestConfig(
                gate=gate,
                motion_reference_frame=self.config.motion_reference_frame,
                min_frame_interval_s=self.config.min_frame_interval_s,
                max_depth_m=self.config.max_depth_m,
                depth2depth_model=depth2depth_model_of(self.config.depth2depth_model),
                depth_max_dt=self.config.depth_max_dt,
                depth_history=self.config.depth_history,
                depth_thumbnail_stride=self.config.depth_thumbnail_stride,
                keep_frames=self.config.keep_frames,
                flat=self.config.flat,
            ),
            lookup=self._lookup,
        )
        logger.info(f"hyperspace patches: {self.model.tags} on {device}, db {self.config.db_path}")
        super().start()

    def _lookup(self, target: str, source: str, ts: float) -> NDArray[np.float64] | None:
        try:
            transform = self.tfbuffer.get(target, source, ts, warn=False)
        except (TypeError, RuntimeError):
            return None
        return None if transform is None else transform_to_matrix(transform)

    async def handle_camera_info(self, info: CameraInfo) -> None:
        if self.ingestor is not None:
            self.ingestor.add_camera_info(info)

    async def handle_depth_camera_info(self, info: CameraInfo) -> None:
        if self.ingestor is not None:
            self.ingestor.add_camera_info(info)

    async def handle_tf(self, msg: TFMessage) -> None:
        if self.ingestor is not None:
            self.ingestor.add_tf(msg)

    async def handle_depth_image(self, image: Image) -> None:
        if self.ingestor is not None:
            self.ingestor.add_depth(image)

    async def handle_color_image(self, image: Image) -> None:
        if self.ingestor is None:
            return
        if self.ingestor.stats["images"] == 0:
            logger.info(
                f"hyperspace patches: first colour frame {image.frame_id} {image.width}x{image.height} {image.format}"
            )
        # Embedding blocks for ~50 ms on a GPU and ~1 s on a CPU; the handler's
        # latest-only dispatch drops the frames that arrive meanwhile.
        await asyncio.get_running_loop().run_in_executor(None, self.ingestor.add_image, image)

    @rpc
    def flush(self) -> int:
        """End of stream: judge what is still buffered. Returns keyframes added."""
        return self.ingestor.flush() if self.ingestor is not None else 0

    @rpc
    def ingest_stats(self) -> dict[str, int]:
        return dict(self.ingestor.stats) if self.ingestor is not None else {}


class HyperspaceConfig(MemoryModuleConfig):
    # Text towers to load: normally left empty, and taken from the store's
    # keyframes (written by HyperspacePatches, whose `models` decide). Set it
    # only to query a store that predates the member record.
    models: list[str] = []
    model_name: str = SIGLIP2_MODEL_NAME
    # "auto" = cuda if available, else cpu (never mps, see start()).
    device: str = "auto"
    # Let "auto" pick Metal on Apple silicon. Off is for the one stack that breaks it: a
    # parent process that compiles a Metal kernel before its workers start leaves every
    # worker unable to reach MTLCompilerService, and the worker aborts rather than raising.
    # `pick_device` has the measurement. Naming `device` outright ignores this entirely.
    allow_mps: bool = True
    # Ensemble stores: how the members' cell scores combine ("min", "2nd",
    # "mean") and the threshold on the pooled score. See QueryConfig.
    pool: str = "min"
    pooled_hot_threshold: float = 0.005
    # Frame answers are given in unless a request names another.
    world_frame: str = "odom"
    # The robot's own link, for a proximity query's "within N metres of me". Its pose
    # comes from tf like everything else; when tf cannot give it, the radius is reported
    # as not applied rather than measured from the origin.
    robot_frame: str = "base_link"
    voxel_size: float = 0.10
    hot_threshold: float = 0.02
    max_hot_patches: int = 6000
    cap_near: float = 0.99
    cap_far: float = 1.01
    # Comma separated; empty uses the indoor defaults.
    background_prompts: str = ""
    # The segment channel (HyperspaceSegments records) added on top; 0 = off.
    segment_weight: float = 1.0
    segment_min_z: float = 2.0
    # Refinement chain (see refine.py); "default" = QueryConfig.refine, "none" = raw map.
    refine: str = "default"
    # Keyframes a voxel must be seen from (the chain's "support" step). 2 was
    # the single-model setting; an ensemble's hot cells are already vetted by
    # several checkpoints and sit tighter on the object, so their thin
    # pyramids overlap less between views: 1 keeps the recall (plan.md 7).
    refine_min_frames: int = 1
    # Depth samples a voxel needs to appear in scene_map.
    scene_min_samples: int = 3
    # --- the detector path (`find_objects`) -------------------------------------
    # OWLv2's per-box acceptance score. See DetectConfig.threshold for what half
    # costs: it is a real refusal, and the scores are not comparable across words.
    owl_threshold: float = 0.5
    # An OWLv2 checkpoint other than the base ensemble. "" = DetectConfig's own.
    owl_checkpoint: str = ""
    # Where the detector runs. Separate from `device`, which is the text towers':
    # the towers are small enough for a cpu and OWLv2 is not.
    #
    # "auto" now includes Apple silicon -- see `pick_device`, which carries the measurement
    # and the one condition that breaks it. An item query that took 143.7 s on this Mac's
    # CPU takes 10.7 s on its GPU, which is the difference between demoable and not.
    #
    # If a stack does hit the Metal case and falls back to the CPU, two things follow:
    # * A 140 s query does not survive the RPC layer. `ModuleConfig.default_rpc_timeout`
    #   and `rpc_timeouts` are 120 s, so `rpc_timeouts={"start_item_query": 600.0}` is
    #   needed -- and an MCP client in front of that may have a cap of its own.
    # * The cost is `max_episodes` x `detect_attempts` forward passes, both config below,
    #   and a REFUSAL spends all of its attempts. 12 x 3 is the worst case. Cutting them
    #   is the first lever on a CPU box and it trades recall for latency, so count the
    #   places before and after rather than assuming only the clock moved.
    owl_device: str = "auto"
    # WHAT THIS RECORDING CALLS ITS CAMERA STREAMS. Empty uses `RecordingFrames`'
    # defaults -- `color_image`, `depth_image`, `camera_info`, `depth_camera_info` -- which
    # is not what every recording calls them, and GUESSING WRONG FAILS SILENTLY: measured
    # on roscon_jpeg.db (`realsense_color_image`, `realsense_depth_image`), every episode
    # came back "refused" in 282 ms with no forward pass, because there were no frames to
    # look at, and the answer read as the detector declining. `LiveConfig` has carried
    # these four fields all along; the module simply never passed them on, so no blueprint
    # could set them.
    color_stream: str = ""
    depth_stream: str = ""
    color_info_stream: str = ""
    depth_info_stream: str = ""
    # Take the detector's candidates from where several members AGREE, rather than from
    # time-split episodes. On by default (it is a smaller and cleaner set), and it needs
    # more than one member searched to mean anything at all -- which is why a
    # single-member search behaves like this being off. Measured wrong for roscon: see
    # `memory_world/blueprints.py`, where it is turned off with the numbers.
    agreement: bool = True
    # Models the frames-first search ranks with. [] = every model in the index,
    # which is what the three-way agreement wants.
    # MEMBER TAGS to search, as `dimos map live --models` takes them (for example
    # "base_patch16_224"), NOT Hugging Face checkpoint names. Empty searches every model
    # the recording holds, which is what the cross-model agreement wants.
    detect_models: list[str] = []
    # Search this member over the whole index and score the others only on the frames it
    # liked; "auto" picks the cheapest member. See `DetectConfig.rank_with` for the
    # measurements and `rank_frames` for the cut that makes it worth anything.
    rank_with: str = ""
    rank_frames: int = 400
    # Episodes to spend a detector call on, frames of each to try, and frames per
    # forward pass. The detector is ~90% of a query, so these are the cost.
    max_episodes: int = 12
    detect_attempts: int = 3
    detect_batch: int = 1
    min_episode_frames: int = 2
    episode_gap_s: float = 1.0
    # Depth spread that is still the object (m), and how close two answers have to
    # be to be one place.
    depth_band_m: float = 0.5
    merge_m: float = 0.75
    # Fill stereo's holes as boxes are placed, for a store nobody has run `fill_depth`
    # over: "auto" only when there is no filled-depth stream, "" off, or a checkpoint
    # by name. Live there usually is one -- the depth2depth module writes it as the
    # robot drives -- and then this costs nothing.
    detect_depth2depth: str = "auto"
    # Where the text towers run. "cpu" by default: on an 8 GB card three of them leave
    # the detector no room at all. See `LiveConfig.tower_device` for the measurement.
    tower_device: str = "cpu"
    # How a heatmap or area cell is scored, and how big a cell is. MEASURED on "kitchen"
    # over sf_office_drive1 (2026-09-14) against the kitchen's real rectangle, under four
    # different contrasts: summing every patch that lands in a 10 cm cell -- what this
    # did -- put the best answer 7.5 to 9.7 m outside the kitchen every time, because a
    # sum rewards a cell that many patches graze over one that a few patches match well,
    # and a wall band seen from across the room collects more patches than a kitchen does.
    # The mean over 25 cm cells seen from three or more viewpoints put the best answer
    # INSIDE the kitchen under all four contrasts, with 7-8 of its top ten in there
    # against 2-4 before. The view gate is what keeps a single stray patch from winning a
    # cell of its own: mean alone answered with one patch seen once.
    heat_cell_m: float = 0.25
    # "mean" or "sum". Sum is what a dense occupancy map wants and it is kept for that.
    heat_score: str = "mean"
    heat_min_views: int = 3
    # There were two more filters here -- ignore patches under 30% of the query's own
    # best, then keep only cells with eight neighbours within half a metre -- and they
    # worked: on "kitchen" over sf_office_drive1 they cut 290 cells to 7, all 7 inside
    # the kitchen's real rectangle against 268 outside it before. They are gone anyway,
    # because they were treating a symptom. The 268 wrong cells were walls, and they were
    # wrong because an area query subtracted object-ness INSTEAD OF surfaces rather than
    # as well as them (see `queries.AREA_PROMPTS`); fixing that took the same 268 to 29
    # at the source. What was left for the filters to do was small enough that keeping
    # them meant carrying two numbers nobody would ever revisit, in front of a stage that
    # no longer needed them.
    # Subtract generic floor/wall/ceiling prompts from every patch score. See
    # `DetectConfig.contrast`; off is the right answer when the query IS a wall.
    contrast: bool = True
    # Depth past this (m) is a hole, not a reading -- RealSense frames carry 65535 mm
    # "no reading" sentinels. The same number `HyperspacePatchesConfig` uses, and the
    # detector needs its own copy because it places boxes off depth the ingest never saw.
    # NOTE it is an INDOOR number: bike.db is an outdoor ride and the 10 m cut refused a
    # real cone at 11 m. Raise it deliberately for outdoor recordings.
    max_depth_m: float = 10.0

    # Demo: after this many seconds, run demo_queries and publish the answers,
    # then repeat every demo_every_s. 0 disables.
    demo_after_s: float = 0.0
    demo_every_s: float = 30.0
    demo_queries: list[str] = ["a chair"]


class Hyperspace(MemoryModule):
    """Ask the map where something is; get scored voxels back.

    Queries arrive on ``query`` as JSON (``{"id": 7, "text": "a chair"}``) or
    bare text, or through the ``find`` skill. Answers go out on
    ``query_result`` (voxel centers with an ``intensity`` score, for viewers)
    and ``query_answer`` (JSON: the request id, the text, the voxel count and
    the best voxels). A caller pairs answers to requests by the id in
    ``query_answer``; there is no RPC and no blocking call.
    """

    config: HyperspaceConfig

    query: In[String]
    tf: In[TFMessage]
    query_result: Out[PointCloud2]
    query_answer: Out[String]
    scene_map: Out[PointCloud2]
    # Every answer `find_objects` gives, also published, so a viewer or a recorder
    # sees them without having made the call.
    found: Out[FoundObjects]

    @rpc
    def start(self) -> None:
        # What every question needs: the store, and which checkpoints this recording was
        # embedded with. The dense voxel path is NOT built here -- see the `engine`
        # property for why, and for what it costs when it is.
        logger.info(f"hyperspace query: opening {self.config.db_path}")
        open_store_with_retry(self)
        specs = self.config.models or store_members(self.store) or [self.config.model_name]
        self._specs = specs
        self._lock = threading.Lock()
        self._ids = iter(range(1, 1 << 30))
        self._load_the_detector(specs)
        logger.info(f"hyperspace query: ready over {self.config.db_path}")
        super().start()

    def _load_the_detector(self, specs: list[str]) -> None:
        """Everything `find_objects` needs, loaded now rather than on the first question.

        OWLv2 is two and a half seconds to bring up and the text towers thirteen, and
        the transforms are a pass over the store. Left lazy, all of that lands on
        whoever asks first -- which on a robot is someone waiting on an answer.
        """
        from dimos.mapping.hyperspace.detect import DetectConfig

        device = pick_device(self.config.owl_device, allow_mps=self.config.allow_mps)
        self.live = LiveQuery(
            self.store,
            LiveConfig(
                detect=DetectConfig(
                    threshold=self.config.owl_threshold,
                    checkpoint=self.config.owl_checkpoint or DetectConfig.checkpoint,
                    device=device,
                    attempts=self.config.detect_attempts,
                    batch=self.config.detect_batch,
                    max_episodes=self.config.max_episodes,
                    min_episode_frames=self.config.min_episode_frames,
                    episode_gap_s=self.config.episode_gap_s,
                    depth_band_m=self.config.depth_band_m,
                    depth2depth=self.config.detect_depth2depth,
                    max_depth_m=self.config.max_depth_m,
                    world_frame=self.config.world_frame,
                    contrast=self.config.contrast,
                    agreement=self.config.agreement,
                    rank_with=self.config.rank_with,
                    rank_frames=self.config.rank_frames,
                ),
                models=list(self.config.detect_models),
                merge_m=self.config.merge_m,
                tower_device=self.config.tower_device,
                color_stream=self.config.color_stream,
                depth_stream=self.config.depth_stream,
                color_info_stream=self.config.color_info_stream,
                depth_info_stream=self.config.depth_info_stream,
            ),
        )
        self.register_disposable(self.live)
        # The tags this query will actually search, turned into the checkpoints behind
        # them. `detect_models` names MEMBER TAGS, the same thing `--models` takes and
        # the same thing `LiveConfig.models` filters on -- it used to be handed to
        # `warm()` as if it were a list of checkpoints, so setting it at all sent a tag
        # like "base_patch16_224" to the Hugging Face hub as a repository name and the
        # module died on a 404. Empty means every model the store holds.
        wanted = [spec_of(tag) for tag, _ in self.live.members()] or specs
        loaded = self.live.warm(wanted)
        logger.info(
            f"hyperspace detect: OWLv2 on {device} in {loaded['detector']:.1f}s, text towers "
            f"in {loaded['towers']:.1f}s, {int(loaded['index'])} patches already indexed, "
            f"threshold {self.config.owl_threshold}"
        )

    @rpc
    def find_objects(self, text: str, top: int = 0) -> FoundObjects:
        """Where is `text`? Boxes in the world, with the frames that found them.

        The whole chain, on the map as it stands this instant: the patch index ranks
        frames, the models' agreement picks the places worth looking at, OWLv2 draws a
        box on the best frame of each, and that box plus the frame's depth becomes a box
        in the world. Answers come back strongest-first, one per place.

        The same object `dimos map live` drives, so what a page shows offline is what
        the robot does.
        """
        self.live.config.top = top
        result = self.live.ask(text)
        self.found.publish(_fits_the_transport(result))
        return result

    def answer(
        self, text: str, request_id: int | None = None, frame: str | None = None
    ) -> dict[str, Any]:
        """Run one query and publish it on both outputs. Returns the JSON answer."""
        with self._lock:
            request_id = next(self._ids) if request_id is None else request_id
            started = time.monotonic()
            answer = self.engine.answer(text, request_id, frame)
            result: hs.Heatmap = answer.pop("heatmap")
            cloud = PointCloud2.from_numpy(
                result.centres().astype(np.float32),
                frame_id=result.frame,
                timestamp=time.time(),
                intensities=result.scores().astype(np.float32),
            )
            self.query_result.publish(cloud)
            answer["ms"] = round((time.monotonic() - started) * 1000)
            self.query_answer.publish(String(json.dumps(answer)))
            logger.info(
                f"hyperspace {text!r}: {answer['voxels']} voxels in {answer['ms']} ms {result.stats}"
            )
            return answer

    async def handle_tf(self, msg: TFMessage) -> None:
        # The subscription TF(port, buffer_size=inf) would make, without its
        # transport timing: transforms land in the buffer the answers read,
        # which the store still tops up with whatever predates this module.
        #
        # Only if the dense path has actually been built. Touching `self.engine` here
        # would construct it -- a text tower and its GPU memory -- on the first
        # transform that arrived, which is to say immediately, for a path most callers
        # never use. One built later reads the transforms back out of the store itself.
        built = getattr(self, "_engine", None)
        if built is not None:
            built.tf.receive_tfmessage(msg)

    async def handle_query(self, msg: String) -> None:
        payload = msg.data.strip()
        request_id, text, frame = None, payload, None
        if payload.startswith("{"):
            try:
                parsed = json.loads(payload)
                request_id = int(parsed.get("id", 0))
                text = str(parsed.get("text", "")).strip()
                frame = parsed.get("frame") or None
            except (ValueError, TypeError, AttributeError) as error:
                logger.warning(f"hyperspace ignored a query: {error}: {payload!r}")
                return
        if not text:
            return
        await asyncio.get_running_loop().run_in_executor(None, self.answer, text, request_id, frame)

    # --- the query surface ------------------------------------------------------
    #
    # Three ways to ask, one implementation, and a handle to come back to. An agent that
    # blocks until every place is found waits ten seconds for an answer whose first line
    # was ready in two, so every start returns what it has and leaves the rest behind an
    # id. See `queries.py` for what separates the three kinds.

    @property
    def engine(self) -> HyperspaceQuery:
        """The dense voxel path, built the first time something asks for it.

        It used to be built in `start()`, which meant every module paid a so400m text
        tower and its GPU memory at boot -- on an 8 GB card, beside OWLv2 and the patch
        index, that is most of what there is. The three query skills do not touch it:
        they search the per-model indexes the recording holds. Only `answer()` and the
        `query` port, which are the older JSON-in-voxels-out path, still do.
        """
        built = getattr(self, "_engine", None)
        if built is not None:
            return built

        device = pick_device(self.config.device, allow_mps=self.config.allow_mps)
        logger.info(f"hyperspace query: loading text towers {self._specs} on {device}")
        self.model = self.register_disposable(
            PatchEnsemble(self._specs, device=device, towers="text")
        )
        self.model.start()
        query_config = hs.QueryConfig(
            hot_threshold=self.config.hot_threshold,
            pool=self.config.pool,
            pooled_hot_threshold=self.config.pooled_hot_threshold,
            max_hot_patches=self.config.max_hot_patches,
            cap_near=self.config.cap_near,
            cap_far=self.config.cap_far,
            segment_weight=self.config.segment_weight,
            segment_min_z=self.config.segment_min_z,
        )
        prompts = [p.strip() for p in self.config.background_prompts.split(",") if p.strip()]
        if prompts:
            query_config.background_prompts = prompts
        built = self._engine = HyperspaceQuery(
            self.store,
            self.model.embed_text,
            query_config,
            world_frame=self.config.world_frame,
            voxel_size=self.config.voxel_size,
            refine_config=refine_config_of(
                self.config.refine, query_config.refine, min_frames=self.config.refine_min_frames
            ),
        )
        return built

    @property
    def queries(self) -> QueryBook:
        """Questions asked and what was found for them, so an agent can come back for
        the rest instead of waiting for all of it up front."""
        book = getattr(self, "_queries", None)
        if book is None:
            book = self._queries = QueryBook()
        return book

    @rpc
    def run_query(
        self,
        text: str,
        kind: str = "item",
        count: int = 1,
        query_id: str = "",
        within_m: float = 0.0,
        at_time: float = 0.0,
        negative_terms: str = "",
        episodes: int = 0,
    ) -> Query:
        """Ask once. The one implementation the three skills wrap.

        `kind` is "item" (OWLv2 draws a box), "heatmap" (hot patches into voxels, no
        detector) or "area" (a room, contrasted against objects rather than against the
        room). `within_m` keeps only places that close to where the robot is -- or to
        where it was at `at_time`, which is how "what was near me when that happened"
        is asked. `query_id` names the question so a second call cannot be mistaken for
        the first; one is made up when it is not given.

        `negative_terms` is a comma separated list of what to subtract -- what the caller
        expects to be in the way. Empty uses the kind's own default. See
        `queries.negative_prompts`.
        """
        text = text.strip()
        if not text:
            raise ValueError("a query needs something to look for")
        if kind not in ("item", "heatmap", "area"):
            raise ValueError(f"unknown query kind {kind!r}; item, heatmap or area")

        negatives = negative_prompts(negative_terms, kind)
        query = Query(
            query_id=query_id or self.queries.next_id(kind),
            text=text,
            kind=kind,
            negatives=negatives or (),
        )
        started = time.monotonic()
        if kind == "item":
            self._fill_from_detector(query, negatives, episodes)
        else:
            self._fill_from_patches(query, kind, negatives)
        query.ms = (time.monotonic() - started) * 1000

        # Only when a radius was actually asked for: finding the robot means a walk of
        # the transform tree, and a query that never mentioned proximity should not pay
        # for it -- nor hang on it, which is what happens when the stamp asked for is
        # outside everything the recording holds.
        origin = self._where_the_robot_is(at_time) if within_m else None
        if within_m and origin is None:
            query.note = (
                f"asked for places within {within_m:.1f} m but the robot's own position "
                "is not known, so the radius was not applied"
            )
        elif within_m:
            before = len(query.places)
            query.places = near_enough(query.places, origin, within_m)
            if before and not query.places:
                query.note = f"{before} place(s) found, none within {within_m:.1f} m"
        if count > 0:
            query.taken = min(count, len(query.places))
        self.queries.put(query)
        logger.info(
            f"hyperspace {kind} {text!r} [{query.query_id}]: {len(query.places)} place(s), "
            f"{query.refused} refused, {query.ms:.0f} ms"
        )
        return query

    def _fill_from_detector(
        self, query: Query, negatives: Sequence[str] | None = None, episodes: int = 0
    ) -> None:
        """The OWLv2 path: boxes, and a detector that is allowed to say no.

        `episodes` is for this question alone and is PUT BACK afterwards: one LiveQuery
        serves every question, so a counting question that left its budget behind would
        quietly make every later question four times slower.
        """
        self.live.config.top = 0
        budget = self.live.config.detect.max_episodes
        # RAISE ONLY. The failure this exists for is a count that was silently capped, and
        # a caller who asks for fewer looks than the recording is configured for would
        # recreate it -- as this did on its first run, asking for 24 against a blueprint
        # that had already been raised to 30.
        if episodes > budget:
            self.live.config.detect.max_episodes = int(episodes)
            logger.info(f"hyperspace item {query.text!r}: {episodes} looks, not {budget}")
        try:
            result = self.live.ask(query.text, background_prompts=negatives)
        finally:
            self.live.config.detect.max_episodes = budget
        result.kind = "item"
        self.found.publish(_fits_the_transport(result))
        query.refused = result.refused
        query.timings = dict(result.timings)
        query.places = [
            Place(
                where=tuple(float(value) for value in found.centre),
                frame=found.frame,
                kind="item",
                score=float(found.confidence),
                distance_m=float(found.depth_m),
                extent=tuple(float(value) for value in found.extent),
                views=int(found.views),
                seen_at=float(found.stamp),
            )
            for found in result.objects
        ]
        if not query.places and result.refused:
            # The difference an agent most needs: the search DID find frames worth
            # looking at, and the detector would not draw a box on any of them. On
            # "kitchen" over an office recording every one of the top frames was the
            # kitchen and OWLv2 refused all twelve, because it boxes objects and a
            # kitchen is a place. Answering "nothing found" there is a lie.
            query.note = (
                f"the search found {result.refused} place(s) worth looking at and the "
                f"detector would not draw a box on any of them. {query.text!r} may be a "
                "place rather than a thing -- try an area or heatmap query"
            )

    def _fill_from_patches(
        self, query: Query, kind: str, negatives: Sequence[str] | None = None
    ) -> None:
        """No detector: the hot patches themselves, put into the world.

        Nothing here can refuse, which is the point. It answers where a box cannot -- a
        query the detector has no word for, and anything that is not an object.

        The search is the SAME one the item path uses: `hot_frames` over the per-model
        indexes this recording actually holds. The dense engine was used here first and
        it answered "nothing scored for kitchen" on a recording where the kitchen is
        plainly found, because it searches whichever checkpoint the module's own config
        names -- so400m-384 against a store indexed with four other checkpoints. Two
        searches over two layouts is one search too many.
        """
        import numpy as np

        from dimos.mapping.hyperspace.frames import hot_frames

        started = time.monotonic()
        frames = hot_frames(
            self.store,
            query.text,
            towers=self.live.towers,
            models=[tag for tag, _ in self.live.members()],
            resident=self.live.held,
            contrast=self.config.contrast,
            background_prompts=negatives,
        )
        query.timings = {"search": time.monotonic() - started}
        if not frames:
            query.note = f"no patch anywhere in the map scored for {query.text!r}"
            return

        # Every hot patch, placed where its own frame was looking. A patch carries the
        # ray it sat on and how far the depth said that was, which is the same arithmetic
        # the box placer does -- see `object_points`.
        size = self.config.heat_cell_m or self.config.voxel_size
        placed: list[tuple[tuple[int, int, int], float, tuple[str, float]]] = []
        # Every viewpoint that put evidence in a cell, gates or no gates: the answer says
        # how much was looked at, not how much of it survived a filter.
        seen: dict[tuple[int, int, int], set[tuple[str, float]]] = {}
        when: dict[tuple[int, int, int], float] = {}
        for frame in frames:
            pose = self.live.frames.pose(frame.frame, frame.ts, self.config.world_frame)
            if pose is None:
                continue
            pose = np.asarray(pose, dtype=float)
            for hit in frame.hits:
                if hit.depth <= 0:
                    continue
                here = pose @ np.array(
                    [hit.ray[0] * hit.depth, hit.ray[1] * hit.depth, hit.depth, 1.0]
                )
                # A patch whose ray or pose carries a NaN has no position, and binning
                # it raises rather than returning nothing -- one bad patch out of two
                # million took the whole query down.
                if not np.isfinite(here[:3]).all():
                    continue
                cell = (
                    int(np.floor(here[0] / size)),
                    int(np.floor(here[1] / size)),
                    int(np.floor(here[2] / size)),
                )
                placed.append((cell, float(hit.score), (frame.frame, frame.ts)))
                seen.setdefault(cell, set()).add((frame.frame, frame.ts))
                when[cell] = min(when.get(cell, frame.ts), frame.ts)
        if not placed:
            query.note = (
                f"{len(frames)} frame(s) scored for {query.text!r} and none of them could "
                "be placed -- no pose, or no depth behind the patches"
            )
            return

        # How well this patch of space matches, not how much of it there is. Nothing is
        # discarded here beyond the view gate: the two filters that used to sit at this
        # point -- a floor at 30% of the query's own best patch, then a demand for eight
        # neighbours within half a metre -- were compensating for a contrast that
        # subtracted nothing relevant, and once an area query subtracted surfaces again
        # they were removing an order of magnitude less than the contrast was. Removed
        # rather than retuned, on the grounds that a filter whose job another stage now
        # does is a filter that will be wrong somewhere nobody is measuring.
        # The view gate stays, and so does its giving way: a short recording may have no
        # cell seen from three viewpoints at all, and "here, weakly" with the reason
        # attached beats silence.
        scored = self._cells_worth_answering(placed, self.config.heat_min_views)
        if not scored:
            scored = self._cells_worth_answering(placed, 1)
            query.note = (
                f"nothing was seen from {self.config.heat_min_views} viewpoints, so this "
                "answer rests on less than it should -- a single look can be a single "
                "mistake"
            )
        ranked = sorted(scored.items(), key=lambda item: -item[1])
        query.places = [
            Place(
                where=((cell[0] + 0.5) * size, (cell[1] + 0.5) * size, (cell[2] + 0.5) * size),
                frame=self.config.world_frame,
                kind=kind,
                score=score,
                views=len(seen[cell]),
                seen_at=when[cell],
            )
            for cell, score in ranked[:50]
        ]
        # Published like the detector path's answers, because a viewer that sees one kind
        # of answer out of three makes `found`'s promise -- "every answer, also published,
        # so a viewer or a recorder sees them without having made the call" -- half true.
        # MemWorld hit exactly that: heatmap and area answered correctly in words over an
        # empty world, which reads as the world being broken rather than the wiring.
        #
        # A cell is NOT a detection and the message says so in `kind`. There is no extent,
        # because nothing measured one; `confidence` carries the cell's score, which is a
        # different quantity from OWLv2's calibrated one and is only comparable within a
        # kind. `camera_frame` and `box2d` stay empty: no photograph was chosen and no box
        # was drawn. `seen_at` is a real frame stamp, which is what a viewer needs to pull
        # the evidence picture.
        self.found.publish(
            FoundObjects(
                query=query.text,
                kind=kind,
                frame=self.config.world_frame,
                objects=[
                    FoundObject(
                        frame=place.frame,
                        centre=place.where,
                        confidence=place.score,
                        views=place.views,
                        stamp=place.seen_at,
                        depth_m=place.distance_m,
                    )
                    for place in query.places
                ],
                ms=query.ms,
                timings=dict(query.timings),
            )
        )

    def _cells_worth_answering(
        self,
        placed: Sequence[tuple[tuple[int, int, int], float, tuple[str, float]]],
        min_views: int,
    ) -> dict[tuple[int, int, int], float]:
        """One score per cell, from every patch that landed in it.

        The mean, not the sum: a wall band driven past a hundred times collects more
        patches than a kitchen looked at ten times, and summing answers with whichever
        surface the camera saw most of. *min_views* is the only thing discarded here --
        a cell one patch reached once is a cell one mistake reached once.
        """
        weight: dict[tuple[int, int, int], float] = {}
        counted: dict[tuple[int, int, int], int] = {}
        seen: dict[tuple[int, int, int], set[tuple[str, float]]] = {}
        for cell, score, key in placed:
            weight[cell] = weight.get(cell, 0.0) + score
            counted[cell] = counted.get(cell, 0) + 1
            seen.setdefault(cell, set()).add(key)
        mean = self.config.heat_score != "sum"
        return {
            cell: (total / counted[cell] if mean else total)
            for cell, total in weight.items()
            if len(seen[cell]) >= min_views
        }

    def _where_the_robot_is(self, at_time: float = 0.0) -> tuple[float, float, float] | None:
        """The robot's own position, now or at a moment, or None when nothing knows.

        None rather than the origin: a radius measured from a position nobody recorded
        would quietly answer about the wrong part of the map.
        """
        frames = getattr(self.live, "frames", None)
        if frames is None:
            return None
        try:
            pose = frames.pose(
                self.config.robot_frame, at_time or time.time(), self.config.world_frame
            )
        except Exception as error:
            logger.debug(f"hyperspace has no pose for the radius: {error}")
            return None
        if pose is None:
            return None
        return (float(pose[0][3]), float(pose[1][3]), float(pose[2][3]))

    @skill
    def start_item_query(
        self,
        text: str,
        count: int = 1,
        query_id: str = "",
        negative_terms: str = "",
        episodes: int = 0,
    ) -> SkillResult:
        """Find a THING and get its position and size. E.g. "a fire extinguisher".

        Returns the strongest place immediately; ask `query_results` with the returned
        `query_id` for the others. A detector draws the box, so it can refuse -- which is
        what makes a box worth trusting, and why a place-like query ("the kitchen") is
        better asked with `start_area_query`.

        `negative_terms` is a comma separated list of what is in the way -- things the
        search should subtract rather than return, like "a poster, a screen" when asking
        for a thing that is often pictured. Empty subtracts generic room surfaces.

        `episodes` RAISES the number of looks for this question alone; anything at or
        below the configured budget leaves it as it is. A count is only ever a floor: the
        detector stops at the budget and refuses most of what it looks at, so the places
        reported are what was afforded and not what is there. Measured on roscon,
        "a fire extinguisher", one index member: 6 looks found 5 places, 12 found 5, 30
        found 7 and 60 found the same 7. The blueprint is set to 30 for that reason, so
        raising it further buys little -- searching BOTH members at 30 finds a different
        seven, which says the limiter past this point is which episodes the ranking puts
        on top rather than how many are looked at.
        """
        return self._answer_with(
            text, "item", count, query_id, negative_terms=negative_terms, episodes=episodes
        )

    @skill
    def start_heatmap_query(
        self,
        text: str,
        count: int = 1,
        query_id: str = "",
        within_m: float = 0.0,
        at_time: float = 0.0,
        negative_terms: str = "",
    ) -> SkillResult:
        """Where does the map LOOK LIKE `text`? Positions, no boxes, nothing refuses.

        The patch scores go straight into the world, so this answers when no detector
        has a word for what is being asked. Set `within_m` to keep only places that
        close to the robot, and `at_time` to measure that from where it was at a moment
        rather than from where it is now.

        `negative_terms` is a comma separated list of what to subtract instead of the
        generic room surfaces -- what the caller expects to be in the way.
        """
        return self._answer_with(
            text, "heatmap", count, query_id, within_m, at_time, negative_terms
        )

    @skill
    def start_area_query(
        self, text: str, count: int = 1, query_id: str = "", negative_terms: str = ""
    ) -> SkillResult:
        """Find a PLACE rather than a thing. E.g. "kitchen", "the loading dock".

        Same search as a heatmap, contrasted against objects instead of against the room,
        so asking for a room does not subtract the room. Use this when the answer is
        somewhere to go rather than something to pick up.

        `negative_terms` is a comma separated list of what to subtract instead of the
        default object contrast -- naming the rooms this one is NOT ("an office, a
        hallway, a conference room") is the other thing worth trying when an area query
        lands in the wrong part of the building.
        """
        return self._answer_with(text, "area", count, query_id, negative_terms=negative_terms)

    @skill
    def query_results(self, query_id: str, count: int = 0) -> SkillResult:
        """The rest of the answers to a query already started. 0 takes all that are left.

        Each call hands back the ones not handed over yet, so calling twice walks the
        list rather than repeating it.
        """
        query = self.queries.get(query_id.strip())
        if query is None:
            return SkillResult.fail(
                "NO_SUCH_QUERY",
                f"no query {query_id!r}; it was never asked or it has aged out of the "
                f"last {self.queries.keep}. Known: {self.queries.ids()}",
            )
        rest = query.places[query.taken :]
        if count > 0:
            rest = rest[:count]
        query.taken += len(rest)
        return SkillResult.ok(
            f"{len(rest)} more for {query.text!r}"
            + (f", {query.remaining} still held" if query.remaining else ", that is all of them"),
            query_id=query.query_id,
            query=query.text,
            kind=query.kind,
            places=[place.as_dict() for place in rest],
            remaining=query.remaining,
        )

    def _answer_with(
        self,
        text: str,
        kind: str,
        count: int,
        query_id: str,
        within_m: float = 0.0,
        at_time: float = 0.0,
        negative_terms: str = "",
        episodes: int = 0,
    ) -> SkillResult:
        """Every skill's body: ask, then say what came back in one readable line."""
        if not text.strip():
            return SkillResult.fail("INVALID_INPUT", "text must not be empty")
        try:
            query = self.run_query(
                text, kind, count, query_id, within_m, at_time, negative_terms, episodes
            )
        except ValueError as error:
            # Only the argument checks raise ValueError deliberately; anything else that
            # happens to is a fault in here, and calling it INVALID_INPUT tells the
            # caller to fix their question when the question was fine.
            if "query" in str(error) or "look for" in str(error):
                return SkillResult.fail("INVALID_INPUT", str(error))
            logger.exception(f"hyperspace {kind} query {text!r} failed")
            return SkillResult.fail("QUERY_FAILED", f"{type(error).__name__}: {error}")
        except Exception as error:
            logger.exception(f"hyperspace {kind} query {text!r} failed")
            return SkillResult.fail("QUERY_FAILED", f"{type(error).__name__}: {error}")
        handed = query.places[: query.taken]
        if not handed:
            return SkillResult.ok(
                query.note or f"nothing in the map looks like {text!r}",
                query_id=query.query_id,
                query=text,
                kind=kind,
                places=[],
                remaining=0,
                refused=query.refused,
                note=query.note,
            )
        first = handed[0]
        return SkillResult.ok(
            f"{text!r}: {len(query.places)} place(s), best at "
            f"{[round(value, 1) for value in first.where]} in {first.frame} "
            f"(score {first.score:.2f})"
            + (f", {query.remaining} more held as {query.query_id}" if query.remaining else ""),
            query_id=query.query_id,
            query=text,
            kind=kind,
            frame=first.frame,
            places=[place.as_dict() for place in handed],
            remaining=query.remaining,
            refused=query.refused,
            note=query.note,
            negatives=list(query.negatives),
            took_ms=round(query.ms),
        )

    @rpc
    def query_page(self, query_id: str, path: str = "") -> str:
        """Write the page a PERSON looks at for a query already asked. Returns the path.

        Not a skill: an agent cannot read a 3D scene, and handing it three megabytes of
        HTML to summarise would waste the one thing the page is good at. It is for the
        human the agent is working for.
        """
        from pathlib import Path

        from dimos.mapping.hyperspace import render

        query = self.queries.get(query_id.strip())
        if query is None:
            raise ValueError(f"no query {query_id!r}; known: {self.queries.ids()}")
        out = Path(path or f"/tmp/hyperspace_{query.query_id}.html").expanduser()
        answers = self.live.answers if query.kind == "item" else []
        self.live.frames.load_tf()
        render.boxes_html(
            out,
            query.text,
            answers,
            render.scene_points(self.store, self.live.frames.tf, self.config.world_frame),
            render.trajectory(self.store, self.live.frames.tf, self.config.world_frame),
            recording=str(getattr(self.store, "path", "")),
            views=render.camera_views(answers, self.live.frames, self.config.world_frame),
            stats={
                "kind": query.kind,
                "places": len(query.places),
                "refused": query.refused,
                "took": f"{query.ms:.0f} ms",
            },
        )
        return str(out)

    def publish_scene(self, frame: str | None = None) -> int:
        """Occupied voxels from the keyframes' depth thumbnails, for viewers."""
        voxels = self.engine.scene_voxels(frame, self.config.scene_min_samples)
        if not voxels:
            return 0
        size = self.config.voxel_size
        centres = (np.asarray([i for i, _ in voxels], dtype=np.float32) + 0.5) * size
        most = max(n for _, n in voxels)
        self.scene_map.publish(
            PointCloud2.from_numpy(
                centres,
                frame_id=frame or self.config.world_frame,
                timestamp=time.time(),
                intensities=np.asarray([n / most for _, n in voxels], dtype=np.float32),
            )
        )
        return len(voxels)

    async def main(self) -> AsyncIterator[None]:
        # Code before the first yield is startup; the demo loop must not block it.
        demo = None
        if self.config.demo_after_s > 0 and self.config.demo_queries:
            demo = asyncio.create_task(self._demo_loop())
        try:
            yield
        finally:
            if demo is not None:
                demo.cancel()

    async def _demo_loop(self) -> None:
        loop = asyncio.get_running_loop()
        await asyncio.sleep(self.config.demo_after_s)
        while True:
            scene = await loop.run_in_executor(None, self.publish_scene)
            logger.info(f"hyperspace demo: scene_map {scene} voxels")
            for text in self.config.demo_queries:
                answer = await loop.run_in_executor(None, self.answer, text)
                logger.info(
                    f"hyperspace demo {text!r}: {answer['voxels']} voxels, best {answer['best'][:1]}"
                )
            await asyncio.sleep(self.config.demo_every_s)

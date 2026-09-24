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

"""Frames in, keyframes and patch embeddings out, into a memory store.

Shared by the live ``HyperspacePatches`` module and the offline CLI so both
keep exactly the same frames and write exactly the same records.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass
import re
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.mapping.hyperspace import patches as hs
from dimos.models.embedding.base import Embedding
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

    from numpy.typing import NDArray

    from dimos.mapping.hyperspace.siglip_embedder import SigLIP2Patches
    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream

logger = setup_logger()

KEYFRAME_STREAM = "hyperspace_keyframes"
PATCH_STREAM = "hyperspace_patches"
# One row per embedding frame, holding only what the occupancy check needs: a small
# point cloud in the CAMERA'S OWN frame. Its world pose is applied at query time, so a
# loop closure moves it; baking the pose in would leave it silently wrong.
THUMBNAIL_STREAM = "hyperspace_depth_thumbnails"
# Depth with the holes stereo leaves filled in, one frame per keyframe, in millimetres.
# Written once -- live by the depth2depth module, or after the fact by `fill_depth` --
# so that placing a box never pays for a model.
FILLED_STREAM = "hyperspace_filled_depth"
# The colour frame each embedding frame was made from, as JPEG. Live there is no
# recording to go back to -- the camera's 30 Hz never touches disk -- and the detector
# has to be handed the picture of a place the robot drove past minutes ago. Keeping the
# gated 5 Hz is about a gigabyte an hour against the 70 the raw stream would cost.
FRAME_STREAM = "hyperspace_frames"
# The camera the frames came through. Live there is no `camera_info` stream to read --
# the intrinsics arrive on a port and are held in memory -- and without them a 2D box
# cannot be turned into a place in the world. One row per camera, written once.
INFO_STREAM = "hyperspace_camera_info"


def index_slug(specs: Sequence[str]) -> str:
    """The name an index built from these checkpoints goes under.

    ``["google/siglip2-base-patch16-224", "...-256"]`` ->
    ``base_patch16_224__base_patch16_256``. Only letters, digits and underscores,
    because the slug ends up in a stream name and those are SQL identifiers -- the
    hyphens and the ``@``/``#`` of a member spec would be rejected.
    """
    from dimos.mapping.hyperspace.siglip_embedder import member_tag

    return "__".join(sql_safe(member_tag(spec)) for spec in specs)


def sql_safe(text: str) -> str:
    """Letters, digits and underscores only: these end up in stream names."""
    return re.sub(r"[^A-Za-z0-9]+", "_", text).strip("_")


def patch_stream_for(slug: str, member: str) -> str:
    """The vec0 stream holding ONE model's patch vectors.

    Always named after the model, even when there is only one. A stream of vectors is
    unusable without knowing which checkpoint made them -- you cannot encode the query
    text to compare against it -- so the name carries the answer rather than leaving
    it to be looked up somewhere else.
    """
    _, patches = stream_names(slug)
    return f"{patches}__m_{sql_safe(member)}"


def thumbnail_stream_for(slug: str = "") -> str:
    """The depth-thumbnail stream of one index."""
    return THUMBNAIL_STREAM if not slug else f"{THUMBNAIL_STREAM}__{slug}"


def filled_stream_for(slug: str = "") -> str:
    """The filled-depth stream of one index."""
    return FILLED_STREAM if not slug else f"{FILLED_STREAM}__{slug}"


def frame_stream_for(slug: str = "") -> str:
    """The kept-colour-frame stream of one index."""
    return FRAME_STREAM if not slug else f"{FRAME_STREAM}__{slug}"


def info_stream_for(slug: str = "") -> str:
    """The camera-intrinsics stream of one index."""
    return INFO_STREAM if not slug else f"{INFO_STREAM}__{slug}"


def _colour_at(colours: Any, stamps: Sequence[float], tolerance: float = 0.01) -> Any:
    """The colour observation taken at each of these stamps, in order.

    The stamps came off frames of this very stream, so the match is exact in all but
    float rounding -- the tolerance is there for rounding, not for pairing.
    """
    for ts in stamps:
        near = colours.at(ts, tolerance=tolerance).to_list()
        if near:
            yield min(near, key=lambda found: abs(float(found.ts) - ts))


def embedded_stamps(store: Any) -> list[float]:
    """When every frame an index in this store embedded was taken.

    One row per embedding frame lands in the depth-thumbnail stream, so that is where
    the answer is; the union across indexes, because a db can hold several and they do
    not have to have kept the same frames.
    """
    seen: dict[int, float] = {}
    for slug in indexes_in(store) or {"": ""}:
        name = thumbnail_stream_for(slug)
        if name not in store.list_streams():
            continue
        for observation in store.stream(name, dict).order_by("ts"):
            ts = float(observation.ts)
            # A microsecond is finer than any two frames are apart, so this only ever
            # folds together two indexes' record of ONE photograph.
            seen.setdefault(round(ts * 1e6), ts)
    return sorted(seen.values())


def fill_depth(
    recording: Any,
    *,
    model: str = "default",
    device: str = "",
    color_stream: str = "color_image",
    depth_stream: str = "depth_image",
    every: int = 1,
    everywhere: bool = False,
    on_frame: Any = None,
) -> int:
    """Write filled depth for a recording's embedding frames, once, into the recording.

    A box is placed off whatever the stereo returned, and off glass or a dark shelf it
    returns nothing -- which is how a basket ends up metres past where it is. Filling
    the holes costs about fifty milliseconds a frame, far too much to pay while someone
    waits for an answer and nothing at all to pay once.

    Only the frames that were embedded, because those are the only frames a box is ever
    placed off -- grocery has 24110 colour frames and 3462 of them are embedded, so this
    is twenty minutes against three. Pass `everywhere` to fill the whole colour stream
    anyway, which is what a recording with no index yet gets.

    Live, the `depth2depth` module does this as the robot drives and the recorder keeps
    it. This is the same thing after the fact, for a recording that was made without it.
    """
    from dimos.mapping.hyperspace.module import depth2depth_model_of
    from dimos.perception.depth2depth.fusion import Depth2Depth

    name = filled_stream_for("")
    if name in recording.list_streams():
        recording.delete_stream(name)
    out = recording.stream(name, dict)

    fuser = Depth2Depth(model_name=depth2depth_model_of(model), device=device or "auto")
    fuser.start()
    logger.info(f"hyperspace: filling depth with {fuser.model_name} on {fuser.device}")

    # By the stream's own declared type, never Image: grocery's colour is
    # CompressedImage and asking for the wrong one is refused outright.
    colours = recording.streams[color_stream]
    depths = recording.streams[depth_stream]
    wanted = [] if everywhere else embedded_stamps(recording)
    if wanted:
        logger.info(f"hyperspace: {len(wanted)} embedding frames to fill")
        chosen: Any = _colour_at(colours, wanted)
    else:
        chosen = (
            observation
            for index, observation in enumerate(colours.order_by("ts"))
            if not index % max(1, every)
        )

    written = 0
    for observation in chosen:
        colour = decoded(observation.data)
        ts = float(observation.ts)
        near = depths.at(ts, tolerance=0.05).to_list()
        if not near:
            continue
        paired = min(near, key=lambda found: abs(float(found.ts) - ts))
        raw = np.asarray(decoded(paired.data).as_numpy())
        metres = raw.astype(np.float32) * (0.001 if raw.dtype == np.uint16 else 1.0)
        metres[~np.isfinite(metres)] = 0.0
        fused = fuser.fuse(np.asarray(colour.to_rgb().data), metres).fused
        # Millimetres as uint16, the same as the sensor's own: a tenth of the bytes of
        # float metres, and finer than anything the depth is accurate to.
        out.append(
            {
                "camera_frame": colour.frame_id,
                "ts": ts,
                "depth_mm": np.clip(fused * 1000.0, 0, 65535).astype(np.uint16),
            },
            ts=ts,
            tags={"camera_frame": colour.frame_id},
        )
        written += 1
        if on_frame is not None:
            on_frame(written, ts)
    return written


def stream_names(slug: str = "") -> tuple[str, str]:
    """(keyframes, patches) for one index.

    A recording holds as many indexes as you want to compare -- one per model or
    ensemble, each in its own pair of streams, because a vec0 index has one fixed width
    and two checkpoints rarely share it. The first index built keeps the bare names and
    is the canonical one; the rest hang off their slug.
    """
    if not slug:
        return KEYFRAME_STREAM, PATCH_STREAM
    return f"{KEYFRAME_STREAM}__{slug}", f"{PATCH_STREAM}__{slug}"


def indexes_in(store: Any) -> dict[str, str]:
    """Every index in a store: slug -> its keyframe stream. "" is the canonical one.

    The slug of the canonical index is read from its keyframes rather than its name,
    so `indexes_in` never reports two names for one model.
    """
    found: dict[str, str] = {}
    for name in store.list_streams():
        if name == KEYFRAME_STREAM:
            found[""] = name
        elif name.startswith(f"{KEYFRAME_STREAM}__"):
            found[name[len(KEYFRAME_STREAM) + 2 :]] = name
    return found


def index_specs(store: Any, slug: str = "") -> list[str]:
    """The checkpoint specs one index was embedded with, or [] if it is not there."""
    keyframes, _ = stream_names(slug)
    if keyframes not in store.list_streams():
        return []
    first = next(iter(store.stream(keyframes, dict).order_by("ts")), None)
    return [] if first is None else list(first.data.get("member_specs", []))


# Written last by an ingest and dropped first, so a run killed outright reads as
# unfinished: keyframes go into the recording one at a time, so their presence alone
# cannot say the ingest finished. Same convention as memory_world's ingest.
COMPLETE_STREAM = "hyperspace_complete"
TF_STREAM = "tf"

# 4x4 target_from_source at a time, or None when the lookup fails.
TransformLookup = "Callable[[str, str, float], NDArray[np.float64] | None]"


def intrinsics_of(info: CameraInfo) -> hs.Intrinsics:
    matrix = np.asarray(info.get_K_matrix(), dtype=float).reshape(3, 3)
    return hs.Intrinsics(
        width=int(info.width),
        height=int(info.height),
        fx=float(matrix[0, 0]),
        fy=float(matrix[1, 1]),
        cx=float(matrix[0, 2]),
        cy=float(matrix[1, 2]),
    )


def transform_to_matrix(transform: Any) -> NDArray[np.float64]:
    t, q = transform.translation, transform.rotation
    return hs.transform_matrix(np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w]))


@dataclass
class IngestConfig:
    gate: hs.KeyframeGateConfig
    motion_reference_frame: str = "odom"
    # Never embed frames closer together than this (s). 0.2 = 5 Hz.
    min_frame_interval_s: float = 1.0 / hs.MAX_KEYFRAME_HZ
    # Write a searchable vec0 index for EVERY model, not just the primary one, so a
    # query can ask each its own nearest-neighbour question instead of loading every
    # keyframe's grids into memory. Costs one insert per patch per model at ingest.
    patch_vectors: bool = True
    # The flat layout: a patch row carries everything needed to place it -- camera
    # frame, timestamp, its own ray and its own depth -- and the per-frame row holds
    # only the depth thumbnail. No keyframe blob, so reading one patch does not
    # unpickle a frame's worth of embeddings.
    #
    # There is deliberately NO shared cell grid across models. Embeddings cannot be
    # resampled onto one without averaging them, and once agreement between models is
    # geometric -- do their boxes overlap -- the cells never need to line up. Each
    # model keeps its own native grid, so patch counts per frame differ between them.
    #
    # Default False until the query side reads it: writing a layout nothing can answer
    # from would break every existing recording. Pass --flat to build one.
    flat: bool = False
    # One model per pass writes its own patch stream, so the fast ones land first and a
    # slow one finishing hours later costs nothing. Only the first pass needs to write
    # the depth thumbnails; the rest would duplicate them.
    thumbnails: bool = True
    # Depth beyond this (m) is a hole: RealSense 65535 mm sentinels and glitches.
    max_depth_m: float = 10.0
    depth_max_dt: float = 0.05
    depth_history: int = 64
    depth_thumbnail_stride: int = 4
    # The keyframe's cell grid: what patch_depth is measured on and what the
    # query pools the members' scores onto. None = the model's own grid for a
    # single fixed-resolution checkpoint (the original layout), and for an
    # ensemble the finest grid any member offers, never coarser than 24x24.
    cell_grid: tuple[int, int] | None = None
    # Fill the depth holes from the colour frame before measuring patch depth
    # (dimos.perception.depth2depth). Stereo returns nothing off glass, shiny
    # floors and dark shelves, and reads *through* a freezer door -- so a patch
    # in front of one is placed metres too far. "" = raw sensor depth only.
    depth2depth_model: str = ""
    # Keep the colour frame each embedding frame was made from. Off for an ingest of a
    # recording, which already holds its colour; ON for a live run, where the camera's
    # frames exist only on the wire and the detector still has to be shown the one from
    # four minutes ago.
    keep_frames: bool = False


def grids_of(model: Any, image: Image) -> list[tuple[NDArray[np.float32], tuple[int, int]]]:
    """One ``(grid, (rows, cols))`` per ensemble member, from an ensemble, a
    single SigLIP2Patches, or any object with ``embed_patches`` and
    ``patches_per_side`` (the test stubs)."""
    if hasattr(model, "embed_grids"):
        grids = model.embed_grids(image)
        return grids if isinstance(grids, list) and grids and isinstance(grids[0], tuple) else grids
    side = int(model.patches_per_side)
    return [(model.embed_patches(image)[0], (side, side))]


def decoded(image: Any) -> Image:
    """A raw frame from whatever the recording holds. Compressed frames go
    through CompressedImage.decode(), except webp, which it has no branch for
    (the lite recorder writes webp colour, so the grocery recordings are all
    webp) -- Pillow reads those."""
    if not isinstance(getattr(image, "format", None), str) or not hasattr(image, "decode"):
        return image
    if not image.format.startswith("webp"):
        return image.decode()
    import io

    from PIL import Image as PillowImage

    pixels = np.asarray(PillowImage.open(io.BytesIO(image.data)).convert("RGB"), np.uint8)
    return Image(data=pixels, format=ImageFormat.RGB, frame_id=image.frame_id, ts=image.ts)


class PatchIngestor:
    """Gate frames, embed the survivors, pair them with depth, write keyframes
    and one vector per patch into ``store``."""

    def __init__(
        self,
        store: Store,
        model: SigLIP2Patches,
        config: IngestConfig,
        lookup: Callable[[str, str, float], NDArray[np.float64] | None] | None = None,
        slug: str = "",
        copy_tf: bool = True,
    ) -> None:
        self.store = store
        self.model = model
        self.config = config
        # Ensemble bookkeeping, written with every keyframe so the query side
        # can load the matching text towers: short tags and the full specs.
        self.members: list[str] = list(getattr(model, "tags", []))
        self.member_specs: list[str] = list(getattr(model, "specs", []))
        # target_from_source(target_frame, source_frame, ts) for the motion gate
        # and depth-to-colour alignment. None = no tf available (both skipped).
        self.lookup = lookup
        self.buffer = hs.RollingBuffer(config.gate)
        self.intrinsics: dict[str, hs.Intrinsics] = {}
        self.depths: deque[tuple[float, str, NDArray[np.float32]]] = deque(
            maxlen=config.depth_history
        )
        # Built on the first colour frame: loading the model costs seconds and
        # an ingest without depth2depth should never pay for it.
        self.fuser: Any = None
        self.slug = slug
        # False when the store IS the recording: see add_tf.
        self.copy_tf = copy_tf
        # Every stream is opened on first write, not here: naming one creates it, and
        # an empty stream is a name pretending to be an index. The flat layout has no
        # keyframe stream at all, so opening one eagerly would leave that lie behind.
        self._keyframes: Stream[Any] | None = None
        self._thumbnails: Stream[Any] | None = None
        self._filled: Stream[Any] | None = None
        self._frames: Stream[Any] | None = None
        self._cameras_written: set[str] = set()
        # One vec0 stream per model; the member list is only certain once it has run.
        self.patches_by_member: dict[str, Stream[Any]] = {}
        self.tf_stream: Stream[TFMessage] = store.stream(TF_STREAM, TFMessage)
        self.last_embedded = -np.inf
        self.stats = {"images": 0, "gated": 0, "embedded": 0, "kept": 0, "kept_without_depth": 0}

    @property
    def keyframes(self) -> Stream[Any]:
        if self._keyframes is None:
            self._keyframes = self.store.stream(stream_names(self.slug)[0], dict)
        return self._keyframes

    @property
    def thumbnails(self) -> Stream[Any]:
        if self._thumbnails is None:
            self._thumbnails = self.store.stream(thumbnail_stream_for(self.slug), dict)
        return self._thumbnails

    @property
    def filled(self) -> Stream[Any]:
        """Where the filled depth lands, opened once."""
        if self._filled is None:
            self._filled = self.store.stream(filled_stream_for(self.slug), dict)
        return self._filled

    @property
    def frames(self) -> Stream[Any]:
        """Where the kept colour frames land, opened once."""
        if self._frames is None:
            self._frames = self.store.stream(frame_stream_for(self.slug), Image)
        return self._frames

    def _keep_frame(self, camera_frame: str, ts: float, rgb: NDArray[np.uint8]) -> None:
        """Keep the picture this embedding frame was made from, if asked to.

        Off by default: a recording already holds its own colour, and writing a second
        copy of it beside the first would be a waste of the disk. On, it is what lets a
        live run answer at all -- the camera's frames are on the wire and nowhere else,
        and the detector needs the one from four minutes ago.
        """
        if not self.config.keep_frames:
            return
        # RGB, said out loud. `Image.from_numpy` defaults to BGR, and the buffer hands
        # this pass RGB -- so the default silently labels the channels backwards and
        # `to_rgb()` then swaps them for real. The detector is shown the result, and
        # OWLv2 asked for "a car" on a street full of cars refused every frame.
        self.frames.append(
            Image.from_numpy(
                np.ascontiguousarray(rgb),
                format=ImageFormat.RGB,
                frame_id=camera_frame,
                ts=ts,
            ),
            ts=ts,
            tags={"camera_frame": camera_frame},
        )

    def patch_stream(self, member: str) -> Stream[Any]:
        """This model's vec0 stream, opened once."""
        if member not in self.patches_by_member:
            self.patches_by_member[member] = self.store.stream(
                patch_stream_for(self.slug, member), dict
            )
        return self.patches_by_member[member]

    def _write_patch_vectors(
        self,
        keyframe_id: int,
        grids: Sequence[tuple[NDArray[np.float16], tuple[int, int]]],
        ts: float,
        model: str,
    ) -> None:
        """Every model's patches, each in its own searchable index.

        One index per model rather than one for the primary: the query asks each model
        its own nearest-neighbour question and keeps the cells both agree on, which it
        cannot do against a single table. Costs one vec0 insert per patch per model.
        """
        for position, (grid, _) in enumerate(grids):
            member = self.members[position] if position < len(self.members) else f"member{position}"
            stream = self.patch_stream(member)
            for index in range(len(grid)):
                stream.append(
                    {"keyframe": keyframe_id, "patch": index, "model": model, "member": member},
                    ts=ts,
                    tags={"keyframe": keyframe_id, "model": model, "member": member},
                    embedding=Embedding(vector=grid[index].astype(np.float32), timestamp=ts),
                )

    def add_camera_info(self, info: CameraInfo) -> None:
        self.intrinsics[info.frame_id] = intrinsics_of(info)
        self._keep_camera_info(info)

    def _keep_camera_info(self, info: CameraInfo) -> None:
        """Write this camera down, once, so a query can place a box off its own store.

        Live, the intrinsics arrive on a port and exist only in this process's memory.
        The query side turns a 2D detector box into a place in the world with them, and
        without them every answer dies as "no camera_info for <frame>" -- one warning per
        episode, no exception, no answers.
        """
        if not self.config.keep_frames or info.frame_id in self._cameras_written:
            return
        self.store.stream(info_stream_for(self.slug), CameraInfo).append(
            info,
            ts=float(getattr(info, "ts", 0.0) or time.time()),
            tags={"camera_frame": info.frame_id},
        )
        self._cameras_written.add(info.frame_id)

    def add_tf(self, msg: TFMessage, ts: float | None = None) -> None:
        """Record tf so the query side can place keyframes at query time.

        Does nothing when the index lives in the recording: the transforms are already
        there, and writing them again appends a second set of the recording's own tf to
        itself. grocery.db reached 85,302 rows over 16,048 distinct stamps -- 5.3x
        duplicated -- across a handful of in-place ingests before this was caught.
        """
        if not self.copy_tf:
            return
        stamps = [float(t.ts) for t in msg.transforms if getattr(t, "ts", None)]
        self.tf_stream.append(
            msg, ts=ts if ts is not None else (max(stamps) if stamps else time.time())
        )

    def add_depth(self, image: Image) -> None:
        image = decoded(image)
        depth = np.asarray(image.as_numpy())
        metres = depth.astype(np.float32) * (0.001 if depth.dtype == np.uint16 else 1.0)
        metres[(metres > self.config.max_depth_m) | ~np.isfinite(metres)] = 0.0
        self.depths.append((float(image.ts), image.frame_id, metres))

    def _speeds(self, camera_frame: str, ts: float) -> tuple[float, float] | None:
        gate = self.config.gate
        if self.lookup is None or (
            gate.max_angular_velocity is None and gate.max_linear_velocity is None
        ):
            return None
        before = self.lookup(self.config.motion_reference_frame, camera_frame, ts - 0.05)
        after = self.lookup(self.config.motion_reference_frame, camera_frame, ts + 0.05)
        if before is None or after is None:
            return None
        delta = np.linalg.inv(before) @ after
        angle = float(np.arccos(np.clip((np.trace(delta[:3, :3]) - 1) / 2, -1.0, 1.0)))
        return angle / 0.1, float(np.linalg.norm(delta[:3, 3])) / 0.1

    def add_image(self, image: Image) -> bool:
        """Returns True when this frame produced a keyframe."""
        image = decoded(image)
        self.stats["images"] += 1
        if self.stats["images"] in (1, 50) or self.stats["images"] % 250 == 0:
            logger.info(f"hyperspace ingest: {self.stats}")
        ts = float(image.ts)
        if ts - self.last_embedded < self.config.min_frame_interval_s:
            return False
        rgb = np.asarray(image.to_rgb().data)
        speeds = self._speeds(image.frame_id, ts)
        if hs.quality_gate(self.config.gate, rgb, speeds) is not None:
            self.stats["gated"] += 1
            return False
        started = time.monotonic()
        grids = grids_of(self.model, image)
        grid = grids[0][0]
        if self.stats["embedded"] == 0:
            logger.info(f"hyperspace ingest: first embed took {time.monotonic() - started:.2f}s")
        self.last_embedded = ts
        self.stats["embedded"] += 1
        # None, not 1.0: without a pose there is no motion to score, and scoring it
        # the maximum made an unplaceable frame outrank every measured one.
        quality = None if speeds is None else 1.0 / (1.0 + speeds[0] + 0.25 * speeds[1])
        # Pair depth now, while its frame is still in the short depth history:
        # the buffer judges this frame ~5 embedded frames later.
        depth = self._paired_depth(image.frame_id, ts)
        kept = self.buffer.push(
            hs.BufferedFrame(
                ts=ts,
                grid=grid.astype(np.float16),
                quality=quality,
                payload=(image.frame_id, depth, grids, rgb),
            )
        )
        if kept is None:
            return False
        self._write_keyframe(kept)
        return True

    def flush(self) -> int:
        kept = self.buffer.flush()
        for frame in kept:
            self._write_keyframe(frame)
        return len(kept)

    def _filled_depth(
        self,
        camera_frame: str,
        ts: float,
        rgb: NDArray[np.uint8],
        depth: NDArray[np.float32] | None,
    ) -> NDArray[np.float32] | None:
        """Fill this keyframe's depth holes, and keep the result in the recording.

        Only keyframes, and only once. Fusing costs about fifty milliseconds, so doing
        it to every frame the camera produced would be most of an ingest for frames
        nothing will ever ask about -- and doing it at query time would be paying it
        again every time somebody asks.
        """
        if depth is None or not self.config.depth2depth_model:
            return depth
        filled = self._fused_depth(rgb, depth)
        self.filled.append(
            {
                "camera_frame": camera_frame,
                "ts": ts,
                # Millimetres as uint16, the same as the sensor's own: a tenth of the
                # bytes of float metres and finer than the depth is accurate to.
                "depth_mm": np.clip(filled * 1000.0, 0, 65535).astype(np.uint16),
            },
            ts=ts,
            tags={"camera_frame": camera_frame},
        )
        return filled

    def _fused_depth(
        self, rgb: NDArray[np.uint8], depth: NDArray[np.float32]
    ) -> NDArray[np.float32]:
        """Depth with its holes filled from the colour frame. Shapes must match:
        the pairing has already put the depth on the colour camera's grid."""
        from dimos.perception.depth2depth.fusion import Depth2Depth, FuseConfig

        if self.fuser is None:
            self.fuser = Depth2Depth(
                config=FuseConfig(far_m=self.config.max_depth_m),
                model_name=self.config.depth2depth_model,
            )
            self.fuser.start()
            logger.info(f"hyperspace ingest: depth2depth on {self.fuser.device}")
        if rgb.shape[:2] != depth.shape[:2]:
            return depth
        return self.fuser.fuse(rgb, depth).fused

    def _paired_depth(self, camera_frame: str, ts: float) -> NDArray[np.float32] | None:
        best = None
        for depth_ts, depth_frame, metres in self.depths:
            dt = abs(depth_ts - ts)
            if dt <= self.config.depth_max_dt and (best is None or dt < best[0]):
                best = (dt, depth_frame, metres)
        if best is None:
            return None
        _, depth_frame, metres = best
        color = self.intrinsics.get(camera_frame)
        depth_intrinsics = self.intrinsics.get(depth_frame)
        if color is None or depth_intrinsics is None:
            return None
        if depth_frame == camera_frame:
            return metres
        if self.lookup is None:
            return None
        color_from_depth = self.lookup(camera_frame, depth_frame, ts)
        if color_from_depth is None:
            return None
        return hs.reproject_depth(metres, depth_intrinsics, color, color_from_depth)

    def cell_grid(
        self, grids: list[tuple[NDArray[np.float32], tuple[int, int]]]
    ) -> tuple[int, int]:
        if self.config.cell_grid is not None:
            return self.config.cell_grid
        # One fixed grid: keep the model's own layout, so the vector index's
        # patch ids and the keyframe cells are the same thing.
        if len(grids) == 1:
            return grids[0][1]
        # Several members: the finest grid any of them offers, so a tiled
        # member's extra resolution is not thrown away resampling onto a
        # coarser common grid. Floored at 24x24, the layout every ensemble
        # used before tiling existed, so the untiled defaults are unchanged.
        return (
            max(24, max(shape[0] for _, shape in grids)),
            max(24, max(shape[1] for _, shape in grids)),
        )

    def _write_flat(self, kept: hs.BufferedFrame) -> None:
        """One row per patch that stands on its own, plus one depth thumbnail."""
        camera_frame, depth, grids, rgb = kept.payload
        depth = self._filled_depth(camera_frame, kept.ts, rgb, depth)
        color = self.intrinsics.get(camera_frame)
        if color is None:
            logger.warning(f"hyperspace: no camera_info for {camera_frame!r} yet; frame dropped")
            return
        model = self.slug or index_slug(self.member_specs or self.members) or "unnamed"
        tags = {"camera_frame": camera_frame, "model": model}
        self._keep_frame(camera_frame, kept.ts, rgb)

        if depth is None:
            self.stats["kept_without_depth"] += 1
        elif self.config.thumbnails:
            stride = max(self.config.depth_thumbnail_stride, 1)
            thinned = depth[::stride, ::stride]
            rows, cols = np.nonzero(np.isfinite(thinned) & (thinned > 0))
            metres = thinned[rows, cols]
            # Camera-frame points, millimetres as int16: the query needs no intrinsics
            # to use them, and the world pose stays outside so a correction can move it.
            us = (cols * stride + 0.5 - color.cx) / color.fx
            vs = (rows * stride + 0.5 - color.cy) / color.fy
            points = np.stack([us * metres, vs * metres, metres], axis=1) * 1000.0
            self.thumbnails.append(
                {
                    "camera_frame": camera_frame,
                    "ts": kept.ts,
                    "points_mm": np.clip(points, -32768, 32767).astype(np.int16),
                },
                ts=kept.ts,
                tags=tags,
            )

        for position, (grid, shape) in enumerate(grids):
            member = self.members[position] if position < len(self.members) else f"member{position}"
            rows_n, cols_n = shape
            patch_depth = (
                np.full(rows_n * cols_n, np.nan, dtype=np.float32)
                if depth is None
                else hs.per_patch_depth(depth, rows_n, cols_n)
            )
            # The ray through each cell's centre at z = 1, so a patch places itself.
            row_index, col_index = np.divmod(np.arange(rows_n * cols_n), cols_n)
            us = ((col_index + 0.5) * color.width / cols_n - color.cx) / color.fx
            vs = ((row_index + 0.5) * color.height / rows_n - color.cy) / color.fy
            stream = self.patch_stream(member)
            member_tags = {**tags, "member": member}
            for index in range(len(grid)):
                stream.append(
                    {
                        "camera_frame": camera_frame,
                        "ts": kept.ts,
                        "cell": index,
                        "grid": [rows_n, cols_n],
                        "ray": [float(us[index]), float(vs[index])],
                        "depth": float(patch_depth[index]),
                        "member": member,
                    },
                    ts=kept.ts,
                    tags=member_tags,
                    embedding=Embedding(vector=grid[index].astype(np.float32), timestamp=kept.ts),
                )
        self.stats["kept"] += 1

    def _write_keyframe(self, kept: hs.BufferedFrame) -> None:
        if self.config.flat:
            self._write_flat(kept)
            return
        camera_frame, depth, grids, rgb = kept.payload
        depth = self._filled_depth(camera_frame, kept.ts, rgb, depth)
        color = self.intrinsics.get(camera_frame)
        if color is None:
            logger.warning(f"hyperspace: no camera_info for {camera_frame!r} yet; keyframe dropped")
            return
        self._keep_frame(camera_frame, kept.ts, rgb)
        rows, cols = self.cell_grid(grids)
        if depth is None:
            self.stats["kept_without_depth"] += 1
            patch_depth = np.full(rows * cols, np.nan, dtype=np.float32)
            thumbnail = np.zeros((0, 0), dtype=np.uint16)
        else:
            patch_depth = hs.per_patch_depth(depth, rows, cols)
            stride = max(self.config.depth_thumbnail_stride, 1)
            thumbnail = np.clip(depth[::stride, ::stride] * 1000.0, 0, 65535).astype(np.uint16)
        payload = {
            "camera_frame": camera_frame,
            "ts": kept.ts,
            "rows": rows,
            "cols": cols,
            "intrinsics": vars(color),
            # The primary member's grid, as before; ``grids`` carries every
            # member (primary first) with its own shape when there is more
            # than one, or when the one grid is not the cell grid.
            "grid": kept.grid,
            "patch_depth": patch_depth,
            "thumbnail_mm": thumbnail,
            "thumbnail_stride": self.config.depth_thumbnail_stride,
        }
        shapes = [shape for _, shape in grids]
        # Provenance on EVERY keyframe, not only ensembles: a db can hold several
        # indexes side by side and "which model wrote this" must never be a guess.
        payload["members"] = self.members
        payload["member_specs"] = self.member_specs
        payload["model"] = self.slug or index_slug(self.member_specs or self.members) or "unnamed"
        has_grids = len(grids) > 1 or shapes[0] != (rows, cols)
        if has_grids:
            payload["grids"] = [grid.astype(np.float16) for grid, _ in grids]
            payload["grid_shapes"] = [list(shape) for shape in shapes]
            payload.setdefault("members", [f"member{i}" for i in range(len(grids))])
        model = payload["model"]
        keyframe = self.keyframes.append(
            payload, ts=kept.ts, tags={"camera_frame": camera_frame, "model": model}
        )
        # EVERY model gets a searchable index, one stream each. It duplicates what the
        # keyframe's `grids` blob already holds, deliberately: reading one patch out of
        # that blob means unpickling the whole keyframe, which is why a query loads all
        # of them into memory. Two k-NN queries against two indexes replace that.
        # Not free -- measured on 30 s of grocery, 345 keyframes: 59.8 s and 386 MB
        # with no index against 96.9 s and 615 MB with one, and this writes one PER
        # MODEL.
        if self.config.patch_vectors:
            self._write_patch_vectors(keyframe.id, grids, kept.ts, model)
        self.stats["kept"] += 1
        logger.info(
            f"hyperspace keyframe {keyframe.id} at {kept.ts:.2f} "
            f"({self.stats['kept']} kept of {self.stats['embedded']} embedded, "
            f"{self.stats['images']} seen)"
        )

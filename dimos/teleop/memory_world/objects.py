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

"""Frames first, geometry second: a detector's box on a ranked frame becomes a 3D box.

The patch index ranks frames, which is what it is good at, and an open-vocabulary
detector does the finding. Each candidate frame goes to OWLv2 with the query; its 2D
box plus that frame's depth image become a box in the world. A recording without a
depth camera gets its depth rendered from the lidar map through the same camera.

A detector box is tight around the object but never only the object: its corners see
whatever is behind, and with a low camera its lower half sees the ground in front.
So ground readings are dropped first, the box's median depth among the rest is taken
to be the object, and pixels further than a band from it are dropped as background
showing through. A box whose measured size is far smaller than the box itself implies
at that depth was not measured on the object at all and is refused. Looks at one place
merge into one answer, weighted by what the detector thought of each look.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np

from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.teleop.memory_world.camera import CameraModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The lidar map is rendered onto a depth grid this many times coarser than the image.
# Gaps between voxels stay empty: the box's median ignores them, while filling them
# from neighbors would pull the nearer ground under a far object into its box.
DEPTH_RENDER_SCALE = 2


@dataclass(frozen=True)
class Box3D:
    """An axis-aligned box in the world frame, with what it was measured from."""

    centre: tuple[float, float, float]
    extent: tuple[float, float, float]
    pixels: int
    depth_m: float


@dataclass
class Measured:
    """The object's points in the camera's frame, or why the box could not be measured."""

    points: np.ndarray | None = None
    median: float = 0.0
    why: str = ""

    def __bool__(self) -> bool:
        return self.points is not None


@dataclass
class Look:
    """One frame the detector was shown for a query."""

    frame_id: int
    ts: float
    image: Image
    world_t_camera: np.ndarray
    depth: np.ndarray | None


@dataclass
class Found:
    """One object the detector found, merged from every look that agreed."""

    query: str
    # The object's own centre and size when a look was measured on it. Otherwise
    # the centre is where the camera stood when the detector saw it, extent None.
    centre: tuple[float, float, float]
    extent: tuple[float, float, float] | None
    depth_m: float
    confidence: float
    frame_id: int
    ts: float
    box2d: tuple[float, float, float, float]
    world_t_camera: np.ndarray = field(repr=False)
    views: int = 1

    @property
    def placed(self) -> bool:
        return self.extent is not None

    def to_json(self) -> dict[str, Any]:
        return {
            "label": self.query,
            "position": list(self.centre),
            "extent": None if self.extent is None else list(self.extent),
            "height": None if self.extent is None else self.extent[2],
            "seen_from_m": self.depth_m,
            "confidence": self.confidence,
            "views": self.views,
            "ts": self.ts,
            "best_frame_id": self.frame_id,
        }


class Detector:
    """What the locate step needs from a detector: boxes for one phrase."""

    def query_detections(self, image: Image, queries: list[str], threshold: float) -> Any: ...


class Segmenter:
    """What the locate step needs from a segmenter: a mask for each box."""

    def segment(self, detections: ImageDetections2D) -> ImageDetections2D: ...


@dataclass
class Sighting:
    """One detector box on one look, with the map points under its mask when measurable."""

    look: Look
    box2d: tuple[float, float, float, float]
    confidence: float
    mask: np.ndarray | None
    points: np.ndarray | None = None
    depth_m: float = 0.0

    @property
    def placed(self) -> bool:
        return self.points is not None


# Splat sizes in render pixels: a voxel's footprint is rounded up to one of these.
_SPLAT_SIZES = (1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48)


def render_depth(
    cloud: np.ndarray,
    world_t_camera: np.ndarray,
    camera: CameraModel,
    *,
    max_depth_m: float,
    voxel_m: float = 0.08,
) -> np.ndarray:
    """The lidar map seen through the camera as a depth image in meters, 0 where nothing is.

    Each voxel covers the pixels its footprint spans at its depth, nearest wins, so a
    near object hides what stands behind it instead of showing it between its voxels.
    """
    scale = DEPTH_RENDER_SCALE
    width, height = -(-camera.width // scale), -(-camera.height // scale)
    rotation = world_t_camera[:3, :3].T
    local = np.asarray(cloud, dtype=np.float32) - world_t_camera[:3, 3].astype(np.float32)
    local = local @ rotation.T
    depth = local[:, 2]
    ahead = (depth > 0.1) & (depth < max_depth_m)
    local, depth = local[ahead], depth[ahead]
    fx, fy = camera.fx / scale, camera.fy / scale
    cx, cy = camera.cx / scale, camera.cy / scale
    u = np.floor(fx * local[:, 0] / depth + cx).astype(np.int64)
    v = np.floor(fy * local[:, 1] / depth + cy).astype(np.int64)
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    u, v, depth = u[inside], v[inside], depth[inside]
    footprint = np.ceil(voxel_m * fx / depth)
    sizes = np.asarray(_SPLAT_SIZES)
    bucket = np.searchsorted(sizes, np.clip(footprint, 1, sizes[-1]))
    empty = np.float32(1e9)
    nearest = np.full((height, width), empty, dtype=np.float32)
    for index in np.unique(bucket):
        chosen = bucket == index
        layer = np.full(width * height, empty, dtype=np.float32)
        order = np.argsort(-depth[chosen])
        layer[v[chosen][order] * width + u[chosen][order]] = depth[chosen][order]
        size = int(sizes[index])
        if size > 1:
            layer = cv2.erode(layer.reshape(height, width), np.ones((size, size), np.uint8))
        nearest = np.minimum(nearest, layer.reshape(height, width))
    nearest[nearest >= empty] = 0.0
    return nearest


def object_points(
    box: Sequence[float],
    image_size: tuple[int, int],
    depth: np.ndarray,
    camera: CameraModel,
    *,
    max_depth_m: float,
    band_m: float,
    min_pixels: int,
    link_m: float = 0.5,
    reach_fraction: float = 0.75,
    mask: np.ndarray | None = None,
    world_t_camera: np.ndarray | None = None,
    ground_z: float | None = None,
) -> Measured:
    """The object's points in the camera's own frame, from a 2D box and a depth image.

    The box is in the pixels of an image of image_size. The depth may be on a coarser
    grid; going through fractions of the frame keeps the two independent. With a pose
    and a ground height, readings on the ground are left out before the median. The
    object is everything connected to the median-depth surface through gaps under
    link_m and no deeper than reach_fraction of the box's implied size, so a car seen
    at an angle keeps its far end and loses the wall behind it. A mask in image pixels
    narrows the box to the object's own pixels, and then only connectivity bounds it.
    """
    height, width = depth.shape
    image_width, image_height = image_size
    if not image_width or not image_height or not width or not height:
        return Measured(why="the box or the depth image has no size")
    left = int(np.clip(np.floor(box[0] / image_width * width), 0, width - 1))
    top = int(np.clip(np.floor(box[1] / image_height * height), 0, height - 1))
    right = int(np.clip(np.ceil(box[2] / image_width * width), left + 1, width))
    bottom = int(np.clip(np.ceil(box[3] / image_height * height), top + 1, height))

    window = np.asarray(depth[top:bottom, left:right], dtype=np.float64)
    read = np.isfinite(window) & (window > 0)
    if mask is not None:
        small = cv2.resize(mask.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST)
        read &= small[top:bottom, left:right] > 0
        if not int(read.sum()):
            return Measured(why="no depth under the mask")
    usable = read & (window <= max_depth_m)
    if not int(read.sum()):
        return Measured(why="no depth inside the box")
    scale_x, scale_y = width / camera.width, height / camera.height
    if world_t_camera is not None and ground_z is not None and int(usable.sum()):
        rows, cols = np.nonzero(usable)
        z = window[rows, cols]
        us = (cols + left + 0.5 - camera.cx * scale_x) / (camera.fx * scale_x)
        vs = (rows + top + 0.5 - camera.cy * scale_y) / (camera.fy * scale_y)
        local = np.stack([us * z, vs * z, z], axis=1)
        world_z = (world_t_camera[:3, :3] @ local.T).T[:, 2] + world_t_camera[2, 3]
        on_ground = world_z <= ground_z
        if int((~on_ground).sum()) < min_pixels:
            return Measured(why="only the ground inside the box")
        usable[rows[on_ground], cols[on_ground]] = False
    if not int(usable.sum()):
        return Measured(
            why=f"everything inside the box is past {max_depth_m:g} m "
            f"(nearest {float(window[read].min()):.1f} m)"
        )
    if int(usable.sum()) < min_pixels:
        return Measured(why=f"only {int(usable.sum())} depth readings inside the box")
    median = float(np.median(window[usable]))
    seed = usable & (np.abs(window - median) <= band_m)
    if int(seed.sum()) < min_pixels:
        return Measured(
            why=f"only {int(seed.sum())} readings within {band_m:g} m of the box's {median:.1f} m"
        )

    if mask is None:
        implied_width = (right - left) / scale_x * median / camera.fx
        implied_height = (bottom - top) / scale_y * median / camera.fy
        reach = max(band_m, reach_fraction * max(implied_width, implied_height))
        usable &= np.abs(window - median) <= reach
    rows, cols = np.nonzero(usable)
    z = window[rows, cols]
    us = (cols + left + 0.5 - camera.cx * scale_x) / (camera.fx * scale_x)
    vs = (rows + top + 0.5 - camera.cy * scale_y) / (camera.fy * scale_y)
    points = np.stack([us * z, vs * z, z], axis=1)
    keep = _grown_from(points, seed[rows, cols], link_m)
    return Measured(points[keep], median)


def _grown_from(points: np.ndarray, seed: np.ndarray, link_m: float) -> np.ndarray:
    """Mask of the points connected to a seed point through gaps of at most link_m.

    Points are binned into cells of half the link, each cell is grown by one cell
    and the grown cells are labeled, so two points bridge when they sit within a
    link of each other. Far background is dense in pixels but sparse in cells.
    """
    from scipy import ndimage

    cell = max(link_m / 2.0, 0.02)
    low = points.min(axis=0)
    span = np.ceil((points.max(axis=0) - low) / cell).astype(int) + 3
    while int(np.prod(span)) > 20_000_000:
        cell *= 2.0
        span = np.ceil((points.max(axis=0) - low) / cell).astype(int) + 3
    index = np.floor((points - low) / cell).astype(int) + 1
    occupied = np.zeros(tuple(span), dtype=bool)
    occupied[index[:, 0], index[:, 1], index[:, 2]] = True
    cube = np.ones((3, 3, 3), dtype=bool)
    grown = ndimage.binary_dilation(occupied, structure=cube)
    labels, _ = ndimage.label(grown, structure=cube)
    point_labels = labels[index[:, 0], index[:, 1], index[:, 2]]
    return np.isin(point_labels, np.unique(point_labels[seed]))


def box_from_points(
    points: np.ndarray, world_t_camera: np.ndarray, depth_m: float, *, trim_percentile: float
) -> Box3D:
    """Camera-frame points through a pose into an axis-aligned world box."""
    return _box_of(_world_points(points, world_t_camera), depth_m, trim_percentile)


def _world_points(points: np.ndarray, world_t_camera: np.ndarray) -> np.ndarray:
    moved = (world_t_camera[:3, :3] @ np.asarray(points, dtype=np.float64).T).T
    return moved + world_t_camera[:3, 3]


def _box_of(moved: np.ndarray, depth_m: float, trim_percentile: float) -> Box3D:
    low = np.percentile(moved, trim_percentile, axis=0)
    high = np.percentile(moved, 100.0 - trim_percentile, axis=0)
    centre = (low + high) / 2.0
    extent = high - low
    return Box3D(
        centre=(float(centre[0]), float(centre[1]), float(centre[2])),
        extent=(float(extent[0]), float(extent[1]), float(extent[2])),
        pixels=len(moved),
        depth_m=float(depth_m),
    )


@dataclass(frozen=True)
class LocateConfig:
    """Every knob of the detector path, in one place."""

    # OWLv2's per-box score is calibrated, so this is a refusal threshold, not a cut.
    threshold: float = 0.5
    # Boxes kept from one frame: a frame really can hold two of the thing asked for.
    per_frame: int = 4
    # Frames of one place shown to the detector before the place counts as refused.
    attempts: int = 2
    max_depth_m: float = 20.0
    band_m: float = 0.5
    min_pixels: int = 20
    trim_percentile: float = 2.0
    # Looks closer than this are one object.
    merge_m: float = 0.75
    # Readings this far below the camera, or lower, are the ground.
    ground_below_camera_m: float = 0.15
    # A measured box smaller than this fraction of what the 2D box implies at
    # its depth was measured on the ground or a wall, not the object.
    min_size_fraction: float = 0.35
    # Unmeasured looks whose cameras stood closer than this are one sighting.
    seen_from_merge_m: float = 2.5
    # Depth readings closer than this to the object's surface belong to it. Foliage
    # returns are sparse, so the link is a few voxels wide.
    link_m: float = 0.5
    # A visible surface spreads in depth by at most this fraction of its width or height.
    reach_fraction: float = 0.75
    # A fused point another view sees this much nearer than its map surface, outside
    # that view's mask, is carved away.
    carve_tolerance_m: float = 0.3


def locate(
    query: str,
    places: Sequence[Sequence[Look]],
    detector: Detector,
    camera: CameraModel,
    *,
    segmenter: Segmenter | None = None,
    config: LocateConfig = LocateConfig(),
    progress: Callable[[str], None] | None = None,
) -> list[Found]:
    """Show the detector each place's frames in turn and fuse what it draws into objects.

    Each inner sequence is one place's frames, best first. A place is settled by the
    first frame the detector accepts; refusals fall through to the next frame. With a
    segmenter every box is narrowed to its mask before the map is measured under it.
    """
    phrasings = detector_phrasings(query)
    placed: list[Sighting] = []
    unplaced: list[Sighting] = []
    for frames in places:
        for attempt, look in enumerate(frames[: max(1, config.attempts)]):
            found = detector.query_detections(look.image, phrasings, config.threshold)
            boxes = _distinct_boxes(sorted(found.detections, key=lambda box: -box.confidence))
            boxes = boxes[: max(1, config.per_frame)]
            if not boxes:
                continue
            for box, mask in zip(boxes, _masks_for(segmenter, look.image, boxes), strict=True):
                box2d = tuple(float(v) for v in box.bbox)
                sighting = _place(look, box2d, float(box.confidence), mask, camera, config)  # type: ignore[arg-type]
                (placed if sighting.placed else unplaced).append(sighting)
            if progress is not None:
                progress(f"{len(placed)} boxes after {attempt + 1} frame(s) of a place")
            break
    found_objects = merge_looks(query, placed, camera, config=config)
    found_objects.extend(merge_sightings(query, unplaced, merge_m=config.seen_from_merge_m))
    return found_objects


def _masks_for(
    segmenter: Segmenter | None, image: Image, boxes: Sequence[Any]
) -> list[np.ndarray | None]:
    """One image-sized mask per box, or None each when there is no segmenter."""
    if segmenter is None:
        return [None] * len(boxes)
    segmented = segmenter.segment(ImageDetections2D(image=image, detections=list(boxes)))
    masks = [np.asarray(item.mask) > 0 for item in segmented.detections]
    if len(masks) != len(boxes):
        logger.warning("segmenter returned %d masks for %d boxes", len(masks), len(boxes))
        return [None] * len(boxes)
    return masks


_ARTICLES = ("a ", "an ", "the ", "some ")
_ES_PLURALS = ("ses", "xes", "zes", "ches", "shes")
_NOT_PLURALS = ("ss", "us", "is")
_IRREGULAR_PLURALS = {"people": "person", "men": "man", "women": "woman", "children": "child"}


def detector_phrasings(query: str) -> list[str]:
    """The phrase as asked plus its bare singular noun, which the detector scores higher."""
    bare = query.strip().lower()
    for article in _ARTICLES:
        if bare.startswith(article):
            bare = bare[len(article) :]
            break
    words = bare.split()
    if words:
        last = words[-1]
        if last in _IRREGULAR_PLURALS:
            last = _IRREGULAR_PLURALS[last]
        elif last.endswith("ies") and len(last) > 4:
            last = last[:-3] + "y"
        elif last.endswith(_ES_PLURALS):
            last = last[:-2]
        elif last.endswith("s") and not last.endswith(_NOT_PLURALS):
            last = last[:-1]
        words[-1] = last
    singular = " ".join(words)
    return [query] if singular in ("", query) else [query, singular]


def _distinct_boxes(boxes: Sequence[Any], overlap: float = 0.5) -> list[Any]:
    """Best-first boxes with any box mostly covering an earlier one dropped."""
    kept: list[Any] = []
    for box in boxes:
        if all(_iou(box.bbox, other.bbox) < overlap for other in kept):
            kept.append(box)
    return kept


def _iou(a: Sequence[float], b: Sequence[float]) -> float:
    width = min(a[2], b[2]) - max(a[0], b[0])
    height = min(a[3], b[3]) - max(a[1], b[1])
    if width <= 0 or height <= 0:
        return 0.0
    inter = width * height
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def _place(
    look: Look,
    box: tuple[float, float, float, float],
    confidence: float,
    mask: np.ndarray | None,
    camera: CameraModel,
    config: LocateConfig,
) -> Sighting:
    sighting = Sighting(look, box, confidence, mask)
    if look.depth is None:
        return sighting
    rgb = look.image.to_rgb().data
    measured = object_points(
        box,
        (int(rgb.shape[1]), int(rgb.shape[0])),
        look.depth,
        camera,
        max_depth_m=config.max_depth_m,
        band_m=config.band_m,
        min_pixels=config.min_pixels,
        link_m=config.link_m,
        reach_fraction=config.reach_fraction,
        mask=mask,
        world_t_camera=look.world_t_camera,
        ground_z=float(look.world_t_camera[2, 3]) - config.ground_below_camera_m,
    )
    if not measured:
        logger.debug("box on frame %d not placed: %s", look.frame_id, measured.why)
        return sighting
    assert measured.points is not None
    world = _world_points(measured.points, look.world_t_camera)
    placed = _box_of(world, measured.median, config.trim_percentile)
    implied_height = (box[3] - box[1]) * measured.median / camera.fy
    implied_width = (box[2] - box[0]) * measured.median / camera.fx
    too_small = (
        placed.extent[2] < config.min_size_fraction * implied_height
        and max(placed.extent[:2]) < config.min_size_fraction * implied_width
    )
    if too_small:
        logger.debug(
            "box on frame %d not placed: measured %.1f m for a box implying %.1f m",
            look.frame_id,
            placed.extent[2],
            implied_height,
        )
        return sighting
    sighting.points = world
    sighting.depth_m = measured.median
    return sighting


def merge_looks(
    query: str,
    sightings: Sequence[Sighting],
    camera: CameraModel,
    *,
    config: LocateConfig = LocateConfig(),
) -> list[Found]:
    """Fuse the sightings of one object: their points united, then carved by every view.

    Sightings whose boxes come within merge_m of each other are one object. A point
    of the union that another view of that object saw clearly, but outside its mask,
    was background from where the first view stood and is dropped.
    """
    groups: list[list[Sighting]] = []
    bounds: list[tuple[np.ndarray, np.ndarray]] = []
    for sighting in sorted(sightings, key=lambda item: -item.confidence):
        assert sighting.points is not None
        low, high = sighting.points.min(axis=0), sighting.points.max(axis=0)
        for index, (group_low, group_high) in enumerate(bounds):
            gap = np.maximum(low - group_high, group_low - high)
            if float(gap.max()) <= config.merge_m:
                groups[index].append(sighting)
                bounds[index] = (np.minimum(group_low, low), np.maximum(group_high, high))
                break
        else:
            groups.append([sighting])
            bounds.append((low, high))
    found: list[Found] = []
    for group in groups:
        points = np.concatenate([item.points for item in group if item.points is not None])
        if len(group) > 1:
            points = carve(points, group, camera, tolerance_m=config.carve_tolerance_m)
        best = group[0]
        box = _box_of(points, best.depth_m, config.trim_percentile)
        found.append(
            Found(
                query=query,
                centre=box.centre,
                extent=box.extent,
                depth_m=best.depth_m,
                confidence=best.confidence,
                frame_id=best.look.frame_id,
                ts=best.look.ts,
                box2d=best.box2d,
                world_t_camera=best.look.world_t_camera,
                views=len(group),
            )
        )
    found.sort(key=lambda item: -item.confidence)
    return found


def carve(
    points: np.ndarray, sightings: Sequence[Sighting], camera: CameraModel, *, tolerance_m: float
) -> np.ndarray:
    """Drop the points a sighting's camera saw unoccluded but outside its mask.

    The render is sparse, so a point counts as occluded only when a nearer reading
    sits within a few pixels of it.
    """
    keep = np.ones(len(points), dtype=bool)
    for sighting in sightings:
        depth = sighting.look.depth
        if depth is None:
            continue
        pose = sighting.look.world_t_camera
        local = (pose[:3, :3].T @ (points - pose[:3, 3]).T).T
        z = local[:, 2]
        in_front = z > 0.1
        safe_z = np.where(in_front, z, 1.0)
        u = local[:, 0] / safe_z * camera.fx + camera.cx
        v = local[:, 1] / safe_z * camera.fy + camera.cy
        in_image = in_front & (u >= 0) & (u < camera.width) & (v >= 0) & (v < camera.height)
        col = np.clip((u * depth.shape[1] / camera.width).astype(int), 0, depth.shape[1] - 1)
        row = np.clip((v * depth.shape[0] / camera.height).astype(int), 0, depth.shape[0] - 1)
        nearest = _nearest_reading(depth)[row, col]
        seen = in_image & (z <= nearest + tolerance_m)
        if sighting.mask is not None:
            inside = sighting.mask[
                np.clip(v.astype(int), 0, sighting.mask.shape[0] - 1),
                np.clip(u.astype(int), 0, sighting.mask.shape[1] - 1),
            ]
        else:
            left, top, right, bottom = sighting.box2d
            inside = (u >= left) & (u <= right) & (v >= top) & (v <= bottom)
        keep &= ~(seen & ~inside)
    return points[keep] if int(keep.sum()) else points


def _nearest_reading(depth: np.ndarray) -> np.ndarray:
    """The nearest depth within a few pixels of each pixel, 1e9 where the render is empty."""
    filled = np.where(depth > 0, depth, np.float32(1e9)).astype(np.float32)
    return cv2.erode(filled, np.ones((5, 5), np.uint8))


def merge_sightings(
    query: str,
    sightings: Sequence[Sighting],
    *,
    merge_m: float,
) -> list[Found]:
    """Detected but unmeasured sightings, one per camera position, best first."""
    found: list[Found] = []
    for sighting in sorted(sightings, key=lambda item: -item.confidence):
        here = sighting.look.world_t_camera[:3, 3]
        for existing in found:
            if float(np.linalg.norm(np.asarray(existing.centre) - here)) <= merge_m:
                existing.views += 1
                break
        else:
            found.append(
                Found(
                    query=query,
                    centre=tuple(float(v) for v in here),  # type: ignore[arg-type]
                    extent=None,
                    depth_m=0.0,
                    confidence=sighting.confidence,
                    frame_id=sighting.look.frame_id,
                    ts=sighting.look.ts,
                    box2d=sighting.box2d,
                    world_t_camera=sighting.look.world_t_camera,
                )
            )
    return found

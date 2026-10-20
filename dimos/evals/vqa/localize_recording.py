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

"""Optional localize evidence for VQA generation from a Memory recording.

Default path (no preset queries):

1. Discover object names in the window with prompt-free YOLO-E.
2. Run Dan localize to verify those names in 3D.
3. Attach per-frame :class:`LocalizeSceneEvidence` from ``base_link`` pose.

Does not modify ``dimos.perception.localize``.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Iterator, Sequence
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path

from dimos.evals.vqa.contracts import QuestionProposal
from dimos.evals.vqa.primitives.localize_scene import (
    LocalizeInstanceEvidence,
    LocalizeSceneEvidence,
    format_referent,
    next_to_referents,
    scene_from_named_hits,
)
from dimos.memory.cli.dataset import open_dataset
from dimos.perception.localize.rig import TF_TOLERANCE, Rig, window_tf
from dimos.perception.localize.types import Localization
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

BASE_FRAME = "base_link"
_DISCOVERY_MAX_FRAMES = 12
_DISCOVERY_MAX_QUERIES = 12
# Room-scale Go2 defaults used by the aa5f offline bench (not xArm desktop defaults).
_ROOM_LOCALIZE_POLICY = {
    "candidate_floor": 0.18,
    "accept_score": 0.32,
    "min_views": 1,
    "cluster_radius_m": 0.30,
    "verify_radius_m": 5.0,
    "max_object_extent_m": 2.0,
    "surface_patch_max_rise_m": 0.08,
    "surface_patch_min_drop_m": -0.06,
    "min_points": 13,
    "min_camera_range_m": 0.5,
    "fuse_voxel_m": 0.03,
}


@dataclass(frozen=True)
class LocalizeWindowResult:
    """Verified hits plus a rig whose tf covers the localize window."""

    named_hits: tuple[tuple[str, Localization], ...]
    rig: Rig
    queries: tuple[str, ...]


def robot_pose_xyz_yaw(
    rig: Rig, timestamp: float
) -> tuple[tuple[float, float, float], float] | None:
    """World ``(xyz, yaw_rad)`` of ``base_link`` at ``timestamp``, if tf resolves."""
    transform = rig.tf.get(BASE_FRAME, rig.world_frame, timestamp, TF_TOLERANCE)
    if transform is None:
        return None
    pose = (-transform).to_pose()
    return (pose.x, pose.y, pose.z), float(pose.yaw)


def scene_at_timestamp(
    named_hits: Sequence[tuple[str, Localization]],
    rig: Rig,
    timestamp: float,
) -> LocalizeSceneEvidence | None:
    """Attach robot pose at ``timestamp`` to verified hits; ``None`` without base_link."""
    pose = robot_pose_xyz_yaw(rig, timestamp)
    if pose is None:
        return None
    robot_xyz, robot_yaw = pose
    return scene_from_named_hits(robot_xyz, robot_yaw, named_hits)


# Cap multi-instance categories so full-episode scenes do not explode case counts.
_MAX_MULTI_CATEGORIES = 6
_MAX_HEIGHT_PAIRS = 12
_MAX_NEXT_TO_PAIRS = 6


def proposals_from_localize_scene(scene: LocalizeSceneEvidence) -> tuple[QuestionProposal, ...]:
    """Deterministic localize proposals: unique, nearest, next-to, any-ahead."""
    by_category: dict[str, list[LocalizeInstanceEvidence]] = {}
    display_name: dict[str, str] = {}
    for instance in scene.instances:
        key = instance.object_name.casefold()
        display_name.setdefault(key, instance.object_name)
        by_category.setdefault(key, []).append(instance)

    singles = sorted(
        (display_name[key] for key, hits in by_category.items() if len(hits) == 1),
        key=str.casefold,
    )
    multi = sorted(
        (
            (
                display_name[key],
                len(hits),
                max(hit.semantic_score for hit in hits),
            )
            for key, hits in by_category.items()
            if len(hits) >= 2
        ),
        key=lambda row: (-row[2], row[1], row[0].casefold()),
    )
    multi_names = [name for name, _, _ in multi[:_MAX_MULTI_CATEGORIES]]

    proposals: list[QuestionProposal] = []
    # 1) Rare / unique categories (bare name).
    for name in singles:
        proposals.append(QuestionProposal(family="ego_fore_aft", object_names=(name,)))
        proposals.append(QuestionProposal(family="ego_turn", object_names=(name,)))
        proposals.append(QuestionProposal(family="ego_any_fore_aft", object_names=(name,)))

    # 2) Multi-instance: nearest + existential (no left/right — unnatural in question text).
    for name in multi_names:
        nearest = format_referent("nearest", name)
        proposals.append(QuestionProposal(family="ego_fore_aft", object_names=(nearest,)))
        proposals.append(QuestionProposal(family="ego_turn", object_names=(nearest,)))
        proposals.append(QuestionProposal(family="ego_any_fore_aft", object_names=(name,)))

    # 3) Relational referents: "<anchor> next to <landmark>" (no leaked turn answer).
    for referent in next_to_referents(scene, max_pairs=_MAX_NEXT_TO_PAIRS):
        proposals.append(QuestionProposal(family="ego_fore_aft", object_names=(referent,)))
        proposals.append(QuestionProposal(family="ego_turn", object_names=(referent,)))

    # Height order among unique categories only (stable GT without selector prefixes).
    height_pairs = 0
    for index, left in enumerate(singles):
        for right in singles[index + 1 :]:
            proposals.append(QuestionProposal(family="height_order", object_names=(left, right)))
            height_pairs += 1
            if height_pairs >= _MAX_HEIGHT_PAIRS:
                return tuple(proposals)
    return tuple(proposals)


def discover_object_names(
    rig: Rig,
    after: float,
    before: float,
    *,
    max_frames: int = _DISCOVERY_MAX_FRAMES,
    max_queries: int = _DISCOVERY_MAX_QUERIES,
) -> tuple[str, ...]:
    """Prompt-free 2D recognition over sharp frames in the window → localize queries."""
    from dimos.memory.transform import QualityWindow
    from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode

    if max_frames < 1:
        raise ValueError("max_frames must be at least 1")
    if before <= after:
        raise ValueError("discovery window before must be greater than after")

    camera = next(iter(rig.cameras.values()))
    span = max(before - after, 1e-3)
    window = max(span / max_frames, 0.5)
    frames = list(
        camera.color.after(after)
        .before(before)
        .transform(QualityWindow(lambda image: image.sharpness, window=window))
    )
    if not frames:
        logger.warning("localize discovery found no frames in window")
        return ()

    counts: Counter[str] = Counter()
    detector = Yoloe2DDetector(prompt_mode=YoloePromptMode.LRPC, conf=0.5)
    try:
        for observation in frames[:max_frames]:
            image = observation.data
            image.ts = observation.ts
            detections = detector.predict_image(image)
            for detection in detections.detections:
                name = str(getattr(detection, "name", "") or "").strip()
                if name:
                    counts[name] += 1
    finally:
        detector.stop()

    queries = tuple(name for name, _ in counts.most_common(max_queries))
    logger.info(
        f"localize discovery: {len(frames[:max_frames])} frames → "
        f"{len(queries)} queries {list(queries)}"
    )
    return queries


@contextmanager
def localize_window(
    dataset: str | Path,
    queries: Sequence[str],
    after: float,
    before: float,
    *,
    embedder: str = "siglip",
    detector: str = "owlv2",
    segmenter: str = "edgetam",
) -> Iterator[LocalizeWindowResult]:
    """Discover (if needed), localize, and yield verified hits + rig over ``[after, before]``."""
    if before <= after:
        raise ValueError("localize window before must be greater than after")

    # Heavy stack stays inside the optional path.
    from dimos.perception.localize.dandetect import DanDetector
    from dimos.perception.localize.localize import embed_index

    store = open_dataset(dataset)
    store.start()
    try:
        rig = Rig.from_store(store)
        rig.tf = window_tf(store, after, before)
        if rig.lidar is not None:
            rig.lidar = rig.lidar.after(after).before(before)

        query_list = [query.strip() for query in queries if query.strip()]
        if not query_list:
            query_list = list(discover_object_names(rig, after, before))
        if not query_list:
            logger.warning("localize window has no discovered or provided queries")
            yield LocalizeWindowResult(named_hits=(), rig=rig, queries=())
            return

        named: list[tuple[str, Localization]] = []
        with DanDetector(embedder, detector, segmenter) as dan:
            index = embed_index(rig, dan.embedder, after, before)
            policy = dan.policy(_ROOM_LOCALIZE_POLICY)
            results = dan.localize(query_list, index=index, rig=rig, policy=policy)
            for query, hits in zip(query_list, results, strict=True):
                logger.info(f"localize verification {query!r}: {len(hits)} hit(s)")
                for hit in hits:
                    named.append((query, hit))

        yield LocalizeWindowResult(
            named_hits=tuple(named),
            rig=rig,
            queries=tuple(query_list),
        )
    finally:
        store.stop()

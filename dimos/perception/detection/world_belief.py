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

from collections import Counter
from dataclasses import dataclass, field
import hashlib
import math
import re
from typing import TYPE_CHECKING, Any
import uuid

import numpy as np
from scipy.optimize import linear_sum_assignment

from dimos.models.embedding.base import Embedding
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.absence import ABSENT, PRESENT, classify_visibility
from dimos.perception.detection.identity_features import (
    add_diverse_embedding_view,
    gallery_cos,
    normalize_embedding,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.perception.detection.type.detection3d.object import Object

logger = setup_logger()

TENTATIVE = "tentative"  # established identity seen from one camera viewpoint
CONFIRMED = "confirmed"  # established identity seen from multiple viewpoints; not action safety

# How the entity's identity was last established
BASIS_NEW = "new"
BASIS_TRACKED = "tracked"
BASIS_REACQUIRED = "reacquired"
BASIS_RESTORED = "restored"  # past-session identity restored from the gallery store
BASIS_UNRESOLVED = "unresolved"  # visible observation without a defensible identity

DEFAULT_DINO_MODEL = "facebook/dinov2-base"

Viewpoint = tuple[Vector3, Vector3]  # (camera position, forward direction)


class WorldBeliefConfig(BaseConfig):
    """Identity and lifecycle thresholds."""

    label_override_cos: float = 0.8
    reacq_cos: float = 0.5
    reacq_margin: float = 0.05
    min_frames: int = 3  # establishment: frames AND span both required
    min_span_s: float = 1.5
    absent_threshold: int = 2  # depth-sees-through votes before state flips to absent
    min_viewpoints: int = 2  # distinct vantages for `confirmed` trust
    viewpoint_angle_deg: float = 10.0
    min_baseline_m: float = 0.05
    gallery_novelty: float = 0.9
    gallery_max: int = 8
    candidate_ttl_s: float = 5.0  # drop unestablished tracks after this long unseen
    max_absent: int = 128  # bound retained absent tracks
    size_window: int = 9
    frame_id: str = "world"  # frame the fold (and every track position) lives in
    history_path: str | None = None  # cross-session gallery store; None = RAM only
    appearance_model_id: str = DEFAULT_DINO_MODEL


def camera_viewpoint(pose: Pose) -> Viewpoint:
    """Return camera position and forward axis for viewpoint diversity."""
    return pose.position, pose.orientation.rotate_vector(Vector3.unit_z())


def is_novel_viewpoint(
    viewpoint: Viewpoint,
    existing: list[Viewpoint],
    *,
    min_baseline_m: float,
    angle_deg: float,
) -> bool:
    """Return whether no stored viewpoint is nearby in both position and angle."""
    pos, fwd = viewpoint
    cos_thresh = math.cos(math.radians(angle_deg))
    for prev_pos, prev_fwd in existing:
        near_in_space = pos.distance(prev_pos) < min_baseline_m
        near_in_angle = fwd.dot(prev_fwd) > cos_thresh
        if near_in_space and near_in_angle:
            return False
    return True


def _appearance_vec(obj: Object) -> NDArray[np.float32] | None:
    return normalize_embedding(obj.visual_embedding)


@dataclass(slots=True)
class _Track:
    """One believed entity: the folded Object plus its identity/lifecycle state."""

    obj: Object
    last_seen: float
    sizes: list[float]
    label_counts: Counter[str]
    gallery: list[NDArray[np.float32]] = field(default_factory=list)
    latest_emb: NDArray[np.float32] | None = None
    viewpoints: list[Viewpoint] = field(default_factory=list)
    basis: str = BASIS_NEW
    first_ts: float = 0.0
    n_frames: int = 1
    established: bool = False
    state: str = "active"  # "active" | "absent"
    absent_votes: int = 0

    @classmethod
    def from_observation(
        cls, obj: Object, ts: float, viewpoint: Viewpoint | None, basis: str
    ) -> _Track:
        try:
            sizes = [float(max(obj.size.x, obj.size.y, obj.size.z))]
        except Exception:
            sizes = []
        return cls(
            obj=obj,
            last_seen=ts,
            sizes=sizes,
            label_counts=Counter([obj.name]),
            viewpoints=[viewpoint] if viewpoint is not None else [],
            basis=basis,
            first_ts=ts,
        )

    @property
    def modal_label(self) -> str:
        return self.label_counts.most_common(1)[0][0]

    def appearance_views(self) -> list[NDArray[np.float32]]:
        """Return gallery views plus the latest embedding."""
        return list(self.gallery) if self.latest_emb is None else [*self.gallery, self.latest_emb]

    def ingest_appearance(self, obj: Object, novelty: float, max_size: int) -> None:
        vec = _appearance_vec(obj)
        if vec is None:
            return
        if self.gallery and vec.size != self.gallery[0].size:
            # A changed visual model must not pollute a gallery calibrated for one model.
            return
        add_diverse_embedding_view(self.gallery, vec, novelty=novelty, max_size=max_size)
        self.latest_emb = vec


def _gallery_namespace(model_id: str) -> str:
    slug = re.sub(r"[^0-9A-Za-z]+", "_", model_id).strip("_")
    # Hash alternate checkpoint IDs to prevent slug collisions.
    suffix = (
        ""
        if model_id == DEFAULT_DINO_MODEL
        else f"_{hashlib.sha256(model_id.encode()).hexdigest()[:16]}"
    )
    return f"object_gallery_dino_{slug}{suffix}_v2"


class _GalleryStore:
    """Persistent, bounded DINO views keyed by object identity."""

    def __init__(
        self,
        path: str,
        *,
        model_id: str = DEFAULT_DINO_MODEL,
        novelty: float,
        max_views: int,
    ) -> None:
        from dimos.memory2.store.sqlite import SqliteStore

        self._store = SqliteStore(path=path)
        self._namespace = _gallery_namespace(model_id)
        self._novelty = novelty
        self._max_views = max_views

    def _stream_for(self, vec: NDArray[np.float32]) -> Any:
        # Explicit model/schema namespace plus dimension prevents incompatible reuse.
        return self._store.stream(f"{self._namespace}_{vec.size}", str)

    def remember(
        self,
        eid: str,
        obj: Object,
        *,
        name: str | None = None,
    ) -> None:
        """Persist a novel view without interrupting the live fold on failure."""
        try:
            vec = _appearance_vec(obj)
            if vec is None:
                return
            stream = self._stream_for(vec)
            views = stream.tags(object_id=eid).to_list()
            # Store the vector payload because tag filters run after vector top-k.
            stored = [np.fromstring(view.data, dtype=np.float32, sep=" ") for view in views]
            similarity = gallery_cos(vec, stored)
            if similarity is not None and similarity >= self._novelty:
                return
            if len(views) >= self._max_views:
                return
            stream.append(
                " ".join(f"{value:.9g}" for value in vec),
                tags={
                    "object_id": eid,
                    "name": name or obj.name,
                },
                embedding=Embedding(vec),
            )
        except Exception as e:
            logger.warning("gallery remember failed (identity continues in RAM): %s", e)

    def lookup(self, obj: Object) -> dict[str, tuple[float, str]]:
        """Best appearance cosine and label per persisted identity."""
        vec = _appearance_vec(obj)
        if vec is None:
            return {}
        best_cos: dict[str, float] = {}
        newest: dict[str, tuple[float, str]] = {}
        for hit in self._stream_for(vec).search(Embedding(vec), k=64).to_list():
            eid = hit.tags.get("object_id")
            if eid is None:
                continue
            cos = float(hit.similarity or 0.0)
            best_cos[eid] = max(best_cos.get(eid, -1.0), cos)
            appended = float(hit.ts or 0.0)
            if eid not in newest or appended >= newest[eid][0]:
                newest[eid] = (appended, hit.tags.get("name") or "")
        return {eid: (best_cos[eid], newest[eid][1]) for eid in best_cos}

    def close(self) -> None:
        self._store.dispose()


class WorldBelief:
    """Fold lifted detections into persistent identities and geometric presence."""

    def __init__(self, config: WorldBeliefConfig | None = None) -> None:
        self._cfg = config or WorldBeliefConfig()
        self._tracks: dict[str, _Track] = {}
        self._unresolved: list[Object] = []
        # Strictly increasing fold watermark.
        self._now: float = float("-inf")
        # A history path enables cross-session identity persistence.
        self._gallery = (
            _GalleryStore(
                self._cfg.history_path,
                model_id=self._cfg.appearance_model_id,
                novelty=self._cfg.gallery_novelty,
                max_views=self._cfg.gallery_max,
            )
            if self._cfg.history_path is not None
            else None
        )

    @property
    def last_fold_ts(self) -> float:
        """Return the latest folded timestamp, or zero before the first fold."""
        return max(self._now, 0.0)

    def observe(
        self,
        objects: list[Object],
        *,
        frame_ts: float,
        camera_transform: Transform | None,
        camera_info: CameraInfo,
        depth_m: NDArray[np.float32],
    ) -> None:
        """Fold one frame once, in timestamp order."""
        ts = float(frame_ts)
        if ts <= self._now:
            return  # duplicate or out of order
        previous_ts = self._now
        # Without a camera pose, fold identity but not viewpoint or absence evidence.
        viewpoint = (
            camera_viewpoint(camera_transform.to_pose()) if camera_transform is not None else None
        )
        for obj in objects:
            # Normalize the center before association and publication.
            obj.set_center(self._observation_center(obj))

        matched, unmatched, unresolved, seen = self._match_frame(objects, ts)
        self._now = ts  # advance only after identity lookup succeeds
        for obj in unresolved:
            obj.identity_status = None
            obj.identity_basis = BASIS_UNRESOLVED
            obj.last_seen_ts = ts
        self._unresolved = unresolved
        for identity, obj, similarity in matched:
            if isinstance(identity, str):
                self._insert(obj, ts, viewpoint, force_id=identity, basis=BASIS_RESTORED)
                track = self._tracks[identity]
                seen.add(id(track))
                logger.info(
                    "belief: %s (%s) restored from persistent gallery cos=%.3f at (%.2f, %.2f)",
                    identity[:8],
                    obj.name,
                    similarity,
                    obj.center.x,
                    obj.center.y,
                )
                continue
            gap = ts - identity.last_seen
            basis = BASIS_TRACKED if identity.last_seen == previous_ts else BASIS_REACQUIRED
            self._hit(identity, obj, ts, viewpoint, basis)
            if basis == BASIS_REACQUIRED:
                logger.info(
                    "belief: %s (%s) reacquired cos=%.3f gap=%.2fs",
                    identity.obj.object_id[:8],
                    identity.modal_label,
                    similarity,
                    gap,
                )
        for obj in unmatched:
            self._insert(obj, ts, viewpoint, basis=BASIS_NEW)
            seen.add(id(self._tracks[obj.object_id]))
        if camera_transform is not None:
            self._vote_absence(seen, camera_info, camera_transform, depth_m)
        self._prune(ts)

    def _match_frame(
        self, objects: list[Object], ts: float
    ) -> tuple[list[tuple[_Track | str, Object, float]], list[Object], list[Object], set[int]]:
        """Globally associate RAM and persisted identities by appearance."""
        tracks = [
            track
            for track in self._tracks.values()
            if track.established or ts - track.last_seen <= self._cfg.candidate_ttl_s
        ]
        if not objects:
            return [], [], [], set()

        history = [
            self._gallery.lookup(obj) if self._gallery is not None else {} for obj in objects
        ]
        history_ids = sorted({eid for hits in history for eid in hits if eid not in self._tracks})
        history_labels: dict[str, str] = {}
        for hits in history:
            for eid, (_cos, label) in hits.items():
                history_labels[eid] = label
        labels = [track.modal_label for track in tracks] + [
            history_labels[eid] for eid in history_ids
        ]
        candidate_count = len(labels)
        if candidate_count == 0:
            return [], objects, [], set()

        scores = np.full((len(objects), candidate_count + len(objects)), self._cfg.reacq_cos)
        appearance = np.full((len(objects), candidate_count), np.nan)
        valid = np.zeros((len(objects), candidate_count), dtype=bool)
        for row, obj in enumerate(objects):
            vec = _appearance_vec(obj)
            for col, track in enumerate(tracks):
                cos = gallery_cos(vec, track.appearance_views())
                stored = history[row].get(track.obj.object_id)
                if stored is not None and (cos is None or stored[0] > cos):
                    cos = stored[0]
                if cos is not None:
                    appearance[row, col] = cos
                threshold = (
                    self._cfg.reacq_cos if obj.name == labels[col] else self._cfg.label_override_cos
                )
                if cos is not None and cos >= threshold:
                    scores[row, col] = cos
                    valid[row, col] = True
            for offset, eid in enumerate(history_ids, start=len(tracks)):
                hit = history[row].get(eid)
                if hit is None:
                    continue
                cos = hit[0]
                appearance[row, offset] = cos
                threshold = (
                    self._cfg.reacq_cos
                    if obj.name == labels[offset]
                    else self._cfg.label_override_cos
                )
                if cos >= threshold:
                    scores[row, offset] = cos
                    valid[row, offset] = True

        rows, cols = linear_sum_assignment(scores, maximize=True)
        best_total = float(scores[rows, cols].sum())
        matched: list[tuple[_Track | str, Object, float]] = []
        used_tracks: set[int] = set()
        used_objs: set[int] = set()
        # Do not mint identities from ambiguous appearance evidence.
        ambiguous_rows = set(np.flatnonzero(np.any(appearance >= self._cfg.reacq_cos, axis=1)))
        for row, obj in enumerate(objects):
            if _appearance_vec(obj) is None and obj.name in labels:
                ambiguous_rows.add(row)
        for row, col in zip(rows, cols, strict=True):
            if col >= candidate_count or not valid[row, col]:
                continue
            rivals = np.delete(appearance[row], col)
            if np.any(
                (rivals >= self._cfg.reacq_cos)
                & (rivals >= scores[row, col] - self._cfg.reacq_margin)
            ):
                logger.debug("belief: %s association refused — DINO identity near-tie", obj.name)
                continue
            alternative = scores.copy()
            alternative[row, col] = self._cfg.reacq_cos
            alt_rows, alt_cols = linear_sum_assignment(alternative, maximize=True)
            alt_total = float(alternative[alt_rows, alt_cols].sum())
            if best_total - alt_total < self._cfg.reacq_margin:
                logger.debug(
                    "belief: %s association refused — global appearance margin %.3f < %.3f",
                    objects[row].name,
                    best_total - alt_total,
                    self._cfg.reacq_margin,
                )
                continue
            identity: _Track | str = (
                tracks[col] if col < len(tracks) else history_ids[col - len(tracks)]
            )
            matched.append((identity, objects[row], float(scores[row, col])))
            if isinstance(identity, _Track):
                used_tracks.add(id(identity))
            used_objs.add(id(objects[row]))
            ambiguous_rows.discard(row)

        return (
            matched,
            [
                obj
                for row, obj in enumerate(objects)
                if id(obj) not in used_objs and row not in ambiguous_rows
            ],
            [obj for row, obj in enumerate(objects) if row in ambiguous_rows],
            used_tracks,
        )

    def _insert(
        self,
        obj: Object,
        ts: float,
        viewpoint: Viewpoint | None,
        *,
        basis: str,
        force_id: str | None = None,
    ) -> None:
        eid = force_id or obj.object_id
        while force_id is None and eid in self._tracks:
            eid = uuid.uuid4().hex
        obj.object_id = eid
        track = _Track.from_observation(obj, ts, viewpoint, basis)
        track.established = basis == BASIS_RESTORED or self._is_established(track, ts)
        track.ingest_appearance(obj, self._cfg.gallery_novelty, self._cfg.gallery_max)
        self._tracks[eid] = track
        self._update_identity_facts(track)
        if self._gallery is not None and track.established:
            self._gallery.remember(eid, obj, name=track.modal_label)

    def _is_established(self, track: _Track, ts: float) -> bool:
        return (
            track.n_frames >= self._cfg.min_frames and (ts - track.first_ts) >= self._cfg.min_span_s
        )

    def _hit(
        self,
        track: _Track,
        obj: Object,
        ts: float,
        viewpoint: Viewpoint | None,
        basis: str,
    ) -> None:
        was_absent = track.state == "absent"
        was_established = track.established
        reacquired = was_absent or basis == BASIS_REACQUIRED
        track.state = "active"
        track.absent_votes = 0
        track.basis = basis
        track.n_frames += 1
        track.label_counts[obj.name] += 1
        if not track.established:
            track.established = self._is_established(track, ts)
        track.ingest_appearance(obj, self._cfg.gallery_novelty, self._cfg.gallery_max)
        new_pos = obj.center
        track.last_seen = max(track.last_seen, ts)
        suspect = self._geometry_suspect(track, obj)
        if suspect and reacquired:
            # Refresh observable geometry without folding a suspect shape.
            track.obj.set_center(new_pos)
            track.obj.pointcloud = obj.pointcloud
            track.obj.ts = obj.ts
            track.obj.frame_id = obj.frame_id
            track.obj.pose.ts = obj.pose.ts
            track.obj.pose.frame_id = obj.pose.frame_id
            track.obj.image = obj.image
            track.obj.mask = obj.mask
            track.obj.bbox = obj.bbox
            track.obj.confidence = obj.confidence
            track.obj.camera_transform = obj.camera_transform
            if obj.visual_embedding is not None:
                track.obj.visual_embedding = obj.visual_embedding
            track.obj.observation_partial = True
        if not suspect:
            track.obj.update_object(obj, accumulate_pointcloud=False)
            try:
                track.sizes.append(float(max(obj.size.x, obj.size.y, obj.size.z)))
                del track.sizes[: -self._cfg.size_window]
            except Exception:
                pass
        if (
            viewpoint is not None
            and len(track.viewpoints) < self._cfg.min_viewpoints
            and is_novel_viewpoint(
                viewpoint,
                track.viewpoints,
                min_baseline_m=self._cfg.min_baseline_m,
                angle_deg=self._cfg.viewpoint_angle_deg,
            )
        ):
            track.viewpoints.append(viewpoint)
        self._update_identity_facts(track)
        if not was_established and track.established:
            logger.info(
                "belief: %s (%s) established frames=%d span=%.2fs",
                track.obj.object_id[:8],
                track.modal_label,
                track.n_frames,
                ts - track.first_ts,
            )
        if self._gallery is not None and track.established:
            self._gallery.remember(
                track.obj.object_id,
                obj,
                name=track.modal_label,
            )

    @staticmethod
    def _observation_center(obj: Object) -> Vector3:
        """Return the median world-space surface point, falling back to ``obj.center``."""
        try:
            pts = np.asarray(obj.pointcloud.pointcloud.points)
        except Exception:
            return obj.center
        if pts.ndim != 2 or len(pts) < 8:
            return obj.center
        return Vector3(np.median(pts, axis=0))

    def _geometry_suspect(self, track: _Track, obj: Object) -> bool:
        """Reject partial or >2x median-size geometry without folding it into the reference."""
        if getattr(obj, "observation_partial", False):
            return True
        try:
            ref = float(np.median(track.sizes)) if track.sizes else 0.0
            if ref <= 0.0:
                return False
            obs_dim = float(max(obj.size.x, obj.size.y, obj.size.z))
            return obs_dim > 2.0 * ref + 0.05
        except Exception:
            return False

    def _vote_absence(
        self,
        matched_tracks: set[int],
        camera_info: CameraInfo,
        world_from_camera: Transform,
        depth_m: NDArray[np.float32],
    ) -> None:
        for track in self._tracks.values():
            if id(track) in matched_tracks or track.state == "absent" or not track.established:
                continue
            # Include the object's near surface in the expected depth band.
            half_extent = 0.0
            try:
                half_extent = min(
                    0.3, 0.5 * max(track.obj.size.x, track.obj.size.y, track.obj.size.z)
                )
            except Exception:
                pass
            verdict = classify_visibility(
                track.obj.center, camera_info, world_from_camera, depth_m, near_extent_m=half_extent
            )
            if verdict == ABSENT:
                track.absent_votes += 1
                if track.absent_votes >= self._cfg.absent_threshold:
                    track.state = "absent"
                    logger.info(
                        "belief: %s (%s) absent — depth saw through its spot %d time(s)",
                        track.obj.object_id[:8],
                        track.modal_label,
                        track.absent_votes,
                    )
            elif verdict == PRESENT:
                track.absent_votes = 0  # observed surface cancels absence evidence
            # Occlusion and out-of-view provide no absence evidence.

    def _prune(self, ts: float) -> None:
        """Remove stale candidates and bound absent-track retention."""
        stale = [
            eid
            for eid, t in self._tracks.items()
            if not t.established and (ts - t.last_seen) > self._cfg.candidate_ttl_s
        ]
        for eid in stale:
            del self._tracks[eid]
        absent = [eid for eid, t in self._tracks.items() if t.state == "absent"]
        if len(absent) > self._cfg.max_absent:
            for eid in sorted(absent, key=lambda e: self._tracks[e].last_seen)[
                : len(absent) - self._cfg.max_absent
            ]:
                del self._tracks[eid]

    def _update_identity_facts(self, track: _Track) -> None:
        track.obj.name = track.modal_label
        track.obj.identity_status = (
            CONFIRMED
            if track.established and len(track.viewpoints) >= self._cfg.min_viewpoints
            else TENTATIVE
            if track.established
            else None
        )
        track.obj.identity_basis = track.basis
        track.obj.last_seen_ts = track.last_seen

    def present(self) -> list[Object]:
        """Return established, non-absent objects; callers must treat them as read-only."""
        present: list[Object] = []
        for track in self._tracks.values():
            if track.state == "active" and track.established:
                self._update_identity_facts(track)
                present.append(track.obj)
        return present

    def observations(self) -> list[Object]:
        """Return stable present identities plus latest unresolved physical observations."""
        return [*self.present(), *self._unresolved]

    def close(self) -> None:
        """Close persistent gallery storage."""
        if self._gallery is not None:
            self._gallery.close()

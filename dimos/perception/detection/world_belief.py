# Copyright 2025-2026 Dimensional Inc.
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

"""Support-gated object identity association.

``WorldBelief`` is API-compatible with ``ObjectDB`` but separates maintained
identity from the currently present set. An identity stays in the maintained
table until eviction; it becomes present only with recent support or explicit
promotion.

Association uses explicit evidence records for tracker continuity, density-aware
geometry, embedding compatibility, and optional re-acquisition for recently
departed identities. Tracker ids are evidence, not authority. Per-frame
co-occurrence prevents two simultaneous tracks from collapsing onto one id.

Time is driven by detection timestamps or explicit empty-frame timestamps. The
wall clock is only a bootstrap fallback, which keeps replayed streams and full
occlusion frames from accidentally evicting every maintained identity.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import replace as dataclass_replace
import threading
import time
from typing import TYPE_CHECKING, Any

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.identity_association import (
    AssociationDecision,
    AssociationEvidence,
    IdentityAssociationCandidate,
    IdentityAssociationPolicy,
)
from dimos.perception.detection.identity_features import (
    add_diverse_embedding_view,
    gallery_cos,
    normalize_embedding,
)
from dimos.perception.detection.world_belief_history import WorldBeliefHistoryStore
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.perception.detection.type.detection3d.object import Object

logger = setup_logger()


class WorldBelief:
    """Support-gated object identity table with the ObjectDB public API."""

    def __init__(
        self,
        distance_threshold: float = 0.2,
        track_id_ttl_s: float = 5.0,
        *,
        min_support: int = 4,
        recent_window: float = 1.5,
        eviction_ttl_s: float = 60.0,
        anchor_window: int = 9,
        sticky_support: bool = True,
        label_gate: bool = True,
        cooccurrence_gate: bool = True,
        enable_history: bool = False,
        history_path: str | None = None,
        history_stream: str = "worldbelief_obs",
        reid_reacquire: bool = True,
        reacq_window: float = 8.0,
        reacq_radius: float | None = 1.0,
        reacq_cos: float = 0.6,
        reacq_margin: float = 0.10,
        gallery_size: int = 8,
        gallery_novelty: float = 0.88,
        density_gate: bool = True,
        density_factor: float = 0.45,
        position_jitter_margin: float = 0.03,
        position_ambiguity_margin: float = 0.03,
        suppress_partial_observation_creates: bool = True,
        candidate_gate_new_objects: bool = True,
        new_candidate_min_age_s: float = 5.0,
        new_candidate_ttl_s: float = 10.0,
        new_candidate_max_count: int = 32,
        reacq_override: bool = True,
    ) -> None:
        self._distance_threshold = distance_threshold
        self._track_id_ttl_s = track_id_ttl_s
        self._min_support = int(min_support)
        self._recent_window = recent_window
        self._eviction_ttl_s = eviction_ttl_s
        self._anchor_window = int(anchor_window)
        self._sticky_support = sticky_support
        self._label_gate = label_gate
        self._cooccurrence_gate = cooccurrence_gate
        self._reid_reacquire = reid_reacquire
        self._reacq_window = reacq_window
        self._reacq_radius = reacq_radius
        self._reacq_cos = reacq_cos
        self._reacq_margin = reacq_margin
        self._gallery_size = int(gallery_size)
        self._gallery_novelty = gallery_novelty
        self._density_gate = density_gate
        self._density_factor = density_factor
        self._position_jitter_margin = position_jitter_margin
        self._position_ambiguity_margin = position_ambiguity_margin
        self._identity_policy = IdentityAssociationPolicy()
        self._suppress_partial_observation_creates = suppress_partial_observation_creates
        self._candidate_gate_new_objects = candidate_gate_new_objects
        self._new_candidate_min_age_s = new_candidate_min_age_s
        self._new_candidate_ttl_s = new_candidate_ttl_s
        self._new_candidate_max_count = int(new_candidate_max_count)
        self._reacq_override = reacq_override

        self._entities: dict[str, Object] = {}
        self._meta: dict[str, dict[str, Any]] = {}
        self._track_id_map: dict[tuple[str, int], str] = {}
        self._promoted: set[str] = set()  # objects force-kept present (e.g. select_object RPC)
        self._new_candidates: dict[str, dict[str, Any]] = {}
        self._last_add_stats: dict[str, int] = {}
        self._last_association_evidence: list[AssociationEvidence] = []
        self._now: float = 0.0
        self._lock = threading.RLock()

        self._history = WorldBeliefHistoryStore(
            enabled=enable_history,
            path=history_path,
            stream_name=history_stream,
            gallery_size=self._gallery_size,
            gallery_novelty=self._gallery_novelty,
        )

    def _frame_time(self, objects: list[Object], frame_ts: float | None) -> float:
        ts_vals = [float(o.ts) for o in objects if getattr(o, "ts", None) is not None]
        if ts_vals:
            return max(ts_vals)
        if frame_ts is not None:
            return max(self._now, float(frame_ts))
        return self._now or time.time()

    def _recent_cut(self) -> float:
        return self._now - self._recent_window

    def _recent_support(self, eid: str, cut: float | None = None) -> int:
        if cut is None:
            cut = self._recent_cut()
        return sum(1 for t in self._meta[eid]["window"] if t >= cut)

    def _is_present(self, eid: str, cut: float | None = None) -> bool:
        return self._recent_support(eid, cut) >= self._min_support or eid in self._promoted

    def _is_departed(self, eid: str, cut: float | None = None) -> bool:
        if cut is None:
            cut = self._recent_cut()
        return self._meta[eid]["last_seen"] < cut

    def _set_empty_add_stats(self) -> None:
        self._last_add_stats = {
            "input": 0,
            "created": 0,
            "updated": 0,
            "matched_track": 0,
            "matched_distance": 0,
            "present": len(self.get_objects()),
            "maintained": len(self._entities),
        }
        self._last_association_evidence = []


    def add_objects(
        self,
        objects: list[Object],
        evict_exempt: set[str] | None = None,
        frame_ts: float | None = None,
    ) -> list[Object]:
        """Ingest one frame of detections and return the current present objects."""
        stats = {
            "input": len(objects),
            "created": 0,
            "updated": 0,
            "matched_track": 0,
            "matched_distance": 0,
        }
        with self._lock:
            now = self._frame_time(objects, frame_ts)
            self._now = now
            scene_established_at_frame_start = len(self._entities) >= max(3, self._min_support)
            frame_claims: dict[
                str, tuple[str, int]
            ] = {}  # eid -> observation claim key for THIS frame
            frame_candidate_claims: dict[str, tuple[str, int]] = {}
            self._last_association_evidence = []
            position_assignments = (
                self._frame_position_assignments(objects)
                if len(objects) > 1 and self._entities
                else {}
            )
            processing_order = sorted(
                range(len(objects)),
                key=lambda idx: (idx not in position_assignments, idx),
            )
            results_by_index: dict[int, Any] = {}
            for idx in processing_order:
                obj = objects[idx]
                eid, reason = self._associate(
                    obj,
                    frame_claims,
                    position_assignment=position_assignments.get(idx),
                )
                if eid is None:
                    # During cold start, even partial detections are useful inventory. Once a
                    # scene is established, suppress only moving-camera partials here; stable
                    # edge-visible objects can still mature through the candidate gate below.
                    suppress_partial = (
                        self._suppress_partial_observation_creates
                        and getattr(obj, "observation_partial_from_camera_motion", False)
                        and scene_established_at_frame_start
                    )
                    if suppress_partial:
                        stats["suppressed_partial"] = stats.get("suppressed_partial", 0) + 1
                        continue
                    if self._candidate_gate_new_objects and scene_established_at_frame_start:
                        promoted = self._update_new_candidate(obj, now, frame_candidate_claims)
                        if promoted is None:
                            stats["pending_candidate"] = stats.get("pending_candidate", 0) + 1
                            continue
                        new_obj = promoted
                    else:
                        new_obj = self._insert(obj, now)
                    results_by_index[idx] = new_obj
                    stats["created"] += 1
                    # Prevent another same-frame detection from merging onto this new entity.
                    frame_claims[new_obj.object_id] = self._frame_claim_key(obj)
                    continue
                if reason == "reid":
                    # Re-acquisition means the old position anchor should not resist the new location.
                    self._meta[eid]["positions"] = []
                self._update(eid, obj, now)
                results_by_index[idx] = self._entities[eid]
                stats["updated"] += 1
                stats["matched_track" if reason == "track" else "matched_distance"] += 1
                frame_claims[eid] = self._frame_claim_key(obj)
            self._evict_stale(now, evict_exempt)
        stats["present"] = len(self.get_objects())
        stats["maintained"] = len(self._entities)
        self._last_add_stats = stats
        if stats["created"] > 0:
            logger.info(f"WorldBelief: {stats}")
        return [results_by_index[idx] for idx in range(len(objects)) if idx in results_by_index]

    def advance_time(self, frame_ts: float, evict_exempt: set[str] | None = None) -> None:
        """Advance the observation clock for a frame that produced no 3D objects.

        This lets the support-confirmed present-set decay during full occlusion or detector blackout
        without falling back to wall-clock time on replayed streams.
        """
        with self._lock:
            if frame_ts <= 0:
                return
            now = max(self._now, float(frame_ts))
            self._now = now
            self._evict_stale(now, evict_exempt)
            self._set_empty_add_stats()

    def get_last_add_stats(self) -> dict[str, int]:
        with self._lock:
            return dict(self._last_add_stats)

    def get_last_association_evidence(self) -> list[dict[str, Any]]:
        """Return evidence records from the most recent add/advance call."""
        with self._lock:
            return [e.to_dict() for e in self._last_association_evidence]

    def get_identity_gallery_stats(self) -> dict[str, int]:
        """Return public diagnostics for restored/stored identity feature galleries."""
        with self._lock:
            semantic_views = sum(len(m.get("semantic_gallery") or ()) for m in self._meta.values())
            visual_views = sum(len(m.get("visual_gallery") or ()) for m in self._meta.values())
            return {
                "entity_count": len(self._meta),
                "semantic_gallery_entity_count": sum(
                    1 for m in self._meta.values() if m.get("semantic_gallery")
                ),
                "semantic_gallery_view_count": semantic_views,
                "visual_gallery_entity_count": sum(
                    1 for m in self._meta.values() if m.get("visual_gallery")
                ),
                "visual_gallery_view_count": visual_views,
            }

    def get_table_snapshot(self) -> list[dict[str, Any]]:
        """Return compact metadata for the maintained object table."""
        with self._lock:
            rows: list[dict[str, Any]] = []
            track_ids_by_entity: dict[str, list[int]] = defaultdict(list)
            for (_namespace, track_id), eid in self._track_id_map.items():
                track_ids_by_entity[eid].append(int(track_id))

            for eid, obj in sorted(self._entities.items()):
                m = self._meta.get(eid, {})
                center = getattr(obj, "center", None)
                size = getattr(obj, "size", None)
                present = self._is_present(eid)
                lifecycle = "present" if present else "maintained"

                rows.append(
                    {
                        "object_id": eid,
                        "name": getattr(obj, "name", ""),
                        "present": present,
                        "lifecycle": lifecycle,
                        "center": None
                        if center is None
                        else [float(center.x), float(center.y), float(center.z)],
                        "size": None
                        if size is None
                        else [float(size.x), float(size.y), float(size.z)],
                        "entered_t": float(m.get("entered_t", 0.0) or 0.0),
                        "last_seen": float(m.get("last_seen", 0.0) or 0.0),
                        "support": int(m.get("support", 0) or 0),
                        "recent_support": int(self._recent_support(eid))
                        if eid in self._meta
                        else 0,
                        "track_ids": sorted(set(track_ids_by_entity.get(eid, []))),
                        "semantic_gallery_size": len(m.get("semantic_gallery") or ()),
                        "visual_gallery_size": len(m.get("visual_gallery") or ()),
                        "partial": bool(getattr(obj, "observation_partial", False)),
                    }
                )
            return rows

    def get_objects(self) -> list[Object]:
        """Present-set = support-confirmed within the recent window (or force-promoted)."""
        with self._lock:
            cut = self._recent_cut()
            return [self._entities[eid] for eid in self._meta if self._is_present(eid, cut)]

    def get_all_objects(self) -> list[Object]:
        with self._lock:
            return list(self._entities.values())

    def promote(self, object_id: str) -> bool:
        with self._lock:
            if object_id in self._entities:
                self._promoted.add(object_id)
                return True
            return False

    def find_by_name(self, name: str) -> list[Object]:
        with self._lock:
            return [o for o in self.get_objects() if o.name == name]

    def find_by_object_id(self, object_id: str) -> Object | None:
        with self._lock:
            return self._entities.get(object_id)

    def find_nearest(self, position: Vector3, name: str | None = None) -> Object | None:
        with self._lock:
            cands = [
                o
                for o in self.get_objects()
                if o.center is not None and (name is None or o.name == name)
            ]
            if not cands:
                return None
            return min(cands, key=lambda o: position.distance(o.center))

    def clear(self) -> None:
        with self._lock:
            for obj in self._entities.values():
                obj.pointcloud = PointCloud2(
                    pointcloud=o3d.geometry.PointCloud(),
                    frame_id=obj.pointcloud.frame_id,
                    ts=obj.pointcloud.ts,
                )
            self._entities.clear()
            self._meta.clear()
            self._track_id_map.clear()
            self._promoted.clear()
            self._new_candidates.clear()
            self._last_association_evidence = []
            logger.info("WorldBelief cleared")

    def agent_encode(self) -> list[dict[str, Any]]:
        with self._lock:
            return [o.agent_encode() for o in self.get_objects()]


    @staticmethod
    def _median_center(positions) -> Vector3:
        m = np.median(np.asarray(positions, dtype=float), axis=0)
        return Vector3(m)

    def _jitter_radius(self) -> float:
        """Radius for recovering an object after small position jitter."""
        return max(2.0 * self._distance_threshold, 0.16)

    @staticmethod
    def _semantic_emb_of(obj: Object):
        return normalize_embedding(getattr(obj, "embedding", None))

    @staticmethod
    def _visual_emb_of(obj: Object):
        return normalize_embedding(getattr(obj, "visual_embedding", None))

    @staticmethod
    def _role_gallery(m: dict[str, Any], role: str) -> list[Any]:
        if role == "visual":
            return m.get("visual_gallery") or (
                [m["visual_emb"]] if m.get("visual_emb") is not None else []
            )
        return m.get("semantic_gallery") or (
            [m["semantic_emb"]] if m.get("semantic_emb") is not None else []
        )

    def _embedding_compatible_for_role(self, obj: Object, eid: str, role: str) -> bool:
        """True when object and entity embeddings are comparable for one evidence role."""
        m = self._meta.get(eid)
        if m is None:
            return False
        if role == "visual":
            if self._visual_emb_of(obj) is None:
                return False
            obj_model = getattr(obj, "visual_embedding_model", None)
            meta_model = m.get("visual_embedding_model")
            obj_dim = getattr(obj, "visual_embedding_dim", None)
            meta_dim = m.get("visual_embedding_dim")
        else:
            if self._semantic_emb_of(obj) is None:
                return False
            obj_model = getattr(obj, "embedding_model", None)
            meta_model = m.get("semantic_embedding_model")
            obj_dim = getattr(obj, "embedding_dim", None)
            meta_dim = m.get("semantic_embedding_dim")
        if obj_model is not None and meta_model is not None and obj_model != meta_model:
            return False
        if obj_dim is not None and meta_dim is not None and int(obj_dim) != int(meta_dim):
            return False
        return bool(self._role_gallery(m, role))

    def _candidate_embedding_role(self, obj: Object, eid: str) -> str | None:
        """Best comparable embedding role for this object/entity pair."""
        if self._embedding_compatible_for_role(obj, eid, "visual"):
            return "visual"
        if self._embedding_compatible_for_role(obj, eid, "semantic"):
            return "semantic"
        return None

    @staticmethod
    def _tracker_key(obj: Object) -> tuple[str, int] | None:
        track_id = getattr(obj, "track_id", -1)
        if track_id is None or int(track_id) < 0:
            return None
        source = (
            getattr(obj, "tracker_source", None) or getattr(obj, "detector_id", None) or "default"
        )
        return str(source), int(track_id)

    def _frame_claim_key(self, obj: Object) -> tuple[str, int]:
        """Per-frame co-occurrence token, even when a detector has no tracker ids."""
        return self._tracker_key(obj) or ("observation", id(obj))

    def _same_frame_duplicate(self, obj: Object, other: Object) -> bool:
        """Allow near-identical same-frame duplicate detections to collapse onto one entity."""
        if obj.center is None or other.center is None:
            return False
        if self._label_gate and obj.name != other.name:
            return False
        if self._observation_identity_mismatch(obj, other):
            return False
        duplicate_radius = max(0.005, min(0.015, 0.15 * self._distance_threshold))
        return obj.center.distance(other.center) <= duplicate_radius

    def _appearance_similarity_for_role(self, obj: Object, eid: str, role: str) -> float | None:
        emb = self._visual_emb_of(obj) if role == "visual" else self._semantic_emb_of(obj)
        if emb is None or not self._embedding_compatible_for_role(obj, eid, role):
            return None
        m = self._meta.get(eid)
        if m is None:
            return None
        return gallery_cos(emb, self._role_gallery(m, role))

    def _appearance_mismatch(self, obj: Object, eid: str) -> bool:
        """Return True when available identity evidence strongly contradicts a candidate.

        Visual/DINO evidence is preferred for physical identity. Semantic CLIP/SigLIP evidence is a fallback
        only when no visual descriptor is available, so semantic similarity does not dominate lookalike cans.
        """
        visual_similarity = self._appearance_similarity_for_role(obj, eid, "visual")
        if visual_similarity is not None:
            return visual_similarity < self._reacq_cos
        semantic_similarity = self._appearance_similarity_for_role(obj, eid, "semantic")
        if semantic_similarity is not None and self._visual_emb_of(obj) is None:
            return semantic_similarity < self._reacq_cos
        return False

    def _association_evidence(
        self,
        obj: Object,
        eid: str,
        *,
        reason: str,
        distance: float | None = None,
        track_match: bool = False,
        hard_appearance: bool = True,
        accepted: bool = False,
        ambiguous: bool = False,
        margin: float | None = None,
    ) -> AssociationEvidence:
        distance_score = None
        if distance is not None:
            gate = max(self._distance_threshold, 1e-6)
            distance_score = max(0.0, 1.0 - min(float(distance) / gate, 1.0))

        meta = self._meta.get(eid, {})
        labels = meta.get("labels")
        label_match = labels.most_common(1)[0][0] == obj.name if labels else None
        support = int(meta.get("support", 0))
        semantic_similarity = self._appearance_similarity_for_role(obj, eid, "semantic")
        visual_similarity = self._appearance_similarity_for_role(obj, eid, "visual")
        appearance_similarity = (
            visual_similarity if visual_similarity is not None else semantic_similarity
        )
        visual_mismatch = visual_similarity is not None and visual_similarity < self._reacq_cos
        semantic_mismatch = (
            semantic_similarity is not None
            and semantic_similarity < self._reacq_cos
            and visual_similarity is None
        )
        appearance_mismatch = visual_mismatch or semantic_mismatch
        vetoes: list[str] = []
        if hard_appearance and visual_mismatch:
            vetoes.append("visual")
        elif hard_appearance and semantic_mismatch:
            vetoes.append("appearance")

        support_score = min(support / max(self._min_support, 1), 1.0)
        score = 0.0
        if distance_score is not None:
            score += 0.45 * distance_score
        if track_match:
            score += 0.35
        if label_match is True:
            score += 0.10
        if visual_similarity is not None:
            score += 0.25 * max(0.0, visual_similarity)
        elif semantic_similarity is not None:
            score += 0.12 * max(0.0, semantic_similarity)
        score += 0.05 * support_score
        score -= float(len(vetoes))

        return AssociationEvidence(
            observation_name=obj.name,
            candidate_id=eid,
            reason=reason,
            distance=distance,
            distance_score=distance_score,
            track_match=track_match,
            label_match=label_match,
            support=support,
            appearance_similarity=appearance_similarity,
            semantic_similarity=semantic_similarity,
            visual_similarity=visual_similarity,
            appearance_mismatch=appearance_mismatch,
            semantic_mismatch=semantic_mismatch,
            visual_mismatch=visual_mismatch,
            score=score,
            accepted=accepted,
            ambiguous=ambiguous,
            margin=margin,
            vetoes=tuple(vetoes),
        )

    def _record_association_evidence(self, evidence: AssociationEvidence) -> None:
        self._last_association_evidence.append(evidence)

    def _record_accepted_decision(self, decision: AssociationDecision) -> None:
        if decision.evidence is None:
            return
        self._record_association_evidence(
            dataclass_replace(
                decision.evidence,
                accepted=True,
                ambiguous=False,
                margin=decision.margin,
            )
        )

    def _record_accepted_association(
        self,
        obj: Object,
        eid: str,
        *,
        reason: str,
        distance: float | None = None,
        track_match: bool = False,
        hard_appearance: bool = True,
    ) -> None:
        self._record_association_evidence(
            self._association_evidence(
                obj,
                eid,
                reason=reason,
                distance=distance,
                track_match=track_match,
                hard_appearance=hard_appearance,
                accepted=True,
            )
        )

    def _record_ambiguous_association(
        self,
        obj: Object,
        eid: str,
        *,
        reason: str,
        distance: float | None = None,
        track_match: bool = False,
        hard_appearance: bool = True,
        margin: float | None = None,
    ) -> None:
        self._record_association_evidence(
            self._association_evidence(
                obj,
                eid,
                reason=reason,
                distance=distance,
                track_match=track_match,
                hard_appearance=hard_appearance,
                accepted=False,
                ambiguous=True,
                margin=margin,
            )
        )

    def _observation_identity_mismatch(self, obj: Object, other: Object) -> bool:
        """Negative identity evidence between two observations before an entity exists in _meta."""
        a_visual = self._visual_emb_of(obj)
        b_visual = self._visual_emb_of(other)
        if a_visual is not None and b_visual is not None and a_visual.size == b_visual.size:
            a_model = getattr(obj, "visual_embedding_model", None)
            b_model = getattr(other, "visual_embedding_model", None)
            if (a_model is None or b_model is None or a_model == b_model) and float(
                np.dot(a_visual, b_visual)
            ) < self._reacq_cos:
                return True
            return False
        a_emb = self._semantic_emb_of(obj)
        b_emb = self._semantic_emb_of(other)
        if a_emb is not None and b_emb is not None and a_emb.size == b_emb.size:
            a_model = getattr(obj, "embedding_model", None)
            b_model = getattr(other, "embedding_model", None)
            if (a_model is None or b_model is None or a_model == b_model) and float(
                np.dot(a_emb, b_emb)
            ) < self._reacq_cos:
                return True
        return False

    def _reacquire(
        self,
        obj: Object,
        frame_claims: dict[str, tuple[str, int]],
    ) -> str | None:
        """Re-acquire a recently departed identity by appearance."""
        best, best_cos, second_cos, _best_role = self._best_departed(obj, frame_claims)
        if best is not None and best_cos >= self._reacq_cos:
            if (best_cos - second_cos) >= self._reacq_margin:
                return best
            be = self._entities.get(best)
            bd = (
                obj.center.distance(be.center)
                if be is not None and be.center is not None and obj.center is not None
                else None
            )
            self._record_ambiguous_association(
                obj,
                best,
                reason="reid_ambiguous",
                distance=bd,
                margin=best_cos - second_cos,
            )
        return None

    def _best_departed(
        self,
        obj: Object,
        frame_claims: dict[str, tuple[str, int]],
    ):
        """Best DEPARTED identity for this detection by gallery max-cos, with covisibility ELIMINATION
        (present entities excluded) + recency/radius bounds. Returns (eid|None, best_cos, second_cos).
        Shared by re-acquisition (no-position-match path) and the cross-collision override."""
        cut = self._recent_cut()
        best, best_cos, second_cos, best_role = None, -1.0, -1.0, None
        for eid, e in self._entities.items():
            if frame_claims.get(eid) is not None:  # already taken this frame
                continue
            m = self._meta[eid]
            if self._is_present(eid, cut):
                continue
            if (self._now - m["last_seen"]) > self._reacq_window:  # departed too long ago
                continue
            if (
                self._reacq_radius is not None
                and e.center is not None
                and obj.center is not None
                and obj.center.distance(e.center) > self._reacq_radius
            ):  # generous sanity bound only
                continue
            role = self._candidate_embedding_role(obj, eid)
            if role is None:
                continue
            emb = self._visual_emb_of(obj) if role == "visual" else self._semantic_emb_of(obj)
            if emb is None:
                continue
            gal = self._role_gallery(m, role)
            cos = gallery_cos(emb, gal)
            if cos is None:
                continue
            if cos > best_cos:
                best, best_cos, second_cos, best_role = eid, cos, best_cos, role
            elif cos > second_cos:
                second_cos = cos
        return best, best_cos, second_cos, best_role

    def _position_candidates(
        self,
        obj: Object,
        frame_claims: dict[str, tuple[str, int]],
        *,
        record_evidence: bool = True,
    ) -> list[tuple[float, str, Object, AssociationEvidence]]:
        assert obj.center is not None
        ents = [(eid, e) for eid, e in self._entities.items() if e.center is not None]
        if self._density_gate and len(ents) > 1:
            cands = self._density_candidates(obj, ents)
        else:
            cands = []
            for eid, e in ents:
                d = obj.center.distance(e.center)
                if d <= self._distance_threshold:
                    cands.append((d, eid, e))

        claim_key = self._frame_claim_key(obj)
        if self._cooccurrence_gate and cands:
            cands = [
                c
                for c in cands
                if (claimed := frame_claims.get(c[1])) is None
                or claimed == claim_key
                or self._same_frame_duplicate(obj, c[2])
            ]

        if self._label_gate and cands:
            same = [c for c in cands if self._meta[c[1]]["labels"].most_common(1)[0][0] == obj.name]
            if same:
                cands = same
        if cands:
            filtered: list[tuple[float, str, Object]] = []
            for d, eid, entity in cands:
                evidence = self._association_evidence(
                    obj,
                    eid,
                    reason="position",
                    distance=d,
                    hard_appearance=True,
                )
                if record_evidence:
                    self._record_association_evidence(evidence)
                if evidence.vetoes:
                    continue
                filtered.append((d, eid, entity, evidence))
            cands = filtered
        return cands

    def _frame_position_assignments(
        self,
        objects: list[Object],
    ) -> dict[int, AssociationDecision]:
        candidate_sets: dict[int, list[IdentityAssociationCandidate]] = {}
        empty_claims: dict[str, tuple[str, int]] = {}
        for idx, obj in enumerate(objects):
            if obj.center is None:
                continue
            cands = self._position_candidates(obj, empty_claims, record_evidence=False)
            if not cands:
                continue
            candidate_sets[idx] = [
                IdentityAssociationCandidate(
                    candidate_id=eid,
                    distance=d,
                    support=int(self._meta[eid]["support"]),
                    evidence=evidence,
                    departed=self._is_departed(eid),
                )
                for d, eid, _, evidence in cands
            ]
        if not candidate_sets:
            return {}
        return self._identity_policy.choose_frame_position_assignments(
            candidate_sets,
            sticky_support=self._sticky_support,
        )

    def _density_candidates(
        self,
        obj: Object,
        ents: list[tuple[str, Object]],
    ) -> list[tuple[float, str, Object]]:
        assert obj.center is not None
        recent_ents = [(eid, e) for eid, e in ents if not self._is_departed(eid)]
        nearest: dict[str, float] = {}
        for eid, e in ents:
            neighbours = (
                recent_ents
                if self._is_departed(eid)
                else [(oeid, o) for oeid, o in recent_ents if oeid != eid]
            )
            if neighbours:
                nearest[eid] = min(e.center.distance(o.center) for _, o in neighbours)
            else:
                nearest[eid] = self._distance_threshold / max(self._density_factor, 1e-6)

        cands: list[tuple[float, str, Object]] = []
        for eid, e in ents:
            d = obj.center.distance(e.center)
            gate = (
                self._distance_threshold
                if self._is_departed(eid)
                else min(self._distance_threshold, self._density_factor * nearest[eid])
            )
            if d <= gate:
                cands.append((d, eid, e))
        return cands

    def _choose_position_candidate(
        self,
        obj: Object,
        cands: list[tuple[float, str, Object, AssociationEvidence]],
    ):
        policy_candidates = [
            IdentityAssociationCandidate(
                candidate_id=eid,
                distance=d,
                support=int(self._meta[eid]["support"]),
                evidence=evidence,
                departed=self._is_departed(eid),
            )
            for d, eid, _, evidence in cands
        ]
        decision = self._identity_policy.choose_position_candidate(
            policy_candidates,
            sticky_support=self._sticky_support,
            ambiguity_margin=self._position_ambiguity_margin,
        )
        if decision.candidate_id is None:
            raise RuntimeError(
                "position association policy returned no candidate for non-empty input"
            )
        return decision

    def _prune_new_candidates(self, now: float) -> None:
        if not self._new_candidates:
            return
        ttl = max(0.0, self._new_candidate_ttl_s)
        if ttl > 0.0:
            for cid, cand in list(self._new_candidates.items()):
                if (now - float(cand.get("last_seen", now))) > ttl:
                    self._new_candidates.pop(cid, None)
        max_count = max(0, self._new_candidate_max_count)
        if max_count and len(self._new_candidates) > max_count:
            ordered = sorted(
                self._new_candidates.items(),
                key=lambda item: float(item[1].get("last_seen", 0.0)),
            )
            for cid, _ in ordered[: len(self._new_candidates) - max_count]:
                self._new_candidates.pop(cid, None)

    def _update_new_candidate(
        self,
        obj: Object,
        now: float,
        frame_candidate_claims: dict[str, tuple[str, int]],
    ) -> Object | None:
        if obj.center is None:
            return None
        self._prune_new_candidates(now)
        radius = self._jitter_radius()
        key = None
        claim_key = self._frame_claim_key(obj)
        for cid, cand in self._new_candidates.items():
            center = cand.get("center")
            cand_obj = cand.get("obj")
            if (
                cand.get("name") == obj.name
                and center is not None
                and obj.center.distance(center) <= radius
            ):
                if cand_obj is not None and self._observation_identity_mismatch(obj, cand_obj):
                    continue
                claimed = frame_candidate_claims.get(cid)
                if (
                    self._cooccurrence_gate
                    and claimed is not None
                    and claimed != claim_key
                    and cand_obj is not None
                    and not self._same_frame_duplicate(obj, cand_obj)
                ):
                    continue
                key = cid
                break
        if key is None:
            key = obj.object_id
            self._new_candidates[key] = {
                "object_id": key,
                "name": obj.name,
                "center": obj.center,
                "support": 1,
                "first_seen": now,
                "last_seen": now,
                "last_support_ts": now,
                "obj": obj,
            }
            frame_candidate_claims[key] = claim_key
            return None
        cand = self._new_candidates[key]
        same_frame_support = float(cand.get("last_support_ts", float("nan"))) == now
        if not same_frame_support:
            cand["support"] += 1
            cand["last_support_ts"] = now
        cand["center"] = obj.center
        cand["last_seen"] = now
        cand["obj"] = obj
        frame_candidate_claims[key] = claim_key
        self._prune_new_candidates(now)
        if (
            cand["support"] < self._min_support
            or (now - cand["first_seen"]) < self._new_candidate_min_age_s
        ):
            return None
        promoted = cand["obj"]
        promoted.object_id = cand["object_id"]
        self._new_candidates.pop(key, None)
        return self._insert(promoted, now)

    def _recent_position_jitter_candidate(
        self,
        obj: Object,
        frame_claims: dict[str, tuple[str, int]],
    ) -> str | None:
        """Recover a recent, unclaimed same-label object after small camera jitter."""
        if obj.center is None:
            return None
        if len(self._entities) < 2 and self._tracker_key(obj) is not None:
            return None
        radius = self._jitter_radius()
        recency_s = max(self._track_id_ttl_s, self._recent_window)
        cands: list[tuple[float, str]] = []
        for eid, e in self._entities.items():
            if e.center is None:
                continue
            claimed = frame_claims.get(eid)
            claim_key = self._frame_claim_key(obj)
            if (
                self._cooccurrence_gate
                and claimed is not None
                and claimed != claim_key
                and not self._same_frame_duplicate(obj, e)
            ):
                continue
            m = self._meta[eid]
            if (self._now - m["last_seen"]) > recency_s:
                continue
            if self._label_gate and m["labels"].most_common(1)[0][0] != obj.name:
                continue
            if self._appearance_mismatch(obj, eid):
                continue
            d = obj.center.distance(e.center)
            if d <= radius:
                cands.append((d, eid))
        if not cands:
            return None
        cands.sort(key=lambda x: x[0])
        if len(cands) > 1 and (cands[1][0] - cands[0][0]) < self._position_jitter_margin:
            self._record_ambiguous_association(
                obj,
                cands[0][1],
                reason="jitter_ambiguous",
                distance=cands[0][0],
                margin=cands[1][0] - cands[0][0],
                hard_appearance=False,
            )
            return None
        return cands[0][1]

    def _associate(
        self,
        obj: Object,
        frame_claims: dict[str, tuple[str, int]],
        *,
        position_assignment: AssociationDecision | None = None,
    ) -> tuple[str | None, str | None]:
        """Sticky, support-prioritised association. Returns (object_id|None, reason)."""
        # priority 1: detector/tracker track_id (generous gate rejects stale-id reuse far away)
        track_key = self._tracker_key(obj)
        if track_key is not None:
            eid = self._track_id_map.get(track_key)
            if eid is not None and eid in self._entities:
                # Do not let track-id continuity collapse two same-frame detections onto one entity.
                claimed = frame_claims.get(eid)
                claim_key = self._frame_claim_key(obj)
                e = self._entities[eid]
                blocked = (
                    self._cooccurrence_gate
                    and claimed is not None
                    and claimed != claim_key
                    and not self._same_frame_duplicate(obj, e)
                )
                meta = self._meta[eid]
                label_blocked = self._label_gate and meta["labels"].most_common(1)[0][0] != obj.name
                # Track ids are continuity evidence; visual mismatch can still veto them.
                visual_similarity = self._appearance_similarity_for_role(obj, eid, "visual")
                visual_blocked = (
                    visual_similarity is not None and visual_similarity < self._reacq_cos
                )
                semantic_blocked = self._is_departed(eid) and self._appearance_mismatch(obj, eid)
                identity_blocked = visual_blocked or semantic_blocked
                if identity_blocked:
                    block_dist = (
                        obj.center.distance(e.center)
                        if e.center is not None and obj.center is not None
                        else None
                    )
                    self._record_association_evidence(
                        self._association_evidence(
                            obj,
                            eid,
                            reason="track",
                            distance=block_dist,
                            track_match=True,
                            hard_appearance=True,
                            accepted=False,
                        )
                    )
                if (
                    not blocked
                    and not label_blocked
                    and not identity_blocked
                    and e.center is not None
                    and obj.center is not None
                    and (track_dist := obj.center.distance(e.center))
                    <= 2 * self._distance_threshold
                    and (self._now - meta["last_seen"]) <= self._track_id_ttl_s
                ):
                    # Let track and position evidence compete before accepting continuity.
                    track_evidence = self._association_evidence(
                        obj,
                        eid,
                        reason="track",
                        distance=track_dist,
                        track_match=True,
                        hard_appearance=self._is_departed(eid),
                    )
                    self._record_association_evidence(track_evidence)
                    identity_candidates: list[IdentityAssociationCandidate] = [
                        IdentityAssociationCandidate(
                            candidate_id=eid,
                            distance=track_dist,
                            support=int(meta["support"]),
                            evidence=track_evidence,
                            departed=self._is_departed(eid),
                        )
                    ]
                    cands = self._position_candidates(obj, frame_claims)
                    identity_candidates.extend(
                        IdentityAssociationCandidate(
                            candidate_id=pos_eid,
                            distance=pos_dist,
                            support=int(self._meta[pos_eid]["support"]),
                            evidence=pos_evidence,
                            departed=self._is_departed(pos_eid),
                        )
                        for pos_dist, pos_eid, _, pos_evidence in cands
                        if pos_eid != eid
                    )
                    decision = self._identity_policy.choose_observation_candidate(
                        identity_candidates,
                        sticky_support=self._sticky_support,
                    )
                    if decision.accepted and decision.candidate_id is not None:
                        self._record_accepted_decision(decision)
                        return decision.candidate_id, (
                            "track" if decision.reason == "track" else "distance"
                        )
                if not blocked and not identity_blocked:
                    # Stale or teleported track id: fall through to geometry.
                    del self._track_id_map[track_key]

        if obj.center is None:
            return None, None

        if (
            position_assignment is not None
            and position_assignment.accepted
            and position_assignment.candidate_id is not None
            and position_assignment.candidate_id in self._entities
        ):
            assigned_eid = position_assignment.candidate_id
            assigned_entity = self._entities[assigned_eid]
            claimed = frame_claims.get(assigned_eid)
            claim_key = self._frame_claim_key(obj)
            blocked = (
                self._cooccurrence_gate
                and claimed is not None
                and claimed != claim_key
                and not self._same_frame_duplicate(obj, assigned_entity)
            )
            if not blocked and not self._appearance_mismatch(obj, assigned_eid):
                assigned_dist = (
                    obj.center.distance(assigned_entity.center)
                    if assigned_entity.center is not None
                    else None
                )
                self._record_accepted_association(
                    obj, assigned_eid, reason="distance", distance=assigned_dist
                )
                return assigned_eid, "distance"

        cands = self._position_candidates(obj, frame_claims)
        if not cands:
            jitter = self._recent_position_jitter_candidate(obj, frame_claims)
            if jitter is not None:
                je = self._entities[jitter]
                jd = obj.center.distance(je.center) if je.center is not None else None
                self._record_accepted_association(obj, jitter, reason="jitter", distance=jd)
                return jitter, "distance"
            if self._reid_reacquire:
                reid = self._reacquire(obj, frame_claims)
                if reid is not None:
                    re = self._entities[reid]
                    rd = obj.center.distance(re.center) if re.center is not None else None
                    self._record_accepted_association(obj, reid, reason="reid", distance=rd)
                    return reid, "reid"
            return None, None

        pos_decision = self._choose_position_candidate(obj, cands)
        pos_eid = pos_decision.candidate_id
        pos_dist = min(d for d, cid, _, _ in cands if cid == pos_eid)
        if pos_decision.ambiguous:
            self._record_ambiguous_association(
                obj,
                pos_eid,
                reason=pos_decision.reason,
                distance=pos_dist,
                margin=pos_decision.margin,
            )
            return None, None

        if self._reacq_override and self._reid_reacquire:
            bd, bc, sc, role = self._best_departed(obj, frame_claims)
            emb = (
                None
                if role is None
                else (self._visual_emb_of(obj) if role == "visual" else self._semantic_emb_of(obj))
            )
            if (
                emb is not None
                and role is not None
                and bd is not None
                and bd != pos_eid
                and bc >= self._reacq_cos
                and (bc - sc) >= self._reacq_margin
            ):
                pg = self._role_gallery(self._meta[pos_eid], role)
                pos_cos = gallery_cos(emb, pg)
                if pos_cos is None or bc >= pos_cos + self._reacq_margin:
                    be = self._entities[bd]
                    bd_dist = obj.center.distance(be.center) if be.center is not None else None
                    self._record_accepted_association(obj, bd, reason="reid", distance=bd_dist)
                    return bd, "reid"
        self._record_accepted_association(obj, pos_eid, reason="distance", distance=pos_dist)
        return pos_eid, "distance"

    def _insert(self, obj: Object, now: float) -> Object:
        if not obj.ts:
            obj.ts = now
        self._entities[obj.object_id] = obj
        c = obj.center
        semantic0 = self._semantic_emb_of(obj)
        visual0 = self._visual_emb_of(obj)
        self._meta[obj.object_id] = {
            "entered_t": now,
            "last_seen": now,
            "support": 1,
            "window": [now],
            "labels": Counter([obj.name]),
            "semantic_emb": semantic0,
            "semantic_gallery": [semantic0] if semantic0 is not None else [],
            "semantic_embedding_model": getattr(obj, "embedding_model", None),
            "semantic_embedding_device": getattr(obj, "embedding_device", None),
            "semantic_embedding_dim": getattr(
                obj, "embedding_dim", int(semantic0.size) if semantic0 is not None else None
            ),
            "visual_emb": visual0,
            "visual_gallery": [visual0] if visual0 is not None else [],
            "visual_embedding_model": getattr(obj, "visual_embedding_model", None),
            "visual_embedding_device": getattr(obj, "visual_embedding_device", None),
            "visual_embedding_dim": getattr(
                obj, "visual_embedding_dim", int(visual0.size) if visual0 is not None else None
            ),
            "positions": [[c.x, c.y, c.z]] if c is not None else [],
        }
        track_key = self._tracker_key(obj)
        if track_key is not None:
            self._track_id_map[track_key] = obj.object_id
        self._history.append_object(obj, now, now)
        return obj

    def _update(self, eid: str, obj: Object, now: float) -> None:
        existing = self._entities[eid]
        m = self._meta[eid]
        partial = getattr(obj, "observation_partial", False)
        stable_center = (
            self._median_center(m["positions"]) if partial and m["positions"] else existing.center
        )
        stable_pose = existing.pose if partial else None
        stable_size = existing.size if partial else None
        stable_pointcloud = existing.pointcloud if partial else None

        existing.update_object(obj)  # latest frame metadata + detection count
        existing.ts = obj.ts or now
        # Partial observations refresh identity without moving stable geometry.
        if partial:
            if stable_center is not None:
                existing.center = stable_center
            if stable_pose is not None:
                existing.pose = stable_pose
                if stable_center is not None:
                    existing.pose.position = stable_center
            if stable_size is not None:
                existing.size = stable_size
            if stable_pointcloud is not None:
                existing.pointcloud = stable_pointcloud
        elif existing.center is not None:
            m["positions"].append([existing.center.x, existing.center.y, existing.center.z])
            if len(m["positions"]) > self._anchor_window:
                m["positions"] = m["positions"][-self._anchor_window :]
            existing.center = self._median_center(m["positions"])
        m["support"] += 1
        m["last_seen"] = now
        if not partial:
            m["labels"][obj.name] += 1
        # Keep the reported label stable across detector wording jitter.
        existing.name = m["labels"].most_common(1)[0][0]
        semantic_emb = self._semantic_emb_of(obj)
        visual_emb = self._visual_emb_of(obj)
        if not partial:
            if semantic_emb is not None:
                obj_model = getattr(obj, "embedding_model", None)
                meta_model = m.get("semantic_embedding_model")
                if obj_model is not None and meta_model is not None and obj_model != meta_model:
                    m["semantic_gallery"] = []
                m["semantic_emb"] = semantic_emb
                m["semantic_embedding_model"] = getattr(obj, "embedding_model", None)
                m["semantic_embedding_device"] = getattr(obj, "embedding_device", None)
                m["semantic_embedding_dim"] = getattr(obj, "embedding_dim", int(semantic_emb.size))
                add_diverse_embedding_view(
                    m.setdefault("semantic_gallery", []),
                    semantic_emb,
                    novelty=self._gallery_novelty,
                    max_size=self._gallery_size,
                )
            if visual_emb is not None:
                obj_model = getattr(obj, "visual_embedding_model", None)
                meta_model = m.get("visual_embedding_model")
                if obj_model is not None and meta_model is not None and obj_model != meta_model:
                    m["visual_gallery"] = []
                m["visual_emb"] = visual_emb
                m["visual_embedding_model"] = getattr(obj, "visual_embedding_model", None)
                m["visual_embedding_device"] = getattr(obj, "visual_embedding_device", None)
                m["visual_embedding_dim"] = getattr(
                    obj, "visual_embedding_dim", int(visual_emb.size)
                )
                add_diverse_embedding_view(
                    m.setdefault("visual_gallery", []),
                    visual_emb,
                    novelty=self._gallery_novelty,
                    max_size=self._gallery_size,
                )
        m["window"].append(now)
        cut = now - self._recent_window
        if len(m["window"]) > 16 and m["window"][0] < cut:
            m["window"] = [t for t in m["window"] if t >= cut]
        track_key = self._tracker_key(obj)
        if track_key is not None:
            self._track_id_map[track_key] = eid
        self._history.append_object(
            existing,
            m["entered_t"],
            now,
            identity_obj=obj,
            include_identity=not partial,
        )

    def _evict_stale(self, now: float, evict_exempt: set[str] | None = None) -> None:
        exempt = evict_exempt or ()
        dead = [
            eid
            for eid, m in self._meta.items()
            if (now - m["last_seen"]) > self._eviction_ttl_s and eid not in exempt
        ]
        for eid in dead:
            self._entities.pop(eid, None)
            self._meta.pop(eid, None)
            self._promoted.discard(eid)
            for tid, mapped in list(self._track_id_map.items()):
                if mapped == eid:
                    del self._track_id_map[tid]


    def when_entered(self, object_id: str) -> float | None:
        """First-seen sim time of an object from live state or durable evidence history."""
        with self._lock:
            m = self._meta.get(object_id)
            if m is not None:
                return m["entered_t"]
            entered = self._history.when_entered(object_id)
            return entered

    def rehydrate(self) -> None:
        """Seed the online maintained table from durable Memory2 identity evidence."""
        if not self._history.enabled:
            return
        with self._lock:
            state = self._history.rehydrate()
            self._entities.clear()
            self._entities.update(state.entities)
            self._meta.clear()
            self._meta.update(state.meta)
            self._track_id_map.clear()
            self._promoted.clear()
            self._new_candidates.clear()
            self._now = state.now
            logger.info(
                f"WorldBelief rehydrated {len(self._entities)} entities from {self._history.path}"
            )

    def close(self) -> None:
        """Release the delegated history store (call on shutdown)."""
        self._history.close()

    def __len__(self) -> int:
        with self._lock:
            return len(self.get_objects())

    def __repr__(self) -> str:
        with self._lock:
            return (
                f"WorldBelief(present={len(self.get_objects())}, maintained={len(self._entities)})"
            )

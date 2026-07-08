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
import math
from typing import TYPE_CHECKING, Any
import uuid

import numpy as np

from dimos.models.embedding.base import Embedding
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.absence import ABSENT, PRESENT, classify_visibility
from dimos.perception.detection.identity_features import (
    add_diverse_embedding_view,
    gallery_cos,
    normalize_embedding,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.perception.detection.type.detection3d.object import Object

logger = setup_logger()

TENTATIVE = "tentative"  # seen enough to report, but NOT verified from multiple viewpoints
CONFIRMED = "confirmed"  # seen from >= min_viewpoints distinct camera vantages — safe to act on

# How the entity's identity was last established
BASIS_NEW = "new"
BASIS_TRACKED = "tracked"
BASIS_REACQUIRED = "reacquired"
BASIS_TWIN_ANCHOR = "twin-anchor"
BASIS_RESTORED = "restored"  # past-session identity restored from the gallery store

Viewpoint = tuple[Vector3, Vector3]  # (camera position, forward direction)


class WorldBeliefConfig(BaseConfig):
    """Tuning constants for the identity core; each consuming rule's docstring gives the
    rationale."""

    radius_base: float = 0.18
    radius_growth: float = 0.15  # match radius growth per second unobserved
    radius_cap: float = 0.40
    mint_guard: float = 0.13  # noise envelope: max static-lift jump
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
    candidate_ttl_s: float = 5.0  # never-established junk is dropped after this long unseen
    max_absent: int = 128  # geometric death never times out, so cap retained corpses
    position_window: int = 9  # running-median window for the entity center
    frame_id: str = "world"  # frame the fold (and every track position) lives in
    history_path: str | None = None  # cross-session gallery store; None = RAM only


def camera_viewpoint(pose: Pose) -> Viewpoint:
    """Camera vantage (world position, forward) from the FK/tf pose — never the noisy
    object center, so a static camera can't jitter into a second viewpoint."""
    return pose.position, pose.orientation.rotate_vector(Vector3.unit_z())


def is_novel_viewpoint(
    viewpoint: Viewpoint,
    existing: list[Viewpoint],
    *,
    min_baseline_m: float,
    angle_deg: float,
) -> bool:
    """True unless some stored vantage is close in BOTH position and viewing angle."""
    pos, fwd = viewpoint
    cos_thresh = math.cos(math.radians(angle_deg))
    for prev_pos, prev_fwd in existing:
        near_in_space = pos.distance(prev_pos) < min_baseline_m
        near_in_angle = fwd.dot(prev_fwd) > cos_thresh
        if near_in_space and near_in_angle:
            return False
    return True


def _dist(a: Vector3 | PointStamped, b: Vector3 | PointStamped) -> float:
    """Euclidean distance across the position types (bare ``Point`` has no helpers)."""
    return math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))


def _stamp(center: Vector3, ts: float, frame_id: str) -> PointStamped:
    p = PointStamped(center.x, center.y, center.z, ts=ts, frame_id=frame_id)
    p.ts = ts  # the ctor swaps ts=0.0 for wall-clock; fold time is authoritative
    return p


def _appearance_vec(obj: Object) -> NDArray[np.float32] | None:
    """The object's identity embedding: DINO visual preferred (the gates are calibrated
    for it), CLIP semantic as fallback when no visual embedder is configured."""
    for attr in ("visual_embedding", "embedding"):
        vec = normalize_embedding(getattr(obj, attr, None))
        if vec is not None:
            return vec
    return None


@dataclass(slots=True)
class _Track:
    """One believed entity: the folded Object plus its identity/lifecycle state."""

    # ── physical state: where/what the entity is ──────────────────────────────────
    obj: Object
    # Association anchors on the LAST raw position (follows a carried object); the
    # REPORTED center is the running median. The stamp doubles as last-seen time.
    last_pos: PointStamped
    positions: list[Vector3]
    sizes: list[float]
    # ── identity evidence: how the entity is recognized / deduplicated ────────────
    label_counts: Counter[str]
    gallery: list[NDArray[np.float32]] = field(default_factory=list)
    latest_emb: NDArray[np.float32] | None = None
    viewpoints: list[Viewpoint] = field(default_factory=list)
    basis: str = BASIS_NEW
    # ── lifecycle ──────────────────────────────────────────────────────────────────
    first_ts: float = 0.0
    n_frames: int = 1
    established: bool = False
    state: str = "active"  # "active" | "absent"
    absent_votes: int = 0

    @classmethod
    def from_observation(
        cls, obj: Object, ts: float, viewpoint: Viewpoint | None, basis: str, frame_id: str
    ) -> _Track:
        try:
            sizes = [float(max(obj.size.x, obj.size.y, obj.size.z))]
        except Exception:
            sizes = []
        return cls(
            obj=obj,
            last_pos=_stamp(obj.center, ts, frame_id),
            positions=[obj.center],
            sizes=sizes,
            label_counts=Counter([obj.name]),
            viewpoints=[viewpoint] if viewpoint is not None else [],
            basis=basis,
            first_ts=ts,
        )

    @property
    def last_seen(self) -> float:
        return self.last_pos.ts

    @property
    def modal_label(self) -> str:
        return self.label_counts.most_common(1)[0][0]

    def full_gallery(self) -> list[NDArray[np.float32]]:
        """Diverse views plus the latest — the set the re-acquire gate scores against
        (the latest view catches a fresh face the novelty gate hasn't kept yet)."""
        return list(self.gallery) if self.latest_emb is None else [*self.gallery, self.latest_emb]

    def ingest_appearance(self, obj: Object, novelty: float, max_size: int) -> None:
        vec = _appearance_vec(obj)
        if vec is None:
            return
        if self.gallery and vec.size != self.gallery[0].size:
            # A different-role embedding (e.g. a CLIP fallback frame when DINO failed
            # once) must not pollute a gallery whose gates are calibrated for one model.
            return
        # Low-confidence association views are ingested too — deliberately: mid-carry /
        # mid-placement crops are exactly the views a later re-acquire must match.
        add_diverse_embedding_view(self.gallery, vec, novelty=novelty, max_size=max_size)
        self.latest_emb = vec


class _GalleryStore:
    """Cross-session identity persistence: appearance views in memory2 vec0 streams
    (novelty-gated mirror of the RAM galleries; ``lookup`` group-max == ``gallery_cos``).
    Reopening the same path in a later process restores re-acquire candidates."""

    def __init__(self, path: str, *, novelty: float) -> None:
        from dimos.memory2.store.sqlite import SqliteStore

        self._store = SqliteStore(path=path)
        self._novelty = novelty

    def _stream_for(self, obj: Object, vec: NDArray[np.float32]) -> Any:
        role = "dino" if normalize_embedding(getattr(obj, "visual_embedding", None)) is not None else "clip"
        # dim in the name → a model swap can't collide (Store.stream caches by name)
        return self._store.stream(f"object_gallery_{role}_{vec.size}", Image)

    def remember(
        self, eid: str, obj: Object, position: NDArray[np.float64], n_frames: int
    ) -> None:
        """Best-effort: a history-db hiccup must never abort a fold (identity continues
        in RAM; only cross-session persistence degrades)."""
        try:
            vec = _appearance_vec(obj)
            if vec is None:
                return
            stream = self._stream_for(obj, vec)
            top = stream.search(Embedding(vec), k=1).to_list()
            if top and (top[0].similarity or 0.0) >= self._novelty and top[0].tags.get("object_id") == eid:
                return  # already have a near-identical view of THIS object
            stream.append(
                self._crop(obj),
                tags={
                    "object_id": eid,
                    "name": obj.name,
                    # the entity's CURRENT running-median center, never a raw sample
                    # (raw samples can be depth-mode flipped)
                    "pos": " ".join(f"{c:.4f}" for c in position),
                    "n": str(int(n_frames)),  # sighting support (restore arbitration)
                },
                embedding=Embedding(vec),
            )
        except Exception as e:
            logger.warning("gallery remember failed (identity continues in RAM): %s", e)

    def lookup(self, obj: Object) -> dict[str, tuple[float, str, NDArray[np.float64] | None, int]]:
        """``{object_id: (best_cos, label, stored_position | None, support)}`` over the
        persisted views. Score = max cosine per id; metadata from the newest view
        (max-cos hands back stale early-view tags; ``n`` resets to 1 on restore)."""
        try:
            vec = _appearance_vec(obj)
            if vec is None:
                return {}
            best_cos: dict[str, float] = {}
            newest: dict[str, tuple[float, str, NDArray[np.float64] | None, int]] = {}
            for hit in self._stream_for(obj, vec).search(Embedding(vec), k=64).to_list():
                eid = hit.tags.get("object_id")
                if eid is None:
                    continue
                cos = float(hit.similarity or 0.0)
                best_cos[eid] = max(best_cos.get(eid, -1.0), cos)
                pos_tag = hit.tags.get("pos")
                try:
                    pos = np.array(pos_tag.split(), dtype=np.float64) if pos_tag else None
                except ValueError:
                    pos = None
                if pos is not None and pos.size != 3:
                    pos = None
                try:
                    support = int(hit.tags.get("n") or 1)
                except ValueError:
                    support = 1
                appended = float(hit.ts or 0.0)
                if eid not in newest or appended >= newest[eid][0]:
                    newest[eid] = (appended, hit.tags.get("name") or "", pos, support)
            return {
                eid: (best_cos[eid], newest[eid][1], newest[eid][2], newest[eid][3])
                for eid in best_cos
            }
        except Exception as e:
            logger.warning("gallery lookup failed (no cross-session candidates): %s", e)
            return {}

    @staticmethod
    def _crop(obj: Object) -> Image:
        try:
            crop = obj.cropped_image(padding=0)
            if crop.width >= 1 and crop.height >= 1:
                return crop
        except Exception:
            pass
        return obj.image

    def close(self) -> None:
        # Views are already durable (WAL auto-commit per append); dispose releases the
        # connections so a later model can reopen the same path.
        self._store.dispose()


class WorldBelief:
    """Identity core: folds lifted detections into stable identities via four rules —
    match (:meth:`_match_frame`), envelope (:meth:`_envelope_neighbours`), re-acquire
    (:meth:`_reacquire_candidate`), mint (:meth:`_insert`) — plus geometric absence."""

    def __init__(self, config: WorldBeliefConfig | None = None) -> None:
        self._cfg = config or WorldBeliefConfig()
        self._tracks: dict[str, _Track] = {}
        # Fold watermark: strictly-forward time is the whole idempotency story — a frame
        # at or before the newest folded ts is skipped entirely.
        self._now: float = float("-inf")
        # Appearance persistence over memory2 vec0 (cross-session by reopening the same
        # path). None → within-session identity only (RAM galleries still work).
        self._gallery = (
            _GalleryStore(self._cfg.history_path, novelty=self._cfg.gallery_novelty)
            if self._cfg.history_path is not None else None
        )

    def _radius(self, dt: float) -> float:
        return min(self._cfg.radius_base + self._cfg.radius_growth * max(dt, 0.0), self._cfg.radius_cap)

    @property
    def last_fold_ts(self) -> float:
        """Newest folded frame ts (0.0 before any fold) — the scanner's catch-up start."""
        return max(self._now, 0.0)

    # ── ingest ────────────────────────────────────────────────────────────────────

    def observe(
        self,
        objects: list[Object],
        *,
        frame_ts: float,
        camera_transform: Transform | None,
        camera_info: CameraInfo,
        depth_m: NDArray[np.float32],
    ) -> None:
        """Fold one frame of lifted + embedded detections into the model. Time only
        moves forward: frames at or before the newest folded ts are skipped entirely
        (a stale pre-removal frame must not resurrect a killed object)."""
        ts = float(frame_ts)
        if ts <= self._now:
            return  # duplicate or out-of-order frame → no fold, no support, no absence vote
        self._now = ts
        # No camera pose → no geometric evidence: identity/support still folds, but no
        # viewpoint credit and no absence voting through a fictitious extrinsic.
        viewpoint = (
            camera_viewpoint(camera_transform.to_pose()) if camera_transform is not None else None
        )
        for obj in objects:
            # Point-median center BEFORE association, so matching/folding/reporting share
            # one estimate (the OBB midpoint swings ~135 mm where the median stays <5 mm).
            obj.center = self._observation_center(obj)

        matched, unmatched, seen = self._match_frame(objects, ts)
        for track, obj in matched:
            self._hit(track, obj, ts, viewpoint, BASIS_TRACKED)
        for obj in unmatched:
            track = self._acquire(obj, ts, viewpoint, seen)
            if track is not None:
                seen.add(id(track))  # a re-acquired/minted track was observed this frame
        if camera_transform is not None:
            self._vote_absence(seen, camera_info, camera_transform, depth_m)
        self._prune(ts)

    def _match_frame(
        self, objects: list[Object], ts: float
    ) -> tuple[list[tuple[_Track, Object]], list[Object], set[int]]:
        """Greedy nearest-position association of one frame (module-header Match rule)."""
        active = [t for t in self._tracks.values() if t.state == "active"]
        pairs: list[tuple[float, _Track, Object]] = []
        for track in active:
            radius = self._radius(ts - track.last_seen)
            for obj in objects:
                dist = _dist(obj.center, track.last_pos)
                if dist > radius:
                    continue
                # A det in a neighbour's envelope never feeds a third party's dt-grown
                # radius (a det-starved track would camp on the neighbour's misfires).
                if dist > self._cfg.radius_base and any(
                    cos is None or cos >= self._cfg.reacq_cos or obj.name == other.modal_label
                    for cos, other in self._envelope_neighbours(obj, exclude=track)
                ):
                    continue
                pairs.append((dist, track, obj))
        pairs.sort(key=lambda p: p[0])
        used_tracks: set[int] = set()
        used_objs: set[int] = set()
        matched: list[tuple[_Track, Object]] = []
        for _d, track, obj in pairs:
            if id(track) in used_tracks or id(obj) in used_objs:
                continue
            cos = gallery_cos(_appearance_vec(obj), track.full_gallery())
            if obj.name != track.modal_label:
                # Label flip: only a strong appearance match overrides (same object across
                # a 180° rotation scores 0.94 median; cross-object max 0.334).
                if cos is None or cos < self._cfg.label_override_cos:
                    continue
            elif cos is not None and cos < self._cfg.reacq_cos:
                # Appearance-contradiction veto: label agrees but the crop is clearly
                # foreign — without it a det-starved track lives off a neighbour's
                # misfires and poisons its own gallery.
                continue
            used_tracks.add(id(track))
            used_objs.add(id(obj))
            matched.append((track, obj))
        return matched, [o for o in objects if id(o) not in used_objs], used_tracks

    def _envelope_neighbours(
        self, obj: Object, exclude: _Track | None = None
    ) -> Iterator[tuple[float | None, _Track]]:
        """Yield ``(appearance_cos | None, track)`` per established active track whose
        noise envelope (mint_guard) contains this detection. The core arbitration
        primitive: the co-located object is the null hypothesis every rival reading
        (far-track claim, absent re-acquire, new mint) must beat."""
        vec = _appearance_vec(obj)
        for track in self._tracks.values():
            if track is exclude or track.state != "active" or not track.established:
                continue
            if _dist(obj.center, track.last_pos) > self._cfg.mint_guard:
                continue
            yield gallery_cos(vec, track.full_gallery()), track

    def _acquire(
        self, obj: Object, ts: float, viewpoint: Viewpoint | None, seen_tracks: set[int]
    ) -> _Track | None:
        """Re-acquire an absent/past-session identity for an unmatched detection, or mint
        a new one; returns the landing track (None = suppressed as a neighbour misfire)."""
        eid, basis, win_cos = self._reacquire_candidate(obj)
        if eid is not None and win_cos is not None and any(  # win_cos clause narrows for mypy only
            cos is not None and cos >= win_cos for cos, _t in self._envelope_neighbours(obj)
        ):
            # Relative evidence: re-acquire inside a neighbour's envelope only when the
            # det resembles the absent id MORE than the neighbour — a true return passes,
            # a borderline misfire can't steal an orphaned id.
            eid = None
        if eid is not None and basis is not None:  # second clause narrows for mypy only
            track = self._tracks.get(eid)
            if track is not None:
                self._hit(track, obj, ts, viewpoint, basis)
                # Announce identity survival live (basis flips back to 'tracked' on the
                # very next frame, so the log is where these events stay visible).
                logger.info(
                    "belief: %s (%s) %s at (%.2f, %.2f) cos=%.2f",
                    eid[:8], track.modal_label, basis, obj.center.x, obj.center.y,
                    win_cos,
                )
                return track
            self._insert(obj, ts, viewpoint, force_id=eid, basis=BASIS_RESTORED)
            logger.info(
                "belief: %s (%s) restored from a previous session at (%.2f, %.2f) cos=%.2f",
                eid[:8], obj.name, obj.center.x, obj.center.y,
                win_cos,
            )
            return self._tracks[eid]
        # Mint-guard: don't mint inside an established neighbour's envelope when the det
        # plausibly IS that neighbour (same label, or look not clearly foreign). A
        # co-detected neighbour proves a second object (a placed twin mints) — unless the
        # det is its different-label clone (a misfire under another prompt).
        for cos, track in self._envelope_neighbours(obj):
            if id(track) in seen_tracks:
                if obj.name != track.modal_label and cos is not None and cos >= self._cfg.reacq_cos:
                    logger.debug("belief: misfire suppressed near %s (%s)", track.obj.object_id[:8], obj.name)
                    return None
                continue
            if cos is None or cos >= self._cfg.reacq_cos or obj.name == track.modal_label:
                logger.debug("belief: mint suppressed near %s (%s)", track.obj.object_id[:8], obj.name)
                return None
        self._insert(obj, ts, viewpoint, basis=BASIS_NEW)
        return self._tracks[obj.object_id]

    def _reacquire_candidate(self, obj: Object) -> tuple[str | None, str | None, float | None]:
        """``(entity_id, basis, winning_cos)`` for the absent/past-session identity this
        detection re-acquires, else ``(None, None, None)``. Gate: cos ≥ reacq_cos with
        margin ≥ reacq_margin (true re-acquires land ~0.66-0.78; 0.8 refuses them all).
        Same-label near-ties are twins (inter-twin cos up to 0.905) — position decides;
        different-label near-ties are refused."""
        vec = _appearance_vec(obj)
        if vec is None:
            return None, None, None
        # (cos, modal label, position, support) per candidate: RAM galleries for this
        # session's absent tracks, the vec0 store for past-session identities.
        scored: dict[str, tuple[float, str, NDArray[np.float64] | None, int]] = {}
        for eid, track in self._tracks.items():
            if track.state != "absent":
                continue
            cos = gallery_cos(vec, track.full_gallery())
            if cos is not None:
                scored[eid] = (cos, track.modal_label, track.obj.center.to_numpy(), track.n_frames)
        if self._gallery is not None:
            for eid, (cos, label, pos, support) in self._gallery.lookup(obj).items():
                track = self._tracks.get(eid)
                if track is not None and track.state == "active":
                    continue  # active in RAM → position association owns it
                prev = scored.get(eid)
                if prev is None:
                    scored[eid] = (cos, label, pos, support)
                elif cos > prev[0]:
                    # cos takes the max; RAM metadata (fresher position/label/support) kept
                    scored[eid] = (cos, prev[1], prev[2], prev[3])
        if not scored:
            return None, None, None
        ranked = sorted(
            ((c, eid, label, pos, n) for eid, (c, label, pos, n) in scored.items()),
            key=lambda r: -r[0],
        )
        best_cos, best_eid, best_label, best_pos, best_n = ranked[0]
        if best_cos < self._cfg.reacq_cos:
            return None, None, None
        # Same acceptance rule as the matcher: label agreement OR appearance >= 0.8
        # (blocks cross-label id theft between same-shape objects).
        if obj.name != best_label and best_cos < self._cfg.label_override_cos:
            logger.debug(
                "belief: re-acquire refused — label %r vs candidate %r at cos %.2f (< %.2f override)",
                obj.name, best_label, best_cos, self._cfg.label_override_cos,
            )
            return None, None, None
        if len(ranked) == 1 or best_cos - ranked[1][0] >= self._cfg.reacq_margin:
            return best_eid, BASIS_REACQUIRED, best_cos
        second_cos, second_eid, second_label, second_pos, second_n = ranked[1]
        if second_label == best_label and second_cos >= self._cfg.reacq_cos:
            # Same-label near-tie: real twins carry comparable support and position
            # decides; a ≥3× support skew means the lightweight is a parasite alias
            # (split-detection shadow) — the accumulated identity wins regardless.
            if best_n >= 3 * second_n or second_n >= 3 * best_n:
                return (best_eid if best_n >= second_n else second_eid), BASIS_TWIN_ANCHOR, best_cos
            # Twins: position decides. Requires stored positions for both candidates.
            center = obj.center.to_numpy()
            dists = [
                (float(np.linalg.norm(center - pos)), eid)
                for pos, eid in ((best_pos, best_eid), (second_pos, second_eid))
                if pos is not None
            ]
            if dists:
                dist, eid = min(dists)
                if dist <= self._cfg.radius_cap:
                    return eid, BASIS_TWIN_ANCHOR, best_cos
        logger.debug(
            "belief: re-acquire refused (%s: %.3f vs %s: %.3f)",
            best_eid[:8], best_cos, second_eid[:8], second_cos,
        )
        return None, None, None

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
        while force_id is None and eid in self._tracks:  # guard the (rare) id collision
            eid = uuid.uuid4().hex[:8]
        obj.object_id = eid  # the entity IS this Object; later folds accumulate into it
        track = _Track.from_observation(obj, ts, viewpoint, basis, self._cfg.frame_id)
        track.established = self._is_established(track, ts)  # true only for 1-frame/0-span configs
        track.ingest_appearance(obj, self._cfg.gallery_novelty, self._cfg.gallery_max)
        self._tracks[eid] = track
        if self._gallery is not None and track.established:
            self._gallery.remember(eid, obj, obj.center.to_numpy(), track.n_frames)

    def _is_established(self, track: _Track, ts: float) -> bool:
        return track.n_frames >= self._cfg.min_frames and (ts - track.first_ts) >= self._cfg.min_span_s

    def _hit(self, track: _Track, obj: Object, ts: float, viewpoint: Viewpoint | None, basis: str) -> None:
        was_absent = track.state == "absent"
        track.state = "active"
        track.absent_votes = 0
        track.n_frames += 1
        track.label_counts[obj.name] += 1
        track.basis = basis
        if not track.established:
            track.established = self._is_established(track, ts)
        track.ingest_appearance(obj, self._cfg.gallery_novelty, self._cfg.gallery_max)
        new_pos = obj.center
        # The anchor follows EVERY accepted observation, even geometry-suspect ones —
        # a frozen anchor with a reset dt forks the identity on the next clean detection.
        track.last_pos = _stamp(new_pos, max(track.last_seen, ts), self._cfg.frame_id)
        suspect = self._geometry_suspect(track, obj)
        if suspect and was_absent:
            # Partial-view re-acquire: too poor to fold, but an imprecise position beats
            # reporting the old spot absence voting already proved empty.
            track.positions = [new_pos]
            track.obj.center = Vector3(new_pos)
            self._reset_cloud(track.obj, obj)
        if not suspect:
            entity = track.obj
            entity.update_object(obj)  # latest size/pose bookkeeping
            # Cloud = freshest observed surface, not fused multi-view accumulation (fusion
            # bought only a movement-trail artifact; consumers can re-fold it from the recording).
            self._reset_cloud(entity, obj)
            try:
                track.sizes.append(float(max(obj.size.x, obj.size.y, obj.size.z)))
                del track.sizes[:-self._cfg.position_window]
            except Exception:
                pass
            positions = track.positions
            if was_absent and positions[-1].distance(new_pos) > self._cfg.radius_base:
                # Re-acquired at a NEW spot: stale pre-departure positions must not
                # outvote the return (would report a ghost at the old empty spot).
                positions.clear()
            positions.append(new_pos)
            del positions[:-self._cfg.position_window]
            # Genuine move: the last 3 observations agree with each other but not the
            # median — reset the window so old positions can't outvote the new spot.
            if len(positions) >= 3:
                last3 = positions[-3:]
                med = Vector3(np.median(np.asarray([p.to_numpy() for p in positions]), axis=0))
                coherent = all(p.distance(last3[-1]) < 0.5 * self._cfg.radius_base for p in last3)
                moved = all(p.distance(med) > self._cfg.radius_base for p in last3)
                if coherent and moved:
                    track.positions = positions = list(last3)
            entity.center = Vector3(np.median(np.asarray([p.to_numpy() for p in positions]), axis=0))
        # Identity/support still counts on geometry-suspect frames (partial view, mask
        # bleed): the object was seen; only this frame's geometry is untrusted.
        if viewpoint is not None and is_novel_viewpoint(
            viewpoint, track.viewpoints,
            min_baseline_m=self._cfg.min_baseline_m, angle_deg=self._cfg.viewpoint_angle_deg,
        ):
            track.viewpoints.append(viewpoint)  # genuinely new vantage → trust credit
        # Only ESTABLISHED identities persist: junk candidates must never be restorable
        # next session (pre-establishment views stay in the RAM gallery).
        if self._gallery is not None and track.established:
            self._gallery.remember(track.obj.object_id, obj, track.obj.center.to_numpy(), track.n_frames)

    @staticmethod
    def _reset_cloud(entity: Object, obj: Object) -> None:
        """Set the entity's cloud to the latest observation's surface (the entity always
        shows where it IS, never a history of where it has been)."""
        try:
            if obj.pointcloud is not None:
                entity.pointcloud = obj.pointcloud
        except Exception:
            pass

    @staticmethod
    def _observation_center(obj: Object) -> Vector3:
        """Robust per-observation center: the component-wise median of the object's
        (already filtered, world-frame) surface cloud; falls back to ``obj.center``."""
        try:
            pts = np.asarray(obj.pointcloud.pointcloud.points)  # o3d legacy geometry points
        except Exception:
            return obj.center
        if pts.ndim != 2 or len(pts) < 8:
            return obj.center
        return Vector3(np.median(pts, axis=0))

    def _geometry_suspect(self, track: _Track, obj: Object) -> bool:
        """True when this observation's geometry must not fold (identity still counts):
        partial view (border bbox), or size far above the running median (mask bleed —
        median reference, so escalating bleed can't ratchet the gate open)."""
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
            # PRESENT band widened toward the camera by the object's own half-extent: a
            # deep object's near surface must reset votes, not read as an occluder.
            half_extent = 0.0
            try:
                half_extent = min(0.3, 0.5 * max(track.obj.size.x, track.obj.size.y, track.obj.size.z))
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
                        track.obj.object_id[:8], track.modal_label, track.absent_votes,
                    )
            elif verdict == PRESENT:
                track.absent_votes = 0  # a surface is there; the detector just missed it
            # OCCLUDED / OUT_OF_VIEW → no evidence, leave the count unchanged

    def _prune(self, ts: float) -> None:
        """Junk hygiene (not lifecycle): drop never-established candidates unseen for
        ``candidate_ttl_s``, and cap retained absent corpses at ``max_absent``."""
        stale = [
            eid for eid, t in self._tracks.items()
            if not t.established and (ts - t.last_seen) > self._cfg.candidate_ttl_s
        ]
        for eid in stale:
            del self._tracks[eid]
        absent = [eid for eid, t in self._tracks.items() if t.state == "absent"]
        if len(absent) > self._cfg.max_absent:
            for eid in sorted(absent, key=lambda e: self._tracks[e].last_seen)[: len(absent) - self._cfg.max_absent]:
                del self._tracks[eid]

    # ── queries ───────────────────────────────────────────────────────────────────

    def present(self, *, min_viewpoints: int | None = None) -> list[Object]:
        """Believed-present objects (established, not geometrically absent); returns the
        live internal entities — treat as read-only. Trust via :meth:`trust_of`, identity
        basis via :meth:`basis_of`."""
        return [
            track.obj for track in self._tracks.values()
            if track.state == "active" and track.established
            and (min_viewpoints is None or len(track.viewpoints) >= min_viewpoints)
        ]

    def trust_of(self, object_id: str) -> str | None:
        """Fresh ``confirmed``/``tentative`` for an established, active entity, else None."""
        track = self._tracks.get(object_id)
        if track is None or track.state != "active" or not track.established:
            return None
        return CONFIRMED if len(track.viewpoints) >= self._cfg.min_viewpoints else TENTATIVE

    def basis_of(self, object_id: str) -> str | None:
        """How this identity was last established: ``tracked`` (position), ``reacquired``
        (appearance), ``twin-anchor`` (position-anchored look-alike — honest ambiguity),
        ``restored`` (past session), or ``new``."""
        track = self._tracks.get(object_id)
        return track.basis if track is not None else None

    def close(self) -> None:
        """Flush/close the gallery store (hands this session's galleries to a later model
        opened on the same ``history_path``)."""
        if self._gallery is not None:
            self._gallery.close()


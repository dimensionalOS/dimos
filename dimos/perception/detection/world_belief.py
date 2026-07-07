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
import math
from typing import TYPE_CHECKING, Any
import uuid

import numpy as np

from dimos.models.embedding.base import Embedding
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.absence import ABSENT, PRESENT, classify_visibility
from dimos.perception.detection.identity_features import (
    add_diverse_embedding_view,
    gallery_cos,
    normalize_embedding,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

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

Viewpoint = tuple["NDArray[np.float64]", "NDArray[np.float64]"]  # (camera position, forward)

_CANDIDATE_TTL_S = 5.0  # never-established junk is dropped after this long unseen
_MAX_ABSENT = 128  # geometric death never times out, so cap retained corpses
_POSITION_WINDOW = 9  # running-median window for the entity center


def camera_viewpoint(world_from_camera: NDArray[np.float64]) -> Viewpoint:
    """The camera vantage (world position, forward direction) from its pose matrix.

    Derived purely from the (accurate, FK/tf-based) camera pose — never the noisy object
    center — so a static camera yields a *constant* vantage that cannot be jittered into a
    second viewpoint.
    """
    mat = np.asarray(world_from_camera, dtype=np.float64)
    return mat[:3, 3].copy(), mat[:3, 2].copy()  # translation, +z axis (viewing direction)


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
        near_in_space = float(np.linalg.norm(pos - prev_pos)) < min_baseline_m
        near_in_angle = float(np.dot(fwd, prev_fwd)) > cos_thresh
        if near_in_space and near_in_angle:
            return False
    return True


def _appearance_vec(obj: Object) -> NDArray[np.float32] | None:
    """The object's identity embedding: DINO visual preferred (what the gates were
    measured on), CLIP semantic as fallback when no visual embedder is configured."""
    for attr in ("visual_embedding", "embedding"):
        vec = normalize_embedding(getattr(obj, attr, None))
        if vec is not None:
            return vec
    return None


class _Track:
    """One believed entity: the folded Object plus its identity/lifecycle state."""

    __slots__ = (
        "absent_votes", "basis", "established", "first_ts", "gallery", "label_counts",
        "last_pos", "last_seen", "latest_emb", "n_frames", "obj", "positions",
        "sizes", "state", "viewpoints",
    )

    def __init__(self, obj: Object, ts: float, viewpoint: Viewpoint | None, basis: str) -> None:
        self.obj = obj
        self.label_counts: Counter[str] = Counter([obj.name])
        self.first_ts = ts
        self.last_seen = ts
        self.n_frames = 1
        # Association anchors on the LAST raw observed position (it follows a carried
        # object frame-to-frame); the REPORTED center is the running median (stable).
        self.last_pos: NDArray[np.float64] = obj.center.to_numpy()
        self.positions: list[NDArray[np.float64]] = [obj.center.to_numpy()]
        try:
            self.sizes: list[float] = [float(max(obj.size.x, obj.size.y, obj.size.z))]
        except Exception:
            self.sizes = []
        self.gallery: list[NDArray[np.float32]] = []
        self.latest_emb: NDArray[np.float32] | None = None
        self.viewpoints: list[Viewpoint] = [viewpoint] if viewpoint is not None else []
        self.absent_votes = 0
        self.state = "active"  # "active" | "absent"
        self.established = False
        self.basis = basis

    @property
    def modal_label(self) -> str:
        return self.label_counts.most_common(1)[0][0]

    def full_gallery(self) -> list[NDArray[np.float32]]:
        """Diverse views UNION the latest view — re-acquire cos was measured over this
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
                    # the entity's CURRENT running-median center — a raw (possibly depth-
                    # mode flipped) sample stored here nearly broke a cross-session restore
                    "pos": " ".join(f"{c:.4f}" for c in position),
                    "n": str(int(n_frames)),  # sighting support (restore arbitration)
                },
                embedding=Embedding(vec),
            )
        except Exception as e:
            logger.warning("gallery remember failed (identity continues in RAM): %s", e)

    def lookup(self, obj: Object) -> dict[str, tuple[float, str, NDArray[np.float64] | None, int]]:
        """``{object_id: (best_cos, label, stored_position | None, support)}`` over the
        persisted views. The SCORE is the max cosine over an id's views, but the
        label/position/support come from its NEWEST view (max ``n`` — monotonic per id):
        taking the max-cos view's tags handed a returning object the stale early-view
        support/position and inverted the twin support-ratio arbitration (verified)."""
        try:
            vec = _appearance_vec(obj)
            if vec is None:
                return {}
            best_cos: dict[str, float] = {}
            newest: dict[str, tuple[int, str, NDArray[np.float64] | None]] = {}
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
                if eid not in newest or support >= newest[eid][0]:
                    newest[eid] = (support, hit.tags.get("name") or "", pos)
            return {
                eid: (best_cos[eid], newest[eid][1], newest[eid][2], newest[eid][0])
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
    """Identity core of the complete adaptive fold. See the module docstring for the four
    association rules and the measurements behind every threshold."""

    def __init__(
        self,
        *,
        min_frames: int = 3,
        min_span_s: float = 1.5,
        absent_threshold: int = 2,
        min_viewpoints: int = 2,
        history_path: str | None = None,
    ) -> None:
        # Measured constants (module docstring has the rationale for each).
        self._radius_base = 0.18
        self._radius_growth = 0.15
        self._radius_cap = 0.40
        self._mint_guard = 0.13
        self._label_override_cos = 0.8
        self._reacq_cos = 0.5
        self._reacq_margin = 0.05
        self._min_frames = min_frames
        self._min_span = min_span_s
        self._absent_threshold = absent_threshold
        self._min_viewpoints = min_viewpoints
        self._viewpoint_angle_deg = 10.0
        self._min_baseline_m = 0.05
        self._gallery_novelty = 0.9
        self._gallery_max = 8
        self._tracks: dict[str, _Track] = {}
        # Fold watermark: strictly-forward time is the whole idempotency story — a frame
        # at or before the newest folded ts is skipped entirely.
        self._now: float = float("-inf")
        # Appearance persistence over memory2 vec0 (cross-session by reopening the same
        # path). None → within-session identity only (RAM galleries still work).
        self._gallery = (
            _GalleryStore(history_path, novelty=0.9) if history_path is not None else None
        )

    def _radius(self, dt: float) -> float:
        return min(self._radius_base + self._radius_growth * max(dt, 0.0), self._radius_cap)

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
        posed = camera_transform is not None
        world_from_camera = (
            camera_transform.to_matrix() if camera_transform is not None
            else np.eye(4, dtype=np.float64)
        )
        viewpoint = camera_viewpoint(world_from_camera) if posed else None
        for obj in objects:
            # Replace the OBB-midpoint center with the robust point-median BEFORE any
            # association, so matching, folding, and reporting share one estimate
            # (midpoint excursions measured at 135 mm on a static can; median < 5 mm).
            obj.center = self._observation_center(obj)

        matched, unmatched, seen = self._match_frame(objects, ts)
        for track, obj in matched:
            self._hit(track, obj, ts, viewpoint, BASIS_TRACKED)
        for obj in unmatched:
            track = self._acquire(obj, ts, viewpoint, seen)
            if track is not None:
                seen.add(id(track))  # a re-acquired/minted track was observed this frame
        if posed:
            self._vote_absence(seen, camera_info, world_from_camera, depth_m)
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
                dist = float(np.linalg.norm(obj.center.to_numpy() - track.last_pos))
                if dist > radius:
                    continue
                # A det inside a neighbour's envelope is never evidence for a third
                # party reaching in on its dt-grown radius (a det-starved track would
                # otherwise camp on a neighbour's label misfires — observed live).
                if dist > self._radius_base and any(
                    cos is None or cos >= self._reacq_cos or obj.name == other.modal_label
                    for cos, other in self._envelope_neighbours(obj, exclude=track)
                ):
                    continue
                pairs.append((dist, track, obj))
        pairs.sort(key=lambda p: p[0])
        used_tracks: set[int] = set()
        used_objs: set[int] = set()
        matched: list[tuple[_Track, Object]] = []
        for _dist, track, obj in pairs:
            if id(track) in used_tracks or id(obj) in used_objs:
                continue
            cos = gallery_cos(_appearance_vec(obj), track.full_gallery())
            if obj.name != track.modal_label:
                # Label flip: only a strong appearance match overrides (same object across
                # a 180° rotation scores 0.94 median; cross-object max 0.334).
                if cos is None or cos < self._label_override_cos:
                    continue
            elif cos is not None and cos < self._reacq_cos:
                # Appearance-contradiction veto: the label agrees but the crop is clearly
                # foreign (a cup misfiring as 'green can' scores max ~0.35 against the
                # can's gallery, while a continuously-tracked object stays high because
                # the gallery ingests every accepted view). Labels can lie — without this
                # veto a det-starved track parks next to a neighbour and lives off its
                # misfires, poisoning its own gallery (observed on the benchmark).
                continue
            used_tracks.add(id(track))
            used_objs.add(id(obj))
            matched.append((track, obj))
        return matched, [o for o in objects if id(o) not in used_objs], used_tracks

    def _envelope_neighbours(self, obj: Object, exclude: _Track | None = None):
        """Yield ``(appearance_cos | None, track)`` for every established active track
        whose noise envelope (0.13 m, the measured max static lift jump) contains this
        detection.

        THE core arbitration primitive: a detection standing on an established object's
        spot is ambiguous by physics — it may simply be that object, whatever label or
        look the detector gave it this frame. The co-located object is therefore the
        null hypothesis, and every other interpretation of the detection (a far track
        claiming it, an absent identity re-acquiring it, a new object being minted from
        it) must beat it. The three call sites are those three interpretations."""
        center = obj.center.to_numpy()
        vec = _appearance_vec(obj)
        for track in self._tracks.values():
            if track is exclude or track.state != "active" or not track.established:
                continue
            if float(np.linalg.norm(center - track.last_pos)) > self._mint_guard:
                continue
            yield gallery_cos(vec, track.full_gallery()), track

    def _acquire(
        self, obj: Object, ts: float, viewpoint: Viewpoint | None, seen_tracks: set[int]
    ) -> _Track | None:
        """An unmatched detection: re-acquire an absent/past-session identity, or mint a
        new one (unless the mint-guard says it is an established neighbour misfiring).
        Returns the track the detection landed on (None when suppressed)."""
        eid, basis, win_cos = self._reacquire_candidate(obj)
        if eid is not None and any(
            cos is not None and cos >= win_cos for cos, _t in self._envelope_neighbours(obj)
        ):
            # Relative evidence: a det in a neighbour's envelope re-acquires an absent
            # id only if it resembles that id MORE than the neighbour (blocks a
            # neighbour's borderline misfire from stealing an orphaned id; a true
            # return at cos-to-self > the 0.843 cross-object ceiling still passes).
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
        # Mint-guard: no new object inside an established neighbour's envelope when the
        # det plausibly IS that neighbour misfiring (same label, or appearance not
        # clearly foreign — a truly different-looking replacement may still mint). A
        # co-detected neighbour is the exception: co-detection is evidence of a second
        # object (a placed twin must mint) — unless the det is a different-label clone
        # of it (its own misfire under another prompt).
        for cos, track in self._envelope_neighbours(obj):
            if id(track) in seen_tracks:
                if obj.name != track.modal_label and cos is not None and cos >= self._reacq_cos:
                    logger.debug("belief: misfire suppressed near %s (%s)", track.obj.object_id[:8], obj.name)
                    return None
                continue
            if cos is None or cos >= self._reacq_cos or obj.name == track.modal_label:
                logger.debug("belief: mint suppressed near %s (%s)", track.obj.object_id[:8], obj.name)
                return None
        self._insert(obj, ts, viewpoint, basis=BASIS_NEW)
        return self._tracks[obj.object_id]

    def _reacquire_candidate(self, obj: Object) -> tuple[str | None, str | None, float | None]:
        """``(entity_id, basis, winning_cos)`` for an absent or past-session identity this
        detection re-acquires, or ``(None, None, None)``.

        Gate: best gallery cos ≥ 0.5 (measured true re-acquires at 0.66-0.78 on
        mid-placement crops — 0.8 would refuse all of them) with best-minus-second margin
        ≥ 0.05. A same-label near-tie is the twin case: appearance is undecidable
        (inter-twin cos up to 0.905), so anchor to the nearer candidate by position and
        say so via basis ``twin-anchor``. Differently-labeled near-ties are refused."""
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
        if best_cos < self._reacq_cos:
            return None, None, None
        # Same acceptance rule as the matcher: label agreement OR appearance >= 0.8
        # (blocks cross-label id theft in the measured 0.56-0.843 same-shape band).
        if obj.name != best_label and best_cos < self._label_override_cos:
            logger.debug(
                "belief: re-acquire refused — label %r vs candidate %r at cos %.2f (< %.2f override)",
                obj.name, best_label, best_cos, self._label_override_cos,
            )
            return None, None, None
        if len(ranked) == 1 or best_cos - ranked[1][0] >= self._reacq_margin:
            return best_eid, BASIS_REACQUIRED, best_cos
        second_cos, second_eid, second_label, second_pos, second_n = ranked[1]
        if second_label == best_label and second_cos >= self._reacq_cos:
            # Same-label near-tie. REAL twins carry comparable sighting support and are
            # decided by position; a lopsided support ratio (≥ 3×) means the lightweight
            # is a parasite alias of the heavyweight (e.g. a hand-split double-detection
            # shadow minted during placement) — the accumulated identity wins, wherever
            # the alias's stale stored position happens to sit.
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
                if dist <= self._radius_cap:
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
        track = _Track(obj, ts, viewpoint, basis)
        track.established = self._is_established(track, ts)  # true only for 1-frame/0-span configs
        track.ingest_appearance(obj, self._gallery_novelty, self._gallery_max)
        self._tracks[eid] = track
        if self._gallery is not None and track.established:
            self._gallery.remember(eid, obj, obj.center.to_numpy(), track.n_frames)

    def _is_established(self, track: _Track, ts: float) -> bool:
        return track.n_frames >= self._min_frames and (ts - track.first_ts) >= self._min_span

    def _hit(self, track: _Track, obj: Object, ts: float, viewpoint: Viewpoint | None, basis: str) -> None:
        was_absent = track.state == "absent"
        track.state = "active"
        track.absent_votes = 0
        track.last_seen = max(track.last_seen, ts)
        track.n_frames += 1
        track.label_counts[obj.name] += 1
        track.basis = basis
        if not track.established:
            track.established = self._is_established(track, ts)
        track.ingest_appearance(obj, self._gallery_novelty, self._gallery_max)
        new_pos = obj.center.to_numpy()
        # The association anchor follows EVERY accepted observation — even a geometry-
        # suspect one. Freezing it while last_seen resets dt left tracks anchored at a
        # stale spot with a shrunken radius (verified: a partial-view re-acquire kept
        # the anchor at the absence-confirmed old spot and the next clean detection
        # minted a fork of the identity).
        track.last_pos = new_pos
        suspect = self._geometry_suspect(track, obj)
        if suspect and was_absent:
            # Partial-view re-acquire: the geometry is too poor to FOLD, but an
            # imprecise observed position strictly beats reporting the old spot that
            # absence voting already proved empty (same for the accumulated cloud).
            track.positions = [new_pos]
            track.obj.center = Vector3(new_pos)
            self._reset_cloud(track.obj, obj)
        if not suspect:
            entity = track.obj
            entity.update_object(obj)  # latest size/pose bookkeeping
            # The entity's cloud is its FRESHEST observed surface — always paired with
            # the reported detection box. (Accumulating a fused multi-view cloud was
            # speculation nothing consumes yet; it bought a movement-trail artifact and
            # unbounded growth. A consumer that wants fusion can fold it from the
            # recording — everything is stored.)
            self._reset_cloud(entity, obj)
            try:
                track.sizes.append(float(max(obj.size.x, obj.size.y, obj.size.z)))
                del track.sizes[:-_POSITION_WINDOW]
            except Exception:
                pass
            positions = track.positions
            if was_absent and float(np.linalg.norm(positions[-1] - new_pos)) > self._radius_base:
                # Re-acquired at a NEW spot: stale pre-departure positions must not
                # outvote the return (probe-confirmed ghost at the old empty spot).
                positions.clear()
            positions.append(new_pos)
            del positions[:-_POSITION_WINDOW]
            # Re-anchor on a genuine move: when the last 3 observations agree with each
            # other but ALL disagree with the running median, the object has moved —
            # reset the window instead of letting old positions outvote the new spot.
            if len(positions) >= 3:
                last3 = positions[-3:]
                med = np.median(np.asarray(positions), axis=0)
                coherent = all(
                    float(np.linalg.norm(p - last3[-1])) < 0.5 * self._radius_base for p in last3
                )
                moved = all(float(np.linalg.norm(p - med)) > self._radius_base for p in last3)
                if coherent and moved:
                    track.positions = positions = list(last3)
            entity.center = Vector3(np.median(np.asarray(positions), axis=0))
        # identity/support still counts on geometry-suspect frames (partial view, mask
        # bleed) — we saw the object; we just don't trust this frame's geometry.
        if viewpoint is not None and is_novel_viewpoint(
            viewpoint, track.viewpoints,
            min_baseline_m=self._min_baseline_m, angle_deg=self._viewpoint_angle_deg,
        ):
            track.viewpoints.append(viewpoint)  # genuinely new vantage → trust credit
        # Only ESTABLISHED identities persist to the store: a junk candidate's views must
        # never become restorable in a later session. (Its pre-establishment views stay
        # in the RAM gallery, so within-session re-acquire loses nothing.)
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
        """True when this observation's geometry should not be folded (identity/support
        still counts): bbox touches the image border (partial view) or the measured size
        is wildly larger than the entity's running median (mask bleed / motion blur —
        median reference, so an escalating bleed can't ratchet the gate open)."""
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
        world_from_camera: NDArray[np.float64],
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
                if track.absent_votes >= self._absent_threshold:
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
        ``_CANDIDATE_TTL_S``, and cap retained absent corpses at ``_MAX_ABSENT``."""
        stale = [
            eid for eid, t in self._tracks.items()
            if not t.established and (ts - t.last_seen) > _CANDIDATE_TTL_S
        ]
        for eid in stale:
            del self._tracks[eid]
        absent = [eid for eid, t in self._tracks.items() if t.state == "absent"]
        if len(absent) > _MAX_ABSENT:
            for eid in sorted(absent, key=lambda e: self._tracks[e].last_seen)[: len(absent) - _MAX_ABSENT]:
                del self._tracks[eid]

    # ── queries ───────────────────────────────────────────────────────────────────

    def present(self, *, min_viewpoints: int | None = None) -> list[Object]:
        """Believed-present objects: established and not geometrically absent.

        ``min_viewpoints`` filters to viewpoint-verified objects. Returned objects are
        the live internal entities — treat as read-only. Per-object trust via
        :meth:`trust_of`, identity basis via :meth:`basis_of`."""
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
        return CONFIRMED if len(track.viewpoints) >= self._min_viewpoints else TENTATIVE

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


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

"""Memory2-backed durable history for WorldBelief."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from pathlib import Path
import re
from typing import TYPE_CHECKING, Any

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.models.embedding.base import Embedding
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.identity_features import (
    add_diverse_embedding_view,
    normalize_embedding,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.perception.detection.type.detection3d.object import Object

logger = setup_logger()


@dataclass(frozen=True)
class RehydratedWorldBeliefState:
    """Compact state restored from durable object evidence history."""

    entities: dict[str, Object]
    meta: dict[str, dict[str, Any]]
    now: float


class WorldBeliefHistoryStore:
    """Memory2 history adapter for WorldBelief durable identity evidence."""

    def __init__(
        self,
        *,
        enabled: bool,
        path: str | None,
        stream_name: str,
        gallery_size: int,
        gallery_novelty: float,
    ) -> None:
        self.enabled = enabled
        self.path = path
        self.stream_name = stream_name
        self.gallery_size = int(gallery_size)
        self.gallery_novelty = float(gallery_novelty)
        self._store: Any = None
        self._stream: Any = None
        self._vector_streams: dict[str, Any] = {}

    def ensure_open(self) -> None:
        """Lazily open the Memory2 SqliteStore used for durable identity history."""
        if not self.enabled or self._stream is not None:
            return
        from dimos.memory2.store.sqlite import SqliteStore

        if self.path is None:
            return
        Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        self._store = SqliteStore(path=self.path)
        self._store.start()
        self._stream = self._store.stream(self.stream_name, dict, codec="pickle")

    def close(self) -> None:
        if self._store is not None:
            try:
                self._store.stop()
            except Exception:
                pass
            self._store = None
            self._stream = None
            self._vector_streams = {}

    @staticmethod
    def _safe_stream_part(value: Any) -> str:
        text = str(value or "unknown").lower()
        text = re.sub(r"[^a-z0-9]+", "_", text).strip("_")
        return text or "unknown"

    def _embedding_stream_name(self, role: str, model: Any, dim: int) -> str:
        role_part = self._safe_stream_part(role)
        model_part = self._safe_stream_part(model)
        return f"{self.stream_name}_{role_part}_{model_part}_{int(dim)}"

    def _embedding_stream(self, role: str, model: Any, dim: int) -> Any:
        name = self._embedding_stream_name(role, model, dim)
        stream = self._vector_streams.get(name)
        if stream is None:
            assert self._store is not None
            stream = self._store.stream(name, dict, codec="pickle")
            self._vector_streams[name] = stream
        return stream

    def _append_vector_evidence(
        self,
        *,
        role: str,
        obj: Object,
        entered_t: float,
        now: float,
        emb: np.ndarray[Any, np.dtype[np.float32]],
        model: Any,
        device: Any,
    ) -> None:
        stream = self._embedding_stream(role, model, int(emb.size))
        c = obj.center
        stream.append(
            {
                "object_id": obj.object_id,
                "name": obj.name,
                "role": role,
                "pos": [c.x, c.y, c.z] if c is not None else None,
                "entered_t": entered_t,
                "embedding_model": model,
                "embedding_device": device,
                "embedding_dim": int(emb.size),
            },
            ts=now,
            tags={"object_id": obj.object_id, "name": obj.name, "role": role},
            embedding=Embedding(vector=emb.astype(np.float32)),
        )

    def append_object(
        self,
        obj: Object,
        entered_t: float,
        now: float,
        *,
        identity_obj: Object | None = None,
        include_identity: bool = True,
    ) -> None:
        """Append one durable object evidence event to Memory2."""
        if not self.enabled:
            return
        try:
            self.ensure_open()
            if self._stream is None:
                return
            c = obj.center
            payload: dict[str, Any] = {
                "object_id": obj.object_id,
                "name": obj.name,
                "pos": [c.x, c.y, c.z] if c is not None else None,
                "entered_t": entered_t,
            }
            if include_identity:
                source = identity_obj or obj
                semantic_emb = normalize_embedding(getattr(source, "embedding", None))
                if semantic_emb is not None:
                    payload["semantic_embedding"] = semantic_emb.astype(np.float32).tolist()
                    payload["semantic_embedding_model"] = getattr(source, "embedding_model", None)
                    payload["semantic_embedding_device"] = getattr(source, "embedding_device", None)
                    payload["semantic_embedding_dim"] = int(semantic_emb.size)
                visual_emb = normalize_embedding(getattr(source, "visual_embedding", None))
                if visual_emb is not None:
                    payload["visual_embedding"] = visual_emb.astype(np.float32).tolist()
                    payload["visual_embedding_model"] = getattr(
                        source, "visual_embedding_model", None
                    )
                    payload["visual_embedding_device"] = getattr(
                        source, "visual_embedding_device", None
                    )
                    payload["visual_embedding_dim"] = int(visual_emb.size)
            self._stream.append(
                payload,
                ts=now,
                tags={"object_id": obj.object_id, "name": obj.name},
            )
            if include_identity:
                source = identity_obj or obj
                if semantic_emb is not None:
                    self._append_vector_evidence(
                        role="semantic",
                        obj=obj,
                        entered_t=entered_t,
                        now=now,
                        emb=semantic_emb,
                        model=getattr(source, "embedding_model", None),
                        device=getattr(source, "embedding_device", None),
                    )
                if visual_emb is not None:
                    self._append_vector_evidence(
                        role="visual",
                        obj=obj,
                        entered_t=entered_t,
                        now=now,
                        emb=visual_emb,
                        model=getattr(source, "visual_embedding_model", None),
                        device=getattr(source, "visual_embedding_device", None),
                    )
        except Exception as exc:
            logger.warning(
                f"WorldBelief history append failed; continuing without persistence: {exc}"
            )

    def when_entered(self, object_id: str) -> float | None:
        if not self.enabled:
            return None
        self.ensure_open()
        if self._stream is None:
            return None
        best = None
        for obs in self._stream.tags(object_id=object_id):
            d = getattr(obs, "data", None) or obs._data
            et = d.get("entered_t", obs.ts) if isinstance(d, dict) else obs.ts
            best = et if best is None else min(best, et)
        return best

    def _aggregate_events(self) -> dict[str, dict[str, Any]]:
        self.ensure_open()
        if self._stream is None:
            return {}
        agg: dict[str, dict[str, Any]] = {}
        for obs in self._stream:
            d = getattr(obs, "data", None) or obs._data
            if not isinstance(d, dict):
                continue
            oid = d.get("object_id")
            if oid is None or d.get("pos") is None:
                continue
            a = agg.setdefault(
                oid,
                {
                    "entered": d.get("entered_t", obs.ts),
                    "last": obs.ts,
                    "pos": d["pos"],
                    "labels": Counter(),
                    "n": 0,
                    "semantic_emb": None,
                    "semantic_emb_ts": -1.0,
                    "semantic_gallery": [],
                    "semantic_embedding_model": None,
                    "semantic_embedding_device": None,
                    "semantic_embedding_dim": None,
                    "visual_emb": None,
                    "visual_emb_ts": -1.0,
                    "visual_gallery": [],
                    "visual_embedding_model": None,
                    "visual_embedding_device": None,
                    "visual_embedding_dim": None,
                },
            )
            a["entered"] = min(a["entered"], d.get("entered_t", obs.ts))
            if obs.ts >= a["last"]:
                a["last"], a["pos"] = obs.ts, d["pos"]
            a["labels"][d.get("name", "object")] += 1
            a["n"] += 1

            raw_semantic = d.get("semantic_embedding")
            if raw_semantic is None and d.get("visual_embedding") is None:
                raw_semantic = d.get("emb")
            if raw_semantic is None:
                raw_semantic = d.get("embedding")
            semantic_emb = normalize_embedding(raw_semantic)
            if semantic_emb is not None:
                add_diverse_embedding_view(
                    a.setdefault("semantic_gallery", []),
                    semantic_emb,
                    novelty=self.gallery_novelty,
                    max_size=self.gallery_size,
                )
                if obs.ts >= a["semantic_emb_ts"]:
                    a["semantic_emb"] = semantic_emb
                    a["semantic_emb_ts"] = obs.ts
                    a["semantic_embedding_model"] = d.get(
                        "semantic_embedding_model",
                        d.get("embedding_model"),
                    )
                    a["semantic_embedding_device"] = d.get(
                        "semantic_embedding_device",
                        d.get("embedding_device"),
                    )
                    a["semantic_embedding_dim"] = d.get(
                        "semantic_embedding_dim",
                        d.get("embedding_dim", int(semantic_emb.size)),
                    )

            visual_emb = normalize_embedding(d.get("visual_embedding"))
            if visual_emb is not None:
                add_diverse_embedding_view(
                    a.setdefault("visual_gallery", []),
                    visual_emb,
                    novelty=self.gallery_novelty,
                    max_size=self.gallery_size,
                )
                if obs.ts >= a["visual_emb_ts"]:
                    a["visual_emb"] = visual_emb
                    a["visual_emb_ts"] = obs.ts
                    a["visual_embedding_model"] = d.get("visual_embedding_model")
                    a["visual_embedding_device"] = d.get("visual_embedding_device")
                    a["visual_embedding_dim"] = d.get("visual_embedding_dim", int(visual_emb.size))

        return agg

    def rehydrate(self) -> RehydratedWorldBeliefState:
        """Return compact maintained-entity state restored from durable evidence."""
        from dimos.perception.detection.type.detection3d.object import Object

        if not self.enabled:
            return RehydratedWorldBeliefState(entities={}, meta={}, now=0.0)
        agg = self._aggregate_events()
        img = Image(np.zeros((2, 2, 3), np.uint8))
        entities: dict[str, Object] = {}
        meta: dict[str, dict[str, Any]] = {}
        for oid, a in agg.items():
            p = a["pos"]
            v = Vector3(p)
            pc = PointCloud2(pointcloud=o3d.geometry.PointCloud(), frame_id="world", ts=a["last"])
            name = a["labels"].most_common(1)[0][0]
            entities[oid] = Object(
                object_id=oid,
                center=v,
                size=Vector3(0.05, 0.05, 0.1),
                pose=PoseStamped(position=v),
                pointcloud=pc,
                image=img,
                bbox=(0, 0, 1, 1),
                track_id=-1,
                class_id=0,
                confidence=0.5,
                name=name,
                ts=a["last"],
                embedding=a["semantic_emb"],
                embedding_model=a["semantic_embedding_model"],
                embedding_device=a["semantic_embedding_device"],
                embedding_dim=a["semantic_embedding_dim"],
                visual_embedding=a["visual_emb"],
                visual_embedding_model=a["visual_embedding_model"],
                visual_embedding_device=a["visual_embedding_device"],
                visual_embedding_dim=a["visual_embedding_dim"],
            )
            meta[oid] = {
                "entered_t": a["entered"],
                "last_seen": a["last"],
                "support": a["n"],
                "window": [],
                "labels": a["labels"],
                "semantic_emb": a["semantic_emb"],
                "semantic_gallery": a["semantic_gallery"],
                "semantic_embedding_model": a["semantic_embedding_model"],
                "semantic_embedding_device": a["semantic_embedding_device"],
                "semantic_embedding_dim": a["semantic_embedding_dim"],
                "visual_emb": a["visual_emb"],
                "visual_gallery": a["visual_gallery"],
                "visual_embedding_model": a["visual_embedding_model"],
                "visual_embedding_device": a["visual_embedding_device"],
                "visual_embedding_dim": a["visual_embedding_dim"],
                "positions": [p],
            }
        now = max((a["last"] for a in agg.values()), default=0.0)
        return RehydratedWorldBeliefState(entities=entities, meta=meta, now=now)

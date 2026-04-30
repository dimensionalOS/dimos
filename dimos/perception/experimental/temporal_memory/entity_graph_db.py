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

"""
Entity Graph Database for storing and querying entity relationships.

Maintains two graph types sharing the same entity nodes:
1. Relations Graph: Interactions between entities (holds, looks_at, talks_to, etc.)
2. Distance Graph: Spatial distances between entities
"""

from __future__ import annotations

import json
from pathlib import Path
import sqlite3
import threading
from typing import TYPE_CHECKING, Any

from dimos.utils.logging_config import setup_logger

from .temporal_utils.parsers import parse_batch_distance_response
from .temporal_utils.prompts import build_batch_distance_estimation_prompt

if TYPE_CHECKING:
    from dimos.models.vl.base import VlModel
    from dimos.msgs.sensor_msgs.Image import Image

logger = setup_logger()


class EntityGraphDB:
    """SQLite-based graph database for entity relationships.

    Thread-safe implementation using connection-per-thread pattern.
    """

    def __init__(self, db_path: str | Path) -> None:
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._local = threading.local()
        self._init_schema()
        logger.info(f"EntityGraphDB initialized at {self.db_path}")

    def _get_connection(self) -> sqlite3.Connection:
        """Get thread-local database connection."""
        if not hasattr(self._local, "conn"):
            self._local.conn = sqlite3.connect(str(self.db_path))
            self._local.conn.row_factory = sqlite3.Row
        conn: sqlite3.Connection = self._local.conn
        return conn

    def _init_schema(self) -> None:
        conn = self._get_connection()
        cursor = conn.cursor()

        cursor.execute("""
            CREATE TABLE IF NOT EXISTS entities (
                entity_id TEXT PRIMARY KEY,
                entity_type TEXT NOT NULL,
                descriptor TEXT,
                first_seen_ts REAL NOT NULL,
                last_seen_ts REAL NOT NULL,
                metadata TEXT
            )
        """)
        cursor.execute(
            "CREATE INDEX IF NOT EXISTS idx_entities_first_seen ON entities(first_seen_ts)"
        )
        cursor.execute(
            "CREATE INDEX IF NOT EXISTS idx_entities_last_seen ON entities(last_seen_ts)"
        )

        cursor.execute("""
            CREATE TABLE IF NOT EXISTS relations (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                relation_type TEXT NOT NULL,
                subject_id TEXT NOT NULL,
                object_id TEXT NOT NULL,
                confidence REAL DEFAULT 1.0,
                timestamp_s REAL NOT NULL,
                evidence TEXT,
                notes TEXT,
                FOREIGN KEY (subject_id) REFERENCES entities(entity_id),
                FOREIGN KEY (object_id) REFERENCES entities(entity_id)
            )
        """)
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_relations_subject ON relations(subject_id)")
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_relations_object ON relations(object_id)")
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_relations_type ON relations(relation_type)")
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_relations_time ON relations(timestamp_s)")

        cursor.execute("""
            CREATE TABLE IF NOT EXISTS distances (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                entity_a_id TEXT NOT NULL,
                entity_b_id TEXT NOT NULL,
                distance_meters REAL,
                distance_category TEXT,
                confidence REAL DEFAULT 1.0,
                timestamp_s REAL NOT NULL,
                method TEXT,
                FOREIGN KEY (entity_a_id) REFERENCES entities(entity_id),
                FOREIGN KEY (entity_b_id) REFERENCES entities(entity_id)
            )
        """)
        cursor.execute(
            "CREATE INDEX IF NOT EXISTS idx_distances_pair ON distances(entity_a_id, entity_b_id)"
        )
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_distances_time ON distances(timestamp_s)")

        # Drop legacy semantic_relations table if it exists
        cursor.execute("DROP TABLE IF EXISTS semantic_relations")

        conn.commit()

    # Entity Operations

    def upsert_entity(
        self,
        entity_id: str,
        entity_type: str,
        descriptor: str,
        timestamp_s: float,
        metadata: dict[str, Any] | None = None,
    ) -> None:
        conn = self._get_connection()
        cursor = conn.cursor()
        metadata_json = json.dumps(metadata) if metadata else None
        cursor.execute(
            """
            INSERT INTO entities (entity_id, entity_type, descriptor, first_seen_ts, last_seen_ts, metadata)
            VALUES (?, ?, ?, ?, ?, ?)
            ON CONFLICT(entity_id) DO UPDATE SET
                last_seen_ts = ?,
                descriptor = COALESCE(excluded.descriptor, descriptor),
                metadata = COALESCE(metadata, excluded.metadata)
            """,
            (
                entity_id,
                entity_type,
                descriptor,
                timestamp_s,
                timestamp_s,
                metadata_json,
                timestamp_s,
            ),
        )
        conn.commit()

    def get_entity(self, entity_id: str) -> dict[str, Any] | None:
        conn = self._get_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM entities WHERE entity_id = ?", (entity_id,))
        row = cursor.fetchone()
        if row is None:
            return None
        return {
            "entity_id": row["entity_id"],
            "entity_type": row["entity_type"],
            "descriptor": row["descriptor"],
            "first_seen_ts": row["first_seen_ts"],
            "last_seen_ts": row["last_seen_ts"],
            "metadata": json.loads(row["metadata"]) if row["metadata"] else None,
        }

    def get_all_entities(self, entity_type: str | None = None) -> list[dict[str, Any]]:
        conn = self._get_connection()
        cursor = conn.cursor()
        if entity_type:
            cursor.execute(
                "SELECT * FROM entities WHERE entity_type = ? ORDER BY last_seen_ts DESC",
                (entity_type,),
            )
        else:
            cursor.execute("SELECT * FROM entities ORDER BY last_seen_ts DESC")
        return [
            {
                "entity_id": row["entity_id"],
                "entity_type": row["entity_type"],
                "descriptor": row["descriptor"],
                "first_seen_ts": row["first_seen_ts"],
                "last_seen_ts": row["last_seen_ts"],
                "metadata": json.loads(row["metadata"]) if row["metadata"] else None,
            }
            for row in cursor.fetchall()
        ]

    def get_entities_by_time(
        self, time_window: tuple[float, float], first_seen: bool = True
    ) -> list[dict[str, Any]]:
        conn = self._get_connection()
        cursor = conn.cursor()
        ts_field = "first_seen_ts" if first_seen else "last_seen_ts"
        cursor.execute(
            f"SELECT * FROM entities WHERE {ts_field} BETWEEN ? AND ? ORDER BY {ts_field} DESC",
            time_window,
        )
        return [
            {
                "entity_id": row["entity_id"],
                "entity_type": row["entity_type"],
                "descriptor": row["descriptor"],
                "first_seen_ts": row["first_seen_ts"],
                "last_seen_ts": row["last_seen_ts"],
                "metadata": json.loads(row["metadata"]) if row["metadata"] else None,
            }
            for row in cursor.fetchall()
        ]

    # Relation Operations

    def add_relation(
        self,
        relation_type: str,
        subject_id: str,
        object_id: str,
        confidence: float,
        timestamp_s: float,
        evidence: list[str] | None = None,
        notes: str | None = None,
    ) -> None:
        conn = self._get_connection()
        cursor = conn.cursor()
        evidence_json = json.dumps(evidence) if evidence else None
        cursor.execute(
            """
            INSERT INTO relations (relation_type, subject_id, object_id, confidence, timestamp_s, evidence, notes)
            VALUES (?, ?, ?, ?, ?, ?, ?)
            """,
            (relation_type, subject_id, object_id, confidence, timestamp_s, evidence_json, notes),
        )
        conn.commit()

    def get_relations_for_entity(
        self,
        entity_id: str,
        relation_type: str | None = None,
        time_window: tuple[float, float] | None = None,
    ) -> list[dict[str, Any]]:
        conn = self._get_connection()
        cursor = conn.cursor()
        query = "SELECT * FROM relations WHERE (subject_id = ? OR object_id = ?)"
        params: list[Any] = [entity_id, entity_id]
        if relation_type:
            query += " AND relation_type = ?"
            params.append(relation_type)
        if time_window:
            query += " AND timestamp_s BETWEEN ? AND ?"
            params.extend(time_window)
        query += " ORDER BY timestamp_s DESC"
        cursor.execute(query, params)
        return [
            {
                "id": row["id"],
                "relation_type": row["relation_type"],
                "subject_id": row["subject_id"],
                "object_id": row["object_id"],
                "confidence": row["confidence"],
                "timestamp_s": row["timestamp_s"],
                "evidence": json.loads(row["evidence"]) if row["evidence"] else None,
                "notes": row["notes"],
            }
            for row in cursor.fetchall()
        ]

    def get_recent_relations(self, limit: int = 50) -> list[dict[str, Any]]:
        conn = self._get_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM relations ORDER BY timestamp_s DESC LIMIT ?", (limit,))
        return [
            {
                "id": row["id"],
                "relation_type": row["relation_type"],
                "subject_id": row["subject_id"],
                "object_id": row["object_id"],
                "confidence": row["confidence"],
                "timestamp_s": row["timestamp_s"],
                "evidence": json.loads(row["evidence"]) if row["evidence"] else None,
                "notes": row["notes"],
            }
            for row in cursor.fetchall()
        ]

    # Distance Operations

    def add_distance(
        self,
        entity_a_id: str,
        entity_b_id: str,
        distance_meters: float | None,
        distance_category: str | None,
        confidence: float,
        timestamp_s: float,
        method: str,
    ) -> None:
        conn = self._get_connection()
        cursor = conn.cursor()
        if entity_a_id > entity_b_id:
            entity_a_id, entity_b_id = entity_b_id, entity_a_id
        cursor.execute(
            """
            INSERT INTO distances (entity_a_id, entity_b_id, distance_meters, distance_category,
                                   confidence, timestamp_s, method)
            VALUES (?, ?, ?, ?, ?, ?, ?)
            """,
            (
                entity_a_id,
                entity_b_id,
                distance_meters,
                distance_category,
                confidence,
                timestamp_s,
                method,
            ),
        )
        conn.commit()

    def get_distance(self, entity_a_id: str, entity_b_id: str) -> dict[str, Any] | None:
        conn = self._get_connection()
        cursor = conn.cursor()
        if entity_a_id > entity_b_id:
            entity_a_id, entity_b_id = entity_b_id, entity_a_id
        cursor.execute(
            "SELECT * FROM distances WHERE entity_a_id = ? AND entity_b_id = ? ORDER BY timestamp_s DESC LIMIT 1",
            (entity_a_id, entity_b_id),
        )
        row = cursor.fetchone()
        if row is None:
            return None
        return {
            "entity_a_id": row["entity_a_id"],
            "entity_b_id": row["entity_b_id"],
            "distance_meters": row["distance_meters"],
            "distance_category": row["distance_category"],
            "confidence": row["confidence"],
            "timestamp_s": row["timestamp_s"],
            "method": row["method"],
        }

    def get_distance_history(self, entity_a_id: str, entity_b_id: str) -> list[dict[str, Any]]:
        conn = self._get_connection()
        cursor = conn.cursor()
        if entity_a_id > entity_b_id:
            entity_a_id, entity_b_id = entity_b_id, entity_a_id
        cursor.execute(
            "SELECT * FROM distances WHERE entity_a_id = ? AND entity_b_id = ? ORDER BY timestamp_s DESC",
            (entity_a_id, entity_b_id),
        )
        return [
            {
                "entity_a_id": row["entity_a_id"],
                "entity_b_id": row["entity_b_id"],
                "distance_meters": row["distance_meters"],
                "distance_category": row["distance_category"],
                "confidence": row["confidence"],
                "timestamp_s": row["timestamp_s"],
                "method": row["method"],
            }
            for row in cursor.fetchall()
        ]

    def get_nearby_entities(
        self, entity_id: str, max_distance: float, latest_only: bool = True
    ) -> list[dict[str, Any]]:
        conn = self._get_connection()
        cursor = conn.cursor()
        if latest_only:
            query = """
                SELECT d.*, e.entity_type, e.descriptor
                FROM distances d
                INNER JOIN entities e ON (
                    CASE
                        WHEN d.entity_a_id = ? THEN e.entity_id = d.entity_b_id
                        WHEN d.entity_b_id = ? THEN e.entity_id = d.entity_a_id
                    END
                )
                WHERE (d.entity_a_id = ? OR d.entity_b_id = ?)
                  AND d.distance_meters IS NOT NULL
                  AND d.distance_meters <= ?
                  AND d.id IN (
                      SELECT MAX(id) FROM distances
                      WHERE (entity_a_id = d.entity_a_id AND entity_b_id = d.entity_b_id)
                      GROUP BY entity_a_id, entity_b_id
                  )
                ORDER BY d.distance_meters ASC
            """
        else:
            query = """
                SELECT d.*, e.entity_type, e.descriptor
                FROM distances d
                INNER JOIN entities e ON (
                    CASE
                        WHEN d.entity_a_id = ? THEN e.entity_id = d.entity_b_id
                        WHEN d.entity_b_id = ? THEN e.entity_id = d.entity_a_id
                    END
                )
                WHERE (d.entity_a_id = ? OR d.entity_b_id = ?)
                  AND d.distance_meters IS NOT NULL
                  AND d.distance_meters <= ?
                ORDER BY d.distance_meters ASC
            """
        cursor.execute(query, (entity_id, entity_id, entity_id, entity_id, max_distance))
        return [
            {
                "entity_id": row["entity_b_id"]
                if row["entity_a_id"] == entity_id
                else row["entity_a_id"],
                "entity_type": row["entity_type"],
                "descriptor": row["descriptor"],
                "distance_meters": row["distance_meters"],
                "distance_category": row["distance_category"],
                "confidence": row["confidence"],
                "timestamp_s": row["timestamp_s"],
            }
            for row in cursor.fetchall()
        ]

    # Neighborhood Query

    def get_entity_neighborhood(
        self,
        entity_id: str,
        max_hops: int = 2,
        include_distances: bool = True,
    ) -> dict[str, Any]:
        visited_entities = {entity_id}
        current_level = {entity_id}
        all_relations: list[dict[str, Any]] = []
        all_distances: list[dict[str, Any]] = []

        for _ in range(max_hops):
            next_level: set[str] = set()
            for ent_id in current_level:
                relations = self.get_relations_for_entity(ent_id)
                all_relations.extend(relations)
                for rel in relations:
                    other_id = (
                        rel["object_id"] if rel["subject_id"] == ent_id else rel["subject_id"]
                    )
                    if other_id not in visited_entities:
                        next_level.add(other_id)
                        visited_entities.add(other_id)
                if include_distances:
                    distances = self.get_nearby_entities(ent_id, max_distance=10.0)
                    all_distances.extend(distances)
                    for dist in distances:
                        other_id = dist["entity_id"]
                        if other_id not in visited_entities:
                            next_level.add(other_id)
                            visited_entities.add(other_id)
            current_level = next_level
            if not current_level:
                break

        entities = [self.get_entity(eid) for eid in visited_entities]
        entities = [e for e in entities if e is not None]
        return {
            "center_entity": entity_id,
            "entities": entities,
            "relations": all_relations,
            "distances": all_distances,
            "num_hops": max_hops,
        }

    # Stats / Summary

    def get_stats(self) -> dict[str, Any]:
        conn = self._get_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT COUNT(*) as count FROM entities")
        entity_count = cursor.fetchone()["count"]
        cursor.execute("SELECT COUNT(*) as count FROM relations")
        relation_count = cursor.fetchone()["count"]
        cursor.execute("SELECT COUNT(*) as count FROM distances")
        distance_count = cursor.fetchone()["count"]
        return {"entities": entity_count, "relations": relation_count, "distances": distance_count}

    def get_summary(self, recent_relations_limit: int = 5) -> dict[str, Any]:
        return {
            "stats": self.get_stats(),
            "entities": self.get_all_entities(),
            "recent_relations": self.get_recent_relations(limit=recent_relations_limit),
        }

    # Bulk Save

    def save_window_data(
        self,
        parsed: dict[str, Any],
        timestamp_s: float,
        metadata: dict[str, Any] | None = None,
    ) -> None:
        """Save parsed window data (entities and relations) to the graph database."""
        try:
            for entity in parsed.get("new_entities", []):
                self.upsert_entity(
                    entity_id=entity["id"],
                    entity_type=entity["type"],
                    descriptor=entity.get("descriptor", "unknown"),
                    timestamp_s=timestamp_s,
                    metadata=metadata,
                )
            for entity in parsed.get("entities_present", []):
                if isinstance(entity, dict) and "id" in entity:
                    descriptor = entity.get("descriptor")
                    if descriptor:
                        self.upsert_entity(
                            entity_id=entity["id"],
                            entity_type=entity.get("type", "unknown"),
                            descriptor=descriptor,
                            timestamp_s=timestamp_s,
                            metadata=metadata,
                        )
                    else:
                        existing = self.get_entity(entity["id"])
                        if existing:
                            self.upsert_entity(
                                entity_id=entity["id"],
                                entity_type=existing["entity_type"],
                                descriptor=existing["descriptor"],
                                timestamp_s=timestamp_s,
                                metadata=metadata,
                            )
            for relation in parsed.get("relations", []):
                subject_id = (
                    relation["subject"].split("|")[0]
                    if "|" in relation["subject"]
                    else relation["subject"]
                )
                object_id = (
                    relation["object"].split("|")[0]
                    if "|" in relation["object"]
                    else relation["object"]
                )
                self.add_relation(
                    relation_type=relation["type"],
                    subject_id=subject_id,
                    object_id=object_id,
                    confidence=relation.get("confidence", 1.0),
                    timestamp_s=timestamp_s,
                    evidence=relation.get("evidence"),
                    notes=relation.get("notes"),
                )
        except Exception as e:
            logger.error(f"Failed to save window data to graph DB: {e}", exc_info=True)

    def estimate_and_save_distances(
        self,
        parsed: dict[str, Any],
        frame_image: Image,
        vlm: VlModel,
        timestamp_s: float,
        max_distance_pairs: int = 5,
    ) -> None:
        """Estimate distances between entities using VLM and save to database."""
        if not frame_image:
            return

        enriched_entities: list[dict[str, Any]] = []
        for entity in parsed.get("new_entities", []):
            if isinstance(entity, dict) and "id" in entity:
                enriched_entities.append(
                    {"id": entity["id"], "descriptor": entity.get("descriptor", "unknown")}
                )
        for entity in parsed.get("entities_present", []):
            if isinstance(entity, dict) and "id" in entity:
                db_entity = self.get_entity(entity["id"])
                if db_entity:
                    enriched_entities.append(
                        {"id": entity["id"], "descriptor": db_entity.get("descriptor", "unknown")}
                    )

        if len(enriched_entities) < 2:
            return

        pairs = [
            (enriched_entities[i], enriched_entities[j])
            for i in range(len(enriched_entities))
            for j in range(i + 1, len(enriched_entities))
            if not self.get_distance(enriched_entities[i]["id"], enriched_entities[j]["id"])
        ][:max_distance_pairs]

        if not pairs:
            return
        try:
            response = vlm.query(frame_image, build_batch_distance_estimation_prompt(pairs))
            for r in parse_batch_distance_response(response, pairs):
                if r["category"] in ("near", "medium", "far"):
                    self.add_distance(
                        entity_a_id=r["entity_a_id"],
                        entity_b_id=r["entity_b_id"],
                        distance_meters=r.get("distance_m"),
                        distance_category=r["category"],
                        confidence=r.get("confidence", 0.5),
                        timestamp_s=timestamp_s,
                        method="vlm",
                    )
        except Exception as e:
            logger.warning(f"Failed to estimate distances: {e}", exc_info=True)

    # Lifecycle

    def commit(self) -> None:
        if hasattr(self._local, "conn"):
            conn = self._local.conn
            conn.commit()
            try:
                conn.execute("PRAGMA wal_checkpoint(FULL)")
            except Exception:
                pass

    def close(self) -> None:
        if hasattr(self._local, "conn"):
            self._local.conn.close()
            del self._local.conn

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

Maintains three types of graphs:
1. Relations Graph: Interactions between entities (holds, looks_at, talks_to, etc.)
2. Distance Graph: Spatial distances between entities
3. Semantic Graph: Conceptual relationships (goes_with, part_of, used_for, etc.)

All graphs share the same entity nodes but have different edge types.
"""

import json
from pathlib import Path
import sqlite3
import threading
from typing import TYPE_CHECKING, Any

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.models.vl.base import VlModel
    from dimos.msgs.sensor_msgs import Image

logger = setup_logger()


class EntityGraphDB:
    """
    SQLite-based graph database for entity relationships.

    Thread-safe implementation using connection-per-thread pattern.
    All graphs share the same entity nodes but maintain separate edge tables.
    """

    def __init__(self, db_path: str | Path) -> None:
        """
        Initialize the entity graph database.

        Args:
            db_path: Path to the SQLite database file
        """
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)

        # Thread-local storage for connections
        self._local = threading.local()

        # Initialize schema
        self._init_schema()

        logger.info(f"EntityGraphDB initialized at {self.db_path}")

    def _get_connection(self) -> sqlite3.Connection:
        """Get thread-local database connection."""
        if not hasattr(self._local, "conn"):
            self._local.conn = sqlite3.connect(str(self.db_path))
            self._local.conn.row_factory = sqlite3.Row
        return self._local.conn  # type: ignore

    def _init_schema(self) -> None:
        """Initialize database schema."""
        conn = self._get_connection()
        cursor = conn.cursor()

        # Entities table (shared nodes)
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

        # Relations table (Graph 1: Interactions)
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

        # Distances table (Graph 2: Spatial)
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

        # Semantic relations table (Graph 3: Knowledge)
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS semantic_relations (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                relation_type TEXT NOT NULL,
                entity_a_id TEXT NOT NULL,
                entity_b_id TEXT NOT NULL,
                confidence REAL DEFAULT 1.0,
                learned_from TEXT,
                first_observed_ts REAL NOT NULL,
                last_observed_ts REAL NOT NULL,
                observation_count INTEGER DEFAULT 1,
                FOREIGN KEY (entity_a_id) REFERENCES entities(entity_id),
                FOREIGN KEY (entity_b_id) REFERENCES entities(entity_id)
            )
        """)
        cursor.execute(
            "CREATE INDEX IF NOT EXISTS idx_semantic_pair ON semantic_relations(entity_a_id, entity_b_id)"
        )
        cursor.execute(
            "CREATE INDEX IF NOT EXISTS idx_semantic_type ON semantic_relations(relation_type)"
        )

        conn.commit()

    def upsert_entity(
        self,
        entity_id: str,
        entity_type: str,
        descriptor: str,
        timestamp_s: float,
        metadata: dict[str, Any] | None = None,
    ) -> None:
        """
        Insert or update an entity.

        Args:
            entity_id: Unique entity identifier (e.g., "E1")
            entity_type: Type of entity (person, object, location, etc.)
            descriptor: Text description of the entity
            timestamp_s: Timestamp when entity was observed
            metadata: Optional additional metadata
        """
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
                metadata = COALESCE(excluded.metadata, metadata)
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
        logger.debug(f"Upserted entity {entity_id} (type={entity_type})")

    def get_entity(self, entity_id: str) -> dict[str, Any] | None:
        """
        Get an entity by ID.
        """
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
        """Get all entities, optionally filtered by type."""
        conn = self._get_connection()
        cursor = conn.cursor()

        if entity_type:
            cursor.execute(
                "SELECT * FROM entities WHERE entity_type = ? ORDER BY last_seen_ts DESC",
                (entity_type,),
            )
        else:
            cursor.execute("SELECT * FROM entities ORDER BY last_seen_ts DESC")

        rows = cursor.fetchall()
        return [
            {
                "entity_id": row["entity_id"],
                "entity_type": row["entity_type"],
                "descriptor": row["descriptor"],
                "first_seen_ts": row["first_seen_ts"],
                "last_seen_ts": row["last_seen_ts"],
                "metadata": json.loads(row["metadata"]) if row["metadata"] else None,
            }
            for row in rows
        ]

    def get_entities_by_time(
        self,
        time_window: tuple[float, float],
        first_seen: bool = True,
    ) -> list[dict[str, Any]]:
        """Get entities first/last seen within a time window.

        Args:
            time_window: (start_ts, end_ts) tuple in seconds
            first_seen: If True, filter by first_seen_ts. If False, filter by last_seen_ts.

        Returns:
            List of entities seen within the time window
        """
        conn = self._get_connection()
        cursor = conn.cursor()

        ts_field = "first_seen_ts" if first_seen else "last_seen_ts"
        cursor.execute(
            f"SELECT * FROM entities WHERE {ts_field} BETWEEN ? AND ? ORDER BY {ts_field} DESC",
            time_window,
        )

        rows = cursor.fetchall()
        return [
            {
                "entity_id": row["entity_id"],
                "entity_type": row["entity_type"],
                "descriptor": row["descriptor"],
                "first_seen_ts": row["first_seen_ts"],
                "last_seen_ts": row["last_seen_ts"],
                "metadata": json.loads(row["metadata"]) if row["metadata"] else None,
            }
            for row in rows
        ]

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
        """
        Add a relation between two entities.

        Args:
            relation_type: Type of relation (holds, looks_at, talks_to, etc.)
            subject_id: Subject entity ID
            object_id: Object entity ID
            confidence: Confidence score (0.0 to 1.0)
            timestamp_s: Timestamp when relation was observed
            evidence: Optional list of evidence strings
            notes: Optional notes
        """
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
        logger.debug(f"Added relation: {subject_id} --{relation_type}--> {object_id}")

    def get_relations_for_entity(
        self,
        entity_id: str,
        relation_type: str | None = None,
        time_window: tuple[float, float] | None = None,
    ) -> list[dict[str, Any]]:
        """
        Get all relations involving an entity.

        Args:
            entity_id: Entity ID
            relation_type: Optional filter by relation type
            time_window: Optional (start_ts, end_ts) tuple

        Returns:
            List of relation dicts
        """
        conn = self._get_connection()
        cursor = conn.cursor()

        query = """
            SELECT * FROM relations
            WHERE (subject_id = ? OR object_id = ?)
        """
        params: list[Any] = [entity_id, entity_id]

        if relation_type:
            query += " AND relation_type = ?"
            params.append(relation_type)

        if time_window:
            query += " AND timestamp_s BETWEEN ? AND ?"
            params.extend(time_window)

        query += " ORDER BY timestamp_s DESC"

        cursor.execute(query, params)
        rows = cursor.fetchall()

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
            for row in rows
        ]

    def get_recent_relations(self, limit: int = 50) -> list[dict[str, Any]]:
        """Get most recent relations."""
        conn = self._get_connection()
        cursor = conn.cursor()

        cursor.execute(
            """
            SELECT * FROM relations
            ORDER BY timestamp_s DESC
            LIMIT ?
        """,
            (limit,),
        )

        rows = cursor.fetchall()
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
            for row in rows
        ]

    # ==================== Distance Operations (Graph 2) ====================

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
        """
        Add distance measurement between two entities.

        Args:
            entity_a_id: First entity ID
            entity_b_id: Second entity ID
            distance_meters: Distance in meters (can be None if only categorical)
            distance_category: Category (near/medium/far)
            confidence: Confidence score
            timestamp_s: Timestamp of measurement
            method: Method used (vlm, depth_estimation, bbox)
        """
        conn = self._get_connection()
        cursor = conn.cursor()

        # Normalize order to avoid duplicates (store alphabetically)
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
        logger.debug(
            f"Added distance: {entity_a_id} <--> {entity_b_id}: {distance_meters}m ({distance_category})"
        )

    def get_distance(
        self,
        entity_a_id: str,
        entity_b_id: str,
    ) -> dict[str, Any] | None:
        """Get most recent distance between two entities.

        Args:
            entity_a_id: First entity ID
            entity_b_id: Second entity ID

        Returns:
            Distance dict or None
        """
        conn = self._get_connection()
        cursor = conn.cursor()

        # Normalize order
        if entity_a_id > entity_b_id:
            entity_a_id, entity_b_id = entity_b_id, entity_a_id

        cursor.execute(
            """
            SELECT * FROM distances
            WHERE entity_a_id = ? AND entity_b_id = ?
            ORDER BY timestamp_s DESC
            LIMIT 1
        """,
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

    def get_distance_history(
        self,
        entity_a_id: str,
        entity_b_id: str,
    ) -> list[dict[str, Any]]:
        """Get all distance measurements between two entities.

        Args:
            entity_a_id: First entity ID
            entity_b_id: Second entity ID

        Returns:
            List of distance dicts, most recent first
        """
        conn = self._get_connection()
        cursor = conn.cursor()

        # Normalize order
        if entity_a_id > entity_b_id:
            entity_a_id, entity_b_id = entity_b_id, entity_a_id

        cursor.execute(
            """
            SELECT * FROM distances
            WHERE entity_a_id = ? AND entity_b_id = ?
            ORDER BY timestamp_s DESC
        """,
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
        self,
        entity_id: str,
        max_distance: float,
        latest_only: bool = True,
    ) -> list[dict[str, Any]]:
        """
        Find entities within a distance threshold.

        Args:
            entity_id: Reference entity ID
            max_distance: Maximum distance in meters
            latest_only: If True, use only latest measurements

        Returns:
            List of nearby entities with distances
        """
        conn = self._get_connection()
        cursor = conn.cursor()

        if latest_only:
            # Get latest distance for each pair
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
            cursor.execute(query, (entity_id, entity_id, entity_id, entity_id, max_distance))
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

        rows = cursor.fetchall()
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
            for row in rows
        ]

    def add_semantic_relation(
        self,
        relation_type: str,
        entity_a_id: str,
        entity_b_id: str,
        confidence: float,
        learned_from: str,
        timestamp_s: float,
    ) -> None:
        """
        Add or update a semantic relation.

        Args:
            relation_type: Relation type (goes_with, opposite_of, part_of, used_for)
            entity_a_id: First entity ID
            entity_b_id: Second entity ID
            confidence: Confidence score
            learned_from: Source (llm, knowledge_base, observation)
            timestamp_s: Timestamp when learned
        """
        conn = self._get_connection()
        cursor = conn.cursor()

        # Normalize order for symmetric relations
        if entity_a_id > entity_b_id:
            entity_a_id, entity_b_id = entity_b_id, entity_a_id

        # Check if relation exists
        cursor.execute(
            """
            SELECT id, observation_count, confidence FROM semantic_relations
            WHERE relation_type = ? AND entity_a_id = ? AND entity_b_id = ?
        """,
            (relation_type, entity_a_id, entity_b_id),
        )

        existing = cursor.fetchone()

        if existing:
            # Update existing relation (increase confidence, increment count)
            new_count = existing["observation_count"] + 1
            new_confidence = min(
                1.0, existing["confidence"] + 0.1
            )  # Increase confidence with observations

            cursor.execute(
                """
                UPDATE semantic_relations
                SET last_observed_ts = ?,
                    observation_count = ?,
                    confidence = ?
                WHERE id = ?
            """,
                (timestamp_s, new_count, new_confidence, existing["id"]),
            )
        else:
            # Insert new relation
            cursor.execute(
                """
                INSERT INTO semantic_relations
                (relation_type, entity_a_id, entity_b_id, confidence, learned_from,
                 first_observed_ts, last_observed_ts, observation_count)
                VALUES (?, ?, ?, ?, ?, ?, ?, 1)
            """,
                (
                    relation_type,
                    entity_a_id,
                    entity_b_id,
                    confidence,
                    learned_from,
                    timestamp_s,
                    timestamp_s,
                ),
            )

        conn.commit()
        logger.debug(f"Added semantic relation: {entity_a_id} --{relation_type}--> {entity_b_id}")

    def get_semantic_relations(
        self,
        entity_id: str | None = None,
        relation_type: str | None = None,
    ) -> list[dict[str, Any]]:
        """
        Get semantic relations, optionally filtered.

        Args:
            entity_id: Optional filter by entity
            relation_type: Optional filter by relation type

        Returns:
            List of semantic relation dicts
        """
        conn = self._get_connection()
        cursor = conn.cursor()

        query = "SELECT * FROM semantic_relations WHERE 1=1"
        params: list[Any] = []

        if entity_id:
            query += " AND (entity_a_id = ? OR entity_b_id = ?)"
            params.extend([entity_id, entity_id])

        if relation_type:
            query += " AND relation_type = ?"
            params.append(relation_type)

        query += " ORDER BY confidence DESC, observation_count DESC"

        cursor.execute(query, params)
        rows = cursor.fetchall()

        return [
            {
                "id": row["id"],
                "relation_type": row["relation_type"],
                "entity_a_id": row["entity_a_id"],
                "entity_b_id": row["entity_b_id"],
                "confidence": row["confidence"],
                "learned_from": row["learned_from"],
                "first_observed_ts": row["first_observed_ts"],
                "last_observed_ts": row["last_observed_ts"],
                "observation_count": row["observation_count"],
            }
            for row in rows
        ]

    # querying

    def get_entity_neighborhood(
        self,
        entity_id: str,
        max_hops: int = 2,
        include_distances: bool = True,
        include_semantics: bool = True,
    ) -> dict[str, Any]:
        """
        Get entity neighborhood (BFS traversal).

        Args:
            entity_id: Starting entity ID
            max_hops: Maximum number of hops to traverse
            include_distances: Include distance graph
            include_semantics: Include semantic graph

        Returns:
            Dict with entities, relations, distances, and semantics
        """
        visited_entities = {entity_id}
        current_level = {entity_id}
        all_relations = []
        all_distances = []
        all_semantics = []

        for _ in range(max_hops):
            next_level = set()

            for ent_id in current_level:
                # Get relations
                relations = self.get_relations_for_entity(ent_id)
                all_relations.extend(relations)

                for rel in relations:
                    other_id = (
                        rel["object_id"] if rel["subject_id"] == ent_id else rel["subject_id"]
                    )
                    if other_id not in visited_entities:
                        next_level.add(other_id)
                        visited_entities.add(other_id)

                # Get distances
                if include_distances:
                    distances = self.get_nearby_entities(ent_id, max_distance=10.0)
                    all_distances.extend(distances)
                    for dist in distances:
                        other_id = dist["entity_id"]
                        if other_id not in visited_entities:
                            next_level.add(other_id)
                            visited_entities.add(other_id)

                # Get semantic relations
                if include_semantics:
                    semantics = self.get_semantic_relations(entity_id=ent_id)
                    all_semantics.extend(semantics)
                    for sem in semantics:
                        other_id = (
                            sem["entity_b_id"]
                            if sem["entity_a_id"] == ent_id
                            else sem["entity_a_id"]
                        )
                        if other_id not in visited_entities:
                            next_level.add(other_id)
                            visited_entities.add(other_id)

            current_level = next_level
            if not current_level:
                break

        # Get all entity details
        entities = [self.get_entity(ent_id) for ent_id in visited_entities]
        entities = [e for e in entities if e is not None]

        return {
            "center_entity": entity_id,
            "entities": entities,
            "relations": all_relations,
            "distances": all_distances,
            "semantic_relations": all_semantics,
            "num_hops": max_hops,
        }

    def get_stats(self) -> dict[str, Any]:
        """Get database statistics."""
        conn = self._get_connection()
        cursor = conn.cursor()

        cursor.execute("SELECT COUNT(*) as count FROM entities")
        entity_count = cursor.fetchone()["count"]

        cursor.execute("SELECT COUNT(*) as count FROM relations")
        relation_count = cursor.fetchone()["count"]

        cursor.execute("SELECT COUNT(*) as count FROM distances")
        distance_count = cursor.fetchone()["count"]

        cursor.execute("SELECT COUNT(*) as count FROM semantic_relations")
        semantic_count = cursor.fetchone()["count"]

        return {
            "entities": entity_count,
            "relations": relation_count,
            "distances": distance_count,
            "semantic_relations": semantic_count,
        }

    def get_summary(self, recent_relations_limit: int = 5) -> dict[str, Any]:
        """Get stats, all entities, and recent relations."""
        return {
            "stats": self.get_stats(),
            "entities": self.get_all_entities(),
            "recent_relations": self.get_recent_relations(limit=recent_relations_limit),
        }

    def save_window_data(self, parsed: dict[str, Any], timestamp_s: float) -> None:
        """Save parsed window data (entities and relations) to the graph database."""
        try:
            # Save new entities
            for entity in parsed.get("new_entities", []):
                self.upsert_entity(
                    entity_id=entity["id"],
                    entity_type=entity["type"],
                    descriptor=entity.get("descriptor", "unknown"),
                    timestamp_s=timestamp_s,
                )

            # Save existing entities (update last_seen)
            for entity in parsed.get("entities_present", []):
                if isinstance(entity, dict) and "id" in entity:
                    descriptor = entity.get("descriptor")
                    if descriptor:
                        self.upsert_entity(
                            entity_id=entity["id"],
                            entity_type=entity.get("type", "unknown"),
                            descriptor=descriptor,
                            timestamp_s=timestamp_s,
                        )
                    else:
                        existing = self.get_entity(entity["id"])
                        if existing:
                            self.upsert_entity(
                                entity_id=entity["id"],
                                entity_type=existing["entity_type"],
                                descriptor=existing["descriptor"],
                                timestamp_s=timestamp_s,
                            )

            # Save relations
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
        frame_image: "Image",
        vlm: "VlModel",
        timestamp_s: float,
        max_distance_pairs: int = 5,
    ) -> None:
        """Estimate distances between entities using VLM and save to database.

        Args:
            parsed: Parsed window data containing entities
            frame_image: Frame image to analyze
            vlm: VLM instance for distance estimation
            timestamp_s: Timestamp for the distance measurements
            max_distance_pairs: Maximum number of entity pairs to estimate
        """
        if not frame_image:
            return

        # Import here to avoid circular dependency
        from . import temporal_utils as tu

        # Collect entities with descriptors
        # new_entities have descriptors from VLM
        enriched_entities = []
        for entity in parsed.get("new_entities", []):
            if isinstance(entity, dict) and "id" in entity:
                enriched_entities.append(
                    {"id": entity["id"], "descriptor": entity.get("descriptor", "unknown")}
                )

        # entities_present only have IDs - need to fetch descriptors from DB
        for entity in parsed.get("entities_present", []):
            if isinstance(entity, dict) and "id" in entity:
                entity_id = entity["id"]
                # Fetch descriptor from DB
                db_entity = self.get_entity(entity_id)
                if db_entity:
                    enriched_entities.append(
                        {"id": entity_id, "descriptor": db_entity.get("descriptor", "unknown")}
                    )

        if len(enriched_entities) < 2:
            return

        # Generate pairs without existing distances
        pairs = [
            (enriched_entities[i], enriched_entities[j])
            for i in range(len(enriched_entities))
            for j in range(i + 1, len(enriched_entities))
            if not self.get_distance(enriched_entities[i]["id"], enriched_entities[j]["id"])
        ][:max_distance_pairs]

        if not pairs:
            return

        try:
            response = vlm.query(frame_image, tu.build_batch_distance_estimation_prompt(pairs))
            for r in tu.parse_batch_distance_response(response, pairs):
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

    def commit(self) -> None:
        """Commit all pending transactions and ensure data is flushed to disk."""
        if hasattr(self._local, "conn"):
            conn = self._local.conn
            conn.commit()
            # Force checkpoint to ensure WAL data is written to main database file
            try:
                conn.execute("PRAGMA wal_checkpoint(FULL)")
            except Exception:
                pass  # Ignore if WAL is not enabled

    def close(self) -> None:
        """Close database connection."""
        if hasattr(self._local, "conn"):
            self._local.conn.close()
            del self._local.conn

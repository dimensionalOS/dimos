# Copyright 2025 Dimensional Inc.
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
Spatial vector database for storing and querying images with XY locations.

This module extends the ChromaDB implementation to support storing images with
their XY locations and querying by location or image similarity.
"""

from typing import Any

import chromadb
import numpy as np

from dimos.agents_deprecated.memory.visual_memory import VisualMemory
from dimos.types.robot_location import RobotLocation
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SpatialVectorDB:
    """
    A vector database for storing and querying images mapped to X,Y,theta absolute locations for SpatialMemory.

    This class extends the ChromaDB implementation to support storing images with
    their absolute locations and querying by location, text, or image cosine semantic similarity.
    """

    def __init__(  # type: ignore[no-untyped-def]
        self,
        collection_name: str = "spatial_memory",
        chroma_client=None,
        visual_memory=None,
        embedding_provider=None,
    ) -> None:
        """
        Initialize the spatial vector database.

        Args:
            collection_name: Name of the vector database collection
            chroma_client: Optional ChromaDB client for persistence. If None, an in-memory client is used.
            visual_memory: Optional VisualMemory instance for storing images. If None, a new one is created.
            embedding_provider: Optional ImageEmbeddingProvider instance for computing embeddings. If None, one will be created.
        """
        self.collection_name = collection_name

        # Use provided client or create in-memory client
        self.client = chroma_client if chroma_client is not None else chromadb.Client()

        # Check if collection already exists - in newer ChromaDB versions list_collections returns names directly
        existing_collections = self.client.list_collections()

        # Handle different versions of ChromaDB API
        try:
            collection_exists = collection_name in existing_collections
        except:
            try:
                collection_exists = collection_name in [c.name for c in existing_collections]
            except:
                try:
                    self.client.get_collection(name=collection_name)
                    collection_exists = True
                except Exception:
                    collection_exists = False

        # Get or create the collection
        self.image_collection = self.client.get_or_create_collection(
            name=collection_name, metadata={"hnsw:space": "cosine"}
        )

        # Use provided visual memory or create a new one
        self.visual_memory = visual_memory if visual_memory is not None else VisualMemory()

        # Store the embedding provider to reuse for all operations
        self.embedding_provider = embedding_provider

        # Initialize the location collection for text-based location tagging
        location_collection_name = f"{collection_name}_locations"
        self.location_collection = self.client.get_or_create_collection(
            name=location_collection_name, metadata={"hnsw:space": "cosine"}
        )

        # Log initialization info with details about whether using existing collection
        client_type = "persistent" if chroma_client is not None else "in-memory"
        try:
            count = len(self.image_collection.get(include=[])["ids"])
            if collection_exists:
                logger.info(
                    f"Using EXISTING {client_type} collection '{collection_name}' with {count} entries"
                )
            else:
                logger.info(f"Created NEW {client_type} collection '{collection_name}'")
        except Exception as e:
            logger.info(
                f"Initialized {client_type} collection '{collection_name}' (count error: {e!s})"
            )

    def add_image_vector(
        self,
        vector_id: str,
        image: np.ndarray,  # type: ignore[type-arg]
        embedding: np.ndarray,  # type: ignore[type-arg]
        metadata: dict[str, Any],
    ) -> None:
        """
        Add an image with its embedding and metadata to the vector database.

        Args:
            vector_id: Unique identifier for the vector
            image: The image to store
            embedding: The pre-computed embedding vector for the image
            metadata: Metadata for the image, including x, y coordinates
        """
        # Store the image in visual memory
        self.visual_memory.add(vector_id, image)

        # Add the vector to ChromaDB
        self.image_collection.add(
            ids=[vector_id], embeddings=[embedding.tolist()], metadatas=[metadata]
        )

        logger.info(f"Added image vector {vector_id} with metadata: {metadata}")

    def query_by_embedding(self, embedding: np.ndarray, limit: int = 5) -> list[dict]:  # type: ignore[type-arg]
        """
        Query the vector database for images similar to the provided embedding.

        Args:
            embedding: Query embedding vector
            limit: Maximum number of results to return

        Returns:
            List of results, each containing the image and its metadata
        """
        results = self.image_collection.query(
            query_embeddings=[embedding.tolist()], n_results=limit
        )

        return self._process_query_results(results)

    # TODO: implement efficient nearest neighbor search
    def query_by_location(
        self, x: float, y: float, radius: float = 2.0, limit: int = 5
    ) -> list[dict]:  # type: ignore[type-arg]
        """
        Query the vector database for images near the specified location.

        Args:
            x: X coordinate
            y: Y coordinate
            radius: Search radius in meters
            limit: Maximum number of results to return

        Returns:
            List of results, each containing the image and its metadata
        """
        results = self.image_collection.get()

        if not results or not results["ids"]:
            return []

        filtered_results = {"ids": [], "metadatas": [], "distances": []}  # type: ignore[var-annotated]

        for i, metadata in enumerate(results["metadatas"]):  # type: ignore[arg-type]
            item_x = metadata.get("x")
            item_y = metadata.get("y")

            if item_x is not None and item_y is not None:
                distance = np.sqrt((x - item_x) ** 2 + (y - item_y) ** 2)

                if distance <= radius:
                    filtered_results["ids"].append(results["ids"][i])
                    filtered_results["metadatas"].append(metadata)
                    filtered_results["distances"].append(distance)

        sorted_indices = np.argsort(filtered_results["distances"])
        filtered_results["ids"] = [filtered_results["ids"][i] for i in sorted_indices[:limit]]
        filtered_results["metadatas"] = [
            filtered_results["metadatas"][i] for i in sorted_indices[:limit]
        ]
        filtered_results["distances"] = [
            filtered_results["distances"][i] for i in sorted_indices[:limit]
        ]

        return self._process_query_results(filtered_results)

    def _process_query_results(self, results) -> list[dict]:  # type: ignore[no-untyped-def, type-arg]
        """Process query results to include decoded images."""
        if not results or not results["ids"]:
            return []

        processed_results = []

        for i, vector_id in enumerate(results["ids"]):
            if isinstance(vector_id, list) and not vector_id:
                continue

            lookup_id = vector_id[0] if isinstance(vector_id, list) else vector_id

            # Create the result dictionary with metadata regardless of image availability
            result = {
                "metadata": results["metadatas"][i] if "metadatas" in results else {},
                "id": lookup_id,
            }

            # Add distance if available
            if "distances" in results:
                result["distance"] = (
                    results["distances"][i][0]
                    if isinstance(results["distances"][i], list)
                    else results["distances"][i]
                )

            # Get the image from visual memory
            image = self.visual_memory.get(lookup_id)
            result["image"] = image

            processed_results.append(result)

        return processed_results

    def query_by_text(self, text: str, limit: int = 5) -> list[dict]:  # type: ignore[type-arg]
        """
        Query the vector database for images matching the provided text description.

        This method uses CLIP's text-to-image matching capability to find images
        that semantically match the text query (e.g., "where is the kitchen").

        Args:
            text: Text query to search for
            limit: Maximum number of results to return

        Returns:
            List of results, each containing the image, its metadata, and similarity score
        """
        if self.embedding_provider is None:
            from dimos.agents_deprecated.memory.image_embedding import ImageEmbeddingProvider

            self.embedding_provider = ImageEmbeddingProvider(model_name="clip")

        text_embedding = self.embedding_provider.get_text_embedding(text)

        results = self.image_collection.query(
            query_embeddings=[text_embedding.tolist()],
            n_results=limit,
            include=["documents", "metadatas", "distances"],
        )

        logger.info(
            f"Text query: '{text}' returned {len(results['ids'] if 'ids' in results else [])} results"
        )
        return self._process_query_results(results)

    def get_all_locations(self) -> list[tuple[float, float, float]]:
        """Get all locations stored in the database."""
        # Get all items from the collection without embeddings
        results = self.image_collection.get(include=["metadatas"])

        if not results or "metadatas" not in results or not results["metadatas"]:
            return []

        # Extract x, y coordinates from metadata
        locations = []
        for metadata in results["metadatas"]:
            if isinstance(metadata, list) and metadata and isinstance(metadata[0], dict):
                metadata = metadata[0]  # Handle nested metadata

            if isinstance(metadata, dict) and "x" in metadata and "y" in metadata:
                x = metadata.get("x", 0)
                y = metadata.get("y", 0)
                z = metadata.get("z", 0) if "z" in metadata else 0
                locations.append((x, y, z))

        return locations

    @property
    def image_storage(self):  # type: ignore[no-untyped-def]
        """Legacy accessor for compatibility with existing code."""
        return self.visual_memory.images

    def tag_location(self, location: RobotLocation) -> None:
        """
        Tag a location with a semantic name/description for text-based retrieval.

        Args:
            location: RobotLocation object with position/rotation data
        """

        location_id = location.location_id
        metadata = location.to_vector_metadata()

        self.location_collection.add(
            ids=[location_id], documents=[location.name], metadatas=[metadata]
        )

    def query_tagged_location(self, query: str) -> tuple[RobotLocation | None, float]:
        """
        Query for a tagged location using semantic text search.

        Args:
            query: Natural language query (e.g., "dining area", "place to eat")

        Returns:
            The best matching RobotLocation or None if no matches found
        """

        results = self.location_collection.query(
            query_texts=[query], n_results=1, include=["metadatas", "documents", "distances"]
        )

        if not (results and results["ids"] and len(results["ids"][0]) > 0):
            return None, 0

        best_match_metadata = results["metadatas"][0][0]  # type: ignore[index]
        distance = float(results["distances"][0][0] if "distances" in results else 0.0)  # type: ignore[index]

        location = RobotLocation.from_vector_metadata(best_match_metadata)  # type: ignore[arg-type]

        logger.info(
            f"Found location '{location.name}' for query '{query}' (distance: {distance:.3f})"
            if distance
            else ""
        )

        return location, distance

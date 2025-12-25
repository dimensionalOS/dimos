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
Spatial Memory module for creating a semantic map of the environment.
"""

from datetime import datetime
import os
import time
from typing import TYPE_CHECKING, Any, Optional
import uuid

import cv2
import numpy as np
from reactivex import Observable, interval, operators as ops
from reactivex.disposable import Disposable

from dimos import spec
from dimos.agents_deprecated.memory.image_embedding import ImageEmbeddingProvider
from dimos.agents_deprecated.memory.spatial_vector_db import SpatialVectorDB
from dimos.agents_deprecated.memory.visual_memory import VisualMemory
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core import DimosCluster, In, Module, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.types.robot_location import RobotLocation
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs import Vector3

_OUTPUT_DIR = DIMOS_PROJECT_ROOT / "assets" / "output"
_MEMORY_DIR = _OUTPUT_DIR / "memory"
_SPATIAL_MEMORY_DIR = _MEMORY_DIR / "spatial_memory"
_DB_PATH = _SPATIAL_MEMORY_DIR / "chromadb_data"
_VISUAL_MEMORY_PATH = _SPATIAL_MEMORY_DIR / "visual_memory.pkl"


logger = setup_logger()


class SpatialMemory(Module):
    """
    A Dask module for building and querying Robot spatial memory.

    This module processes video frames and odometry data from LCM streams,
    associates them with XY locations, and stores them in a vector database
    for later retrieval via RPC calls. It also maintains a list of named
    robot locations that can be queried by name.
    """

    # LCM inputs
    color_image: In[Image]

    def __init__(
        self,
        collection_name: str = "spatial_memory",
        embedding_model: str = "clip",
        embedding_dimensions: int = 512,
        min_distance_threshold: float = 0.01,  # Min distance in meters to store a new frame
        min_time_threshold: float = 1.0,  # Min time in seconds to record a new frame
        db_path: str | None = str(_DB_PATH),  # Path for ChromaDB persistence
        visual_memory_path: str | None = str(
            _VISUAL_MEMORY_PATH
        ),  # Path for saving/loading visual memory
        new_memory: bool = True,  # Whether to create a new memory from scratch
        output_dir: str | None = str(
            _SPATIAL_MEMORY_DIR
        ),  # Directory for storing visual memory data
        chroma_client: Any = None,  # Optional ChromaDB client for persistence
        visual_memory: Optional[
            "VisualMemory"
        ] = None,  # Optional VisualMemory instance for storing images
    ) -> None:
        """
        Initialize the spatial perception system.

        Args:
            collection_name: Name of the vector database collection
            embedding_model: Model to use for image embeddings ("clip", "resnet", etc.)
            embedding_dimensions: Dimensions of the embedding vectors
            min_distance_threshold: Minimum distance in meters to record a new frame
            min_time_threshold: Minimum time in seconds to record a new frame
            chroma_client: Optional ChromaDB client for persistent storage
            visual_memory: Optional VisualMemory instance for storing images
            output_dir: Directory for storing visual memory data if visual_memory is not provided
        """
        self.collection_name = collection_name
        self.embedding_model = embedding_model
        self.embedding_dimensions = embedding_dimensions
        self.min_distance_threshold = min_distance_threshold
        self.min_time_threshold = min_time_threshold

        # Set up paths for persistence
        # Call parent Module init
        super().__init__()

        self.db_path = db_path
        self.visual_memory_path = visual_memory_path

        # Setup ChromaDB client if not provided
        self._chroma_client = chroma_client
        if chroma_client is None and db_path is not None:
            # Create db directory if needed
            os.makedirs(db_path, exist_ok=True)

            # Clean up existing DB if creating new memory
            if new_memory and os.path.exists(db_path):
                try:
                    logger.info("Creating new ChromaDB database (new_memory=True)")
                    # Try to delete any existing database files
                    import shutil

                    for item in os.listdir(db_path):
                        item_path = os.path.join(db_path, item)
                        if os.path.isfile(item_path):
                            os.unlink(item_path)
                        elif os.path.isdir(item_path):
                            shutil.rmtree(item_path)
                    logger.info(f"Removed existing ChromaDB files from {db_path}")
                except Exception as e:
                    logger.error(f"Error clearing ChromaDB directory: {e}")

            import chromadb
            from chromadb.config import Settings

            self._chroma_client = chromadb.PersistentClient(
                path=db_path, settings=Settings(anonymized_telemetry=False)
            )

        # Initialize or load visual memory
        self._visual_memory = visual_memory
        if visual_memory is None:
            if new_memory or not os.path.exists(visual_memory_path or ""):
                logger.info("Creating new visual memory")
                self._visual_memory = VisualMemory(output_dir=output_dir)
            else:
                try:
                    logger.info(f"Loading existing visual memory from {visual_memory_path}...")
                    self._visual_memory = VisualMemory.load(
                        visual_memory_path,  # type: ignore[arg-type]
                        output_dir=output_dir,
                    )
                    logger.info(f"Loaded {self._visual_memory.count()} images from previous runs")
                except Exception as e:
                    logger.error(f"Error loading visual memory: {e}")
                    self._visual_memory = VisualMemory(output_dir=output_dir)

        self.embedding_provider: ImageEmbeddingProvider = ImageEmbeddingProvider(
            model_name=embedding_model, dimensions=embedding_dimensions
        )

        self.vector_db: SpatialVectorDB = SpatialVectorDB(
            collection_name=collection_name,
            chroma_client=self._chroma_client,
            visual_memory=self._visual_memory,
            embedding_provider=self.embedding_provider,
        )

        self.last_position: Vector3 | None = None
        self.last_record_time: float | None = None

        self.frame_count: int = 0
        self.stored_frame_count: int = 0

        # List to store robot locations
        self.robot_locations: list[RobotLocation] = []

        # Track latest data for processing
        self._latest_video_frame: np.ndarray | None = None  # type: ignore[type-arg]
        self._process_interval = 1

        logger.info(f"SpatialMemory initialized with model {embedding_model}")

    @rpc
    def start(self) -> None:
        super().start()

        # Subscribe to LCM streams
        def set_video(image_msg: Image) -> None:
            # Convert Image message to numpy array
            if hasattr(image_msg, "data"):
                frame = image_msg.data
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                self._latest_video_frame = frame
            else:
                logger.warning("Received image message without data attribute")

        unsub = self.color_image.subscribe(set_video)
        self._disposables.add(Disposable(unsub))

        # Start periodic processing using interval
        unsub = interval(self._process_interval).subscribe(lambda _: self._process_frame())  # type: ignore[assignment]
        self._disposables.add(Disposable(unsub))

    @rpc
    def stop(self) -> None:
        # Save data before shutdown
        self.save()

        if self._visual_memory:
            self._visual_memory.clear()

        super().stop()

    def _process_frame(self) -> None:
        """Process the latest frame with pose data if available."""
        tf = self.tf.get("map", "base_link")
        if self._latest_video_frame is None or tf is None:
            return

        # Create Pose object with position and orientation
        current_pose = tf.to_pose()

        # Process the frame directly
        try:
            self.frame_count += 1

            # Check distance constraint
            if self.last_position is not None:
                distance_moved = np.linalg.norm(
                    [
                        current_pose.position.x - self.last_position.x,
                        current_pose.position.y - self.last_position.y,
                        current_pose.position.z - self.last_position.z,
                    ]
                )
                if distance_moved < self.min_distance_threshold:
                    logger.debug(
                        f"Position has not moved enough: {distance_moved:.4f}m < {self.min_distance_threshold}m, skipping frame"
                    )
                    return

            # Check time constraint
            if self.last_record_time is not None:
                time_elapsed = time.time() - self.last_record_time
                if time_elapsed < self.min_time_threshold:
                    logger.debug(
                        f"Time since last record too short: {time_elapsed:.2f}s < {self.min_time_threshold}s, skipping frame"
                    )
                    return

            current_time = time.time()

            # Get embedding for the frame
            frame_embedding = self.embedding_provider.get_embedding(self._latest_video_frame)

            frame_id = f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"
            # Get euler angles from quaternion orientation for metadata
            euler = tf.rotation.to_euler()

            # Create metadata dictionary with primitive types only
            metadata = {
                "pos_x": float(current_pose.position.x),
                "pos_y": float(current_pose.position.y),
                "pos_z": float(current_pose.position.z),
                "rot_x": float(euler.x),
                "rot_y": float(euler.y),
                "rot_z": float(euler.z),
                "timestamp": current_time,
                "frame_id": frame_id,
            }

            # Store in vector database
            self.vector_db.add_image_vector(
                vector_id=frame_id,
                image=self._latest_video_frame,
                embedding=frame_embedding,
                metadata=metadata,
            )

            # Update tracking variables
            self.last_position = current_pose.position
            self.last_record_time = current_time
            self.stored_frame_count += 1

            logger.info(
                f"Stored frame at position ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}, {current_pose.position.z:.2f}), "
                f"rotation ({euler.x:.2f}, {euler.y:.2f}, {euler.z:.2f}) "
                f"stored {self.stored_frame_count}/{self.frame_count} frames"
            )

            # Periodically save visual memory to disk
            if self._visual_memory is not None and self.visual_memory_path is not None:
                if self.stored_frame_count % 100 == 0:
                    self.save()

        except Exception as e:
            logger.error(f"Error processing frame: {e}")

    @rpc
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
        return self.vector_db.query_by_location(x, y, radius, limit)

    @rpc
    def save(self) -> bool:
        """
        Save the visual memory component to disk.

        Returns:
            True if memory was saved successfully, False otherwise
        """
        if self._visual_memory is not None and self.visual_memory_path is not None:
            try:
                saved_path = self._visual_memory.save(self.visual_memory_path)
                logger.info(f"Saved {self._visual_memory.count()} images to {saved_path}")
                return True
            except Exception as e:
                logger.error(f"Failed to save visual memory: {e}")
        return False

    def process_stream(self, combined_stream: Observable) -> Observable:  # type: ignore[type-arg]
        """
        Process a combined stream of video frames and positions.

        This method handles a stream where each item already contains both the frame and position,
        such as the stream created by combining video and transform streams with the
        with_latest_from operator.

        Args:
            combined_stream: Observable stream of dictionaries containing 'frame' and 'position'

        Returns:
            Observable of processing results, including the stored frame and its metadata
        """

        def process_combined_data(data):  # type: ignore[no-untyped-def]
            self.frame_count += 1

            frame = data.get("frame")
            position_vec = data.get("position")  # Use .get() for consistency
            rotation_vec = data.get("rotation")  # Get rotation data if available

            if position_vec is None or rotation_vec is None:
                logger.info("No position or rotation data available, skipping frame")
                return None

            # position_vec is already a Vector3, no need to recreate it
            position_v3 = position_vec

            if self.last_position is not None:
                distance_moved = np.linalg.norm(
                    [
                        position_v3.x - self.last_position.x,
                        position_v3.y - self.last_position.y,
                        position_v3.z - self.last_position.z,
                    ]
                )
                if distance_moved < self.min_distance_threshold:
                    logger.debug("Position has not moved, skipping frame")
                    return None

            if (
                self.last_record_time is not None
                and (time.time() - self.last_record_time) < self.min_time_threshold
            ):
                logger.debug("Time since last record too short, skipping frame")
                return None

            current_time = time.time()

            frame_embedding = self.embedding_provider.get_embedding(frame)

            frame_id = f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"

            # Create metadata dictionary with primitive types only
            metadata = {
                "pos_x": float(position_v3.x),
                "pos_y": float(position_v3.y),
                "pos_z": float(position_v3.z),
                "rot_x": float(rotation_vec.x),
                "rot_y": float(rotation_vec.y),
                "rot_z": float(rotation_vec.z),
                "timestamp": current_time,
                "frame_id": frame_id,
            }

            self.vector_db.add_image_vector(
                vector_id=frame_id, image=frame, embedding=frame_embedding, metadata=metadata
            )

            self.last_position = position_v3
            self.last_record_time = current_time
            self.stored_frame_count += 1

            logger.info(
                f"Stored frame at position ({position_v3.x:.2f}, {position_v3.y:.2f}, {position_v3.z:.2f}), "
                f"rotation ({rotation_vec.x:.2f}, {rotation_vec.y:.2f}, {rotation_vec.z:.2f}) "
                f"stored {self.stored_frame_count}/{self.frame_count} frames"
            )

            # Create return dictionary with primitive-compatible values
            return {
                "frame": frame,
                "position": (position_v3.x, position_v3.y, position_v3.z),
                "rotation": (rotation_vec.x, rotation_vec.y, rotation_vec.z),
                "frame_id": frame_id,
                "timestamp": current_time,
            }

        return combined_stream.pipe(
            ops.map(process_combined_data), ops.filter(lambda result: result is not None)
        )

    @rpc
    def query_by_image(self, image: np.ndarray, limit: int = 5) -> list[dict]:  # type: ignore[type-arg]
        """
        Query the vector database for images similar to the provided image.

        Args:
            image: Query image
            limit: Maximum number of results to return

        Returns:
            List of results, each containing the image and its metadata
        """
        embedding = self.embedding_provider.get_embedding(image)
        return self.vector_db.query_by_embedding(embedding, limit)

    @rpc
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
        logger.info(f"Querying spatial memory with text: '{text}'")
        return self.vector_db.query_by_text(text, limit)

    @rpc
    def add_robot_location(self, location: RobotLocation) -> bool:
        """
        Add a named robot location to spatial memory.

        Args:
            location: The RobotLocation object to add

        Returns:
            True if successfully added, False otherwise
        """
        try:
            # Add to our list of robot locations
            self.robot_locations.append(location)
            logger.info(f"Added robot location '{location.name}' at position {location.position}")
            return True

        except Exception as e:
            logger.error(f"Error adding robot location: {e}")
            return False

    @rpc
    def add_named_location(
        self,
        name: str,
        position: list[float] | None = None,
        rotation: list[float] | None = None,
        description: str | None = None,
    ) -> bool:
        """
        Add a named robot location to spatial memory using current or specified position.

        Args:
            name: Name of the location
            position: Optional position [x, y, z], uses current position if None
            rotation: Optional rotation [roll, pitch, yaw], uses current rotation if None
            description: Optional description of the location

        Returns:
            True if successfully added, False otherwise
        """
        tf = self.tf.get("map", "base_link")
        if not tf:
            logger.error("No position available for robot location")
            return False

        # Create RobotLocation object
        location = RobotLocation(  # type: ignore[call-arg]
            name=name,
            position=tf.translation,
            rotation=tf.rotation.to_euler(),
            description=description or f"Location: {name}",
            timestamp=time.time(),
        )

        return self.add_robot_location(location)  # type: ignore[no-any-return]

    @rpc
    def get_robot_locations(self) -> list[RobotLocation]:
        """
        Get all stored robot locations.

        Returns:
            List of RobotLocation objects
        """
        return self.robot_locations

    @rpc
    def find_robot_location(self, name: str) -> RobotLocation | None:
        """
        Find a robot location by name.

        Args:
            name: Name of the location to find

        Returns:
            RobotLocation object if found, None otherwise
        """
        # Simple search through our list of locations
        for location in self.robot_locations:
            if location.name.lower() == name.lower():
                return location

        return None

    @rpc
    def get_stats(self) -> dict[str, int]:
        """Get statistics about the spatial memory module.

        Returns:
            Dictionary containing:
                - frame_count: Total number of frames processed
                - stored_frame_count: Number of frames actually stored
        """
        return {"frame_count": self.frame_count, "stored_frame_count": self.stored_frame_count}

    @rpc
    def tag_location(self, robot_location: RobotLocation) -> bool:
        try:
            self.vector_db.tag_location(robot_location)
        except Exception:
            return False
        return True

    @rpc
    def query_tagged_location(self, query: str) -> RobotLocation | None:
        location, semantic_distance = self.vector_db.query_tagged_location(query)
        if semantic_distance < 0.3:
            return location
        return None


def deploy(  # type: ignore[no-untyped-def]
    dimos: DimosCluster,
    camera: spec.Camera,
):
    spatial_memory = dimos.deploy(SpatialMemory, db_path="/tmp/spatial_memory_db")  # type: ignore[attr-defined]
    spatial_memory.color_image.connect(camera.color_image)
    spatial_memory.start()
    return spatial_memory


spatial_memory = SpatialMemory.blueprint

__all__ = ["SpatialMemory", "deploy", "spatial_memory"]

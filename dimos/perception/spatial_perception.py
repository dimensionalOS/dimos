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

import logging
import uuid
import time
import os
import math
from typing import Dict, List, Tuple, Optional, Any, Union

import numpy as np
import cv2
from reactivex import Observable, disposable
from reactivex import operators as ops
from reactivex.subject import Subject
from reactivex.disposable import CompositeDisposable
from datetime import datetime

from dimos.utils.logging_config import setup_logger
from dimos.agents.memory.spatial_vector_db import SpatialVectorDB
from dimos.agents.memory.image_embedding import ImageEmbeddingProvider
from dimos.agents.memory.visual_memory import VisualMemory
from dimos.types.vector import Vector
from dimos.types.robot_location import RobotLocation

logger = setup_logger("dimos.perception.spatial_memory")

class SpatialMemory:
    """
    A class for building and querying Robot spatial memory.
    
    This class processes video frames from ROSControl, associates them with
    XY locations, and stores them in a vector database for later retrieval.
    It also maintains a list of named robot locations that can be queried by name.
    """
    
    def __init__(
        self,
        collection_name: str = "spatial_memory",
        embedding_model: str = "clip", 
        embedding_dimensions: int = 512,
        min_distance_threshold: float = 0.01,  # Min distance in meters to store a new frame
        min_time_threshold: float = 1.0,  # Min time in seconds to record a new frame
        db_path: Optional[str] = None,  # Path for ChromaDB persistence
        visual_memory_path: Optional[str] = None,  # Path for saving/loading visual memory
        new_memory: bool = False,  # Whether to create a new memory from scratch
        output_dir: Optional[str] = None,  # Directory for storing visual memory data
        chroma_client: Any = None,  # Optional ChromaDB client for persistence
        visual_memory: Optional['VisualMemory'] = None,  # Optional VisualMemory instance for storing images
        video_stream: Optional[Observable] = None,  # Video stream to process
        transform_provider: Optional[callable] = None,  # Function that returns position and rotation
    ):
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
        self.db_path = db_path
        self.visual_memory_path = visual_memory_path
        self.output_dir = output_dir
        
        # Setup ChromaDB client if not provided
        self._chroma_client = chroma_client
        if chroma_client is None and db_path is not None:
            # Create db directory if needed
            os.makedirs(db_path, exist_ok=True)
            
            # Clean up existing DB if creating new memory
            if new_memory and os.path.exists(db_path):
                try:
                    logger.info(f"Creating new ChromaDB database (new_memory=True)")
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
            
            from chromadb.config import Settings
            import chromadb
            self._chroma_client = chromadb.PersistentClient(
                path=db_path,
                settings=Settings(anonymized_telemetry=False)
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
                    self._visual_memory = VisualMemory.load(visual_memory_path, output_dir=output_dir)
                    logger.info(f"Loaded {self._visual_memory.count()} images from previous runs")
                except Exception as e:
                    logger.error(f"Error loading visual memory: {e}")
                    self._visual_memory = VisualMemory(output_dir=output_dir)
        
        # Initialize vector database
        self.vector_db: SpatialVectorDB = SpatialVectorDB(
            collection_name=collection_name,
            chroma_client=self._chroma_client,
            visual_memory=self._visual_memory
        )
        
        self.embedding_provider: ImageEmbeddingProvider = ImageEmbeddingProvider(
            model_name=embedding_model,
            dimensions=embedding_dimensions
        )
        
        self.last_position: Optional[Vector] = None
        self.last_record_time: Optional[float] = None
        
        self.frame_count: int = 0
        self.stored_frame_count: int = 0
        
        # For tracking stream subscription
        self._subscription = None
        
        # List to store robot locations
        self.robot_locations: List[RobotLocation] = []
        
        logger.info(f"SpatialMemory initialized with model {embedding_model}")
        
        # Start processing video stream if provided
        if video_stream is not None and transform_provider is not None:
            self.start_continuous_processing(video_stream, transform_provider)
    
    
    def query_by_location(self, x: float, y: float, radius: float = 2.0, limit: int = 5) -> List[Dict]:
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
    
    def start_continuous_processing(self, video_stream: Observable, transform_provider: callable) -> disposable.Disposable:
        """
        Start continuous processing of video frames from an Observable stream.
        
        Args:
            video_stream: Observable of video frames
            transform_provider: Callable that returns position and rotation for each frame
            
        Returns:
            Disposable subscription that can be used to stop processing
        """
        # Stop any existing subscription
        self.stop_continuous_processing()
        
        # Map each video frame to include transform data
        combined_stream = video_stream.pipe(
            ops.map(lambda video_frame: {
                "frame": video_frame,
                **transform_provider()
            })
        )
        
        # Process with spatial memory
        result_stream = self.process_stream(combined_stream)
        
        # Subscribe to the result stream
        self._subscription = result_stream.subscribe(
            on_next=self._on_frame_processed,
            on_error=lambda e: logger.error(f"Error in spatial memory stream: {e}"),
            on_completed=lambda: logger.info("Spatial memory stream completed")
        )
        
        logger.info("Continuous spatial memory processing started")
        return self._subscription
    
    def stop_continuous_processing(self) -> None:
        """
        Stop continuous processing of video frames.
        """
        if self._subscription is not None:
            try:
                self._subscription.dispose()
                self._subscription = None
                logger.info("Stopped continuous spatial memory processing")
            except Exception as e:
                logger.error(f"Error stopping spatial memory processing: {e}")
    
    def _on_frame_processed(self, result: Dict[str, Any]) -> None:
        """
        Handle updates from the spatial memory processing stream.
        """
        # Log successful frame storage (if stored)
        position = result.get('position')
        if position is not None:
            logger.debug(f"Spatial memory updated with frame at ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})")
        
        # Periodically save visual memory to disk (e.g., every 100 frames)
        if self._visual_memory is not None and self.visual_memory_path is not None:
            if self.stored_frame_count % 100 == 0:
                self.save()
    
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
    
    def process_stream(self, combined_stream: Observable) -> Observable:
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
        self.last_position = None
        self.last_record_time = None
        
        def process_combined_data(data):
            self.frame_count += 1
            
            frame = data.get('frame')
            position_vec = data.get('position')  # Use .get() for consistency
            rotation_vec = data.get('rotation')  # Get rotation data if available
            
            
            if not position_vec or not rotation_vec:
                logger.info("No position or rotation data available, skipping frame")
                return None
            
            if self.last_position is not None and (self.last_position - position_vec).length() < self.min_distance_threshold:
                logger.debug("Position has not moved, skipping frame")
                return None

            if self.last_record_time is not None and (time.time() - self.last_record_time) < self.min_time_threshold:
                logger.debug("Time since last record too short, skipping frame")
                return None
            
            current_time = time.time()
                        
            frame_embedding = self.embedding_provider.get_embedding(frame)
            
            frame_id = f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"
            
            # Create metadata dictionary with primitive types only
            metadata = {
                "pos_x": float(position_vec.x),
                "pos_y": float(position_vec.y),
                "pos_z": float(position_vec.z),
                "rot_x": float(rotation_vec.x),
                "rot_y": float(rotation_vec.y),
                "rot_z": float(rotation_vec.z),
                "timestamp": current_time,
                "frame_id": frame_id
            }
            
            self.vector_db.add_image_vector(
                vector_id=frame_id,
                image=frame,
                embedding=frame_embedding,
                metadata=metadata
            )
            
            self.last_position = position_vec
            self.last_record_time = current_time
            self.stored_frame_count += 1
            
            logger.info(f"Stored frame at position {position_vec}, rotation {rotation_vec})"
                        f" stored {self.stored_frame_count}/{self.frame_count} frames")
            
            # Create return dictionary with primitive-compatible values
            return {
                "frame": frame,
                "position": (position_vec.x, position_vec.y, position_vec.z),
                "rotation": (rotation_vec.x, rotation_vec.y, rotation_vec.z),
                "frame_id": frame_id,
                "timestamp": current_time
            }
                    
        return combined_stream.pipe(
            ops.map(process_combined_data),
            ops.filter(lambda result: result is not None)
        )

    def query_by_image(self, image: np.ndarray, limit: int = 5) -> List[Dict]:
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
    
    def query_by_text(self, text: str, limit: int = 5) -> List[Dict]:
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
    
    def get_robot_locations(self) -> List[RobotLocation]:
        """
        Get all stored robot locations.
        
        Returns:
            List of RobotLocation objects
        """
        return self.robot_locations
    
    def find_robot_location(self, name: str) -> Optional[RobotLocation]:
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
    
    def cleanup(self):
        """Clean up resources."""
        # Stop any ongoing processing
        self.stop_continuous_processing()
        
        # Save data if possible
        self.save()
        
        # Log cleanup
        if self.vector_db:
            logger.info(f"Cleaning up SpatialMemory, stored {self.stored_frame_count} frames")

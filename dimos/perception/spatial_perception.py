#
#
#

"""
Spatial perception module for creating a semantic map of the environment.

This module implements the approach described in "Semantic Spatial Perception for Embodied Agents"
(https://arxiv.org/pdf/2410.20666v1) to build a vectorDB of images tagged with XY locations.
"""

import logging
import uuid
import numpy as np
import cv2
from typing import Dict, List, Tuple, Optional, Any
from reactivex import Observable
from reactivex import operators as ops
import time
from datetime import datetime

from dimos.utils.logging_config import setup_logger
from dimos.agents.memory.spatial_vector_db import SpatialVectorDB
from dimos.agents.memory.image_embedding import ImageEmbeddingProvider

logger = setup_logger("dimos.perception.spatial_memory")

class SpatialMemory:
    """
    A class for building and querying Robot spatial memory.
    
    This class processes video frames from ROSControl, associates them with
    XY locations, and stores them in a vector database for later retrieval.
    """
    
    def __init__(
        self,
        collection_name: str = "spatial_memory",
        embedding_model: str = "clip", 
        embedding_dimensions: int = 512,
        min_distance_threshold: float = 0.5,  # Min distance in meters to store a new frame
        min_time_threshold: float = 2.0,  # Min time in seconds to store a new frame
    ):
        """
        Initialize the spatial perception system.
        
        Args:
            collection_name: Name of the vector database collection
            embedding_model: Model to use for image embeddings ("clip", "resnet", etc.)
            embedding_dimensions: Dimensions of the embedding vectors
            min_distance_threshold: Minimum distance in meters to record a new frame
            min_time_threshold: Minimum time in seconds to record a new frame
        """
        self.collection_name = collection_name
        self.embedding_model = embedding_model
        self.embedding_dimensions = embedding_dimensions
        self.min_distance_threshold = min_distance_threshold
        self.min_time_threshold = min_time_threshold
        
        self.vector_db = SpatialVectorDB(collection_name=collection_name)
        
        self.embedding_provider = ImageEmbeddingProvider(
            model_name=embedding_model,
            dimensions=embedding_dimensions
        )
        
        self.last_position: Optional[Tuple[float, float]] = None
        self.last_record_time: Optional[float] = None
        
        self.frame_count = 0
        self.stored_frame_count = 0
        
        logger.info(f"SpatialMemory initialized with model {embedding_model}")
    
    def process_video_stream(self, video_stream: Observable, position_stream: Observable) -> Observable:
        """
        Process video frames and position updates, storing frames in the vector database.
        
        Args:
            video_stream: Observable stream of video frames
            position_stream: Observable stream of position updates (x, y coordinates)
            
        Returns:
            Observable of processing results, including the stored frame and its metadata
        """
        self.current_position: Optional[Tuple[float, float]] = None
        
        def on_position(position: Tuple[float, float]):
            self.current_position = position
            logger.debug(f"Position updated: ({position[0]:.2f}, {position[1]:.2f})")
        
        position_stream.subscribe(on_position)
        
        def process_frame(frame):
            self.frame_count += 1
            
            if self.current_position is None:
                logger.debug("No position data available yet, skipping frame")
                return None
            
            current_time = time.time()
            x, y = self.current_position
            
            should_store = False
            
            if self.last_position is None or self.last_record_time is None:
                should_store = True
            else:
                last_x, last_y = self.last_position
                distance = np.sqrt((x - last_x)**2 + (y - last_y)**2)
                time_diff = current_time - self.last_record_time
                
                if (distance >= self.min_distance_threshold or 
                    time_diff >= self.min_time_threshold):
                    should_store = True
            
            if should_store:
                frame_embedding = self.embedding_provider.get_embedding(frame)
                
                frame_id = f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"
                
                metadata = {
                    "x": float(x),
                    "y": float(y),
                    "timestamp": current_time,
                    "frame_id": frame_id
                }
                
                self.vector_db.add_image_vector(
                    vector_id=frame_id,
                    image=frame,
                    embedding=frame_embedding,
                    metadata=metadata
                )
                
                self.last_position = (x, y)
                self.last_record_time = current_time
                self.stored_frame_count += 1
                
                logger.info(f"Stored frame at position ({x:.2f}, {y:.2f}), "
                            f"stored {self.stored_frame_count}/{self.frame_count} frames")
                
                return {
                    "frame": frame,
                    "position": (x, y),
                    "frame_id": frame_id,
                    "timestamp": current_time
                }
            
            return None
        
        return video_stream.pipe(
            ops.map(process_frame),
            ops.filter(lambda result: result is not None)
        )
    
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
            
            frame = data['frame']
            position = data['position']
            
            if not position:
                logger.debug("No position data available, skipping frame")
                return None
                
            current_time = time.time()
            x, y, z = position  # Extract all three dimensions from the transform
            
            should_store = False
            
            if self.last_position is None or self.last_record_time is None:
                should_store = True
            else:
                last_x, last_y, *_ = self.last_position  # Handle both 2D and 3D positions
                distance = np.sqrt((x - last_x)**2 + (y - last_y)**2)
                time_diff = current_time - self.last_record_time
                
                if (distance >= self.min_distance_threshold or 
                    time_diff >= self.min_time_threshold):
                    should_store = True
            
            if should_store:
                frame_embedding = self.embedding_provider.get_embedding(frame)
                
                frame_id = f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"
                
                metadata = {
                    "x": float(x),
                    "y": float(y),
                    "z": float(z),
                    "timestamp": current_time,
                    "frame_id": frame_id
                }
                
                self.vector_db.add_image_vector(
                    vector_id=frame_id,
                    image=frame,
                    embedding=frame_embedding,
                    metadata=metadata
                )
                
                self.last_position = (x, y, z)
                self.last_record_time = current_time
                self.stored_frame_count += 1
                
                logger.info(f"Stored frame at position ({x:.2f}, {y:.2f}, {z:.2f}), "
                            f"stored {self.stored_frame_count}/{self.frame_count} frames")
                
                return {
                    "frame": frame,
                    "position": (x, y, z),
                    "frame_id": frame_id,
                    "timestamp": current_time
                }
            
            return None
        
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
    
    def cleanup(self):
        """Clean up resources."""
        if self.vector_db:
            logger.info(f"Cleaning up SpatialMemory, stored {self.stored_frame_count} frames")

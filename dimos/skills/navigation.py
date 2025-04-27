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
Semantic map skills for building and navigating spatial memory maps.

This module provides two skills:
1. BuildSemanticMap - Builds a semantic map by recording video frames at different locations
2. Navigate - Queries an existing semantic map using natural language
"""

import os
import sys
import time
import threading
import logging
import numpy as np
import json
from typing import Optional, Dict, Tuple, Any
from dimos.utils.threadpool import get_scheduler

import chromadb
import reactivex
from reactivex import operators as ops
from pydantic import Field

from dimos.skills.skills import AbstractRobotSkill
from dimos.perception.spatial_perception import SpatialMemory
from dimos.agents.memory.visual_memory import VisualMemory
from dimos.utils.threadpool import get_scheduler
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.skills.semantic_map_skills")

class BuildSemanticMap(AbstractRobotSkill):
    """
    A skill that builds a semantic map of the environment by recording video frames
    at different locations as the robot moves.
    
    This skill records video frames and their associated positions, storing them in
    a vector database for later querying. It runs until terminated (Ctrl+C).
    """
    
    db_path: str = Field("/home/stash/dimensional/dimos/assets/spatial_memory/chromadb_data", 
                        description="Path to store the ChromaDB database")
    collection_name: str = Field("spatial_memory", 
                                description="Name of the collection in the ChromaDB database")
    min_distance_threshold: float = Field(1.0, 
                                        description="Min distance in meters to record a new frame")
    min_time_threshold: float = Field(1.0, 
                                    description="Min time in seconds to record a new frame")
    visual_memory_dir: str = Field("/home/stash/dimensional/dimos/assets/spatial_memory", 
                                 description="Directory to store visual memory data")
    visual_memory_file: str = Field("visual_memory.pkl", 
                                   description="Filename for visual memory storage")
    new_map: bool = Field(False,
                        description="If True, creates new spatial and visual memory from scratch instead of using existing.")

    def __init__(self, robot=None, **data):
        """
        Initialize the BuildSemanticMap skill.
        
        Args:
            robot: The robot instance
            **data: Additional data for configuration
        """
        super().__init__(robot=robot, **data)
        self._stop_event = threading.Event()
        self._subscription = None
        self._scheduler = get_scheduler()
        self._stored_count = 0
        
    def __call__(self):
        """
        Start building a semantic map.
        
        Returns:
            A message indicating whether the map building started successfully
        """
        super().__call__()
        
        if self._robot is None:
            error_msg = "No robot instance provided to BuildSemanticMap skill"
            logger.error(error_msg)
            return error_msg
        
        self.stop()  # Stop any existing execution
        self._stop_event.clear()
        self._stored_count = 0
        
        # Setup output directory for visual memory
        os.makedirs(self.visual_memory_dir, exist_ok=True)
        
        # Setup persistent storage path for visual memory
        visual_memory_path = os.path.join(self.visual_memory_dir, self.visual_memory_file)
        
        # Create a new visual memory or try to load existing one
        if self.new_map:
            logger.info("Creating new visual memory as requested (new_map=True)")
            visual_memory = VisualMemory(output_dir=self.visual_memory_dir)
        elif os.path.exists(visual_memory_path):
            try:
                logger.info(f"Loading existing visual memory from {visual_memory_path}...")
                visual_memory = VisualMemory.load(visual_memory_path, output_dir=self.visual_memory_dir)
                logger.info(f"Loaded {visual_memory.count()} images from previous runs")
            except Exception as e:
                logger.error(f"Error loading visual memory: {e}")
                visual_memory = VisualMemory(output_dir=self.visual_memory_dir)
        else:
            logger.info("No existing visual memory found. Starting with empty visual memory.")
            visual_memory = VisualMemory(output_dir=self.visual_memory_dir)
        
        # Setup a persistent database for ChromaDB
        db_client = self._setup_persistent_chroma_db(new_map=self.new_map)
        
        # Get the ros_control instance from the robot
        ros_control = self._robot.ros_control
        
        # Create spatial memory instance with persistent storage
        logger.info("Creating SpatialMemory with persistent vector database...")
        spatial_memory = SpatialMemory(
            collection_name=self.collection_name,
            min_distance_threshold=self.min_distance_threshold,
            min_time_threshold=self.min_time_threshold,
            chroma_client=db_client,
            visual_memory=visual_memory
        )
        
        logger.info("Setting up video stream...")
        video_stream = self._robot.get_ros_video_stream()
        
        # Setup video stream processing with transform acquisition
        logger.info("Setting up video stream with position mapping...")
        
        # Map each video frame to include both position and rotation from transform
        combined_stream = video_stream.pipe(
            ops.map(lambda video_frame: {
                "frame": video_frame,
                **self._extract_transform_data(*ros_control.transform_euler("base_link"))
            })
        )
        
        # Process with spatial memory
        result_stream = spatial_memory.process_stream(combined_stream)
        
        # Subscribe to the result stream
        logger.info("Subscribing to spatial perception results...")
        self._subscription = result_stream.subscribe(
            on_next=self._on_stored_frame,
            on_error=lambda e: logger.error(f"Error in spatial memory stream: {e}"),
            on_completed=lambda: logger.info("Spatial memory stream completed")
        )
        
        # Store the spatial memory instance for later cleanup
        self._spatial_memory = spatial_memory
        self._visual_memory = visual_memory
        
        skill_library = self._robot.get_skills()
        self.register_as_running("BuildSemanticMap", skill_library, self._subscription)
        
        logger.info(f"BuildSemanticMap started with min_distance={self.min_distance_threshold}m, "
                 f"min_time={self.min_time_threshold}s")
        return (f"BuildSemanticMap started. Recording frames with min_distance={self.min_distance_threshold}m, "
                f"min_time={self.min_time_threshold}s. Press Ctrl+C to stop.")
    
    def _setup_persistent_chroma_db(self, new_map=False):
        """
        Set up a persistent ChromaDB database at the specified path.
        
        Args:
            new_map: If True, deletes existing database and creates a new one
            
        Returns:
            The ChromaDB client instance
        """
        logger.info(f"Setting up persistent ChromaDB at: {self.db_path}")
        
        # Ensure the directory exists
        os.makedirs(self.db_path, exist_ok=True)
        
        # If new_map is True, remove the existing ChromaDB files
        if new_map:
            try:
                logger.info(f"Creating new ChromaDB database (new_map=True)")
                # Try to delete any existing database files
                import shutil
                for item in os.listdir(self.db_path):
                    item_path = os.path.join(self.db_path, item)
                    if os.path.isfile(item_path):
                        os.unlink(item_path)
                    elif os.path.isdir(item_path):
                        shutil.rmtree(item_path)
                logger.info(f"Removed existing ChromaDB files from {self.db_path}")
            except Exception as e:
                logger.error(f"Error clearing ChromaDB directory: {e}")
        
        return chromadb.PersistentClient(path=self.db_path)

    def _extract_transform_data(self, position, rotation):
        """
        Extract both position and rotation data from a transform message in a single operation.
        
        Args:
            position: The position message
            rotation: The rotation message
            
        Returns:
            A dictionary containing:
            - 'position': tuple of (x, y, z) coordinates
            - 'rotation': the quaternion object for complete orientation data
        """
        if position is None or rotation is None:
            return {
                "position": None,
                "rotation": None
            }

        return {
            "position": position,
            "rotation": rotation
        }
    
    def _on_stored_frame(self, result):
        """
        Callback for when a frame is stored in the vector database.
        
        Args:
            result: The result of storing the frame
        """
        # Only count actually stored frames (not debug frames)
        if not result.get('stored', True) == False:
            self._stored_count += 1
            pos = result['position']
            logger.info(f"Stored frame #{self._stored_count} at ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
    
    def stop(self):
        """
        Stop building the semantic map.
        
        Returns:
            A message indicating whether the map building was stopped successfully
        """
        if self._subscription is not None and not self._subscription.is_disposed:
            logger.info("Stopping BuildSemanticMap")
            self._stop_event.set()
            self._subscription.dispose()
            self._subscription = None
            
            # Save visual memory to disk for later use
            if hasattr(self, '_visual_memory') and self._visual_memory is not None:
                saved_path = self._visual_memory.save(self.visual_memory_file)
                logger.info(f"Saved {self._visual_memory.count()} images to disk at {saved_path}")
                self._visual_memory = None
            
            # Clean up spatial memory
            if hasattr(self, '_spatial_memory') and self._spatial_memory is not None:
                self._spatial_memory.cleanup()
                self._spatial_memory = None
                        
            return f"BuildSemanticMap stopped. Stored {self._stored_count} frames."
        return "BuildSemanticMap was not running."


class Navigate(AbstractRobotSkill):
    """
    A skill that queries an existing semantic map using natural language.
    
    This skill takes a text query and returns the XY coordinates of the best match
    in the semantic map. For example, "Find the kitchen" will return the coordinates
    where the kitchen was observed.
    """
    
    query: str = Field("", description="Text query to search for in the semantic map")
    db_path: str = Field("/home/stash/dimensional/dimos/assets/spatial_memory/chromadb_data",
                        description="Path to the ChromaDB database")
    collection_name: str = Field("spatial_memory",
                                description="Name of the collection in the ChromaDB database")
    visual_memory_path: str = Field("/home/stash/dimensional/dimos/assets/spatial_memory/visual_memory.pkl",
                                   description="Path to the visual memory file")
    limit: int = Field(1, description="Maximum number of results to return")
    
    def __init__(self, robot=None, **data):
        """
        Initialize the Navigate skill.
        
        Args:
            robot: The robot instance
            **data: Additional data for configuration
        """
        super().__init__(robot=robot, **data)
        self._stop_event = threading.Event()
        self._spatial_memory = None
        self._scheduler = get_scheduler()  # Use the shared DiMOS thread pool
        self._navigation_disposable = None  # Disposable returned by scheduler.schedule()
    
    def __call__(self):
        """
        Query the semantic map with the provided text query.
        
        Returns:
            A dictionary with the best matching position and query details
        """
        super().__call__()
        
        if not self.query:
            error_msg = "No query provided to Navigate skill"
            logger.error(error_msg)
            return error_msg
        
        logger.info(f"Querying semantic map for: '{self.query}'")
        
        # Setup the persistent ChromaDB client
        db_client = self._setup_persistent_chroma_db()
        
        # Setup output directory for any saved results
        output_dir = os.path.dirname(self.visual_memory_path)
        
        # Load the visual memory
        logger.info(f"Loading visual memory from {self.visual_memory_path}...")
        if os.path.exists(self.visual_memory_path):
            visual_memory = VisualMemory.load(self.visual_memory_path, output_dir=output_dir)
            logger.info(f"Loaded {visual_memory.count()} images from visual memory")
        else:
            visual_memory = VisualMemory(output_dir=output_dir)
            logger.warning("No existing visual memory found. Query results won't include images.")
        
        # Create SpatialMemory with the existing database and visual memory
        spatial_memory = SpatialMemory(
            collection_name=self.collection_name,
            chroma_client=db_client,
            visual_memory=visual_memory
        )
        
        # Run the query
        results = spatial_memory.query_by_text(self.query, limit=self.limit)
        
        if not results:
            logger.warning(f"No results found for query: '{self.query}'")
            return {
                "success": False,
                "query": self.query,
                "error": "No matching location found"
            }
        
        # Get the best match
        best_match = results[0]
        metadata = best_match.get('metadata', {})
        
        if isinstance(metadata, list) and metadata:
            metadata = metadata[0]
        
        # Extract coordinates from metadata
        if isinstance(metadata, dict) and 'x' in metadata and 'y' in metadata:
            pos = metadata.get('position', (0, 0, 0))
            rot = metadata.get('rotation', (0, 0, 0))
            x, y, _ = pos
            theta = rot[2]
            
            # Calculate similarity score (distance is inverse of similarity)
            similarity = 1.0 - (best_match.get('distance', 0) if best_match.get('distance') is not None else 0)
            
            logger.info(f"Found match for '{self.query}' at ({x:.2f}, {y:.2f}, {z:.2f}) with similarity: {similarity:.4f}")
            

            logger.info(f"Starting navigation to position: ({x:.2f}, {y:.2f})")

            # Reset the stop event before starting navigation
            self._stop_event.clear()
            
            # The scheduler approach isn't working, switch to direct threading
            # Define a navigation function that will run on a separate thread
            def run_navigation():
                try:
                    logger.info(f"Starting navigation to ({x:.2f}, {y:.2f})")
                    # Pass our stop_event to allow cancellation
                    result = False
                    try:
                        result = self._robot.global_planner.set_goal((x, y), goal_theta = theta, stop_event=self._stop_event)
                    except Exception as e:
                        logger.error(f"Error calling global_planner.set_goal: {e}")
                        
                    if result:
                        logger.info("Navigation completed successfully")
                    else:
                        logger.error("Navigation did not complete successfully")
                    return result
                except Exception as e:
                    logger.error(f"Unexpected error in navigation thread: {e}")
                    return False
            
            # Cancel any existing navigation before starting a new one
            # Signal stop to any running navigation
            self._stop_event.set()
            # Clear stop event for new navigation
            self._stop_event.clear()
            
            # Create and start direct thread instead of using scheduler
            logger.info("Creating direct navigation thread")
            nav_thread = threading.Thread(target=run_navigation, daemon=True)
            logger.info("Starting direct navigation thread")
            nav_thread.start()
            logger.info("Direct navigation thread started successfully")

            return {
                "success": True,
                "query": self.query,
                "position": (x, y, z),
                "similarity": similarity,
                "metadata": metadata
            }
        else:
            logger.warning(f"No valid position data found for query: '{self.query}'")
            return {
                "success": False,
                "query": self.query,
                "error": "No valid position data found"
            }
    
    def _setup_persistent_chroma_db(self):
        """
        Set up a persistent ChromaDB database at the specified path.
            
        Returns:
            The ChromaDB client instance
        """
        logger.info(f"Setting up persistent ChromaDB at: {self.db_path}")
        
        # Ensure the directory exists
        os.makedirs(self.db_path, exist_ok=True)
        
        return chromadb.PersistentClient(path=self.db_path)

    def stop(self):
        """
        Stop the navigation skill and clean up resources.
        
        Returns:
            A message indicating whether the navigation was stopped successfully
        """
        logger.info("Stopping Navigate skill")
        
        # Signal any running processes to stop via the shared event
        self._stop_event.set()
        
        # Dispose of any existing navigation task
        if hasattr(self, '_navigation_disposable') and self._navigation_disposable:
            logger.info("Disposing navigation task")
            try:
                self._navigation_disposable.dispose()
            except Exception as e:
                logger.error(f"Error disposing navigation task: {e}")
            self._navigation_disposable = None
        
        # Clean up spatial memory if it exists
        if hasattr(self, '_spatial_memory') and self._spatial_memory is not None:
            logger.info("Cleaning up spatial memory")
            self._spatial_memory.cleanup()
            self._spatial_memory = None
        
        return "Navigate skill stopped successfully."

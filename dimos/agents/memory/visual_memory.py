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
Visual memory storage for managing image data persistence and retrieval
"""
import os
import pickle
import base64
import logging
import numpy as np
import cv2

from typing import Dict, Optional, Tuple, Any, List
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.agents.memory.visual_memory")

class VisualMemory:
    """
    A class for storing and retrieving visual memories (images) with persistence.
    
    This class handles the storage, encoding, and retrieval of images associated
    with vector database entries. It provides persistence mechanisms to save and
    load the image data from disk.
    """
    
    def __init__(self, output_dir: str = None):
        """
        Initialize the visual memory system.
        
        Args:
            output_dir: Directory to store the serialized image data
        """
        self.images = {}  # Maps IDs to encoded images
        self.output_dir = output_dir
        
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
            logger.info(f"VisualMemory initialized with output directory: {output_dir}")
        else:
            logger.info("VisualMemory initialized with no persistence directory")
    
    def add(self, image_id: str, image: np.ndarray) -> None:
        """
        Add an image to visual memory.
        
        Args:
            image_id: Unique identifier for the image
            image: The image data as a numpy array
        """
        # Encode the image to base64 for storage
        success, encoded_image = cv2.imencode('.jpg', image)
        if not success:
            logger.error(f"Failed to encode image {image_id}")
            return
        
        image_bytes = encoded_image.tobytes()
        b64_encoded = base64.b64encode(image_bytes).decode('utf-8')
        
        # Store the encoded image
        self.images[image_id] = b64_encoded
        logger.debug(f"Added image {image_id} to visual memory")
    
    def get(self, image_id: str) -> Optional[np.ndarray]:
        """
        Retrieve an image from visual memory.
        
        Args:
            image_id: Unique identifier for the image
            
        Returns:
            The decoded image as a numpy array, or None if not found
        """
        if image_id not in self.images:
            logger.warning(f"Image not found in storage for ID {image_id}. Incomplete or corrupted image storage.")
            return None
        
        try:
            encoded_image = self.images[image_id]
            image_bytes = base64.b64decode(encoded_image)
            image_array = np.frombuffer(image_bytes, dtype=np.uint8)
            image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
            return image
        except Exception as e:
            logger.warning(f"Failed to decode image for ID {image_id}: {str(e)}")
            return None
    
    def contains(self, image_id: str) -> bool:
        """
        Check if an image ID exists in visual memory.
        
        Args:
            image_id: Unique identifier for the image
            
        Returns:
            True if the image exists, False otherwise
        """
        return image_id in self.images
    
    def count(self) -> int:
        """
        Get the number of images in visual memory.
        
        Returns:
            The number of images stored
        """
        return len(self.images)
    
    def save(self, filename: Optional[str] = None) -> str:
        """
        Save the visual memory to disk.
        
        Args:
            filename: Optional filename to save to. If None, uses a default name in the output directory.
            
        Returns:
            The path where the data was saved
        """
        if not self.output_dir:
            logger.warning("No output directory specified for VisualMemory. Cannot save.")
            return ""
        
        if not filename:
            filename = "visual_memory.pkl"
        
        output_path = os.path.join(self.output_dir, filename)
        
        try:
            with open(output_path, 'wb') as f:
                pickle.dump(self.images, f)
            logger.info(f"Saved {len(self.images)} images to {output_path}")
            return output_path
        except Exception as e:
            logger.error(f"Failed to save visual memory: {str(e)}")
            return ""
    
    @classmethod
    def load(cls, path: str, output_dir: Optional[str] = None) -> 'VisualMemory':
        """
        Load visual memory from disk.
        
        Args:
            path: Path to the saved visual memory file
            output_dir: Optional output directory for the new instance
            
        Returns:
            A new VisualMemory instance with the loaded data
        """
        instance = cls(output_dir=output_dir)
        
        if not os.path.exists(path):
            logger.warning(f"Visual memory file {path} not found")
            return instance
        
        try:
            with open(path, 'rb') as f:
                instance.images = pickle.load(f)
            logger.info(f"Loaded {len(instance.images)} images from {path}")
            return instance
        except Exception as e:
            logger.error(f"Failed to load visual memory: {str(e)}")
            return instance
    
    def clear(self) -> None:
        """Clear all images from memory."""
        self.images = {}
        logger.info("Visual memory cleared")

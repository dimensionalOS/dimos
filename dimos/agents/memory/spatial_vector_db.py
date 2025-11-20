#
#
#

"""
Spatial vector database for storing and querying images with XY locations.

This module extends the ChromaDB implementation to support storing images with
their XY locations and querying by location or image similarity.
"""

import os
import logging
import numpy as np
import cv2
import json
import base64
from typing import List, Dict, Tuple, Any, Optional, Union
import chromadb
from chromadb.utils import embedding_functions

from dimos.agents.memory.base import AbstractAgentSemanticMemory
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.agents.memory.spatial_vector_db", level=logging.INFO)

class SpatialVectorDB:
    """
    A vector database for storing and querying images with XY locations.
    
    This class extends the ChromaDB implementation to support storing images with
    their XY locations and querying by location or image similarity.
    """
    
    def __init__(self, collection_name: str = "spatial_memory"):
        """
        Initialize the spatial vector database.
        
        Args:
            collection_name: Name of the vector database collection
        """
        self.collection_name = collection_name
        
        self.client = chromadb.Client()
        
        self.image_collection = self.client.get_or_create_collection(
            name=collection_name,
            metadata={"hnsw:space": "cosine"}
        )
        
        self.image_storage = {}
        
        logger.info(f"SpatialVectorDB initialized with collection: {collection_name}")
    
    def add_image_vector(self, vector_id: str, image: np.ndarray, embedding: np.ndarray, 
                       metadata: Dict[str, Any]) -> None:
        """
        Add an image with its embedding and metadata to the vector database.
        
        Args:
            vector_id: Unique identifier for the vector
            image: The image to store
            embedding: The pre-computed embedding vector for the image
            metadata: Metadata for the image, including x, y coordinates
        """
        _, buffer = cv2.imencode('.jpg', image)
        encoded_image = base64.b64encode(buffer).decode('utf-8')
        
        self.image_storage[vector_id] = encoded_image
        
        if 'x' not in metadata or 'y' not in metadata:
            raise ValueError("Metadata must include 'x' and 'y' coordinates")
        
        self.image_collection.add(
            ids=[vector_id],
            embeddings=[embedding.tolist()],
            metadatas=[metadata]
        )
        
        logger.debug(f"Added image vector with ID: {vector_id}, position: ({metadata['x']}, {metadata['y']})")
    
    def query_by_embedding(self, embedding: np.ndarray, limit: int = 5) -> List[Dict]:
        """
        Query the vector database for images similar to the provided embedding.
        
        Args:
            embedding: Query embedding vector
            limit: Maximum number of results to return
            
        Returns:
            List of results, each containing the image and its metadata
        """
        results = self.image_collection.query(
            query_embeddings=[embedding.tolist()],
            n_results=limit
        )
        
        return self._process_query_results(results)
    
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
        results = self.image_collection.get()
        
        if not results or not results['ids']:
            return []
        
        filtered_results = {
            'ids': [],
            'metadatas': [],
            'distances': []
        }
        
        for i, metadata in enumerate(results['metadatas']):
            item_x = metadata.get('x')
            item_y = metadata.get('y')
            
            if item_x is not None and item_y is not None:
                distance = np.sqrt((x - item_x)**2 + (y - item_y)**2)
                
                if distance <= radius:
                    filtered_results['ids'].append(results['ids'][i])
                    filtered_results['metadatas'].append(metadata)
                    filtered_results['distances'].append(distance)
        
        sorted_indices = np.argsort(filtered_results['distances'])
        filtered_results['ids'] = [filtered_results['ids'][i] for i in sorted_indices[:limit]]
        filtered_results['metadatas'] = [filtered_results['metadatas'][i] for i in sorted_indices[:limit]]
        filtered_results['distances'] = [filtered_results['distances'][i] for i in sorted_indices[:limit]]
        
        return self._process_query_results(filtered_results)
    
    def _process_query_results(self, results) -> List[Dict]:
        """Process query results to include decoded images."""
        if not results or not results['ids']:
            return []
        
        processed_results = []
        
        for i, vector_id in enumerate(results['ids']):
            if vector_id in self.image_storage:
                encoded_image = self.image_storage[vector_id]
                image_bytes = base64.b64decode(encoded_image)
                image_array = np.frombuffer(image_bytes, dtype=np.uint8)
                image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                
                result = {
                    'image': image,
                    'metadata': results['metadatas'][i] if 'metadatas' in results else {},
                    'id': vector_id
                }
                
                if 'distances' in results:
                    result['distance'] = results['distances'][i]
                
                processed_results.append(result)
        
        return processed_results
    
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
        from dimos.agents.memory.image_embedding import ImageEmbeddingProvider
        
        embedding_provider = ImageEmbeddingProvider(model_name="clip")
        
        text_embedding = embedding_provider.get_text_embedding(text)
        
        results = self.image_collection.query(
            query_embeddings=[text_embedding.tolist()],
            n_results=limit
        )
        
        logger.info(f"Text query: '{text}' returned {len(results['ids'] if 'ids' in results else [])} results")
        
        return self._process_query_results(results)
    
    def get_all_locations(self) -> List[Tuple[float, float]]:
        """
        Get all stored locations (x, y coordinates).
        
        Returns:
            List of (x, y) tuples
        """
        results = self.image_collection.get()
        
        if not results or not results['metadatas']:
            return []
        
        locations = []
        for metadata in results['metadatas']:
            if 'x' in metadata and 'y' in metadata:
                locations.append((metadata['x'], metadata['y']))
        
        return locations

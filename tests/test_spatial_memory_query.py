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
Test script for querying an existing spatial memory database

Usage:
  python test_spatial_memory_query.py --query "kitchen table" --limit 5 --threshold 0.7 --save-all
  python test_spatial_memory_query.py --query "robot" --limit 3 --save-one
"""
import os
import sys
import argparse
import numpy as np
import cv2
import matplotlib.pyplot as plt
import chromadb
from datetime import datetime

import tests.test_header
from dimos.perception.spatial_perception import SpatialMemory
from dimos.agents.memory.visual_memory import VisualMemory

def setup_persistent_chroma_db(db_path):
    """Set up a persistent ChromaDB client at the specified path."""
    print(f"Setting up persistent ChromaDB at: {db_path}")
    os.makedirs(db_path, exist_ok=True)
    return chromadb.PersistentClient(path=db_path)

def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Query spatial memory database.")
    parser.add_argument("--query", type=str, default=None,
                        help="Text query to search for (e.g., 'kitchen table')")
    parser.add_argument("--limit", type=int, default=3,
                        help="Maximum number of results to return")
    parser.add_argument("--threshold", type=float, default=None,
                        help="Similarity threshold (0.0-1.0). Only return results above this threshold.")
    parser.add_argument("--save-all", action="store_true",
                        help="Save all result images")
    parser.add_argument("--save-one", action="store_true",
                        help="Save only the best matching image")
    parser.add_argument("--visualize", action="store_true",
                        help="Create a visualization of all stored memory locations")
    parser.add_argument("--db-path", type=str, 
                        default="/home/stash/dimensional/dimos/assets/test_spatial_memory/chromadb_data",
                        help="Path to ChromaDB database")
    parser.add_argument("--visual-memory-path", type=str, 
                        default="/home/stash/dimensional/dimos/assets/test_spatial_memory/visual_memory.pkl",
                        help="Path to visual memory file")
    return parser.parse_args()

def main():
    args = parse_args()
    print("Loading existing spatial memory database for querying...")
    
    # Setup the persistent ChromaDB client
    db_client = setup_persistent_chroma_db(args.db_path)
    
    # Setup output directory for any saved results
    output_dir = os.path.dirname(args.visual_memory_path)
    
    # Load the visual memory
    print(f"Loading visual memory from {args.visual_memory_path}...")
    if os.path.exists(args.visual_memory_path):
        visual_memory = VisualMemory.load(args.visual_memory_path, output_dir=output_dir)
        print(f"Loaded {visual_memory.count()} images from visual memory")
    else:
        visual_memory = VisualMemory(output_dir=output_dir)
        print("No existing visual memory found. Query results won't include images.")
    
    # Create SpatialMemory with the existing database and visual memory
    spatial_memory = SpatialMemory(
        collection_name="test_spatial_memory",
        chroma_client=db_client,
        visual_memory=visual_memory
    )
    
    # Create a visualization if requested
    if args.visualize:
        print("\nCreating visualization of spatial memory...")
        common_objects = [
            "kitchen", "conference room", "vacuum", "office", 
            "bathroom", "boxes", "telephone booth"
        ]
        visualize_spatial_memory_with_objects(
            spatial_memory, 
            objects=common_objects, 
            output_filename="spatial_memory_map.png"
        )
    
    # Handle query if provided
    if args.query:
        query = args.query
        limit = args.limit
        print(f"\nQuerying for: '{query}' (limit: {limit})...")
        
        # Run the query
        results = spatial_memory.query_by_text(query, limit=limit)
        
        if not results:
            print(f"No results found for query: '{query}'")
            return
            
        # Filter by threshold if specified
        if args.threshold is not None:
            print(f"Filtering results with similarity threshold: {args.threshold}")
            filtered_results = []
            for result in results:
                # Distance is inverse of similarity (0 is perfect match)
                # Convert to similarity score (1.0 is perfect match)
                similarity = 1.0 - (result.get('distance', 0) if result.get('distance') is not None else 0)
                if similarity >= args.threshold:
                    filtered_results.append((result, similarity))
            
            # Sort by similarity (highest first)
            filtered_results.sort(key=lambda x: x[1], reverse=True)
            
            if not filtered_results:
                print(f"No results met the similarity threshold of {args.threshold}")
                return
                
            print(f"Found {len(filtered_results)} results above threshold")
            results_with_scores = filtered_results
        else:
            # Add similarity scores for all results
            results_with_scores = []
            for result in results:
                similarity = 1.0 - (result.get('distance', 0) if result.get('distance') is not None else 0)
                results_with_scores.append((result, similarity))
        
        # Process and display results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        for i, (result, similarity) in enumerate(results_with_scores):
            metadata = result.get('metadata', {})
            if isinstance(metadata, list) and metadata:
                metadata = metadata[0]
                
            # Display result information
            print(f"\nResult {i+1} for '{query}':")
            print(f"Similarity: {similarity:.4f} (distance: {1.0 - similarity:.4f})")
            
            # Extract and display position information
            if isinstance(metadata, dict):
                x = metadata.get('x', 0)
                y = metadata.get('y', 0)
                z = metadata.get('z', 0)
                print(f"Position: ({x:.2f}, {y:.2f}, {z:.2f})")
                if 'timestamp' in metadata:
                    print(f"Timestamp: {metadata['timestamp']}")
                if 'frame_id' in metadata:
                    print(f"Frame ID: {metadata['frame_id']}")
            
            # Save image if requested and available
            if 'image' in result and result['image'] is not None:
                # Only save first image, or all images based on flags
                if args.save_one and i > 0:
                    continue
                if not (args.save_all or args.save_one):
                    continue
                    
                # Create a descriptive filename
                clean_query = query.replace(' ', '_').replace('/', '_').lower()
                output_filename = f"{clean_query}_result_{i+1}_{timestamp}.jpg"
                
                # Save the image
                cv2.imwrite(output_filename, result["image"])
                print(f"Saved image to {output_filename}")
            elif 'image' in result and result['image'] is None:
                print("Image data not available for this result")
    else:
        print("No query specified. Use --query \"text to search for\" to run a query.")
        print("Use --help to see all available options.")
    
    print("\nQuery completed successfully!")

def visualize_spatial_memory_with_objects(spatial_memory, objects, output_filename="spatial_memory_map.png"):
    """Visualize spatial memory with labeled objects."""
    # Define colors for different objects
    colors = ['red', 'green', 'orange', 'purple', 'brown', 'cyan', 'magenta', 'yellow']
    
    # Get all stored locations for background
    locations = spatial_memory.vector_db.get_all_locations()
    if not locations:
        print("No locations stored in spatial memory.")
        return
    
    # Extract coordinates
    if len(locations[0]) >= 3:
        x_coords = [loc[0] for loc in locations]
        y_coords = [loc[1] for loc in locations]
    else:
        x_coords, y_coords = zip(*locations)
    
    # Create figure
    plt.figure(figsize=(12, 10))
    plt.scatter(x_coords, y_coords, c='blue', s=50, alpha=0.5, label='All Frames')
    
    # Container for object coordinates
    object_coords = {}
    
    # Query for each object
    for i, obj in enumerate(objects):
        color = colors[i % len(colors)]
        print(f"Processing {obj} query for visualization...")
        
        # Get best match
        results = spatial_memory.query_by_text(obj, limit=1)
        if not results:
            print(f"No results found for '{obj}'")
            continue
            
        # Process result
        result = results[0]
        metadata = result['metadata']
        
        if isinstance(metadata, list) and metadata:
            metadata = metadata[0]
            
        if isinstance(metadata, dict) and 'x' in metadata and 'y' in metadata:
            x = metadata.get('x', 0)
            y = metadata.get('y', 0)
            
            # Store coordinates
            object_coords[obj] = (x, y)
            
            # Plot position
            plt.scatter([x], [y], c=color, s=100, alpha=0.8, label=obj.title())
            
            # Add annotation
            obj_abbrev = obj[0].upper() if len(obj) > 0 else 'X'
            plt.annotate(f"{obj_abbrev}", (x, y), textcoords="offset points", 
                       xytext=(0,10), ha='center')
            
            # Save image if available
            if 'image' in result and result['image'] is not None:
                clean_name = obj.replace(' ', '_').lower()
                output_img_filename = f"{clean_name}_result.jpg"
                cv2.imwrite(output_img_filename, result["image"])
                print(f"Saved {obj} image to {output_img_filename}")
    
    # Finalize plot
    plt.title("Spatial Memory Map with Query Results")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    
    # Add origin marker
    plt.gca().add_patch(plt.Circle((0, 0), 1.0, fill=False, color='blue', linestyle='--'))
    
    # Save visualization
    plt.savefig(output_filename, dpi=300)
    print(f"Saved visualization to {output_filename}")
    
    return object_coords

if __name__ == "__main__":
    main()

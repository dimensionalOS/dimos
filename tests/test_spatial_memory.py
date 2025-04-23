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

import os
import sys
import time
import pickle
import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import reactivex
from reactivex import operators as ops
import chromadb

from dimos.agents.memory.visual_memory import VisualMemory

import tests.test_header

from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.perception.spatial_perception import SpatialMemory

def extract_position(transform):
    """Extract position coordinates from a transform message"""
    if transform is None:
        return (0, 0, 0)
    
    pos = transform.transform.translation
    return (pos.x, pos.y, pos.z)

def setup_persistent_chroma_db(db_path="chromadb_data"):
    """
    Set up a persistent ChromaDB database at the specified path.
    
    Args:
        db_path: Path to store the ChromaDB database
        
    Returns:
        The ChromaDB client instance
    """
    # Create a persistent ChromaDB client
    full_db_path = os.path.join("/home/stash/dimensional/dimos/assets/test_spatial_memory", db_path)
    print(f"Setting up persistent ChromaDB at: {full_db_path}")
    
    # Ensure the directory exists
    os.makedirs(full_db_path, exist_ok=True)
    
    return chromadb.PersistentClient(path=full_db_path)

def main():
    print("Starting spatial memory test...")
    
    # Initialize ROS control and robot
    ros_control = UnitreeROSControl(
        node_name="spatial_memory_test",
        mock_connection=False
    )
    
    robot = UnitreeGo2(
        ros_control=ros_control,
        ip=os.getenv('ROBOT_IP')
    )
    
    # Create counters for tracking
    frame_count = 0
    transform_count = 0
    stored_count = 0
    
    print("Setting up video stream...")
    video_stream = robot.get_ros_video_stream()  
    
    # Create transform stream at 1 Hz
    print("Setting up transform stream...")
    transform_stream = ros_control.get_transform_stream(
        child_frame="map",
        parent_frame="base_link",
        rate_hz=1.0  # 1 transform per second
    )
    
    # Setup output directory for visual memory
    visual_memory_dir = "/home/stash/dimensional/dimos/assets/test_spatial_memory"
    os.makedirs(visual_memory_dir, exist_ok=True)
    
    # Setup persistent storage path for visual memory
    visual_memory_path = os.path.join(visual_memory_dir, "visual_memory.pkl")
    
    # Try to load existing visual memory if it exists
    if os.path.exists(visual_memory_path):
        try:
            print(f"Loading existing visual memory from {visual_memory_path}...")
            visual_memory = VisualMemory.load(visual_memory_path, output_dir=visual_memory_dir)
            print(f"Loaded {visual_memory.count()} images from previous runs")
        except Exception as e:
            print(f"Error loading visual memory: {e}")
            visual_memory = VisualMemory(output_dir=visual_memory_dir)
    else:
        print("No existing visual memory found. Starting with empty visual memory.")
        visual_memory = VisualMemory(output_dir=visual_memory_dir)
    
    # Setup a persistent database for ChromaDB
    db_client = setup_persistent_chroma_db()
    
    # Create spatial perception instance with persistent storage
    print("Creating SpatialMemory with persistent vector database...")
    spatial_memory = SpatialMemory(
        collection_name="test_spatial_memory",
        min_distance_threshold=1,  # Store frames every 1 meter
        min_time_threshold=1,  # Store frames at least every 1 second
        chroma_client=db_client,  # Use the persistent client
        visual_memory=visual_memory  # Use the visual memory we loaded or created
    )
    
    # Combine streams using combine_latest
    # This will pair up items properly without buffering
    combined_stream = reactivex.combine_latest(video_stream, transform_stream).pipe(
        ops.map(lambda pair: {
            "frame": pair[0],  # First element is the frame
            "position": extract_position(pair[1])  # Second element is the transform
        })
    )
    
    # Process with spatial memory
    result_stream = spatial_memory.process_stream(combined_stream)
    
    # Simple callback to track stored frames and save them to the assets directory
    def on_stored_frame(result):
        nonlocal stored_count
        # Only count actually stored frames (not debug frames)
        if not result.get('stored', True) == False:
            stored_count += 1
            pos = result['position']
            print(f"\nStored frame #{stored_count} at ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            
            # Save the frame to the assets directory
            if 'frame' in result:
                frame_filename = f"/home/stash/dimensional/dimos/assets/test_spatial_memory/frame_{stored_count:03d}.jpg"
                cv2.imwrite(frame_filename, result['frame'])
                print(f"Saved frame to {frame_filename}")
    
    # Subscribe to results
    print("Subscribing to spatial perception results...")
    result_subscription = result_stream.subscribe(on_stored_frame)
    
    print("\nRunning until interrupted...")
    try:
        while True:
            time.sleep(1.0)  
            print(f"Running: {stored_count} frames stored so far", end="\r")
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        # Clean up resources
        print("\nCleaning up...")
        if 'result_subscription' in locals():
            result_subscription.dispose()
    
    # Visualize spatial memory with multiple object queries
    visualize_spatial_memory_with_objects(
        spatial_memory, 
        objects=["kitchen", "conference room", "vacuum", "office", "bathroom", "boxes", "telephone booth"], 
        output_filename="spatial_memory_map.png"
    )
    
    # Save visual memory to disk for later use
    saved_path = spatial_memory.vector_db.visual_memory.save("visual_memory.pkl")
    print(f"Saved {spatial_memory.vector_db.visual_memory.count()} images to disk at {saved_path}")
    
def visualize_spatial_memory_with_objects(spatial_memory, objects, output_filename="spatial_memory_map.png"):
    """
    Visualize a spatial memory map with multiple labeled objects.
    
    Args:
        spatial_memory: SpatialMemory instance
        objects: List of object names to query and visualize (e.g. ["kitchen", "office"])
        output_filename: Filename to save the visualization
    """
    # Define colors for different objects - will cycle through these
    colors = ['red', 'green', 'orange', 'purple', 'brown', 'cyan', 'magenta', 'yellow']
    
    # Get all stored locations for background
    locations = spatial_memory.vector_db.get_all_locations()
    if not locations:
        print("No locations stored in spatial memory.")
        return
    
    # Extract coordinates from all stored locations
    if len(locations[0]) >= 3:
        x_coords = [loc[0] for loc in locations]
        y_coords = [loc[1] for loc in locations]
    else:
        x_coords, y_coords = zip(*locations)
    
    # Create figure
    plt.figure(figsize=(12, 10))
    
    # Plot all points in blue
    plt.scatter(x_coords, y_coords, c='blue', s=50, alpha=0.5, label='All Frames')
    
    # Container for all object coordinates
    object_coords = {}
    
    # Query for each object and store the result
    for i, obj in enumerate(objects):
        color = colors[i % len(colors)]  # Cycle through colors
        print(f"\nProcessing {obj} query for visualization...")
        
        # Get best match for this object
        results = spatial_memory.query_by_text(obj, limit=1)
        if not results:
            print(f"No results found for '{obj}'")
            continue
            
        # Get the first (best) result
        result = results[0]
        metadata = result['metadata']
        
        # Extract coordinates from the first metadata item
        if isinstance(metadata, list) and metadata:
            metadata = metadata[0]
            
        if isinstance(metadata, dict) and 'x' in metadata and 'y' in metadata:
            x = metadata.get('x', 0)
            y = metadata.get('y', 0)
            
            # Store coordinates for this object
            object_coords[obj] = (x, y)
            
            # Plot this object's position
            plt.scatter([x], [y], c=color, s=100, alpha=0.8, label=obj.title())
            
            # Add annotation
            obj_abbrev = obj[0].upper() if len(obj) > 0 else 'X'
            plt.annotate(f"{obj_abbrev}", (x, y), textcoords="offset points", 
                        xytext=(0,10), ha='center')
            
            # Save the image to a file using the object name
            if 'image' in result and result['image'] is not None:
                # Clean the object name to make it suitable for a filename
                clean_name = obj.replace(' ', '_').lower()
                output_img_filename = f"{clean_name}_result.jpg"
                cv2.imwrite(output_img_filename, result["image"])
                print(f"Saved {obj} image to {output_img_filename}")
    
    # Finalize the plot
    plt.title("Spatial Memory Map with Query Results")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    
    # Add origin circle
    plt.gca().add_patch(Circle((0, 0), 1.0, fill=False, color='blue', linestyle='--'))
    
    # Save the visualization
    plt.savefig(output_filename, dpi=300)
    print(f"Saved enhanced map visualization to {output_filename}")
    
    return object_coords
    
    # Final cleanup
    print("Performing final cleanup...")
    spatial_memory.cleanup()
    
    try:
        robot.cleanup()
    except Exception as e:
        print(f"Error during robot cleanup: {e}")
    
    print("Test completed successfully")

if __name__ == "__main__":
    main()

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
import time

import chromadb
import cv2
from matplotlib.patches import Circle
import matplotlib.pyplot as plt
import reactivex
from reactivex import operators as ops

from dimos.agents.memory.visual_memory import VisualMemory
from dimos.msgs.geometry_msgs import Quaternion, Vector3

# from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2  # Uncomment when properly configured
from dimos.perception.spatial_perception import SpatialMemory


def extract_pose_data(transform):
    """Extract position and rotation from a transform message"""
    if transform is None:
        return None, None

    pos = transform.transform.translation
    rot = transform.transform.rotation

    # Convert to Vector3 objects expected by SpatialMemory
    position = Vector3(x=pos.x, y=pos.y, z=pos.z)

    # Convert quaternion to euler angles for rotation vector
    quat = Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w)
    euler = quat.to_euler()
    rotation = Vector3(x=euler.x, y=euler.y, z=euler.z)

    return position, rotation


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

    # Create counters for tracking
    frame_count = 0
    transform_count = 0
    stored_count = 0

    print("Note: This test requires proper robot connection setup.")
    print("Please ensure video_stream and transform_stream are properly configured.")

    # These need to be set up based on your specific robot configuration
    video_stream = None  # TODO: Set up video stream from robot
    transform_stream = None  # TODO: Set up transform stream from robot

    if video_stream is None or transform_stream is None:
        print("\nWARNING: Video or transform streams not configured.")
        print("Exiting test. Please configure streams properly.")
        return

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
        visual_memory=visual_memory,  # Use the visual memory we loaded or created
    )

    # Combine streams using combine_latest
    # This will pair up items properly without buffering
    combined_stream = reactivex.combine_latest(video_stream, transform_stream).pipe(
        ops.map(
            lambda pair: {
                "frame": pair[0],  # First element is the frame
                "position": extract_pose_data(pair[1])[0],  # Position as Vector3
                "rotation": extract_pose_data(pair[1])[1],  # Rotation as Vector3
            }
        ),
        ops.filter(lambda data: data["position"] is not None and data["rotation"] is not None),
    )

    # Process with spatial memory
    result_stream = spatial_memory.process_stream(combined_stream)

    # Simple callback to track stored frames and save them to the assets directory
    def on_stored_frame(result):
        nonlocal stored_count
        # Only count actually stored frames (not debug frames)
        if not not result.get("stored", True):
            stored_count += 1
            pos = result["position"]
            if isinstance(pos, tuple):
                print(
                    f"\nStored frame #{stored_count} at ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
                )
            else:
                print(f"\nStored frame #{stored_count} at position {pos}")

            # Save the frame to the assets directory
            if "frame" in result:
                frame_filename = f"/home/stash/dimensional/dimos/assets/test_spatial_memory/frame_{stored_count:03d}.jpg"
                cv2.imwrite(frame_filename, result["frame"])
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
        if "result_subscription" in locals():
            result_subscription.dispose()

    # Visualize spatial memory with multiple object queries
    visualize_spatial_memory_with_objects(
        spatial_memory,
        objects=[
            "kitchen",
            "conference room",
            "vacuum",
            "office",
            "bathroom",
            "boxes",
            "telephone booth",
        ],
        output_filename="spatial_memory_map.png",
    )

    # Save visual memory to disk for later use
    saved_path = spatial_memory.vector_db.visual_memory.save("visual_memory.pkl")
    print(f"Saved {spatial_memory.vector_db.visual_memory.count()} images to disk at {saved_path}")

    spatial_memory.stop()

    print("Test completed successfully")


def visualize_spatial_memory_with_objects(
    spatial_memory, objects, output_filename="spatial_memory_map.png"
):
    """
    Visualize a spatial memory map with multiple labeled objects.

    Args:
        spatial_memory: SpatialMemory instance
        objects: List of object names to query and visualize (e.g. ["kitchen", "office"])
        output_filename: Filename to save the visualization
    """
    # Define colors for different objects - will cycle through these
    colors = ["red", "green", "orange", "purple", "brown", "cyan", "magenta", "yellow"]

    # Get all stored locations for background
    locations = spatial_memory.vector_db.get_all_locations()
    if not locations:
        print("No locations stored in spatial memory.")
        return

    # Extract coordinates from all stored locations
    x_coords = []
    y_coords = []
    for loc in locations:
        if isinstance(loc, dict):
            x_coords.append(loc.get("pos_x", 0))
            y_coords.append(loc.get("pos_y", 0))
        elif isinstance(loc, tuple | list) and len(loc) >= 2:
            x_coords.append(loc[0])
            y_coords.append(loc[1])
        else:
            print(f"Unknown location format: {loc}")

    # Create figure
    plt.figure(figsize=(12, 10))

    # Plot all points in blue
    plt.scatter(x_coords, y_coords, c="blue", s=50, alpha=0.5, label="All Frames")

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
        metadata = result["metadata"]

        # Extract coordinates from the first metadata item
        if isinstance(metadata, list) and metadata:
            metadata = metadata[0]

        if isinstance(metadata, dict):
            # New metadata format uses pos_x, pos_y
            x = metadata.get("pos_x", metadata.get("x", 0))
            y = metadata.get("pos_y", metadata.get("y", 0))

            # Store coordinates for this object
            object_coords[obj] = (x, y)

            # Plot this object's position
            plt.scatter([x], [y], c=color, s=100, alpha=0.8, label=obj.title())

            # Add annotation
            obj_abbrev = obj[0].upper() if len(obj) > 0 else "X"
            plt.annotate(
                f"{obj_abbrev}", (x, y), textcoords="offset points", xytext=(0, 10), ha="center"
            )

            # Save the image to a file using the object name
            if "image" in result and result["image"] is not None:
                # Clean the object name to make it suitable for a filename
                clean_name = obj.replace(" ", "_").lower()
                output_img_filename = f"{clean_name}_result.jpg"
                cv2.imwrite(output_img_filename, result["image"])
                print(f"Saved {obj} image to {output_img_filename}")

    # Finalize the plot
    plt.title("Spatial Memory Map with Query Results")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.grid(True)
    plt.axis("equal")
    plt.legend()

    # Add origin circle
    plt.gca().add_patch(Circle((0, 0), 1.0, fill=False, color="blue", linestyle="--"))

    # Save the visualization
    plt.savefig(output_filename, dpi=300)
    print(f"Saved enhanced map visualization to {output_filename}")

    return object_coords


if __name__ == "__main__":
    main()

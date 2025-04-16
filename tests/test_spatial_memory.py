
import os
import sys
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import reactivex
from reactivex import operators as ops

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.perception.spatial_perception import SpatialMemory

def extract_position(transform):
    """Extract position coordinates from a transform message"""
    if transform is None:
        return (0, 0, 0)
    
    pos = transform.transform.translation
    return (pos.x, pos.y, pos.z)

def main():
    print("Starting spatial memory test...")
    
    # Initialize ROS control and robot
    ros_control = UnitreeROSControl(
        node_name="spatial_perception_test",
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
    
    # Create video stream at 5 FPS
    print("Setting up video stream...")
    video_stream = robot.get_ros_video_stream()  
    
    # Create transform stream at 1 Hz
    print("Setting up transform stream...")
    transform_stream = ros_control.get_transform_stream(
        child_frame="map",
        parent_frame="base_link",
        rate_hz=1.0  # 1 transform per second
    )
    
    # Create spatial perception instance
    spatial_perception = SpatialMemory(
        collection_name="test_spatial_memory",
        min_distance_threshold=1,  # Store frames every 0.2 meters
        min_time_threshold=1,  # Store frames at least every 1 second
    )
    
    # Combine streams using zip operator
    print("Creating combined stream with zip...")
    combined_stream = reactivex.zip(video_stream, transform_stream).pipe(
        ops.map(lambda pair: {
            "frame": pair[0],  # First element is the frame
            "position": extract_position(pair[1])  # Second element is the transform
        })
    )
    
    # Process with spatial perception
    result_stream = spatial_perception.process_stream(combined_stream)
    
    # Simple callback to track stored frames (avoids cv2.imshow which can cause issues)
    def on_stored_frame(result):
        nonlocal stored_count
        # Only count actually stored frames (not debug frames)
        if not result.get('stored', True) == False:
            stored_count += 1
            pos = result['position']
            print(f"\nStored frame #{stored_count} at ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
    
    # Subscribe to results
    print("Subscribing to spatial perception results...")
    result_subscription = result_stream.subscribe(on_stored_frame)
    
    # Run for 60 seconds or until interrupted
    print("\nRunning for 60 seconds or until interrupted...")
    try:
        start_time = time.time()
        while time.time() - start_time < 60:
            time.sleep(1.0)  
            print(f"Running: {stored_count} frames stored so far", end="\r")
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        # Clean up resources
        print("\nCleaning up...")
        if 'result_subscription' in locals():
            result_subscription.dispose()
        
        # Clean up OpenCV windows
        cv2.destroyAllWindows()
    
    # Query the spatial database for testing
    print("\nQuerying by location (0, 0)...")
    location_results = spatial_perception.query_by_location(0, 0, radius=1.0, limit=3)
    print(location_results)
    
    for i, result in enumerate(location_results):
        print(f"Result {i+1}: Position ({result['metadata']['x']:.2f}, {result['metadata']['y']:.2f}, {result['metadata'].get('z', 0):.2f}), "
              f"Distance: {result.get('distance', 'N/A')}")
        
    # if location_results:
    #     print("\nQuerying by image similarity...")
    #     image_results = spatial_perception.query_by_image(location_results[0]["image"], limit=3)
        
    #     for i, result in enumerate(image_results):
    #         print(f"Result {i+1}: Position ({result['metadata']['x']:.2f}, {result['metadata']['y']:.2f}, {result['metadata'].get('z', 0):.2f}), "
    #               f"Distance: {result.get('distance', 'N/A')}")
            
    #         cv2.imshow(f"Image Result {i+1}", result["image"])
    #         cv2.waitKey(0)
        
    #     cv2.destroyAllWindows()
    
    # Visualize spatial memory map
    locations = spatial_perception.vector_db.get_all_locations()
    
    if locations:
        # Extract x and y coordinates from locations
        # If locations are 3D, ignore z for the 2D plot
        if len(locations[0]) >= 3:
            x_coords = [loc[0] for loc in locations]
            y_coords = [loc[1] for loc in locations]
        else:
            x_coords, y_coords = zip(*locations)
        
        plt.figure(figsize=(10, 8))
        plt.scatter(x_coords, y_coords, c='blue', s=50, alpha=0.7)
        plt.title("Spatial Memory Map")
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.grid(True)
        plt.axis('equal')
        
        plt.gca().add_patch(Circle((0, 0), 1.0, fill=False, color='red', linestyle='--'))
        
        plt.savefig("spatial_memory_map.png")
        plt.show()
    
    # Final cleanup
    print("Performing final cleanup...")
    spatial_perception.cleanup()
    
    try:
        robot.cleanup()
    except Exception as e:
        print(f"Error during robot cleanup: {e}")
    
    print("Test completed successfully")

if __name__ == "__main__":
    main()

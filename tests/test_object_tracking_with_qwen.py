import os
import sys
import time
import cv2
import numpy as np
import queue
import threading
import json
from reactivex import Subject, operators as RxOps
from openai import OpenAI

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dimos.stream.video_provider import VideoProvider
from dimos.perception.object_tracker import ObjectTrackingStream
from dimos.models.qwen.video_query import get_bbox_from_qwen
from dimos.utils.logging_config import logger

# Global variables for tracking control
object_size = 0.30  # Hardcoded object size in meters (adjust based on your tracking target)
tracking_object_name = "object"  # Will be updated by Qwen
object_name = "cup"  # Example object name for Qwen

global tracker_initialized, detection_in_progress

# Create queues for thread communication
frame_queue = queue.Queue(maxsize=5)
stop_event = threading.Event()

# Logitech C920e camera parameters at 480p
width, height = 640, 480
focal_length_mm = 3.67  # mm
sensor_width_mm = 4.8   # mm (1/4" sensor)
sensor_height_mm = 3.6  # mm

# Calculate focal length in pixels
focal_length_x_px = width * focal_length_mm / sensor_width_mm
focal_length_y_px = height * focal_length_mm / sensor_height_mm
cx, cy = width / 2, height / 2

# Final camera intrinsics in [fx, fy, cx, cy] format
camera_intrinsics = [focal_length_x_px, focal_length_y_px, cx, cy]

# Initialize video provider and object tracking stream
video_provider = VideoProvider("webcam", video_source=0)
tracker_stream = ObjectTrackingStream(
    camera_intrinsics=camera_intrinsics,
    camera_pitch=0.0,
    camera_height=0.5
)

# Create video streams
video_stream = video_provider.capture_video_as_observable(realtime=True, fps=10)
tracking_stream = tracker_stream.create_stream(video_stream)


# Define callbacks for the tracking stream
def on_next(result):
    global tracker_initialized, detection_in_progress
    if stop_event.is_set():
        return

    # Get the visualization frame
    viz_frame = result["viz_frame"]
    
    # Add information to the visualization
    cv2.putText(viz_frame, f"Tracking {tracking_object_name}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(viz_frame, f"Object size: {object_size:.2f}m", (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # Show tracking status
    status = "Tracking" if tracker_initialized else "Waiting for detection"
    color = (0, 255, 0) if tracker_initialized else (0, 0, 255)
    cv2.putText(viz_frame, f"Status: {status}", (10, 90), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    
    # If detection is in progress, show a message
    if detection_in_progress:
        cv2.putText(viz_frame, "Querying Qwen...", (10, 120), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    # Put frame in queue for main thread to display
    try:
        frame_queue.put_nowait(viz_frame)
    except queue.Full:
        pass

def on_error(error):
    print(f"Error: {error}")
    stop_event.set()

def on_completed():
    print("Stream completed")
    stop_event.set()

# Start the subscription
subscription = None

try:
    # Initialize global flags
    tracker_initialized = False
    detection_in_progress = False
    # Subscribe to start processing in background thread
    subscription = tracking_stream.subscribe(
        on_next=on_next,
        on_error=on_error,
        on_completed=on_completed
    )
    
    print("Object tracking with Qwen started. Press 'q' to exit.")
    print("Waiting for initial object detection...")
    
    # Main thread loop for displaying frames and updating tracking
    while not stop_event.is_set():
        # Check if we need to update tracking

        if not detection_in_progress:
            detection_in_progress = True
            print("Requesting object detection from Qwen...")

            print("detection_in_progress: ", detection_in_progress)
            print("tracker_initialized: ", tracker_initialized)

            def detection_task():
                global detection_in_progress, tracker_initialized
                try:
                    bbox = get_bbox_from_qwen(video_stream, object_name=object_name)
                    
                    if bbox:
                        print(f"Detected {tracking_object_name} at {bbox}")
                        # Initialize tracker with the new bbox and default size
                        tracker_stream.track(bbox, size=object_size)
                        tracker_initialized = True
                    else:
                        print("No object detected by Qwen")
                        tracker_initialized = False
                        tracker_stream.stop_track()

                except Exception as e:
                    print(f"Error in update_tracking: {e}")
                finally:
                    detection_in_progress = False

            # Run detection task in a separate thread
            threading.Thread(target=detection_task, daemon=True).start()
            
        try:
            # Get frame with timeout
            viz_frame = frame_queue.get(timeout=0.1)
            
            # Display the frame
            cv2.imshow("Object Tracking with Qwen", viz_frame)
            
            # Check for exit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Exit key pressed")
                break
                
        except queue.Empty:
            # No frame available, check if we should continue
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Exit key pressed")
                break
            continue
            
except KeyboardInterrupt:
    print("\nKeyboard interrupt received. Stopping...")
finally:
    # Signal threads to stop
    stop_event.set()
    
    # Clean up resources
    if subscription:
        subscription.dispose()
    
    video_provider.dispose_all()
    tracker_stream.cleanup()
    cv2.destroyAllWindows()
    print("Cleanup complete")
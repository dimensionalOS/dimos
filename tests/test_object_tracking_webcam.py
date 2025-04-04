import cv2
import numpy as np
import os
import sys
import queue
import threading

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dimos.stream.video_provider import VideoProvider
from dimos.perception.object_tracker import ObjectTrackingStream

# Global variables for bounding box selection
selecting_bbox = False
bbox_points = []
current_bbox = None
tracker_initialized = False
object_size = 0.30  # Hardcoded object size in meters (adjust based on your tracking target)

def mouse_callback(event, x, y, flags, param):
    global selecting_bbox, bbox_points, current_bbox, tracker_initialized, tracker_stream
    
    if event == cv2.EVENT_LBUTTONDOWN:
        # Start bbox selection
        selecting_bbox = True
        bbox_points = [(x, y)]
        current_bbox = None
        tracker_initialized = False
    
    elif event == cv2.EVENT_MOUSEMOVE and selecting_bbox:
        # Update current selection for visualization
        current_bbox = [bbox_points[0][0], bbox_points[0][1], x, y]
    
    elif event == cv2.EVENT_LBUTTONUP:
        # End bbox selection
        selecting_bbox = False
        if bbox_points:
            bbox_points.append((x, y))
            x1, y1 = bbox_points[0]
            x2, y2 = bbox_points[1]
            # Ensure x1,y1 is top-left and x2,y2 is bottom-right
            current_bbox = [min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)]
            # Add the bbox to the tracking queue
            if param.get('bbox_queue') and not tracker_initialized:
                param['bbox_queue'].put((current_bbox, object_size))
                tracker_initialized = True


def main():
    global tracker_initialized
    
    # Create queues for thread communication
    frame_queue = queue.Queue(maxsize=5)
    bbox_queue = queue.Queue()
    stop_event = threading.Event()
    
    # Logitech C920e camera parameters at 480p
    # Convert physical parameters to pixel-based intrinsics
    width, height = 640, 480
    focal_length_mm = 3.67  # mm
    sensor_width_mm = 4.8   # mm (1/4" sensor)
    sensor_height_mm = 3.6  # mm
    
    # Calculate focal length in pixels
    focal_length_x_px = width * focal_length_mm / sensor_width_mm
    focal_length_y_px = height * focal_length_mm / sensor_height_mm
    
    # Principal point (assuming center of image)
    cx = width / 2
    cy = height / 2
    
    # Final camera intrinsics in [fx, fy, cx, cy] format
    camera_intrinsics = [focal_length_x_px, focal_length_y_px, cx, cy]
    
    # Initialize video provider and object tracking stream
    video_provider = VideoProvider("test_camera", video_source=0)
    tracker_stream = ObjectTrackingStream(
        camera_intrinsics=camera_intrinsics,
        camera_pitch=0.0,  # Adjust if your camera is tilted
        camera_height=0.5  # Height of camera from ground in meters (adjust as needed)
    )
    
    # Create video stream
    video_stream = video_provider.capture_video_as_observable(realtime=True, fps=30)
    tracking_stream = tracker_stream.create_stream(video_stream)
    
    # Define callbacks for the tracking stream
    def on_next(result):
        if stop_event.is_set():
            return

        # Get the visualization frame
        viz_frame = result["viz_frame"]
        
        # If we're selecting a bbox, draw the current selection
        if selecting_bbox and current_bbox is not None:
            x1, y1, x2, y2 = current_bbox
            cv2.rectangle(viz_frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
        
        # Add instructions
        cv2.putText(viz_frame, "Click and drag to select object", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(viz_frame, f"Object size: {object_size:.2f}m", (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Show tracking status
        status = "Tracking" if tracker_initialized else "Not tracking"
        cv2.putText(viz_frame, f"Status: {status}", (10, 90), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if tracker_initialized else (0, 0, 255), 2)
        
        # Put frame in queue for main thread to display
        try:
            frame_queue.put_nowait(viz_frame)
        except queue.Full:
            # Skip frame if queue is full
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
        # Subscribe to start processing in background thread
        subscription = tracking_stream.subscribe(
            on_next=on_next,
            on_error=on_error,
            on_completed=on_completed
        )
        
        print("Object tracking started. Click and drag to select an object. Press 'q' to exit.")
        
        # Create window and set mouse callback
        cv2.namedWindow("Object Tracker")
        cv2.setMouseCallback("Object Tracker", mouse_callback, {'bbox_queue': bbox_queue})
        
        # Main thread loop for displaying frames and handling bbox selection
        while not stop_event.is_set():
            # Check if there's a new bbox to track
            try:
                new_bbox, size = bbox_queue.get_nowait()
                print(f"New object selected: {new_bbox}, size: {size}m")
                # Initialize tracker with the new bbox and size
                tracker_stream.track(new_bbox, size=size)
            except queue.Empty:
                pass
                
            try:
                # Get frame with timeout
                viz_frame = frame_queue.get(timeout=1.0)
                
                # Display the frame
                cv2.imshow("Object Tracker", viz_frame)
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


if __name__ == "__main__":
    main()
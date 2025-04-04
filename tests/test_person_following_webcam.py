import cv2
import numpy as np
import os
import sys
import queue
import threading

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dimos.stream.video_provider import VideoProvider
from dimos.perception.person_tracker import PersonTrackingStream
from dimos.perception.visual_servoing import VisualServoing


def main():
    # Create a queue for thread communication (limit to prevent memory issues)
    frame_queue = queue.Queue(maxsize=5)
    result_queue = queue.Queue(maxsize=5)  # For tracking results
    stop_event = threading.Event()
    
    # Logitech C920e camera parameters at 480p
    # Convert physical parameters to intrinsics [fx, fy, cx, cy]
    resolution = (640, 480)  # 480p resolution
    focal_length_mm = 3.67  # mm
    sensor_size_mm = (4.8, 3.6)  # mm (1/4" sensor)
    
    # Calculate focal length in pixels
    fx = (resolution[0] * focal_length_mm) / sensor_size_mm[0]
    fy = (resolution[1] * focal_length_mm) / sensor_size_mm[1]
    
    # Principal point (typically at image center)
    cx = resolution[0] / 2
    cy = resolution[1] / 2
    
    # Camera intrinsics in [fx, fy, cx, cy] format
    camera_intrinsics = [fx, fy, cx, cy]
    
    # Camera mounted parameters
    camera_pitch = np.deg2rad(-5)  # negative for downward pitch
    camera_height = 1.4  # meters
    
    # Initialize video provider and person tracking stream
    video_provider = VideoProvider("test_camera", video_source=0)
    person_tracker = PersonTrackingStream(
        camera_intrinsics=camera_intrinsics,
        camera_pitch=camera_pitch,
        camera_height=camera_height
    )
    
    # Create streams
    video_stream = video_provider.capture_video_as_observable(realtime=False, fps=20)
    person_tracking_stream = person_tracker.create_stream(video_stream)
    
    # Create visual servoing object
    visual_servoing = VisualServoing(
        tracking_stream=person_tracking_stream,
        max_linear_speed=0.5,
        max_angular_speed=0.75,
        desired_distance=2.5
    )
    
    # Track if we have selected a person to follow
    selected_point = None
    tracking_active = False
    
    # Define callbacks for the tracking stream
    def on_next(result):
        if stop_event.is_set():
            return

        # Get the visualization frame which already includes person detections
        # with bounding boxes, tracking IDs, and distance/angle information
        viz_frame = result["viz_frame"]
        
        # Store the result for the main thread to use with visual servoing
        try:
            result_queue.put_nowait(result)
        except queue.Full:
            # Skip if queue is full
            pass
        
        # Put frame in queue for main thread to display (non-blocking)
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
    
    # Mouse callback for selecting a person to track
    def mouse_callback(event, x, y, flags, param):
        nonlocal selected_point, tracking_active
        
        if event == cv2.EVENT_LBUTTONDOWN:
            # Store the clicked point
            selected_point = (x, y)
            tracking_active = False  # Will be set to True if start_tracking succeeds
            print(f"Selected point: {selected_point}")
    
    # Start the subscription
    subscription = None
    
    try:
        # Subscribe to start processing in background thread
        subscription = person_tracking_stream.subscribe(
            on_next=on_next,
            on_error=on_error,
            on_completed=on_completed
        )
        
        print("Person tracking visualization started.")
        print("Click on a person to start visual servoing. Press 'q' to exit.")
        
        # Set up mouse callback
        cv2.namedWindow("Person Tracking")
        cv2.setMouseCallback("Person Tracking", mouse_callback)
        
        # Main thread loop for displaying frames
        while not stop_event.is_set():
            try:
                # Get frame with timeout (allows checking stop_event periodically)
                frame = frame_queue.get(timeout=1.0)
                
                # Call the visual servoing if we have a selected point
                if selected_point is not None:
                    # If not actively tracking, try to start tracking
                    if not tracking_active:
                        tracking_active = visual_servoing.start_tracking(selected_point)
                        if not tracking_active:
                            print("Failed to start tracking")
                            selected_point = None
                    
                    # If tracking is active, update tracking
                    if tracking_active:
                        servoing_result = visual_servoing.updateTracking()
                        
                        # Display visual servoing output on the frame
                        linear_vel = servoing_result.get("linear_vel", 0.0)
                        angular_vel = servoing_result.get("angular_vel", 0.0)
                        running = visual_servoing.running
                        
                        status_color = (0, 255, 0) if running else (0, 0, 255)  # Green if running, red if not
                        
                        # Add velocity text to frame
                        cv2.putText(frame, f"Linear: {linear_vel:.2f} m/s", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                        cv2.putText(frame, f"Angular: {angular_vel:.2f} rad/s", (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                        cv2.putText(frame, f"Tracking: {'ON' if running else 'OFF'}", (10, 90),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                        
                        # If tracking is lost, reset selected_point and tracking_active
                        if not running:
                            selected_point = None
                            tracking_active = False
                
                # Display the frame in main thread
                cv2.imshow("Person Tracking", frame)
                
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
        
        visual_servoing.cleanup()
        video_provider.dispose_all()
        person_tracker.cleanup()
        cv2.destroyAllWindows()
        print("Cleanup complete")


if __name__ == "__main__":
    main()

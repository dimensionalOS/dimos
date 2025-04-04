import cv2
import numpy as np
import os
import sys
import queue
import threading

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dimos.stream.video_provider import VideoProvider
from dimos.perception.semantic_seg import SemanticSegmentationStream

def main():
    # Create a queue for thread communication (limit to prevent memory issues)
    frame_queue = queue.Queue(maxsize=5)
    stop_event = threading.Event()
    
    # Logitech C920e camera parameters at 480p
    camera_params = {
        'resolution': (640, 480),  # 480p resolution
        'focal_length': 3.67,  # mm
        'sensor_size': (4.8, 3.6)  # mm (1/4" sensor)
    }
    
    # Initialize video provider and segmentation stream
    video_provider = VideoProvider("test_camera", video_source=0)
    seg_stream = SemanticSegmentationStream(enable_mono_depth=True, camera_params=camera_params, gt_depth_scale=512.0)
    
    # Create streams
    video_stream = video_provider.capture_video_as_observable(realtime=False, fps=5)
    segmentation_stream = seg_stream.create_stream(video_stream)
    
    # Define callbacks for the segmentation stream
    def on_next(segmentation):
        if stop_event.is_set():
            return

        # Get the frame and visualize
        vis_frame = segmentation.metadata["viz_frame"]
        depth_viz = segmentation.metadata["depth_viz"]
        # Get the image dimensions
        height, width = vis_frame.shape[:2]
        depth_height, depth_width = depth_viz.shape[:2]

        # Resize depth visualization to match segmentation height 
        # (maintaining aspect ratio if needed)
        depth_resized = cv2.resize(depth_viz, (int(depth_width * height / depth_height), height))

        # Create a combined frame for side-by-side display
        combined_viz = np.hstack((vis_frame, depth_resized))

        # Add labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(combined_viz, "Semantic Segmentation", (10, 30), font, 0.8, (255, 255, 255), 2)
        cv2.putText(combined_viz, "Depth Estimation", (width + 10, 30), font, 0.8, (255, 255, 255), 2)

        # Put frame in queue for main thread to display (non-blocking)
        try:
            frame_queue.put_nowait(combined_viz)
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
        subscription = segmentation_stream.subscribe(
            on_next=on_next,
            on_error=on_error,
            on_completed=on_completed
        )
        
        print("Semantic segmentation visualization started. Press 'q' to exit.")
        
        # Main thread loop for displaying frames
        while not stop_event.is_set():
            try:
                # Get frame with timeout (allows checking stop_event periodically)
                combined_viz = frame_queue.get(timeout=1.0)
                
                # Display the frame in main thread
                cv2.imshow("Semantic Segmentation", combined_viz)
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
        seg_stream.cleanup()
        cv2.destroyAllWindows()
        print("Cleanup complete")


if __name__ == "__main__":
    main()
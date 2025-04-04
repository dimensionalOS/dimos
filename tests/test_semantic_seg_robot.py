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
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.stream.video_operators import VideoOperators as MyVideoOps, Operators as MyOps
from dimos.stream.frame_processor import FrameProcessor
from reactivex import operators as RxOps


def main():
    # Create a queue for thread communication (limit to prevent memory issues)
    frame_queue = queue.Queue(maxsize=5)
    stop_event = threading.Event()
    
    # Unitree Go2 camera parameters at 1080p
    camera_params = {
        'resolution': (1920, 1080),  # 1080p resolution
        'focal_length': 3.2,  # mm
        'sensor_size': (4.8, 3.6)  # mm (1/4" sensor)
    }
    
    # Initialize video provider and segmentation stream
    #video_provider = VideoProvider("test_camera", video_source=0)
    robot = UnitreeGo2(ip=os.getenv('ROBOT_IP'),
                        ros_control=UnitreeROSControl(),)
            
    seg_stream = SemanticSegmentationStream(enable_mono_depth=False, camera_params=camera_params, gt_depth_scale=512.0)
    
    # Create streams
    video_stream = robot.get_ros_video_stream(fps=5)
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
        print_emission_args = {
            "enabled": True,
            "dev_name": "SemanticSegmentation",
            "counts": {},
        }


        frame_processor = FrameProcessor(delete_on_init=True)
        subscription = segmentation_stream.pipe(
            MyOps.print_emission(id="A", **print_emission_args),
            RxOps.share(),
            MyOps.print_emission(id="B", **print_emission_args),
            RxOps.map(lambda x: x.metadata["viz_frame"] if x is not None else None),
            MyOps.print_emission(id="C", **print_emission_args),
            RxOps.filter(lambda x: x is not None),
            MyOps.print_emission(id="D", **print_emission_args),
            # MyVideoOps.with_jpeg_export(frame_processor=frame_processor, suffix="_frame_"), 
            MyOps.print_emission(id="E", **print_emission_args),
        )

        print("Semantic segmentation visualization started. Press 'q' to exit.")

        streams = {
            "segmentation_stream": subscription,
        }
        fast_api_server = RobotWebInterface(port=5555, **streams)
        fast_api_server.run()
                
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Stopping...")
    finally:
        # Signal threads to stop
        stop_event.set()
        
        # Clean up resources
        if subscription:
            subscription.dispose()
        
        seg_stream.cleanup()
        cv2.destroyAllWindows()
        print("Cleanup complete")


if __name__ == "__main__":
    main()
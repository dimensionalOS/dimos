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
from dimos.agents.agent import OpenAIAgent

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
                        ros_control=UnitreeROSControl(),
                        skills=MyUnitreeSkills())
            
    seg_stream = SemanticSegmentationStream(enable_mono_depth=True, camera_params=camera_params, gt_depth_scale=512.0)
    
    # Create streams
    video_stream = robot.get_ros_video_stream(fps=5)
    segmentation_stream = seg_stream.create_stream(video_stream.pipe(MyVideoOps.with_fps_sampling(fps=.5))) # Throttling to slowdown SegmentationAgent calls (TODO: add Agent parameter to handle this called api_call_interval)

    frame_processor = FrameProcessor(delete_on_init=True)
    seg_subscription = segmentation_stream.pipe(
        RxOps.share(),
        RxOps.map(lambda x: x.metadata["viz_frame"] if x is not None else None),
        RxOps.filter(lambda x: x is not None),
        # MyVideoOps.with_jpeg_export(frame_processor=frame_processor, suffix="_frame_"), # debugging
    )

    depth_subscription = segmentation_stream.pipe(
        RxOps.share(),
        RxOps.map(lambda x: x.metadata["depth_viz"] if x is not None else None),
        RxOps.filter(lambda x: x is not None),
    )

    object_subscription = segmentation_stream.pipe(
        RxOps.share(),
        RxOps.map(lambda x: x.metadata["objects"] if x is not None else None),
        RxOps.filter(lambda x: x is not None),
        RxOps.map(lambda objects: "\n".join(
            f"Object {obj['object_id']}: {obj['label']} (confidence: {obj['prob']:.2f})" + 
            (f", depth: {obj['depth']:.2f}m" if 'depth' in obj else "")
            for obj in objects
        ) if objects else "No objects detected."),
    )

    segmentation_agent = OpenAIAgent(
        dev_name="SemanticSegmentationAgent",
        model_name="gpt-4o",
        system_query="You are a helpful assistant that can help with tasks related to semantic segmentation. You will get a list of objects as input, reprint them out to me in a list. Your task will be to execute the MOVE() skill towards the box on the ground in front of you",
        input_query_stream=object_subscription,
        process_all_inputs=False,
        skills=robot.get_skills()
    )

    print("Semantic segmentation visualization started. Press 'q' to exit.")

    streams = {
            "depth_stream": depth_subscription,
            "seg_stream": seg_subscription,
        }
    text_streams = {
        "object_stream": object_subscription,
    }

    try:
        fast_api_server = RobotWebInterface(port=5555, text_streams=text_streams, **streams)
        fast_api_server.run()
                
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Stopping...")
    finally:
        # Signal threads to stop
        stop_event.set()
        
        seg_stream.cleanup()
        cv2.destroyAllWindows()
        print("Cleanup complete")

if __name__ == "__main__":
    main()
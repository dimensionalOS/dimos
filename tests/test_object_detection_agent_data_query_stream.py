import os
import time
import sys
import argparse
import threading
from typing import List, Dict, Any
from reactivex import Subject, operators as ops

from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.utils.logging_config import logger
from dimos.stream.video_provider import VideoProvider
from dimos.perception.object_detection_stream import ObjectDetectionStream
from dimos.types.vector import Vector
from dimos.utils.reactive import backpressure
from dimos.perception.detection2d.detic_2d_det import Detic2DDetector
from dimos.agents.claude_agent import ClaudeAgent

from dotenv import load_dotenv


def parse_args():
    parser = argparse.ArgumentParser(description='Test ObjectDetectionStream for object detection and position estimation')
    parser.add_argument('--mode', type=str, default="webcam", choices=["robot", "webcam"],
                        help='Mode to run: "robot" or "webcam" (default: webcam)')
    return parser.parse_args()
load_dotenv()

class ResultPrinter:
    def __init__(self, print_interval: float = 1.0):
        """
        Initialize a result printer that limits console output frequency.
        
        Args:
            print_interval: Minimum time between console prints in seconds
        """
        self.print_interval = print_interval
        self.last_print_time = 0
        
    def print_results(self, objects: List[Dict[str, Any]]):
        """Print object detection results to console with rate limiting."""
        current_time = time.time()
        
        # Only print results at the specified interval
        if current_time - self.last_print_time >= self.print_interval:
            self.last_print_time = current_time
            
            if not objects:
                print("\n[No objects detected]")
                return
                
            print("\n" + "="*50)
            print(f"Detected {len(objects)} objects at {time.strftime('%H:%M:%S')}:")
            print("="*50)
            
            for i, obj in enumerate(objects):
                pos = obj["position"]
                rot = obj["rotation"]
                size = obj["size"]
                
                print(f"{i+1}. {obj['label']} (ID: {obj['object_id']}, Conf: {obj['confidence']:.2f})")
                print(f"   Position: x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f} m")
                print(f"   Rotation: yaw={rot['yaw']:.2f} rad")
                print(f"   Size: width={size['width']:.2f}, height={size['height']:.2f} m")
                print(f"   Depth: {obj['depth']:.2f} m")
                print("-" * 30)


def main():
    # Get command line arguments
    args = parse_args()
    
    # Set up the result printer for console output
    result_printer = ResultPrinter(print_interval=1.0)

    # Set default parameters
    min_confidence = 0.6
    class_filter = None  # No class filtering
    web_port = 5555
    
    # Initialize detector
    detector = Detic2DDetector(vocabulary=None, threshold=min_confidence)
    
    # Initialize based on mode
    if args.mode == "robot":
        print("Initializing in robot mode...")
        
        # Get robot IP from environment
        robot_ip = os.getenv('ROBOT_IP')
        if not robot_ip:
            print("Error: ROBOT_IP environment variable not set.")
            sys.exit(1)
        
        # Initialize robot
        robot = UnitreeGo2(
            ip=robot_ip,
            ros_control=UnitreeROSControl(),
            skills=MyUnitreeSkills(),
        )
        # Create video stream from robot's camera
        video_stream = robot.video_stream_ros

        # Initialize ObjectDetectionStream with robot and transform function
        object_detector = ObjectDetectionStream(
            camera_intrinsics=robot.camera_intrinsics,
            min_confidence=min_confidence,
            class_filter=class_filter,
            transform_to_map=robot.ros_control.transform_pose,
            detector=detector,
            video_stream=video_stream
        )
        

        
    else:  # webcam mode
        print("Initializing in webcam mode...")
        
        # Define camera intrinsics for the webcam
        # These are approximate values for a typical 640x480 webcam
        width, height = 640, 480
        focal_length_mm = 3.67  # mm (typical webcam)
        sensor_width_mm = 4.8   # mm (1/4" sensor)
        
        # Calculate focal length in pixels
        focal_length_x_px = width * focal_length_mm / sensor_width_mm
        focal_length_y_px = height * focal_length_mm / sensor_width_mm
        
        # Principal point (center of image)
        cx, cy = width / 2, height / 2
        
        # Camera intrinsics in [fx, fy, cx, cy] format
        camera_intrinsics = [focal_length_x_px, focal_length_y_px, cx, cy]
        
        # Initialize video provider and ObjectDetectionStream
        video_provider = VideoProvider("test_camera", video_source=0)  # Default camera
        # Create video stream
        video_stream = backpressure(video_provider.capture_video_as_observable(realtime=True, fps=30))

        object_detector = ObjectDetectionStream(
            camera_intrinsics=camera_intrinsics,
            min_confidence=min_confidence,
            class_filter=class_filter,
            detector=detector,
            video_stream=video_stream
        )
        
        
        
        # Set placeholder robot for cleanup
        robot = None
    
    # Create visualization stream for web interface
    viz_stream = object_detector.get_stream().pipe(
        ops.share(),
        ops.map(lambda x: x["viz_frame"] if x is not None else None),
        ops.filter(lambda x: x is not None),
    )

    # Create object data observable for Agent
    object_data_stream = object_detector.get_stream().pipe(
        ops.share(),
        ops.map(lambda x: x["objects"] if x is not None else None),
        ops.filter(lambda x: x is not None),
    )



    # Create stop event for clean shutdown
    stop_event = threading.Event()
    
    # Define subscription callback to print results
    def on_next(result):
        if stop_event.is_set():
            return
        
        # Print detected objects to console
        if "objects" in result:
            result_printer.print_results(result["objects"])
    
    def on_error(error):
        print(f"Error in detection stream: {error}")
        stop_event.set()
    
    def on_completed():
        print("Detection stream completed")
        stop_event.set()
    
    try:
        # # Subscribe to the detection stream
        # subscription = object_detector.get_stream().subscribe(
        #     on_next=on_next,
        #     on_error=on_error,
        #     on_completed=on_completed
        # )
        
        # Set up web interface
        print("Initializing web interface...")
        web_interface = RobotWebInterface(
            port=web_port,
            object_detection=viz_stream
        )

        agent = ClaudeAgent(
            dev_name="test_agent",
            # input_query_stream=stt_node.emit_text(),
            input_query_stream=web_interface.query_stream,
            input_data_stream=object_data_stream,
            skills=robot.get_skills(),
            system_query="Tell me what you see",
            model_name="claude-3-7-sonnet-latest",
            thinking_budget_tokens=0
        )    
        
        # Print configuration information
        print("\nObjectDetectionStream Test Running:")
        print(f"Mode: {args.mode}")
        print(f"Web Interface: http://localhost:{web_port}")
        print("\nPress Ctrl+C to stop the test\n")
        
        # Start web server (blocking call)
        web_interface.run()
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        # Clean up resources
        print("Cleaning up resources...")
        stop_event.set()
        
        # if subscription:
        #     subscription.dispose()
        
        if args.mode == "robot" and robot:
            robot.cleanup()
        elif args.mode == "webcam":
            if 'video_provider' in locals():
                video_provider.dispose_all()
        
        print("Test completed")


if __name__ == "__main__":
    main()

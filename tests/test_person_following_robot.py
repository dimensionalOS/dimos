import os
import time
import sys
from reactivex import operators as RxOps
import tests.test_header

from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.utils.logging_config import logger
from dimos.models.qwen.video_query import query_single_frame


def main():
    # Hardcoded parameters
    timeout = 60.0  # Maximum time to follow a person (seconds)
    distance = 0.5  # Desired distance to maintain from target (meters)
    
    print("Initializing Unitree Go2 robot...")
    
    # Initialize the robot with ROS control and skills
    robot = UnitreeGo2(
        ip=os.getenv('ROBOT_IP'),
        ros_control=UnitreeROSControl(),
        skills=MyUnitreeSkills(),
    )

    tracking_stream = robot.person_tracking_stream
    viz_stream = tracking_stream.pipe(
        RxOps.share(),
        RxOps.map(lambda x: x["viz_frame"] if x is not None else None),
        RxOps.filter(lambda x: x is not None),
    )
    video_stream = robot.get_ros_video_stream()
    
    try:
        # Set up web interface
        logger.info("Initializing web interface")
        streams = {
            "unitree_video": video_stream,
            "person_tracking": viz_stream
        }
        
        web_interface = RobotWebInterface(
            port=5555,
            **streams
        )
        
        # Wait for camera and tracking to initialize
        print("Waiting for camera and tracking to initialize...")
        time.sleep(5)
        # Get initial point from Qwen

        max_retries = 5
        delay = 3  
        
        for attempt in range(max_retries):
            try:
                qwen_point = eval(query_single_frame(
                    video_stream,
                    "Look at this frame and point to the person shirt. Return ONLY their center coordinates as a tuple (x,y)."
                ).pipe(RxOps.take(1)).run())  # Get first response and convert string tuple to actual tuple
                logger.info(f"Found person at coordinates {qwen_point}")
                break  # If successful, break out of retry loop
            except Exception as e:
                if attempt < max_retries - 1:
                    logger.error(f"Person not found. Attempt {attempt + 1}/{max_retries} failed. Retrying in {delay}s... Error: {e}")
                    time.sleep(delay)
                else:
                    logger.error(f"Person not found after {max_retries} attempts. Last error: {e}")
                    return
        
        # Start following human in a separate thread
        import threading
        follow_thread = threading.Thread(
            target=lambda: robot.follow_human(timeout=timeout, distance=distance, point=qwen_point),
            daemon=True
        )
        follow_thread.start()
        
        print(f"Following human at point {qwen_point} for {timeout} seconds...")
        print("Web interface available at http://localhost:5555")
        
        # Start web server (blocking call)
        web_interface.run()
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        print("Test completed")
        robot.cleanup()


if __name__ == "__main__":
    main()

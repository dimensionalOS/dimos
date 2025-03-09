"""Unitree Go2 robot agent demo with FastAPI server integration.

Connects a Unitree Go2 robot to an OpenAI agent with a web interface.

Environment Variables:
    OPENAI_API_KEY: Required. OpenAI API key.
    ROBOT_IP: Required. IP address of the Unitree robot.
    CONN_TYPE: Required. Connection method to the robot.
    ROS_OUTPUT_DIR: Optional. Directory for ROS output files.
"""

import os
import sys

# Add project root to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Local application imports
from dimos.agents.agent import OpenAIAgent
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.utils.logging_config import logger
from dimos.web.robot_web_interface import RobotWebInterface


def main():
    # Get environment variables
    robot_ip = os.getenv("ROBOT_IP")
    if not robot_ip:
        raise ValueError("ROBOT_IP environment variable is required")
    connection_method = os.getenv("CONN_TYPE") or 'webrtc'
    output_dir = os.getenv("ROS_OUTPUT_DIR",
                           os.path.join(os.getcwd(), "assets/output/ros"))

    try:
        # Initialize robot
        logger.info("Initializing Unitree Robot")
        robot = UnitreeGo2(ip=robot_ip,
                           connection_method=connection_method,
                           output_dir=output_dir)

        # Set up video stream
        logger.info("Starting video stream")
        video_stream = robot.get_ros_video_stream()

        # Create FastAPI server with video stream
        logger.info("Initializing FastAPI server")
        streams = {"unitree_video": video_stream}
        web_interface = RobotWebInterface(port=5555, **streams)

        # Initialize agent with robot skills
        skills_instance = MyUnitreeSkills(robot=robot)
        logger.info("Starting perception agent")
        agent = OpenAIAgent(
            dev_name="UnitreeQueryPerceptionAgent",
            input_query_stream=web_interface.query_stream,
            output_dir=output_dir,
            skills=skills_instance,
        )

        # Start server (blocking call)
        logger.info("Starting FastAPI server")
        web_interface.run()

    except KeyboardInterrupt:
        print("Stopping demo...")
    except Exception as e:
        logger.error(f"Error: {e}")
        return 1
    finally:
        if robot:
            robot.cleanup()

    return 0


if __name__ == "__main__":
    sys.exit(main())

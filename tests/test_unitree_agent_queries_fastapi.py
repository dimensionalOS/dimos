"""Unitree Go2 robot agent demo with FastAPI server integration.

Connects a Unitree Go2 robot to an OpenAI agent with a web interface.

Environment Variables:
    OPENAI_API_KEY: Required. OpenAI API key.
    ROBOT_IP: Required. IP address of the Unitree robot.
    CONN_TYPE: Required. Connection method to the robot.
    ROS_OUTPUT_DIR: Optional. Directory for ROS output files.
"""

import tests.test_header
import os
import sys
import reactivex as rx
import reactivex.operators as ops

# Local application imports
from dimos.agents.agent import OpenAIAgent
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.utils.logging_config import logger
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.web.fastapi_server import FastAPIServer


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
                           output_dir=output_dir,
                           skills=MyUnitreeSkills())

        # Set up video stream
        logger.info("Starting video stream")
        video_stream = robot.get_ros_video_stream()

        # Create FastAPI server with video stream and text streams
        logger.info("Initializing FastAPI server")
        streams = {"unitree_video": video_stream}
        
        # Create a subject for agent responses
        agent_response_subject = rx.subject.Subject()
        agent_response_stream = agent_response_subject.pipe(ops.share())
        
        text_streams = {
            "agent_responses": agent_response_stream,
        }
        
        web_interface = FastAPIServer(port=5555, text_streams=text_streams, **streams)

        logger.info("Starting action primitive execution agent")
        agent = OpenAIAgent(
            dev_name="UnitreeQueryExecutionAgent",
            input_query_stream=web_interface.query_stream,
            output_dir=output_dir,
            skills=robot.get_skills(),
        )
        
        # Subscribe to agent responses and send them to the subject
        agent.get_response_observable().subscribe(
            lambda x: agent_response_subject.on_next(x)
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

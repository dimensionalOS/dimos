"""Planning agent demo with FastAPI server and robot integration.

Connects a planning agent, execution agent, and robot with a web interface.

Environment Variables:
    OPENAI_API_KEY: Required. OpenAI API key.
    ROBOT_IP: Required. IP address of the robot.
    CONN_TYPE: Required. Connection method to the robot.
    ROS_OUTPUT_DIR: Optional. Directory for ROS output files.
"""

import tests.test_header
import os
import sys

# -----

from textwrap import dedent
import threading
import time
import reactivex as rx
import reactivex.operators as ops

# Local application imports
from dimos.agents.agent import OpenAIAgent
from dimos.agents.planning_agent import PlanningAgent
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.utils.logging_config import logger
# from dimos.web.fastapi_server import FastAPIServer
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.utils.threadpool import make_single_thread_scheduler

def main():
    # Get environment variables
    robot_ip = os.getenv("ROBOT_IP")
    if not robot_ip:
        raise ValueError("ROBOT_IP environment variable is required")
    connection_method = os.getenv("CONN_TYPE") or 'webrtc'
    output_dir = os.getenv("ROS_OUTPUT_DIR",
                           os.path.join(os.getcwd(), "assets/output/ros"))

    # Initialize components as None for proper cleanup
    robot = None
    web_interface = None
    planner = None
    executor = None

    try:
        # Initialize robot
        logger.info("Initializing Unitree Robot")
        robot = UnitreeGo2(ip=robot_ip,
                           connection_method=connection_method,
                           output_dir=output_dir,
                           mock_connection=False,
                           skills=MyUnitreeSkills())
        # Set up video stream
        logger.info("Starting video stream")
        video_stream = robot.get_ros_video_stream()

        # Initialize robot skills
        logger.info("Initializing robot skills")

        # Create subjects for planner and executor responses
        logger.info("Creating response streams")
        planner_response_subject = rx.subject.Subject()
        planner_response_stream = planner_response_subject.pipe(ops.share())
        
        executor_response_subject = rx.subject.Subject()
        executor_response_stream = executor_response_subject.pipe(ops.share())
        
        # Web interface mode with FastAPI server
        logger.info("Initializing FastAPI server")
        streams = {"unitree_video": video_stream}
        text_streams = {
            "planner_responses": planner_response_stream,
            "executor_responses": executor_response_stream,
        }
        
        web_interface = RobotWebInterface(
            port=5555, text_streams=text_streams, **streams)

        logger.info("Starting planning agent with web interface")
        planner = PlanningAgent(
            dev_name="TaskPlanner",
            model_name="gpt-4o",
            input_query_stream=web_interface.query_stream,
            skills=robot.get_skills()
        )
    
        # Get planner's response observable
        logger.info("Setting up agent response streams")
        planner_responses = planner.get_response_observable()
        
        # Connect planner to its subject
        planner_responses.subscribe(
            lambda x: planner_response_subject.on_next(x)
        )

        planner_responses.subscribe(
            on_next=lambda x: logger.info(f"Planner response: {x}"),
            on_error=lambda e: logger.error(f"Planner error: {e}"),
            on_completed=lambda: logger.info("Planner completed")
        )
        
        # Initialize execution agent with robot skills
        logger.info("Starting execution agent")
        system_query=dedent(
            """
            You are a robot execution agent that can execute tasks on a virtual
            robot. The sole text you will be given is the task to execute.
            You will be given a list of skills that you can use to execute the task.
            ONLY OUTPUT THE SKILLS TO EXECUTE, NOTHING ELSE.
            """
        )
        executor = OpenAIAgent(
            dev_name="StepExecutor",
            input_query_stream=planner_responses,
            output_dir=output_dir,
            skills=robot.get_skills(),
            system_query=system_query,
            pool_scheduler=make_single_thread_scheduler()
        )

        # Get executor's response observable
        executor_responses = executor.get_response_observable()

        # Subscribe to responses for logging
        executor_responses.subscribe(
            on_next=lambda x: logger.info(f"Executor response: {x}"),
            on_error=lambda e: logger.error(f"Executor error: {e}"),
            on_completed=lambda: logger.info("Executor completed")
        )
        
        # Connect executor to its subject
        executor_responses.subscribe(
            lambda x: executor_response_subject.on_next(x)
        )

        # Start web server (blocking call)
        logger.info("Starting FastAPI server")
        web_interface.run()

    except KeyboardInterrupt:
        print("Stopping demo...")
    except Exception as e:
        logger.error(f"Error: {e}")
        return 1
    finally:
        # Clean up all components
        logger.info("Cleaning up components")
        if executor:
            executor.dispose_all()
        if planner:
            planner.dispose_all()
        if web_interface:
            web_interface.dispose_all()
        if robot:
            robot.cleanup()
        # Halt execution forever
        while True:
            time.sleep(1)


if __name__ == "__main__":
    sys.exit(main())

# Example Task: Move the robot forward by 1 meter, then turn 90 degrees clockwise, then move backward by 1 meter, then turn a random angle counterclockwise, then repeat this sequence 5 times.

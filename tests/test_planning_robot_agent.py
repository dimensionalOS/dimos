"""Planning agent demo with FastAPI server and robot integration.

Connects a planning agent, execution agent, and robot with a web interface.

Environment Variables:
    OPENAI_API_KEY: Required. OpenAI API key.
    ROBOT_IP: Required. IP address of the robot.
    CONN_TYPE: Required. Connection method to the robot.
    ROS_OUTPUT_DIR: Optional. Directory for ROS output files.
    USE_TERMINAL: Optional. If set to "true", use terminal interface instead of web.
"""

import os
import sys
import threading
import time

# Add project root to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Local application imports
from dimos.agents.agent import OpenAIAgent
from dimos.agents.planning_agent import PlanningAgent
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.utils.logging_config import logger
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
    use_terminal = os.getenv("USE_TERMINAL", "").lower() == "true"

    use_terminal = True
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
                           mock_connection=True)

        # Set up video stream
        logger.info("Starting video stream")
        video_stream = robot.get_ros_video_stream()

        # Initialize robot skills
        logger.info("Initializing robot skills")
        skills_instance = MyUnitreeSkills(robot=robot)

        if use_terminal:
            # Terminal mode - no web interface needed
            logger.info("Starting planning agent in terminal mode")
            planner = PlanningAgent(
                dev_name="TaskPlanner",
                model_name="gpt-4",
                max_steps=10,
                use_terminal=True,
                skills=skills_instance
            )
        else:
            # Web interface mode
            logger.info("Initializing FastAPI server")
            streams = {"unitree_video": video_stream}
            web_interface = RobotWebInterface(port=5555, **streams)

            logger.info("Starting planning agent with web interface")
            planner = PlanningAgent(
                dev_name="TaskPlanner",
                model_name="gpt-4",
                max_steps=10,
                input_query_stream=web_interface.query_stream,
                skills=skills_instance
            )
        
        # Get planner's response observable
        logger.info("Setting up agent response streams")
        planner_responses = planner.get_response_observable()
        
        # Initialize execution agent with robot skills
        logger.info("Starting execution agent")
        stragentsys="You are a robot execution agent that can execute tasks on a virtual robot. You are given a task to execute and a list of skills that you can use to execute the task. ONLY OUTPUT THE SKILLS TO EXECUTE, NOTHING ELSE."
            
        executor = OpenAIAgent(
            dev_name="StepExecutor",
            input_query_stream=planner_responses,
            output_dir=output_dir,
            skills=skills_instance,
            system_query=stragentsys,
            system_query_without_documents=stragentsys,
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

        if use_terminal:
            # In terminal mode, just wait for the planning session to complete
            logger.info("Waiting for planning session to complete")
            while not planner.plan_confirmed:
                pass
            logger.info("Planning session completed")
        else:
            # Start web server (blocking call)
            logger.info("Starting FastAPI server")
            web_interface.run()

        # Keep the main thread alive
        logger.error("NOTE: Keeping main thread alive")
        while True:
            time.sleep(1)

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

    return 0

if __name__ == "__main__":
    sys.exit(main())

# Move the robot forward by 1 meter, then turn 90 degrees clockwise, then move backward by 1 meter, then turn a random angle counterclockwise, then repeat this sequence 5 times.
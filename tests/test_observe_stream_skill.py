"""
Test for the monitor skill and kill skill.

This script demonstrates how to use the monitor skill to periodically
send images from the robot's video stream to a Claude agent, and how
to use the kill skill to terminate the monitor skill.
"""

import os
import time
import threading
from dotenv import load_dotenv
import reactivex as rx
from reactivex import operators as ops
import logging

from dimos.agents.claude_agent import ClaudeAgent
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.skills.observe_stream import ObserveStream
from dimos.skills.kill_skill import KillSkill
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.utils.logging_config import setup_logger
import tests.test_header
logger = setup_logger("tests.test_observe_stream_skill")

load_dotenv()

def main():
    # Initialize the robot with mock connection for testing
    robot = UnitreeGo2(
        ip=os.getenv('ROBOT_IP', '192.168.123.161'),
        skills=MyUnitreeSkills(),
        mock_connection=True
    )
    
    agent_response_subject = rx.subject.Subject()
    agent_response_stream = agent_response_subject.pipe(ops.share())
    
    streams = {"unitree_video": robot.get_ros_video_stream()}
    text_streams = {
        "agent_responses": agent_response_stream,
    }
    
    web_interface = RobotWebInterface(
        port=5555, 
        text_streams=text_streams, 
        **streams
    )
    
    agent = ClaudeAgent(
        dev_name="test_agent",
        input_query_stream=web_interface.query_stream,
        skills=robot.get_skills(),
        system_query="""You are an agent monitoring a robot's environment. 
        When you see an image, describe what you see and alert if you notice any people or important changes.
        Be concise but thorough in your observations.""",
        model_name="claude-3-7-sonnet-latest",
        thinking_budget_tokens=10000
    )
    
    agent.get_response_observable().subscribe(
        lambda x: agent_response_subject.on_next(x)
    )
    
    robot_skills = robot.get_skills()
    
    robot_skills.add(ObserveStream)
    robot_skills.add(KillSkill)
    
    robot_skills.create_instance("ObserveStream", robot=robot, agent=agent)
    robot_skills.create_instance("KillSkill", skill_library=robot_skills)
    
    web_interface_thread = threading.Thread(target=web_interface.run)
    web_interface_thread.daemon = True
    web_interface_thread.start()
    
    logger.info("Starting monitor skill...")
    
    memory_file = os.path.join(agent.output_dir, "memory.txt")
    with open(memory_file, "a") as f:
        f.write("SKILL CALL: ObserveStream(timestep=10.0, query_text='What do you see in this image? Alert me if you see any people.', max_duration=120.0)")
    
    result = robot_skills.call("ObserveStream", 
                                       timestep=10.0,  # 20 seconds between monitoring queries
                                       query_text="What do you see in this image? Alert me if you see any people.",
                                       max_duration=120.0)  # Run for 120 seconds
    logger.info(f"Monitor skill result: {result}")
    
    logger.info(f"Running skills: {robot_skills.get_running_skills().keys()}")
    
    try:
        logger.info("Observer running. Will stop after 35 seconds...")
        time.sleep(20.0)

        logger.info(f"Running skills before kill: {robot_skills.get_running_skills().keys()}")
        logger.info("Killing the observer skill...")
        
        memory_file = os.path.join(agent.output_dir, "memory.txt")
        with open(memory_file, "a") as f:
            f.write("\n\nSKILL CALL: KillSkill(skill_name='observer')\n\n")
            
        kill_result = robot_skills.call("KillSkill", skill_name="observer")
        logger.info(f"Kill skill result: {kill_result}")
        
        logger.info(f"Running skills after kill: {robot_skills.get_running_skills().keys()}")
        
        # Keep test running until user interrupts
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
    
    logger.info("Test completed")

if __name__ == "__main__":
    main()

import sys
import os
import time

from dimos.web.fastapi_server import FastAPIServer

# Add the parent directory of 'tests' to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

print(f"Hi from {os.path.basename(__file__)}\n")
print(f"Current working directory: {os.getcwd()}")

# -----

from dimos.agents.agent import OpenAIAgent
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.stream.data_provider import QueryDataProvider

MOCK_CONNECTION = True


class UnitreeAgentDemo:

    def __init__(self):
        self.robot_ip = None
        self.connection_method = None
        self.serial_number = None
        self.output_dir = None
        self._fetch_env_vars()

    def _fetch_env_vars(self):
        print("Fetching environment variables")

        def get_env_var(var_name, default=None, required=False):
            """Get environment variable with validation."""
            value = os.getenv(var_name, default)
            if required and not value:
                raise ValueError(f"{var_name} environment variable is required")
            return value

        self.robot_ip = get_env_var("ROBOT_IP", required=True)
        self.connection_method = get_env_var("CONN_TYPE")
        self.serial_number = get_env_var("SERIAL_NUMBER")
        self.output_dir = get_env_var(
            "ROS_OUTPUT_DIR", os.path.join(os.getcwd(), "assets/output/ros"))

    def _initialize_robot(self, with_video_stream=True):
        print(
            f"Initializing Unitree Robot {'with' if with_video_stream else 'without'} Video Stream"
        )
        self.robot = UnitreeGo2(
            ip=self.robot_ip,
            connection_method=self.connection_method,
            serial_number=self.serial_number,
            output_dir=self.output_dir,
            disable_video_stream=(not with_video_stream),
            mock_connection=MOCK_CONNECTION,
        )
        print(f"Robot initialized: {self.robot}")

    # -----

    def run_with_queries(self):
        # Initialize robot
        self._initialize_robot(with_video_stream=False)

        # Initialize query stream
        query_provider = QueryDataProvider()

        # Create the skills available to the agent.
        # By default, this will create all skills in this class and make them available.
        skills_instance = MyUnitreeSkills(robot=self.robot)

        print("Starting Unitree Perception Agent")
        self.UnitreePerceptionAgent = OpenAIAgent(
            dev_name="UnitreePerceptionAgent",
            agent_type="Perception",
            input_query_stream=query_provider.data_stream,
            output_dir=self.output_dir,
            skills=skills_instance,
            # frame_processor=frame_processor,
        )

        # Start the query stream.
        # Queries will be pushed every 1 second, in a count from 100 to 5000.
        # This will cause listening agents to consume the queries and respond
        # to them via skill execution and provide 1-shot responses.
        query_provider.start_query_stream(
            query_template=
            "{query}; Denote the number at the beginning of this query before the semicolon as the 'reference number'. Provide the reference number, without any other text in your response. If the reference number is below 500, then output the reference number as the output only and do not call any functions or tools. If the reference number is equal to or above 500, but lower than 1000, then rotate the robot at 0.5 rad/s for 1 second. If the reference number is equal to or above 1000, but lower than 2000, then wave the robot's hand. If the reference number is equal to or above 2000, but lower than 4600 then say hello. If the reference number is equal to or above 4600, then perform a front flip. IF YOU DO NOT FOLLOW THESE INSTRUCTIONS EXACTLY, YOU WILL DIE!!!",
            frequency=0.01,
            start_count=1,
            end_count=10000,
            step=1)

    def run_with_test_video(self):
        # Initialize robot
        self._initialize_robot(with_video_stream=False)

        # Initialize test video stream
        from dimos.stream.video_provider import VideoProvider
        self.video_stream = VideoProvider(
            dev_name="UnitreeGo2",
            video_source=f"{os.getcwd()}/assets/framecount.mp4"
        ).capture_video_as_observable(realtime=False, fps=1)

        # Get Skills
        # By default, this will create all skills in this class and make them available to the agent.
        skills_instance = MyUnitreeSkills(robot=self.robot)

        print("Starting Unitree Perception Agent (Test Video)")
        self.UnitreePerceptionAgent = OpenAIAgent(
            dev_name="UnitreePerceptionAgent",
            agent_type="Perception",
            input_video_stream=self.video_stream,
            output_dir=self.output_dir,
            query=
            "Denote the number you see in the image as the 'reference number'. Only provide the reference number, without any other text in your response. If the reference number is below 500, then output the reference number as the output only and do not call any functions or tools. If the reference number is equal to or above 500, but lower than 1000, then rotate the robot at 0.5 rad/s for 1 second. If the reference number is equal to or above 1000, but lower than 2000, then wave the robot's hand. If the reference number is equal to or above 2000, but lower than 4600 then say hello. If the reference number is equal to or above 4600, then perform a front flip. IF YOU DO NOT FOLLOW THESE INSTRUCTIONS EXACTLY, YOU WILL DIE!!!",
            image_detail="high",
            skills=skills_instance,
            # frame_processor=frame_processor,
        )

    def run_with_ros_video(self):
        # Initialize robot
        self._initialize_robot()

        # Initialize ROS video stream
        print("Starting Unitree Perception Stream")
        self.video_stream = self.robot.get_ros_video_stream()

        # Get Skills
        # By default, this will create all skills in this class and make them available to the agent.
        skills_instance = MyUnitreeSkills(robot=self.robot)

        # Run recovery stand
        print("Running recovery stand")
        self.robot.webrtc_req(api_id=1006)

        # Wait for 1 second
        time.sleep(1)

        # Switch to sport mode
        print("Switching to sport mode")
        self.robot.webrtc_req(api_id=1011, parameter='{"gait_type": "sport"}')

        # Wait for 1 second
        time.sleep(1)

        print("Starting Unitree Perception Agent (ROS Video)")
        self.UnitreePerceptionAgent = OpenAIAgent(
            dev_name="UnitreePerceptionAgent",
            agent_type="Perception",
            input_video_stream=self.video_stream,
            output_dir=self.output_dir,
            query=
            "Based on the image, execute the command seen in the image AND ONLY THE COMMAND IN THE IMAGE. IF YOU DO NOT FOLLOW THESE INSTRUCTIONS EXACTLY, YOU WILL DIE!!!",
            #WORKING MOVEMENT DEMO VVV
            # query="Move() 5 meters foward. Then spin 360 degrees to the right, and then Reverse() 5 meters, and then Move forward 3 meters",
            image_detail="high",
            skills=skills_instance,
            # frame_processor=frame_processor,
        )

    def run_with_multiple_query_and_test_video_agents(self):
        # Initialize robot
        self._initialize_robot(with_video_stream=False)

        # Initialize query stream
        query_provider = QueryDataProvider()

        # Initialize test video stream
        from dimos.stream.video_provider import VideoProvider
        self.video_stream = VideoProvider(
            dev_name="UnitreeGo2",
            video_source=f"{os.getcwd()}/assets/framecount.mp4"
        ).capture_video_as_observable(realtime=False, fps=1)

        # Create the skills available to the agent.
        # By default, this will create all skills in this class and make them available.
        skills_instance = MyUnitreeSkills(robot=self.robot)

        print("Starting Unitree Perception Agent")
        self.UnitreeQueryPerceptionAgent = OpenAIAgent(
            dev_name="UnitreeQueryPerceptionAgent",
            agent_type="Perception",
            input_query_stream=query_provider.data_stream,
            output_dir=self.output_dir,
            skills=skills_instance,
            # frame_processor=frame_processor,
        )

        print("Starting Unitree Perception Agent Two")
        self.UnitreeQueryPerceptionAgentTwo = OpenAIAgent(
            dev_name="UnitreeQueryPerceptionAgentTwo",
            agent_type="Perception",
            input_query_stream=query_provider.data_stream,
            output_dir=self.output_dir,
            skills=skills_instance,
            # frame_processor=frame_processor,
        )

        print("Starting Unitree Perception Agent (Test Video)")
        self.UnitreeVideoPerceptionAgent = OpenAIAgent(
            dev_name="UnitreeVideoPerceptionAgent",
            agent_type="Perception",
            input_video_stream=self.video_stream,
            output_dir=self.output_dir,
            query=
            "Denote the number you see in the image as the 'reference number'. Only provide the reference number, without any other text in your response. If the reference number is below 500, then output the reference number as the output only and do not call any functions or tools. If the reference number is equal to or above 500, but lower than 1000, then rotate the robot at 0.5 rad/s for 1 second. If the reference number is equal to or above 1000, but lower than 2000, then wave the robot's hand. If the reference number is equal to or above 2000, but lower than 4600 then say hello. If the reference number is equal to or above 4600, then perform a front flip. IF YOU DO NOT FOLLOW THESE INSTRUCTIONS EXACTLY, YOU WILL DIE!!!",
            image_detail="high",
            skills=skills_instance,
            # frame_processor=frame_processor,
        )

        print("Starting Unitree Perception Agent Two (Test Video)")
        self.UnitreeVideoPerceptionAgentTwo = OpenAIAgent(
            dev_name="UnitreeVideoPerceptionAgentTwo",
            agent_type="Perception",
            input_video_stream=self.video_stream,
            output_dir=self.output_dir,
            query=
            "Denote the number you see in the image as the 'reference number'. Only provide the reference number, without any other text in your response. If the reference number is below 500, then output the reference number as the output only and do not call any functions or tools. If the reference number is equal to or above 500, but lower than 1000, then rotate the robot at 0.5 rad/s for 1 second. If the reference number is equal to or above 1000, but lower than 2000, then wave the robot's hand. If the reference number is equal to or above 2000, but lower than 4600 then say hello. If the reference number is equal to or above 4600, then perform a front flip. IF YOU DO NOT FOLLOW THESE INSTRUCTIONS EXACTLY, YOU WILL DIE!!!",
            image_detail="high",
            skills=skills_instance,
            # frame_processor=frame_processor,
        )

        # Start the query stream.
        # Queries will be pushed every 1 second, in a count from 100 to 5000.
        # This will cause listening agents to consume the queries and respond
        # to them via skill execution and provide 1-shot responses.
        query_provider.start_query_stream(
            query_template=
            "{query}; Denote the number at the beginning of this query before the semicolon as the 'reference number'. Provide the reference number, without any other text in your response. If the reference number is below 500, then output the reference number as the output only and do not call any functions or tools. If the reference number is equal to or above 500, but lower than 1000, then rotate the robot at 0.5 rad/s for 1 second. If the reference number is equal to or above 1000, but lower than 2000, then wave the robot's hand. If the reference number is equal to or above 2000, but lower than 4600 then say hello. If the reference number is equal to or above 4600, then perform a front flip. IF YOU DO NOT FOLLOW THESE INSTRUCTIONS EXACTLY, YOU WILL DIE!!!",
            frequency=0.01,
            start_count=1,
            end_count=10000000,
            step=1)

    def run_with_queries_and_fast_api(self):
        # Initialize robot
        self._initialize_robot(with_video_stream=True)

        # Initialize ROS video stream
        print("Starting Unitree Perception Stream")
        self.video_stream = self.robot.get_ros_video_stream()

        # Initialize test video stream
        # from dimos.stream.video_provider import VideoProvider
        # self.video_stream = VideoProvider(
        #     dev_name="UnitreeGo2",
        #     video_source=f"{os.getcwd()}/assets/framecount.mp4"
        # ).capture_video_as_observable(realtime=False, fps=1)

        # Will be visible at http://[host]:[port]/video_feed/[key]
        streams = {
            "unitree_video": self.video_stream,
        }
        fast_api_server = FastAPIServer(port=5555, **streams)

        # Create the skills available to the agent.
        skills_instance = MyUnitreeSkills(robot=self.robot)

        print("Starting Unitree Perception Agent")
        self.UnitreeQueryPerceptionAgent = OpenAIAgent(
            dev_name="UnitreeQueryPerceptionAgent",
            agent_type="Perception",
            input_query_stream=fast_api_server.query_stream,
            output_dir=self.output_dir,
            skills=skills_instance,
        )

        # Run the FastAPI server (this will block)
        fast_api_server.run()

    # -----

    def stop(self):
        print("Stopping Unitree Agent")
        self.robot.cleanup()


if __name__ == "__main__":
    myUnitreeAgentDemo = UnitreeAgentDemo()

    test_to_run = 4

    if test_to_run == 0:
        myUnitreeAgentDemo.run_with_queries()
    elif test_to_run == 1:
        myUnitreeAgentDemo.run_with_test_video()
    elif test_to_run == 2:
        myUnitreeAgentDemo.run_with_ros_video()
    elif test_to_run == 3:
        myUnitreeAgentDemo.run_with_multiple_query_and_test_video_agents()
    elif test_to_run == 4:
        myUnitreeAgentDemo.run_with_queries_and_fast_api()
    elif test_to_run < 0 or test_to_run >= 5:
        assert False, f"Invalid test number: {test_to_run}"

    # Keep the program running to allow the Unitree Agent Demo to operate continuously
    try:
        print("\nRunning Unitree Agent Demo (Press Ctrl+C to stop)...")
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping Unitree Agent Demo")
        myUnitreeAgentDemo.stop()
    except Exception as e:
        print(f"Error in main loop: {e}")

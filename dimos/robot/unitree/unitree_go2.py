import multiprocessing
import cv2
from dimos.robot.robot import MyUnitreeSkills, Robot
from dimos.hardware.interface import HardwareInterface
from dimos.agents.agent import Agent, OpenAIAgent, OpenAIAgent
from dimos.agents.agent_config import AgentConfig
from dimos.robot.skills import AbstractSkill, SkillsHelper
from dimos.stream.frame_processor import FrameProcessor
from dimos.stream.video_provider import VideoProvider
from dimos.stream.video_providers.unitree import UnitreeVideoProvider
from dimos.stream.videostream import VideoStream
from dimos.stream.video_provider import AbstractVideoProvider
from dimos.stream.video_operators import VideoOperators as vops
from reactivex import Observable, create
from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable
import asyncio
import logging
import threading
import time
from queue import Queue
from dimos.robot.unitree.external.go2_webrtc_connect.go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from aiortc import MediaStreamTrack
import os
from datetime import timedelta
from dotenv import load_dotenv, find_dotenv
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from reactivex.scheduler import ThreadPoolScheduler

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# UnitreeGo2 Print Colors (Magenta)
UNITREE_GO2_PRINT_COLOR = "\033[35m"
UNITREE_GO2_RESET_COLOR = "\033[0m"

class UnitreeGo2(Robot):
    def __init__(self, 
                 agent_config: AgentConfig = None,
                 ros_control: UnitreeROSControl = None,
                 ip = None,
                 connection_method: WebRTCConnectionMethod = WebRTCConnectionMethod.LocalSTA,
                 serial_number: str = None,
                 output_dir: str = os.getcwd(), # TODO: Pull from ENV variable to handle docker and local development
                 api_call_interval: int = 5,
                 use_ros: bool = True,
                 use_webrtc: bool = False):

        """Initialize the UnitreeGo2 robot.
        
        Args:
            agent_config: Configuration for the agents
            ros_control: ROS control interface, if None a new one will be created
            ip: IP address of the robot (for LocalSTA connection)
            connection_method: WebRTC connection method (LocalSTA or LocalAP)
            serial_number: Serial number of the robot (for LocalSTA with serial)
            output_dir: Directory for output files
            api_call_interval: Interval between API calls in seconds
            use_ros: Whether to use ROSControl and ROS video provider
            use_webrtc: Whether to use WebRTC video provider ONLY
        """
        if not (use_ros ^ use_webrtc):  # XOR operator ensures exactly one is True
            raise ValueError("Exactly one video/control provider (ROS or WebRTC) must be enabled")

        # Initialize ros_control if it is not provided and use_ros is True
        if ros_control is None and use_ros:
            ros_control = UnitreeROSControl(node_name="unitree_go2")
        super().__init__(agent_config=agent_config, ros_control=ros_control)
        
        # Initialize UnitreeGo2-specific attributes
        self.output_dir = output_dir
        self.ip = ip
        self.api_call_interval = api_call_interval
        self.disposables = CompositeDisposable()
        self.main_stream_obs = None

        # Initialize thread pool scheduler
        self.optimal_thread_count = multiprocessing.cpu_count()
        self.thread_pool_scheduler = ThreadPoolScheduler(self.optimal_thread_count // 2)

        if (connection_method == WebRTCConnectionMethod.LocalSTA) and (ip is None):
            raise ValueError("IP address is required for LocalSTA connection")

        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)
        print(f"Agent outputs will be saved to: {os.path.join(self.output_dir, 'memory.txt')}")

        # Choose data provider based on configuration
        if use_ros:
            # Use ROS video provider from ROSControl
           self.video_stream = self.ros_control.video_provider
        elif use_webrtc:
            # Use WebRTC ONLY video provider
            self.video_stream = UnitreeVideoProvider(
                dev_name="UnitreeGo2",
                connection_method=connection_method,
                serial_number=serial_number,
                ip=self.ip if connection_method == WebRTCConnectionMethod.LocalSTA else None
            )
        # Initialize video stream with specified connection method
        # self.video_stream = UnitreeVideoProvider(

        #     dev_name="UnitreeGo2",
        #     connection_method=connection_method,
        #     serial_number=serial_number,
        #     ip=self.ip if connection_method == WebRTCConnectionMethod.LocalSTA else None
        # )
        
        # For local testing. TODO: remove
        #self.video_stream = VideoProvider(
        #    dev_name="UnitreeGo2",
         #   video_source="/dimos/assets/framecount.mp4"
        #)

    def move(self, x: float, y: float, yaw: float, duration: float = 0.0) -> bool:
        """Move the robot using velocity commands.
        
        Args:
            x: Forward/backward velocity (m/s)
            y: Left/right velocity (m/s)
            yaw: Rotational velocity (rad/s)
            duration: How long to move (seconds). If 0, command is continuous
            
        Returns:
            bool: True if command was sent successfully
        """
        if self.ros_control is None:
            raise RuntimeError("No ROS control interface available for movement")
        return self.ros_control.move(x, y, yaw, duration)
    
    def start_perception(self):
        print(f"Starting video stream with {self.api_call_interval} second intervals...")
        # Create video stream observable with desired FPS
        video_stream_obs = self.video_stream.capture_video_as_observable(fps=30)
        
        # Use closure for frame counting
        def create_frame_counter():
            count = 0
            def increment():
                nonlocal count
                count += 1
                return count
            return increment
        
        frame_counter = create_frame_counter()
        
        # Define a frame processor that logs the frames to disk as jpgs
        frame_processor = FrameProcessor(
            delete_on_init=True,
            output_dir=os.path.join(self.output_dir, "frames")
        )

        # # Debugging ZMQ Socket Code
        # import zmq
        # context = zmq.Context()
        # my_socket = context.socket(zmq.PUB)
        # my_socket.bind("tcp://*:5555")

        # Add rate limiting to the video stream
        rate_limited_stream = video_stream_obs.pipe(
            # Add logging and count frames
            # ops.do_action(lambda _: print(f"Frame {frame_counter()} received")),
            # Sample the latest frame every api_call_interval seconds
            vops.with_fps_sampling(fps=30, use_latest=False),
            # Output to jpgs on disk for debugging
            # vops.with_jpeg_export(frame_processor, suffix="openai_frame_", save_limit=100),
            # Log when a frame is sampled
            # ops.do_action(lambda _: print(f"=== Processing frame at {time.strftime('%H:%M:%S')} ===")),
            # Add error handling
            ops.catch(lambda e, _: print(f"Error in stream processing: {e}")),
            # Share the stream among multiple subscribers
            # ops.share(),
            # Send to debugging socket
            # vops.with_zmq_socket(my_socket)  
        )

        # print(f"{UNITREE_GO2_PRINT_COLOR}Initializing Wiggler Agent...{UNITREE_GO2_RESET_COLOR}")
        # self.UnitreeWigglerAgent = OpenAIAgent(
        #     dev_name="WigglerAgent", 
        #     agent_type="Wiggler", 
        #     input_video_stream=rate_limited_stream,
        #     output_dir=self.output_dir,
        #     # query="Based on the image, if you see a peace sign pose from a boy in an orange shirt, wiggle the robot's hips but ONLY if its a boy in an orange shirt making a peace sign pose. Describe what you see in the image in detail, also specifically include the time seen in the image.",
        #     # query="Denote the number you see in the image. Only provide the number, without any other text in your response.",
        #     query="Based on the image, if you see a human with a peace sign pose, wiggle the robot's hips but ONLY if you see as described with a high level of confidence. Also describe what you see in the image in detail, also specifically include the time seen in the image.",
        #     image_detail="high",
        #     # query="Wiggle the robot's hips. Describe what you see in the image in detail.",
        #     robot_video_provider=self.video_stream,
        #     list_of_skills=[Skills.Wiggle]
        # )

        # Skills Library
        skills_instance = MyUnitreeSkills(robot=self)
        list_of_skills: list[AbstractSkill] = [skills_instance.Move]
        list_of_skills_json = SkillsHelper.get_list_of_skills_as_json(list_of_skills)
        skills_instance.create_instance("Move", {"robot": self})
        print(f"skills_instance: {skills_instance}")
        print(f"list_of_skills_json: {list_of_skills_json}")
        skills_instance.set_tools(list_of_skills_json)

        print(f"{UNITREE_GO2_PRINT_COLOR}Initializing Move Agent...{UNITREE_GO2_RESET_COLOR}")
        self.UnitreeMoveAgent = OpenAIAgent(
            dev_name="MoveAgent", 
            agent_type="Move", 
            input_video_stream=rate_limited_stream,
            output_dir=self.output_dir,
            # query="Based on the image, if you see a human, rotate the robot at 0.5 rad/s for 1 second.",
            query="Denote the number you see in the image. Only provide the number, without any other text in your response. If the number is above 500, but lower than 1000, then rotate the robot at 0.5 rad/s for 1 second.",
            image_detail="high",
            skills=skills_instance,
            pool_scheduler=self.thread_pool_scheduler,
            frame_processor=frame_processor,
        )

    def do(self, *args, **kwargs):
        pass

    def __del__(self):
        """Cleanup resources when the robot is destroyed."""
        try:
            if hasattr(self, 'video_stream'):
                self.video_stream.dispose_all()
        except Exception as e:
            print(f"Error during cleanup: {e}")

    def read_agent_outputs(self):
        """Read and print the latest agent outputs from the memory file."""
        memory_file = os.path.join(self.output_dir, 'memory.txt')
        try:
            with open(memory_file, 'r') as file:
                content = file.readlines()
                if content:
                    print("\n=== Agent Outputs ===")
                    for line in content:
                        print(line.strip())
                    print("==================\n")
                else:
                    print("Memory file exists but is empty. Waiting for agent responses...")
        except FileNotFoundError:
            print("Waiting for first agent response...")
        except Exception as e:
            print(f"Error reading agent outputs: {e}")

    
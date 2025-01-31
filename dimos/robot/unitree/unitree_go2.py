from dimos.robot.robot import Robot
from dimos.hardware.interface import HardwareInterface
from dimos.agents.agent import Agent, OpenAI_Agent
from dimos.agents.agent_config import AgentConfig
from dimos.stream.frame_processor import FrameProcessor
from dimos.stream.videostream import VideoStream
from dimos.stream.video_provider import AbstractVideoProvider
from dimos.stream.video_operators import VideoOperators as vops
from reactivex import Observable, create
from reactivex import operators as ops
import asyncio
import logging
import threading
import time
from queue import Queue
from external.go2_webrtc_connect.go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from aiortc import MediaStreamTrack
import os
from datetime import timedelta
from dotenv import load_dotenv, find_dotenv

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class UnitreeVideoStream(AbstractVideoProvider):
    def __init__(self, dev_name: str = "UnitreeGo2", connection_method: WebRTCConnectionMethod = WebRTCConnectionMethod.LocalSTA, serial_number: str = None, ip: str = None):
        """Initialize the Unitree video stream with WebRTC connection.
        
        Args:
            dev_name: Name of the device
            connection_method: WebRTC connection method (LocalSTA, LocalAP, Remote)
            serial_number: Serial number of the robot (required for LocalSTA with serial)
            ip: IP address of the robot (required for LocalSTA with IP)
        """
        super().__init__(dev_name)
        self.frame_queue = Queue()
        self.loop = None
        self.asyncio_thread = None
        
        # Initialize WebRTC connection based on method
        if connection_method == WebRTCConnectionMethod.LocalSTA:
            if serial_number:
                self.conn = Go2WebRTCConnection(connection_method, serialNumber=serial_number)
            elif ip:
                self.conn = Go2WebRTCConnection(connection_method, ip=ip)
            else:
                raise ValueError("Either serial_number or ip must be provided for LocalSTA connection")
        elif connection_method == WebRTCConnectionMethod.LocalAP:
            self.conn = Go2WebRTCConnection(connection_method)
        else:
            raise ValueError("Unsupported connection method")

    async def _recv_camera_stream(self, track: MediaStreamTrack):
        """Receive video frames from WebRTC and put them in the queue."""
        while True:
            frame = await track.recv()
            # Convert the frame to a NumPy array in BGR format
            img = frame.to_ndarray(format="bgr24")
            self.frame_queue.put(img)

    def _run_asyncio_loop(self, loop):
        """Run the asyncio event loop in a separate thread."""
        asyncio.set_event_loop(loop)
        
        async def setup():
            try:
                await self.conn.connect()
                self.conn.video.switchVideoChannel(True)
                self.conn.video.add_track_callback(self._recv_camera_stream)
            except Exception as e:
                logging.error(f"Error in WebRTC connection: {e}")
                raise

        loop.run_until_complete(setup())
        loop.run_forever()

    def capture_video_as_observable(self, fps: int = 30) -> Observable:
        """Create an observable that emits video frames at the specified FPS.
        
        Args:
            fps: Frames per second to emit (default: 30)
            
        Returns:
            Observable emitting video frames
        """
        frame_interval = 1.0 / fps

        def emit_frames(observer, scheduler):
            try:
                # Start asyncio loop if not already running
                if not self.loop:
                    self.loop = asyncio.new_event_loop()
                    self.asyncio_thread = threading.Thread(
                        target=self._run_asyncio_loop,
                        args=(self.loop,)
                    )
                    self.asyncio_thread.start()

                frame_time = time.monotonic()
                
                while True:
                    if not self.frame_queue.empty():
                        frame = self.frame_queue.get()
                        
                        # Control frame rate
                        now = time.monotonic()
                        next_frame_time = frame_time + frame_interval
                        sleep_time = next_frame_time - now
                        
                        if sleep_time > 0:
                            time.sleep(sleep_time)
                            
                        observer.on_next(frame)
                        frame_time = next_frame_time
                    else:
                        time.sleep(0.001)  # Small sleep to prevent CPU overuse

            except Exception as e:
                logging.error(f"Error during frame emission: {e}")
                observer.on_error(e)
            finally:
                if self.loop:
                    self.loop.call_soon_threadsafe(self.loop.stop)
                if self.asyncio_thread:
                    self.asyncio_thread.join()
                observer.on_completed()

        return create(emit_frames).pipe(
            ops.share()  # Share the stream among multiple subscribers
        )

    def dispose_all(self):
        """Clean up resources."""
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.asyncio_thread:
            self.asyncio_thread.join()
        super().dispose_all()


class UnitreeGo2(Robot):
    def __init__(self, 
                 agent_config: AgentConfig = None, 
                 ip: str="192.168.9.140",
                 output_dir: str = os.getcwd(),
                 api_call_interval: int = 5):
        
        super().__init__(agent_config)
        self.output_dir = output_dir
        self.ip = ip
        self.api_call_interval = api_call_interval

        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)
        print(f"Agent outputs will be saved to: {os.path.join(self.output_dir, 'memory.txt')}")

        # Initialize video stream with default LocalSTA connection
        self.video_stream = UnitreeVideoStream(
            dev_name="UnitreeGo2",
            connection_method=WebRTCConnectionMethod.LocalSTA,
            ip=self.ip,
        )
    
        print("Initializing Perception Agent...")
        self.UnitreePerceptionAgent = OpenAI_Agent(
            dev_name="PerceptionAgent", 
            agent_type="Vision",
            output_dir=self.output_dir,
            query="What do you see in this image? Describe the scene and any notable objects or movements.",
        )

        print("Initializing Execution Agent...")
        self.UnitreeExecutionAgent = OpenAI_Agent(
            dev_name="ExecutionAgent", 
            agent_type="Execution", 
            output_dir=self.output_dir,
            query="Based on the image, what actions would you take? Describe potential movements or interactions.",
        )

        self.agent_config = AgentConfig(agents=[self.UnitreePerceptionAgent, self.UnitreeExecutionAgent])
    
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

        # Add rate limiting to the video stream
        rate_limited_stream = video_stream_obs.pipe(
            # Add logging and count frames
            ops.do_action(lambda _: print(f"Frame {frame_counter()} received")),
            # Sample the latest frame every api_call_interval seconds
            vops.with_fps_sampling(sample_interval=timedelta(seconds=self.api_call_interval), use_latest=False),
            # Output to jpgs on disk for debugging
            vops.with_jpeg_export(frame_processor, suffix="openai_frame_", save_limit=100),
            # Log when a frame is sampled
            ops.do_action(lambda _: print(f"\n=== Processing frame at {time.strftime('%H:%M:%S')} ===")),
            # Add error handling
            ops.catch(lambda e, _: print(f"Error in stream processing: {e}")),
            # Share the stream among multiple subscribers
            ops.share()
        )
        
        print("Subscribing agents to video stream...")
        try:
            # Subscribe perception agent to the rate-limited video stream
            self.UnitreePerceptionAgent.subscribe_to_image_processing(rate_limited_stream)
            self.UnitreeExecutionAgent.subscribe_to_image_processing(rate_limited_stream)
            print("Agents subscribed successfully")
        except Exception as e:
            print(f"Error subscribing agents to video stream: {e}")

    def do(self, *args, **kwargs):
        pass

    def __del__(self):
        """Cleanup resources when the robot is destroyed."""
        self.video_stream.dispose_all()

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


if __name__ == "__main__":
    
    # Initialize the robot with 5-second API call interval
    print("Initializing UnitreeGo2...")
    robot = UnitreeGo2(
        ip="192.168.9.140", 
        output_dir=os.path.join(os.getcwd(), "output"),  # Explicitly use current working directory
        api_call_interval=5  # Specify the interval between API calls
    )
    
    try:
        # Start perception
        print("\nStarting perception system...")
        robot.start_perception()
        
        print("\nMonitoring agent outputs (Press Ctrl+C to stop)...")
        # Monitor agent outputs every 5 seconds
        while True:
            time.sleep(5)
            robot.read_agent_outputs()
            
    except KeyboardInterrupt:
        print("\nStopping perception...")
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        # Cleanup
        print("Cleaning up resources...")
        del robot
        print("Cleanup complete.")

    
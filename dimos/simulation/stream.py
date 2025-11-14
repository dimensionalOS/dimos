from isaacsim import SimulationApp
import cv2
import numpy as np
import subprocess
import time
from typing import Literal, Optional, Union
from pathlib import Path

AnnotatorType = Literal['rgb', 'normals', 'bounding_box_3d', 'motion_vectors']
TransportType = Literal['tcp', 'udp']

class SimulationStream:
    """Class to handle streaming from Isaac Sim simulation."""
    
    def __init__(
        self,
        simulator,
        width: int = 1920,
        height: int = 1080,
        fps: int = 60,
        camera_path: str = "/World/alfred_parent_prim/alfred_base_descr/chest_cam_rgb_camera_frame/chest_cam",
        annotator: AnnotatorType = 'rgb',
        transport: TransportType = 'tcp',
        rtsp_url: str = "rtsp://mediamtx:8554/stream",
        usd_path: Optional[Union[str, Path]] = None
    ):
        """Initialize the simulation stream.
        
        Args:
            simulator: Simulator instance
            width: Stream width in pixels
            height: Stream height in pixels
            fps: Frames per second
            camera_path: USD path to the camera
            annotator: Type of annotator to use
            transport: Transport protocol (tcp/udp)
            rtsp_url: RTSP stream URL
            usd_path: Optional USD file path to load
        """
        self.simulator = simulator
        self.width = width
        self.height = height
        self.fps = fps
        self.camera_path = camera_path
        self.annotator_type = annotator
        self.transport = transport
        self.rtsp_url = rtsp_url
        
        # Import omni.replicator only after SimulationApp is initialized
        import omni.replicator.core as rep
        self.rep = rep
        
        # Initialize stage if USD path provided
        if usd_path:
            self._load_stage(usd_path)
            
        # Setup streaming components
        self._setup_camera()
        self._setup_ffmpeg()
        self._setup_annotator()
        
    def _load_stage(self, usd_path: Union[str, Path]):
        """Load USD stage from file."""
        import omni.usd
        # Convert to absolute path
        abs_path = str(Path(usd_path).resolve())
        omni.usd.get_context().open_stage(abs_path)
        self.stage = self.simulator.get_stage()
        if not self.stage:
            raise RuntimeError(f"Failed to load stage: {abs_path}")
            
    def _setup_camera(self):
        """Setup and validate camera."""
        self.stage = self.simulator.get_stage()
        camera_prim = self.stage.GetPrimAtPath(self.camera_path)
        if not camera_prim:
            raise RuntimeError(f"Failed to find camera at path: {self.camera_path}")
            
        # Create render product
        self.render_product = self.rep.create.render_product(
            self.camera_path,
            resolution=(self.width, self.height)
        )
        
    def _setup_ffmpeg(self):
        """Setup FFmpeg process for streaming."""
        command = [
            'ffmpeg',
            '-y',
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f"{self.width}x{self.height}",
            '-r', str(self.fps),
            '-i', '-',
            '-an',  # No audio
            '-c:v', 'h264_nvenc',
            '-preset', 'fast',
            '-f', 'rtsp',
            '-rtsp_transport', self.transport,
            self.rtsp_url
        ]
        self.proc = subprocess.Popen(command, stdin=subprocess.PIPE)
        
    def _setup_annotator(self):
        """Setup the specified annotator."""
        self.annotator = self.rep.AnnotatorRegistry.get_annotator(self.annotator_type)
        self.annotator.attach(self.render_product)
        
    def stream(self):
        """Start the streaming loop."""
        try:
            print("[Stream] Starting camera stream loop...")
            frame_count = 0
            start_time = time.time()
            
            while True:
                frame_start = time.time()
                
                # Step simulation and get frame
                step_start = time.time()
                self.rep.orchestrator.step()
                step_time = time.time() - step_start
                print(f"[Stream] Simulation step took {step_time*1000:.2f}ms")
                frame = self.annotator.get_data()
                
                # Convert frame format
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                
                # Ensure frame is contiguous
                # if not frame.flags['C_CONTIGUOUS']:
                #     frame = np.ascontiguousarray(frame)
                    
                # Write to FFmpeg
                self.proc.stdin.write(frame.tobytes())
                self.proc.stdin.flush()
                
                # Calculate and log metrics
                frame_time = time.time() - frame_start
                print(f"[Stream] Total frame processing took {frame_time*1000:.2f}ms")
                frame_count += 1
                
                if frame_count % 100 == 0:
                    elapsed_time = time.time() - start_time
                    current_fps = frame_count / elapsed_time
                    print(f"[Stream] Processed {frame_count} frames | Current FPS: {current_fps:.2f}")
                    
        except KeyboardInterrupt:
            print("\n[Stream] Received keyboard interrupt, stopping stream...")
        finally:
            self.cleanup()
            
    def cleanup(self):
        """Cleanup resources."""
        print("[Cleanup] Stopping FFmpeg process...")
        if hasattr(self, 'proc'):
            self.proc.stdin.close()
            self.proc.wait()
        print("[Cleanup] Closing simulation...")
        self.simulator.close()
        print("[Cleanup] Successfully cleaned up resources") 
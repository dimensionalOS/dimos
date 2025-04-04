from abc import ABC, abstractmethod
from typing import Literal, Optional, Union
from pathlib import Path
import subprocess

AnnotatorType = Literal['rgb', 'normals', 'bounding_box_3d', 'motion_vectors']
TransportType = Literal['tcp', 'udp']

class StreamBase(ABC):
    """Base class for simulation streaming."""
    
    @abstractmethod
    def __init__(
        self,
        simulator,
        width: int = 1920,
        height: int = 1080,
        fps: int = 60,
        camera_path: str = "/World/camera",
        annotator_type: AnnotatorType = 'rgb',
        transport: TransportType = 'tcp',
        rtsp_url: str = "rtsp://mediamtx:8554/stream",
        usd_path: Optional[Union[str, Path]] = None
    ):
        """Initialize the stream.
        
        Args:
            simulator: Simulator instance
            width: Stream width in pixels
            height: Stream height in pixels
            fps: Frames per second
            camera_path: Camera path in scene
            annotator: Type of annotator to use
            transport: Transport protocol
            rtsp_url: RTSP stream URL
            usd_path: Optional USD file path to load
        """
        self.simulator = simulator
        self.width = width
        self.height = height
        self.fps = fps
        self.camera_path = camera_path
        self.annotator_type = annotator_type
        self.transport = transport
        self.rtsp_url = rtsp_url
        self.proc = None
        
    @abstractmethod
    def _load_stage(self, usd_path: Union[str, Path]):
        """Load stage from file."""
        pass
        
    @abstractmethod
    def _setup_camera(self):
        """Setup and validate camera."""
        pass
        
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
            '-an',
            '-c:v', 'h264_nvenc',
            '-preset', 'fast',
            '-f', 'rtsp',
            '-rtsp_transport', self.transport,
            self.rtsp_url
        ]
        self.proc = subprocess.Popen(command, stdin=subprocess.PIPE)
        
    @abstractmethod
    def _setup_annotator(self):
        """Setup annotator."""
        pass
        
    @abstractmethod
    def stream(self):
        """Start streaming."""
        pass
        
    @abstractmethod
    def cleanup(self):
        """Cleanup resources."""
        pass 
import numpy as np
import subprocess
import queue
import threading
import av
import time
from typing import Optional

class NVENCStreamer:
    def __init__(self, width: int = 1920, height: int = 1080, fps: int = 30, 
                 whip_endpoint: str = "http://localhost:8080/whip"):
        self.width = width
        self.height = height
        self.fps = fps
        self.whip_endpoint = whip_endpoint
        self.frame_queue = queue.Queue(maxsize=2)  # Small queue size to minimize latency
        self.running = False
        self.encoder_thread = None
        
        # FFmpeg command optimized for low latency
        self.ffmpeg_command = [
            'ffmpeg',
            '-y',  # Overwrite output files
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-s', f'{width}x{height}',
            '-pix_fmt', 'rgba',
            '-r', str(fps),
            '-i', '-',  # Input from pipe
            '-c:v', 'h264_nvenc',  # Use NVENC encoder
            '-preset', 'p1',  # Lowest latency preset
            '-tune', 'ull',  # Ultra-low latency tuning
            '-zerolatency', '1',
            '-b:v', '5M',  # 5 Mbps bitrate
            '-maxrate', '5M',
            '-bufsize', '1M',
            '-f', 'whip',  # WHIP output
            '-whip_header', 'Authorization: Bearer your_token_here',
            self.whip_endpoint
        ]
        
    def start(self):
        if self.running:
            return
            
        self.running = True
        self.encoder_thread = threading.Thread(target=self._encoder_loop)
        self.encoder_thread.start()
        
    def stop(self):
        self.running = False
        if self.encoder_thread:
            self.encoder_thread.join()
            
    def push_frame(self, frame: np.ndarray):
        """Push a new frame to the encoding queue"""
        try:
            self.frame_queue.put_nowait(frame)
        except queue.Full:
            # Drop frame if queue is full to maintain low latency
            pass
            
    def _encoder_loop(self):
        process = subprocess.Popen(
            self.ffmpeg_command,
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        while self.running:
            try:
                frame = self.frame_queue.get(timeout=1.0)
                process.stdin.write(frame.tobytes())
            except queue.Empty:
                continue
            except BrokenPipeError:
                break
                
        process.stdin.close()
        process.wait() 
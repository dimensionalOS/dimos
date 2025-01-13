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
        print(f"[NVENCStreamer] Initializing with RTSP output")
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_queue = queue.Queue(maxsize=2)
        self.running = False
        self.encoder_thread = None
        
        # FFmpeg command using RTSP output
        self.ffmpeg_command = [
            'ffmpeg',
            '-y',
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-s', f'{width}x{height}',
            '-pix_fmt', 'rgba',
            '-r', str(fps),
            '-i', '-',
            '-c:v', 'h264_nvenc',
            '-preset', 'p1',
            '-tune', 'ull',
            '-zerolatency', '1',
            '-b:v', '5M',
            '-maxrate', '5M',
            '-bufsize', '1M',
            '-f', 'rtsp',
            '-rtsp_transport', 'tcp',  # TCP for reliability
            f'rtsp://18.189.249.222:8554/live'  # Stream to your EC2 IP
        ]
        print(f"[NVENCStreamer] FFmpeg command: {' '.join(self.ffmpeg_command)}")
        
    def start(self):
        if self.running:
            return
            
        self.running = True
        self.encoder_thread = threading.Thread(target=self._encoder_loop)
        self.encoder_thread.start()
        print("[NVENCStreamer] Encoder thread started")
        
    def stop(self):
        self.running = False
        if self.encoder_thread:
            self.encoder_thread.join()
        print("[NVENCStreamer] Encoder thread stopped")
            
    def push_frame(self, frame: np.ndarray):
        """Push a new frame to the encoding queue"""
        try:
            self.frame_queue.put_nowait(frame)
            print("[NVENCStreamer] Frame queued")
        except queue.Full:
            print("[NVENCStreamer] Queue full, dropping frame")
            pass
            
    def _encoder_loop(self):
        print("[NVENCStreamer] Starting encoder loop")
        process = subprocess.Popen(
            self.ffmpeg_command,
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=False  # Changed to handle binary data
        )
        
        # Start a thread to read stderr
        def log_stderr():
            while True:
                line = process.stderr.readline()
                if not line:
                    break
                print(f"[FFmpeg] {line.decode().strip()}")
        
        stderr_thread = threading.Thread(target=log_stderr)
        stderr_thread.daemon = True
        stderr_thread.start()
        
        while self.running:
            try:
                frame = self.frame_queue.get(timeout=1.0)
                process.stdin.write(frame.tobytes())  # Write raw bytes
                process.stdin.flush()
                print("[NVENCStreamer] Frame sent to FFmpeg")
            except queue.Empty:
                continue
            except BrokenPipeError:
                print("[NVENCStreamer] Broken pipe error!")
                break
            except Exception as e:
                print(f"[NVENCStreamer] Error: {str(e)}")
                break
                
        process.stdin.close()
        process.wait() 
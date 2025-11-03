import numpy as np
import subprocess
import queue
import threading
import time
import cv2

class NVENCStreamer:
    def __init__(self, width: int = 1920, height: int = 1080, fps: int = 30):
        print(f"[NVENCStreamer] Initializing RTSP stream at {fps} FPS")
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_interval = 1.0 / fps
        self.last_frame_time = 0
        self.frame_queue = queue.Queue(maxsize=5)
        self.running = False
        self.encoder_thread = None
        self.frames_processed = 0
        self.start_time = None
        
        # FFmpeg command using RTSP output
        self.ffmpeg_command = [
            'ffmpeg',
            '-y',
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-s', f'{width}x{height}',
            '-pix_fmt', 'bgr24',
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
            '-rtsp_transport', 'tcp',
            f'rtsp://18.189.249.222:8554/live'
        ]

    def start(self):
        """Start the encoder thread"""
        if self.running:
            return
        self.running = True
        self.encoder_thread = threading.Thread(target=self._encoder_loop)
        self.encoder_thread.start()
        print("[NVENCStreamer] Encoder thread started")

    def stop(self):
        """Stop the encoder thread"""
        print("[NVENCStreamer] Stopping encoder...")
        self.running = False
        if self.encoder_thread:
            self.encoder_thread.join()
        print("[NVENCStreamer] Encoder stopped")
            
    def push_frame(self, frame: np.ndarray):
        """Push a new frame to the encoding queue with rate limiting"""
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_frame_time < self.frame_interval:
            print("[NVENCStreamer] Frame skipped due to rate limiting")
            return
            
        try:
            print("[NVENCStreamer] Converting RGBA to BGR...")
            # Convert RGBA to BGR
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            
            # If queue is full, remove oldest frame
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                    print("[NVENCStreamer] Queue full, dropped oldest frame")
                except queue.Empty:
                    pass
                    
            self.frame_queue.put_nowait(frame_bgr)
            self.last_frame_time = current_time
            print("[NVENCStreamer] Frame successfully queued")
            
        except Exception as e:
            print(f"[NVENCStreamer] Frame processing error: {str(e)}")
            
    def _encoder_loop(self):
        print("[NVENCStreamer] Starting encoder loop")
        if self.start_time is None:
            self.start_time = time.time()
            
        print(f"[NVENCStreamer] Starting FFmpeg with command: {' '.join(self.ffmpeg_command)}")
        process = subprocess.Popen(
            self.ffmpeg_command,
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=False
        )
        
        # Start a thread to read stderr and print everything for debugging
        def log_stderr():
            print("[FFmpeg] Starting stderr logging thread")
            while True:
                line = process.stderr.readline()
                if not line:
                    break
                print(f"[FFmpeg] {line.decode().strip()}")
        
        stderr_thread = threading.Thread(target=log_stderr)
        stderr_thread.daemon = True
        stderr_thread.start()
        
        last_fps_print = time.time()
        
        while self.running:
            try:
                print("[NVENCStreamer] Waiting for frame from queue...")
                frame = self.frame_queue.get(timeout=1.0)
                print("[NVENCStreamer] Got frame from queue, writing to FFmpeg...")
                process.stdin.write(frame.tobytes())
                process.stdin.flush()
                self.frames_processed += 1
                print("[NVENCStreamer] Frame successfully written to FFmpeg")
                
                # Print FPS every 5 seconds
                current_time = time.time()
                if current_time - last_fps_print >= 5.0:
                    elapsed = current_time - self.start_time
                    fps = self.frames_processed / elapsed
                    print(f"[NVENCStreamer] Current FPS: {fps:.2f}")
                    last_fps_print = current_time
                    
            except queue.Empty:
                print("[NVENCStreamer] Queue empty, waiting for next frame...")
                continue
            except BrokenPipeError:
                print("[NVENCStreamer] Broken pipe error!")
                break
            except Exception as e:
                print(f"[NVENCStreamer] Error: {str(e)}")
                break
                
        print("[NVENCStreamer] Encoder loop ending, cleaning up...")
        process.stdin.close()
        process.wait()
        print("[NVENCStreamer] Encoder loop ended") 
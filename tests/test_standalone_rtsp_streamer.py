from datetime import timedelta
import sys
import os

# Add the parent directory of 'tests' to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# -----

import cv2
import subprocess
import sys

# Define the codec and create VideoWriter object
width, height = 1920, 1080  # Make sure this matches your input video source
fps = 60  # Make sure this matches your input video source
command = [
    'ffmpeg',
    '-y',
    '-loglevel', 'debug',  # Add this for detailed logs
    '-f', 'rawvideo',
    '-vcodec', 'rawvideo',
    '-pix_fmt', 'bgr24',
    '-s', f"{width}x{height}",
    '-r', str(fps),
    '-i', '-',
    '-an',  # No audio
    '-c:v', 'libx264',
    '-preset', 'ultrafast',
    '-f', 'rtsp',
    'rtsp://mediamtx:8554/stream',
    '-rtsp_transport', 'tcp'  # Enforce TCP transport
]

# Open a subprocess with ffmpeg command
proc = subprocess.Popen(command, stdin=subprocess.PIPE)

# Start capturing video from the webcam
cap = cv2.VideoCapture("/app/assets/video.mov")

if not cap.isOpened():
    print("Error opening video stream")
    sys.exit(1)
print("Opened video stream")

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret:
        proc.stdin.write(frame.tobytes())
    else:
        print('No Video (Resetting)')
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        continue

# Release everything if job is finished
cap.release()
proc.stdin.close()
proc.wait()

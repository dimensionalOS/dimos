import sys
import os

# Get the absolute path of the 'go2_webrtc_driver' directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# Build absolute path to 'libs' directory
libs_path = os.path.abspath(os.path.join(current_dir, '..', 'libs'))

# Add aiortc and aioice directories to sys.path
# sys.path.insert(0, os.path.join(libs_path, 'aiortc', 'src'))
sys.path.insert(0, os.path.join(libs_path, 'aioice', 'src'))
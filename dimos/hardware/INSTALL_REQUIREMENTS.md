# GStreamer Installation Requirements

## For RTSP Server with Absolute Timestamps

To use the RTSP server (rtsp_sender.py) for proper absolute timestamp support, install:

```bash
# Install GStreamer RTSP server library
sudo apt install -y \
    gstreamer1.0-rtsp \
    libgstrtspserver-1.0-0 \
    libgstrtspserver-1.0-dev \
    gir1.2-gst-rtsp-server-1.0

# Install Python GStreamer bindings
sudo apt install -y \
    python3-gst-1.0 \
    python3-gi \
    python3-gi-cairo \
    gir1.2-gstreamer-1.0

# Install GStreamer plugins for video
sudo apt install -y \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools
```

## Alternative: TCP Socket Stream with Timestamps

Since RTP doesn't preserve absolute timestamps properly, and if RTSP server isn't available,
use the TCP-based sender/receiver instead (see tcp_sender.py and tcp_camera.py).

## Usage

### With RTSP (recommended, requires RTSP server library):
```bash
# On sender machine:
python3 rtsp_sender.py --device /dev/video0 --port 8554

# On receiver machine:
python3 test_rtsp_camera.py --rtsp-host <sender-ip> --rtsp-port 8554
```

### With TCP sockets (fallback, always works):
```bash
# On sender machine:
python3 tcp_sender.py --device /dev/video0 --port 5555

# On receiver machine:
python3 test_tcp_camera.py --host <sender-ip> --port 5555
```
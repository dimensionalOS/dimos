# Hardware

## Remote camera stream with timestamps

### Required Ubuntu packages:

```bash
# Install Python GStreamer bindings
sudo apt install -y \
    python3-gst-1.0 \
    python3-gi \
    gir1.2-gstreamer-1.0

# Install GStreamer plugins for video encoding/decoding
sudo apt install -y \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-tools
```

### Usage

On sender machine (with the camera):

```bash
python3 gstreamer_sender.py --device /dev/video0 --port 5555
```

On receiver machine:

```bash
python3 gstreamer_camera_test_script.py --host <sender-ip> --port 5555
```
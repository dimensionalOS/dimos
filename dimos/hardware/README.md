# Hardware

## Remote camera stream with timestamps

### Required Ubuntu packages:

```bash
sudo apt install python3-gi python3-gi-cairo gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0 gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x \
    gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 gstreamer1.0-pulseaudio libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev
```

### Usage

On sender machine (with the camera):

```bash
python3 gstreamer_sender.py --device /dev/video0 --host 0.0.0.0 --port 5000
```

If it's a stereo camera and you only want to send the left side (the left camera):

```bash
python3 gstreamer_sender.py --device /dev/video0 --host 0.0.0.0 --port 5000 --single-camera
```

On receiver machine:

```bash
python3 gstreamer_camera_test_script.py --host 10.0.0.227 --port 5000
```
# Luxonis OAK

`OakCamera` (`camera.py`) streams an OAK-D camera through DepthAI v3: `color_image`, `depth_image` aligned to colour, `camera_info`, `depth_camera_info`, `tf` (`camera_link` down to `camera_color_optical_frame`) and, with `enable_pointcloud`, `pointcloud`.

Install the SDK with the `oak` extra, and on Linux let your user open the camera (the rule from the Luxonis install docs; replug the camera afterwards):

```bash skip
uv sync --extra oak
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

`dimos run oak-camera-vis` shows colour, depth and the camera in Rerun; add `--enable-pointcloud true` for the RGBD cloud.

The defaults (1280x720 colour at 15 Hz, depth at 5 Hz, colour MJPEG-encoded on the camera) fit a USB 2 link. A camera that is still re-enumerating after a relaunch, fails to open, or hangs while it starts is given up after `open_timeout` and brought up again.

import time
import numpy as np
import pyzed.sl as sl

print("1: imports ok")

zed = sl.Camera()
print("2: camera created")

ip = sl.InitParameters()
ip.camera_resolution = sl.RESOLUTION.VGA
ip.camera_fps = 15
ip.depth_mode = sl.DEPTH_MODE.PERFORMANCE
print("3: init params set")

err = zed.open(ip)
print(f"4: open result: {err}")
if err != sl.ERROR_CODE.SUCCESS:
    exit(1)

print("5: enabling tracking")
tp = sl.PositionalTrackingParameters()
zed.enable_positional_tracking(tp)
print("6: tracking enabled")

rt = sl.RuntimeParameters()
print("7: grabbing first frame")
err = zed.grab(rt)
print(f"8: grab result: {err}")

print("9: retrieving image")
img = sl.Mat()
zed.retrieve_image(img, sl.VIEW.LEFT, sl.MEM.CPU)
print(f"10: image shape: {img.get_data().shape}")

print("11: retrieving depth")
depth = sl.Mat()
zed.retrieve_measure(depth, sl.MEASURE.DEPTH, sl.MEM.CPU)
print(f"12: depth shape: {depth.get_data().shape}")

print("13: retrieving XYZRGBA")
pc = sl.Mat()
zed.retrieve_measure(pc, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
print(f"14: pc shape: {pc.get_data().shape}")

zed.close()
print("15: done")
